#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include <stdbool.h>
#include "esp_log.h"
#include "driver/gpio.h"

#define SAMPLE_RATE     16384  // Optimal power-of-2 sample rate (2^14) for excellent bin alignment
#define FRAME_SIZE      4096   // Power-of-2 for optimal performance and FFT compatibility (4 Hz resolution)
#define BIN_1989        497    // Center bin for 1989 Hz (1989 / 4 = 497.25 → bin 497)
#define BIN_2401        600    // Center bin for 2401 Hz (2401 / 4 = 600.25 → bin 600)

// Calibration factors to normalize frequency response (1989Hz measured ~165300, 2401Hz measured ~49500)
#define CAL_1989        1.0f    // Reference frequency (no scaling)
#define CAL_2401        3.34f   // Scale factor: 165300/49500 ≈ 3.34
static const char *TAG = "ADC";

static adc_continuous_handle_t adc_handle;
static float kaiser_window[FRAME_SIZE];

// Static buffers to avoid stack overflow with large frame sizes
static uint8_t frame_buffer[FRAME_SIZE * sizeof(adc_digi_output_data_t)];
static float samples[FRAME_SIZE];

static float snr_1989 = 0.0f;
static float snr_2401 = 0.0f;
static SemaphoreHandle_t mag_mutex = NULL;
static TaskHandle_t worker_task_handle = NULL;


// Modified Bessel function I0 for Kaiser window
static float bessel_i0(float x) {
    float sum = 1.0f;
    float term = 1.0f;
    float x_half_sq = (x * 0.5f) * (x * 0.5f);
    
    for (int k = 1; k < 20; k++) {  // 20 terms gives good accuracy
        term *= x_half_sq / (k * k);
        sum += term;
        if (term < 1e-8f) break;  // Convergence check
    }
    return sum;
}

static void compute_kaiser_window(void) {
    const float beta = 10.0f;  // β=10 for good sidelobe suppression (~-60dB)
    const float i0_beta = bessel_i0(beta);
    
    for (int i = 0; i < FRAME_SIZE; i++) {
        float n = (float)i;
        float N_1 = (float)(FRAME_SIZE - 1);
        
        // Kaiser window formula
        float arg = beta * sqrtf(1.0f - powf((2.0f * n / N_1) - 1.0f, 2.0f));
        kaiser_window[i] = bessel_i0(arg) / i0_beta;
    }
}

static float goertzel(const float *samples, int N, float bin) {
    float omega = 2.0f * M_PI * bin / N;
    float coeff = 2.0f * cosf(omega);
    float Q0 = 0, Q1 = 0, Q2 = 0;

    for (int i = 0; i < N; i++) {
        Q0 = coeff * Q1 - Q2 + samples[i];
        Q2 = Q1;
        Q1 = Q0;
    }

    return sqrtf(Q1 * Q1 + Q2 * Q2 - Q1 * Q2 * coeff);
}

static bool adc_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    /* ISR-context callback: notify the worker task that data is available.
     * Keep this function minimal and non-blocking. The worker task will
     * perform the actual adc_continuous_read and processing in task context.
     */
    BaseType_t higher_woken = pdFALSE;
    if (worker_task_handle != NULL) {
        xTaskNotifyFromISR(worker_task_handle, 1, eSetValueWithOverwrite, &higher_woken);
        portYIELD_FROM_ISR(higher_woken);
    }
    return true;
}

#define PRINT_SAMPLES_ONCE
static void worker_task(void *arg)
{
    uint32_t bufsize = sizeof(frame_buffer);
    uint32_t notified_val = 0;

    for (;;) {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &notified_val, portMAX_DELAY) == pdTRUE) {
            uint32_t bytes_read = 0;
            esp_err_t ret = adc_continuous_read(adc_handle, frame_buffer, bufsize, &bytes_read, pdMS_TO_TICKS(100));
            if (ret == ESP_OK && bytes_read == bufsize) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&frame_buffer;
                
                // Check for ADC saturation/clipping and calculate DC offset
                float dc_sum = 0.0f;
                uint32_t saturated_samples = 0;
                uint32_t min_val = 4095, max_val = 0;
                
                for (int i = 0; i < FRAME_SIZE; i++) {
                    uint32_t sample = p[i].type2.data;
                    dc_sum += (float)sample;
                    
                    // Track min/max for saturation detection
                    if (sample < min_val) min_val = sample;
                    if (sample > max_val) max_val = sample;
                    
                    // Count saturated samples (within 1% of full scale)
                    if (sample <= 40 || sample >= 4055) {
                        saturated_samples++;
                    }
                }
                
                float dc_offset = dc_sum / FRAME_SIZE;
                
                // Log warnings for problematic conditions
                if (saturated_samples > (FRAME_SIZE / 100)) {  // More than 1% saturated
                    ESP_LOGW(TAG, "ADC saturation detected: %lu samples, DC=%.1f, range=%lu-%lu", 
                             saturated_samples, dc_offset, min_val, max_val);
                }
                
                // Apply Kaiser window and remove DC offset
                for (int i = 0; i < FRAME_SIZE; i++) {
                    samples[i] = ((float)p[i].type2.data - dc_offset) * kaiser_window[i];
                }

                // Measure noise floor from bins away from both target frequencies
                // Use bins that are far from both 1989Hz (497) and 2401Hz (600)
                float noise_bins[] = {
                    goertzel(samples, FRAME_SIZE, 400.0f),  // 1600 Hz
                    goertzel(samples, FRAME_SIZE, 450.0f),  // 1800 Hz
                    goertzel(samples, FRAME_SIZE, 550.0f),  // 2200 Hz
                    goertzel(samples, FRAME_SIZE, 650.0f),  // 2600 Hz
                    goertzel(samples, FRAME_SIZE, 700.0f),  // 2800 Hz
                };
                
                // Calculate average noise floor
                float noise_sum = 0.0f;
                for (int i = 0; i < 5; i++) {
                    noise_sum += noise_bins[i];
                }
                float noise_floor = noise_sum / 5.0f;

                // Implement differential detection to reduce spectral leakage
                // For 1989 Hz: Check target bins vs nearby bins to isolate the signal
                float m1989_center = goertzel(samples, FRAME_SIZE, (float)BIN_1989);      // bin 497
                float m1989_high = goertzel(samples, FRAME_SIZE, (float)(BIN_1989 + 1));  // bin 498
                
                // Check adjacent bins for leakage (2000Hz is at bin 500)
                float m1989_leak_high = goertzel(samples, FRAME_SIZE, (float)(BIN_1989 + 3)); // bin 500 (2000Hz)
                float m1989_leak_low = goertzel(samples, FRAME_SIZE, (float)(BIN_1989 - 3));  // bin 494
                
                // Differential detection: target energy minus average leakage
                float m1989_target = sqrtf(0.5625f*m1989_center*m1989_center + 0.0625f*m1989_high*m1989_high);
                float m1989_leakage = (m1989_leak_high + m1989_leak_low) / 2.0f;
                
                // For 2401 Hz: Similar differential approach
                float m2401_center = goertzel(samples, FRAME_SIZE, (float)BIN_2401);      // bin 600
                float m2401_high = goertzel(samples, FRAME_SIZE, (float)(BIN_2401 + 1));  // bin 601
                
                // Check for leakage from distant bins
                float m2401_leak_high = goertzel(samples, FRAME_SIZE, (float)(BIN_2401 + 3)); // bin 603
                float m2401_leak_low = goertzel(samples, FRAME_SIZE, (float)(BIN_2401 - 3));  // bin 597
                
                float m2401_target = sqrtf(0.5625f*m2401_center*m2401_center + 0.0625f*m2401_high*m2401_high);
                float m2401_leakage = (m2401_leak_high + m2401_leak_low) / 2.0f;
                
                // Calculate SNR in dB for both frequencies
                float signal_1989 = fmaxf(0.0f, m1989_target - 0.1f*m1989_leakage) * CAL_1989;
                float signal_2401 = fmaxf(0.0f, m2401_target - 0.1f*m2401_leakage) * CAL_2401;
                
                // Convert to SNR in dB (avoid log(0) by using small minimum value)
                float snr_1989_db = 20.0f * log10f(fmaxf(signal_1989, 1.0f) / fmaxf(noise_floor, 1.0f));
                float snr_2401_db = 20.0f * log10f(fmaxf(signal_2401, 1.0f) / fmaxf(noise_floor, 1.0f));

                /* Store SNR values into globals protected by mutex */
                if (mag_mutex) {
                    if (xSemaphoreTake(mag_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        snr_1989 = snr_1989_db;
                        snr_2401 = snr_2401_db;
                        xSemaphoreGive(mag_mutex);
                    }
                }
            } 
            else {
                ESP_LOGW(TAG, "worker_task: adc_continuous_read failed ret=%d bytes=%u", ret, bytes_read);
            }
        }
        else {
            ESP_LOGW(TAG, "worker_task: xTaskNotifyWait failed");
        }
    }
}


static void setup_adc_continuous(void) {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = FRAME_SIZE * sizeof(adc_digi_output_data_t),
        .conv_frame_size = FRAME_SIZE * sizeof(adc_digi_output_data_t),
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,           // 12dB attenuation
        .channel = ADC_CHANNEL_6,           // GPIO34
        .unit = ADC_UNIT_1,                 // ADC unit 1
        .bit_width = ADC_BITWIDTH_12,       // 12-bit width
    };

    adc_continuous_config_t adc_config = {
        .sample_freq_hz = SAMPLE_RATE,                    // 20 kHz sample rate
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 1,
        .adc_pattern = &pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    /* Register event callback: when ADC conversion finishes, the driver will
     * invoke this callback (in ISR context). We notify the processing task
     * with the buffer index.
     */
    static adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void app_main(void) {

    compute_kaiser_window();

    /* Create single worker task that will be notified by the ADC callback,
     * read frames from the driver and process them. */
    xTaskCreate(worker_task, "adc_worker", 4096, NULL, tskIDLE_PRIORITY + 2, &worker_task_handle);

    setup_adc_continuous();

    /* Initialize mutex for magnitude protection */
    mag_mutex = xSemaphoreCreateMutex();

    while (true) {
        if (mag_mutex && xSemaphoreTake(mag_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "SNR: 1989Hz=%.1fdB | 2401Hz=%.1fdB", snr_1989, snr_2401);
            xSemaphoreGive(mag_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Ensure 2s between prints
    }
}
