/*
 * 2-Tone Decoder for ESP32-S3
 * Detects configurable tone sequences using Goertzel algorithm
 * Current configuration: TONE1_FREQ and TONE2_FREQ (see defines)
 */

// ============================================================================
// INCLUDES
// ============================================================================
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "driver/gpio.h"

// ============================================================================
// DEFINES
// ============================================================================
#define TONE1_FREQ      1185.2f // First tone frequency in Hz
#define TONE2_FREQ      1285.8f // Second tone frequency in Hz

// Tone duration requirements
#define TONE1_DURATION_MS 500   // Duration Tone1 must be held (ms)
#define TONE2_DURATION_MS 2000  // Duration Tone2 must be held (ms)

#define SAMPLE_RATE     8192   // Optimal power-of-2 sample rate (2^13) for excellent bin alignment
#define FRAME_SIZE      2048   // Power-of-2 for optimal performance and FFT compatibility (4 Hz resolution)
#define BIN_TONE1       296    // Center bin for TONE1_FREQ (1185.2 * 2048 / 8192 = 296.3 → bin 296)
#define BIN_TONE2       321    // Center bin for TONE2_FREQ (1285.8 * 2048 / 8192 = 321.45 → bin 321)

// Calibration factors to normalize frequency response (values need to be re-measured for new frequencies)
#define CAL_TONE1       1.0f    // Reference frequency (no scaling)
#define CAL_TONE2       1.0f    // Scale factor: needs to be measured and calibrated

// SNR averaging parameters
#define SNR_AVERAGE_COUNT 2     // Number of samples to average (2 frames = 0.5 seconds)

// ============================================================================
// TYPE DEFINITIONS
// ============================================================================
// State machine for 2-tone detection
typedef enum {
    STATE_IDLE,           // Waiting for Tone1
    STATE_TONE1_DETECTED, // Tone1 detected, waiting for duration
    STATE_WAIT_TONE2,     // Waiting for Tone2
    STATE_TONE2_DETECTED, // Tone2 detected, waiting for duration
    STATE_SEQUENCE_COMPLETE // Both tones detected in sequence
} detection_state_t;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
static const char* TAG = "ADC";

// ADC and processing
static adc_continuous_handle_t adc_handle;
static float hamming_window[FRAME_SIZE];

// Static buffers to avoid stack overflow with large frame sizes
static uint8_t frame_buffer[FRAME_SIZE * sizeof(adc_digi_output_data_t)];
static float samples[FRAME_SIZE];

// Shared data with mutex protection
static float snr_tone1 = 0.0f;
static float snr_tone2 = 0.0f;
static SemaphoreHandle_t mag_mutex = NULL;

// SNR averaging buffers
static float snr_tone1_history[SNR_AVERAGE_COUNT];
static float snr_tone2_history[SNR_AVERAGE_COUNT];
static uint32_t snr_index = 0;
static bool snr_buffer_full = false;

// State machine
static detection_state_t current_state = STATE_IDLE;
static uint32_t state_timer = 0;
static SemaphoreHandle_t state_mutex = NULL;
static TaskHandle_t worker_task_handle = NULL;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
// Hamming window
static void compute_hamming_window(void);

// SNR averaging
static void update_snr_average(float new_snr_tone1, float new_snr_tone2);

// Signal processing
static float goertzel(const float *samples, int N, float bin);

// Task functions
static void worker_task(void *arg);
static void state_machine_task(void *pvParameters);

// ADC setup
static void setup_adc_continuous(void);

// Thread-safe state access
static detection_state_t get_current_state(void);
static void set_current_state(detection_state_t new_state);
static void set_current_state_with_timer(detection_state_t new_state);

// ISR callback
static bool adc_isr_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

// ============================================================================
// FUNCTION IMPLEMENTATIONS
// ============================================================================

static void compute_hamming_window(void) {
    for (int i = 0; i < FRAME_SIZE; i++) {
        float n = (float)i;
        float N_1 = (float)(FRAME_SIZE - 1);
        // Hamming window: 0.54 - 0.46 * cos(2πn/(N-1))
        hamming_window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * n / N_1);
    }
}

static void update_snr_average(float new_snr_tone1, float new_snr_tone2) {
    // Add new values to circular buffer
    snr_tone1_history[snr_index] = new_snr_tone1;
    snr_tone2_history[snr_index] = new_snr_tone2;
    
    // Advance index (circular buffer)
    snr_index = (snr_index + 1) % SNR_AVERAGE_COUNT;
    if (snr_index == 0) {
        snr_buffer_full = true;
    }
    
    // Calculate averages
    float sum_tone1 = 0.0f;
    float sum_tone2 = 0.0f;
    uint32_t count = snr_buffer_full ? SNR_AVERAGE_COUNT : snr_index;
    
    for (uint32_t i = 0; i < count; i++) {
        sum_tone1 += snr_tone1_history[i];
        sum_tone2 += snr_tone2_history[i];
    }
    
    // Update global averages (protected by mutex in calling function)
    snr_tone1 = sum_tone1 / count;
    snr_tone2 = sum_tone2 / count;
}

static float goertzel(const float *samples, int N, float bin) {
    float omega = 2.0f * M_PI * bin / N;
    float coeff = 2.0f * cosf(omega);
    float q1 = 0.0f, q2 = 0.0f;
    
    for (int i = 0; i < N; i++) {
        float q0 = samples[i] + coeff * q1 - q2;
        q2 = q1;
        q1 = q0;
    }
    
    return sqrtf(q1*q1 + q2*q2 - q1*q2*coeff);
}

static bool IRAM_ATTR adc_isr_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t higher_woken = pdFALSE;
    
    if (worker_task_handle != NULL) {
        xTaskNotifyFromISR(worker_task_handle, 1, eSetValueWithOverwrite, &higher_woken);
    }
    
    return higher_woken == pdTRUE;
}

// Thread-safe state access functions
static detection_state_t get_current_state(void) {
    detection_state_t state = STATE_IDLE;
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = current_state;
        xSemaphoreGive(state_mutex);
    }
    return state;
}

static void set_current_state(detection_state_t new_state) {
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        current_state = new_state;
        xSemaphoreGive(state_mutex);
    }
}

static void set_current_state_with_timer(detection_state_t new_state) {
    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        current_state = new_state;
        state_timer = xTaskGetTickCount();
        xSemaphoreGive(state_mutex);
    }
}

static void worker_task(void *arg) {
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
                
                // Apply Hamming window and remove DC offset
                for (int i = 0; i < FRAME_SIZE; i++) {
                    samples[i] = ((float)p[i].type2.data - dc_offset) * hamming_window[i];
                }

                // Measure noise floor from bins away from both target frequencies
                // Use bins that are far from both TONE1_FREQ (bin 296) and TONE2_FREQ (bin 321)
                float noise_bins[] = {
                    goertzel(samples, FRAME_SIZE, 200.0f),  // 800 Hz
                    goertzel(samples, FRAME_SIZE, 250.0f),  // 1000 Hz
                    goertzel(samples, FRAME_SIZE, 350.0f),  // 1400 Hz
                    goertzel(samples, FRAME_SIZE, 400.0f),  // 1600 Hz
                    goertzel(samples, FRAME_SIZE, 450.0f),  // 1800 Hz
                };
                
                // Calculate average noise floor
                float noise_sum = 0.0f;
                for (int i = 0; i < 5; i++) {
                    noise_sum += noise_bins[i];
                }
                float noise_floor = noise_sum / 5.0f;

                // With Hamming window, use simplified single-bin detection
                // The 4-bin main lobe captures energy from ±5Hz frequency variations
                float mtone1_signal = goertzel(samples, FRAME_SIZE, (float)BIN_TONE1);      // bin 296 (TONE1_FREQ)
                float mtone2_signal = goertzel(samples, FRAME_SIZE, (float)BIN_TONE2);      // bin 321 (TONE2_FREQ)
                
                // Check for leakage from distant bins for differential detection
                float mtone1_leak_high = goertzel(samples, FRAME_SIZE, (float)(BIN_TONE1 + 3)); // bin 299
                float mtone1_leak_low = goertzel(samples, FRAME_SIZE, (float)(BIN_TONE1 - 3));  // bin 293
                float mtone2_leak_high = goertzel(samples, FRAME_SIZE, (float)(BIN_TONE2 + 3)); // bin 324
                float mtone2_leak_low = goertzel(samples, FRAME_SIZE, (float)(BIN_TONE2 - 3));  // bin 318
                
                // Differential detection with simplified energy calculation
                float mtone1_leakage = (mtone1_leak_high + mtone1_leak_low) / 2.0f;
                float mtone2_leakage = (mtone2_leak_high + mtone2_leak_low) / 2.0f;
                
                // Calculate SNR in dB for both frequencies (simplified with Hamming window)
                float signal_tone1 = fmaxf(0.0f, mtone1_signal - 0.1f*mtone1_leakage) * CAL_TONE1;
                float signal_tone2 = fmaxf(0.0f, mtone2_signal - 0.1f*mtone2_leakage) * CAL_TONE2;
                
                // Convert to SNR in dB (avoid log(0) by using small minimum value)
                float snr_tone1_db = 20.0f * log10f(fmaxf(signal_tone1, 1.0f) / fmaxf(noise_floor, 1.0f));
                float snr_tone2_db = 20.0f * log10f(fmaxf(signal_tone2, 1.0f) / fmaxf(noise_floor, 1.0f));

                /* Store averaged SNR values into globals protected by mutex */
                if (mag_mutex) {
                    if (xSemaphoreTake(mag_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        update_snr_average(snr_tone1_db, snr_tone2_db);
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

static void state_machine_task(void *pvParameters) {
    const float SNR_THRESHOLD = 20.0f;        // 20dB SNR detection threshold
    
    while (true) {
        if (mag_mutex && xSemaphoreTake(mag_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            float local_snr_tone1 = snr_tone1;
            float local_snr_tone2 = snr_tone2;
            xSemaphoreGive(mag_mutex);
            
            // State machine logic
            detection_state_t state = get_current_state();
            uint32_t current_tick = xTaskGetTickCount();
            
            switch (state) {
                case STATE_IDLE:
                    if (local_snr_tone1 > SNR_THRESHOLD) {
                        ESP_LOGI(TAG, "Tone1 detected (%.1fdB), starting timer", local_snr_tone1);
                        set_current_state_with_timer(STATE_TONE1_DETECTED);
                    }
                    break;
                    
                case STATE_TONE1_DETECTED:
                    if (local_snr_tone1 > SNR_THRESHOLD) {
                        // Check if we've held the tone for required duration
                        if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            if ((current_tick - state_timer) >= pdMS_TO_TICKS(TONE1_DURATION_MS)) {
                                ESP_LOGI(TAG, "Tone1 held for %lu ms, waiting for Tone2", TONE1_DURATION_MS);
                                current_state = STATE_WAIT_TONE2;
                                state_timer = current_tick;
                            }
                            xSemaphoreGive(state_mutex);
                        }
                    } else {
                        ESP_LOGI(TAG, "Tone1 lost, returning to idle");
                        set_current_state(STATE_IDLE);
                    }
                    break;
                    
                case STATE_WAIT_TONE2:
                    if (local_snr_tone2 > SNR_THRESHOLD) {
                        ESP_LOGI(TAG, "Tone2 detected (%.1fdB), starting timer", local_snr_tone2);
                        set_current_state_with_timer(STATE_TONE2_DETECTED);
                    } else {
                        // Timeout if we don't get Tone2 within reasonable time (10 seconds)
                        if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            if ((current_tick - state_timer) >= pdMS_TO_TICKS(10000)) {
                                ESP_LOGI(TAG, "Timeout waiting for Tone2, returning to idle");
                                current_state = STATE_IDLE;
                            }
                            xSemaphoreGive(state_mutex);
                        }
                    }
                    break;
                    
                case STATE_TONE2_DETECTED:
                    if (local_snr_tone2 > SNR_THRESHOLD) {
                        // Check if we've held the tone for required duration
                        if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            if ((current_tick - state_timer) >= pdMS_TO_TICKS(TONE2_DURATION_MS)) {
                                ESP_LOGI(TAG, "Tone2 held for %lu ms - SEQUENCE COMPLETE!", TONE2_DURATION_MS);
                                current_state = STATE_SEQUENCE_COMPLETE;
                                state_timer = current_tick;
                            }
                            xSemaphoreGive(state_mutex);
                        }
                    } else {
                        ESP_LOGI(TAG, "Tone2 lost, returning to IDLE");
                        set_current_state_with_timer(STATE_IDLE);
                    }
                    break;
                    
                case STATE_SEQUENCE_COMPLETE:
                    ESP_LOGI(TAG, "2-tone sequence completed! Ready for GPIO actions.");
                    // TODO: Add GPIO control tasks here
                    
                    // Reset to idle after 5 seconds
                    if (state_mutex && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        if ((current_tick - state_timer) >= pdMS_TO_TICKS(60000)) {
                            ESP_LOGI(TAG, "Resetting to idle state");
                            current_state = STATE_IDLE;
                        }
                        xSemaphoreGive(state_mutex);
                    }
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check state every 100ms
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
        .sample_freq_hz = SAMPLE_RATE,                    // 16384 Hz sample rate
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 1,
        .adc_pattern = &pattern,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_isr_callback,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void app_main(void) {

    compute_hamming_window();

    /* Create single worker task that will be notified by the ADC callback,
     * read frames from the driver and process them. */
    xTaskCreate(worker_task, "adc_worker", 4096, NULL, tskIDLE_PRIORITY + 2, &worker_task_handle);

    setup_adc_continuous();

    /* Initialize mutexes for thread safety */
    mag_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    
    /* Create state machine task for 2-tone sequence detection */
    xTaskCreate(state_machine_task, "state_machine", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    ESP_LOGI(TAG, "2-Tone Decoder initialized - waiting for tone sequence...");

    while (true) {
        if (mag_mutex && xSemaphoreTake(mag_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            float local_snr_tone1 = snr_tone1;
            float local_snr_tone2 = snr_tone2;
            xSemaphoreGive(mag_mutex);
            
            detection_state_t current = get_current_state();
            ESP_LOGI(TAG, "State: %d | SNR: Tone1=%.1fdB, Tone2=%.1fdB", 
                     current, local_snr_tone1, local_snr_tone2);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Ensure 1s between prints
    }
}