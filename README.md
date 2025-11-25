# 2-Tone Decoder for ESP32-S3

A high-performance dual-tone sequential decoder designed for ESP32-S3 microcontrollers. This project implements real-time audio signal processing using the Goertzel algorithm to detect specific tone sequences and trigger GPIO outputs.

## Features

- **Dual-tone sequential detection** with configurable frequencies
- **Real-time signal processing** using Goertzel algorithm with Hamming windowing
- **Adaptive SNR averaging** for robust detection in noisy environments
- **GPIO control** with LED indicators and speaker (relay) output
- **Manual bypass switch** for testing and emergency activation
- **State machine architecture** for reliable sequence detection
- **Configurable parameters** via preprocessor defines

## Hardware Requirements

- **ESP32-S3** development board
- **ADC input** on GPIO2 for audio signal
- **LEDs** connected to GPIO4 (Tone1) and GPIO5 (Tone2)
- **Speaker (via relay)** connected to GPIO1
- **Bypass switch** connected to GPIO7 (with pulldown)
- **Audio input circuit** (amplifier/filter recommended)

## Default Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Tone 1 Frequency** | 1185.2 Hz | First tone to detect |
| **Tone 2 Frequency** | 1285.8 Hz | Second tone to detect |
| **Tone 1 Duration** | 500 ms | Required hold time for Tone 1 |
| **Tone 2 Duration** | 2000 ms | Required hold time for Tone 2 |
| **Sample Rate** | 8192 Hz | ADC sampling frequency |
| **Frame Size** | 2048 samples | Processing window size |
| **SNR Threshold** | 20.0 dB | Detection sensitivity |
| **SNR Averaging** | 2 frames | Noise reduction |

## GPIO Pin Assignments

| Pin | Function | Description |
|-----|----------|-------------|
| GPIO2 | ADC Input | Audio signal input (ADC1_CH1) |
| GPIO7 | LED 1 | Indicates waiting for Tone 1 |
| GPIO6 | LED 2 | Indicates waiting for Tone 2 |
| GPIO5 | Speaker LED | Indicates speaker state |
| GPIO1 | Speaker (relay) Output | Activates (connects speaker) when sequence complete |
| GPIO12 | Bypass Switch | Manual override (HIGH=bypass) |

## Operation Modes

### Normal Operation
1. **WAIT_TONE1**: LED1 ON, waiting for Tone 1
2. **TONE1_DETECTED**: LED1 ON, verifying Tone 1 duration
3. **WAIT_TONE2**: LED2 ON, waiting for Tone 2
4. **TONE2_DETECTED**: LED2 ON, verifying Tone 2 duration
5. **SEQUENCE_COMPLETE**: All LEDs OFF, Speaker ON (relay connects speaker)

### Bypass Mode
- **Switch HIGH**: Immediately activates speaker (bypass tone detection)
- **Switch LOW**: Returns to normal operation
- **HIGH→LOW transition**: Resets state machine to WAIT_TONE1

## Signal Processing

The decoder uses advanced DSP techniques for reliable detection:

- **Goertzel Algorithm**: Efficient frequency-domain analysis
- **Hamming Windowing**: Reduces spectral leakage and handles ±5Hz frequency variations
- **Adaptive SNR Averaging**: 2-frame moving average for noise rejection
- **Differential Detection**: Leakage compensation for improved selectivity
- **DC Offset Removal**: Automatic bias compensation

## Detection Performance

- **Detection Speed**: 0.25s initial, 0.5s stable
- **Frequency Resolution**: 4 Hz per bin
- **Frequency Tolerance**: ±5 Hz with Hamming window
- **SNR Sensitivity**: Configurable threshold (default 20 dB)
- **False Positive Rejection**: High (differential detection + averaging)

## Building and Flashing

### Prerequisites
- ESP-IDF v5.0 or later
- ESP32-S3 development board

### Build Commands
```bash
# Configure for ESP32-S3
idf.py set-target esp32s3

# Configure project (optional)
idf.py menuconfig

# Build project
idf.py build

# Flash and monitor
idf.py -p COMx flash monitor
```

## Configuration

All key parameters can be modified in `main/main.c`:

```c
// Tone frequencies
// Tone frequencies (typed constants)
static const float TONE1_FREQ = 1185.2f; // First tone frequency in Hz
static const float TONE2_FREQ = 1285.8f; // Second tone frequency in Hz

// Tone duration requirements (ms)
static const uint32_t TONE1_DURATION_MS = 500;
static const uint32_t TONE2_DURATION_MS = 2000;

// Detection sensitivity
static const float SNR_THRESHOLD = 20.0f; // SNR detection threshold in dB
```

## Timing Characteristics

| Event | Time |
|-------|------|
| **Frame Rate** | 4 Hz (250ms per frame) |
| **Detection Start** | 250ms (first frame) |
| **Stable Detection** | 500ms (full averaging) |
| **Minimum Sequence** | 2.75s (0.5s + 0.25s + 2s) |
| **State Check Rate** | 10 Hz (100ms intervals) |

## Troubleshooting

### Common Issues

**No tone detection:**
- Check audio input level and bias voltage
- Verify ADC reference voltage settings
- Adjust SNR_THRESHOLD for your signal levels

**False detections:**
- Increase SNR_THRESHOLD
- Check for noise on ADC input
- Verify ground connections

**Timing issues:**
- Check crystal oscillator accuracy
- Verify frame rate in logs

### Debug Output

Enable debug output by uncommenting the debug section in `app_main()`:
```c
// Uncomment for debug output
ESP_LOGI(TAG, "State: %d | SNR: Tone1=%.1fdB, Tone2=%.1fdB", 
         current, local_snr_tone1, local_snr_tone2);
```

## Architecture

The system uses a multi-task FreeRTOS architecture:

- **ADC Worker Task**: High-priority ADC data processing
- **State Machine Task**: Medium-priority sequence detection and GPIO control
- **Main Task**: Low-priority initialization and optional monitoring

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Author

**TheMadTexan42**

## Acknowledgments

- ESP-IDF framework by Espressif Systems
- Goertzel algorithm implementation optimized for real-time processing
- FreeRTOS for task management and synchronization