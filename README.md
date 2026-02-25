# 2‑Tone Decoder for ESP32‑S3  
**Real‑time dual‑tone sequence detector using Goertzel analysis + FreeRTOS**

This project implements a **2‑tone sequential decoder** (similar to fire‑pager / selective‑call systems) on an **ESP32‑S3**.  
It continuously samples audio via the ADC, applies a **Hamming window**, computes tone magnitudes using the **Goertzel algorithm**, and runs a **state machine** to detect a specific two‑tone sequence.

When the correct sequence is detected, the system activates a **speaker relay** and indicator LEDs. A physical **bypass switch** allows manual override.

---

## ✨ Features

- **Real‑time ADC sampling** using ESP‑IDF’s continuous ADC driver  
- **Goertzel‑based tone detection** optimized for two specific frequencies  
- **Hamming window** to reduce spectral leakage  
- **SNR‑based detection** with averaging and noise‑floor estimation  
- **Robust state machine** with timing requirements for each tone  
- **Bypass/override mode** via momentary active‑low switch  
- **LED indicators** for Tone1, Tone2, and Speaker state  
- **Speaker relay control** (active‑low)  
- **ADC saturation detection** and logging  
- **Thread‑safe shared data** using FreeRTOS mutexes  
- **Two‑task architecture** (worker + state machine)

---

## 📡 Tone Detection Parameters

The system detects two tones using Goertzel bins:

| Tone | Frequency (Hz) | FFT Bin | Notes |
|------|----------------|---------|-------|
| Tone 1 | ~1989 Hz | 497 | Must be held for **500 ms** |
| Tone 2 | ~2401 Hz | 600 | Must be held for **2000 ms** |

These bins are derived from:

- **Sample rate:** 8192 Hz  
- **Frame size:** 2048 samples  
- **Bin resolution:** 4 Hz  

You can change the tone frequencies by adjusting `BIN_TONE1` and `BIN_TONE2`.

---

## 🧠 System Architecture

### Tasks

| Task | Purpose |
|------|---------|
| **worker_task** | Reads ADC frames, applies windowing, computes Goertzel magnitudes, updates SNR values |
| **state_machine_task** | Runs the tone‑sequence state machine, handles bypass switch, updates GPIO outputs |

### State Machine

```
STATE_WAIT_TONE1
    ↓ (Tone1 SNR > threshold for 500 ms)
STATE_TONE1_DETECTED
    ↓
STATE_WAIT_TONE2
    ↓ (Tone2 SNR > threshold for 2000 ms)
STATE_TONE2_DETECTED
    ↓
STATE_SEQUENCE_COMPLETE
    ↓ (auto‑reset after 60s or button press)
STATE_WAIT_TONE1
```

---

## 🔧 Hardware Connections

### GPIO Assignments

| Function | GPIO | Polarity |
|----------|------|----------|
| LED Tone1 | 7 | Active‑high |
| LED Tone2 | 6 | Active‑high |
| LED Speaker | 5 | Active‑high |
| Speaker Relay | 1 | **Active‑low** (LOW = speaker ON) |
| Bypass Switch | 12 | Active‑low, internal pull‑up enabled |

Additional unused pins (3,4,8,9,10,11,13) are driven LOW to reduce noise.

### ADC Input

- ADC Unit 1, Channel 1 (GPIO2)
- 12‑bit resolution
- 12 dB attenuation
- Continuous sampling at **8192 Hz**

---

## 🧮 Signal Processing Pipeline

1. **Read 2048‑sample frame** from ADC  
2. **Compute DC offset** and detect saturation  
3. **Apply Hamming window**  
4. **Compute Goertzel magnitude** for:  
   - Target bins (Tone1, Tone2)  
   - Neighbor bins (leakage estimation)  
   - Several unrelated bins (noise floor estimation)  
5. **Compute SNR (dB)** for each tone  
6. **Update averaged SNR values**  
7. **State machine consumes SNR values**

---

## 🚦 Bypass / Manual Override

A momentary active‑low button toggles bypass mode:

- If pressed while idle → **force SEQUENCE_COMPLETE**
- If pressed while active → **return to WAIT_TONE1**

On boot, the system **starts in bypass mode**.

---

## 🛠 Building & Flashing

This project is built using **ESP‑IDF v5.5.2** and developed with the **ESP‑IDF Extension for VS Code**, which provides integrated configuration, building, flashing, and monitoring tools.

You may build and flash the firmware using either the **VS Code GUI workflow** or the **command‑line tools** included with ESP‑IDF.

---

### 🔹 ESP‑IDF Version

This project targets:

- **ESP‑IDF:** v5.5.2  
- **Target chip:** `esp32s3`  
- **Toolchain:** Installed automatically by the ESP‑IDF VS Code extension or via `install.sh` / `install.bat`

Make sure your environment is set to ESP‑IDF 5.5.2 before building.

---

## 🚀 Building & Flashing Using VS Code (Recommended)

The ESP‑IDF VS Code extension provides a streamlined workflow:

1. Open the project folder in VS Code  
2. Ensure the ESP‑IDF extension is installed and configured  
3. In the left ESP‑IDF sidebar:
   - Click **Set Target** → choose **ESP32‑S3**
   - Click **Build** to compile the project  
   - Click **Flash** to upload firmware to the board  
   - Click **Monitor** to view serial output

The extension automatically handles:

- Environment activation  
- CMake configuration  
- Toolchain selection  
- Serial port detection  

This is the easiest and most reliable workflow for ESP‑IDF 5.5.2.

---

## 🚀 Building & Flashing Using Command Line

If you prefer the terminal, activate your ESP‑IDF environment and run:

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash
idf.py monitor
```

Or combine flashing + monitoring:

```bash
idf.py flash monitor
```

### Notes

- The project uses **CMake**, which is handled automatically by `idf.py`.
- Ensure your USB serial port is accessible (e.g., `/dev/ttyACM0`, `/dev/ttyUSB0`, or COMx on Windows).
- If you have multiple ESP devices connected, specify the port:

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

---

## 🧩 Project Structure

```
main/
 └── main.c        # Full application: ADC, DSP, state machine, GPIO, tasks
CMakeLists.txt
README.md
```

---

## ⚠️ Important Build Notes

- This project uses **continuous ADC mode**, which is fully supported in ESP‑IDF 5.5.2.
- If you switch ESP‑IDF versions, re-run:
  ```bash
  idf.py fullclean
  ```
  to avoid cached build artifacts from older toolchains.
- Always verify that the VS Code extension is using the correct ESP‑IDF version (shown in the bottom status bar).


---

## 📁 File Overview

```
main/
 └── main.c        # Full application: ADC, DSP, state machine, GPIO, tasks
CMakeLists.txt
README.md
```

---

## 📝 Configuration Notes

You may want to adjust:

- `BIN_TONE1` / `BIN_TONE2` for different tone frequencies  
- `TONE1_DURATION_MS` / `TONE2_DURATION_MS`  
- `SNR_THRESHOLD`  
- GPIO assignments  
- ADC attenuation depending on input amplitude  

---

## 🧪 Debugging & Logging

The system logs:

- ADC saturation events  
- Tone detection events  
- State transitions  
- Bypass switch actions  

Use:

```bash
idf.py monitor
```

---

## 📜 License

This project is released under the **Creative Commons Zero v1.0 Universal (CC0)** dedication.

You may use this software for **any purpose**, including commercial, private, or educational use.  
You may modify it, redistribute it, sublicense it, or incorporate it into any other work without restriction.  
**Attribution is not required.**  
**No warranty is provided.**

For full legal text, see: https://creativecommons.org/publicdomain/zero/1.0/

