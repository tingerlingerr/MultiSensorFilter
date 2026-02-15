# Multi Sensor Software Analog Filters for Arduino ![C++](https://img.shields.io/badge/C++-red) ![Arduino](https://img.shields.io/badge/Arduino-teal)

## **About**
A lightweight, efficient Arduino library that implements various software-based analog filters aimed for real-time signal processing. Large-scale projects usually use multimodal sensor data, each of which may require different filters. This library was born from a genuine requirement at my research job. Perfect for quick experimentation between different filters for smoothing sensor data, removing noise, and signal conditioning without additional hardware.

## Features

- **Multiple Analog Sensors at a go**: Mention direct GPIO pin number. Add as many analog sensors as you want. 
- **Multiple Filter Types**: Rolling Moving Average, Rolling Median, Exponential IIR, and 2nd Order IIR Butterworth filters
- **Configurable Parameters**: Easy setup through configuration structures
- **Memory Efficient**: Dynamic allocation only for required, active filters
- **Real-time Filtering**: Aimed for embedded systems
- **Arduino Compatible**: Works with any Arduino-IDE compatible board with analog pins


## :clipboard: **Folder Structure**

```
MultiSensorFilter/
├── examples/
├── results/
├── src/
├ Changelog
├ LICENSE
├ library.properties
├ keywords
└ README
```

## Supported Filters

| Filter Type | Description | Parameters |
|-------------|-------------|------------|
| `MOV_AVG` | Moving Average | `param1`: Window size (2-50 samples), `param2`: NA |
| `MEDIAN` | Median Filter | `param1`: Window size (2-50 samples), `param2`: NA |
| `EXPONENTIAL` | Exponential IIR Smoothing | `param1`: Alpha (0-1, lower = more smoothing, slower response), `param2`: NA |
| `BUTTER2_LPF` | 2nd Order Butterworth Low Pass | `param1`: Cutoff freq (Hz), `param2`: Q factor |
| `BUTTER2_HPF` | 2nd Order Butterworth High Pass | `param1`: Cutoff freq (Hz), `param2`: Q factor |
| `BUTTER2_BPF` | 2nd Order Butterworth Band Pass | `param1`: Center freq (Hz), `param2`: Q factor |
| `BUTTER2_NOTCH` | 2nd Order Butterworth Notch | `param1`: Notch freq (Hz), `param2`: Q factor |
| `LINEAR_KALMAN` | Simple Kalman Filter | `param1`: Q (process noise variance), `param2`: R (measurement noise variance) |

## Installation

### Method 1: Arduino Library Manager
1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for "SoftwareAnalogFilters"
4. Click Install

### Method 2: Manual Installation
1. Download the latest release from GitHub
2. Extract to your Arduino libraries folder
3. Restart Arduino IDE

## Quick Start

### Basic example

```.ino
#include "MultiSensorFilter.h"

MultiSensorFilter filt;

// Define your filter configuration
FilterConfig Config[] = {
    {GPIO0, "Sensor1", MOV_AVG, 10},            // Moving average, 10 samples
    {GPIO1, "Sensor2", BUTTER2_LPF, 5, 0.707},  // Low pass, 5Hz cutoff
    {GPIO3, "Sensor3", EXPONENTIAL, 0.2}        // Exponential smoothing
};
const size_t config_len = sizeof(Config) / sizeof(Config[0]);

void setup() {
    Serial.begin(9600);
    filt.Init(Config, config_len, 100); // 100Hz system acquisition rate
}

void loop() {
    float filtered_value = filt.analog_filter(GPIO0);
    Serial.println(filtered_value);
    delay(10);
}

```
### Exemplary plots
Refer results folder

## Troubleshooting

**Q: Filter not working on my pin**
A: Ensure the pin is configured in your FilterConfig array and properly initialized.

**Q: Butterworth filter behaving strangely**
A: Check that your acquisition rate (acq_hz) is correctly set and at least 2× your filter frequency.

**Q: Memory allocation errors**
A: Reduce filter window sizes or number of active filters to conserve memory.

## Future Work (not arranged in priority)
- Currently assumes all input GPIOs need a Filter. That may not be the case.
- Needs to write better find_filters_idx code. Currently runs a loop for each GPIO, not ideal. Have thought of introducing a context / filter entry pointer.
- Add more filters.
- The .ino interface needs some refactoring. Automatic calculation of the size of FilterConfig array needs to be done at the backend.
- Only works for ADC reads. Should work for digital sensor data like I2C, SPI, UART also.

---
## Inspiration / Credits

1. https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
2. https://e2e.ti.com/support/audio-group/audio/f/audio-forum/911062/ccs-tas5825m-we-need-software-source-code-of-the-ppc3-eq-module
3. N. IWANAGA, T. MATSUMURA, A. YOSHIDA, W. KOBAYASHI, and T. ONOYE, “Embedded System Implementation of Sound Localization in Proximal Region,” IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences, doi: https://doi.org/10.1093/ietfec/e91-a.3.763.
4. https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf


---