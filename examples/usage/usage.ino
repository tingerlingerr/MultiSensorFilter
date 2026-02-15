#include "MultiSensorFilter.h"

#define test_sens 33

MultiSensorFilter filt;

FilterConfig Config[] = {
    {test_sens, "Sens1", MOV_AVG, 10},        // Moving Average Filter - @param1: window size               
    {32, "Sens2", BUTTER2_BPF, 4, 0.707},     // 2nd order Butterworth BPF Filter - @param1: cutoff freq, @param2: Q-factor
    {35, "Sens3", MEDIAN, 5},                 // Median Filter - @param1: window size
    {34, "Sens4", EXPONENTIAL, 0.75},         // 1st order IIR / Exponential Filter - @param1: alpha
    {39, "Sens5", BUTTER2_LPF, 2, 0.707},     // 2nd order Butterworth LPF Filter - @param1: cutoff freq, @param2: Q-factor
    {36, "Sens6", BUTTER2_HPF, 2, 0.707},     // 2nd order Butterworth HPF Filter - @param1: cutoff freq, @param2: Q-factor
    {14, "Sens7", BUTTER2_NOTCH, 2, 0.707},   // 2nd order Butterworth Notch Filter - @param1: cutoff freq, @param2: Q-factor
    {15, "Sens8", LINEAR_KALMAN, 0.05, 0.1},  // Linear Kalman filter - @param1: Q, @param2: R
};  // filter names are CaSe_SeNsItIvE
const size_t config_len = sizeof(Config) / sizeof(Config[0]);
  
void setup() {
  Serial.begin(9600);
  filt.Init(Config, config_len, 500); // 500 Hz acquisition rate
  
  Serial.println("Raw,Filtered"); // Serial Plotter
}

void loop() {
  
  float raw = analogRead(test_sens) * 3.3 / 4095.0;  
  float filtered = filt.analog_filter(test_sens);

  float differential = filt.differential_read(test_sens, 32);

  float fused = filt.simple_sensor_fusion({
                  {test_sens, 0.4},
                  {32, 1.6},
                  });

  Serial.print(raw); Serial.print(",");
  Serial.print(filtered); Serial.print(",");
  Serial.print(differential); Serial.print(",");
  Serial.println(fused);

  delay(2);
}