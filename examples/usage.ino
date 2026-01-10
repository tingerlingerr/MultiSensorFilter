#include "SoftwareAnalogFilters.h"

SoftwareAnalogFilters filt;

FilterConfig Config[] = {
    {33, "Sens1", MOV_AVG, 10},             // Moving Average Filter - @param1: window size               
    {32, "Sens2", BUTTER2_BPF, 4, 0.707},   // 2nd order Butterworth BPF Filter - @param1: cutoff freq, @param2: Q-factor
    {35, "Sens3", MEDIAN, 5},               // Median Filter - @param1: window size
    {34, "Sens4", EXPONENTIAL, 0.75},       // 1st order IIR / Exponential Filter - @param1: alpha
    {39, "Sens5", BUTTER2_LPF, 2, 0.707},   // 2nd order Butterworth LPF Filter - @param1: cutoff freq, @param2: Q-factor
    {36, "Sens6", BUTTER2_HPF, 2, 0.707},   // 2nd order Butterworth HPF Filter - @param1: cutoff freq, @param2: Q-factor
    {15, "Sens7", BUTTER2_NOTCH, 2, 0.707}  // 2nd order Butterworth Notch Filter - @param1: cutoff freq, @param2: Q-factor
};  // filter names are CaSe_SeNsItIvE
const size_t config_len = sizeof(Config) / sizeof(Config[0]);
  
void setup() {
  Serial.begin(9600);
  filt.Init(Config, config_len, 500); // 500 Hz acquisition rate
  
  Serial.println("Raw,Filtered"); // Serial Plotter
}

void loop() {
  
  float raw = analogRead(32) * 3.3 / 4095.0;  
  float filtered = filt.analog_filter(32);

  Serial.print(raw); Serial.print(",");
  Serial.println(filtered); 

  delay(2);
}
