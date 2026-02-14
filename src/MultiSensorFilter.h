#pragma once

/* FILTERS README below:
*
1. Exp IIR (1st order):
    Base Eqn: y[n] = alpha * x[n] + (1-alpha) * (y[n-1])


2. Butterworth 2nd order:
    Base Eqn: y[n]=b0​*x[n]+b1*​x[n−1]+b2*​x[n−2]−a1*​y[n−1]−a2*​y[n−2]

    ω0 = 2 * pi * fc / Fs       // Normalized angular frequency
    c = cos(ω0)                 // cos
    s = sin(ω0)                 // sin
    alpha = s / (2 * Q)         // Bandwidth parameter
    a0 = 1 + alpha              // Used for normalization

    Final coeff calculations:
        LPF:   
        a1 = (-2c) / a0
        a2 = (1 - alpha) / a0
        b0 = (1 - c) / 2a0
        b1 = (1 - c) / a0
        b2 = (1 - c) / 2a0
        
        HPF:
        a1 = (-2c) / a0
        a2 = (1 - alpha) / a0
        b0 = (1 + c) / 2a0
        b1 = -(1 + c) / a0
        b2 = (1 + c) / 2a0
        
        BPF: (constant skirt gain, peak gain = Q)
        a1 = (-2c) / a0
        a2 = (1 - alpha) / a0
        b0 = (alpha * Q) / a0
        b1 = 0
        b2 = (-alpha * Q) / a0

        NOTCH:
        a1 = (-2c) / a0
        a2 = (1 - alpha) / a0
        b0 = 1 / a0
        b1 = (-2c) / a0
        b2 = 1 / a0

3. Kalman
    Prediction:
    P[n-1] = P[n-1] + Q -> Error covariance prediction

    Update:
    K[n] = P[n-1] / (P[n-1] + R) -> Kalman gain
    z[n] = ADC READ
    x[n] = x[n-1] + K[n](z[n] - x[n-1]) -> State update
    P[n] = (1-K[n])*P[n-1] -> Error covariance update

*/

#include "Arduino.h"
#include <math.h>

#define MULTI_SENSOR_FILTER_VERSION (F("0.0.3"))

// Filter types
typedef enum : uint8_t {
    FILTER_NONE = 0,
    // FIRs
    MOV_AVG,    
    MEDIAN,
    // IIRs
    EXPONENTIAL, // 1st order IIR
    BUTTER2_LPF,
    BUTTER2_HPF,
    BUTTER2_BPF,
    BUTTER2_NOTCH,

    LINEAR_KALMAN,
    // Reserve

    TOT_ANALOG_FILTERS  // MOV_AVG = 1, MEDIAN = 2, EXPONENTIAL = 3, BUTTER2_LPF = 4, BUTTER2_HPF = 5, BUTTER2_BPF = 6, BUTTER2_NOTCH = 7
} AnalogFilters;

struct FilterConfig {
    uint8_t gpio;                // GPIO pin number
    const char* name;            // Device name, for user reference (not used in library now)
    AnalogFilters filter_type;   // Filter type
    float param1;                // window size (mov avg, median); alpha (exp iir); or cutoff freq (2nd order)
    float param2;                // Q value for 2nd order butterworth
};

class MultiSensorFilter {

public:
    MultiSensorFilter();
    
    /**
    *  Initialises all filter channels with circular buffers. Safe, dynamically allocated memory at first run.
    *
    * \param[in] the config list from .ino
    * \param[in] list_len calcualted from .ino
    * \param[in] signal acquisition rate. OPTIONAL, needed for higher order IIRs. Default = 100Hz
    * \return none
    */
    void Init(FilterConfig* config_list, size_t list_len, float acq_hz);

    /**
    *  User API for returning the filtered value of each GPIO pin
    *
    * \param[in] GPIO pin
    * \return Filtered value
    */
    float analog_filter(uint8_t pin);
    

    // Utility functions, exposed to user
    /**
    *  Returns currently running no. of filters
    *
    * \return number of filters declared by user
    */
    size_t get_filter_count() { return _filter_count; }

    /**
    *  Returns used defined signal acq rate
    *
    * \return user defined signal acq rate
    */
    float get_acq_hz() { return _acq_hz; }

private:

    struct Filter {
        uint8_t pin, idx = 0;
        uint8_t filter_type;  // 1=MOV_AVG, 2=MEDIAN, 3=EXPONENTIAL, 4=BUTTER2_LPF, 5=BUTTER2_HPF, 6=BUTTER2_BPF, 7=BUTTER2_NOTCH

        float param1 = 1.0f;    // default value
        float param2 = 0.707f;  // default Q is 1/sqrt(2)
        
        union {
            uint16_t* ibuff;    
            float* fbuff;
        };
        int sum = 0;   // only used for rolling mov avg
    };

    Filter* _filters;
    size_t _filter_count;
    float _acq_hz;
    uint8_t _current_filter_idx;

    // Filter implementations

    /**
    *  Returning the mov-avg filtered value of selected GPIO pin
    *
    * \param[in] GPIO pin
    * \param[in] window or sample size
    * \return Filtered value
    */
    float _mov_avg_filter_read(uint8_t pin, uint8_t filter_samples);

    /**
    *  Returning the median filtered value of selected GPIO pin
    *
    * \param[in] GPIO pin
    * \param[in] window or sample size
    * \return Filtered value
    */
    float _median_filter_read(uint8_t pin, uint8_t filter_samples);

    /**
    *  Returning the exp / 1st order iir filtered value of selected GPIO pin
    *
    * \param[in] GPIO pin
    * \param[in] alpha value [0, 1]
    * \return Filtered value
    */
    float _exp_iir_filter_read(uint8_t pin, float alpha);

    /**
    *  Different 2nd order Butterworth filters require different coeff values, this init works for that
    *
    * \param[in] filter_type from AnalogFilters enum
    * \param[in] idx value from the config_list
    * \return none
    */
    void _butter2_init(uint8_t filter_type, uint8_t idx);

    /**
    *  Returning the 2nd order butterworth (LPF/HPF/BPF/Notch) filtered value of selected GPIO pin
    *
    * \param[in] GPIO pin
    * \return Filtered value
    */
    float _butter_order2_read(uint8_t pin);
    
    /**
    *  Returning the Linear Kalman filtered value of selected GPIO pin
    *
    * \param[in] GPIO pin
    * \param[in] process noise variance Q
    * \param[in] measurement noise variance R
    * \return Filtered value
    */
    float _linear_kalman_read(uint8_t pin, float Q, float R);
    

    // Utility functions
    /**
    *  Returns filter index of the GPIO pin being called at Arduino IDE void loop()
    *
    * \param[in] GPIO pin
    * \return idx of called filter
    */
    int8_t _find_filter_index(uint8_t pin);

    static constexpr float _ADC_SCALE = 3.3 / 4095.0;
};