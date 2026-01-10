#include "SoftwareAnalogFilters.h"

SoftwareAnalogFilters::SoftwareAnalogFilters() : _filters(nullptr), _filter_count(0), _acq_hz(100), _current_filter_idx(0) {
    // constructor
}

void SoftwareAnalogFilters::Init(FilterConfig* config_list, size_t list_len, float acq_rate_hz) {
    
    if(_filters!= nullptr) free(_filters);
    
    if(config_list == nullptr || list_len == 0) {
        Serial.println("No filters configured! Nothing to initialize.");
        return;
    }

    if(acq_rate_hz<=0) {
        Serial.println("Negative or Zero acqusition rate impossible!!"); return;
    } 

    _filter_count = list_len;
    _acq_hz = acq_rate_hz;

    _filters = (Filter*)calloc(_filter_count, sizeof(Filter));
    if(!_filters) {
        Serial.println("Failed to allocate, memory shortage");
        return;    // alloc failed, memory low
    }

    memset(_filters, 0, _filter_count * sizeof(Filter));

    for(size_t i = 0; i < _filter_count; i++) {
        _filters[i].pin = config_list[i].gpio;
        _filters[i].filter_type = config_list[i].filter_type;
        _filters[i].param1 = config_list[i].param1;
        _filters[i].param2 = config_list[i].param2;
        _filters[i].idx = 0;

        pinMode(_filters[i].pin, INPUT);

        switch(_filters[i].filter_type) {

            case 1: { // MOV_AVG
                uint8_t samples = (uint8_t)_filters[i].param1;
                if (samples > 50) samples = 50;  // moving avg window capped at 50

                _filters[i].ibuff = (uint16_t*)calloc(samples, sizeof(uint16_t));
                if (!_filters[i].ibuff) return;
                
                for (uint8_t n = 0; n < samples; n++) {
                    _filters[i].ibuff[n] = analogRead(_filters[i].pin);
                    _filters[i].sum += _filters[i].ibuff[n];
                    delay(1);
                }
                break;
            }

            case 2:  { // MEDIAN
                uint8_t samples = (uint8_t)_filters[i].param1;
                if (samples > 50) samples = 50;  // median filter window capped at 50
                _filters[i].ibuff = (uint16_t*)calloc(samples, sizeof(uint16_t));
                if (!_filters[i].ibuff) return;
                
                for (uint8_t n = 0; n < samples; n++) {
                    _filters[i].ibuff[n] = analogRead(_filters[i].pin);
                    delay(1);
                }
                break;
            }

            case 3: { // EXPONENTIAL
                if(config_list[i].param1 < 0 || config_list[i].param1 > 1) return;

                _filters[i].ibuff = (uint16_t*)calloc(2, sizeof(uint16_t));
                if (!_filters[i].ibuff) return;

                _filters[i].ibuff[0] = 0;
                _filters[i].ibuff[1] = analogRead(_filters[i].pin);

                break;
            }

            case 4:     // BUTTER2 LPF
            case 5:     // BUTTER2 HPF
            case 6:     // BUTTER2 BPF
            case 7: {   // BUTTER2 NOTCH
                if(config_list[i].param1 < acq_rate_hz / 2) return;
                if(config_list[i].param2 < 0) return;


                _filters[i].fbuff = (float*)calloc(11, sizeof(float));
                if (!_filters[i].fbuff) return;

                _butter2_init(_filters[i].filter_type, i);

                break;
            }
        }// Switch
    }

    Serial.printf("%d filters initialized.\n", (uint8_t)_filter_count);

}// init

float SoftwareAnalogFilters::analog_filter(uint8_t pin) {
    int filter_idx = _find_filter_index(pin);
    if (filter_idx == -1) return analogRead(pin) * _ADC_SCALE;

    _current_filter_idx = filter_idx;
    Filter& filt = _filters[filter_idx];

    switch (filt.filter_type) {
        case 1: return _mov_avg_filter_read(pin, filt.param1);
        case 2: return _median_filter_read(pin, filt.param1);
        case 3: return _exp_iir_filter_read(pin, filt.param1);
        case 4: 
        case 5: 
        case 6: 
        case 7: 
                return _butter_order2_read(pin);
        default: 
                return analogRead(pin) * _ADC_SCALE;
    }
}// .ino API end here








// Filter implementation methods

// 1. MOV_AVG
float SoftwareAnalogFilters::_mov_avg_filter_read(uint8_t pin, uint8_t filter_samples) {
    if (filter_samples <= 1) return analogRead(pin) * _ADC_SCALE;
    
    uint16_t new_val = analogRead(pin);
    Filter& filt = _filters[_current_filter_idx];

    filt.sum = filt.sum - filt.ibuff[filt.idx] + new_val;
    filt.ibuff[filt.idx] = new_val;
    filt.idx = (filt.idx + 1) % filter_samples;
    
    return (filt.sum / filter_samples) * _ADC_SCALE;
}

// 2. MEDIAN
float SoftwareAnalogFilters::_median_filter_read(uint8_t pin, uint8_t filter_samples) {
    
    if(filter_samples <= 1) return analogRead(pin) * _ADC_SCALE;
    
    uint16_t new_val = analogRead(pin);
    Filter& filt = _filters[_current_filter_idx];

    filt.ibuff[filt.idx] = new_val;
    filt.idx = (filt.idx + 1) % filter_samples;

    uint16_t* temp = (uint16_t*)malloc(filter_samples * sizeof(uint16_t));
    if (!temp) return new_val * _ADC_SCALE;

    memmove(temp, filt.ibuff, filter_samples * sizeof(uint16_t));

    // Insertion sort
    for (uint8_t i = 1; i < filter_samples; ++i) {
        uint16_t key = temp[i];
        int8_t j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    float median_val;
    if (filter_samples % 2 == 1) {
        median_val = (float)temp[filter_samples / 2];
    } else {
        median_val = ((float)temp[filter_samples/2 - 1] + (float)temp[filter_samples/2]) / 2.0f;
    }

    free(temp);

    if (abs((int)new_val - (int)median_val) < 50)
        return median_val * _ADC_SCALE;
    else
        return new_val * _ADC_SCALE;
}

// 3. EXPONENTIAL
float SoftwareAnalogFilters::_exp_iir_filter_read(uint8_t pin, float alpha) {
    
    /*
        Base Eqn: y[n] = _alpha * x[n] + (1-_alpha) * (y[n-1])
        lower alpha, greater smoothening but slower
        
        pseudocode:
        init buff:
            buff[0] = prev output -> y[n-1]
            buff[1] = curr output -> y[n]
        left move:
            buff[0] <- buff[1]; buff[1] becomes garbage.
        run filt eqn:
            get new curr output, immediately put into buff[1].
            return buff[1]
        loop
    */
    
    Filter& filt = _filters[_current_filter_idx];
    
    
    memmove(&filt.ibuff[0], &filt.ibuff[1], sizeof(uint16_t));
    float filt_op = alpha * analogRead(pin) + (1 - alpha) * filt.ibuff[0];
    filt.ibuff[1] = filt_op;

    return filt_op * _ADC_SCALE;
}

// 4. BUTTER 2ND ORDER BUTTERWORTH FILTERS
void SoftwareAnalogFilters::_butter2_init(uint8_t filter_type, uint8_t idx) {
    Filter& filt = _filters[idx];
    
    float _w0 = 2 * PI * filt.param1 / _acq_hz;
    float _c = cos(_w0);
    float _s = sin(_w0);
    float _alpha = _s / (2 * filt.param2);
    float _a0 = 1 + _alpha;

    switch(filter_type) {
        case 4: // LPF
            filt.fbuff[0] = (1 - _alpha) / _a0;     // a2
            filt.fbuff[1] = (-2 * _c) / _a0;        // a1
            filt.fbuff[2] = (1 - _c) / (2 * _a0);   // b2
            filt.fbuff[3] = (1 - _c) / _a0;         // b1
            filt.fbuff[4] = (1 - _c) / (2 * _a0);   // b0
            break;
        case 5: // HPF
            filt.fbuff[0] = (1 - _alpha) / _a0;
            filt.fbuff[1] = (-2 * _c) / _a0;
            filt.fbuff[2] = (1 + _c) / (2 * _a0);
            filt.fbuff[3] = -(1 + _c) / _a0;
            filt.fbuff[4] = (1 + _c) / (2 * _a0);
            break;
        case 6: // BPF
            filt.fbuff[0] = (1 - _alpha) / _a0;
            filt.fbuff[1] = (-2 * _c) / _a0;
            filt.fbuff[2] = (-_alpha * filt.param2) / _a0;
            filt.fbuff[3] = 0.0f;
            filt.fbuff[4] = (_alpha * filt.param2) / _a0;
            break;
        case 7: // NOTCH
            filt.fbuff[0] = (1 - _alpha) / _a0;
            filt.fbuff[1] = (-2 * _c) / _a0;
            filt.fbuff[2] = 1 / _a0;
            filt.fbuff[3] = (-2 * _c) / _a0;
            filt.fbuff[4] = 1 / _a0;
            break;
    }

    filt.fbuff[5] = 0.0f; //x[n-2]
    filt.fbuff[6] = 0.0f; //x[n-1]
    filt.fbuff[7] = _ADC_SCALE * analogRead(filt.pin); //x[n]

    filt.fbuff[8] = 0.0f; //y[n-2]
    filt.fbuff[9] = 0.0f; //y[n-1]
    filt.fbuff[10] = filt.fbuff[4] * filt.fbuff[7]; //y[n] = first y[n] = b0 * x[n] + 0 + 0 - 0 - 0

}

float SoftwareAnalogFilters::_butter_order2_read(uint8_t pin) {
    
    /*
        Base Eqn: y[n] = b0 ​* x[n] + b1 * ​x[n−1] + b2 * ​x[n−2] − a1 * ​y[n−1] − a2 * ​y[n−2]
                : y[n] = buff[4]*buff[7] + buff[3]*buff[6] + buff[2]*buff[5] - buff[1]*buff[9] - buff[0]*buff[8]
        
        buff 5 to 10 need to be shifted:
        buff[5] = x[n-2]
        buff[6] = x[n-1]
        buff[7] = x[n]
        buff[8] = y[n-2]
        buff[9] = y[n-1]
        buff[10]= y[n]

        pseudocode:
        init buff:
            buff[7] = first adc read
            buff[10] = calculate first y[n]

        left moves:
            for x:
            buff[5] <- buff[6]
            buff[6] <- buff[7] (prev input)
            buff[7] = curr adc read

            for y:
            buff[8] <- buff[9]
            buff[9] <- buff[10]
            buff[10]= garbage

        run filt eqn:
            get new curr output, immediately put into buff[10].
            return buff[10]
        

    */
    
    
    Filter& filt = _filters[_current_filter_idx];
    
    // Shift input history
    memmove(&filt.fbuff[5], &filt.fbuff[6], sizeof(float));
    memmove(&filt.fbuff[6], &filt.fbuff[7], sizeof(float));
    filt.fbuff[7] = analogRead(pin) * _ADC_SCALE;

    // Shift y
    memmove(&filt.fbuff[8], &filt.fbuff[9], sizeof(float));
    memmove(&filt.fbuff[9], &filt.fbuff[10], sizeof(float));

    // Calculate new y
    float filt_op = filt.fbuff[4] * filt.fbuff[7] // b0 * xn
                  + filt.fbuff[3] * filt.fbuff[6] // b1 * x(n-1)
                  + filt.fbuff[2] * filt.fbuff[5] // b2 * x(n-2)
                  - filt.fbuff[1] * filt.fbuff[9] // a1 * y(n-1)
                  - filt.fbuff[0] * filt.fbuff[8];// a2 * y(n-2)
    
    filt.fbuff[10] = filt_op;
    return filt_op;
}

// Utility functions
inline int8_t SoftwareAnalogFilters::_find_filter_index(uint8_t pin) {
    for (size_t i = 0; i < _filter_count; i++) {
        if (_filters[i].pin == pin)
            return i;
    }
    return -1;
}
