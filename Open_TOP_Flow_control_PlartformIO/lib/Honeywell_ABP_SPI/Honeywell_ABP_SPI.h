#ifndef Honeywell_ABP_SPI
#define Honeywell_ABP_SPI

#include "Arduino.h"
#include <SPI.h>

class p_sensor {

public:
    p_sensor(int pin, double p_min, double p_max);
    void update();
    void getPressure();
    void getTemp();
    void stream_P_and_T();
    boolean begin();
    
private:
    int _pin;
    // Calibration parameters based on datasheet Figure 4
    double _press_counts = 0; // digital pressure reading [counts]
    double _temp_counts = 0; // digital temperature reading [counts]
    byte _temp_counts_bytes; // holds the bits of the temperature data
    double _pressure = 0; // pressure reading [bar, psi, kPa, etc.]
    double _temperature = 0; // temperature reading in deg C
    double _outputmax = 14745; // output at maximum pressure [counts]
    double _outputmin = 1638; // output at minimum pressure [counts]
    double _pmax = 1; // maximum value of pressure range [bar, psi, kPa, etc.]
    double _pmin = -1; // minimum value of pressure range [bar, psi, kPa, etc.]
    double _percentage = 0; // holds percentage of full scale data
    char _printBuffer[200], _pBuffer[20], _tBuffer[20]; // holds strings of values
    byte _data[4] = {0x00, 0x00, 0x00, 0x00}; // holds output data
};
#endif