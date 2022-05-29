/*
  Code adapted from https://github.com/Sensirion/arduino-liquid-flow-snippets
*/

/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef Sensirion_Flow
#define Sensirion_Flow

#include <Arduino.h>
#include <Wire.h>

class flow_sensor{
    public:

    flow_sensor();
    void set_address(int address);
    void verbose();
    void setup();
    void set_resolution(uint8_t sensor_resolution);
    void soft_reset();
    void read_scale_factor();
    void meassurement_mode();
    void temp();
    void print_temp();
    void setup(int resolution);
    void measure();
    void avg_measure(int num_val_to_avg);
    void volume_measurement(int num_val_to_avg);
    void print_flow(int interval);



    private:

    int _ret;
    uint16_t _raw_sensor_value, _user_reg, _scale_factor_address, _unit_code;
    float _sensor_reading, _sensor_temp_reading;
    int _ADDRESS = 0x44; // Standard address for Liquid Flow Sensors
    byte _crc1, _crc2;
    
    #define _CRC_POLYNOMIAL 0x131  // P(x)=x^8+x^5+x^4+1 (binary 0001 0011 0001)
    bool _VERBOSE_OUTPUT = false; // set to false for less verbose output

    // EEPROM Addresses for factor and unit of calibration fields 0,1,2,3,4.
    const uint16_t _SCALE_FACTOR_ADDRESSES[5] = {0x2B6, 0x5B6, 0x8B6, 0xBB6, 0xEB6};
    const uint16_t UNIT_ADDRESSES[5] = {0x2B7, 0x5B7, 0x8B7, 0xBB7, 0xEB6};

    // Flow Units and their respective codes.
    const char    *FLOW_UNIT[5] = {"nl/min", "ul/min", "ml/min", "ul/sec", "ml/h"};
    const uint16_t FLOW_UNIT_CODES[5] = {2115, 2116, 2117, 2100, 2133};

    uint16_t _scale_factor;
    const char *unit;

    //Read temperature and voltage readings
    const float SCALEFACTOR_FLOW = 1.0f;
    const float _SCALEFACTOR_TEMP = 10.0f;
    const float SCALEFACTOR_VDD  = 1000.0f;

    const byte TRIGGER_FLOW_COMMAND = 0xF1;
    const byte _TRIGGER_TEMP_COMMAND = 0xF3;
    const byte TRIGGER_VDD_COMMAND  = 0xF5;

    const char *MEASUREMENT_TYPE[3] = {"Flow (signed value)", "Temp", "VDD"};
    const char *MEASUREMENT_UNIT[3] = {""," C"," Volt"};

    uint16_t _reg;

    // set sensor resolution
    uint8_t _sensor_resolution;
    uint16_t _adv_user_reg_original, _adv_user_reg_new, _resolution_mask;

    // Variables for flow measurement
    int _i{}, _j{};
    float _flow{}, _avg_flow{}, _recorded_volume{};
    byte _crc, _data[2], _calc_crc;
};

#endif