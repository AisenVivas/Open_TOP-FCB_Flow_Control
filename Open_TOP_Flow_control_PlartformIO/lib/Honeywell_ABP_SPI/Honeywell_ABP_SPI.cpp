#include "Honeywell_ABP_SPI.h"

p_sensor::p_sensor(int pin, double p_min, double p_max){
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    _pin = pin;
    _pmin = p_min;
    _pmax = p_max;
    SPI.begin();
}

void p_sensor::update(){
    // get data
    SPI.beginTransaction(SPISettings(80000, MSBFIRST, SPI_MODE0)); //SPI at 80kHz
    digitalWrite(10, LOW);
    SPI.transfer(_data, 4);
    digitalWrite(_pin, HIGH);
    SPI.endTransaction();
}

void p_sensor::getPressure(){
    
    //process data
    byte statusBits = _data[0];
    statusBits &= 0b11000000;       //select the two first bits
    _data[0] &= 0b00111111;         //select the remainder bits of the byte - MSB from pressure

    //combine the pressure bytes DataByte1 and DataByte2 into one 14-bit number:
    _press_counts = ((_data[0] << 8) | (_data[1]));

    //use the transfer function A from datasheet
    _pressure = ((_press_counts - _outputmin) * (_pmax - _pmin)) / (_outputmax - _outputmin) + _pmin;
    dtostrf(_pressure, 4, 1, _pBuffer);
    // Serial.println(_pBuffer);
}

void p_sensor::getTemp(){

    //process data into 11 bit digit
    _temp_counts = ((_data[2] << 3) | (_data[3]));

    //use tranfer function for temperature
    _temperature = ((_temp_counts_bytes / 2047) * 200) - 50;
    dtostrf(_temperature, 4, 1, _tBuffer);
    // Serial.println(_tBuffer);
}

void p_sensor::stream_P_and_T(){
    p_sensor::getPressure();
    p_sensor::getTemp();
    sprintf(_printBuffer, "%s\t%s", _pBuffer, _tBuffer);
    Serial.println(_printBuffer);
}