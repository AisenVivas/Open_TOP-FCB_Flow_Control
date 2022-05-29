#include "Solenoid_valve_ctrl.h"

void valve::set_pin(int pin){
    _pin = pin;
}

void valve::on(){
    digitalWrite(_pin, HIGH);
}

void valve::off(){
    digitalWrite(_pin, LOW);
}

//26 and 27 for ESP32 and 26 and 28 for Arduino Mega 2560
void valve_pair::setup(int pin_valve_A, int pin_valve_B){ 
    A.set_pin(pin_valve_A);
    B.set_pin(pin_valve_B);
}

void valve_pair::pos1(){
    A.on();
    B.off();
}

void valve_pair::pos2(){
    A.off();
    B.on();
}

void valve_pair::valve_test(int time_interval = 2000){
    valve_pair::pos1();
    delay(time_interval);
    valve_pair::pos2();
    delay(time_interval);
}

void valve_pair::switch_position(){
    if (_valve_pos == 1){
        valve_pair::pos2();
        _valve_pos = 2;
    }
    else{
        valve_pair::pos1();
        _valve_pos = 1;
    }
}