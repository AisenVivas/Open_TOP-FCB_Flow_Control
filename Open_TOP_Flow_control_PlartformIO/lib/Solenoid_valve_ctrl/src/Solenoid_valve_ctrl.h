#ifndef Solenoid_valve_ctrl
#define Solenoid_valve_ctrl

#include "Arduino.h"

class valve{
    public:

    void set_pin(int pin);
    void on();
    void off();
    
    private:
    int _pin;
};

class valve_pair{
    valve A, B;

    public:
    void setup(int pin_valve_A, int pin_valve_B);
    void pos1();
    void pos2();
    void valve_test(int time_interval);
    void switch_position();

    private:
    int _valve_pos{};
};

#endif