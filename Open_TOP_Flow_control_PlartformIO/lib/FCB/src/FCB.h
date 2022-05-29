#ifndef FCB
#define FCB

#include "Solenoid_valve_ctrl.h"
#include "Sensirion_Flow.h"
#include "ttp_pressure_ctrl.h"
// #include "Honeywell_ABP.h"

class Fluidic_Board{
    public:
    Fluidic_Board();
    void setup();
    void test_valves();
    //pressure in Pa and Time in seconds
    void test_pressure(int pressure, int interval);
    void test_flow_sensor();
    private:
    valve_pair _valves;
    flow_sensor _flow_sen;
    pressure_ctrl _p_ctrl1, _p_ctrl2;
};

#endif