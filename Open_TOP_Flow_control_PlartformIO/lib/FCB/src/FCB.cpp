#include "FCB.h"
Fluidic_Board::Fluidic_Board(){

}
void Fluidic_Board::setup(){
    //setup flow sensor
    _flow_sen.verbose();
    _flow_sen.setup(13);
    
    //Setup valves pins
    _valves.setup(26,28);

    pressure_ctrl _p_ctrl1(19, 23, 9600, Serial1, Serial1), _p_ctrl2(16, 17, 9600, Serial2, Serial2);
}


void Fluidic_Board::test_valves(){  //Test the valves
    _valves.valve_test(1000);  
}

void Fluidic_Board::test_pressure(int pressure, int interval){
    
    interval*=1000;
    _p_ctrl1.set_pressure(pressure);
    delay(interval);
    _p_ctrl1.set_pressure(0);

    _p_ctrl2.set_pressure(pressure);
    delay(interval);
    _p_ctrl2.set_pressure(0);
    Serial.println("test finished");

}

void Fluidic_Board::test_flow_sensor(){
    _flow_sen.print_flow(1000);
}