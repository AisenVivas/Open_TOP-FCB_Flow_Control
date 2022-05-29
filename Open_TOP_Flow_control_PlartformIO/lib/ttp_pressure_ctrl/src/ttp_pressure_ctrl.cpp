#include "ttp_pressure_ctrl.h"
// #include <Arduino.h>

// pressure_ctrl::pressure_ctrl(){}

pressure_ctrl::pressure_ctrl(   
                            int RXD, 
                            int TXD, 
                            int baudrate, 
                            HardwareSerial& _port_ser, 
                            Stream& port_ref) : _HardSerial(_port_ser), _port(port_ref){
    _pin_RXD = RXD; 
    _pin_TXD = TXD;
    _port_ser.begin(baudrate, SERIAL_8N1, _pin_RXD, _pin_TXD);

    // _port_ser.begin(baudrate);
}


void pressure_ctrl::setup(){
    
    //turn off data streaming mode
    _port.println("#W2,0");

    delay(100);
    
    //turn the pump off whilst configuring system
    _port.println("#W0,0");

    delay(100);

    //set the pump to PID control mode
    _port.println("#W10,1");

    delay(100);

    //set the PID setpoint to register 23
    _port.println("#W12,0");

    delay(100);

    //set the PID input to the pressure sensor (analog input 2);
    _port.println("#W13,2");

    delay(100);

    //set the PID pro_portional coefficient to 10
    _port.println("#W14,10");

    delay(100);

    //set the PID integral coefficient to 50
    _port.println("#W15,50");

    delay(100);

    //set the target pressure to 20
    _port.println("#W23,20");

    delay(100);

    //set the target pressure to 0
    _port.println("#W23,0");

    delay(100);

    //turn the pump on
    _port.println("#W0,1");

    delay(100);
}

void pressure_ctrl::set_pressure(float pressure_value){
    _port.println("#W23," + (String)pressure_value);
}

