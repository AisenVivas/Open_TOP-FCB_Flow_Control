#ifndef ttp_pressure_ctrl
#define ttp_pressure_ctrl
#include <Stream.h>
#include <HardwareSerial.h>


class pressure_ctrl{
    public:
    // pressure_ctrl();
    pressure_ctrl(int RXD, int TXD, int baudrate, HardwareSerial& _port_ser, Stream& port_ref);
    void setup();
    void set_pressure(float pressure_value);

    private:
    int _pin_RXD, _pin_TXD;

    //declared to access the .begin() command
    HardwareSerial& _HardSerial;

    //declared to send commands
    Stream& _port;
};
#endif