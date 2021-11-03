#include <Wire.h>
#include <PID_v1.h>

//Pins for valve control in ESP-32 LOLIN32
#define Ain1 26
#define Bin1 27

// pins for I2C 2nd channel serial communication
#define SDA_2 33
#define SCL_2 32

// Additional UART ports for pumps
//pump 1 Serial 1
#define RXD1 19
#define TXD1 23

//pump 2 Serial 2
#define RXD2 16
#define TXD2 17

// for PID control
double Input, Output, Setpoint;

//sample time (ms): how often the PID should update Output
//int SampleTime = 28;  // for pulsatile flow
int SampleTime = 80;    // for constant flow

//PID set value
float set_flow{};

//Initialize variable to store the flow value
float flow{}; 

//initialize variable for meassurement interval
unsigned long lastMeassure{};

// last data update time stamp
unsigned long data_update{};

//initialize variable for volume meassurement 
float volume_pumped{};

//initialize variable to keep track of valve position
int current_pump_position = 2;
int current_valve_position = 2;

//Variable to track time controlled recirculation switch
unsigned long recirculation_last_change{};

//Variables for signal generation
// unsigned long elapsed_time_signal = millis();
// float signal_counter{};
float sin_signal{};

//Variables for impulse step 
unsigned long imp_start_mcu{};
double imp_amp{};

//Variables for frequency sweep
float freq_sweep_start{};


void setup() {
  // serial connection for PC interaction
  Serial.begin(115200);

  //serial connections for pressure pumps
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // I2C bus initialization
  Wire.begin();
  
  //I2C Scan
  I2C_scanner();

  //pressure pumps setup
  pressure_pump_setup(Serial1);
  pressure_pump_setup(Serial2);
  
  //setup_temp_sensor();
  //start_temp_hum_sensor();
  
  //For valve control

  pinMode(Ain1, OUTPUT);  //Ain1
  pinMode(Bin1, OUTPUT);  //Bin1


  //Flow Sensor Setup
  flow_sensor_setup();

  //PID Control setup
  setup_PID(0, 0, 180, &SampleTime); //set flow, minimum Pressure pump, max Pressure pump, PID update in ms
  
  //start in position 2
  pos2();

  //print to serial monitor
  Serial.println("Ready!");

  //MCU start time stamp
  imp_start_mcu = millis();
}

void loop() {
  /*
    Below each of the operation modes can be found. 
    uncomment the desired function and load it to the MCU.
    Functions include:
      [] continuous flow:
        - Recirculation:
          [] Time controlled
          [] Volume controlled
        - Step impulse
          [] closed loop
          [] open loop
      [] Pulsatile flow
  */
  
//   cont_flow_recirculation_time_ctrl(6,                          // Avg. measurements
//                                      2000,                       // Cycle time in ms
//                                      100,                        // flow rate
//                                      &recirculation_last_change, // Last change time stamp
//                                      &current_pump_position);    // Var for tracking position

  
//   cont_flow_recirculation_vol_ctrl(6,                       // Avg. measurements
//                                     &volume_pumped,           // pumped volume var
//                                     100,                      // flow rate
//                                     50,                       // volume to be pumped in µL
//                                     &current_pump_position);  // Var for tracking position
  

  //Step impulse
  
  //  impulse_step_closed_loop(8000,      //start_delay_time
  //                            10000,    //imp_length
  //                            &imp_amp, //imp_amp
  //                            false,    //valve_off_end
  //                            true,     //multiple steps
  //                            250,      //step increase
  //                            750);     //max step
  
  // impulse_step_open_loop(8000,  //start_delay_time
  //                       10000,  //imp_length
  //                       50,     //pressure imp_amp
  //                       false); //close valves at the end


 
  
  //Pulsatile flow

//  pulsatile_flow(0.0625,                // signal frequency
//                80,                     // Signal amplitude
//                100,                    // Signal offset 
//                1000,                   // Flow target
//                &current_pump_position,  // Var for tracking position
//                &volume_pumped,         // pumped volume var
//                &Setpoint);
  
    // Pulsatile flow with delay
//   pulsatile_flow_delay(5000,      //start_delay_time
//                       100000,     //imp_length
//                       true,       //valve_off_end
//                       0.5,        //frequency
//                       30,         //amplitude
//                       100,        //offset
//                       &Setpoint); //signal output


    // Frequency Sweep with initial delay
//   pulsatile_flow_freq_sweep_delay(8000,               //start_delay_time
//                                   30000,              //imp_length
//                                   true,               //valve_off_end
//                                   &freq_sweep_start,  //
//                                   1,                  //Max frequency
//                                   30,                 //amplitude
//                                   100,                //offset
//                                   0.5,                //freq_increment
//                                   &Setpoint);         //signal output

  // Frequency sweep function
//  freq_sweep(5000,  //delay
//             0.0,   //start frequency
//             0.25,  //stop frequency
//             280.0, // total time in sec
//             50.0,  //amplitude
//             100.0, //offset
//             0.0,   //phase
//             &Setpoint);

  // Frequency sweep with two frequency changing rates
//  freq_sweep_ramps(10000, //delay
//                   1.0,   //start frequency
//                   0.00,   //stop frequency
//                   100.0, // total time in sec
//                   50.0,  //amplitude
//                   100.0, //offset
//                   0.0,   //phase
//                   &Setpoint);
}
