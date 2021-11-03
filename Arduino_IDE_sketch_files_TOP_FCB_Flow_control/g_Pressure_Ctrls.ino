// -----------------------------------------------------------------------------
// Setup pressure pump function
// -----------------------------------------------------------------------------
void pressure_pump_setup(Stream &port) {
  
  //turn off data streaming mode
  port.println("#W2,0");

  delay(100);

  //turn the pump off whilst configuring system
  port.println("#W0,0");

  delay(100);

  //set the pump to PID control mode
  port.println("#W10,1");

  delay(100);

  //set the PID setpoint to register 23
  port.println("#W12,0");

  delay(100);

  //set the PID input to the pressure sensor (analog input 2);
  port.println("#W13,2");

  delay(100);

  //set the PID proportional coefficient to 10
  port.println("#W14,10");

  delay(100);

  //set the PID integral coefficient to 50
  port.println("#W15,50");

  delay(100);

  //set the target pressure to 0
  port.println("#W23,20");

  delay(100);

  port.println("#W23,0");

  delay(100);

  //turn the pump on
  port.println("#W0,1");

  delay(100);
}

// -----------------------------------------------------------------------------
// Set pump pressure function
// -----------------------------------------------------------------------------

void set_pressure(int current_pump_position, double *output) {
  if (current_pump_position == 1) {
    Serial2.println("#W23,0");
    Serial1.println("#W23," + (String)*output);
  } else {
    Serial1.println("#W23,0");
    Serial2.println("#W23," + (String)*output);
  }
}

// -----------------------------------------------------------------------------
// Set ressure pump time controlled function
// -----------------------------------------------------------------------------

void cont_flow_recirculation_time_ctrl(int num_meassurements,
                                              unsigned long cycle_period, 
                                              float flow_rate,
                                              unsigned long *recirculation_last_change, 
                                              int *current_pump_position) {

  // set flow rate
  Setpoint = flow_rate;
  
  // set the pressure of the current pump at the output pressure of the flow PID
  set_pressure(*current_pump_position, &Output);

  if (millis() - *recirculation_last_change > cycle_period) {
    if (*current_pump_position == 1) {
      change_valve_position(&current_valve_position);
      *current_pump_position = 2;
    } else {
      change_valve_position(&current_valve_position);
      *current_pump_position = 1;
    }
    *recirculation_last_change = millis();
  }

  // Makes n meassurements specified by num_meassurements and saves it in the *flow* variable
  avg_flow_measurement(num_meassurements, &flow);

  // calls the PID function and calculates the pressure output to correct the volumetric flow
  PID_flow_control(&flow);

  python_log(true, &data_update);
}

// -----------------------------------------------------------------------------
// Set ressure pump volume controlled function
// -----------------------------------------------------------------------------

float measured_vol{};
void cont_flow_recirculation_vol_ctrl(int num_meassurements, 
                                      float *volume,
                                      float flow_rate, 
                                      float target_vol, 
                                      int *current_pump_position) {
  
  // set flow rate
  Setpoint = flow_rate;
  
  // set the pressure of the current pump at the output pressure of the flow PID
  set_pressure(*current_pump_position, &Output);
  
  measured_vol = avg_flow_measurement_integration(num_meassurements, &flow);
  *volume += measured_vol;
  
  if (*volume - (target_vol * 1000.0) > 0) { // conversion from µL to nL
    
    if (*current_pump_position == 1) {
      change_valve_position(&current_valve_position);
      *current_pump_position = 2;
    } else {
      change_valve_position(&current_valve_position);
      *current_pump_position = 1;
    }
    *volume = 0.0;
  }

  // calls the PID function and calculates the pressure output to correct the volumetric flow
  PID_flow_control(&flow);

  python_log(true, &data_update);

}

// -----------------------------------------------------------------------------
// Step function flow control - Closed loop
// -----------------------------------------------------------------------------
int state_imp_step = 0; // Switch variable for impulse step
unsigned long start_PID_time = 0;

void impulse_step_closed_loop(unsigned long start_delay_time,
                  unsigned long imp_length,
                  double *imp_amp,
                  bool valve_off_end,
                  bool mult_steps,
                  double step_increase,
                  double max_step_amp) {

  // Start impulse step after delay period
  if (millis() > start_delay_time + imp_start_mcu && state_imp_step == 0) {
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    //setpoint of the PID
    Setpoint = *imp_amp;
    start_PID_time = millis();
    state_imp_step = 1;
  }

  // end impulse step
  if (millis() - start_PID_time > imp_length && state_imp_step == 1) {
    //turn off PID control
    Setpoint = 0.0;
    Output = 0.0;
    state_imp_step = 2;
    myPID.SetMode(MANUAL);
  }

  // close valve if valve_off_end is true
  if (millis() - start_PID_time > imp_length + start_PID_time && state_imp_step == 2 && valve_off_end) {
    valve_A_off();
    state_imp_step = 3;
  }

  // end step impulse and increase the step amplitude by step_increase
  if (millis() - start_PID_time > imp_length && (state_imp_step == 3 || state_imp_step == 2) && mult_steps) {
    if (*imp_amp > max_step_amp) {
      state_imp_step = 4;
    }
    *imp_amp = *imp_amp + step_increase;
    state_imp_step = 0;
  }
  
  // Makes n meassurements specified by num_meassurements and saves it in the *flow* variable
  avg_flow_measurement(6, &flow);

  // calls the PID function and calculates the pressure output to correct the volumetric flow
  PID_flow_control(&flow);

  // set the pressure of pump #1 at the output pressure of the flow PID
  set_pressure(1, &Output);

  python_log(true, &data_update);
}

// -----------------------------------------------------------------------------
// Step function flow control - Open loop
// -----------------------------------------------------------------------------
void impulse_step_open_loop(unsigned long start_delay_time,
                            unsigned long imp_length,
                            double P_imp_amp,
                            bool valve_off_end) {

  // Start impulse step after delay period
  if (millis() > start_delay_time + imp_start_mcu && state_imp_step == 0) {
    
    //setpoint of the pressure controller
    Output = P_imp_amp;
    set_pressure(1, &Output);

    //record start time
    start_PID_time = millis();

    //change the phase of the process variable
    state_imp_step = 1;
  }

  // impulse step
  if (millis() - start_PID_time > imp_length && state_imp_step == 1) {
    
    //Set input pressure to 0
    Output = 0;
    set_pressure(1, &Output);

    //change the phase of the process variable
    state_imp_step = 2;
  }

  // end step impulse and close valve if valve_off_end is true
  if (millis() - start_PID_time > imp_length + start_PID_time && state_imp_step == 2 && valve_off_end) {
    valve_A_off();
    state_imp_step = 3;
  }
  
  // Makes n meassurements specified by num_meassurements and saves it in the *flow* variable
  avg_flow_measurement(6, &flow);

  //print sensor data to serial for python recording script
  python_log(true, &data_update);
}
