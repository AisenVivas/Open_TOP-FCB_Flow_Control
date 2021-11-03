// -----------------------------------------------------------------------------
// Pulsatile flow
// -----------------------------------------------------------------------------

const float pi = 3.14159;

void pulsatile_flow(float frequency,
                    float amplitude,
                    float ofset,
                    float target_vol,
                    int *current_pump_position,
                    float *volume,
                    double *sin_signal) {

  float sample_period = 1000.0 / frequency;
  *sin_signal = sin(2.000 * pi * (float)millis() * (frequency / 1000.000)) * amplitude + ofset;

  measured_vol = avg_flow_measurement_integration(6, &flow);
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

  // set the pressure of pump #1 at the output pressure of the flow PID
  set_pressure(*current_pump_position, &Output);

  //send data to serial via USB
  python_log(true, &data_update);
}

// -----------------------------------------------------------------------------
// Pulsatile flow with delay
// -----------------------------------------------------------------------------
int pulse_signal_state = 0;
void pulsatile_flow_delay(unsigned long start_delay_time,
                          unsigned long imp_length,
                          bool valve_off_end,
                          float frequency,
                          float amplitude,
                          float ofset,
                          double *sin_signal) {

  //Start signal after delay
  if (millis() > start_delay_time + imp_start_mcu && pulse_signal_state == 0) {
    start_PID_time = millis();
    pulse_signal_state = 1;
  }


  if (pulse_signal_state == 1) {
    float sample_period = 1000.0 / frequency;
    *sin_signal = sin(2.000 * pi * (float)millis() * (frequency / 1000.000)) * amplitude + ofset;
  }

  //end signal after length of input has passed
  if (millis() - start_PID_time > imp_length && pulse_signal_state == 1) {
    Setpoint = 0.0;
    Output = 0.0;
    myPID.SetMode(MANUAL);
    pulse_signal_state = 2;
  }

  if (millis() - start_PID_time > imp_length + start_delay_time && pulse_signal_state == 2 && valve_off_end) {
    valve_A_off();
  }

  //meassure flow
  avg_flow_measurement(6, &flow);

  // calls the PID function and calculates the pressure output to correct the volumetric flow
  PID_flow_control(&flow);

  // set the pressure of pump #1 at the output pressure of the flow PID
  set_pressure(1, &Output);

  //send data to serial via USB
  python_log(true, &data_update);
}

// -----------------------------------------------------------------------------
// Frequency Sweep with initial delay
// -----------------------------------------------------------------------------

unsigned long last_freq_change{};
void pulsatile_flow_freq_sweep_delay(unsigned long start_delay_time,
                                     unsigned long imp_length,
                                     bool valve_off_end,
                                     float *freq_sweep_start,
                                     float max_frequency,
                                     float amplitude,
                                     float ofset,
                                     float freq_increment,
                                     double *sin_signal) {
  //Start signal after delay
  
  if (millis() > start_delay_time + imp_start_mcu && state_imp_step == 0) {
    start_PID_time = millis();
    state_imp_step = 1;
    pulse_signal_state = 1;
  }


  if (pulse_signal_state == 1) {
    float sample_period = 1000.0 / max_frequency;
    *sin_signal = sin(2.000 * pi * (float) millis() * (*freq_sweep_start / 1000.000)) * amplitude + ofset;
  }

  //end signal after length of input has passed
  if (millis() - start_PID_time > imp_length && state_imp_step == 1) {
    //turn off PID control
    Setpoint = 0.0;
    Output = 0.0;
    myPID.SetMode(MANUAL);
    state_imp_step = 4;
    pulse_signal_state = 0;
  }

  if (millis() - start_PID_time > imp_length + start_delay_time && state_imp_step == 4 && valve_off_end) {
    Serial.println("done ");
    valve_A_off();
    state_imp_step = 4;
  }

  //increment frequency every 10 periods by freq_increment
  if (millis() - last_freq_change > 10000 + start_delay_time && *freq_sweep_start < max_frequency && pulse_signal_state == 1) {
    *freq_sweep_start = *freq_sweep_start + freq_increment;
    last_freq_change = millis();
    if (*freq_sweep_start > max_frequency) {
      pulse_signal_state = 0;
      *sin_signal = ofset;
    }
  }

  //meassure flow
  avg_flow_measurement(6, &flow);

  // calls the PID function and calculates the pressure output to correct the volumetric flow
  PID_flow_control(&flow);

  // set the pressure of pump #1 at the output pressure of the flow PID
  set_pressure(1, &Output);

  //send data to serial via USB
  python_log(true, &data_update);
}

// -----------------------------------------------------------------------------
// Frequency sweep function
// -----------------------------------------------------------------------------

void freq_sweep(double delay_sweep,
                double f0,
                double f1,
                double T,
                double A,
                double ofset,
                double phi,
                double *sin_signal) {
  double t = 0, k = 0, fre = 0, y = 0;

  t = ((double)millis() - delay_sweep - imp_start_mcu) / 1000.0;
  k = (f1 - f0) / T;
  fre = (k) * t + f0;

  if (millis() < delay_sweep + imp_start_mcu) {
    *sin_signal = ofset;
  }

  if (millis() > delay_sweep + imp_start_mcu && fre <= f1) {
    *sin_signal = A * cos(2 * pi * fre * t + phi) + ofset;
    freq_sweep_start = fre; //used to print the actual freq. to the python script
  }

  if (fre > f1) {
    *sin_signal = 100.0;
    freq_sweep_start = 0;
  }

  //meassure flow
  avg_flow_measurement(6, &flow);

  // calls the PID function and calculates the pressure output to correct the volumetric flow
  PID_flow_control(&flow);

  // set the pressure of pump #1 at the output pressure of the flow PID
  set_pressure(1, &Output);

  //send data to serial via USB
  python_log(true, &data_update);
}
// -----------------------------------------------------------------------------
// Frequency sweep with two frequency changing rates
// -----------------------------------------------------------------------------

double start{};
int freq_sweep_state{1};
void freq_sweep_ramps(double delay_sweep,                   // start delay in seconds 
                      double f0,                            // start frequency
                      double f1,                            // end frequency
                      double T,                             // signal time length
//                      double time_fraction;
                      double A,                             // amplitude
                      double ofset,                         // signal offset
                      double phi,                           // phase
                      double *sin_signal) {                 // Signal variable pointer
  // Variables initialization
  double t{}, k{}, fre{}, y{}, f_start{}, f_end{};

  // Convert time to seconds and offset it to 0
  t = ((double) millis() - delay_sweep - imp_start_mcu) / 1000.0;

  // During signal delay
  if (t < 0) {
    fre = f0;
    *sin_signal = A * cos(fre * pi * t + phi) + ofset;
  }

  // during the first frequency change rate
  if (t >= 0 && t < T) {
    f_start = f0;
    f_end = f0 / 5.0;
    fre = ((f_end - f_start) / T) * t + f0;
    *sin_signal = A * cos(pi * fre * (t - T) + phi) + ofset;
    freq_sweep_state = 1;
  }

  if (t>= T && freq_sweep_state == 1){
    start = t;
    freq_sweep_state = 0;
  }

  if (t > T && t <= 6 * T) {
    f_start = f0/5.0;
    f_end = f1;
    t -= start;
    fre = ((f_end - f_start) / (5*T)) * t + f_start;
    *sin_signal = A * cos(pi * fre * (t - 6 * T) + phi) + ofset;
  }

  if (t > 6 * T) {
    *sin_signal = 0.0;
    //    valve_A_off();
    //    fre = 0.01;
    //    *sin_signal = A * cos(pi * fre * (t - T) + phi) + ofset;
  }

  //meassure flow
  avg_flow_measurement(6, &flow);

  // calls the PID function and calculates the pressure output to correct the volumetric flow
  PID_flow_control(&flow);

  // set the pressure of pump #1 at the output pressure of the flow PID
  set_pressure(1, &Output);

  //send data to serial via USB
  freq_sweep_start = fre; //used to print the actual freq. to the python script
  python_log(true, &data_update);
}