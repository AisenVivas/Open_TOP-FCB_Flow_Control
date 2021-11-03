// -----------------------------------------------------------------------------
// Data logging with Python: 
//  This function transmits all data to the PC using a python script to log it.
// -----------------------------------------------------------------------------

void python_log(bool python, unsigned long *last_data_update) {
  if (python == true && millis() - *last_data_update >= SampleTime) {
    Serial.println("f");
    Serial.println((float)flow, 4);
    Serial.println("p");
    Serial.println((float)Output);
    Serial.println("v");
    Serial.println((float)current_pump_position);
    Serial.println("r");
    Serial.println((float)volume_pumped);
    Serial.println("a");
    Serial.println((float)Setpoint);
    Serial.println("t");
    Serial.println((float)(millis()), 4);
    Serial.println("q");
    Serial.println(freq_sweep_start, 6);

    // Save time stamp last update
    *last_data_update = millis();
  }
}
