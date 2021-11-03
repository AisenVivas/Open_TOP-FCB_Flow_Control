// -----------------------------------------------------------------------------
// Basic functions for valve control
// -----------------------------------------------------------------------------

//valve A
void valve_A_on(){
  digitalWrite(Ain1, HIGH);
}

void valve_A_off(){
  digitalWrite(Ain1, LOW);
}

//valve B
void valve_B_on(){
  digitalWrite(Bin1, HIGH);
}

void valve_B_off(){
  digitalWrite(Bin1, LOW);
}

void pos1() {
  valve_A_on();
  valve_B_off();
}

void pos2() {
  valve_A_off();
  valve_B_on();
}

// function for testing the pins values
void valve_cycle(int time_cycle) {
  pos1();
  delay(time_cycle);
  pos2();
  delay(time_cycle);
}

// -----------------------------------------------------------------------------
// Valve control for recirculation protocol based on time interval
// -----------------------------------------------------------------------------

void valve_position_change(unsigned long recirculation_cycle_time_min, 
                            unsigned long *recirculation_last_change, int *current_valve_position ) {

  if (millis() - *recirculation_last_change > recirculation_cycle_time_min) {

    if (*current_valve_position == 1) {
      
      pos2();
      *current_valve_position = 2;
    }
    else {
      
      pos1();
      *current_valve_position = 1;
    }

    //save time stamp of last change
    *recirculation_last_change = millis();
  }
}

// -----------------------------------------------------------------------------
// Valve control for recirculation protocol based on volume control
// -----------------------------------------------------------------------------

void change_valve_position(int *current_valve_position) {
  if (*current_valve_position == 1) {
    pos2();
    *current_valve_position = 2;
  }
  else {
    pos1();
    *current_valve_position = 1;
  }
}
