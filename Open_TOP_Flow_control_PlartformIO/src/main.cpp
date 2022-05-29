#include <Arduino.h>
#include "FCB.h"

Fluidic_Board board;


void setup() {
  Serial.begin(9600);
  board.setup();
}

void loop() {
  board.test_valves();
  board.test_flow_sensor();
  Serial.println("main loop");
  board.test_pressure(50, 2);
}