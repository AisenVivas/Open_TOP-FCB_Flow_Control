/*
  Code adapted from https://github.com/Sensirion/arduino-liquid-flow-snippets
*/

/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Sensirion_Flow.h>


flow_sensor::flow_sensor(){
  // _VERBOSE_OUTPUT = verbose_set;
}

//Set Address in the hexadecimal format 0xFF
void flow_sensor::set_address(int address){
  _ADDRESS = address;
}

void flow_sensor::verbose(){
  _VERBOSE_OUTPUT = true;
}

void flow_sensor::soft_reset(){
    // Soft reset the sensor
    Wire.beginTransmission(_ADDRESS);
    Wire.write(0xFE);
    _ret = Wire.endTransmission();
    if (_ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      // continue;
    }
    delay(50); // wait long enough for reset
}

void flow_sensor::read_scale_factor(){
  // Read the user register to get the active configuration field
  Wire.beginTransmission(_ADDRESS);
  Wire.write(0xE3);
  _ret = Wire.endTransmission();

  if (_ret != 0) {
    Serial.println("Error while setting register read mode");
    // continue;
  }

  Wire.requestFrom(_ADDRESS, 2);
  if (Wire.available() < 2) {
    Serial.println("Error while reading register settings");
    // continue;
  }
  _user_reg  = Wire.read() << 8;
  _user_reg |= Wire.read();

  // The active configuration field is determined by bit <6:4>
  // of the User Register
  _scale_factor_address = _SCALE_FACTOR_ADDRESSES[(_user_reg & 0x0070) >> 4];
  
  // Read scale factor and measurement unit
  Wire.beginTransmission(_ADDRESS);
  Wire.write(0xFA); // Set EEPROM read mode
  // Write left aligned 12 bit EEPROM address
  Wire.write(_scale_factor_address >> 4);
  Wire.write((_scale_factor_address << 12) >> 8);
  _ret = Wire.endTransmission();
  if (_ret != 0) {
    Serial.println("Error during write EEPROM address");
    // continue;
  }

  Wire.requestFrom(_ADDRESS, 6);

  if (Wire.available() < 6) {
    Serial.println("Error while reading EEPROM");
    // continue;
  }
  _scale_factor = Wire.read() << 8;
  _scale_factor|= Wire.read();
  _crc1         = Wire.read();
  _unit_code    = Wire.read() << 8;
  _unit_code   |= Wire.read();
  _crc2         = Wire.read();

  switch (_unit_code) {
    case 2115:
      { unit = FLOW_UNIT[0]; }
      break;
    case 2116:
      { unit = FLOW_UNIT[1]; }
      break;
    case 2117:
      { unit = FLOW_UNIT[2]; }
      break;
    case 2100:
      { unit = FLOW_UNIT[3]; }
      break;
    case 2133:
      { unit = FLOW_UNIT[4]; }
      break;
    default:
      Serial.println("Error: No matching unit code");
      break;
  }

  if (_VERBOSE_OUTPUT) {
    Serial.println();
    Serial.println("-----------------------");
    Serial.print("Scale factor: ");
    Serial.println(_scale_factor);
    Serial.print("Unit: ");
    Serial.print(unit);
    Serial.print(", code: ");
    Serial.println(_unit_code);
    Serial.println("-----------------------");
    Serial.println();
  }
}

void flow_sensor::meassurement_mode(){
  Wire.beginTransmission(_ADDRESS);
  Wire.write(0xF1);
  _ret = Wire.endTransmission();
  if (_ret != 0) {
    Serial.println("Error during write measurement mode command");
  }
}

void flow_sensor::temp(){

  Wire.beginTransmission(_ADDRESS);
  Wire.write(_TRIGGER_TEMP_COMMAND);
  _ret = Wire.endTransmission();
  if (_ret != 0){
    Serial.println("Error during write measurement mode");
    // continue;
  }

  Wire.requestFrom(_ADDRESS, 2);
  if (Wire.available() < 2) {
    Serial.println("Error during read");
    // continue;
  }
  _raw_sensor_value  = Wire.read() << 8;
  _raw_sensor_value |= Wire.read();

  _sensor_temp_reading = ((int16_t) _raw_sensor_value) / _SCALEFACTOR_TEMP;
}

void flow_sensor::print_temp(){
  flow_sensor::temp();
  Serial.println((String)_sensor_temp_reading);
}

void flow_sensor::set_resolution(uint8_t sensor_resolution){
    
  if(sensor_resolution < 9){
    _sensor_resolution = 9;
  }

  if(sensor_resolution > 16){
    _sensor_resolution = 16;
  }

  _resolution_mask = 0xF1FF | ((_sensor_resolution - 9) << 9);

  // Change mode to read adv. user register
  Wire.beginTransmission(_ADDRESS);
  Wire.write(0xE5);
  _ret = Wire.endTransmission();
  if (_ret != 0) {
    Serial.println("Error during write register read mode");
  }
  else {
    // Read the content of the adv user register
    Wire.requestFrom(_ADDRESS, 2);
    if (Wire.available() < 2) {
      Serial.println("Error during read register settings");

    } 
    else {
      _adv_user_reg_original  = Wire.read() << 8;
      _adv_user_reg_original |= Wire.read();
      _adv_user_reg_new = (_adv_user_reg_original | 0x0E00) & _resolution_mask;
    }
    if (_VERBOSE_OUTPUT) {
      // Display register values and settings
      Serial.println();
      Serial.println("----------------");
      Serial.print("New resolution setting:       ");
      Serial.println(_sensor_resolution);
      Serial.print("Resolution bit setting (BIN): ");
      Serial.println(_sensor_resolution - 9, BIN);
      Serial.print("Resolution mask:     ");
      Serial.println(_resolution_mask, BIN);
      Serial.print("Adv. user reg read:   ");
      Serial.println(_adv_user_reg_original, BIN);
      Serial.print("Adv. user reg write:  ");
      Serial.println(_adv_user_reg_new, BIN);
      Serial.println("----------------");
    }

    // Apply resolution changes:
    // Change mode to write to adv. user register
    Wire.beginTransmission(_ADDRESS);
    Wire.write(0xE4);                               // Send command
    Wire.write((byte)(_adv_user_reg_new >> 8));      // Send MSB
    Wire.write((byte)(_adv_user_reg_new & 0xFF));    // Send LSB
    _ret = Wire.endTransmission();
    if (_ret != 0) {
      Serial.println("Error during write register settings");
    }
  }
}

void flow_sensor::setup(int resolution){

  Wire.begin();         // join i2c bus (address optional for master)

  do {
    delay(1000);        // Error handling for example: wait a second, then try again

    // Soft reset the sensor
    flow_sensor::soft_reset();

    // Read the scale factor and the adjacent unit
    flow_sensor::read_scale_factor();

    // Set sensor resolution
    flow_sensor::set_resolution(resolution);

    // Switch to measurement mode
    flow_sensor::meassurement_mode();
  } while (_ret != 0);
}

void flow_sensor::measure(){

  /*
    Measures flow using CRC for data integrity
  */
  Wire.requestFrom(_ADDRESS, 3); // reading 2 measurement bytes + the CRC byte
  if (Wire.available() < 3) {
    Serial.println("Error while reading flow measurement");

  } else {
    _data[0] = Wire.read(); // read the MSB from the sensor
    _data[1] = Wire.read(); // read the LSB from the sensor
    _crc = Wire.read();
    // compute 8bit checksum (takes 16 micro seconds on the arduino uno)
    _calc_crc = 0;
    for (int i = 0; i < 2; ++i) {
      _calc_crc ^= (_data[i]);
      for (int j = 8; j > 0; --j) {
        if (_calc_crc & 0x80) {
          _calc_crc = (_calc_crc << 1) ^ _CRC_POLYNOMIAL;
        } else {
          _calc_crc = (_calc_crc << 1);
        }
      }
    }

    if (_calc_crc != _crc) {
      Serial.println("Reading Error.\nCRC mismatch while reading flow measurement");
    } else {
      _raw_sensor_value  = _data[0] << 8; // read the MSB from the sensor
      _raw_sensor_value |= _data[1];      // read the LSB from the sensor
      _flow = ((int16_t) _raw_sensor_value) / ((float) _scale_factor);
    }
  }
}

void flow_sensor::avg_measure(int num_val_to_avg){
  for (int i = 0; i < num_val_to_avg; i++) {
    flow_sensor::measure();           // measure flow once and save value in _flow
    _avg_flow += _flow;               // Read sensor data and add it to value
    delay(3);                         // give time for sensor to reset
  }
    _avg_flow /= num_val_to_avg;          // average readings  }
}

void flow_sensor::volume_measurement(int num_val_to_avg){
  //start clock to track time
  unsigned long tic = millis();

  // get average volume from num_val_to_avg measurements
  flow_sensor::avg_measure(num_val_to_avg);

  // stop clock
  unsigned long toc = (millis() - tic);
  
  // calculate volume
  _recorded_volume = _avg_flow * (((float)toc)/60.00); // unit: µL/min * sec

}

void flow_sensor::print_flow(int interval){
  flow_sensor::measure();
  Serial.println((String) _flow + " µL/min");
  
  delay(interval);
}