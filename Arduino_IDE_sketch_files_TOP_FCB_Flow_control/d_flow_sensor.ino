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


// -----------------------------------------------------------------------------
// Constants for CRC, I2C Address and verbose output
// -----------------------------------------------------------------------------

#define CRC_POLYNOMIAL 0x131  // P(x)=x^8+x^5+x^4+1 (binary 0001 0011 0001)

const int ADDRESS = 0x40; // Standard address for Liquid Flow Sensors

const bool VERBOSE_OUTPUT = true; // set to false for less verbose output

// -----------------------------------------------------------------------------
// for voltage and temperature meassurements
// -----------------------------------------------------------------------------

const float SCALEFACTOR_FLOW = 1.0f;
const float SCALEFACTOR_TEMP = 10.0f;
const float SCALEFACTOR_VDD  = 1000.0f;

const byte TRIGGER_FLOW_COMMAND = 0xF1;
const byte TRIGGER_TEMP_COMMAND = 0xF3;
const byte TRIGGER_VDD_COMMAND  = 0xF5;
const byte TRIGGER[] = {TRIGGER_FLOW_COMMAND,
                        TRIGGER_TEMP_COMMAND,
                        TRIGGER_VDD_COMMAND
                       };
const float MEASUREMENT_FACTOR[] = {SCALEFACTOR_FLOW,
                                    SCALEFACTOR_TEMP,
                                    SCALEFACTOR_VDD
                                   };
const char *MEASUREMENT_TYPE[] = {"Flow (signed value)", "Temp", "VDD"};
const char *MEASUREMENT_UNIT[] = {"", " C", " Volt"};


void temp_volt_meassurement() {
  int i;
  int ret;
  uint16_t reg;
  uint16_t raw_sensor_value;
  float sensor_reading;

  // Loop through the measurement of flow, temperature and VDD
  for (i = 0; i <= 2; ++i) {

    Wire.beginTransmission(ADDRESS);
    Wire.write(TRIGGER[i]);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode");
      continue;
    }

    Wire.requestFrom(ADDRESS, 2);
    if (Wire.available() < 2) {
      Serial.println("Error during read");
      continue;
    }
    raw_sensor_value  = Wire.read() << 8;
    raw_sensor_value |= Wire.read();

    sensor_reading = ((int16_t) raw_sensor_value) / MEASUREMENT_FACTOR[i];

    Serial.print(MEASUREMENT_TYPE[i]);
    Serial.print(" Measurement: ");
    Serial.print(sensor_reading);
    Serial.println(MEASUREMENT_UNIT[i]);

    delay(500);
  }
}

// -----------------------------------------------------------------------------
//  Change sensor resolution
// -----------------------------------------------------------------------------

void change_resolution() {
  static unsigned int sensor_resolution = 13;

  int ret;
  uint16_t raw_sensor_value;
  uint16_t adv_user_reg_original;
  uint16_t adv_user_reg_new;
  uint16_t resolution_mask;
  resolution_mask = 0xF1FF | ((sensor_resolution - 9) << 9);

  // Change mode to read adv. user register
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xE5);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error during write register read mode");

  } else {
    // Read the content of the adv user register
    Wire.requestFrom(ADDRESS, 2);
    if (Wire.available() < 2) {
      Serial.println("Error during read register settings");

    } else {
      adv_user_reg_original  = Wire.read() << 8;
      adv_user_reg_original |= Wire.read();
      adv_user_reg_new = (adv_user_reg_original | 0x0E00) & resolution_mask;

      if (VERBOSE_OUTPUT) {
        // Display resolution
        Serial.print("Resolution setting:       ");
        Serial.println(sensor_resolution);
      }

      // Apply resolution changes:
      // Change mode to write to adv. user register
      Wire.beginTransmission(ADDRESS);
      Wire.write(0xE4);                           // Send command
      Wire.write((byte)(adv_user_reg_new >> 8));      // Send MSB
      Wire.write((byte)(adv_user_reg_new & 0xFF));    // Send LSB
      ret = Wire.endTransmission();
      if (ret != 0) {
        Serial.println("Error during write register settings");
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Scale factor function
// -----------------------------------------------------------------------------

// EEPROM Addresses for factor and unit of calibration fields 0,1,2,3,4.
const uint16_t SCALE_FACTOR_ADDRESSES[] = {0x2B6, 0x5B6, 0x8B6, 0xBB6, 0xEB6};
const uint16_t UNIT_ADDRESSES[] =         {0x2B7, 0x5B7, 0x8B7, 0xBB7, 0xEB6};

// Flow Units and their respective codes.
const char    *FLOW_UNIT[] = {"nl/min", "ul/min", "ml/min", "ul/sec", "ml/h"};
const uint16_t FLOW_UNIT_CODES[] = {2115, 2116, 2117, 2100, 2133};

uint16_t scale_factor;
const char *unit;

void read_scale_factor() {
  int ret;

  uint16_t user_reg;
  uint16_t scale_factor_address;

  uint16_t unit_code;

  byte crc1;
  byte crc2;

  do {
    delay(500); // Error handling for example: wait a second, then try again

    // Read the user register to get the active configuration field
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xE3);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while setting register read mode");
      continue;
    }

    Wire.requestFrom(ADDRESS, 2);
    if (Wire.available() < 2) {
      Serial.println("Error while reading register settings");
      continue;
    }
    user_reg  = Wire.read() << 8;
    user_reg |= Wire.read();

    // The active configuration field is determined by bit <6:4>
    // of the User Register
    scale_factor_address = SCALE_FACTOR_ADDRESSES[(user_reg & 0x0070) >> 4];

    // Read scale factor and measurement unit
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xFA); // Set EEPROM read mode
    // Write left aligned 12 bit EEPROM address
    Wire.write(scale_factor_address >> 4);
    Wire.write((scale_factor_address << 12) >> 8);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write EEPROM address");
      continue;
    }

    // Read the scale factor and the adjacent unit
    Wire.requestFrom(ADDRESS, 6);
    if (Wire.available() < 6) {
      Serial.println("Error while reading EEPROM");
      continue;
    }
    scale_factor = Wire.read() << 8;
    scale_factor |= Wire.read();
    crc1         = Wire.read();
    unit_code    = Wire.read() << 8;
    unit_code   |= Wire.read();
    crc2         = Wire.read();

    switch (unit_code) {
      case 2115:
        {
          unit = FLOW_UNIT[0];
        }
        break;
      case 2116:
        {
          unit = FLOW_UNIT[1];
        }
        break;
      case 2117:
        {
          unit = FLOW_UNIT[2];
        }
        break;
      case 2100:
        {
          unit = FLOW_UNIT[3];
        }
        break;
      case 2133:
        {
          unit = FLOW_UNIT[4];
        }
        break;
      default:
        Serial.println("Error: No matching unit code");
        break;
    }

    if (VERBOSE_OUTPUT) {
      Serial.println();
      Serial.println("-----------------------");
      Serial.print("Scale factor: ");
      Serial.println(scale_factor);
      Serial.print("Unit: ");
      Serial.print(unit);
      Serial.print(", code: ");
      Serial.println(unit_code);
      Serial.println("-----------------------");
      Serial.println();
    }
  } while (ret != 0);
}

// -----------------------------------------------------------------------------
//  Flow sensor Setup Function
// -----------------------------------------------------------------------------

void flow_sensor_setup() {

  int ret;

  do {
    delay(500); // Error handling for example: wait and then try again

    // Soft reset the sensor
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xFE);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      continue;
    }
    delay(50);                  // wait long enough for reset

    change_resolution();        // change flow sensor resolution
    temp_volt_meassurement();   //check temp and voltage 

    // Switch to measurement mode
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xF1);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    }
  } while (ret != 0);
}

// -----------------------------------------------------------------------------
// Flow Meassurement function
// -----------------------------------------------------------------------------

float flow_measurement() {
  int ret;
  int b, i;
  byte crc;
  byte calc_crc;
  byte data[2];
  uint16_t raw_sensor_value;
  float sensor_reading;

  Wire.requestFrom(ADDRESS, 3); // reading 2 measurement bytes + the CRC byte
  if (Wire.available() < 3) {
    Serial.println("Error while reading flow measurement");

  } else {
    data[0] = Wire.read(); // read the MSB from the sensor
    data[1] = Wire.read(); // read the LSB from the sensor
    crc = Wire.read();
    // compute 8bit checksum (takes 16 micro seconds on the arduino uno)
    calc_crc = 0;
    for (b = 0; b < 2; ++b) {
      calc_crc ^= (data[b]);
      for (i = 8; i > 0; --i) {
        if (calc_crc & 0x80) {
          calc_crc = (calc_crc << 1) ^ CRC_POLYNOMIAL;
        } else {
          calc_crc = (calc_crc << 1);
        }
      }
    }

    if (calc_crc != crc) {
      Serial.println("Reading Error.\nCRC mismatch while reading flow measurement");
    } else {
      raw_sensor_value  = data[0] << 8; // read the MSB from the sensor
      raw_sensor_value |= data[1];      // read the LSB from the sensor
      sensor_reading = ((int16_t) raw_sensor_value) / ((float) scale_factor);
    }
  }
  return sensor_reading;
}

// -----------------------------------------------------------------------------
//  Averaged flow meassurement
// -----------------------------------------------------------------------------

float avg_flow_measurement(int numReadings, float *flow) {

  float value{};
  for (int i = 0; i < numReadings; i++) {
    
    delay(3);                    // give time for sensor to reset
    value += flow_measurement(); // Read sensor data and add it to value
  }

  value /= numReadings;          // average readings

  *flow = value;                 //save average flow
}

// -----------------------------------------------------------------------------
//  Averaged flow meassurement Integration function
// -----------------------------------------------------------------------------
float avg_flow_measurement_integration(int numReadings, float *flow) {

  unsigned long tic = millis();
  float sum_meassurements{}, avg_value{};

  for (int i = 0; i < numReadings; i++) {
    // Read sensor data.
    sum_meassurements += flow_measurement();
  }
  
  //average meassurements
  avg_value = sum_meassurements / numReadings;

  *flow = avg_value;
  unsigned long toc = (millis() - tic);
  float volume = 0.00;
  
  volume = avg_value * (((float)toc)/60.00); // unit: µL/min * sec
  //volume = volume/60.00;// unit is nl/ms
  //Serial.println("vol avg " + (String)volume+ " time " + (float)toc);
  
  return volume;

}
