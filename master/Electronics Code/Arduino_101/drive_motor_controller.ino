#include <Wire.h>
#include "I2C_Anything.h"

// I2C 101 -> Mega2560 constants
const byte SLAVE_ADDRESS = 42;

byte check_byte = 254;

byte bytLinearVelocity;
byte bytAngularVelocity;
float fltLinearVelocity;
float fltAngularVelocity;

void setup() {
    // Begin I2C connection to Mega2560
    Wire.begin();

    // Begin serial connection to Braswell
    Serial.begin(9600);
}

// Loop code

void loop() {
    if(Serial.available() == 3){
      if (Serial.read() == check_byte){
        bytLinearVelocity = Serial.read();
        bytAngularVelocity = Serial.read();
      }
      fltLinearVelocity = bytLinearVelocity / 100.;
      fltAngularVelocity = bytAngularVelocity / 100.;
      Wire.beginTransmission(SLAVE_ADDRESS);
      I2C_writeAnything (fltLinearVelocity);
      I2C_writeAnything (fltAngularVelocity);
      Wire.endTransmission();
      delay(200);
      Serial.end();
      Serial.begin(9600);
    }
}nÑ
