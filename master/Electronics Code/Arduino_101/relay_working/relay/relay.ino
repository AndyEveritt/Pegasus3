#include <Wire.h>
#include "I2C_Anything.h"

// I2C 101 -> Mega2560 constants
const byte SLAVE_ADDRESS = 42;

/* Byte values to distinguish whether following bytes are drive system or
arm commands, bytTypeByte distinguishes between the two in void loop() */
byte bytDriveByte = 253;
byte bytArmByte = 254;
byte bytTypeByte;

/*
driveBytes[0]: Linear velocity byte
driveBytes[1]: Angular velocity byte
*/
byte arrDriveBytes[2] = {128, 128};

/*
armBytes[0]: Base rotation byte
armBytes[1]: Actuator 1 byte
armBytes[2]: Actuator 2 byte
armBytes[3]: Wrist rotation byte
armBytes[4]: Wrist actuator byte
armBytes[5]: Gripper byte
*/
byte arrArmBytes[6] = {128, 128, 128, 128, 128, 128};

void setup() {
    /* Begin I2C connection to Mega2560 */
    Wire.begin();

    /* Begin serial connection to Braswell */
    Serial.begin(9600);
}
byte bytPassValue;
byte ctrlByte;

void loop() {
  if(Serial.available() > 6)
  { 
    ctrlByte = Serial.read();

    if(ctrlByte == 253)
    {
      Wire.beginTransmission(SLAVE_ADDRESS);
      I2C_writeAnything((byte) 253);
      for(int i = 0; i < 2; i++)
      {
        bytPassValue = Serial.read();
        I2C_writeAnything((byte) bytPassValue);
      }
      Wire.endTransmission();
      delay(20);
      Serial.end();
      Serial.begin(9600);
    } 
    else if (ctrlByte == 254)
    {
      Wire.beginTransmission(SLAVE_ADDRESS);
      I2C_writeAnything((byte) 254);
      for(int i = 0; i < 6; i++)
      {
        bytPassValue = Serial.read();
        I2C_writeAnything((byte) bytPassValue);
      }
      Wire.endTransmission();
      delay(20);
      Serial.end();
      Serial.begin(9600);
    }
  }
/*
  bytTypeByte = Serial.read();
  
  if(bytTypeByte == 253 &&  Serial.available() >= 3)
  {
    for(int i = 0; i < 6; i++)
    {
      arrDriveBytes[i] = Serial.read();
    }

    Wire.beginTransmission(SLAVE_ADDRESS);
    for(int i = 0; i < 6; i++)
    {
      I2C_writeAnything ((byte)arrDriveBytes[i]);
    }
    Wire.endTransmission();

    delay(10);
    Serial.end();
    Serial.begin(9600);
  }

  if(bytTypeByte == 254 &&  Serial.available() >= 7)
  {
    for(int i = 0; i < 6; i++)
    {
      arrArmBytes[i] = Serial.read();
    }

    Wire.beginTransmission(SLAVE_ADDRESS);
    for(int i = 0; i < 6; i++)
    {
      I2C_writeAnything ((byte)arrArmBytes[i]);
    }
    Wire.endTransmission();

    delay(10);
    Serial.end();
    Serial.begin(9600);
  }
  */
}
