#include <Wire.h>
#include "I2C_Anything.h"

// define the PWM output pins for each motor and add to array
#define BASE_PIN 2
#define ACTUATOR_1_PIN 3
#define ACTUATOR_2_PIN 5
#define WRIST_ROTATION_PIN 6
#define WRIST_ACTUATOR_PIN 7
#define GRIPPER_PIN 8

#define BASE_DIR_PIN 24
#define ACTUATOR_1_DIR_PIN 26
#define ACTUATOR_2_DIR_PIN 28
#define WRIST_ROTATION_DIR_PIN 30
#define WRIST_ACTUATOR_DIR_PIN 32
#define GRIPPER_DIR_PIN 34

int arrPWMPins[6] = {BASE_PIN, ACTUATOR_1_PIN, ACTUATOR_2_PIN, WRIST_ROTATION_PIN, WRIST_ACTUATOR_PIN, GRIPPER_PIN};
int arrDirPins[6] = {BASE_DIR_PIN, ACTUATOR_1_DIR_PIN, ACTUATOR_2_DIR_PIN, WRIST_ROTATION_DIR_PIN, WRIST_ACTUATOR_DIR_PIN, GRIPPER_DIR_PIN};

// define the address number for the slave
const byte MY_ADDRESS = 42;

// define variables to store the incoming data
volatile boolean haveData = false;
volatile byte bytBase;
volatile byte bytActuator1;
volatile byte bytActuator2;
volatile byte bytWristRotation;
volatile byte bytWristActuator;
volatile byte bytGripper;


// define an array to hold the up-to-date PWM values

byte arrBytePWMVals[6] = {128, 128, 128, 128, 128, 128};

void setup() {
  int myEraser = 7;
  TCCR3B &= ~myEraser;
  TCCR4B &= ~myEraser;
  int myPrescaler = 1;
  TCCR3B |= myPrescaler;
  TCCR4B |= myPrescaler;

  // enable arm direction pins
  for(int i = 0; i < 6; i++)
  {
    pinMode(arrDirPins[i], OUTPUT);
    digitalWrite(arrDirPins[i], LOW);
  }

  // listen for incoming data from Arduino 101 via I2C
  Wire.begin (MY_ADDRESS);
  Serial.begin (9600);
  Wire.onReceive (receiveEvent);
}

void loop() {
  update_arm();
  int i;
  for(i = 0; i < 6; i++) {
    Serial.print(arrBytePWMVals[i]);
    Serial.print(' ');
  }
  Serial.println();
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
    if (howMany >= (sizeof(bytBase) * 6)) {
      I2C_readAnything (bytBase);
      arrBytePWMVals[0] = bytBase;
      I2C_readAnything(bytActuator1);
      arrBytePWMVals[1] = bytActuator1;
      I2C_readAnything (bytActuator2);
      arrBytePWMVals[2] = bytActuator2;
      I2C_readAnything(bytWristRotation);
      arrBytePWMVals[3] = bytWristRotation;
      I2C_readAnything (bytBase);
      arrBytePWMVals[4] = bytBase;
      I2C_readAnything(bytGripper);
      arrBytePWMVals[5] = bytGripper;
    }
}

// update the motors with the most current PWM values
void update_arm(){
  // set PWM duty cycle for each wheel
  for (int i = 0; i < 6; i++){
    if(arrBytePWMVals[i] < 128)
    {
      digitalWrite(arrDirPins[i], HIGH);
    }
    else if(arrBytePWMVals[i] > 128)
    {
      digitalWrite(arrDirPins[i], LOW);
    }
    analogWrite(arrPWMPins[i], 5 * abs(arrBytePWMVals[i] - 128));
  }
}
