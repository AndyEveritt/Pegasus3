#include <Wire.h>
#include "I2C_Anything.h"

// define the PWM output pins for each motor and add to array
#define FL_PIN 2
#define ML_PIN 3
#define RL_PIN 5
#define FR_PIN 6
#define MR_PIN 7
#define RR_PIN 8
int arrPWMPins[6] = {FL_PIN, ML_PIN, RL_PIN, FR_PIN, MR_PIN, RR_PIN};

// define the address number for the slave
const byte MY_ADDRESS = 42;

// define variables to store the incoming data
volatile boolean haveData = false;
volatile double dblLinearVelocity = 1.0;
volatile double dblAngularVelocity = 1.0;

// define an array to hold the up-to-date PWM values
float arrPWMVals[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
byte arrBytePWMVals[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  int myEraser = 7;
  TCCR3B &= ~myEraser;
  TCCR4B &= ~myEraser;
  int myPrescaler = 1;
  TCCR3B |= myPrescaler;
  TCCR4B |= myPrescaler;

  // enable direction pins (FORWARDS CONFIGURATION)
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);
  pinMode(32, OUTPUT);
  digitalWrite(32, LOW);

  // listen for incoming data from Arduino 101 via I2C
  Wire.begin (MY_ADDRESS);
  Serial.begin (9600);
  Wire.onReceive (receiveEvent);
}

void loop() {
  update_motors();
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
  if (howMany >= (sizeof dblLinearVelocity) + (sizeof dblAngularVelocity)) {
    I2C_readAnything (dblLinearVelocity);
    I2C_readAnything (dblAngularVelocity);
    for (int intMotorNum = 0; intMotorNum < 6; intMotorNum++){
      if (intMotorNum <= 2){
        arrPWMVals[intMotorNum] = (dblLinearVelocity - (dblAngularVelocity - 1));
      } else {
        arrPWMVals[intMotorNum] = (dblLinearVelocity + (dblAngularVelocity - 1));
      }
    }
    haveData = true;
  }  // end if have enough data
}

// update the motors with the most current PWM values
void update_motors(){
  // check if left motors should be turning backwards
  if (arrPWMVals[0] < 1.0 || arrPWMVals[1] < 1.0 || arrPWMVals[2] < 1.0) { // They should be spinning backwards
    digitalWrite(31, LOW);
  } else if (arrPWMVals[0] > 1.0 || arrPWMVals[1] > 1.0 || arrPWMVals[2] > 1.0) { // They shouldn't be spinning backwards
    digitalWrite(31, HIGH);
  } // Otherwise the duty cycle is exactly 1.0 and the direction does not need to change

  // check if right motors should be turning backwards
  if (arrPWMVals[3] < 1.0 || arrPWMVals[4] < 1.0 || arrPWMVals[5] < 1.0) { // They should be spinning backwards
    digitalWrite(32, HIGH);
  } else if (arrPWMVals[3] > 1.0 || arrPWMVals[4] > 1.0 || arrPWMVals[5] > 1.0) { // They shouldn't be spinning backwards
    digitalWrite(32, LOW);
  } // Otherwise the duty cycle is exactly 1.0 and the direction does not need to change


  // convert 0 -> 1 duty cycle into 0 -> 255 duty cycle for analogWrite
  int i;
  for (i = 0; i < 6; i++) {
    arrBytePWMVals[i] = byte((abs(arrPWMVals[i] - 1.0)) * 510);
  }

  // set PWM duty cycle for each wheel
  for (i = 0; i < 6; i++){
    analogWrite(arrPWMPins[i], arrBytePWMVals[i]);
  }
}
