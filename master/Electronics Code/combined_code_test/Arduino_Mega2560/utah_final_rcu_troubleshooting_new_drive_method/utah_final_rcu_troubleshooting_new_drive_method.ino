#include <Wire.h>
#include "I2C_Anything.h"

/* Define the address of the slave (this) device */
const byte MY_ADDRESS = 42;

/*
Pins connected to arm base: PWM: [0]. Direction pin: [0]
Pins connected to arm actuator: 1 PWM: [1]. Direction pin: [1]
Pins connected to arm actuator: 2 PWM: [2]. Direction pin: [2]
Pins connected to arm wrist rotation: PWM: [3]. Direction pin: [3]
Pins connected to arm wrist actuator: PWM: [4]. Direction pin: [4]
Pins connected to arm gripper: PWM: [5]. Direction pin: [5]
*/
int arrArmPWMPins[6] = {8, 9, 10, 11, 12, 13};
int arrArmDirPins[6] = {28, 29, 30, 31, 32, 33};

/*
Pins connected to drive front left (FL) motor: PWM: [0]. Direction pin: [0]
Pins connected to drive middle left (ML) motor: 1 PWM: [1]. Direction pin: [1]
Pins connected to drive rear left (RL) motor: 2 PWM: [2]. Direction pin: [2]
Pins connected to drive front right (FR) motor: PWM: [3]. Direction pin: [3]
Pins connected to drive middle right (MR) motor: PWM: [4]. Direction pin: [4]
Pins connected to drive rear right (RR) motor: PWM: [5]. Direction pin: [5]
*/
int arrDrivePWMPins[6] = {2, 4, 6, 3, 5, 7};
int arrDriveDirPins[6] = {22, 24, 26, 23, 25, 27};

/* bytReceivedByte stores the incoming data shortly before it is written */
volatile byte bytReceivedByte;

/* bytTypeByte is used to store the value of an incoming I2C byte to check
which type of control the following bytes refer to */
volatile byte bytTypeByte;

/* Arrays for the 8 bit PWM values for the motor, arm motors and actuators */
byte arrDriveMessage[2] = {100, 100};
byte arrDrivePWM[6] = {100, 100, 100, 100, 100, 100};
byte arrArmPWM[6] = {100, 100, 100, 100, 100, 100};

void setup() {
  int myEraser = 7;
  TCCR0B &= ~myEraser;
  TCCR1B &= ~myEraser;
  TCCR2B &= ~myEraser;
  TCCR3B &= ~myEraser;
  TCCR4B &= ~myEraser;
  int myPrescaler = 1;
  TCCR0B |= myPrescaler;
  TCCR1B |= myPrescaler;
  TCCR2B |= myPrescaler;
  TCCR3B |= myPrescaler;
  TCCR4B |= myPrescaler;

  /* Enable and initialise drive motor direction pins */
  pinMode(arrDriveDirPins[0], OUTPUT);
  digitalWrite(arrDriveDirPins[0], HIGH);
  pinMode(arrDriveDirPins[1], OUTPUT);
  digitalWrite(arrDriveDirPins[1], HIGH);
  pinMode(arrDriveDirPins[2], OUTPUT);
  digitalWrite(arrDriveDirPins[2], HIGH);
  pinMode(arrDriveDirPins[3], OUTPUT);
  digitalWrite(arrDriveDirPins[3], LOW);
  pinMode(arrDriveDirPins[4], OUTPUT);
  digitalWrite(arrDriveDirPins[4], LOW);
  pinMode(arrDriveDirPins[5], OUTPUT);
  digitalWrite(arrDriveDirPins[5], LOW);

  /* enable and initialise arm motor / actuator direction pins */
  for(int i = 0; i < 6; i++)
  {
    pinMode(arrArmDirPins[i], OUTPUT);
    digitalWrite(arrArmDirPins[i], LOW);
  }

  // listen for incoming data from Arduino 101 via I2C
  Wire.begin (MY_ADDRESS);
  Serial.begin (9600);
  Wire.onReceive (receiveEvent);
}

void loop() {
  update_arm();
  update_drive();
  
  Serial.print(arrDriveMessage[0]);
  Serial.print(' ');
  Serial.print(arrDriveMessage[1]);
  Serial.println();
  
  /*
  for(int i = 0; i < 6; i++)
  {
    Serial.print(arrDrivePWM[i]);
    Serial.print(' ');
  }
  */
  /*
  for(int i = 0; i < 6; i++)
  {
    Serial.print(arrArmPWM[i]);
    Serial.print(' ');
  }
  Serial.println();
  */
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  I2C_readAnything (bytTypeByte);

  if(bytTypeByte == 253 && howMany == 3)
  {
    for(int i = 0; i < 2; i++)
    {
      I2C_readAnything(bytReceivedByte);
      arrDriveMessage[i] = bytReceivedByte;
    }
  }

  if(bytTypeByte == 254 && howMany == 7)
  {
    for(int i = 0; i < 6; i++)
    {
      I2C_readAnything(bytReceivedByte);
      arrArmPWM[i] = bytReceivedByte;
    }
  }
}

// update the motors with the most current PWM values
void update_arm(){
  // set PWM duty cycle for each wheel
  for (int i = 0; i < 6; i++){
    if(arrArmPWM[i] < 100)
    {
      digitalWrite(arrArmDirPins[i], HIGH);
    }
    else if(arrArmPWM[i] > 100)
    {
      digitalWrite(arrArmDirPins[i], LOW);
    }
    analogWrite(arrArmPWMPins[i], (int) (1.5 * abs(arrArmPWM[i] - 100)));
  }
}

void update_drive(){
  for(int i = 0; i < 6; i++)
  {
    if(i < 3)
    {
      arrDrivePWM[i] = (arrDriveMessage[0] - (int) (0.5 * (arrDriveMessage[1] - 100)));
    }
    else
    {
      arrDrivePWM[i] = (arrDriveMessage[0] + (int) (0.5 * (arrDriveMessage[1] - 100)));
    }
  }

  
  // check if left motors should be turning backwards
  if (arrDriveMessage[0] < 100 || arrDriveMessage[1] > 105)
  { // They should be spinning backwards
    digitalWrite(arrDriveDirPins[0], LOW);
    digitalWrite(arrDriveDirPins[1], LOW);
    digitalWrite(arrDriveDirPins[2], LOW);
  }
  else if (arrDriveMessage[0] >= 100 || arrDriveMessage[1] < 95)
  { // They shouldn't be spinning backwards
    digitalWrite(arrDriveDirPins[0], HIGH);
    digitalWrite(arrDriveDirPins[1], HIGH);
    digitalWrite(arrDriveDirPins[2], HIGH);
  } // Otherwise the duty cycle is exactly 1.0 and the direction does not need to change

  // check if right motors should be turning backwards
  if (arrDriveMessage[0] < 100 || arrDriveMessage[1] < 95)
  { // They should be spinning backwards
    digitalWrite(arrDriveDirPins[3], HIGH);
    digitalWrite(arrDriveDirPins[4], HIGH);
    digitalWrite(arrDriveDirPins[5], HIGH);
  }
  else if (arrDriveMessage[0] >= 100 || arrDriveMessage[1] > 105)
  { // They shouldn't be spinning backwards
    digitalWrite(arrDriveDirPins[3], LOW);
    digitalWrite(arrDriveDirPins[4], LOW);
    digitalWrite(arrDriveDirPins[5], LOW);
  } // Otherwise the duty cycle is exactly 100 and the direction does not need to change
  
  // set PWM duty cycle for each wheel
  for (int i = 0; i < 6; i++){
    analogWrite(arrDrivePWMPins[i], Overflow(2 * abs(arrDrivePWM[i] - 100)));
    //Serial.print(Overflow(abs(arrDrivePWM[i] - 100)));
    //Serial.print(' ');
  }
  //Serial.println();
}

/* Function to stop an overflow in the 8-bit PWM value occurring */
int Overflow(int pwmInput)
{
  if(pwmInput > 250)
  {
    return 250;
  }
  else if(pwmInput < 0)
  {
    return 0;
  }
  else
  {
    return pwmInput;
  }
}
