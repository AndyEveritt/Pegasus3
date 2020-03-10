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
int arrArmPWMPins[6] = {3, 5, 6, 9, 10, 11};
int arrArmDirPins[6] = {24, 26, 28, 30, 32, 34};

/*
Pins connected to drive front left (FL) motor: PWM: [0]. Direction pin: [0]
Pins connected to drive middle left (ML) motor: 1 PWM: [1]. Direction pin: [1]
Pins connected to drive rear left (RL) motor: 2 PWM: [2]. Direction pin: [2]
Pins connected to drive front right (FR) motor: PWM: [3]. Direction pin: [3]
Pins connected to drive middle right (MR) motor: PWM: [4]. Direction pin: [4]
Pins connected to drive rear right (RR) motor: PWM: [5]. Direction pin: [5]
*/
int arrDrivePWMPins[6] = {3, 5, 6, 9, 10, 11};
int arrDriveDirPins[6] = {12, 13};

/* bytReceivedByte stores the incoming data shortly before it is written */
volatile byte bytReceivedByte;

/* bytTypeByte is used to store the value of an incoming I2C byte to check
which type of control the following bytes refer to */
volatile byte bytTypeByte;

/* Arrays for the 8 bit PWM values for the motor, arm motors and actuators */
byte arrDriveCurrent[2] = {100, 100};
byte arrDriveTarget[2] = {100, 100};
byte arrDriveControl[2] = {100, 100};
float drive_I = 0;
float prev_error = 0;

byte arrDrivePWM[6] = {100, 100, 100, 100, 100, 100};
byte arrArmPWM[6] = {128, 128, 128, 128, 128, 128};

void setup() 
{
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
  digitalWrite(arrDriveDirPins[1], LOW);

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
  /*update_drive();*/
  /*
  Serial.print(arrDriveMessage[0]);
  Serial.print(' ');
  Serial.print(arrDriveMessage[1]);
  Serial.println();
  */
  for(int i = 0; i < 6; i++)
  {
    Serial.print(arrDrivePWM[i]);
    Serial.print(' ');
  }
  for(int i = 0; i < 6; i++)
  {
    Serial.print(arrArmPWM[i]);
    Serial.print(' ');
  }
  Serial.println();
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  I2C_readAnything (bytTypeByte);
  if(bytTypeByte == 253 && howMany >= (sizeof(byte)) * 2)
  {
    for(int i = 0; i < 2; i++)
    {
      I2C_readAnything(bytReceivedByte);
      arrDriveTarget[i] = bytReceivedByte;
      drivePID(i);
    }
  }

  if(bytTypeByte == 254)
  {
    for(int i = 0; i < 6; i++)
    {
      I2C_readAnything(bytReceivedByte);
      arrArmPWM[i] = bytReceivedByte;
    }
  }
}

void PID(int i)
{
  float P, dI, D;
  float Kp, Ki, Kd;
  float error;

  Kp = 1;
  Ki = 0;
  Kd = 0;

  error = arrDriveTarget[i] - arrDriveCurrent[i];

  P = Kp * error;
  dI = Ki * error * 0.01;
  D = (Kd/0.01) * (error - prev_error);

  arrDriveControl[i] = P + (drive_I + dI) + D;

  prev_error = error;

  if (arrDriveControl[i] > 250)
    arrDriveControl[i] = 250;
  else if (arrDriveControl[i] < 0)
    arrDriveControl[i] = 0;
  else
    drive_I += dI;
}

// update the motors with the most current PWM values
void update_arm(){
  // set PWM duty cycle for each wheel
  for (int i = 0; i < 6; i++){
    if(arrArmPWM[i] < 128)
    {
      digitalWrite(arrArmDirPins[i], HIGH);
    }
    else if(arrArmPWM[i] > 128)
    {
      digitalWrite(arrArmDirPins[i], LOW);
    }
    analogWrite(arrArmPWMPins[i], 4 * abs(arrArmPWM[i] - 128));
  }
  delay(10);
}

void update_drive(){
  for(int i = 0; i < 6; i++)
  {
    if(i < 3)
    {
      arrDrivePWM[i] = (arrDriveControl[0] - (arrDriveControl[1] - 100));
    }
    else
    {
      arrDrivePWM[i] = (arrDriveControl[0] + (arrDriveControl[1] - 100));
    }
  }
  
  // check if left motors should be turning backwards
  if (arrDrivePWM[0] <= 100 || arrDrivePWM[1] <= 100 || arrDrivePWM[2] <= 100)
  { // They should be spinning backwards
    digitalWrite(arrDriveDirPins[0], LOW);
  }
  else if (arrDrivePWM[0] > 100 || arrDrivePWM[1] > 100 || arrDrivePWM[2] > 100)
  { // They shouldn't be spinning backwards
    digitalWrite(arrDriveDirPins[0], HIGH);
  } // Otherwise the duty cycle is exactly 1.0 and the direction does not need to change

  // check if right motors should be turning backwards
  if (arrDrivePWM[3] <= 100 || arrDrivePWM[4] <= 100 || arrDrivePWM[5] <= 100)
  { // They should be spinning backwards
    digitalWrite(arrDriveDirPins[1], HIGH);
  }
  else if (arrDrivePWM[3] > 100 || arrDrivePWM[4] > 100 || arrDrivePWM[5] > 100)
  { // They shouldn't be spinning backwards
    digitalWrite(arrDriveDirPins[1], LOW);
  } // Otherwise the duty cycle is exactly 1.0 and the direction does not need to change

  // set PWM duty cycle for each wheel
  for (int i = 0; i < 6; i++){
    analogWrite(arrDrivePWMPins[i], 3 * abs(arrDriveControl[i]-100));
  }
  delay(10);
}
