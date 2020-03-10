#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

std_msgs::Int16MultiArray drive_pwm_msg; // message to debug PWM signals in ros
std_msgs::Int16MultiArray drive_dir_msg; // message to debug PWM signals in ros
ros::Publisher pubDrivePWM("arduino/drive_pwm", &drive_pwm_msg);
ros::Publisher pubDriveDir("arduino/drive_dir", &drive_dir_msg);

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

/* Arrays for the 8 bit PWM values for the motor, arm motors and actuators */
int arrDriveMessage[2] = {100, 100};
int arrDrivePWM[6] = {100, 100, 100, 100, 100, 100};
int arrDrivePWMDebug[6] = {100, 100, 100, 100, 100, 100};
int arrDriveDir[6] = {1, 1, 1, 0, 0, 0};
int arrArmPWM[6] = {100, 100, 100, 100, 100, 100};

// update the motors with the most current PWM values
void update_arm(const std_msgs::Int16MultiArray &msg)
{
  // set PWM duty cycle for each wheel
  for (int i = 0; i < 6; i++)
  {
    if (msg.data[i] < 100)
    {
      digitalWrite(arrArmDirPins[i], HIGH);
    }
    else if (msg.data[i] > 100)
    {
      digitalWrite(arrArmDirPins[i], LOW);
    }
    if (i != 3)
    {
      analogWrite(arrArmPWMPins[i], (int)(3 * abs(msg.data[i] - 100)));
    }
    else
    {
      analogWrite(arrArmPWMPins[i], (int)(0.5 * abs(msg.data[i] - 100)));
    }
  }
}

void update_drive(const std_msgs::Int16MultiArray &msg)
{
  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
    {
      arrDrivePWM[i] = (msg.data[0] - (msg.data[1] - 100));
    }
    else
    {
      arrDrivePWM[i] = (msg.data[0] + (msg.data[1] - 100));
    }
  }

  // forwards
  if (msg.data[0] > 105)
  {
    for (int i = 0; i < 6; i++)
    {
      if (i < 3)
      {
        digitalWrite(arrDriveDirPins[i], HIGH);
        arrDriveDir[i] = 1;
      }
      else
      {
        digitalWrite(arrDriveDirPins[i], LOW);
        arrDriveDir[i] = 0;
      }
    }
  }

  //backwards
  if (msg.data[0] < 95)
  {
    for (int i = 0; i < 6; i++)
    {
      if (i < 3)
      {
        digitalWrite(arrDriveDirPins[i], LOW);
        arrDriveDir[i] = 0;
      }
      else
      {
        digitalWrite(arrDriveDirPins[i], HIGH);
        arrDriveDir[i] = 1;
      }
    }
  }

  // left turn-on-spot
  if (abs(msg.data[0] - 100) <= 5 && msg.data[1] < 95)
  {
    for (int i = 0; i < 6; i++)
    {
      if (i < 3)
      {
        digitalWrite(arrDriveDirPins[i], HIGH);
        arrDriveDir[i] = 1;
      }
      else
      {
        digitalWrite(arrDriveDirPins[i], HIGH);
        arrDriveDir[i] = 1;
      }
    }
  }

  // right turn-on-spot
  if (abs(msg.data[0] - 100) <= 5 && msg.data[1] > 105)
  {
    for (int i = 0; i < 6; i++)
    {
      if (i < 3)
      {
        digitalWrite(arrDriveDirPins[i], LOW);
        arrDriveDir[i] = 0;
      }
      else
      {
        digitalWrite(arrDriveDirPins[i], LOW);
        arrDriveDir[i] = 0;
      }
    }
  }

  // set PWM duty cycle for each wheel
  for (int i = 0; i < 6; i++)
  {
    arrDrivePWMDebug[i] = Overflow(3 * abs(arrDrivePWM[i] - 100));
    analogWrite(arrDrivePWMPins[i], arrDrivePWMDebug[i]);
  }

  drive_pwm_msg.data = arrDrivePWMDebug;
  drive_pwm_msg.data_length = 6;

  pubDrivePWM.publish(&drive_pwm_msg);

  drive_dir_msg.data = arrDriveDir;
  drive_dir_msg.data_length = 6;

  pubDriveDir.publish(&drive_dir_msg);
}

/* Function to stop an overflow in the 8-bit PWM value occurring */
int Overflow(int pwmInput)
{
  if (pwmInput > 250)
  {
    return 250;
  }
  else if (pwmInput < 0)
  {
    return 0;
  }
  else
  {
    return pwmInput;
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> subDriveCommand("arduino/drive_command", &update_drive);
ros::Subscriber<std_msgs::Int16MultiArray> subArmCommand("arduino/arm_command", &update_arm);



void setup()
{
  nh.initNode();
  nh.subscribe(subDriveCommand);
  nh.subscribe(subArmCommand);
  nh.advertise(pubDrivePWM);
  nh.advertise(pubDriveDir);

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
  for (int i = 0; i < 6; i++)
  {
    pinMode(arrArmDirPins[i], OUTPUT);
    digitalWrite(arrArmDirPins[i], LOW);
  }

}

void loop()
{
  nh.spinOnce();
  delay(1);
}

