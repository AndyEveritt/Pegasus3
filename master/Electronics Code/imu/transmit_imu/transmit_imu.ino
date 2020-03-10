#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "NAxisMotion.h" //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

NAxisMotion mySensor;             //Object that for the sensor
unsigned long lastStreamTime = 0; //To store the last streamed time stamp
const int streamPeriod = 20;      //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
float AngularVelX, AngularVelY, AngularVelZ, Tmp, EulerH, EulerR, EulerP, LinAcX, LinAcY, LinAcZ;


void LEDtoggle(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }
}


//Set up the ros node and publisher
std_msgs::String imu_msg;
ros::Publisher imu("imu/raw", &imu_msg);
ros::Subscriber<std_msgs::Bool> sub("marker_found", &LEDtoggle);
ros::NodeHandle nh;

void setup()
{
  // autonomous tennis ball marker found LED
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.advertise(imu);
  nh.subscribe(sub);

  //Peripheral Initialization
  I2C.begin(); //Initialize I2C communication to the let the library communicate with the sensor.

  //Sensor Initialization
  mySensor.initSensor();                          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF); //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);                 //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor
}

long publisher_timer = 0;

void loop()
{
  lastStreamTime = millis();
  mySensor.updateEuler();
  mySensor.updateGyro();
  mySensor.updateLinearAccel();
  mySensor.updateCalibStatus();

  EulerH = mySensor.readEulerHeading();
  EulerR = mySensor.readEulerRoll();
  EulerP = mySensor.readEulerPitch();
  AngularVelX = mySensor.readGyroX();
  AngularVelY = mySensor.readGyroY();
  AngularVelZ = mySensor.readGyroZ();
  LinAcX = mySensor.readLinearAccelX();
  LinAcY = mySensor.readLinearAccelY();
  LinAcZ = mySensor.readLinearAccelZ();

  // float data_float[9] = {EulerH, EulerR, EulerP, AngularVelX, AngularVelY, AngularVelZ, LinAcX, LinAcY, LinAcZ};

  char AX[15], AY[15], AZ[15], LX[15], LY[15], LZ[15], OX[15], OY[15], OZ[15];

  dtostrf(AngularVelX, 6, 2, AX);
  dtostrf(AngularVelY, 6, 2, AY);
  dtostrf(AngularVelZ, 6, 2, AZ);
  dtostrf(LinAcX, 6, 2, LX);
  dtostrf(LinAcY, 6, 2, LY);
  dtostrf(LinAcZ, 6, 2, LZ);
  dtostrf(EulerP, 6, 2, OX);
  dtostrf(EulerR, 6, 2, OY);
  dtostrf(EulerH, 6, 2, OZ);

  char data[100];
  // sprintf(data, "A%sB%sC%sD", OX, OY, OZ);
  sprintf(data, "A%sB%sC%sD%sE%sF%sG%sH%sI%sJ", OX, OY, OZ, LX, LY, LZ, AX, AY, AZ);
  // int length = data.indexOf("J") + 2;
  // char data_final[length + 1];
  // data.toCharArray(data_final, length + 1);

  if (millis() > publisher_timer)
  {
    Serial.println(data);
    // for (int i=0; i<9; i++){
    //   Serial.print(data_float[i]);
    //   Serial.print(' ');
    // }
    // Serial.print('\n');

    // step 1: request reading from sensor
    imu_msg.data = data;
    imu.publish(&imu_msg);
    publisher_timer = millis() + 100; //publish ten times a second
    nh.spinOnce();
  }
}