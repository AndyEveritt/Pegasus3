#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle nh;

/* Servo global variables */
Servo servoYaw;
Servo servoPitch;

const int servoYawPin = 9;
const int servoPitchPin = 10;
float angleYaw = 0;
float anglePitch = 0;
int8_t angleMax = 100;
int8_t angleMin = 20;

/* Encoder global variables */
unsigned int count[6] = {0, 0, 0, 0, 0, 0};  // int which counts the number of times a wheel has completed a full revolution
const int CSn[6] = {22, 26, 30, 34, 38, 42}; // Chip select pin
const int CLK[6] = {23, 27, 31, 35, 39, 43}; // Clock pin
const int DO[6] = {24, 28, 32, 36, 40, 44};  // Encoder output pin

void updateServoYaw(const std_msgs::Float32 &msg)
{
    if (abs(msg.data) >= 0.2)
    {
        angleYaw += msg.data;
    }
    
    if (angleYaw >= angleMax)
    {
        angleYaw = angleMax;
    }
    else if (angleYaw <= angleMin)
    {
        angleYaw = angleMin;
    }

    servoYaw.write(int(angleYaw));
}

void updateServoPitch(const std_msgs::Float32 &msg)
{
    if (abs(msg.data) >= 0.2)
    {
        anglePitch += msg.data;
    }

    if (anglePitch >= angleMax)
    {
        anglePitch = angleMax;
    }
    else if (anglePitch <= angleMin)
    {
        anglePitch = angleMin;
    }

    servoPitch.write(int(anglePitch));
}

// Returns a value between 0 and 1023, indicating position of shaft
unsigned int readSensor(const int i)
{
    unsigned int dataOut = 0;

    digitalWrite(CSn[i], LOW);
    delayMicroseconds(1); // Wait for t_clkfe

    // Read each of the 10 bits in the encoder message
    for (int x = 0; x < 10; x++)
    {
        digitalWrite(CLK[i], LOW);
        delayMicroseconds(1); // Wait for t_clk/2
        digitalWrite(CLK[i], HIGH);
        delayMicroseconds(1);                          // Wait for t_clk/2
        dataOut = (dataOut << 1) | digitalRead(DO[i]); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
    }

    digitalWrite(CSn[i], HIGH);
    // Serial.println(dataOut);
    delayMicroseconds(1); // Wait for t_cs
    return dataOut;
}

ros::Subscriber<std_msgs::Float32> subYawRate("servo_yaw_rate", &updateServoYaw);
ros::Subscriber<std_msgs::Float32> subPitchRate("servo_pitch_rate", &updateServoPitch);

std_msgs::UInt16MultiArray encoder_msg;
ros::Publisher pubEncoder("encoder_counts", &encoder_msg);

void setup()
{
    nh.initNode();
    nh.subscribe(subYawRate);
    nh.subscribe(subPitchRate);
    nh.advertise(pubEncoder);

    /* Camera servo setup */
    servoYaw.attach(servoYawPin);
    servoPitch.attach(servoPitchPin);
    servoYaw.write(0);
    servoPitch.write(50);

    /* Encoder setup */
    // Declare the pin modes for the chip select, clock and encoder output pin
    for (int i = 0; i < 6; i++)
    {
        pinMode(CSn[i], OUTPUT);
        pinMode(CLK[i], OUTPUT);
        pinMode(DO[i], INPUT);

        // Set the clock and chip select to high at first
        digitalWrite(CLK[i], HIGH);
        digitalWrite(CSn[i], HIGH);
    }
}

void loop()
{
    for (int i = 0; i < 6; i++)
    {
        count[i] = readSensor(i);
    }

    encoder_msg.data = count;
    encoder_msg.data_length = 6;

    pubEncoder.publish(&encoder_msg);

    nh.spinOnce();
    delay(1);
}