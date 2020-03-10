// Read the output of a single AEAT-6010-A06 encoder
// Modified from https://forum.arduino.cc/index.php?topic=164353.0
// Written using the output signal described in the AEAT-6010 datasheet

// int which counts the number of times a wheel has completed a full revolution
unsigned int count = 0;

// holds most recent sensor reading
unsigned int prevReading = readSensor();

const int CSn = 4; // Chip select pin
const int CLK = 7; // Clock pin
const int DO = 8; // Encoder output pin

void setup()
{

  // Begin serial

  // Declare the pin modes for the chip select, clock and encoder output pin

  pinMode(CSn, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DO, INPUT);

  // Set the clock and chip select to high at first
  digitalWrite(CLK, HIGH);
  digitalWrite(CSn, HIGH);
}



void loop() 
{
  Serial.println(odometry_speed());
}

// returns a float representing the total distance moved by a wheel (m)
// need to detect if moving backwards
float odometry_distance()
{
  float newReading;
  float delta_r;

  newReading = readSensor();

  if(newReading < prevReading)
  {
    count++;
    delta_r = (newReading/(float) 1024);
  }
  else
  {
    delta_r = ((newReading - prevReading)/(float) 1024);
  }

  return (count + delta_r) * 2 * PI * 0.1 * 1/(float)49;
}

float odometry_speed()
{
  float newReading;
  float delta_r;

  newReading = readSensor();

  if(newReading < prevReading)
  {
    count++;
    delta_r = (newReading/(float) 1024);
  }
  else
  {
    delta_r = ((newReading - prevReading)/(float) 1024);
  }

  return delta_r;
}

// Returns a value between 0 and 1023, indicating position of shaft
unsigned int readSensor()
{
  unsigned int dataOut = 0;

  digitalWrite(CSn, LOW);
  delayMicroseconds(1); // Wait for t_clkfe

  // Read each of the 10 bits in the encoder message
  for(int x = 0; x < 10; x++)
  {
    digitalWrite(CLK, LOW);
    delayMicroseconds(1); // Wait for t_clk/2
    digitalWrite(CLK, HIGH);
    delayMicroseconds(1); // Wait for t_clk/2
    dataOut = (dataOut << 1) | digitalRead(DO); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
  }

  digitalWrite(CSn, HIGH);
  Serial.println(dataOut);
  delayMicroseconds(1); // Wait for t_cs
  return dataOut;
}
