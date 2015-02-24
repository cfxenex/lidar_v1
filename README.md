// lidar_v1
// created by chris farquer

#include <Wire.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

//continous read int
int reading = 0;

//calibration ints
int sensorValue = 0;    
int sensorMin = 1000;        //30ft is approximatly a value of 900,  note every 10ft = 300
int sensorMax = 0;

//averaging and filter ints
const int numSamples = 10;
int dist_reading[numSamples];
int total = 0;
int index = 0;
int average = 0;


void setup()
{
  //digital pin output for buzzer
  pinMode(13,OUTPUT);
  //digital pin output for LED
  pinMode(8,OUTPUT);
  
  //calibrate sensor  
  while (millis() < 5000)
  {
    Wire.begin(); // join i2c bus
    Serial.begin(115200); // start serial communication at 115200bps
    Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
    Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
    Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
    Wire.endTransmission(); // stop transmitting
    delay(20); // Wait 20ms for transmit
    Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
    Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
    Wire.endTransmission(); // stop transmitting
    delay(20); // Wait 20ms for transmit
    Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite
    if(2 <= Wire.available()) // if two bytes were received
    {
      sensorValue = Wire.read(); // receive high byte (overwrites previous reading)
      sensorValue = sensorValue << 8; // shift high byte to be high 8 bits
      sensorValue |= Wire.read(); // receive low byte as lower 8 bits
    }
    if (sensorValue > sensorMax)
    {
      sensorMax = sensorValue;
    }
    if (sensorValue < sensorMin)
    {
      sensorMin = sensorValue;
    }
  }
  Serial.print("LIDAR Sensor Calibration Value=");
  Serial.println(sensorValue);
}


void loop()
{
  //continously read distance
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
    {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits  
   }
  
  //indexing values
  total = total - dist_reading[index];
  dist_reading[index] = reading;
  total = total + dist_reading[index];
  
  /*filter to compensate for bad sensor readings
  replaces extreme highs and all negative values with
  previous sensor reading*/
  if (reading >= 1000 || reading < 0)
  {
    index = index - 1;
  }
  else
  {
    index = index + 1;
  }
  
  //averaging
  if( index >= numSamples)
  {
    index = 0;
    average = total / numSamples;
    Serial.print("Average Distance=");
    Serial.println(average);
    delay(1);
  }  
  
  //turn buzzer/LED on if average distance changes by 3cm from calibrated value
  if(average <= sensorValue - 2 || average >= sensorValue + 2)
  {
   digitalWrite(13,HIGH);  //buzzer
   digitalWrite(8,HIGH);  //LED
  }
  else
  {
  digitalWrite(13,LOW);
  digitalWrite(8,LOW);
  }
  delay(1);  
}
