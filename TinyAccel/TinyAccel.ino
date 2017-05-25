/*
TinyDuino Accelerometer Demo
  
Updated 15 August 2016 to use the correct serial monitor interface for
TinyDuino or TinyScreen+

This example code is in the public domain.

http://www.tinycircuits.com

*/


#include <Wire.h>
#include "BMA250.h"

//
//#if defined(ARDUINO_ARCH_SAMD)
//#define SerialMonitorInterface SerialUSB
//#else
//#define SerialMonitorInterface Serial
//#endif

BMA250 accel;


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  accel.begin(BMA250_range_4g, BMA250_update_time_32ms);//This sets up the BMA250 accelerometer
}

void loop()
{
 float accelScale =12.4;
  accel.read();//This function gets new data from the accelerometer
  float accelData; 
  accelData =  sqrt(sq(accel.X/accelScale) + sq(accel.Y/accelScale) + sq(accel.Z/accelScale));
    Serial.print("mag ");
      Serial.print(accelData);

  Serial.print("X = ");
  Serial.print(accelNorm(accel.X));
  Serial.print("  ");
  Serial.print("Y = ");
  Serial.print(accelNorm(accel.Y));
  Serial.print("  ");
  Serial.print("Z = ");
  Serial.print(accelNorm(accel.Z));
  Serial.print("  Temperature(C) = ");
  Serial.println((accel.rawTemp*0.5)+24.0,1);
  delay(250);//We'll make sure we're over the 64ms update time set on the BMA250
  
}


 double accelNorm(int value)
{
    double unit = 16.0 / 1023; // -4/+4G / signed 1023 (Range); 
    double normalized = value /12.44 ;
    return normalized;
}
