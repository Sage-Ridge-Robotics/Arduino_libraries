/*

Sage Ridge Robotics
SRS Robotics version of the Adafruit_CS34725 library
CIE L*a*b* color return added

This example uses a single sensor to return raw RGB and Pan (clear) values
Note: Use pins A5 for scl and A4 for sda on the Arduino UNO we use in class

*/

#include <Wire.h>
#include "SRS_Robotics_Adafruit_TCS34725.h"

// Instantiate an TCS34725 object
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


// Set up sensor pins and start serial connection
void setup() {
  Serial.begin(9600);
  Serial.println("Color View Test!");

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}


// Loop
void loop(){
  
  read_RGBP(&tcs);  

}

void read_RGBP(Adafruit_TCS34725 *sensor) {
  
    uint16_t r, g, b, p;
    tcs.setInterrupt(false);
    delay(60);
    sensor->getRawData(&r, &g, &b, &p);
    Serial.print("RGB Red: ");
    Serial.print(r);
    Serial.print(", Green: ");
    Serial.print(g);
    Serial.print(", Blue: ");
    Serial.println(b);
    Serial.print(", Pan: ");
    Serial.println(p);   
    tcs.setInterrupt(true);
}