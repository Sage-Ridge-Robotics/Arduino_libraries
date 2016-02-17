/*

Sage Ridge Robotics
SRS Robotics version of the Adafruit_CS34725 library
CIE L*a*b* color return added

This example uses a single sensor to return an averaged CIE L*a*b* color value
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
  
  read_lab(&tcs);  

}

void read_lab(Adafruit_TCS34725 *sensor) {
  
    float L, a, b;
    tcs.setInterrupt(false);
    delay(60);
    sensor->getLAB(&L, &a, &b);
    Serial.print("CIELAB\tL*: ");
    Serial.print(L);
    Serial.print(",\ta*: ");
    Serial.print(a);
    Serial.print(",\tb*: ");
    Serial.println(b);
    tcs.setInterrupt(true);
}