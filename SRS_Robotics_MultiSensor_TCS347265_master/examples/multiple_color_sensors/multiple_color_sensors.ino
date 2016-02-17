/*

Sage Ridge Robotics
TCS34725 with CIE L*a*b* example
Uses one or two sensors to return CIE L*a*b* color values
Note: Use pins A1 for scl and A0 for sda on the Arduino UNO we use in class. These are the standard pins used. If you need
to instantiate a second I2C object and bus, you might consider using Analog pins A2 and A3.

*/

#include "SRS_Robotics_Multisensor_TCS34725.h"
#include "SRS_SoftI2CMaster.h"

#define ERROR_MESSAGE(err) Serial.println(err)

/*
 * Pin definitions
 * Use the following to define the SDA and SCL pins
 *  
 * Digital 0--7 are referenced as 0 through 7, considered PORTD (PD)
 * Digital 8--13 are referenced as 8 through 13, considered PORTB (PB)
 * Analog 0--5 are referenced as 14 throug 20, considered PORTC (PC)
 * 
 * It is important to take care not to accidentally change the pin status of
 * those used for serial communication. 
 * 
 */

int pinTCS01SCL = 0;                             // Digital pin  0 (PORTD)
int pinTCS01SDA = 1;                             // Digital pin  1 (PORTD)
//int pinTCS01SCL = 11;                          // Digital pin 11 (PORTB)
//int pinTCS01SDA = 12;                          // Digital pin 12 (PORTB)
//int pinTCS01SCL = 14;                          // Analog  pin  0 (PORTC)
//int pinTCS01SDA = 15;                          // Analog  pin  0 (PORTC)


//int pinTCS02SDA = 2;                           // Digital pin  2 (PORTD)
//int pinTCS02SCL = 3;                           // Digital pin  3 (PORTD)



// Instantiate TCS34725 object(s)
TCS34725 tcs01;
//TCS34725 tcs02;


// Set up sensor pins and start serial connection
void setup() {
  Serial.begin(9600);
}


// Loop
void loop(){
  
  // Each time we loop we will initialize and enable the sensor(s), take a set of readings, and then shut down the sensor
  
  // Here we establish communication with the sensor(s). 
  // Note below an error is returned in the case of a problem
  init_sensor(&tcs01, pinTCS01SCL, pinTCS01SDA);
  //init_sensor(&tcs02, pinTCS02SCL, pinTCS02SDA);

  // To collect readings from a second sensor or to print RGB instead of LAB values, modify this function below
  collect_sensor_readings();

  tcs01.disable();
  //tcs02.disable();
  
  // wait 10 seconds until taking next reading
  delay(10000); 

}

void init_sensor(TCS34725 *sensor, int sclPin, int sdaPin) {

  // Initializing the sensor requires we pass a desired integration time, the gain (sensitivity to light), 
  // and the clock and data pins of the I2C bus
  *sensor = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X, sclPin, sdaPin);

  if (sensor->begin()) {
    sensor->enable();
  }
  else {
     ERROR_MESSAGE("Sensor not found");
   }
   delay(1000); // let sensor "warm up"
}

void read_lab(TCS34725 *sensor, int reading_number) {
    float L, a, b;

    sensor->getLAB(&L, &a, &b);

    Serial.print("Reading number: ");
    Serial.print(reading_number);
    Serial.print("CIE L*a*b* L*: ");
    Serial.print(L);
    Serial.print(", a*: ");
    Serial.print(a);
    Serial.print(", b*: ");
    Serial.println(b);
}


void read_sensor(TCS34725 *sensor, int reading_number) {
    uint16_t clear, red, green, blue;

    sensor->getRawData(&red, &green, &blue, &clear);

    Serial.print("Reading number: ");
    Serial.print(reading_number);
    Serial.print(", Clear: ");
    Serial.print(clear);
    Serial.print(", Red: ");
    Serial.print(red);
    Serial.print(", Green: ");
    Serial.print(green);
    Serial.print(", Blue: ");
    Serial.println(blue);
}

void collect_sensor_readings() {
    for (int i = 0; i < 10; i += 1) { // take 10 sensor readings
        read_lab(&tcs01, i);
        //read_lab(&tcs02, i);
        delay(1000);
    }
}