/*
* SRS_SoftI2CMaster.h -- Multi-instance software I2C Master library
*
 * 2010-12 Tod E. Kurt, http://todbot.com/blog/
 *
 * This code takes some tricks from:
 * http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html
 *
 * 2014, by Testato: update library and examples for follow Wire’s API of Arduino IDE 1.x
 * Modifications 2015 by Christopher von Nagy borrowing some code from Leo Linbeck III's 2015 Spark board implementation
 * 2015 Integrated into SRS_Robotics_multisensor_TCS34725 library
*
*/
#ifndef SRS_SoftI2CMaster_h
#define SRS_SoftI2CMaster_h
#include <inttypes.h>
#define _SRS_SOFTI2CMASTER_VERSION 13 // software version of this library


class SRS_SoftI2CMaster
{

private:
  
  // per object data
  uint8_t _sdaPin;
  uint8_t _sclPin;

  uint8_t usePullups;

  uint8_t _sclBitMask;
  uint8_t _sdaBitMask;
  volatile uint8_t *_sclPortReg;
  volatile uint8_t *_sdaPortReg;
  volatile uint8_t *_sclDirReg;
  volatile uint8_t *_sdaDirReg;


  // private methods
  void i2c_init(void);
  void i2c_start(void);
  void i2c_repstart(void);
  void i2c_stop(void);
  void i2c_writebit( uint8_t c );
  void i2c_write( uint8_t c );
  void observeExecution();
  uint8_t i2c_readbit(void);
  uint8_t i2c_read( uint8_t ack );
  


public:
  // public methods
  
  SRS_SoftI2CMaster();
  SRS_SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin);
  SRS_SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin, uint8_t usePullups);
  void setPins(uint8_t sclPin, uint8_t sdaPin, uint8_t pullups);
  
  void beginTransmission(uint8_t address);
  void beginTransmission(int address);
  
  void beginWriteTransmission(uint8_t address, uint8_t reg);
  void beginWriteTransmission(int address, uint8_t reg);
  void beginReadTransmission(uint8_t address, uint8_t reg);
  void beginReadTransmission(int address, uint8_t reg);
  
  uint8_t endTransmission(void);
  
  uint8_t write(uint8_t);
  void write(uint8_t*, uint8_t);
  void write(int);
  void write(char*);
  
  void requestFrom(int address);
  void requestFrom(uint8_t address);
  uint8_t read( uint8_t ack );
  uint8_t read();
  uint8_t readLast();

};

#endif