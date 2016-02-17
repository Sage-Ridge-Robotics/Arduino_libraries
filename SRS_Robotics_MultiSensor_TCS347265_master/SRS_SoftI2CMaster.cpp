/*
 * SRS_SoftI2CMaster.cpp -- Multi-instance software I2C Master library
 *
 *
 * 2010-12 Tod E. Kurt, http://todbot.com/blog/
 *
 * This code takes some tricks from:
 * http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html
 *
 * 2014, by Testato: update library and examples for follow Wire’s API of Arduino IDE 1.x
 * Modifications 2015 by Christopher von Nagy incorporating code from Leo Linbeck III's 2015 Spark implementation
 * 2015 Integrated into SRS_Robotics_multisensor_TCS34725 library
 *
 */

#include <util/delay.h>
#include <string.h>

#include <avr/io.h>
#include <Arduino.h>

#include "SRS_SoftI2CMaster.h"


#define  i2cbitdelay 50

#define  I2C_ACK  1 
#define  I2C_NAK  0


/*
 * The following macros are expanded during compilation. For instance, i2c_sda_hi() becomes
 * *_sdaDirReg   &=~ _sdaBitMask; if(usePullups) { *_sdaPortReg  |=  _sdaBitMask; }. Note the
 * semi-colons indicating two complete lines of code. 
 * 
 */

#define i2c_scl_release()  *_sclDirReg   &=~ _sclBitMask
#define i2c_sda_release()  *_sdaDirReg   &=~ _sdaBitMask
#define i2c_scl_lo()       *_sclPortReg  &=~ _sclBitMask; *_sclDirReg   |=  _sclBitMask;                    // sets SCL low and drives output
#define i2c_sda_lo()       *_sdaPortReg  &=~ _sdaBitMask; *_sdaDirReg   |=  _sdaBitMask;                    // sets SDA low and drives output
#define i2c_scl_hi()       *_sclDirReg   &=~ _sclBitMask; if(usePullups) { *_sclPortReg  |=  _sclBitMask; } // set SCL high and to input 
                                                                                                            // (releases pin)
#define i2c_sda_hi()       *_sdaDirReg   &=~ _sdaBitMask; if(usePullups) { *_sdaPortReg  |=  _sdaBitMask; } // set SDA high and to input 
                                                                                                            // (releases pin)


//
// Constructor
//
SRS_SoftI2CMaster::SRS_SoftI2CMaster()
{
}

SRS_SoftI2CMaster::SRS_SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin) 
{
    setPins(sclPin, sdaPin, true);
    i2c_init();
    
}

SRS_SoftI2CMaster::SRS_SoftI2CMaster(uint8_t sclPin, uint8_t sdaPin, uint8_t pullups)
{
    setPins(sclPin, sdaPin, pullups);
    i2c_init();
}

//
// Turn Arduino pin numbers into PORTx, DDRx, and PINx
//
void SRS_SoftI2CMaster::setPins(uint8_t sclPin, uint8_t sdaPin, uint8_t pullups)
{
    uint8_t port;
    
    usePullups = pullups;

    _sclPin = sclPin;
    _sdaPin = sdaPin;
    
    _sclBitMask = digitalPinToBitMask(sclPin);  // digitalPinToBitMask() is a macro that returns the bitmask of a specified pin.
    _sdaBitMask = digitalPinToBitMask(sdaPin);  // digitalPinToBitMask() is defined in hardware/arduino/cores/arduino/Arduino.h
    
    port = digitalPinToPort(sclPin);            // digitalPinToPort() is a macro that returns the port of a specified pin.
    _sclPortReg  = portOutputRegister(port);    // digital_pin_to_port_PGM is defined in hardware/arduino/variants/standard/pins_arduino.h
    _sclDirReg   = portModeRegister(port);      // portOutputRegister() is a macro that returns an output port register of the specified port.
                                                // portModeRegister() is a macro that returns a mode register that controls 
                                                // the mode of the specified port.

    port = digitalPinToPort(sdaPin);
    _sdaPortReg  = portOutputRegister(port);
    _sdaDirReg   = portModeRegister(port);
    
}

//
// 
//

void SRS_SoftI2CMaster::beginTransmission(uint8_t address)
{
    i2c_start();
    i2c_write((address<<1) | 0); // clr read bit
}

void SRS_SoftI2CMaster::beginTransmission(int address)
{
    beginTransmission((uint8_t)address);
}

void SRS_SoftI2CMaster::requestFrom(uint8_t address)
{
    i2c_start();
    i2c_write((address<<1) | 1); // set read bit
}

void SRS_SoftI2CMaster::requestFrom(int address)
{
    requestFrom( (uint8_t) address);
}

void SRS_SoftI2CMaster::beginWriteTransmission(uint8_t address, uint8_t reg)
{
    i2c_start();
    i2c_write((address<<1) | 0); // clr read bit
    i2c_write(reg);
}

void SRS_SoftI2CMaster::beginWriteTransmission(int address, uint8_t reg)
{
    beginWriteTransmission((uint8_t) address, reg);
}

void SRS_SoftI2CMaster::beginReadTransmission(uint8_t address, uint8_t reg)
{
    i2c_start();
    i2c_write((address<<1) | 0); // clr read bit
    i2c_write(reg);
        i2c_start();
    i2c_write((address<<1) | 1); // set read bit
}

void SRS_SoftI2CMaster::beginReadTransmission(int address, uint8_t reg)
{
    beginReadTransmission( (uint8_t) address, reg);
}

//
//
//
uint8_t SRS_SoftI2CMaster::endTransmission(void)
{
    i2c_stop();
}


// must be called in:
// slave tx event callback
// or after beginTransmission(address)
uint8_t SRS_SoftI2CMaster::write(uint8_t data)
{
    // return i2c_write(data);
    i2c_write(data);
}



// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void SRS_SoftI2CMaster::write(uint8_t* data, uint8_t quantity)
{
    for(uint8_t i = 0; i < quantity; ++i){
        write(data[i]);
    }
}


// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void SRS_SoftI2CMaster::write(char* data)
{
    write((uint8_t*)data, strlen(data));
}


// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void SRS_SoftI2CMaster::write(int data)
{
    write((uint8_t)data);
}

// 
uint8_t SRS_SoftI2CMaster::read( uint8_t ack )
{
    return i2c_read( ack );
}

//
uint8_t SRS_SoftI2CMaster::read()
{
  return i2c_read( I2C_ACK );
  // return i2c_read(true);
}

//
uint8_t SRS_SoftI2CMaster::readLast()
{
  return i2c_read( I2C_NAK );
  // return i2c_read(false);
}


//------ PRIVATE METHODS --------------------------------------------

//
// Inits bitbanging port, must be called before using the functions below
//
void SRS_SoftI2CMaster::i2c_init(void)
{
  /*
   * I2C_PORT &=~ (_BV( I2C_SDA ) | _BV( I2C_SCL ));
   *_sclPortReg &=~ (_sdaBitMask | _sclBitMask);
   * 
   * We initialize SCL/SDA pins as output and set the bus high.
   * The equivalent of 
   * 
   * pinMode(_sdaPin, OUTPUT);
   * digitalWrite(_sdaPin, HIGH);
   * pinMode(_sclPin, OUTPUT);
   * digitalWrite(_sclPin, HIGH);
   * 
   */

//   i2c_sda_hi();
//   i2c_scl_hi();
  pinMode(_sdaPin, OUTPUT);
  digitalWrite(_sdaPin, HIGH);
  pinMode(_sclPin, OUTPUT);
  digitalWrite(_sclPin, HIGH);
  _delay_us(i2cbitdelay);
}

// Send a START Condition
//
void SRS_SoftI2CMaster::i2c_start(void)
{
   // set both to high at the same time
   //I2C_DDR &=~ (_BV( I2C_SDA ) | _BV( I2C_SCL ));
   //*_sclDirReg &=~ (_sdaBitMask | _sclBitMask);
//    i2c_sda_hi();
//    i2c_scl_hi();
  digitalWrite(_sdaPin, HIGH);
  digitalWrite(_sclPin, HIGH);

  _delay_us(i2cbitdelay);
  
//   i2c_sda_lo();
//   _delay_us(i2cbitdelay);

//   i2c_scl_lo();
//   _delay_us(i2cbitdelay);
  
  digitalWrite(_sdaPin, LOW);
  _delay_us(i2cbitdelay);
  digitalWrite(_sclPin, LOW);
  _delay_us(i2cbitdelay);

}

// Send a STOP Condition
//
void SRS_SoftI2CMaster::i2c_stop(void)
{
//   i2c_scl_hi();
//   _delay_us(i2cbitdelay);
// 
//   i2c_sda_hi();
//   _delay_us(i2cbitdelay);
  
  digitalWrite(_sdaPin, LOW);
  _delay_us(i2cbitdelay);
  digitalWrite(_sclPin, HIGH);
  _delay_us(i2cbitdelay);
  digitalWrite(_sdaPin, HIGH);
  _delay_us(i2cbitdelay);
}


// write a byte to the I2C slave device
//
void SRS_SoftI2CMaster::i2c_writebit( uint8_t c )
{
  if ( c > 0 ) {
      i2c_sda_hi();
  } else {
      i2c_sda_lo();
  }

  i2c_scl_hi();
  _delay_us(i2cbitdelay);

  i2c_scl_lo();
  _delay_us(i2cbitdelay);

  if ( c > 0 ) {
      i2c_sda_lo();
  }
  _delay_us(i2cbitdelay);
}

void SRS_SoftI2CMaster::i2c_write( uint8_t c )
{
    for ( uint8_t i=0;i<8;i++) {
        i2c_writebit( c & 128 );
        c<<=1;
    }
  Serial.print("Port for output: ");
  Serial.print(DDRD, BIN);  
  Serial.print("\t byte written:");
  Serial.print(c, HEX);
  Serial.print("\n");


    
}

// read a byte from the I2C slave device
//
uint8_t SRS_SoftI2CMaster::i2c_readbit(void)
{
  i2c_sda_hi();
  i2c_scl_hi();
  _delay_us(i2cbitdelay);

  uint8_t port = digitalPinToPort(_sdaPin);
  volatile uint8_t* pinReg = portInputRegister(port);
  uint8_t c = *pinReg;  // I2C_PIN;

  i2c_scl_lo();
  _delay_us(i2cbitdelay);

  return ( c & _sdaBitMask) ? 1 : 0;
}

uint8_t SRS_SoftI2CMaster::i2c_read( uint8_t ack )
{
  uint8_t res = 0;

  for ( uint8_t i=0;i<8;i++) {
      res <<= 1;
      res |= i2c_readbit();  
  }

  if ( ack )
      i2c_writebit( 0 );
  else
      i2c_writebit( 1 );

  _delay_us(i2cbitdelay);
  Serial.print("Port for input: ");
  Serial.print(DDRD, BIN);  
  Serial.print("\t byte read:");
  Serial.print(res, HEX);
  Serial.print("\n");
  
  return res;
}

void SRS_SoftI2CMaster::i2c_repstart(void)
{
  // set both to high at the same time (releases drive on both lines)
  //I2C_DDR &=~ (_BV( I2C_SDA ) | _BV( I2C_SCL ));
  //*_sclDirReg &=~ (_sdaBitMask | _sclBitMask);
  i2c_sda_hi();
  i2c_scl_hi();

  i2c_scl_lo();                           // force SCL low
  _delay_us(i2cbitdelay);

  i2c_sda_release();                      // release SDA
  _delay_us(i2cbitdelay);

  i2c_scl_release();                      // release SCL
  _delay_us(i2cbitdelay);

  i2c_sda_lo();                           // force SDA low
  _delay_us(i2cbitdelay);
}

void SRS_SoftI2CMaster::observeExecution(void)
{

  byte v01 = _sclBitMask;
  uint8_t v02 = *_sclPortReg;
  uint8_t v03 = *_sclDirReg;
  Serial.print("\nscl Pin:\t");
  Serial.print(_sclPin);
  Serial.print("\tPort:\t");
  Serial.print(digitalPinToPort(_sclPin), BIN);
  Serial.print("*\tSCL Bitmask:\t");
  Serial.print(v01, BIN);
  Serial.print("\tSCL Port (write) register state:\t");
  Serial.print(v02, BIN);
  Serial.print("\tSCL Mode register state:\t");
  Serial.print(v03, BIN);
  Serial.print("\n");
  
  byte v04 = _sdaBitMask;
  uint8_t v05 = *_sdaPortReg;
  uint8_t v06 = *_sdaDirReg;
  Serial.print("sda Pin:\t");
  Serial.print(_sdaPin);
  Serial.print("\tPort:\t");
  Serial.print(digitalPinToPort(_sdaPin), BIN);
  Serial.print("*\tSDA Bitmask:\t");
  Serial.print(v04, BIN);
  Serial.print("\tSDA Port (write) register state:\t");
  Serial.print(v05, BIN);
  Serial.print("\tSDA Mode register state:\t");
  Serial.print(v06, BIN);
  Serial.print("\n\n");

}

   
   
   
   
   
   
   
   
   
   
   
   
   
   








