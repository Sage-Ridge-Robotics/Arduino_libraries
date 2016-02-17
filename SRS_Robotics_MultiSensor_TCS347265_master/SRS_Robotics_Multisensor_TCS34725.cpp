/**************************************************************************/
/*!
    @file     SRS_Robotics_Multisensor_TCS34725.cpp
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release

    Modifications by Leo Linbeck III to use SoftI2CMaster library
    to allow multiple sensors to be used with the same I2C address
    by using different SCL and SDA pins for each sensor
    
    Modifications by Christopher von Nagy to add a method to return L*a*b* 
    and XYZ color values.
*/
/**************************************************************************/
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "SRS_SoftI2CMaster.h"
#include "SRS_Robotics_Multisensor_TCS34725.h"

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void TCS34725::write8 (uint8_t reg, uint8_t value)
{
  _i2c.beginWriteTransmission(TCS34725_ADDRESS, (uint8_t) TCS34725_COMMAND_BIT | reg);
  _i2c.write(value);
  _i2c.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t TCS34725::read8(uint8_t reg)
{
  uint16_t res;

  _i2c.beginReadTransmission(TCS34725_ADDRESS, (uint8_t) TCS34725_COMMAND_BIT | reg);
  res = _i2c.readLast();
  _i2c.endTransmission();
  return res;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t TCS34725::read16(uint8_t reg)
{
  uint16_t x; uint16_t t;

  _i2c.beginReadTransmission(TCS34725_ADDRESS, (uint8_t) TCS34725_COMMAND_BIT | reg);
  t = _i2c.read();
  x = _i2c.readLast();
  _i2c.endTransmission();

  x <<= 8;
  x |= t;
  return x;
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void TCS34725::enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  _delay_ms(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void TCS34725::disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
TCS34725::TCS34725() {

}

TCS34725::TCS34725(tcs34725IntegrationTime_t it, tcs34725Gain_t gain, uint8_t sclPin, uint8_t sdaPin)
{
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
  _sdaPin = sdaPin;
  _sclPin = sclPin;
  _i2c = SRS_SoftI2CMaster(sclPin, sdaPin);
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
bool TCS34725::begin(void)
{
  /* Make sure we're actually connected */
  uint8_t x = read8(TCS34725_ID);
  if (x != 0x44)
  {
    Serial.print("Error initializing sensor: ");
    Serial.println(x, HEX);
    _tcs34725Initialised = false;
    return false;
  }
  else {
    _tcs34725Initialised = true;

    /* Set default integration time and gain */
    setIntegrationTime(_tcs34725IntegrationTime);
    setGain(_tcs34725Gain);

    /* Note: by default, the device is in power down mode on bootup */
    enable();
    return true;
  }
}

/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void TCS34725::setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void TCS34725::setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void TCS34725::getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) begin();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);

  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      _delay_ms(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      _delay_ms(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      _delay_ms(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      _delay_ms(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      _delay_ms(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      _delay_ms(700);
      break;
  }
}


/**************************************************************************/
/*!
    @brief  Returns CIE XYZ values for the sensed color
*/
/**************************************************************************/
void TCS34725::getXYZ (float *X, float *Y, float *Z)
{
  if (!_tcs34725Initialised) begin();
  
  uint16_t r_raw, g_raw, b_raw, c_raw;
  float rgb_matrix[3];
  
  c_raw = read16(TCS34725_CDATAL);
  r_raw = read16(TCS34725_RDATAL);
  g_raw = read16(TCS34725_GDATAL);
  b_raw = read16(TCS34725_BDATAL);
 
  // Rescale gain-dependent uint16 color levels to 0..1
  
  uint32_t sum = c_raw;
  rgb_matrix[0] = r_raw; rgb_matrix[0] /= sum;
  rgb_matrix[1] = g_raw; rgb_matrix[1] /= sum;
  rgb_matrix[2] = b_raw; rgb_matrix[2] /= sum;
  
  // Assuming sRGB (D65)
  if (rgb_matrix[0] <= 0.04045){rgb_matrix[0] = rgb_matrix[0]/12;} else {rgb_matrix[0] = powf((rgb_matrix[0]+0.055)/1.055,2.4);}
  if (rgb_matrix[1] <= 0.04045){rgb_matrix[1] = rgb_matrix[1]/12;} else {rgb_matrix[1] = powf((rgb_matrix[1]+0.055)/1.055,2.4);}
  if (rgb_matrix[2] <= 0.04045){rgb_matrix[2] = rgb_matrix[2]/12;} else {rgb_matrix[2] = powf((rgb_matrix[2]+0.055)/1.055,2.4);}
  
  *X = (0.436052025 * rgb_matrix[0]) + (0.385081593 * rgb_matrix[1]) + (0.143087414 * rgb_matrix[2]);
  *Y = (0.222491598 * rgb_matrix[0]) + (0.71688606 * rgb_matrix[1]) + (0.060621486 * rgb_matrix[2]);
  *Z = (0.013929122 * rgb_matrix[0]) + (0.097097002 * rgb_matrix[1]) + (0.71418547 * rgb_matrix[2]);


  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      _delay_ms(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      _delay_ms(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      _delay_ms(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      _delay_ms(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      _delay_ms(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      _delay_ms(700);
      break;
  }
}



/**************************************************************************/
/*!
    @brief  Returns CIE L*a*b* values for the sensed color
*/
/**************************************************************************/
void TCS34725::getLAB (float *L, float *a, float *b)
{
  if (!_tcs34725Initialised) begin();
  
  uint16_t r_raw, g_raw, b_raw, c_raw;
  float rgb_matrix[3];
  
  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;
  float fx, fy, fz;
  float xr, yr, zr;
  float Ls, as, bs;
  float eps = 0.00885645; // eps = 216.0/24389
  float k = 903.296;      // k = 24389.0/27
  double ex = 1/3;
  
  c_raw = read16(TCS34725_CDATAL);
  r_raw = read16(TCS34725_RDATAL);
  g_raw = read16(TCS34725_GDATAL);
  b_raw = read16(TCS34725_BDATAL);
 
  // Rescale gain-dependent uint16 color levels to 0..1
  
  uint32_t sum = c_raw;
  rgb_matrix[0] = r_raw; rgb_matrix[0] /= sum;
  rgb_matrix[1] = g_raw; rgb_matrix[1] /= sum;
  rgb_matrix[2] = b_raw; rgb_matrix[2] /= sum;
    
  float D50[3] = {0.964221,1.0,0.825211};         // CIE XYZ D50 standard white point values.  Best fits the TCS34725's LED (4150°K)?
  
  // Assuming sRGB (D65)
  if (rgb_matrix[0] <= 0.04045){rgb_matrix[0] = rgb_matrix[0]/12;} else {rgb_matrix[0] = powf((rgb_matrix[0]+0.055)/1.055,2.4);}
  if (rgb_matrix[1] <= 0.04045){rgb_matrix[1] = rgb_matrix[1]/12;} else {rgb_matrix[1] = powf((rgb_matrix[1]+0.055)/1.055,2.4);}
  if (rgb_matrix[2] <= 0.04045){rgb_matrix[2] = rgb_matrix[2]/12;} else {rgb_matrix[2] = powf((rgb_matrix[2]+0.055)/1.055,2.4);}
  
  X = (0.436052025 * rgb_matrix[0]) + (0.385081593 * rgb_matrix[1]) + (0.143087414 * rgb_matrix[2]);
  Y = (0.222491598 * rgb_matrix[0]) + (0.71688606 * rgb_matrix[1]) + (0.060621486 * rgb_matrix[2]);
  Z = (0.013929122 * rgb_matrix[0]) + (0.097097002 * rgb_matrix[1]) + (0.71418547 * rgb_matrix[2]);
  
  // CIE XYZ to CIE L*a*b* conversion
  // White point correction
  xr = X/D50[0];
  yr = Y/D50[1];
  zr = Z/D50[2];
  
  if ( xr > eps ) {fx = powf(xr,0.333333);} else {fx = (k * xr + 16) / 116;}
  if ( yr > eps ) {fy = powf(yr,0.333333);} else {fy = (k * yr + 16) / 116;}
  if ( yr > eps ) {fz = powf(zr,0.333333);} else {fz = (k * zr + 16) / 116;}
  
  Ls = ( 116 * fy ) - 16;
  as = 500*(fx-fy);
  bs = 200*(fy-fz);
  
  *L = (( 2.55 * Ls ) + .5);
  *a = (as + .5); 
  *b = (bs + .5);

  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      _delay_ms(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      _delay_ms(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      _delay_ms(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      _delay_ms(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      _delay_ms(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      _delay_ms(700);
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t TCS34725::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}


void TCS34725::setInterrupt(bool flag) {
//   uint8_t r = read8(TCS34725_ENABLE);
//   if (flag) {
//     r |= TCS34725_ENABLE_AIEN;
//   } else {
//     r &= ~TCS34725_ENABLE_AIEN;
//   }
//   write8(TCS34725_ENABLE, r);
  if (flag) {
     write8(TCS34725_ENABLE, 0x11);
  	 _delay_ms(10);
     write8(TCS34725_ENABLE, 0x13);
  }
  else {
     write8(TCS34725_ENABLE, 0x01);
  	 _delay_ms(10);
     write8(TCS34725_ENABLE, 0x03);
  }
}

void TCS34725::startReading(bool useLED) {
  uint8_t r;

  if (useLED) {
	 do {
     	write8(TCS34725_ENABLE, 0x01);
  	 	_delay_ms(10);
     	write8(TCS34725_ENABLE, 0x03);
	 	r = read8(TCS34725_ENABLE);
	 } while (r != 0x03);
  }
  else {
	 do {
     	write8(TCS34725_ENABLE, 0x11);
  	 	_delay_ms(10);
     	write8(TCS34725_ENABLE, 0x13);
	 	r = read8(TCS34725_ENABLE);
	 } while (r != 0x13);
  }
}

void TCS34725::endReading(void) {
	uint8_t r;

	do {
		write8(TCS34725_ENABLE, 0x11);
	 	r = read8(TCS34725_ENABLE);
	} while (r != 0x11);
}

void TCS34725::clearInterrupt(void) {
	write8(TCS34725_ADDRESS, 0x66);
}


void TCS34725::setIntLimits(uint16_t low, uint16_t high) {
   write8(0x04, low & 0xFF);
   write8(0x05, low >> 8);
   write8(0x06, high & 0xFF);
   write8(0x07, high >> 8);
}
