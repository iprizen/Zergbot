/*
 * File  : wiinunchuck.h V0.9
 * Author: Tim Teatro
 * Date  : Feb 2012
 *
 * Description:
 *
 *   Library to set up and poll a Wii nunchuck with Arduino. There are
 * many libraries available to do this, none of which I really liked.
 * I was fond of Tod Kurt's, but his was incomplete as it did not work
 * with knockoff nunchucks, it did not consider the least significant
 * bits of accelerometer data and didn't have any advanced functions
 * for processing the data such as calculating pitch and roll angles.
 *
 *
 * Provides functions:
 *  void nunchuck_setpowerpins()
 *  void nunchuck_init()
 *  int nunchuck_get_data()
 *  void nunchuck_calibrate_joy()
 *  inline unsigned int nunchuck_zbutton()
 *  inline unsigned int nunchuck_cbutton()
 *  inline int nunchuck_joy_x()
 *  inline int nunchuck_cjoy_x()
 *  inline int nunchuck_cjoy_y()
 *  inline uint16_t nunchuck_accelx()
 *  inline uint16_t nunchuck_accely()
 *  inline uint16_t nunchuck_accelz()
 *  inline int nunchuck_caccelx()
 *  inline int nunchuck_caccely()
 *  inline int nunchuck_caccelz()
 *  inline int nunchuck_joyangle()
 *  inline int nunchuck_rollangle()
 *  inline int nunchuck_pitchangle()
 *  void nunchuck_calibrate_accelxy()
 *  void nunchuck_calibrate_accelz()
 *
 * This library is inspired by the work of Tod E. Kurt,
 *	(http://todbot.com/blog/bionicarduino/)
 *
 * (c) 2012 by Tim Teatro
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//
// These are suitable defaults for most nunchucks, including knockoffs.
// If you intend to use the same nunchuck all the time and demand accu-
// racy, it is worth your time to measure these on your own.
//     If you may want to use various nunchucks, you may want to
// calibrate using functions
//   nunchuck_calibrate_joy()
//   nunchuck_calibrate_accelxy()
//   nunchuck_calibrate_accelz()
//
#define DEFAULT_CENTRE_JOY_X 124
#define DEFAULT_CENTRE_JOY_Y 132
#define ACCEL_ZEROX 490
#define ACCEL_ZEROY 500
#define ACCEL_ZEROZ 525

//
// Global vars are kept to a minimum.
//
uint8_t ctrlr_type[6];   // Used externally?
uint8_t nunchuck_buf[6];  // Keeps data payload from nunchuck
// Accelerometer values and callibration centres:
uint16_t accel_zerox, accel_zeroy, accel_zeroz;
// Joystick values and calibration centres:
int joy_x, joy_y, joy_zerox, joy_zeroy;

//
//
// Uses port C (analog in) pins as power & ground for nunchuck
//
void nunchuck_setpowerpins()
	{
	#define pwrpin PORTC3
	#define gndpin PORTC2
	DDRC |= _BV(pwrpin) | _BV(gndpin);
	PORTC &=~ _BV(gndpin);
	PORTC |=  _BV(pwrpin);
	delay(100); // wait for things to stabilize
	}

//
//
// Initialize and join the I2C bus, and tell the nunchuck we're
// talking to it. This function will work both with Nintendo
// nunchucks, or knockoffs.
//
// See http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264805255
//
void nunchuck_init()
	{
	Wire.begin();
	delay(1);
	Wire.beginTransmission(0x52);  // device address
	#if (ARDUINO >= 100)
		Wire.write((uint8_t)0xF0);  // 1st initialisation register
		Wire.write((uint8_t)0x55);  // 1st initialisation value
		Wire.endTransmission();
		delay(1);
		Wire.beginTransmission(0x52);
		Wire.write((uint8_t)0xFB);  // 2nd initialisation register
		Wire.write((uint8_t)0x00);  // 2nd initialisation value
	#else
		Wire.send((uint8_t)0xF0);   // 1st initialisation register
		Wire.send((uint8_t)0x55);   // 1st initialisation value
		Wire.endTransmission();
		delay(1);
		Wire.beginTransmission(0x52);
		Wire.send((uint8_t)0xFB);   // 2nd initialisation register
		Wire.send((uint8_t)0x00);   // 2nd initialisation value
	#endif
	Wire.endTransmission();
	delay(1);
	//
	// Set default calibration centres:
	//
	joy_zerox = DEFAULT_CENTRE_JOY_X;
	joy_zeroy = DEFAULT_CENTRE_JOY_Y;
	accel_zerox = ACCEL_ZEROX;
	accel_zeroy = ACCEL_ZEROY;
	accel_zeroz = ACCEL_ZEROZ;
	}

//
//
// T.T.
// Standard nunchucks use a byte-wise encryption using bit-wise XOR
// with 0x17. This function decodes a byte.
//
// This function is not needed since the way we initialize the nunchuck
// does not XOR encrypt the bits.
//
//static inline char nunchuck_decode_byte (char x)
//	{
//	x = (x ^ 0x17) + 0x17;
//	return x;
//	}

static void nunchuck_send_request()
{
	Wire.beginTransmission(0x52);// transmit to device 0x52
	#if (ARDUINO >= 100)
		Wire.write((uint8_t)0x00);// sends one byte
	#else
		Wire.send((uint8_t)0x00);// sends one byte
	#endif
	Wire.endTransmission();// stop transmitting
}

//
//
// Gets data from the nunchuck and packs it into the nunchuck_buff byte
// aray. That array will be processed by other functions to extract
// the data from the sensors and analyse.
//
int nunchuck_get_data()
	{
	int cnt=0;
	  // Request six bytes from the chuck.
	Wire.requestFrom (0x52, 6);
	while (Wire.available ())
		{
	  // receive byte as an integer
		#if (ARDUINO >= 100)
			nunchuck_buf[cnt] = Wire.read();
		#else
			nunchuck_buf[cnt] = Wire.receive();
		#endif
		cnt++;
		}

	Wire.beginTransmission(0x52);// transmit to device 0x52
	#if (ARDUINO >= 100)
		Wire.write((uint8_t)0x00);// sends one byte
	#else
		Wire.send((uint8_t)0x00);// sends one byte
	#endif
	Wire.endTransmission();// stop transmitting

	if (cnt >= 5)
		{
		return 1;   // success
		}
	return 0; // failure
	}

//
//
//
// Calibrate joystick so that we read the centre position as (0,0).
// Otherwise, we use the default values from the header.
//
void nunchuck_calibrate_joy()
	{
	joy_zerox = joy_x;
	joy_zeroy = joy_y;
	}

// Returns c and z button states: 1=pressed, 0=not
// The state is in the two least significant bits of the 6th byte.
// In the data, a 1 is unpressed and 0 is pressed, so this will be
// reversed. These functions use a bitwise AND to determine the value
// and then the () ? true : false; conditional structure to pass out
// the appropriate state.
//
static inline unsigned int nunchuck_zbutton()
	{
	return ((nunchuck_buf[5] >> 0) & 1) ? 0 : 1;
	}

static inline unsigned int nunchuck_cbutton()
	{
	return ((nunchuck_buf[5] >> 1) & 1) ? 0 : 1;
	}

//
//
// Returns the raw x and y values of the the joystick, cast as ints.
//
static inline int nunchuck_joy_x()
	{
	return (int) nunchuck_buf[0];
	}

static inline int nunchuck_joy_y()
	{
	return (int) nunchuck_buf[1];
	}

//
//
// Return calibrated x and y values of the joystick.
//
static inline int nunchuck_cjoy_x()
	{
	return (int)nunchuck_buf[0] - joy_zerox;
	}

static inline int nunchuck_cjoy_y()
	{
	return (int)nunchuck_buf[1] - joy_zeroy;
	}

//
//
// Returns the raw 10-bit values from the 3-axis accelerometer sensor.
// Of the six bytes recieved in a data payload from the nunchuck, bytes
// 2, 3 and 4 are the most significant 8 bits of each 10-bit reading.
// The final two bits are stored in the 6th bit along with the states
// of the c and z button. These functions take the most significant
// 8-bits and stacks it into a 16 bit unsigned integer, and then tacks
// on the least significant bits from the 6th byte of the data
// payload.
// 
// Load the most sig digits into a blank 16-bit unsigned int leaving
// two bits in the bottom ( via a 2-bit shift, << 2) for the least sig
// bits:
// 	0x0000 | nunchuck_buff[*] << 2
// Add to the above, the least sig bits. The code for x:
// 	nunchuck_buf[5] & B00001100
// for example selects the 3rd and 4th bits from the 6th byte of the
// payload to be concatinated with nunchuck_buff[2] to complete the 10-
// bit datum for a given axis.
//
static inline uint16_t nunchuck_accelx()
	{
	return (  0x0000 | ( nunchuck_buf[2] << 2 ) +
		( ( nunchuck_buf[5] & B00001100 ) >> 2 )  );
	}

static inline uint16_t nunchuck_accely()
	{
	return (  0x0000 ^ ( nunchuck_buf[3] << 2 ) +
		( ( nunchuck_buf[5] & B00110000 ) >> 4 )  );
	}

static inline uint16_t nunchuck_accelz()
	{
	return (  0x0000 ^ ( nunchuck_buf[4] << 2 ) +
		( ( nunchuck_buf[5] & B11000000 ) >> 6 )  );
	}

//
//
// Returns the x,y and z accelerometer values with calibration values
// subtracted.
//
static inline int nunchuck_caccelx()
	{
		return (int)(nunchuck_accelx() - accel_zerox);
	}

static inline int nunchuck_caccely()
	{
		return (int)(nunchuck_accely() - accel_zeroy);
	}

static inline int nunchuck_caccelz()
	{
		return (int)(nunchuck_accelz() - accel_zeroz);
	}

//
//
// Returns joystick angle in degrees. It uses the ratio of calibrated
// x and y potentiometer readings to find the angle, zero being direct
// right (positive x) and measured counter-clockwise from there.
//
// If the atan2 function returns a negative angle, it is rotated back
// into a positive angle. For those unfamiliar, the atan2 function
// is a more inteligent atan function which quadrant the vector <x,y>
// is in, and returns the appropriate angle.
//
static inline int nunchuck_joyangle()
	{
	double theta;
	theta = atan2( nunchuck_cjoy_y(), nunchuck_cjoy_x() );
	while (theta < 0) theta += 2*M_PI;
	return (int)(theta * 180/M_PI);
	}

//
//
// Returns roll angle in degrees. Under the assumption that the
// only acceleration detected by the accelerometer is acceleration due
// to gravity, this function uses the ratio of the x and z
// accelerometer readings to gauge pitch. This only works while the
// nunchuck is being held still or at constant velocity with zero ext-
// ernal force.
//
static inline int nunchuck_rollangle()
	{
	return (int) (  atan2( (double) nunchuck_caccelx(),
		(double) nunchuck_caccelz() ) * 180 / M_PI  );
	}

//
//
// Returns pitch angle in degrees. Under the assumption that the
// only acceleration detected by the accelerometer is acceleration due
// to gravity, this function uses the ratio of the y and z
// accelerometer readings to gauge pitch.  This only works while the
// nunchuck is being held still or at constant velocity with zero ext-
// ernal force.
//
static inline int nunchuck_pitchangle()
	{
	return (int) (  atan2( (double) nunchuck_caccely(),
		(double)nunchuck_caccelz() ) * 180 / M_PI  );
	}

//
//
// Because gravity pulls down on the z-accelerometer while the nunchuck
// is upright, we need to calibrate {x,y} and {z} separately. Execute
// this function while the nunchuck is known to be upright and then 
// execute nunchuck_calibrate_accelz() when the nunchuck is on its side.
//
void nunchuck_calibrate_accelxy()
	{
	accel_zerox = nunchuck_accelx();
	accel_zeroy = nunchuck_accely();
	}

//
//
// See documentation for nunchuck_calibrate_xy()
//
void nunchuck_calibrate_accelz()
	{
	accel_zeroz = nunchuck_accelz();
	}
//
//
// EOF
