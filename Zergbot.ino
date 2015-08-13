

// The Wiring of Nunchuck 
//-------------------------------------------------
/*
 * Wiring Details
 * white  - Analog 2 - Ground
 * red    - Analog 3 - Power
 * green  - data   - Analog 4
 * yellow - clock  - Analog 5
 */

// The Wiring of Motor Driver
//-------------------------------------------------
/*
 * Wiring Details
 * INA - Digital 2
 * INB - Digital 3
 * ENA - Digital 9
 * INC - Digital 4
 * IND - Digital 5
 * ENB - Digital 10
 * VCC - (+) Power from battery bank
 * GND - (-) Power from battery bank
 * +5 - VCC on Arduino
 */


#define INA 2
#define INB 3
#define ENA 9
#define INC 4
#define IND 5
#define ENB 10
#define DEFAULT_STICK_ZERO 128
#define DEBUGGING

#include <Wire.h>
#include "wiinunchuck.h"
#include "Motors.h"



int loop_cnt=0;
byte joyx,joyy,accx, accy, accz, zbut,cbut;

void setup(){
  Serial.begin(19200);   //Starts the serial port (used for debuging however makes servos jumpy)
  nunchuck_setpowerpins(); // set the Arduino power pins for Nunchuck
  nunchuck_init ();     // send the nunchuck initilization handshake
  initMots(); // set the Arduino pins for Motor driver

}

void loop(){

nunchuck_get_data();


if( loop_cnt > 100 ) { // every 100 msecs get new data
  loop_cnt = 0;


  joyx  = nunchuck_joy_x(); 
  joyy  = nunchuck_joy_y(); 
  accx = nunchuck_accelx(); // ranges from approx 70 - 182
  accy = nunchuck_accely();// ranges from approx 65 - 173
  accz = nunchuck_accelz();// ranges from approx 65 - 173
  zbut = nunchuck_zbutton();
  cbut = nunchuck_cbutton(); 

robotOperation(joyx, joyy, accx, accy, accz, zbut, cbut);
/*
#ifdef DEBUGGING            
        Serial.print("joyx: "); Serial.print((byte)joyx,DEC);
        Serial.print("\tjoyy: "); Serial.print((byte)joyy,DEC);
        Serial.print("\taccx: "); Serial.print((byte)accx,DEC);
        Serial.print("\taccy: "); Serial.print((byte)accy,DEC);
        Serial.print("\taccz: "); Serial.print((byte)accz,DEC);
        Serial.print("\tzbut: "); Serial.print((byte)zbut,DEC);
        Serial.print("\tcbut: "); Serial.println((byte)cbut,DEC);
#endif
*/
            }
    loop_cnt++;
    delay(1);
}

