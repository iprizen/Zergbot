
//------------------------------------------------------------------------
//START OF ARDUINO CONTROLLED ROBOT (Motors) ROUTINES

/*
 * sets up your arduino to address your SERB using the included routines
*/
struct vectorMove {
  int Speed;
  short Direction;
};

void initMots(){
// Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.
//Define L298N Dual H-Bridge Motor Controller Pins

pinMode(INA,OUTPUT); //Left Motor
pinMode(INB,OUTPUT); // Left Motor
pinMode(ENA,OUTPUT);
pinMode(INC,OUTPUT); // Right Motor
pinMode(IND,OUTPUT); // Right Motor
pinMode(ENB,OUTPUT);

/*
#ifdef DEBUGGING
  Serial.print(millis());
  Serial.print(" : Initialize Motors - "); Serial.println(100, DEC);
   analogWrite(ENA, 100);
 digitalWrite(INA, LOW);
 digitalWrite(INB, HIGH);
 analogWrite(ENB, 100);
 digitalWrite(INC, LOW);
 digitalWrite(IND, HIGH);
#endif  
*/
}


/*
 * sets the speed of the robot between 6-(reversed) and 255-(full speed). 128 is stop
 * NOTE: speed will not change the current speed you must change speed 
 * then call one of the go methods before changes occur.
 * it will return the absolute speed value between 1 - 255 and the direction (1 - forward or right, -1 - backward or left, 0 - stop)
 * Return the speed between 1 and 255
*/ 
struct vectorMove setMoveMotor(int newSpeed){
  short newDirection = 0;
  struct vectorMove newVector ;
  
  if (newSpeed > DEFAULT_STICK_ZERO){
    newVector.Speed = newSpeed - DEFAULT_STICK_ZERO;
    newVector.Direction = 1;
  }
  else if (newSpeed < DEFAULT_STICK_ZERO){
    newVector.Speed = DEFAULT_STICK_ZERO - newSpeed;
    newVector.Direction = -1;
  }
  else if (newSpeed == DEFAULT_STICK_ZERO){
    newVector.Speed = newSpeed;
    newVector.Direction = 0;
  }
  else newVector.Speed = -1;
    
    return newVector;
}

/*
 * sends the robot forwards
 */
void moveForward(int newSpeed){
 analogWrite(ENA, newSpeed);
 digitalWrite(INA, LOW);
 digitalWrite(INB, HIGH);
 analogWrite(ENB, newSpeed);
 digitalWrite(INC, LOW);
 digitalWrite(IND, HIGH);
    
#ifdef DEBUGGING
  Serial.print(millis());
  Serial.print(" : Move Forward - "); Serial.println(newSpeed, DEC);
#endif
 
}

/*
 * sends the robot backwards
 */
void moveBackward(int newSpeed){
  analogWrite(ENA, newSpeed);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
  analogWrite(ENB, newSpeed);
  digitalWrite(INC, HIGH);
  digitalWrite(IND, LOW);

#ifdef DEBUGGING
  Serial.print(millis());
  Serial.print(" : Move Backward - "); Serial.println(newSpeed, DEC);
#endif  
}
  
/*
 * sends the robot right
 */
void moveRight(int newSpeed){
  analogWrite(ENA, newSpeed);
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);

#ifdef DEBUGGING
  Serial.print(millis());
  Serial.print(" : Move Right - "); Serial.println(newSpeed, DEC);
#endif  
}

/*
 * sends the robot left
 */
void moveLeft(int newSpeed){
  analogWrite(ENB, newSpeed);
  digitalWrite(INC, LOW);
  digitalWrite(IND, HIGH);

#ifdef DEBUGGING
  Serial.print(millis());
  Serial.print(" : Move Left - "); Serial.println(newSpeed, DEC);
#endif  
}

/*
 * stops the robot
 */
void Stop(){
  analogWrite(ENA, 0);
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
  analogWrite(ENB, 0);
  digitalWrite(INC, LOW);
  digitalWrite(IND, HIGH);

#ifdef DEBUGGING
  Serial.print(millis());
  Serial.println(" : Stop!");
#endif  
} 


void moveDifferential(byte value_x, byte value_y){
  struct vectorMove tempMove;

  tempMove = setMoveMotor(value_y);

  if (tempMove.Direction == 1){
    moveForward(tempMove.Speed);
  } 
  else if (tempMove.Direction == -1){
    moveBackward(tempMove.Speed);
  }
  
  tempMove = setMoveMotor(value_x);

  if (tempMove.Direction == 1){
    moveRight(tempMove.Speed);
  }
  else if (tempMove.Direction == -1){
    moveLeft(tempMove.Speed);
  }

 if (value_x == DEFAULT_STICK_ZERO && value_y == DEFAULT_STICK_ZERO){
  Stop();
 }
 
}

void robotOperation(byte joyx,byte joyy,byte accx,byte accy,byte accz, byte zbut,byte cbut){
  if (!cbut){
    #ifdef DEBUGGING
      Serial.print(millis());
      Serial.println("robotOperation joystick called");
    #endif
    
    moveDifferential(joyx, joyy);
  }
  else
  {
    #ifdef DEBUGGING
      Serial.print(millis());
      Serial.println("robotOperation accellerometer called");
    #endif
    
    moveDifferential(accx, accy);
  }
}


//END OF ARDUINO CONTROLLED SERVO ROBOT (SERB) ROUTINES
//---------------------------------------------------------------------------

/*
void moveWiiAcelerometer(){
 moveDifferential(getYGs() * (float)100,getXGs()*(float)100); 
}

void moveWiiJoystick(){
 moveDifferential(map(getNunValue(YSTICK),30,220,-100,100),map(getNunValue(XSTICK),30,220,-100,100));
 }

//Takes in a speed and a direction input (like a joystick) and translates it to speed commands 
void moveDifferential(int speed1, int direction1){
  speed1 = deadBandFilter(speed1);
  direction1 = deadBandFilter(direction1);
  setSpeedLeft(speed1 + direction1);
  setSpeedRight(speed1 - direction1);
}

*/
