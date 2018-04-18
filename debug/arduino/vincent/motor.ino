/*
 * Vincent's motor movements.
 *
 */
// Motor control pins
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  9  // Right forward pin
#define RR                  10  // Right reverse pin

// Calibrated values for Vincent to move straight
#define ADJUSTMENT_PWM_FWD      7
#define ADJUSTMENT_PWM_REV      2

 
// Variables to hold PWM values
int pwm_speed_LF = 0;
int pwm_speed_LR = 0; 
int pwm_speed_RF = 0; 
int pwm_speed_RR = 0;

// Convert percentages to PWM values
int pwmVal(float speed) {
  if(speed < 0.0)
    speed = 0;
    
  if(speed > 100.0)
    speed = 100.0;
    
  return (int) ((speed / 100.0) * 255.0);
}

// Estimate number of wheel ticks needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
  // Assume that angular distance moved = linear distance moved in one wheel
  // revolution. This is probably incorrect but simplifies calculations
  // of wheel revs to make one full 360 turn via vincentCirc / WHEEL_CIRC
  // This is for 360 degrees. For any degrees it will be (ang * vincentCirc) / (360 * WHEEL_CIRC)
  // To convert to ticks, multiply by COUNTS_PER_REV

  unsigned long ticks = (unsigned long) ((ang * vincentCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}

// Move Vincent forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Vincent will
// continue moving forward indefinitely.
void forward(float dist, float speed) {
  dir = FORWARD;
  
  int val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 99999999;

  newDist = forwardDist + deltaDist;
  
  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  /*
  analogWrite(LF, val);
  analogWrite(RF, val - ADJUSTMENT_PWM_FWD);
  analogWrite(LR,0);
  analogWrite(RR, 0);
  */
  pwm_speed_LF = val;
  pwm_speed_RF = val - ADJUSTMENT_PWM_FWD;
  
  leftMotorForward();
  rightMotorForward();
    
}

// Reverse Vincent "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Vincent will
// continue reversing indefinitely.
void reverse(float dist, float speed) {
  dir = BACKWARD;
    
  int val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 99999999;

  newDist = reverseDist + deltaDist;
    
  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  /*
  analogWrite(LR, val);
  analogWrite(RR, val - ADJUSTMENT_PWM_REV);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  */
  pwm_speed_LR = val;
  pwm_speed_RR = val - ADJUSTMENT_PWM_REV;
    
  leftMotorReverse();
  rightMotorReverse();
  
  //BARE METAL END
}


// Turn Vincent left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn left indefinitely.
void left(float ang, float speed) {
  dir = LEFT;
    
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 9999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = leftReverseTicksTurn + deltaTicks;
    
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  /*
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
  */

  pwm_speed_RF = 0;
  pwm_speed_LR = val;
  
  leftMotorReverse();
  rightMotorForward();
  
  //BARE METAL END
}

// Turn Vincent right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn right indefinitely.
void right(float ang, float speed) {
  dir = RIGHT;
    
  int val = pwmVal(speed);

  if (ang == 0)
    deltaTicks = 9999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = rightReverseTicksTurn + deltaTicks;
    
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  /*
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  */
  pwm_speed_LF = val;
  pwm_speed_RR = 0;
    
  leftMotorForward();
  rightMotorReverse();
}

// Stop Vincent. To replace with bare-metal code later.
void stop() {
  dir = STOP;
  /*
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
  */
  pwm_speed_LF = 0;
  pwm_speed_LR = 0;
  pwm_speed_RF = 0;
  pwm_speed_RR = 0;
}
