#include <math.h>
#include <serialize.h>
#include <buffer.h>

#include "packet.h"
#include "constants.h"

typedef enum {
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;


/*
 * Vincent's configuration constants
 */
 
// Number of ticks per revolution from the
// wheel encoder.
#define COUNTS_PER_REV          192

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          20.42

// PI, for calculating turn circumference
//#define PI                  3.141592654

// Vincent's length and breadth in cm
#define VINCENT_LENGTH      17.20    //17.5 ++  (updated 09/04)
#define VINCENT_BREADTH     10.90    //12.7     (updated 09/04)

/*
 *    Vincent's State Variables
 */

// Store the ticks from Vincent's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurn;
volatile unsigned long rightForwardTicksTurn;
volatile unsigned long leftReverseTicksTurn;
volatile unsigned long rightReverseTicksTurn;

// Store the revolutions on Vincent's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

// Vincent's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change.
float vincentDiagonal = 0.0;

// Vincent's turning circumference, calculated once
float vincentCirc = 0.0;

// Variable for ultrasonic sensors
volatile unsigned long ultraInCm;

// Variables for IR sensors
volatile unsigned long rightIRreading;
volatile unsigned long leftIRreading;


/*
 * Vincent's setup and run codes
 *
 */

// Clears all our counters
void clearCounters() {
  leftForwardTicks=0;
  leftReverseTicks=0;
  rightForwardTicks=0;
  rightReverseTicks=0;
  
  leftForwardTicksTurn=0;
  leftReverseTicksTurn=0;
  rightForwardTicksTurn=0;
  rightReverseTicksTurn=0;
  
  //leftRevs=0;
  //rightRevs=0;
  forwardDist=0;
  reverseDist=0;
}


// Intialize Vincet's internal states
void initializeState() {
  clearCounters();
}


void waitForHello() {
  int exit=0;
    
  while(!exit) {
    TPacket hello;
    TResult result;
    
    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);
    
    if(result == PACKET_OK) { 
      if(hello.packetType == PACKET_TYPE_HELLO) {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else if(result == PACKET_BAD) {
      sendBadPacket();
    }
    else if(result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // exit!
}

void setup() {
  // put your setup code here, to run once:

  //Compute the diagonal
  vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) + (VINCENT_BREADTH * VINCENT_BREADTH));
  vincentCirc = PI * vincentDiagonal;
  
  cli();
  setupEINT();
  setupSerial();
  setupBuffers();
  startSerial();
  setupBuzzer();
  setupMotors();
  startMotors();
  setupSensors();
  enablePullups();
  initializeState();
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  startSensors(); // Initialize ultrasound and IR sensors
  
  TPacket recvPacket; // This holds commands from the Pi
    
  TResult result = readPacket(&recvPacket);
    
  if(result == PACKET_OK) {
      //Serial.println("PACKET OK!");
      handlePacket(&recvPacket);
  }
  else if(result == PACKET_BAD) {
      //Serial.println("PACKET BAD!");
      sendBadPacket();
  }
  else if(result == PACKET_CHECKSUM_BAD) {
      sendBadChecksum();
  }

  if(deltaDist > 0) {
    if(dir == FORWARD) {
      if(forwardDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
        sendDone();
      }
    }
    else if(dir == BACKWARD) {
      if(reverseDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
        sendDone();
      }
    }
    else if(dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
      sendDone();
    }
  }

  if(deltaTicks > 0) {
    if(dir == LEFT) {
      if(leftReverseTicksTurn >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop(); 
        sendDone();
      }
    }
    else if (dir == RIGHT) {
      if(rightReverseTicksTurn >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        sendDone();
      }
    }
    else if(dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
      sendDone();
    }
  }
}


