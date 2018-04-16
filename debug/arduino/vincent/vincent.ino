#include <math.h>
#include <serialize.h>
//#include <buffer.h>

#include "packet.h"
#include "constants.h"

/*
CHECK
AND
COMBINE ALL THE DDRX (B,C,D)
TYPE "bare metal start" IN CTRL + F FOR EASY ACCESS TO ALL THE ADDED BARE METAL CODES
UPDATE THE VINCENT LENGTH AND BREADTH ONCE FINAL CONFIGURATION OF VINCENT IS DONE
TO SET 1, use |=
TO SET 0, use &=
 
*/
// Data Type for Serial Communication
//static TBuffer _recvBuffer, _xmitBuffer;

#define XMIT_SIZE      128
#define RECV_SIZE      128

//////////////////////
//Ultrasonic sensors
//trigPin: grey jumper
//echoPin: white jumpemr 
//////////////////////
int trigPinU = 11;
int echoPinU = 8;
int duration;
volatile unsigned long ultraInCm;
//////////////////////
//////////////////////
//IR sensors
//////////////////////
#define leftIR A4
#define rightIR A5

volatile unsigned long rightIRreading;
volatile unsigned long leftIRreading;
/////////////////////

typedef enum {
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;


/////////////////
// BUZZER
/////////////////
#define BUZZER              7
// Buzzer tones
#define  Cused              956       // 1046.5 Hz C5
#define  Gsharp             1204      // 830.61 Hz G4#
#define  Asharp             1073      // 932.33 Hz A4#
#define  R                  0         // rest 0 Hz
// Array of notes to be played and the duration to be played
int melody[] = {Cused, R, Cused, R, Cused, R, Cused, Gsharp, Asharp, Cused, R, Asharp, Cused, R};
int beats[]  = {10, 5, 10, 5, 10, 5, 45, 45, 45, 20, 5, 15, 45, 10};
int MAX_COUNT = sizeof(melody)/2;
// Set overall tempo
long tempo = 10000;
// Set length of pause between notes
int pause = 1000;
// Loop variable to increase Rest length
int rest_count = 100; //<-BLETCHEROUS HACK; See NOTES
// Initialize core variables
int tone_ = 0;
int beat = 0;
long tone_duration  = 0;

/*
 * Vincent's configuration constants
 */

// Number of ticks per revolution from the
// wheel encoder.
#define ADJUSTMENT_PWM_FWD      25
#define ADJUSTMENT_PWM_REV      22
#define COUNTS_PER_REV          192
//#define COUNTS_PER_REV_RIGHT      266


// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42

// Motor control pins. You need to adjust these till
// Vincent moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  9  // Right forward pin
#define RR                  10  // Right reverse pin

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


/*
 *
 * Vincent Communication Routines.
 *
 */

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".
    
  char buffer[PACKET_SIZE];
  int len;
    
  len = readSerial(buffer);
    
  if(len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  
  TPacket statusPacket;
  statusPacket.packetType=PACKET_TYPE_RESPONSE;
  statusPacket.command=RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurn;
  statusPacket.params[5] = rightForwardTicksTurn;
  statusPacket.params[6] = leftReverseTicksTurn;
  statusPacket.params[7] = rightReverseTicksTurn;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  /////all the sensors' readings/////
  statusPacket.params[10] = ultraInCm; //ultrasound sensor
  statusPacket.params[11] = leftIRreading;
  statusPacket.params[12] = rightIRreading;

  
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
    
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);  
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
    
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.
    
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;
    
  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


////////////////////////////////////////////////////
void sendDone(){
  TPacket donePacket;
  donePacket.packetType = PACKET_TYPE_RESPONSE;
  donePacket.command = RESP_DONE;
  sendResponse (&donePacket);
}
////////////////////////////////////////////////////

/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011; // Set port 2 and 3 as input
  PORTD |= 0b00001100;    // Set port 2 and 3 as HIGH output  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long)((float)leftForwardTicks/COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long)((float)leftReverseTicks/COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      leftReverseTicksTurn++;
      break;
    case RIGHT:
      leftForwardTicksTurn++;
      break;
  }

  //Serial.println(leftForwardTicks);
  
  //leftTicks++;
  //leftRevs = leftTicks / COUNTS_PER_REV;
  //Serial.print("LEFT: ");
  //Serial.println(leftRevs);
}

void rightISR() {
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
      break;
    case LEFT:
      rightForwardTicksTurn++;
      break;
    case RIGHT:
      rightReverseTicksTurn++;
      break;
  }

  //Serial.println(rightForwardTicks);
  //rightTicks++;
  //rightRevs = rightTicks / COUNTS_PER_REV
  //Serial.print("RIGHT: ");
  //Serial.println(rightRevs);
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EIMSK = 0b00000011;  // Activate INT0 and INT1
  EICRA = 0b00001010; // set INT0 and INT1 to trigger interrupt on falling edge
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}
// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 *
 */
 

// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  Serial.begin(9600);

  //USART to work at 9600 in 8N1 format
  //UCSR0C = 0b00000110; // Set USART to Asynchronous mode, N partity mode, 1 stop bit and 8-bit character size

  // Set baud rate to 9600
  //UBRR0L = 103;  //CHANGES 12042018
  //UBRR0H = 0;     //CHANGES 12042018
    
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on. 
  
  // Start the serial port
  // Enable RXC and UDRIE0
  // Enable USART receiver and transmitter
  //UCSR0B = 0b10111000; 
}

/*
void setupBuffers()
{
  // Initialize the receive and transmit buffers.
  initBuffer(&_recvBuffer, RECV_SIZE);
  initBuffer(&_xmitBuffer, XMIT_SIZE);
}
ISR(USART_RX_vect) {
  // Write received data
  unsigned char data = UDR0;
  writeBuffer(&_recvBuffer, data);
}
ISR(USART_UDRE_vect) {
  unsigned char data;
  TBufferResult result = readBuffer(&_xmitBuffer, &data);
  if(result == BUFFER_OK)
    UDR0 = data;
  else 
    if (result == BUFFER_EMPTY)
      UCSR0B &= 0b11011111;
}
*/

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

  
  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
  
  /*
  int count = 0;
  TBufferResult result;
  do {
    result = readBuffer(&_recvBuffer, &buffer[count]);
    if (result == BUFFER_OK)
      count++;
  } while (result == BUFFER_OK);
  return count;
  */
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);

  /*
  TBufferResult result = BUFFER_OK;
  int i;
  for (int i=1; i<len && result == BUFFER_OK; i++)
    result = writeBuffer(&_xmitBuffer, buffer[i]);
  UDR0 = buffer[0];
  UCSR0B |= 0b00100000;
  */
}

/*
 * Vincent's motor drivers.
 *
 */

// Set up Vincent's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
int pwm_speed_LF =0 , pwm_speed_LR =0, pwm_speed_RF =0, pwm_speed_RR =0; //declare 4 global variables to change the pwm values in the ISRs
void setupMotors() {
  
    /*    Our motor set up is:
     *    A1IN - Pin 5, PD5, OC0B (Left reverse pin, LR)
     *    A2IN - Pin 6, PD6, OC0A (Left foward pin, LF)
     *    B1IN - Pin 10, PB2, OC1B (Right reverse pin, RR)
     *    B2In - pIN 9, PB1, OC1A (Right forward pin, RF)
     */

  DDRD |= 0b01100000; // Pin 5 and 6 for left motor
  DDRB |= 0b00000110; // Pin 9 and 10 for right motor

  // Set up TCN0 for left motor
  TCNT0 = 0;
  OCR0A = 0;
  OCR0B = 0;
  TIMSK0 |= 0b110;

  // Set up TCN1 for right motor
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  TIMSK1 |= 0b110;

}

// Start the PWM for Vincent's motors.
// We will implement this later. For now it is
// blank.
void startMotors() {
  // Set TCN0 and TCN1 to prescalar value of 64
  
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
}


void setupSensors() {
  /* CHECK EVERYTHING PLS
   * int trigPinU = 11;    // Arduino Pin 11 = PB3
   * int echoPinU = 8;    // Arduino Pin 8 = PB0
   * #define leftIR A4    // Arduino Analog Pin 4 (ADC4) = PC4    //check from Week 4 Studio, GPIO Programming Slides
   * #define rightIR A5   // Arduino Analog Pin 5 (ADC5) = PC5
   */
  
  //DDRB |= 00001000;   //trigPinU (PB3) set to OUTPUT (1), 
  //DDRB &= 11111110;   //echoPinU (PB0) set to INPUT (0)
  DDRC &= 11001111;   //PC4 and PC5 both set to INPUT (0)

  
  pinMode(trigPinU, OUTPUT);
  pinMode(echoPinU, INPUT);
  //pinMode(leftIR, INPUT);
  //pinMode(rightIR, INPUT);
  
}

void startSensors() {
  // Ultrasound
  digitalWrite(trigPinU, LOW);
  //PORTB &= 11110111;    //digitalWrite(trigPinU, LOW),  trigPinU = PB3, set to 0
  delayMicroseconds(5);
  
  digitalWrite(trigPinU, HIGH);
  //PORTB |= 00001000;    //digitalWrite(trigPinU, HIGH), trigPinU = PB3, set to 1
  delayMicroseconds(10);
    
  duration = pulseIn(echoPinU, HIGH, 4000);
  ultraInCm = (duration/2) / 29.1;

  //IR Sensors
  //its either CLEAR or TOO NEAR
  //rightIRreading = digitalRead(rightIR);
  rightIRreading = (PINC & 0b00010000) ? 1 : 0;  //rightIRreading = digitalRead(rightIR), Arduino Analog Pin 4 (ADC4) = PC4

  //leftIRreading = digitalRead(leftIR);
  leftIRreading =  (PINC & 0b00100000) ? 1 : 0;  //leftIRreading = digitalRead(leftIR), Arduino Analog Pin 5 (ADC5) = PC5
}
  

// Convert percentages to PWM values
int pwmVal(float speed) {
  if(speed < 0.0)
    speed = 0;
    
  if(speed > 100.0)
    speed = 100.0;
    
  return (int) ((speed / 100.0) * 255.0);
}

  
ISR(TIMER0_COMPA_vect) {
  OCR0A = pwm_speed_LF;     //pin 6, PD6, OC0A, left forward
}

ISR(TIMER0_COMPB_vect) { 
  OCR0B = pwm_speed_LR;     //pin 5, PD5, OC0B, left reverse
}

ISR(TIMER1_COMPA_vect){
  OCR1A = pwm_speed_RF;      //pin 9, PB1, OC2A, right forward
}

ISR(TIMER1_COMPB_vect){      //pin 10, PB2, OC1B, right reverse
  OCR1B = pwm_speed_RR;      //originally was OCR2B since in Appendix C was using pin PD3 (OC2B)
}


void right_motor_forward(void) {
  // Using OCR1A counter
  TCCR1A = 0b10000001;
  PORTB &= 0b11111101;
}

void right_motor_reverse(void) {
  // Using OCR1B counter
  TCCR1A = 0b00100001;
  PORTB &= 0b11111011;
}

void left_motor_forward(void) {
  // Using OCR0A counter
  TCCR0A = 0b10000001;
  PORTD &= 0b11011111;
}
  
void left_motor_reverse(void) {
  // Using OCR0B counter
  TCCR0A = 0b00100001;
  PORTD &= 0b10111111;
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
  
  left_motor_forward();
  right_motor_forward();
    
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
    
  left_motor_reverse();
  right_motor_reverse();
  
  //BARE METAL END
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

  pwm_speed_RF = val - ADJUSTMENT_PWM_FWD;
  pwm_speed_LR = val;
  
  left_motor_reverse();
  right_motor_forward();
  
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
  pwm_speed_RR = val - ADJUSTMENT_PWM_REV;
    
  left_motor_forward();
  right_motor_reverse();
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

//stop vincent and play the sound
void mark_location(){
  dir = STOP;
  pwm_speed_LF = 0;
  pwm_speed_LR = 0;
  pwm_speed_RF = 0;
  pwm_speed_RR = 0;
  
  //digitalWrite(LED_BUILTIN, HIGH);
  //delay(500);
  //digitalWrite(LED_BUILTIN, LOW);

  for (int i=0; i<MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];

    tone_duration = beat * tempo; // Set up timing

    playTone(); 
    // A pause between notes...
    delayMicroseconds(pause);
  }
}

void setupBuzzer(){
  //pinMode(BUZZER,OUTPUT);    //pin 7 for buzzer, PD7
  DDRD |= 10000000;        //pin 7, PD7 set to OUTPUT (1)
}

void playTone() {
  long elapsed_time = 0;
  if (tone_ > 0) { // if this isn't a Rest beat, while the tone has 
    //  played less long than 'tone_duration', pulse Buzzer HIGH and LOW
    while (elapsed_time < tone_duration) {
     
      PORTD |= 10000000; //digitalWrite(BUZZER,HIGH);   BUZZER = pin 7, PD7, set to 1
      delayMicroseconds(tone_ / 2);
      
      // DOWN
      PORTD &= 01111111; //digitalWrite(BUZZER, LOW);   BUZZER = pin 7, PD7, set to 0
      delayMicroseconds(tone_ / 2);

      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    } 
  }
  else { // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++) { // See NOTE on rest_count
      delayMicroseconds(tone_duration);  
    }                                
  }                                 
}

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

// Clears one particular counter
void clearOneCounter(int which) {
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState() {
  clearCounters();
}

void handleCommand(TPacket *command) {
  switch(command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_STOP:
      stop();
      sendDone(); //edited
      break;
    
    case COMMAND_GET_STATS:
      sendStatus();
      sendDone(); //edited
      break;
    
    case COMMAND_CLEAR_STATS:
      //clearOneCounter(command->params[0]);
      clearCounters();
      sendOK();
      sendDone(); //edited
      break;

    ///////////////////////////////////////////////////////////////////////////////////////
    case COMMAND_MARK:
      sendOK();
      mark_location(); //play the buzzer in this function //to be implemented
      sendDone(); //edited
      break;
    //////////////////////////////////////////////////////////////////////////////////////
    
      
    default:
      sendBadCommand();
      sendDone();
    }
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
  //setupBuffers();
  startSerial();
  setupBuzzer();
  setupMotors();
  startMotors();
  setupSensors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet) {
  switch(packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;
      
    case PACKET_TYPE_RESPONSE:
      break;
            
    case PACKET_TYPE_ERROR:
      break;
            
    case PACKET_TYPE_MESSAGE:
      break;
            
    case PACKET_TYPE_HELLO:
      break;
  }
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


