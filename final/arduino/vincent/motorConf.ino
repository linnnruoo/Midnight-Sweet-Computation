/*
 * Vincent's motor drivers.
 *
 */

// Set up Vincent's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
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


void rightMotorForward(void) {
  // Using OCR1A counter
  TCCR1A = 0b10000001;
  PORTB &= 0b11111101;
}

void rightMotorReverse(void) {
  // Using OCR1B counter
  TCCR1A = 0b00100001;
  PORTB &= 0b11111011;
}

void leftMotorForward(void) {
  // Using OCR0A counter
  TCCR0A = 0b10000001;
  PORTD &= 0b11011111;
}
  
void leftMotorReverse(void) {
  // Using OCR0B counter
  TCCR0A = 0b00100001;
  PORTD &= 0b10111111;
}
