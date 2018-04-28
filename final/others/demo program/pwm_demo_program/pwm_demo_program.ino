int val = 200;

void InitPWM()
{
  TCNT0 = 0;
  OCR0A = 0;
  OCR0B = 0;
  TIMSK0 |= 0b110;

  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  TIMSK1 |= 0b110;

  //TCNT2 = 0;
  //OCR2A = 0;
  //OCR2B = 0;
  //TIMSK2 |= 0b010;
}

void startPWM()
{
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
  //TCCR2B = 0b00000011;
  sei();
}

ISR(TIMER0_COMPA_vect)
{
  OCR0A = val;
}
ISR(TIMER0_COMPB_vect)
{
  OCR0B = val;
}
ISR(TIMER1_COMPA_vect)
{
  OCR1A = val;
}
ISR(TIMER1_COMPB_vect)
{
  OCR1B = val;
}

void left_motor_forward(void)  // left forward
{
  TCCR0A = 0b10000001;
  PORTD &= 0b11011111;
}
void left_motor_reverse(void)  // left reverse
{
  TCCR0A = 0b00100001;
  PORTD &= 0b10111111;
}
void right_motor_forward(void) // right forward
{
  TCCR1A = 0b10000001;
  PORTB &= 0b11111101;
}

void right_motor_reverse(void) // right reverse
{
  TCCR1A = 0b00100001;
  PORTB &= 0b11111011;
}

void setup() {
  // put your setup code here, to run once:
  DDRD |= 0b01100000;
  DDRB |= 0b00000110;
  InitPWM();
  startPWM();
}


void loop() {
  // put your main code here, to run repeatedly:
    //right_motor_reverse();
    left_motor_forward();
    _delay_ms(1000);
    //right_motor_forward();
    left_motor_reverse();
    _delay_ms(1000);
    val = 0;
    
}
