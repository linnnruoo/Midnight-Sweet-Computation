volatile unsigned long ultraInCm;
volatile unsigned long rightIRreading;
volatile unsigned long leftIRreading;
int duration;

#define trigPinU 11
#define echoPinU 8
#define leftIR A4
#define rightIR A5

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
    
  duration = pulseIn(echoPinU, HIGH);
  unsigned long temp = (duration/2) / 29.1;
  ultraInCm = (temp > 300 || temp < 3) ? 0 : temp;


  //IR Sensors
  //its either CLEAR or TOO NEAR
  //rightIRreading = digitalRead(rightIR);
  rightIRreading = (PINC & 0b00010000) ? 1 : 0;  //rightIRreading = digitalRead(rightIR), Arduino Analog Pin 4 (ADC4) = PC4

  //leftIRreading = digitalRead(leftIR);
  leftIRreading =  (PINC & 0b00100000) ? 1 : 0;  //leftIRreading = digitalRead(leftIR), Arduino Analog Pin 5 (ADC5) = PC5
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupSensors();
}

void loop() {
  // put your main code here, to run repeatedly:
  startSensors();
  Serial.print("Ultrasonic Reading is: ");
  Serial.println(ultraInCm);
  Serial.print("\tRight IR Reading is :");
  Serial.println(rightIRreading);
  Serial.print("\tLeft IR Reading is :");
  Serial.println(leftIRreading);
  delay(1000);
}
