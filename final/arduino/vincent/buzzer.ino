/*
 * Vincent's buzzer configuration
 *
 */
 
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

void setupBuzzer(){
  //pinMode(BUZZER,OUTPUT);    //pin 7 for buzzer, PD7
  DDRD |= 10000000;        //pin 7, PD7 set to OUTPUT (1)
}

void startBuzzer() {
  // Initiate buzzer sequence
  for (int i=0; i<MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];

    tone_duration = beat * tempo; // Set up timing

    playTone(); 
    // A pause between notes...
    delayMicroseconds(pause);
  }
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
