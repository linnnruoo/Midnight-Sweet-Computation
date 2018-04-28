// Start by defining the relationship between
//       note, period, &  frequency.
#define  c     3830        // 261 Hz
#define  d     3400        // 294 Hz 
#define  e     3038        // 329 Hz 
#define  f     2864        // 349 Hz 
#define  g     2550        // 392 Hz 
#define  a     2272        // 440 Hz 
#define  b     2028        // 493 Hz 
#define  C     1912        // 523 Hz 
#define  Cused   956       // 1046.5 Hz C5
#define  Gsharp  1204      // 830.61 Hz G4#
#define  Asharp  1073      // 932.33 Hz A4#
// Define a special note, 'R', to represent a rest
#define  R     0

int speakerOut = 9;
// Do we want debugging on serial out? 1 for yes, 0 for no
int DEBUG = 1;

void setup() { 
  pinMode(speakerOut, OUTPUT);
}

// MELODY and TIMING  =======================================
//  melody[] is an array of notes, accompanied by beats[], 
//  which sets each note's relative length (higher #, longer note) 
int melody[] = {Cused, R, Cused, R, Cused, R, Cused, Gsharp, Asharp, Cused, R, Asharp, Cused, R};
int beats[]  = {10, 5, 10, 5, 10, 5, 45, 45, 45, 20, 5, 15, 45, 10};
//int melody[] = {Cused, Cused, Cused, Cused, Gsharp, Asharp, Cused, Asharp, Cused};
//int beats[]  = {15, 15, 15, 45, 45, 45, 30, 15, 45};
int MAX_COUNT = sizeof(melody) / 2; // Melody length, for looping.

// Set overall tempo
long tempo = 10000;
// Set length of pause between notes
int pause = 1000;
// Loop variable to increase Rest length
int rest_count = 100; //<-BLETCHEROUS HACK; See NOTES

// Initialize core variables
int tone_ = 0;
int beat = 0;
long duration  = 0;

// PLAY TONE  ==============================================
// Pulse the speaker to play a tone for a particular duration
void playTone() {
  long elapsed_time = 0;
  if (tone_ > 0) { // if this isn't a Rest beat, while the tone has 
    //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < duration) {

      digitalWrite(speakerOut,HIGH);
      delayMicroseconds(tone_ / 2);

      // DOWN
      digitalWrite(speakerOut, LOW);
      delayMicroseconds(tone_ / 2);

      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    } 
  }
  else { // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++) { // See NOTE on rest_count
      delayMicroseconds(duration);  
    }                                
  }                                 
}

// LET THE WILD RUMPUS BEGIN =============================
void loop() {
  // Set up a counter to pull from melody[] and beats[]
  for (int i=0; i<MAX_COUNT; i++) {
    tone_ = melody[i];
    beat = beats[i];

    duration = beat * tempo; // Set up timing

    playTone(); 
    // A pause between notes...
    delayMicroseconds(pause);
  }
}
