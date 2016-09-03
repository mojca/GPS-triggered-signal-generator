/*
 * Drive a signal generator using GPS module
 * http://github.com/mojca/GPS-triggered-signal-generator
 *
 * This code is licenced under MIT/X11.
 *
 * Copyright (c) 2016 Mojca Miklavec
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* TODO: Take a look at the time library TimeLib.h:
 * - http://www.pjrc.com/teensy/td_libs_Time.html
 * - https://github.com/PaulStoffregen/Time
 *
 * #include <TimeLib.h>
 */

// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

const int pinInterrupt = 3;   // input:  PPS signal
const int pinLed = 13;        // output: for driving the coil
const int pinButtonStart = 5; // input:  red button to start and stop the coil
const int pinButtonIncr  = 6; // input:  green button to change the time period

int valButtonIncr   = 0;
int valButtonIncrX  = 0;

int startButtonState;                  // 0: OFF; 1: ON
unsigned long startButtonPressTimeMs;  // timestamp in ms when the red start/stop button was last pressed
int startButtonPressCounter;           // number of times the button was pressed

int periodButtonState;                 // 0: OFF; 1: ON
unsigned long periodButtonPressTimeMs; // timestamp in ms when the green period button was last pressed

String t_nema;                         // time read from the NEMA sentence
String t_true;                         // true time (nema + 1 second)

const int pulseLength    = 4;          // selectable pulse duration (via the green button press)
int pulseLengthCountdown = 0;


class Clock;
class RunCoil;

/*
 * Class for time-related functionality
 */
class Clock {
  private:
    void adjustHMS();
    unsigned long ms; // milliseconds since the beginning of the day
    byte hms[3];
    static const unsigned long secperunit =    1000;
    static const unsigned long minperunit =   60000;
    static const unsigned long hrsperunit = 3600000;
    const unsigned long hmsperunit[3] = {secperunit, minperunit, hrsperunit};
  public:
    Clock();
    Clock(byte h, byte m, byte s);
    Clock(String s);
    void set(String s);
    void set(Clock c);
    void set(byte h, byte m, byte s);
    void add(long millisecs);
    long secondsTo(Clock c) { return (c.ms - ms) / 1000; };
    byte hours()   { return hms[0]; };
    byte minutes() { return hms[1]; };
    byte seconds() { return hms[2]; };
    //byte milliseconds();
    String str();
    static const unsigned long second =    1000;
    static const unsigned long minute =   60000;
    static const unsigned long hour   = 3600000;
};

Clock tt_true(0,0,0);

/*
 * Class for switching the coil ON and OFF
 */
class RunCoil {
  private:
    int  lengthOfPulse; // duration of pulse in time
    int  count;
    bool isRequested;   // whether starting the coil was requested (before the full minute the coil remains OFF)
    bool isRunning;     // whether the coil is currently running (switching between ON and OFF)
    bool isHigh;        // whether the coil is currently switched ON
    bool isHighNext;    // whether the coil should be switched ON in the next step
    Clock t_start;      // the time when the coil was/will be switched ON
    String timer_str;

  public:
    RunCoil();          // constructor
    ~RunCoil() {};      // destructor

    void start();       // will set: isRequested=1 isRunning=0 isHigh=0
    void stop();        // will set: isRequested=0 isRunning=0 isHigh=0
    void pps();         // called for each PPS pulse

    void cyclePeriod(); // increase cycle period (1 -> 2 -> 4 -> ... -> 256 -> 1)

    String getStatus();     // what to print to the serial output (for debugging reasons only)
    String getStatusLCD1(); // what to print to the first line of the LCD screen
    String getStatusLCD2(); // what to print to the second line of the LCD screen

    bool isRequestedOrRunning() {
      return isRequested || isRunning;
    }
};

RunCoil::RunCoil() {
  lengthOfPulse = 4;
  count         = 0;
  isRequested   = false;
  isRunning     = false;
  isHigh        = false;
  isHighNext    = false;
}

/*
 * Send some debugging output to Serial (may be removed)
 */
String RunCoil::getStatus() {
  if (isRequested) {
    String s = "Requested, pulse length=";
    String q = String(lengthOfPulse);
    return (s+q);
  } else if(isRunning) {
    if (isHigh) {
      return "High";
    } else {
      return "Low";
    }
  } else {
    return "Stopped";
  }
}

/* 
 * First line on the LCD screen
 */
String RunCoil::getStatusLCD1() {
  String s = timer_str + String("   L=") + String(lengthOfPulse) + String("  ");
  return s;
}

/* 
 * Second line on the LCD screen
 */
String RunCoil::getStatusLCD2() {
  String r;
  if (isRunning || isRequested) {
    r = t_start.str();
  } else {
    r = String("--:--:--");
  }
  if (isRequested) {
    r += String(" waiting");
  } else if(isRunning) {
    if (isHigh) {
      r += String("|");
    } else {
      r += String(".");
    }
    r += String("running");
  } else {
    r += String(" stopped");
  }
  return String(r + String("                ")).substring(0,16);
}

/* 
 * Increase the cycling period by a factor of two (and return to 1)
 */
void RunCoil::cyclePeriod() {
  if (lengthOfPulse > 255) {
    lengthOfPulse = 1;
  } else {
    lengthOfPulse *= 2;
  }
}

/*
 * Prepare to start the coil once the start/stop button is pressed
 */
void RunCoil::start() {
  isRequested = true;
  isRunning   = false;
  isHigh      = false;
  isHighNext  = false;

  // set the coil starting time to current time, add one minute, set seconds to 0
  t_start.set(tt_true);
  t_start.add(Clock::minute);
  t_start.set(t_start.hours(), t_start.minutes(), byte(0));
  // and in case the button was pressed too late, add one extra minute
  if (tt_true.seconds() > 57)
    t_start.add(Clock::minute);
}

/*
 * immediately switch the coil off once the start/stop button is pressed
 */
void RunCoil::stop() {
  isRequested = false;
  isRunning   = false;
  isHigh      = false;
  isHighNext  = false;
}

/*
 * Process the PPS signal from the GPS unit
 */
void RunCoil::pps() {
  isHigh     = isHighNext;

  // the first time we run into one second before the full minute
  if (isRequested) {
    // detect if time is at one second before the starting time, then switch to running state
    if (tt_true.secondsTo(t_start) == 1) {
      isHighNext = true;
      Serial.println("next second is high");
    } else if (tt_true.secondsTo(t_start) == 0) {
      isRunning   = true;
      isRequested = false;
      if (lengthOfPulse == 1) {
        isHighNext = false;
      }
    }
  } else if(isRunning) {
    // TODO: only if we didn't reach midnight yet (this needs a fix)
    if (t_start.secondsTo(tt_true) >= 0) {
      // count the seconds since starting the coil
      unsigned long secsSinceStart = t_start.secondsTo(tt_true) + 1;
      if (secsSinceStart % (lengthOfPulse*2) < lengthOfPulse) {
        isHighNext = true;
      } else {
        isHighNext = false;
      }
    } else {
      Serial.println("Counter cycle, switching the coil OFF.");
      // TODO: after midnight
      isRunning   = false;
      isRequested = false;
      //isHigh      = false;
      isHighNext  = false;
    }
  } else {
    isHighNext = false;
  }

  digitalWrite(pinLed, isHighNext);
  Serial.print("\tCoil will next be ");
  Serial.println(isHighNext);
  timer_str = tt_true.str();
}

Clock::Clock() {
  ms = 0;
  hms[0] = 0; hms[1] = 0; hms[2] = 0;
}

void Clock::set(byte h, byte m, byte s) {
  hms[0] = h;
  hms[1] = m;
  hms[2] = s;
  ms = h*hrsperunit + m*minperunit + s*secperunit;
}


Clock::Clock(byte h, byte m, byte s) {
  set(h, m, s);
}

Clock::Clock(String t) {
  int i;
  set(t);
}

void Clock::set(String t) {
  int i;
  for(i=0; i<3; i++) {
    hms[i] = t.substring(2*i,2*(i+1)).toInt();
  }
  ms = hms[0]*hrsperunit + hms[1]*minperunit + hms[2]*secperunit;
}

void Clock::set(Clock c) {
  hms[0] = c.hours();
  hms[1] = c.minutes();
  hms[2] = c.seconds();
  ms = hms[0]*hrsperunit + hms[1]*minperunit + hms[2]*secperunit;
}

void Clock::adjustHMS() {
  unsigned long t = ms/1000;
  hms[2] = t%60;
  t = t/60;
  hms[1] = t%60;
  t = t/60;
  hms[0] = t%24;
}

void Clock::add(long millisecs) {
  ms = ms + millisecs;
  adjustHMS();
}

String Clock::str() {
  int i;
  String r = "";
  for(i=0; i<3; i++) {
    if(hms[i] < 10)
      r += "0";
    r += hms[i];
    if(i<2)
      r += ":";
  }
  return r;
}

RunCoil coil;


void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);

  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);
  attachInterrupt(digitalPinToInterrupt(pinInterrupt), event_pps, RISING);
  // initialize the two buttons
  pinMode(pinButtonStart, INPUT);
  pinMode(pinButtonIncr,  INPUT);

  startButtonState        = 0;
  startButtonPressTimeMs  = 0;
  startButtonPressCounter = 0;

  periodButtonState       = 0;
  periodButtonPressTimeMs = 0;
}

/*
 * main loop: just reading the buttons and printing to the LCD screen
 *
 * TODO:
 * - check the time since the last PPS pulse and switch the coil off in case of lost signal
 * - also switch the coil off in case of lost NEMA sentence providing the time
 */
void loop() {
  // Simple de-bounce code. Transitions from OFF to ON are well-defined and non-oscillatory.
  // De-bounce time is 100ms.
  
  int valButtonStart = digitalRead(pinButtonStart);
  unsigned long timeSinceLastPress = millis() - startButtonPressTimeMs;
  
  if ((startButtonState == 0) && (valButtonStart == HIGH)) {
    // go to state 1 and save timestamp
    startButtonState = 1;
    startButtonPressTimeMs = millis();
    startButtonPressCounter++;
    // start/stop me...

    if (coil.isRequestedOrRunning()) {
      // already running, stop it
      coil.stop();
      Serial.println("Coil stopped.");
    } else {
      coil.start();
      Serial.println("Coil started.");
    }    
  } else if ((startButtonState == 1) && (timeSinceLastPress > 200) && (valButtonStart == LOW)) {
    // go back to state 0
    startButtonState = 0;
    // do not stop me...
  }

  // only allow changing the time period when the coil is not running
  if(!coil.isRequestedOrRunning()) {
    // Period button
    int valButtonIncr = digitalRead(pinButtonIncr);
    timeSinceLastPress = millis() - periodButtonPressTimeMs;

    if ((periodButtonState == 0) && (valButtonIncr == HIGH)) {
      // go to state 1 and save timestamp
      periodButtonState = 1;
      periodButtonPressTimeMs = millis();
      // start/stop me...
      coil.cyclePeriod();
    } else if ((periodButtonState == 1) && (timeSinceLastPress > 200) && (valButtonIncr == LOW)) {
      // go back to state 0
      periodButtonState = 0;
      // do not stop me...
    }
  }

  // print on LCD
  lcd.setCursor(0,0);
  lcd.print(coil.getStatusLCD1());
  lcd.setCursor(0,1);
  lcd.print(coil.getStatusLCD2());
}

/*
 * Process the pulse-per-second signal
 */
void event_pps() {
  coil.pps();

  // debugging output only
  Serial.print(tt_true.str()); // the Clock instance converted to string
  Serial.print(" ");
  Serial.print(coil.getStatus());
  Serial.println();
}

/*
 * Parse the NEMA sentences received from the GPS
 */
void serialEvent() {
  String s;
  while (Serial.available()) {
    /*
     * Parse the NEMA sentence GPGGA (it would also be possible to parse GPRMC)
     * 
     * $GPGGA,184418.000,4542.9792,N,01353.1328,E,1,04,2.01,327.7,M,45.1,M,,*64
     * $GPRMC,184418.000,A,4542.9792,N,01353.1328,E,0.32,170.36,030216,,,A*64
     */
    s = Serial.readStringUntil('\n');
    if (s.substring(1,6) == "GPGGA") {
      t_nema = s.substring(7,13);
      tt_true.set(t_nema);
      tt_true.add(Clock::second);
    }
  }
}

