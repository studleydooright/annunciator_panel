#include <TimeLib.h>
#define FASTLED_INTERNAL
#include <FastLED.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

/*
  Annunciator Panel SketchThis program will light up the Annunciator Panel based on curent status of the Aircraft.The circuit:
  Momentary switch attached from pin 4 to ground (Nose Gear)
  Momentary switch attached from pin 6 to ground (Canopy); 0 is CLOSED (LOW), 1 is OPEN (HIGH)
  Momentary switch attached from pin 7 to ground (Landing Brake); 0 is OPEN (LOW), 1 is CLOSED (HIGH)
  Momentary switch attached from pin 5 to ground (alarm silence)
  sounder attached from pin 3 to ground
  Potentiometer connected to A0 (Throttle position)
  Built-in/External LED on pin 13 (green Gear Up)
  External LED on pin 12 (green Canopy open)
  External LED on pin 11 (green landing brake deployed)
  External LED on pin10(RED Master Caution)
  Unlike in Mode(INPUT),there is no pull-down resistor necessary. An internal 20K-ohm resistor is pulled to 5V. This configuration causes the input to read HIGH when the switch is open, and LOW when it is closed.

  Requires the FastLED and Time libraries
*/
#define LED_PIN     2
#define NUM_LEDS    10
#define BRIGHTNESS  70
//#define CHR_DELAY 1000 // time in ms between characters

const unsigned long alertInterval = 11000; //11 seconds
const unsigned long playInterval = 3000;
const unsigned long silenceInterval = 60000; // 1 minute
const unsigned long boostalertInterval = 300000; //5 minutes

CRGB leds[NUM_LEDS];

SoftwareSerial mySoftwareSerial(14, 16); // RX(MOSI), TX(MISO)
DFRobotDFPlayerMini myDFPlayer;

//Pin assignments
int GEAR_SW = 3;
int CANOPY_SW = 4;
int TEST_SW = 5;
int SILENCE_SW = 7;  //Was pin6, but it died on the arduino
int LB_SW = 10;
int LOWVOLT_SW = 20;
int THROTTLE_IN = 19; // analog pin A1; digital pin 19
int BOOSTPUMP_SW = 21; // Analog 3
int IGN1_SW = 22; //Now analog 5 was digital 12; //DSUB pin 8
int IGN2_SW = 23; //Now analog 4 was digital 13; //DSUB pin 9

//Throttle position is analog from MAP sensor
// MAP Sensor voltage map (0-5 volt):
// Manifold valuum ->
// Wide open Throttle <-     -> Idle
// Hg  0  2.9  5.9  8.9  11.8  14.8  17.7  20.7  23.7  26.7  29.6
// Vol 4.9 4.4 3.8  3.3   2.7   2.2   1.7   1.1   0.6   0.3   0.3
// Arduino Analog to Digital conv range 0 - 1023

int THROTTLE_LOW = 220; // fast idle (N40EB needs 260) (1.28volts)
int THROTTLE_ADVANCED = 350; // above taxi power (N40EB needs 350)  (1.71volts)

// switch variables and starting values
int gearVal = 0;
int canopyVal = 0;
int lbVal = 0;
int testVal = 0;
int silenceVal = 0;
int lowvoltVal = 0;
int boostpumpVal = 0;
int throttleVal = 0;
int ign1Val = 0;
int ign2Val = 0;
float voltage = throttleVal * (5.0 / 1023.0);

// alert variables and starting values
int gear_warn = 0;
int brake_warn = 0;
int canopy_warn = 0;
int lowvolt_warn = 0;
int boostpump_warn = 0;
int adjbright;
int alert = 0;
int silenced_state = 0;
int alertplaycount = 0;
int boostpumpplaycount = 0;

// Set Nicknames for the switch positions (HIGH/LOW) for the gear, landing brake, and canopy; change these as needed for testing
int GEAR_IS_RETRACTED = HIGH; //N40EB needs HIGH
int LB_IS_EXTENDED = LOW; //N40EB needs LOW
int CANOPY_IS_OPEN = HIGH; //N40EB needs HIGH
int TEST_IS_PRESSED = LOW; //N40EB needs LOW
int BOOSTPUMP_IS_ON = HIGH;
int IGN1_IS_OFF = HIGH; // LED is ON when Ignition is off (HIGH; do we need pull-down resistors?)
int IGN2_IS_OFF = HIGH; // LED is ON when Ignition is off (HIGH)

int mp3player_fault = 0;

// Set some time based variables
unsigned long currentMillis = 0;
unsigned long previousAlertMillis = 0;
unsigned long previousBoostAlertMillis = 0;
unsigned long previousSilencedMillis = 0;
unsigned long intervalBoostPumpPlayMillis = 0 ;
unsigned long intervalPlayMillis = 0;
unsigned long lastPlayMillis = 0;

// Define the number of throttle samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 50;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int throttleAverage = 0;        // the throttleAverage

void printDetail(uint8_t type, int value);

void setup() { //configure input pins as an input and enable the internal pull-up resistor
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS );
  pinMode(GEAR_SW, INPUT_PULLUP);
  pinMode(CANOPY_SW, INPUT_PULLUP);
  pinMode(LB_SW, INPUT_PULLUP);
  pinMode(SILENCE_SW, INPUT_PULLUP);
  pinMode(TEST_SW, INPUT_PULLUP);
  pinMode(LOWVOLT_SW, INPUT);
  pinMode(BOOSTPUMP_SW, INPUT);
  pinMode(THROTTLE_IN, INPUT);
  pinMode(IGN1_SW, INPUT);
  pinMode(IGN2_SW, INPUT);

  mySoftwareSerial.begin(9600);
  Serial.begin(9600);
  delay(1000);

  // initialize all the Throttle readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  delay(250);
  if (!myDFPlayer.begin(mySoftwareSerial, true)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    int ledCells [] = {5, 4, 6, 3, 7, 2, 8, 2, 9, 0}; // 5,4,6,3,7,2,8,2,9,0 or 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
    uint32_t colorArray [] = {CRGB::Red, CRGB::Black, CRGB::Red, CRGB::Black, CRGB::Red};
    for (uint32_t thisColor : colorArray) {
      for (int thisCell : ledCells) {
        leds[thisCell] = thisColor;
        FastLED.show();
        delay(50);
      }
    }
    mp3player_fault = 1;
    //while (true);
  } else {
    int ledCells [] = {9, 0, 8, 1, 7, 2, 6, 3, 5, 4}; // 9,0,8,1,7,2,6,3,5,4
    uint32_t colorArray [] = {CRGB::Blue, CRGB::Yellow, CRGB::Green, CRGB::Black, CRGB::Blue, CRGB::Yellow, CRGB::Green};
    for (uint32_t thisColor : colorArray) {
      for (int thisCell : ledCells) {
        leds[thisCell] = thisColor;
        FastLED.show();
        delay(50);
      }
    }
  }

  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  playMp3(1, 1);  //Play the first mp3, once (SystemTest)

}

/** * Read all the switches and call the display function
*/
void loop() {
  // Store the current milliseconds value each loop
  currentMillis = millis();

  // Read the values of all the switches
  readSwitchState();

  // Set the alert state and return value
  alert = isAlertState();

  // Call the display function after reading the alert state
  display();

  // Invoke the MP3 module
  queueAudio();


  //Serial.println(myDFPlayer.readState());
  //Serial.print("\t");
  Serial.print("currentMillis - previousSilencedMillis:");
  Serial.print("\t");
  Serial.print(currentMillis - previousSilencedMillis);
  Serial.println();
  Serial.print("currentMillis - intervalPlayMillis:");
  Serial.print("\t");
  Serial.print(currentMillis - intervalPlayMillis);
  Serial.println();
  Serial.print("currentMillis - lastPlayMillis:");
  Serial.print("\t");
  Serial.print(currentMillis - lastPlayMillis);
  Serial.println();
  Serial.print("currentMillis - intervalBoostPumpPlayMillis:");
  Serial.print("\t");
  Serial.print(currentMillis - intervalBoostPumpPlayMillis);
  Serial.println();
  //Serial.println();
  Serial.print("Low Volt Value:");
  Serial.print("\t");
  Serial.print(lowvoltVal);
  Serial.println();
  Serial.print("Low Volt Warn:");
  Serial.print("\t");
  Serial.print(lowvolt_warn);
  Serial.println();
  Serial.print("Alert Play Count:");
  Serial.print("\t");
  Serial.print(alertplaycount);
  Serial.println();
  Serial.print("Boost Pump Val:");
  Serial.print("\t");
  Serial.print(boostpumpVal);
  Serial.println();  Serial.print("Boost Pump Play Count:");
  Serial.print("\t");
  Serial.print(boostpumpplaycount);
  Serial.println();
  delay(250);
}

void readSwitchState() {
  //read the switch value into a variable
  gearVal = digitalRead(GEAR_SW);
  canopyVal = digitalRead(CANOPY_SW);
  lbVal = digitalRead(LB_SW);
  testVal = digitalRead(TEST_SW);
  silenceVal = digitalRead(SILENCE_SW);
  lowvoltVal = digitalRead(LOWVOLT_SW); //reads 3 volts when activated (632 value)
  boostpumpVal = digitalRead(BOOSTPUMP_SW);
  throttleVal = analogRead(THROTTLE_IN);
  ign1Val = digitalRead(IGN1_SW);
  ign2Val = digitalRead(IGN2_SW);
  voltage = throttleVal * (5.0 / 1023.0);

  // subtract the last reading of the throttle to average it out:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = throttleVal;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the throttleAverage:
  throttleAverage = total / numReadings;
  throttleAverage = throttleVal;
  delay(1);        // delay in between reads for stability

  // Check to see if the Silenced switch was toggled; if so, reset the timer
  if (!silenceVal) {  // Read if the Silence switch had been toggled; if so, start the 1 minute timer (processed in the playMp3() function
    previousSilencedMillis = currentMillis;  // reset the previousSilencedMillis counter
    Serial.println("The Silenced Switch was pressed!");
  }
}

/* * check the logic to see if there is a master warning to display
  returns 1 if the master warning should be set.
*/
int isAlertState()
{
  // reset the alert and warning variables each loop
  alert = 0;
  canopy_warn = 0;
  brake_warn = 0;
  gear_warn = 0;
  lowvolt_warn = 0;
  boostpump_warn = 0;

  if (!lowvoltVal) {
    lowvolt_warn = 1;
  }

  // Read throttle position and correlate if we should alert based on gear, brake, and canopy states
  if ((throttleAverage >= THROTTLE_ADVANCED) && (canopyVal == CANOPY_IS_OPEN)) {
    alert = 1;
    canopy_warn = 1;
    //Serial.println("Throttle is advanced, and Canopy is open; canopy_warn = 1");
  }
  if ((throttleAverage >= THROTTLE_ADVANCED) && (lbVal == LB_IS_EXTENDED)) {
    alert = 1;
    brake_warn = 1;
    //Serial.println("Throttle is advanced, and Landing Brake is extended; brake_warn = 1");
  }
  if ((throttleAverage >= THROTTLE_ADVANCED) && (lbVal == LB_IS_EXTENDED) && (canopyVal == CANOPY_IS_OPEN)) {
    alert = 1;
    brake_warn = 1;
    canopy_warn = 1;
    //Serial.println("Throttle is advanced, and Landing Brake is extended; brake_warn = 1 AND Canopy is open; canopy_warn = 1");
  }
  if ((throttleAverage <= THROTTLE_LOW) && (gearVal == GEAR_IS_RETRACTED)) {
    alert = 1;
    gear_warn = 1;
    //Serial.println("Throttle is closed, gear retracted; gear_warn = 1");
  }

  return alert;
}

void display()
{
  FastLED.clear();

  // If the Test switch is toggled, light up all the LEDs, play the System Test MP3, and increase the brightness
  if (testVal == TEST_IS_PRESSED) {
    //Serial.println("Test button pressed");
    // MASTER (red)
    leds[8] = CRGB::Red;
    leds[9] = CRGB::Red;
    // GEAR (green)
    leds[7] = CRGB::Orange;
    leds[6] = CRGB::Orange;
    // BRAKE (green)
    leds[5] = CRGB::Orange;
    leds[4] = CRGB::Orange;
    // CANOPY (green)
    leds[3] = CRGB::Orange;
    leds[2] = CRGB::Orange;
    //LOW VOLT (yellow)
    leds[1] = CRGB::Yellow;
    leds[0] = CRGB::Yellow;
    // increment brightness per button press
    if (adjbright <= 255) {
      adjbright += 25;
      FastLED.setBrightness(adjbright);
      FastLED.show();
    } else {
      adjbright = 1;
    }
    playMp3(1, 1);
  } else {  // If in the Alert state, correctly set the LEDs per the warning variables
    if (alert) {
      //Serial.println("Master Caution issued");
      leds[9] = CRGB::Red;
      leds[8] = CRGB::Red;
      //Serial.println("Alert State");
      if (gear_warn) {
        //Serial.println("Caution issued due to Gear warning");
        leds[7] = CRGB::Orange;
        leds[6] = CRGB::Orange;
      } else {
        leds[7] = CRGB::Black;
        leds[6] = CRGB::Black;
      }
      if (brake_warn) {
        //Serial.println("Caution issued due to Brake warning");
        leds[5] = CRGB::Orange;
        leds[4] = CRGB::Orange;
      } else {
        leds[5] = CRGB::Black;
        leds[4] = CRGB::Black;
      }
      if (canopy_warn) {
        //Serial.println("Caution issued due to Canopy warning");
        leds[3] = CRGB::Orange;
        leds[2] = CRGB::Orange;
      } else {
        leds[3] = CRGB::Black;
        leds[2] = CRGB::Black;
      }
      if (!(lowvolt_warn) && ((ign1Val == IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF))) {
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Black;
      }
      if (!(lowvolt_warn) && ((ign1Val != IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF))) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Blue;
      }
      if (!(lowvolt_warn) && ((ign1Val == IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF))) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
      if (!(lowvolt_warn) && ((ign1Val != IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF))) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
    } else { // When not in Alarm state, ensure Black LEDs with exception of when IGN1|IGN2 are not on...
      if (gearVal == GEAR_IS_RETRACTED) { //HIGH means that the gear is retracted; LOW means that the gear is extended
        leds[7] = CRGB::Black;
        leds[6] = CRGB::Black;
        //Serial.println("Gear is retracted");
      }
      if (lbVal == LB_IS_EXTENDED) { //LOW means LB is extended; HIGH means that the LB is retracted
        leds[5] = CRGB::Black;
        leds[4] = CRGB::Black;
        //Serial.println("Landing Brake is extended");
      }
      if (canopyVal == CANOPY_IS_OPEN) { //HIGH means the canopy is open; LOW means the microswitches are pressed (canopy closed)
        leds[3] = CRGB::Black;
        leds[2] = CRGB::Black;
        //Serial.println("Canopy is open");
      }
      if (!(lowvolt_warn) && ((ign1Val == IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF))) {
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Black;
      }
      if (!(lowvolt_warn) && ((ign1Val != IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF))) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Blue;
      }
      if (!(lowvolt_warn) && ((ign1Val == IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF))) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
      if (!(lowvolt_warn) && ((ign1Val != IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF))) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      } else { // output the master alarm status
        leds[9] = CRGB::Black;
        leds[8] = CRGB::Black;
      }
    }
  }
  // Examine the Low Volt or Boost Pump states regardless of Alarm state
  if (lowvolt_warn) {
    //Serial.println("Caution issued due to Low Volt warning");
    leds[1] = CRGB::Yellow;
    leds[0] = CRGB::Yellow;
  }
  if (boostpumpVal) { // This uses the left half of the Canopy light cell
    //Serial.println("Caution issued due to Boost Pump warning");
    leds[1] = CRGB::Purple;
    leds[0] = CRGB::Purple;
  }

  FastLED.show();
}

void playMp3(int file, int playnum) {
  if (!mp3player_fault) {
  int playerState = 0;
  for (int i = 0; i < playnum; i++) {  //repeat per the playnum
      myDFPlayer.play(file);
      delay(400);
      while (playerState != 512) {
        delay(500);
        playerState = myDFPlayer.readState();
      }
    }
    return playerState;
  }
}

void queueAudio() {
  if (alert) {
    if ((alertplaycount > 1) && (currentMillis - previousSilencedMillis) <= silenceInterval) {  // if the initial play count is low, bypass the previous silenced check
      //Serial.println("Previous Silenced is <= the Silence Interval");
      //Serial.println();
    } else {
      //Serial.println("Previous Silenced is >= the Silence Interval");
      //Serial.println();
      if ((currentMillis - intervalPlayMillis) >= alertInterval) {
        if (gear_warn) {
          playMp3(2, 2);
        }
        if (brake_warn && !canopy_warn) {
          playMp3(3, 2);
        }
        if (canopy_warn && !brake_warn) {
          playMp3(4, 2);
        }
        if ((brake_warn && canopy_warn)) {
          playMp3(3, 1);
          playMp3(4, 1);
        }
        intervalPlayMillis = currentMillis;  // reset the lastPlayMillis counter
        alertplaycount++;
      }
    }
  }
  if (boostpumpVal) {
    if ((boostpumpplaycount > 0) && (currentMillis - intervalBoostPumpPlayMillis) <= boostalertInterval) {
    } else {
      playMp3(9, 1);
      intervalBoostPumpPlayMillis = currentMillis;  // reset the intervalBoostPumpPlayMillis counter
      boostpumpplaycount++;
    }

  }
}
