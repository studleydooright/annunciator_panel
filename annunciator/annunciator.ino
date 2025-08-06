// Annunciator Panel Sketch - Refactored and Improved
#include <TimeLib.h>
#define FASTLED_INTERNAL
#include <FastLED.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// === Configuration ===
#define LED_PIN         2
#define NUM_LEDS        10
#define BRIGHTNESS      70
#define VOLUME_LEVEL    30
#define DEBUG            // Comment this line to disable debug output

// === Timing Constants (ms) ===
const unsigned long ALERT_INTERVAL = 11000;
const unsigned long PLAY_INTERVAL = 3000;
const unsigned long SILENCE_INTERVAL = 60000;
const unsigned long BOOST_ALERT_INTERVAL = 300000;

// === Analog Thresholds ===
const int THROTTLE_LOW = 220;
const int THROTTLE_ADVANCED = 350;

// === Input Pins ===
const uint8_t GEAR_SW = 3;
const uint8_t CANOPY_SW = 4;
const uint8_t TEST_SW = 5;
const uint8_t SILENCE_SW = 7;
const uint8_t LB_SW = 10;
const uint8_t LOWVOLT_SW = 20;
const uint8_t THROTTLE_IN = 19;
const uint8_t BOOSTPUMP_SW = 21;
const uint8_t IGN1_SW = 22;
const uint8_t IGN2_SW = 23;

// === Switch Logic Levels ===
const bool GEAR_IS_RETRACTED = HIGH;
const bool LB_IS_EXTENDED = LOW;
const bool CANOPY_IS_OPEN = HIGH;
const bool TEST_IS_PRESSED = LOW;
const bool BOOSTPUMP_IS_ON = HIGH;
const bool IGN_OFF = HIGH;

// === State Structs ===
struct SwitchState {
  bool gear, canopy, brake, test, silence, lowVolt, boostPump, ign1, ign2;
  int throttleRaw, throttleAvg;
  float voltage;
};

struct AlertState {
  bool gear, canopy, brake, lowVolt, boostPump, active;
};

SwitchState switches;
AlertState alerts;

// === LED and Audio ===
CRGB leds[NUM_LEDS];
SoftwareSerial mp3Serial(14, 16);
DFRobotDFPlayerMini mp3;
bool mp3Fault = false;

// === Timing State ===
unsigned long millisNow = 0;
unsigned long lastAlertMillis = 0;
unsigned long lastSilenceMillis = 0;
unsigned long lastBoostAlertMillis = 0;
unsigned long lastPlayMillis = 0;
int alertPlayCount = 0;
int boostPlayCount = 0;

// === Throttle Averaging ===
const int NUM_READINGS = 50;
int throttleReadings[NUM_READINGS];
int throttleIndex = 0;
int throttleTotal = 0;

// === Setup ===
void setup() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
const uint8_t pullupPins[] = {GEAR_SW, CANOPY_SW, LB_SW, TEST_SW, SILENCE_SW};
const uint8_t inputPins[]  = {LOWVOLT_SW, BOOSTPUMP_SW, THROTTLE_IN, IGN1_SW, IGN2_SW};

for (uint8_t i = 0; i < sizeof(pullupPins) / sizeof(pullupPins[0]); i++) {
  pinMode(pullupPins[i], INPUT_PULLUP);
}
for (uint8_t i = 0; i < sizeof(inputPins) / sizeof(inputPins[0]); i++) {
  pinMode(inputPins[i], INPUT);
}


  mp3Serial.begin(9600);
  Serial.begin(9600);
  delay(1000);

  for (int i = 0; i < NUM_READINGS; i++) throttleReadings[i] = 0;

  #ifdef DEBUG
  Serial.println(F("Initializing DFPlayer..."));
  #endif

  if (!mp3.begin(mp3Serial, true)) {
    mp3Fault = true;
    #ifdef DEBUG
    Serial.println(F("DFPlayer failed to initialize"));
    #endif
    signalFault();
  } else {
    mp3.volume(VOLUME_LEVEL);
    playMp3(1, 1);
    signalReady();
  }
}

// === Main Loop ===
void loop() {
  millisNow = millis();
  readSwitches();
  updateAlertState();
  updateLEDs();
  handleAudio();
}

// === Read all inputs and throttle averaging ===
void readSwitches() {
  switches.gear = digitalRead(GEAR_SW);
  switches.canopy = digitalRead(CANOPY_SW);
  switches.brake = digitalRead(LB_SW);
  switches.test = digitalRead(TEST_SW);
  switches.silence = digitalRead(SILENCE_SW);
  switches.lowVolt = digitalRead(LOWVOLT_SW);
  switches.boostPump = digitalRead(BOOSTPUMP_SW);
  switches.ign1 = digitalRead(IGN1_SW);
  switches.ign2 = digitalRead(IGN2_SW);
  switches.throttleRaw = analogRead(THROTTLE_IN);
  switches.voltage = switches.throttleRaw * (5.0 / 1023.0);

  // Update throttle average
  throttleTotal -= throttleReadings[throttleIndex];
  throttleReadings[throttleIndex] = switches.throttleRaw;
  throttleTotal += throttleReadings[throttleIndex];
  throttleIndex = (throttleIndex + 1) % NUM_READINGS;
  switches.throttleAvg = throttleTotal / NUM_READINGS;

  if (!switches.silence) lastSilenceMillis = millisNow;
}

// === Evaluate Alert Conditions ===
void updateAlertState() {
  alerts = {false, false, false, false, false, false};

  if (!switches.lowVolt) alerts.lowVolt = true;
  if ((switches.throttleAvg >= THROTTLE_ADVANCED) && switches.canopy == CANOPY_IS_OPEN) alerts.canopy = true;
  if ((switches.throttleAvg >= THROTTLE_ADVANCED) && switches.brake == LB_IS_EXTENDED) alerts.brake = true;
  if ((switches.throttleAvg <= THROTTLE_LOW) && switches.gear == GEAR_IS_RETRACTED) alerts.gear = true;

  alerts.active = alerts.gear || alerts.brake || alerts.canopy;
}

// === LED Update ===
void updateLEDs() {
  FastLED.clear();

  if (switches.test == TEST_IS_PRESSED) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Orange;
    FastLED.show();
    playMp3(1, 1);
    return;
  }

  if (alerts.active) {
    if (alerts.gear) { leds[6] = leds[7] = CRGB::Orange; }
    if (alerts.brake) { leds[4] = leds[5] = CRGB::Orange; }
    if (alerts.canopy) { leds[2] = leds[3] = CRGB::Orange; }
    leds[8] = leds[9] = CRGB::Red;
  }

  if (alerts.lowVolt) {
    leds[0] = leds[1] = CRGB::Yellow;
  } else if (switches.boostPump) {
    leds[0] = leds[1] = CRGB::Purple;
  } else if (switches.ign1 == IGN_OFF) {
    leds[0] = CRGB::Blue;
  } else if (switches.ign2 == IGN_OFF) {
    leds[1] = CRGB::Blue;
  }

  FastLED.show();
}

// === Audio Handling ===
void handleAudio() {
  if (alerts.active && (alertPlayCount < 1 || millisNow - lastSilenceMillis >= SILENCE_INTERVAL)) {
    if (millisNow - lastPlayMillis >= ALERT_INTERVAL) {
      if (alerts.gear) playMp3(2, 2);
      if (alerts.brake && !alerts.canopy) playMp3(3, 2);
      if (alerts.canopy && !alerts.brake) playMp3(4, 2);
      if (alerts.brake && alerts.canopy) { playMp3(3, 1); playMp3(4, 1); }
      lastPlayMillis = millisNow;
      alertPlayCount++;
    }
  }

  if (switches.boostPump && (boostPlayCount < 1 || millisNow - lastBoostAlertMillis >= BOOST_ALERT_INTERVAL)) {
    playMp3(9, 1);
    lastBoostAlertMillis = millisNow;
    boostPlayCount++;
  }
}

// === Play MP3 Helper ===
int playMp3(int file, int times) {
  if (mp3Fault) return -1;
  int state;
  for (int i = 0; i < times; i++) {
    mp3.play(file);
    unsigned long start = millis();
    do {
      delay(100);
      state = mp3.readState();
    } while (state != 512 && millis() - start < 5000);
  }
  return state;
}

// === Startup Light Animations ===
void signalFault() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red;
    FastLED.show();
    delay(50);
    leds[i] = CRGB::Black;
  }
}

void signalReady() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Green;
    FastLED.show();
    delay(50);
    leds[i] = CRGB::Black;
  }
}
