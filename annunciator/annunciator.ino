#include <TimeLib.h>
#include <FastLED.h>

/*
  Annunciator Panel SketchThis program will light up the Annunciator Panel based on curent status of the Aircraft.The circuit:
  Momentary switch attached from pin 4 to ground (Nose Gear)
  Momentary switch attached from pin 6 to ground (Canopy)
  Momentary switch attached from pin 7 to ground (Landing Brake)
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
#define BRIGHTNESS  50
//#define CHR_DELAY 1000 // time in ms between characters

CRGB leds[NUM_LEDS];


//Give all the pins a name. Change the pin numbers here and the logic won't have to change.
int GEAR_SW = 3;
int CANOPY_SW = 4;
int TEST_SW = 5;
int LB_SW = 10;
int LOWVOLT_SW = 11;

//Silence pushbutton. Once button released, alarm is silenced for 5 seconds.
int SILENCE_SW = 6;

//Alarm sounder is on pin 3
int ALARM_OUT = 9;

//Throttle position is analog
int THROTTLE_IN = 1; // analog pin 0
int THROTTLE_CLOSED = 800; // fast idle
int THROTTLE_MAX = 200; // takeoff power

int gear_warn = 0;
int brake_warn = 0;
int canopy_warn = 0;
int lowvolt_warn = 0;
int adjbright;

void setup() { //configure input pins as an input and enable the internal pull-up resistor
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS );
  pinMode(GEAR_SW, INPUT_PULLUP);
  pinMode(CANOPY_SW, INPUT_PULLUP);
  pinMode(LB_SW, INPUT_PULLUP);
  pinMode(SILENCE_SW, INPUT_PULLUP);
  pinMode(TEST_SW, INPUT_PULLUP);

  // configure output pins for output.


  Serial.begin(9600);
  Serial.print("hello!");
  //  tft.begin();

  Serial.println("init");

  uint16_t time = millis();
  //  tft.fillRect(0, 0, 128, 128, BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  Serial.println("done");
  delay(1000);
}

/** * Read all the switches and call the display function
*/
void loop() {
  //read the switch value into a variable
  int gearVal = digitalRead(GEAR_SW);
  int canopyVal = digitalRead(CANOPY_SW);
  int lbVal = digitalRead(LB_SW);
  int testVal = digitalRead(TEST_SW);
  int silenceVal = digitalRead(SILENCE_SW);
  int lowvoltVal = digitalRead(LOWVOLT_SW);
  int throttleVal = analogRead(THROTTLE_IN);
  //int throttleVal = digitalRead(THROTTLE_SW);

  display(gearVal, canopyVal, lbVal, throttleVal, silenceVal, testVal, lowvoltVal);
  float voltage = throttleVal * (5.0 / 1024.0);
  // write the voltage value to the serial monitor:
  Serial.println(voltage);
  Serial.println(throttleVal);
}

/* * check the logic to see if there is a master warning to display
  returns 1 if the master warning should be set.
*/
int isAlertState(int gearVal, int canopyVal, int lbVal, int throttleVal, int lowvoltVal)
{
  int alert = 0;
  gear_warn = 0;
  brake_warn = 0;
  canopy_warn = 0;
  lowvolt_warn = 0;

  // low volt check
  if (lowvoltVal) {
    alert = 1;
    lowvolt_warn = 1;
  }

  // throttle closed, and landing gear up
  if ((throttleVal >= THROTTLE_CLOSED) && (gearVal)) {
    alert = 1;
    gear_warn = 1;
  }

  // throttle max and canopy not closed, and landing brake not up
  if ((throttleVal <= THROTTLE_MAX) && (canopyVal || !lbVal)) {
    alert = 1;
    if (canopyVal) {
      canopy_warn = 1;
    }
    if (!lbVal) {
      brake_warn = 1;
    }
  }


  return alert;
}

/** * Process all the logic, to allow displaying LEDs in
  proper state, and handle master alarm state,
  including sound.
*/
void display(int gearVal, int canopyVal, int lbVal, int throttleVal, int silenceVal, int testVal, int lowvoltVal)
{
  int alert = 0;
  FastLED.clear();
  // gap is the cycle time for the tone. state is the tone state.
  static int gap = 0;
  static int state = LOW;

  // silenced at is the time the alarm was silenced at.
  static time_t silencedAt = 0;

  // Keep in mind the pullup means the pushbutton's
  // logic is inverted. It goes HIGH when it's open,
  // and LOW when it's pressed.:


  if (testVal == LOW) {
    //Serial.println("Test button pressed");
    digitalWrite(ALARM_OUT, HIGH);
    // MASTER (red)
    leds[8] = CRGB::Red;
    leds[9] = CRGB::Red;
    // GEAR (green)
    leds[7] = CRGB::Green;
    leds[6] = CRGB::Green;
    // BRAKE (green)
    leds[5] = CRGB::Green;
    leds[4] = CRGB::Green;
    // CANOPY (green)
    leds[3] = CRGB::Green;
    leds[2] = CRGB::Green;
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
    Serial.println(adjbright);
    delay(1000);
  } else {
    if (gearVal == LOW) {
      //Serial.println("Gear button pressed");
      leds[7] = CRGB::Green;
      leds[6] = CRGB::Green;
    } else {
      leds[7] = CRGB::Black;
      leds[6] = CRGB::Black;
    }
    if (lbVal == HIGH) {
      //Serial.println("Brake button pressed");
      leds[5] = CRGB::Green;
      leds[4] = CRGB::Green;
    } else {
      leds[5] = CRGB::Black;
      leds[4] = CRGB::Black;
    }
    if (canopyVal == LOW) {
      //Serial.println("Canopy button pressed");
      leds[3] = CRGB::Green;
      leds[2] = CRGB::Green;
    } else {
      leds[3] = CRGB::Black;
      leds[2] = CRGB::Black;
    }
    if (lowvoltVal == LOW) {
      leds[1] = CRGB::Yellow;
      leds[0] = CRGB::Yellow;
    } else {
      leds[1] = CRGB::Black;
      leds[0] = CRGB::Black;
    }
    // The silence button is open normally. Logic is reversed.
    if (silenceVal) {
      silencedAt = now();
    }

    alert = isAlertState(gearVal, canopyVal, lbVal, throttleVal, lowvoltVal);

    // output the master alarm status
    if (alert) {
      Serial.println("Master Caution issued");
      leds[9] = CRGB::Red;
      leds[8] = CRGB::Red;
      if (gear_warn) {
        //Serial.println("Caution issued due to Gear warning");
        leds[7] = CRGB::Orange;
        leds[6] = CRGB::Orange;
      }
      if (brake_warn) {
        //Serial.println("Caution issued due to Brake warning");
        leds[5] = CRGB::Orange;
        leds[4] = CRGB::Orange;
      }
      if (canopy_warn) {
        //Serial.println("Caution issued due to Canopy warning");
        leds[3] = CRGB::Orange;
        leds[2] = CRGB::Orange;
      }
      /* if (lowvolt_warn) {
        Serial.println("Caution issued due to Low Volt warning");
        leds[3] = CRGB::Orange;
        leds[2] = CRGB::Orange;
        } */
      if (now() > silencedAt + 60) {
        digitalWrite(ALARM_OUT, HIGH);
      } else {
        digitalWrite(ALARM_OUT, LOW);
      }
    } else {
      leds[9] = CRGB::Black;
      leds[8] = CRGB::Black;
      digitalWrite(ALARM_OUT, LOW);
    }
  }
  FastLED.show();
}
