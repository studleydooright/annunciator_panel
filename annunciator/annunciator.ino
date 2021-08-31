#include <TimeLib.h>
#include <FastLED.h>

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

CRGB leds[NUM_LEDS];


//Give all the pins a name. Change the pin numbers here and the logic won't have to change.
int GEAR_SW = 3;
int CANOPY_SW = 4;
int TEST_SW = 5;
int LB_SW = 10;
int LOWVOLT_SW = 11;
int IGN1_SW = 23; //Now analog 5 was digital 12; //DSUB pin 8
int IGN2_SW = 22; //Now analog 4 was digital 13; //DSUB pin 9

//Silence pushbutton. Once button released, alarm is silenced for 5 seconds.
int SILENCE_SW = 6;

//Alarm sounder is on pin 3
int ALARM_OUT = 9;

//Throttle position is analog from MAP sensor
// MAP Sensor voltage map (0-5 volt):
// Manifold valuum ->
// Wide open Throttle <-     -> Idle
// Hg  0  2.9  5.9  8.9  11.8  14.8  17.7  20.7  23.7  26.7  29.6
// Vol 4.9 4.4 3.8  3.3   2.7   2.2   1.7   1.1   0.6   0.3   0.3
// Arduino Analog to Digital conv range 0 - 1023

int THROTTLE_IN = 1; // analog pin 0
int THROTTLE_LOW = 260; // fast idle (N40EB needs 200)
int THROTTLE_ADVANCED = 350; // takeoff power (N40EB needs 350)

int gear_warn = 0;
int brake_warn = 0;
int canopy_warn = 0;
int lowvolt_warn = 0;
int adjbright;
int alert = 0;

// Set the switch positions (HIGH/LOW) for the gear, landing brake, and canopy; change these as needed for testing
int GEAR_IS_RETRACTED = HIGH; //N40EB needs HIGH
int LB_IS_EXTENDED = LOW; //N40EB needs LOW
int CANOPY_IS_OPEN = HIGH; //N40EB needs HIGH
int TEST_IS_PRESSED = LOW; //N40EB needs LOW
int IGN1_IS_OFF = HIGH; // LED is ON when Ignition is off (HIGH; do we need pull-down resistors?)
int IGN2_IS_OFF = HIGH; // LED is ON when Ignition is off (HIGH)

// Define the number of throttle samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 50;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int throttleAverage = 0;        // the throttleAverage

void setup() { //configure input pins as an input and enable the internal pull-up resistor
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS );
  pinMode(GEAR_SW, INPUT_PULLUP);
  pinMode(CANOPY_SW, INPUT_PULLUP);
  pinMode(LB_SW, INPUT_PULLUP);
  pinMode(SILENCE_SW, INPUT_PULLUP);
  pinMode(TEST_SW, INPUT_PULLUP);
  pinMode(LOWVOLT_SW, INPUT_PULLUP);
  pinMode(THROTTLE_IN, INPUT);
  pinMode(IGN1_SW, INPUT);
  pinMode(IGN2_SW, INPUT);
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
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
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
  int ign1Val = digitalRead(IGN1_SW);
  int ign2Val = digitalRead(IGN2_SW);
  float voltage = throttleVal * (5.0 / 1023.0);

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
  // send it to the computer as ASCII digits
  //Serial.println(throttleAverage);
  delay(1);        // delay in between reads for stability

  // Set the alert state
  alert = isAlertState(gearVal, canopyVal, lbVal, throttleAverage, lowvoltVal);

  // Call the display function after reading the alert state
  display(gearVal, canopyVal, lbVal, throttleAverage, silenceVal, testVal, lowvoltVal, ign1Val, ign2Val);

  // write the voltage value to the serial monitor:
  //Serial.println(voltage);
  //Serial.println(throttleAverage);
  //Serial.println(ign1Val);
  Serial.println(ign2Val);

}

/* * check the logic to see if there is a master warning to display
  returns 1 if the master warning should be set.
*/
int isAlertState(int gearVal, int canopyVal, int lbVal, int throttleAverage, int lowvoltVal)
{
alert = 0;
canopy_warn = 0;
brake_warn = 0;
gear_warn = 0;
lowvolt_warn = 0;

  // low volt check
  if (lowvoltVal) {
    //alert = 1;
    //lowvolt_warn = 1;
  }
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

  Serial.println(alert);
  return alert;
}

/** * Process all the logic, to allow displaying LEDs in
  proper state, and handle master alarm state,
  including sound.
*/
void display(int gearVal, int canopyVal, int lbVal, int throttleAverage, int silenceVal, int testVal, int lowvoltVal, int ign1Val, int ign2Val)
{

  //Serial.println("Alert state ");
  //Serial.println(alert);

  FastLED.clear();
  // gap is the cycle time for the tone. state is the tone state.
  static int gap = 0;
  static int state = LOW;

  // silenced at is the time the alarm was silenced at.
  static time_t silencedAt = 0;

  // Keep in mind the pullup means the pushbutton's
  // logic is inverted. It goes HIGH when it's open,
  // and LOW when it's pressed.:

  if (testVal == TEST_IS_PRESSED) {
    //Serial.println("Test button pressed");
    digitalWrite(ALARM_OUT, HIGH);
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
    Serial.println(adjbright);
    delay(1000);
  } else {
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
      if (lowvolt_warn) {
        //Serial.println("Caution issued due to Low Volt warning");
        //leds[1] = CRGB::Yellow;
        //leds[0] = CRGB::Yellow;
      } else {
        leds[1] = CRGB::Black;
        leds[0] = CRGB::Black;
      }
      if ((ign1Val == IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF)) {
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Black;
      }
      if ((ign1Val != IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF)) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Blue;
      }
      if ((ign1Val == IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF)) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
      if ((ign1Val != IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF)) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
      if (now() > silencedAt + 60) {
        digitalWrite(ALARM_OUT, HIGH);
      } else {
        digitalWrite(ALARM_OUT, LOW);
      }
    } else {
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
      if (lowvoltVal == LOW) {
        leds[1] = CRGB::Black;
        leds[0] = CRGB::Black;
      }
      if ((ign1Val == IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF)) {
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Black;
      }
      if ((ign1Val != IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF)) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Blue;
      }
      if ((ign1Val == IGN1_IS_OFF) && (ign2Val == IGN2_IS_OFF)) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
      if ((ign1Val != IGN1_IS_OFF) && (ign2Val != IGN2_IS_OFF)) {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
      }
      // The silence button is open normally. Logic is reversed.
      if (silenceVal) {
        silencedAt = now();
      } else { // output the master alarm status
        leds[9] = CRGB::Black;
        leds[8] = CRGB::Black;
        digitalWrite(ALARM_OUT, LOW);
      }
    }
  }
  FastLED.show();
}
