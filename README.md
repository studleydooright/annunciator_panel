This is based on the Tom Brusehaver "Build Your Own Annunciator Panel" article published in Kitplanes.com https://www.kitplanes.com/build-your-own-annunciator-panel/ (also published at: http://enginemonitor.blogspot.com/2015/08/arduino-annunciator-panel.html )

This advances the project with an LED "light box" annunciator panel built using specialty LED aluminum channel with a frosted white diffusing top, inset with an individually addressible LED strip such as the WS2812B to provide the specific colored panel backlights for the following indicators:

(MASTER | GEAR | BRAKE | CANOPY | LOW VOLT) 

The WS2812B light strip allows individual LED (or groups of LEDs) illumination, color, brightness, etc...  All while only using 3 wires (power|ground|data-in).  The specific WS2812B that I chose is powered by 5v (from the Arduino), and at max brightness, each of the 10 LEDs draws upwards of 40ma per.

"Light box" construction:
I use a 4" length of the aluminum channel/diffuser, along with 10 LEDs from the strip (self-adheased into the bottom of the aluminum channel).  I'm designating 2 LEDs per indicator label, with a spacing of .8" between each pair of LEDs.  Four pieces of plastic gift card rectangle cuttings are hot glued vertically between pairs of LED cells serving as light baffles.

"Light Box" Materials:
Aluminum LED light strip channel with white frosted diffuser:
https://www.amazon.com/gp/product/B07KCR7K4G or https://www.amazon.com/gp/product/B01DM7EZO4

WS2812B LED light strip (lp30) sized with 100leds/meter (I chose this for the .8" spacing between LED pairs):
https://www.amazon.com/gp/product/B07BTTY4FL

Annunciator logic:
- If the throttle (potentiometer) is below a certain threshold (defined in the sketch), and the landing gear is UP, illuminate the GEAR label as orange, and the MASTER label as red.
- If the throttle is above a certain threshold, and the canopy or landing brake are open or extended, illuminate either/both the BRAKE|CANOPY as orange, and the MASTER as red.
- A Test switch is used to illuminate (MASTER | GEAR | BRAKE | CANOPY | LOW VOLT) indicators (for 1sec).  Each press of the Test switch will also incrementally increase the LED brightness until reaching Max (11 presses), where upon it'll drop to minumum brightness and cycle up again.

Photos (see the /images folder):
![Test Indicator Illumination](/images/test_indicators.jpg)
![Low Throttle-Gear Up Warning](/images/low_throttle-gear_warning.jpg)
![Gear Down indicator](/images/gear_down.jpg)
![High Throttle-Brake and Canopy Warnings](/images/high_throttle-canopy_brake_warning.jpg)
