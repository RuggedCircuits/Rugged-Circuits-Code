/* Play a tone on the speaker whenever a  pushbutton  is pressed
 based on

http://www.ruggedcircuits.com/html/gadget_shield.html#SampleSketches

 play a tone when button  0 or 1 is pushed
 use more modular technique

 License: GPLv3+
 Copyright (c) 2011 R P Herrold info@owlriver.com
 */

#include <GadgetShield.h>

void setup(void) {
  GS.Setup();
  Serial.begin(115200);
}

void loop(void) {
  int dirty = 0;
  for (int ctr = 0; ctr < 2 ; ctr++) {
    if (GS.IsSwitch(ctr) == 1) {
      //   we generate either 440 [A over middle C] or 330 Hz [the E]
      //    which are not an even harmonic
      //   and so easier to distinguish audibly
      // http://en.wikipedia.org/wiki/Mathematics_of_musical_scales
      GS.Speaker(330 + 110 * ctr );
      GS.LED(ctr,1);
      Serial.print("Switch ");
      Serial.print(ctr,DEC);
      Serial.print(" ");
      dirty=1;
      delay(100);
    }
  }
  if(dirty == 1) {
    Serial.println();
    for (int ctr = 0; ctr < 2 ; ctr++) {
      GS.LED(ctr,0);
      GS.Speaker(0);
    }
  }

} 

