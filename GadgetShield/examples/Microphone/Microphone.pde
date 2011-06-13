/* 
   This application illustrates the MicrophoneAmplitude() function of the
   Gadget Shield. It illuminates the on-board LED's in proportion to how much
   sound is detected by the microphone. The microphone amplitude is filtered
   so oscillations at the audio frequency do not cause LED's to oscillate too.
  
   Application Version 1.0 -- February 2011

   Copyright (c) 2011 Rugged Circuits LLC.  All rights reserved.
   http://ruggedcircuits.com

   This file is part of the Rugged Circuits Gadget Shield library and code
   samples for Arduino.

   This library is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation; either version 3 of the License, or (at your option)
   any later version.

   This library is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   A copy of the GNU General Public License can be viewed at <http://www.gnu.org/licenses>
 */
#include <GadgetShield.h>

IIRFilter Filter;

void setup() {
  GS.Setup();

  // First establish baseline "quiet" reading. Ensure the microphone is in a quiet
  // environment when the Arduino is reset or powered up.
  GS.MicrophoneSetBaseline();
}

void loop() {
  uint8_t reading;

  Filter.input(GS.MicrophoneAmplitude());
  reading = Filter.result();

  if (reading > 100) {
    GS.LED(3,1);
  } else {
    GS.LED(3,0);
  }
  if (reading > 75) {
    GS.LED(2,1);
  } else {
    GS.LED(2,0);
  }
  if (reading > 50) {
    GS.LED(1,1);
  } else {
    GS.LED(1,0);
  }
  if (reading > 25) {
    GS.LED(0,1);
  } else {
    GS.LED(0,0);
  }
}
// vim: syntax=cpp ai cindent sw=2 ts=2 expandtab
