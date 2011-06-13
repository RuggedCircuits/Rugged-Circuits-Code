/* 
   This application illustrates the IRTransmit() and IRReceive() functions of the Gadget Shield. The
   application sets the power of the IR transmitter according to the potentiometer setting. If a
   reflection is received by the IR receiver, the on-board RGB LED turns red.

   Adjust the potentiometer for best sensitivity. Too much IR transmit power and the RGB LED will
   stay on all the time due to leakage. Too little power and the IR receiver won't see a reflection.
  
   Application Version 1.0 -- February 2011

   Copyright (c) 2011 Rugged Circuits LLC.  All rights reserved.
   http://ruggedcircuits.com

   This file is part of the Rugged Circuits Gadget Shield library and code
   samples for Arduino.

   This library is free software; you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free Software
   Foundation; either version 3 of the License, or (at your option) any later
   version.

   This library is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
   FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
   details.

   A copy of the GNU General Public License can be viewed at <http://www.gnu.org/licenses>
 */
#include <GadgetShield.h>

unsigned long lastMillis;

void setup() {
  GS.Setup();

  lastMillis = millis();
}

void loop() {
  if ((signed long)(millis() - lastMillis) >= 100) {
    lastMillis += 100;
    GS.IRTransmit(1, GS.Pot()/13); // Go up to about 20% duty cycle
  }

  if (GS.IRReceive()) {
    GS.RGB(16, 0, 0);
  } else {
    GS.RGB(0, 0, 0);
  }
}
// vim: syntax=cpp ai cindent sw=2 ts=2 expandtab
