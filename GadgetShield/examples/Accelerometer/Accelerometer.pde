/* 
   This application illustrates the accelerometer functions on the Gadget
   Shield. The RGB LED is set to a color that represents the tilt orientation
   of the board. If a "shake" condition is found (vibration of the
   accelerometer) then the 4 on-board LED's are lit.
  
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

// How often to take accelerometer readings
#define SAMPLE_RATE 30  // milliseconds

// The time at which to turn off the shake display
unsigned long shakeTimeout;
uint8_t isShake;

// The time at which to take the next reading
unsigned long sampleTime;

void all_leds(uint8_t ison)
{
  GS.LED(0,ison);
  GS.LED(1,ison);
  GS.LED(2,ison);
  GS.LED(3,ison);
}

void setup() {
  GS.Setup();

  isShake = 0;
  sampleTime = millis() + SAMPLE_RATE;
  Serial.begin(19200);

  // Strong filtering to get smooth transitions
  GS.AccelSetFilterX(252);
  GS.AccelSetFilterY(252);
  GS.AccelSetFilterZ(252);
}

// Function to map accelerometer readings to desired color brightness
uint8_t colormap(uint8_t reading)
{
  return map(constrain(reading, 12, 52), 12, 52, 0, 16);
}

void loop() {
  uint8_t x,y,z;
  
  // If it's time to take another sample, do so and update our RGB LED
  if ((long)(millis() - sampleTime) >= 0) {
    sampleTime += SAMPLE_RATE;
    GS.AccelSample();

    x = colormap(GS.AccelResultX());
    y = colormap(GS.AccelResultY());
    z = colormap(GS.AccelResultZ());
    Serial.print((unsigned)x); Serial.print(" ");
    Serial.print((unsigned)y); Serial.print(" ");
    Serial.print((unsigned)z); Serial.println();

    GS.RGB(x, y, z);
  } else return;

  // If a "shake" condition was found then light the 4 LED's for a while as an indicator
  if (GS.AccelShake()) {
    isShake = 1;
    all_leds(1);
    shakeTimeout = millis()+500;
  } else if (isShake && ((long)(millis() - shakeTimeout) >= 0)) {
    isShake = 0;
    all_leds(0);
  }
}
// vim: syntax=cpp ai cindent sw=2 ts=2 expandtab
