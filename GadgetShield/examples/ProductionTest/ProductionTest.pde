/* 
   This application exercises all of the functions on the Gadget Shield.

   * The 3-axis orientation of the Gadget Shield controls the RGB LED color
   * An accelerometer "shake" causes the speaker to emit a tone
   * The potentiometer controls the brightness of LED #1.
   * The IR emitter and detector are configured as object detectors. If an
     object is detected LED #3 lights.
   * The visible light sensor turns on LED #2.
   * Sounds detected by the microphone are reflected in LED #4.
   * If pushbutton #1 is pressed then all 4 LED's blink
   * If pushbutton #2 is pressed then the RGB LED turns off

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

// The time at which to turn off the shake sound
unsigned long shakeTimeout;
uint8_t isShake;

// The time at which to take the next accelerometer reading
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

  GS.MicrophoneSetBaseline();
}

void loop() {
  uint8_t millis_duty;

  // If it's time to take another accelerometer sample, do so and update our RGB LED.
  // Also do shake detection.
  if ((long)(millis() - sampleTime) >= 0) {
    uint8_t x,y,z,r,g,b;

    sampleTime += SAMPLE_RATE;

    // If a "shake" condition was found then emit a tone. Read this pin before addressing the
    // accelerometer since any I2C access to it clears the interrupt. We use the interrupt
    // pin method of determining whether a shake condition is found (rather than calling
    // AccelShake()) just to test connectivity/functionality of this pin.
    if (GS.AccelIsInt()) {
      isShake = 1;
      GS.Speaker(440);
      shakeTimeout = millis()+500;
    } else if (isShake && ((long)(millis() - shakeTimeout) >= 0)) {
      isShake = 0;
      GS.Speaker(0);
    }

    // Read the 3 axes
    GS.AccelSample();

    x = GS.AccelResultX();
    y = GS.AccelResultY();
    z = GS.AccelResultZ();
    Serial.print((unsigned)x); Serial.print(" ");
    Serial.print((unsigned)y); Serial.print(" ");
    Serial.print((unsigned)z); Serial.println();

    r=g=b=0;
    if (! GS.IsSwitch(1)) {
      if (x<y && x<z) {
        r=16;
      } else if (y<x && y<z) {
        g=16;
      } else {
        b=16;
      }
    } else {
      GS.IRTransmit(1, 2 + GS.Pot()/25);
    }
    GS.RGB(r,g,b);
  }

  // Pushbutton #1 either controls normal LED operation or allows other objects to control LED's
  if (GS.IsSwitch(0)) {
    uint8_t millis_duty;

    millis_duty = (uint8_t) millis();
    all_leds(millis_duty < 128);
  } else {
    // Potentiometer controls LED #1....this LED happens to be on a PWM channel on the Mega
    GS.LED(0, GS.Pot() < 128);

    // Light sensor controls LED #2
    GS.LED(1, GS.LightSensor() < 192);

    // Object detector controls LED #3
    GS.LED(2, (GS.IRReceive() ? 1 : 0));

    // Microphone controls LED #4
    GS.LED(3, GS.MicrophoneAmplitude() > 30);
  }
}
// vim: syntax=cpp ai cindent sw=2 ts=2 expandtab
