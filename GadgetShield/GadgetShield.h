/*
  GadgetShield.h - Library for Rugged Circuits Gadget Shield
  Copyright (c) 2011 Rugged Circuits LLC.  All rights reserved.
  http://ruggedcircuits.com

  This file is part of the Rugged Circuits Gadget Shield library for Arduino.

	This library is free software; you can redistribute it and/or modify it under
	the terms of the GNU General Public License as published by the Free Software
	Foundation; either version 3 of the License, or (at your option) any later
	version.

	This library is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
	FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
	details.

	A copy of the GNU General Public License can be viewed at
	<http://www.gnu.org/licenses>
*/

#ifndef _GADGETSHIELD_H_
#define _GADGETSHIELD_H_

#include <inttypes.h>
#include <math.h>
#include "WProgram.h"

// Pin definitions //////////////////////////////////////////////////
#define IRIN_PIN        2 // Digital
#define IROUT_PIN       3
#define LED2_PIN        4
#define RED_PIN         5
#define GREEN_PIN       6
#define ACC_INTn_PIN    7
#define PB2_PIN         8
#define SPEAKER_PIN     9
#define BLUE_PIN       10
#define LED1_PIN       11
#define PB1_PIN        12
#define LED3_PIN       13
#define MICROPHONE_PIN  0  // Analog
#define LIGHTSENSOR_PIN 1  // Analog
#define POT_PIN         2  // Analog
#if defined(__AVR_ATmega328P__)
#  define LED4_PIN       17  // Digital
#  define SDA_PIN        18  // Digital
#  define SCL_PIN        19  // Digital
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#  define LED4_PIN       57  // Digital
#  define SDA_PIN        20  // Digital
#  define SCL_PIN        21  // Digital
#else
#  error "The GadgetShield library is not supported on this processor"
#endif

/* Simple first-order IIR filter on 8-bit unsigned inputs. We use a 8-bit
 * state variable with an extra 3 bits of fixed-point precision:

            +-------------------------+-------------+
            |     8-bit state         | 3-bit state |
            +-------------------------.-------------+
            \__saved as the reading__/ 
*/
class IIRFilter
{
  public:
    IIRFilter(uint8_t alpha=230 /* 90% of 255 */) {
      set_alpha(alpha);
      reset();
    }

    void input(uint8_t sample) {
      m_state = (m_alpha*(uint32_t)m_state
                + (uint8_t)((uint8_t)255-m_alpha)*(uint32_t)(((uint16_t)sample) << 3)
                + 128) >> 8;
    }

    void reset(void) {
      m_state = 0;
    }

    uint8_t result(void) {
      return (uint8_t) (m_state >> 3);
    }

    uint8_t get_alpha(void) { return m_alpha; }
    void set_alpha(uint8_t alpha) { m_alpha = alpha; }

  private:
    uint8_t m_alpha;
    uint16_t m_state;
};

class GadgetShield
{
  private:
    IIRFilter filter_x, filter_y, filter_z; // Filter X/Y/Z accelerometer axes
    void accel_init(void);
    void errorWrite(uint8_t code);
    uint8_t blue_duty_cycle;
    uint8_t serialDebug;

    uint8_t is_shake; // Store accelerometer status until queried by user code
    uint8_t is_tap;
    uint8_t m_pola;
    uint8_t m_bafro;

    uint8_t m_mic_baseline; // Average mic reading (i.e., reading when there is quiet)

    void twiWrite(const void *buf, uint8_t length);
    uint8_t s2u(uint8_t val);
    uint8_t readAxes(uint8_t *x, uint8_t *y, uint8_t *z, uint8_t *shake, uint8_t *tap, uint8_t *PoLa, uint8_t *BaFro);

    // Variables used to build up a set of 4 readings (X/Y/Z/Tilt) over perhaps multiple
    // TWI accesses, since the Alert flag might be set on one or more readings.
    uint8_t val[4];     // Values of registers 0-3 read on a single transaction
    uint8_t valid[4];   // Values of registers 0-3 that have been validated (i.e., Alert is not set)
    uint8_t numValid; // How many validated values we have

  public:
    GadgetShield();

    // Call this function in your setup() code
      void Setup(void);

    // Accelerometer interface
      // Blocking function that reads all 3 axes and performs filtering
      void AccelSample(void);

      // Returns true if accelerometer interrupt is asserted. Currently the
      // accelerometer asserts an interrupt if a "shake" condition is detected.
      uint8_t AccelIsInt(void);

      // Return accelerometer X/Y/Z readings in the range 0-63
      uint8_t AccelResultX(void) { return filter_x.result(); }
      uint8_t AccelResultY(void) { return filter_y.result(); }
      uint8_t AccelResultZ(void) { return filter_z.result(); }

      // Change filtering on accelerometer readings. Filter parameter is in the range
      // 0 to 255 where higher numbers represent more filtering.
      void AccelSetFilterX(uint8_t alpha) { filter_x.set_alpha(alpha); }
      void AccelSetFilterY(uint8_t alpha) { filter_y.set_alpha(alpha); }
      void AccelSetFilterZ(uint8_t alpha) { filter_z.set_alpha(alpha); }

      // Return true if a shake condition was recorded since the last time this function was called
      uint8_t AccelShake(void) { uint8_t retval = is_shake; is_shake=0; return retval; }

      // Return true if a tap condition was recorded since the last time this function was called
      uint8_t AccelTap(void) { uint8_t retval = is_tap; is_tap=0; return retval; }

      // Return current portrait/landscape orientation (see MMA7660FC documentation)
      uint8_t AccelPoLa(void) { return m_pola; }

      // Return current back/front orientation (see MMA7660FC documentation)
      uint8_t AccelBaFro(void) { return m_bafro; }

      // Change the debounce parameter for tap/point detection.
      void AccelSetDebounce(uint8_t val);

    // General-purpose LED interface
      // Turn the given LED (which is 0 to 3) on or off (ison==1 to turn on)
      void LED(uint8_t which, uint8_t ison);

      // Slightly simpler functions for the above
      void LEDSet(uint8_t which) { LED(which, 1); }
      void LEDClear(uint8_t which) { LED(which, 0); }

    // General-purpose switch interface
      // Returns true if the given switch (which is 0 or 1) is pressed
      uint8_t IsSwitch(uint8_t which);

    // RGB LED interface
      // Set the brightness of each RGB LED (parameters are in the range 0 - 255).
      // NOTE: Brightness levels of 16 or lower are recommended. The RGB LED is
      // *VERY BRIGHT* and at high levels should not be looked at directly.
      void RGB(uint8_t r, uint8_t g, uint8_t b);

    // Speaker interface
      // Turn the speaker on with the given frequency in Hertz. Specify 'freq==0'
      // to turn the speaker off.
      void Speaker(uint16_t freq);

    // Potentiometer interface
      // Return potentiometer setting in the range 0 - 255
      uint8_t Pot(void);

    // Microphone interface
      // Return microphone sample in the range [-128,127].
      int8_t Microphone(void); // Signed value!

      // Return absolute value of Microphone()
      uint8_t MicrophoneAmplitude(void);

      // Take 64 samples of the microphone and average them. This average reading
      // is then subtracted out of all future calls to Microphone() and MicrophoneAmplitude().
      // Thus, this function should be called when the microphone is in a "quiet" condition.
      void MicrophoneSetBaseline(void);

    // Light sensor interface
      // Return light sensor reading in the range 0 - 255. NOTE: Lower readings
      // indicate more light.
      uint8_t LightSensor(void);

    // IR receiver interface
      // Return true if the IR logic detector is detecting 38 kHz IR energy
      uint8_t IRReceive(void) { return digitalRead(IRIN_PIN)==LOW; }

      // Turn on IR transmitter at 38 kHz (if ison is non-zero) at the given
      // duty cycle (dc is in the range 1 - 99).
      void IRTransmit(uint8_t ison, uint8_t dc);

      // Blocking function that waits until the given number of 38 kHz pulses
      // have been transmitted. Useful for constructing precise remote control transmissions.
      void IRTransmitCountPulses(uint16_t numPulses);

      // Fast enable/disable of IR transmitter without changing timer parameters.
      void IRTransmitEnable(uint8_t ison);

    // Serial debugging...some functions use Serial.print() for debugging if this is enabled
      void SetSerialDebug(uint8_t ison) { serialDebug = ison; }
};

extern GadgetShield GS;

#endif // _GADGETSHIELD_H_
// vim: expandtab ts=2 sw=2 ai cindent

