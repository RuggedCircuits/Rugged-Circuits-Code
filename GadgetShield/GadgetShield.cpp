/*
  GadgetShield.cpp - Library for Rugged Circuits Gadget Shield
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

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include "gs_twi.h"
}

#include "GadgetShield.h"

// Accelerometer I2C slave address (fixed...do not change)
#define ACC_SLAVE_ADDR 0x4C

// Class Variables //////////////////////////////////////////////////


// Constructors ////////////////////////////////////////////////////////////////

GadgetShield::GadgetShield()
{
  blue_duty_cycle = 0; // Used for Blue channel of RGB LED
  numValid = 0;  // Used for low-level accelerometer accesses
}

void GadgetShield::Setup(void)
{
  // Configure infrared emitter and detector
  pinMode(IRIN_PIN, INPUT);     // There is already an external pullup in the detector itself but it
  digitalWrite(IRIN_PIN, HIGH); // doesn't hurt to add our own pullup as the detector's is weak.
  pinMode(IROUT_PIN, OUTPUT);
  digitalWrite(IROUT_PIN, LOW);

  // Configure general-purpose LED's
  pinMode(LED1_PIN, OUTPUT); pinMode(LED2_PIN, OUTPUT); pinMode(LED3_PIN, OUTPUT); pinMode(LED4_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW); digitalWrite(LED2_PIN, LOW); digitalWrite(LED3_PIN, LOW); digitalWrite(LED4_PIN, LOW);

  // Configure RGB LED
  pinMode(RED_PIN, OUTPUT); pinMode(GREEN_PIN, OUTPUT); pinMode(BLUE_PIN, OUTPUT);
  RGB(0,0,0);

  // Configure pushbuttons
  pinMode(PB1_PIN, INPUT); // There are already external pullups so no internal pullup is needed
  pinMode(PB2_PIN, INPUT);

  // Configure speaker
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);
  Speaker(1000); // Just to initialize timer infrastructure for speaker and Blue RGB LED pin
  Speaker(0);

  // Configure TWI interface to accelerometer and restore to default settings
  twi_init();

  // Initialize accelerometer
  accel_init();

  // Assume microphone baseline is 2.5V. Establish experimentally by calling MicrophoneSetBaseline().
  m_mic_baseline = 127;
}

// Internal function for debugging TWI communication problems
void GadgetShield::errorWrite(uint8_t code)
{
  if (serialDebug) {
    switch (code)  {
      case 0:  // OK
        break;

      case 1:
        Serial.println("TWI: Length too long");
        break;

      case 2:
        Serial.println("NAK on address send");
        break;

      case 3:
        Serial.println("NAK on data send");
        break;

      default:
        Serial.println("Unknown TWI error");
        break;
    }
  }
}

// Helper function to write to the accelerometer and interpret the error code, if any
void GadgetShield::twiWrite(const void *buf, uint8_t length)
{
  uint8_t code;

  code = twi_writeTo(ACC_SLAVE_ADDR, (uint8_t *)buf, length, 1);
  errorWrite(code);
}

// Set initial accelerometer parameters....modify as appropriate for your design.
void GadgetShield::accel_init(void)
{
  twiWrite("\x07\x00", 2); // Access mode register, place device in standby mode so we can modify other regs
  twiWrite(
      "\x05"       // Set address of sleep count register
      "\x00"       // Reset sleep count register to 0
      "\xE0"       // Enable shake interrupts (X/Y/Z)
      "\x00"       // Write 0 to mode register again, just so we don't start another write cycle
      "\x00"       // Reset all filtering, auto-wake averaging, and set 120 samples/s
      "\x00"       // Reset tap-pulse detection register to threshold of 1, all axes enabled for tap
      "\x60"       // Set tap-pulse debounce count register to 96 (seems to work well)
      , 7);

  twiWrite("\x07\x41", 2); // Enter active mode, set interrupt output to push-pull
}

// Internal helper function to convert signed two's complement 6-bit
// accelerometer readings to unsigned values in the range [0,63].
uint8_t GadgetShield::s2u(uint8_t val)
{
  val &= 0x3F; 
  if (val > 31) {
    return (val - 32);
  } else {
    return (val + 32);
  }
}

// Internal function to query the accelerometer for registers 0-3 and process.
// FYI, PoLa and BaFro are nomenclature from the MMA7660FC datasheet itself,
// representing Portrait/Landscape and Back/Front orientations, respectively.
uint8_t GadgetShield::readAxes(uint8_t *x, uint8_t *y, uint8_t *z, uint8_t *shake, uint8_t *tap, uint8_t *PoLa, uint8_t *BaFro)
{
  uint8_t len, code;

  // Upon wanting to read an entirely new set of values, set valid[] to all 0xFF to
  // indicate invalid values.
  if (numValid == 0) {
    memset(valid, 0xFF, sizeof(valid)); // 0xFF has bit 6 set which is the alert flag
  }

  if (0==(code=twi_writeToReadFrom(ACC_SLAVE_ADDR, (uint8_t *)"\x00", 1, val, 4, &len))) {
    if (len==4) {
      uint8_t i;

      numValid = 0;

      for (i=0; i < 4; i++) {
        if (! (val[i] & 0x40)) { // If alert flag is not set, update valid reading
          valid[i] = val[i];
        }
        if (valid[i] != (uint8_t)0xFF) { // See how many valid readings we have, either new or from last time
          numValid++;
        }
      }
      
      if (numValid == 4) {
        numValid = 0;
        if (x) *x = s2u(valid[0]);
        if (y) *y = s2u(valid[1]);
        if (z) *z = s2u(valid[2]);
        if (shake) *shake = (valid[3] & 0x80) ? 1 : 0;
        if (tap) *tap = (valid[3] & 0x20) ? 1 : 0;
        if (PoLa) *PoLa = (valid[3] >> 2) & 0x7;
        if (BaFro) *BaFro = (valid[3] & 0x3);
        return 1;
      }
    }
  } else {
    errorWrite(code);
  }

  return 0;
}

// Example function for modifying accelerometer registers. This function sets the
// debounce for tap/pulse detection.
void GadgetShield::AccelSetDebounce(uint8_t val)
{
  uint8_t buf[2];

  // No need to wait to write to mode register
  twiWrite("\x07\x00", 2); // Standby mode...required for modifying registers
  buf[0] = '\x0A'; // Tap/Pulse Debounce register
  buf[1] = val;
  twiWrite(buf, 2);
  twiWrite("\x07\x41", 2); // Active mode
  //setTWITimeout();
}

// Public Methods //////////////////////////////////////////////////////////////
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
// See comments for Speaker() and RGB() function for why we use an interrupt
// to control the Blue channel of the RGB LED.
ISR(TIMER2_COMPB_vect)
{
  // Manually clear OC2A (Blue) on OC2B compare
  TCCR2A &= ~_BV(COM2A0); // Make OC2A go low on compare
  TCCR2B |= _BV(FOC2A);   // Force a compare (this makes OC2A go low)
  TCCR2A |= _BV(COM2A0);  // Make OC2A go high on compare
}
#endif

// Drive the speaker with the given frequency in Hertz
void GadgetShield::Speaker(uint16_t freq)
{
  /* We set a target frequency range of 0-3 kHz as the speaker isn't very responsive beyond that. Pin 9
     is a different timer on different processors:
        * TIMER1A on ATmega328P
        * TIMER2B on ATmega2560/1280

     For TIMER1A we want fast PWM mode (WGM13:0=1111) with OCR1A as the TOP
     value and we configure COM1A1:0=01 so that OC1A toggles on each compare
     match. Thus we want OCR1A to be half the desired target period.

     Here is how we choose prescale (we want the lowest prescale for maximum resolution):
        Prescale=1 (16 MHz): freq = 123 Hz (OCR1A=65040) to 3 kHz (OCR1A=5333)
        Prescale=8 (2 MHz): freq = 16 Hz (OCR1A=62500) to 123 Hz (OCR1A=8130)

     Guiding equation:
              OCR1A = (F_CPU/Prescale)/TargetFreq/2 - 1 = F_CPU/(Prescale*TargetFreq*2) - 1
  */
  uint32_t divisor;
  uint16_t ocrval;

#ifdef __AVR_ATmega328P__
  if (freq == 0) {
    TCCR1A &= ~_BV(COM1A0); // Disconnect output compare from pin
    digitalWrite(SPEAKER_PIN, LOW);
  } else {
    TCCR1A = (TCCR1A & ~(_BV(COM1A1))) | _BV(COM1A0) | _BV(WGM11) | _BV(WGM10); // Set output mode 01, toggle OC1A on compare match, waveform generation mode 1111 (Fast PWM, OCR1A as top)
    TCNT1=0;
    if (freq >= 123) {
      divisor = 1*freq*2;
      TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Divide-by-1 prescale, Fast PWM mode (1111)
    } else {
      divisor = 8*freq*2;
      TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // Divide-by-8 prescale, Fast PWM mode (1111)
    }
    // OCR1A sets the frequency and the Speaker pin toggles on OC1A.
    // 
    // Now, for the Blue LED (OC1B), we set it up such that OC1B is cleared on
    // every compare (COM1B1:0=10) and is set to 1 at BOTTOM.
    ocrval = (uint16_t)((F_CPU + divisor/2) / divisor) - 1;
    OCR1A = ocrval; // Not updated until BOTTOM so save it in a variable for OCR1B computation

    // Preserve duty cycle on OC1B since that's D10, the blue part of the RGB LED
    OCR1B = (uint16_t) (((uint32_t)blue_duty_cycle*ocrval) >> 8);
  }
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  /* For TIMER2B we want CTC mode (WGM22:0=010) with OCR2A as the TOP
     value and we configure COM2B1:0=01 so that OC2B toggles on each compare
     match. Thus we want OCR2A to be half the desired target period.

     For 8-bit Timer 2, here is how we choose prescale:
        Prescale=32 (500 kHz):  freq = 977 Hz (OCR2A = 254) to 3 kHz (OCR2A = 82)
        Prescale=64 (250 kHz):  freq = 489 Hz (OCR2A = 254) to 977 Hz (OCR2A = 126)
        Prescale=128 (125 kHz): freq = 244 Hz (OCR2A = 255) to 489 Hz (OCR2A = 126)
        Prescale=256 (62.5 kHz): freq = 123 Hz (OCR2A = 253) to 244 Hz (OCR2A = 127)
        Prescale=1024 (15.625 kHz): freq = 31 Hz (OCR2A = 251) to 123 Hz (OCR2A = 62)

     Since OC2A is the Blue channel of the RGB LED, we implement a poor-man's PWM mode.
     We set COM2A1:0=11 to set OC2A at the timer TOP value (OCR2A) and we clear OC2A
     in an interrupt service routine for a compare on OCR2B. Thus OCR2B sets the Blue
     channel duty cycle, but it actually controls the OC2A pin.
  */
  if (freq == 0) {
    TCCR2A &= ~_BV(COM2B0);
    digitalWrite(SPEAKER_PIN, LOW);
  } else {
    TCCR2A = (TCCR2A & ~(_BV(COM2B1)|_BV(WGM20))) | _BV(COM2B0) | _BV(WGM21); // Set output mode 01, toggle OC2B on compare match, waveform generation mode 010 (CTC)
    if (freq >= 977) {
      divisor = 32UL*freq*2;
      TCCR2B = _BV(CS21) | _BV(CS20);
    } else if (freq >= 489) {
      divisor = 64UL*freq*2;
      TCCR2B = _BV(CS22);
    } else if (freq >= 244) {
      divisor = 128UL*freq*2;
      TCCR2B = _BV(CS22) | _BV(CS20);
    } else if (freq >= 123) {
      divisor = 256UL*freq*2;
      TCCR2B = _BV(CS22) | _BV(CS21);
    } else {
      divisor = 1024UL*freq*2;
      TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
    }
    // OCR2A sets the frequency, but the Speaker pin toggles on OC2B. It
    // doesn't matter what OC2B is, however, as long as it's in the range
    // [0,OCR2A]. Once every period OC2B toggles and the speaker has a 50% duty
    // cycle waveform at the frequency set by OCR2A. 
    // 
    // Now, for the Blue LED, we set it up such that OC2A is automatically set
    // to 1 on every compare (COM2A1:0=11) and is cleared in an ISR when we
    // have a compare match on OCR2B.
    OCR2A = (uint8_t) ((F_CPU + divisor/2) / divisor) - 1;
    OCR2B = (uint8_t) (((uint16_t)blue_duty_cycle*(OCR2A+1)) >> 8);
  }
#endif
}

// Set the RGB colors according to brightness levels in the range 0 to 255
void GadgetShield::RGB(uint8_t r, uint8_t g, uint8_t b)
{
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  
  // Must handle BLUE pin specially since it shares a timer with the Speaker
  blue_duty_cycle = b;
  if (b==0) {
#ifdef __AVR_ATmega328P__
    TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0)); // Disconnect output compare from Blue pin
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0)); // Disconnect output compare from Blue pin
    TIFR2 = _BV(OCF2B);     // Clear and disable the OCR2B ISR
    TIMSK2 &= ~_BV(OCIE2B);
#endif
    digitalWrite(BLUE_PIN, 0);
  } else {
#ifdef __AVR_ATmega328P__
    TCCR1A |= _BV(COM1B1); // Clear OC1B on compare match, set it at BOTTOM
    OCR1B = (uint16_t) (((uint32_t)blue_duty_cycle*OCR1A) >> 8);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR2A |= _BV(COM2A1) | _BV(COM2A0); // Set OC2A on compare match
    TIFR2 = _BV(OCF2B);
    TIMSK2 |= _BV(OCIE2B); // Interrupt on OCR2B match (manually clear OC2A in the ISR)
    OCR2B = (uint8_t) (((uint16_t)blue_duty_cycle*OCR2A) >> 8);
#endif
  }
}

// Turn on/off one of the 4 general-purpose LED's (0 to 3).
void GadgetShield::LED(uint8_t which, uint8_t ison)
{
  uint8_t pin;

  switch (which) {
    case 0: pin=LED1_PIN; break;
    case 1: pin=LED2_PIN; break;
    case 2: pin=LED3_PIN; break;
    case 3: pin=LED4_PIN; break;
    default: return;
  }
  digitalWrite(pin, ison ? HIGH : LOW);
}

// Return true (non-zero) if the given switch is pressed (switch is 0 or 1)
uint8_t GadgetShield::IsSwitch(uint8_t which)
{
  return ! (which ? digitalRead(PB2_PIN) : digitalRead(PB1_PIN));
}

// Return the potentiometer setting in the range 0-255
uint8_t GadgetShield::Pot(void)
{
  return analogRead(POT_PIN) >> 2;
}

// Return the microphone amplitude in the range [-128,127] after subtracting
// out the baseline reading. So if the baseline has been established with
// MicrophoneSetBaseline(), this function should return near 0 when there is
// quiet. Otherwise positive and negative values will be returned to represent
// the sound oscillation.
int8_t GadgetShield::Microphone(void)
{
  return (int8_t)(analogRead(MICROPHONE_PIN) >> 2) - m_mic_baseline;
}

// Listen to the microphone for 64 samples on the assumption that everything
// is quiet. The average reading is then set as the baseline and is subtracted
// out from future calls to Microphone() and MicrophoneAmplitude().
void GadgetShield::MicrophoneSetBaseline(void)
{
  uint8_t sample;
  uint16_t sum;

  for (sample=0, sum=0; sample < 64; sample++) {
    sum += analogRead(MICROPHONE_PIN);
  }
  // Shift right by 6 to divide by 64 and get an average. Shift right again by 2
  // to store as an 8-bit number since that's how we process microphone readings.
  // The additions +32 and +2 are for rounding.
  m_mic_baseline = (uint8_t) ((((sum+32) >> 6)+2) >> 2);
}

// Simply return the amplitude of the microphone reading. Useful for detecting
// minimum sound levels.
uint8_t GadgetShield::MicrophoneAmplitude(void)
{
  int8_t val;

  val = Microphone();
  return abs(val);
}

// Return the magnitude of the light sensor in the range [0,255]. Lower
// readings represent more light.
uint8_t GadgetShield::LightSensor(void)
{
  return analogRead(LIGHTSENSOR_PIN) >> 2;
}

// Poll the interrupt output of the accelerometer. This can represent various
// things, depending on how the accelerometer was configured. By default it is
// configured to interrupt when it detects a "shake" condition. Note that
// interrupts are cleared whenever the accelerometer is accessed over I2C.
uint8_t GadgetShield::AccelIsInt(void)
{
  return digitalRead(ACC_INTn_PIN)==LOW;
}

// Take a reading of the accelerometer axes. This is a blocking function
// that keeps reading the accelerometer until valid values are obtained,
// with 30ms breaks in between samples if necessary. The X/Y/Z axis
// readings are IIR-filtered to smooth out spikes.
void GadgetShield::AccelSample(void)
{
  uint8_t x, y, z, shake, tap;

  do {
    if (readAxes(&x, &y, &z, &shake, &tap, &m_pola, &m_bafro)) {
      is_shake |= shake;
      is_tap |= tap;
      filter_x.input(x);
      filter_y.input(y);
      filter_z.input(z);
      break;
    }
    delay(30);
  } while (1);
}

// Blocking function that waits until the given number of 38 kHz pulses
// have been transmitted on the IR transmitter. This function is useful
// for generating precise remote control transmissions.
void GadgetShield::IRTransmitCountPulses(uint16_t numPulses)
{
#ifdef __AVR_ATmega328P__
	do {
    // OC2B is IR output, and OC2A is used as TOP value
		TIFR2 = _BV(OCF2A);
		while ( (TIFR2 & _BV(OCF2A)) == 0) /* NULL */ ;
	} while (--numPulses);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	do {
    // OC3C is IR output, and ICR3 is used as TOP value
		TIFR3 = _BV(ICF3);
		while ( (TIFR3 & _BV(ICF3)) == 0) /* NULL */ ;
	} while (--numPulses);
#endif
}

// Quickly turn on or off the IR transmitter without changing any
// of the timer parameters.
void GadgetShield::IRTransmitEnable(uint8_t ison)
{
#ifdef __AVR_ATmega328P__
  TCCR2A = (ison ? _BV(COM2B1) : 0) | _BV(WGM20);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  TCCR3A = (ison ? _BV(COM3C1) : 0) | _BV(WGM31);
#endif
}

// Start/stop IR transmission at 38 kHz with the given duty cycle.
void GadgetShield::IRTransmit(uint8_t ison, uint8_t dc)
{
  // No reason for 100% or 0% duty cycles. Just set 'ison==0' for 0%.
  if (dc > 99) dc=99;
  if (dc < 1) dc=1;

#ifdef __AVR_ATmega328P__
  /* OC2B is our IR emitter. Turn it on at 38 kHz and given duty cycle.

     Use phase correct PWM mode (WGM22:0=101), set OC2B when downcounting and
     clear when upcounting (mode COM2B1:0=10).

     Use prescale of 1 (16 MHz) so that 38 kHz is given by an OCR2A of:

         OCR2A = F_CPU/(2*1*38kHz) = 210 (actually 38.095 kHz)
  */
  OCR2A = (uint8_t) (F_CPU/(2*1*38000UL)) - 1;
  OCR2B = (uint8_t) ((dc*(uint16_t)(OCR2A+1) + 50)/100);
  TCCR2A = (ison ? _BV(COM2B1) : 0) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS20);
  TCNT2 = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  /* OC3C is our IR emitter. Turn it on at 38 kHz and given duty cycle.

     Use phase-correct PWM mode (WGM33:0=1010) with ICR3 as top (OCR3A is used
     for PWM on pin 5), set OC3C when downcounting and clear when upcounting
     (mode COM3C1:0=10).

     Use prescale of 1 (16 MHz) so that 38 kHz is given by an ICR3 of:

         ICR3 = F_CPU/(2*1*38kHz) = 210  (actually 38.095 kHz)
   */
  ICR3 = (uint16_t) (F_CPU/(2*1*38000UL)) - 1;
  OCR3C = (uint16_t) ((dc*(ICR3+1)+50)/100) - 1;
  TCCR3A = (ison ? _BV(COM3C1) : 0) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(CS30);
  TCNT3 = 0;
#endif
}

// Preinstantiate Objects //////////////////////////////////////////////////////

GadgetShield GS = GadgetShield();

// vim: expandtab ts=2 sw=2 ai cindent
