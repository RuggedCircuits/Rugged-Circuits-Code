/* 
   This application illustrates the Speaker() function of the Gadget Shield. It
   takes the list of notes stored in the Notes[] array created by the user (see
   below) and plays them one at a time.
  
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
#include <avr/pgmspace.h>
#include <GadgetShield.h>

// How much silent time (in milliseconds) to leave between notes
#define MARCATO_MS  1

// How fast to play...the base note time is this many milliseconds. The 'duration'
// field of each Note[] element is multiplied by this number to get total note
// duration. A duration of 0 ends the song.
#define BASE_TEMPO_MS 60

/* Each note is stored in a 16-bit field:
 
   Lower 6 bits: note number from 0 to 62, and 63 means "silence" (i.e., a rest)
   Upper 10 bits: duration, as a multiple of BASE_TEMP_MS (or 0 to indicate end-of-song)
*/
#define MAKE_NOTE(note,dur) ((dur)<<6 | (note))
prog_uint16_t Notes[] PROGMEM = {
  MAKE_NOTE(19, 1), MAKE_NOTE(18, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1),
  MAKE_NOTE(15, 1), MAKE_NOTE(20, 1), MAKE_NOTE(19, 1), MAKE_NOTE(18, 1),

  MAKE_NOTE(19, 1), MAKE_NOTE(18, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1),
  MAKE_NOTE(15, 1), MAKE_NOTE(16, 1), MAKE_NOTE(17, 1), MAKE_NOTE(18, 1),

  MAKE_NOTE(19, 1), MAKE_NOTE(18, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1),
  MAKE_NOTE(15, 1), MAKE_NOTE(20, 1), MAKE_NOTE(19, 1), MAKE_NOTE(18, 1),

  MAKE_NOTE(19, 1), MAKE_NOTE(18, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1),
  MAKE_NOTE(15, 1), MAKE_NOTE(16, 1), MAKE_NOTE(17, 1), MAKE_NOTE(18, 1),
/////////////
  MAKE_NOTE(19, 1), MAKE_NOTE(18, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1),
  MAKE_NOTE(17, 1), MAKE_NOTE(16, 1), MAKE_NOTE(15, 1), MAKE_NOTE(14, 1),

  MAKE_NOTE(15, 1), MAKE_NOTE(16, 1), MAKE_NOTE(17, 1), MAKE_NOTE(18, 1),
  MAKE_NOTE(19, 1), MAKE_NOTE(20, 1), MAKE_NOTE(19, 1), MAKE_NOTE(18, 1),

  MAKE_NOTE(19, 1), MAKE_NOTE(18, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1),
  MAKE_NOTE(17, 1), MAKE_NOTE(16, 1), MAKE_NOTE(15, 1), MAKE_NOTE(14, 1),

  MAKE_NOTE(15, 1), MAKE_NOTE(16, 1), MAKE_NOTE(17, 1), MAKE_NOTE(18, 1),
  MAKE_NOTE(19, 1), MAKE_NOTE(21, 1), MAKE_NOTE(22, 1), MAKE_NOTE(23, 1),
/////////////
  MAKE_NOTE(24, 1), MAKE_NOTE(23, 1), MAKE_NOTE(22, 1), MAKE_NOTE(21, 1),
  MAKE_NOTE(20, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),

  MAKE_NOTE(24, 1), MAKE_NOTE(23, 1), MAKE_NOTE(22, 1), MAKE_NOTE(21, 1),
  MAKE_NOTE(20, 1), MAKE_NOTE(21, 1), MAKE_NOTE(22, 1), MAKE_NOTE(23, 1),

  MAKE_NOTE(24, 1), MAKE_NOTE(23, 1), MAKE_NOTE(22, 1), MAKE_NOTE(21, 1),
  MAKE_NOTE(20, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),

  MAKE_NOTE(24, 1), MAKE_NOTE(23, 1), MAKE_NOTE(22, 1), MAKE_NOTE(21, 1),
  MAKE_NOTE(20, 1), MAKE_NOTE(21, 1), MAKE_NOTE(22, 1), MAKE_NOTE(23, 1),
/////////////
  MAKE_NOTE(24, 1), MAKE_NOTE(23, 1), MAKE_NOTE(22, 1), MAKE_NOTE(21, 1),
  MAKE_NOTE(22, 1), MAKE_NOTE(21, 1), MAKE_NOTE(20, 1), MAKE_NOTE(19, 1),

  MAKE_NOTE(20, 1), MAKE_NOTE(21, 1), MAKE_NOTE(22, 1), MAKE_NOTE(23, 1),
  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),

  MAKE_NOTE(24, 1), MAKE_NOTE(23, 1), MAKE_NOTE(22, 1), MAKE_NOTE(21, 1),
  MAKE_NOTE(22, 1), MAKE_NOTE(21, 1), MAKE_NOTE(20, 1), MAKE_NOTE(19, 1),

  MAKE_NOTE(20, 1), MAKE_NOTE(21, 1), MAKE_NOTE(22, 1), MAKE_NOTE(23, 1),
  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),
/////////////
  MAKE_NOTE(8, 1), MAKE_NOTE(12, 1), MAKE_NOTE(8, 1), MAKE_NOTE(12, 1),
  MAKE_NOTE(8, 1), MAKE_NOTE(12, 1), MAKE_NOTE(8, 1), MAKE_NOTE(12, 1),

  MAKE_NOTE(11, 1), MAKE_NOTE(13, 1), MAKE_NOTE(11, 1), MAKE_NOTE(13, 1),
  MAKE_NOTE(11, 1), MAKE_NOTE(13, 1), MAKE_NOTE(11, 1), MAKE_NOTE(13, 1),

  MAKE_NOTE(11, 1), MAKE_NOTE(12, 1), MAKE_NOTE(11, 1), MAKE_NOTE(12, 1),
  MAKE_NOTE(11, 1), MAKE_NOTE(12, 1), MAKE_NOTE(11, 1), MAKE_NOTE(12, 1),

  MAKE_NOTE(11, 1), MAKE_NOTE(13, 1), MAKE_NOTE(11, 1), MAKE_NOTE(13, 1),
  MAKE_NOTE(11, 1), MAKE_NOTE(13, 1), MAKE_NOTE(11, 1), MAKE_NOTE(13, 1),
/////////////
  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),
  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),

  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),
  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(24, 1), MAKE_NOTE(23, 1),

  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(26, 1), MAKE_NOTE(27, 1),
  MAKE_NOTE(28, 1), MAKE_NOTE(27, 1), MAKE_NOTE(26, 1), MAKE_NOTE(25, 1),

  MAKE_NOTE(24, 1), MAKE_NOTE(25, 1), MAKE_NOTE(26, 1), MAKE_NOTE(27, 1),
  MAKE_NOTE(28, 1), MAKE_NOTE(27, 1), MAKE_NOTE(26, 1), MAKE_NOTE(25, 1),
/////////////
  MAKE_NOTE(12, 1), MAKE_NOTE(17, 1), MAKE_NOTE(12, 1), MAKE_NOTE(17, 1),
  MAKE_NOTE(12, 1), MAKE_NOTE(17, 1), MAKE_NOTE(12, 1), MAKE_NOTE(17, 1),

  MAKE_NOTE(16, 1), MAKE_NOTE(18, 1), MAKE_NOTE(16, 1), MAKE_NOTE(18, 1),
  MAKE_NOTE(16, 1), MAKE_NOTE(18, 1), MAKE_NOTE(16, 1), MAKE_NOTE(18, 1),

  MAKE_NOTE(16, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1), MAKE_NOTE(17, 1),
  MAKE_NOTE(16, 1), MAKE_NOTE(17, 1), MAKE_NOTE(16, 1), MAKE_NOTE(17, 1),

  MAKE_NOTE(16, 1), MAKE_NOTE(18, 1), MAKE_NOTE(16, 1), MAKE_NOTE(18, 1),
  MAKE_NOTE(16, 1), MAKE_NOTE(18, 1), MAKE_NOTE(16, 1), MAKE_NOTE(18, 1),
/////////////
  MAKE_NOTE(29, 1), MAKE_NOTE(30, 1), MAKE_NOTE(29, 1), MAKE_NOTE(28, 1),
  MAKE_NOTE(29, 1), MAKE_NOTE(30, 1), MAKE_NOTE(29, 1), MAKE_NOTE(28, 1),

  MAKE_NOTE(29, 1), MAKE_NOTE(30, 1), MAKE_NOTE(29, 1), MAKE_NOTE(28, 1),
  MAKE_NOTE(29, 1), MAKE_NOTE(30, 1), MAKE_NOTE(29, 1), MAKE_NOTE(28, 1),

  MAKE_NOTE(29, 1), MAKE_NOTE(30, 1), MAKE_NOTE(31, 1), MAKE_NOTE(32, 1),
  MAKE_NOTE(33, 1), MAKE_NOTE(32, 1), MAKE_NOTE(31, 1), MAKE_NOTE(30, 1),

  MAKE_NOTE(29, 1), MAKE_NOTE(30, 1), MAKE_NOTE(31, 1), MAKE_NOTE(32, 1),
  MAKE_NOTE(33, 1), MAKE_NOTE(32, 1), MAKE_NOTE(31, 1), MAKE_NOTE(30, 1),
/////////////
  MAKE_NOTE(29, 1), MAKE_NOTE(28, 1), MAKE_NOTE(27, 1), MAKE_NOTE(26, 1),
  MAKE_NOTE(25, 1), MAKE_NOTE(30, 1), MAKE_NOTE(29, 1), MAKE_NOTE(28, 1),

  MAKE_NOTE(29, 1), MAKE_NOTE(28, 1), MAKE_NOTE(27, 1), MAKE_NOTE(26, 1),
  MAKE_NOTE(25, 1), MAKE_NOTE(26, 1), MAKE_NOTE(27, 1), MAKE_NOTE(28, 1),

  MAKE_NOTE(29, 1), MAKE_NOTE(40, 1), MAKE_NOTE(39, 1), MAKE_NOTE(38, 1),
  MAKE_NOTE(39, 1), MAKE_NOTE(38, 1), MAKE_NOTE(37, 1), MAKE_NOTE(36, 1),

  MAKE_NOTE(37, 1), MAKE_NOTE(38, 1), MAKE_NOTE(39, 1), MAKE_NOTE(40, 1),
  MAKE_NOTE(39, 1), MAKE_NOTE(40, 1), MAKE_NOTE(41, 1), MAKE_NOTE(42, 1),
/////////////
  MAKE_NOTE(43, 1), MAKE_NOTE(42, 1), MAKE_NOTE(41, 1), MAKE_NOTE(40, 1),
  MAKE_NOTE(41, 1), MAKE_NOTE(40, 1), MAKE_NOTE(39, 1), MAKE_NOTE(38, 1),

  MAKE_NOTE(39, 1), MAKE_NOTE(38, 1), MAKE_NOTE(37, 1), MAKE_NOTE(36, 1),
  MAKE_NOTE(35, 1), MAKE_NOTE(34, 1), MAKE_NOTE(33, 1), MAKE_NOTE(32, 1),

  MAKE_NOTE(31, 1), MAKE_NOTE(32, 1), MAKE_NOTE(31, 1), MAKE_NOTE(30, 1),
  MAKE_NOTE(31, 1), MAKE_NOTE(32, 1), MAKE_NOTE(31, 1), MAKE_NOTE(30, 1),

  MAKE_NOTE(31, 1), MAKE_NOTE(32, 1), MAKE_NOTE(33, 1), MAKE_NOTE(34, 1),
  MAKE_NOTE(35, 1), MAKE_NOTE(36, 1), MAKE_NOTE(37, 1), MAKE_NOTE(38, 1),
/////////////
  MAKE_NOTE(39, 1), MAKE_NOTE(40, 1), MAKE_NOTE(41, 1), MAKE_NOTE(42, 1),
  MAKE_NOTE(43, 1), MAKE_NOTE(45, 1), MAKE_NOTE(46, 1), MAKE_NOTE(47, 1),

  MAKE_NOTE(48, 2), MAKE_NOTE(63, 6),

  MAKE_NOTE(36, 2), MAKE_NOTE(63, 6),

  MAKE_NOTE(12, 2),

  0
};

// Convert note number to physical frequencies. This mapping uses 110Hz as
// note 0.
uint16_t note2freq[64] = {
  110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208,
  220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415,
  440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831,
  880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661,
  1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322,
  3520, 3729, 3951, 0/*63==silence*/ };

// Which note is currently playing
unsigned note_index;

// When to play the next note
unsigned long millis_at_next_note;

// Is the next event a between-note silence?
unsigned marcato_time;

uint8_t lastLed;
void setup() {
  GS.Setup();
  note_index = 0;
  marcato_time = 0;
  millis_at_next_note = millis() + 100; // Start playing in 0.1s
  lastLed = 0;
}

// Just a little "light show" to play along with the music. The higher
// the frequency the "higher up" the LED's go, with a little randomness
// thrown in so things don't get too boring.
void rand_led(uint8_t note)
{
  GS.LED(lastLed, 0);
  if (note==63) return;

  if (note <= 12) {
    lastLed = (random(4) >= 2) ? 1 : 0;
  } else if (note <= 24) {
    switch (random(8)) {
      case 7: case 6: lastLed = 2; break;
      case 5: case 4: lastLed = 0; break;
      default: lastLed = 1; break;
    }
  } else if (note <= 36) {
    switch (random(8)) {
      case 7: case 6: lastLed = 1; break;
      case 5: case 4: lastLed = 3; break;
      default: lastLed = 2; break;
    }
  } else {
    lastLed = (random(4) >= 2) ? 2 : 3;
  }
  GS.LED(lastLed, 1);
}

void loop() {
  uint8_t note;
  uint16_t duration;

  if (millis() >= millis_at_next_note) {
    if (marcato_time) {
      GS.Speaker(0);
      marcato_time = 0;
      millis_at_next_note += MARCATO_MS;
    } else {
      duration = pgm_read_word(Notes + note_index);
      note_index++;

      note = (uint8_t) (duration & 0x3F);

      if (note==0) {
        GS.LED(lastLed, 0);
        while (1) /* NULL */ ;
      }

      GS.Speaker(note2freq[note]);
      rand_led(note);

      duration >>= 6;
      millis_at_next_note += BASE_TEMPO_MS * (uint32_t)duration - MARCATO_MS;
      marcato_time = 1;
    }
  }
}
// vim: syntax=cpp ai cindent sw=2 ts=2 expandtab
