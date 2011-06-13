/* 
   This Gadget Shield application illustrates the decoding of an infrared
   remote control. The application learns the pattern of a remote control
   transmission then is capable of reproducing it on demand, or indicating when
   it happens again.  This application is often called a "universal remote
   control" as it can mimic a wide variety of remote control models. Encodings
   are stored in EEPROM so that learned patterns persist even when power is
   turned off.

   Transmissions can be sent in response to incoming serial commands. As a
   shortcut, the two Gadget Shield pushbuttons can be used to transmit the
   first two buttons stored in EEPROM.

   For the Duemilanove/Uno, Timer 1 is used for input capture on incoming
   transmissions so the use of the speaker and Blue RGB LED may be affected.
   For the Mega, nothing is affected.

   Application Version 2.1 -- April 2011

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

#include <ctype.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <GadgetShield.h>
 
#if defined(__AVR_ATmega328P__)
#  define BOARD 0
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#  define BOARD 1
#endif

/* Define how many edges per transmission you're allowing.
   Too low a number and you may not be able to discriminate between
   transmissions. Too high a number and you'll run out of RAM. With 2K of RAM
   (ATmega324P/328P) you have room for about 800 edges...more than enough for
   any practical remote control protocol.

   A value of 128 seems to work for just about everything.
*/
#define MAX_EDGES 128

/* Define how many milliseconds to wait for another edge before
   declaring the end of a transmission. 50ms is fairly generous for most remote
   controls.
*/
#define MS_BEFORE_IDLE 50

/* When parameterizing a transmission, this definition sets
   a limit on how many unique edge-to-edge times there can be. Often there are
   only 4-5 unique edge-to-edge times.  If the cluster tolerance (see below) is
   too small then you will see excess codebook entries.

   A value of 8 seems to be generous.
*/
#define CODEBOOK_MAX_CODES 8

/* This definition controls how tightly edge-to-edge times
   can be clustered. The bigger the number, the more tolerant the recognition
   algorithm will be to variations in edge-to-edge timing, but there's a chance
   that what should be unique codes are improperly merged.

   16 seems to work well for just about everything.
*/
#define CODEBOOK_CLUSTER_TOLERANCE 16

/* This definition controls how much extra time to drive the output LED to
 * account for lag in the decoder. That is, edge-to-edge input times are
 * actually artificially too short since the decoder takes a while to recognize
 * the signal. Only very fussy decoders may need a value other than 0.
 */
#define EDGE_TIME_COMPENSATION 0   // Units of 0.1%

/* This definition controls how many extra pulses the IR LED should be given
 * when active, and how many fewer pulses it should be given when inactive.
 * Again, this is to compensate for lag in decoders. Only very fussy decoders
 * may need a value other than 0.
 */
#define EDGE_COUNT_COMPENSATION 0

/**********************************************************/
/* YOU SHOULDN'T HAVE TO CHANGE ANYTHING BELOW THIS POINT */
/**********************************************************/

// Not really a tweakable parameter (unless you modify the GadgetShield class
// IRTransmit() function)
#define F_OUT 38 // kHz

/*********************************************************/
/*                                                       */
/* IR hardware configuration                          */
/*                                                       */
/*********************************************************/

// In case no board is defined, guess from the processor
#ifndef BOARD
#  error "You must define BOARD near the top of this file"
#endif

#if (BOARD!=0) && (BOARD!=1)
#error "Unknown board"
#endif

// Interrupt service routine helper functions
static void handleEdge(uint16_t edgeTime);
static void handleOverflow(void);

//
//
// Routines common to Duemilanove/Uno and Mega
//
//
static void configureInputTimer(uint8_t enable)
{
	// At 16 MHz, use Timer 1 to measure 4us periods with divide-by-64 prescale
  // This gives a 250ms range.
	TCCR1A = 0;  // Normal mode operation, divide-by-64
	TCCR1B = _BV(CS11) | _BV(CS10);

	// Clear pending interrupt
	TIFR1 = _BV(TOV1);

	// Enable interrupt on overflow as that indicates end of reception
	TIMSK1 = enable ? _BV(TOIE1) : 0;

	// Keep IR LED off for now but configure for 50% duty cycle (max power)
  GS.IRTransmit(0,50);
}

// Start TCNT1 at EDGE_START_TIME, and if it overflows after 65535 then MS_BEFORE_IDLE milliseconds
// have gone by and this incoming transmission must be over.
static const unsigned EDGE_START_TIME = (65535U - (unsigned)(250U*MS_BEFORE_IDLE)); // 250 ticks/ms
static void resetTimer(void)
{
	TCNT1 = EDGE_START_TIME;
}
ISR(TIMER1_OVF_vect)
{
	handleOverflow();
}
static void waitForInputTimerOverflow(void)
{
	// Clear pending flag
	TIFR1 = _BV(TOV1);
  while ((TIFR1 & _BV(TOV1)) == 0) /* NULL */ ;
}

static void configureOutputTimer(uint8_t enable)
{
  GS.IRTransmit(enable, 50);
}

static void generateOutput(uint16_t numPulses, uint8_t ledon)
{
  GS.IRTransmitEnable(ledon);
  GS.IRTransmitCountPulses(numPulses);
}

static uint16_t convertOutputPulses(uint16_t numPulses)
{
	// The input parameter is in units of INPUT pulses, i.e., 4us periods.
	// This number then gets scaled by (100+E)% where E is
	// EDGE_TIME_COMPENSATION/10. Thus if EDGE_TIME_COMPENSATION is 10,
	// the total pulse duration should be scaled by 101%.
	uint32_t duration;

	duration = (uint32_t)numPulses * 4; // Units of us
	duration = ((duration * (1000 + EDGE_TIME_COMPENSATION)) + 500) / 1000; // Slightly higher units of us

	// Now we need to divide by (1/F_OUT kHz), or equivalently multiply by F_OUT
	return (uint16_t) ((duration * (uint8_t)F_OUT + 500U) / 1000U);
}

#if BOARD==0 /* Duemilanove: PORTD2 is INT0: use this interrupt. */
static void configureIRInterrupt(uint8_t enable)
{
	// Enable interrupt on both edges
	EICRA &= ~(_BV(ISC01)|_BV(ISC00));
	EICRA |= _BV(ISC00);

	// Clear any pending interrupts
	EIFR = _BV(0);

	// Enable/disable interrupt 0
  if (enable) {
    EIMSK |= _BV(0); 
  } else {
    EIMSK &= ~_BV(0);
  }
}
ISR(INT0_vect)
{
	handleEdge(TCNT1);
}

#elif BOARD==1 /* Mega: PORTE4 is INT4: use this interrupt. */
static void configureIRInterrupt(uint8_t enable)
{
	// Clear any pending interrupts
	EIFR = _BV(4);

	// Enable interrupt on both edges
	EICRB &= ~(_BV(ISC41)|_BV(ISC40));
	EICRB |= _BV(ISC40);

	// Enable interrupt 4
	if (enable) {
		EIMSK |= _BV(4);
	} else {
		EIMSK &= ~_BV(4);
	}
}
ISR(INT4_vect)
{
	handleEdge(TCNT1);
}
#endif

/*********************************************************/
/*                                                       */
/* Interrupt Service Routines                            */
/*                                                       */
/*********************************************************/

/* The gState variable is S_IDLE when waiting for incoming activity, S_ACQUIRING
 * while activity is still detected (at least 1 edge every MS_BEFORE_IDLE milliseconds),
 * and S_DONE when MS_BEFORE_IDLE milliseconds elapses without any activity. The main
 * loop() function notices the S_DONE state, interprets the incoming transmission, and
 * returns the state to S_IDLE.
 */
typedef enum {
	S_IDLE,
	S_ACQUIRING,
	S_DONE
} state_t;
static volatile state_t gState;

/* Incoming edge-to-edge times are stored here in units of 4us */
static volatile uint16_t gEdgeDelta[MAX_EDGES];
static uint8_t gEncoding[MAX_EDGES+1]; // gEdgeDelta values converted according to codebook
static volatile uint8_t gNumEdges;     // How many edges are stored in gEdgeDelta[]

static void allLEDs(uint8_t enable)
{
  GS.LED(0, enable);
  GS.LED(1, enable);
  GS.LED(2, enable);
  GS.LED(3, enable);
}

// This is called in an ISR in response to either a rising or falling edge. It simply
// stores the time between the previous edge and this edge in the gEdgeDelta[] array.
static void handleEdge(uint16_t edgeTime)
{
	switch (gState) {
		case S_IDLE: // First edge....start acquiring more edges
			gState = S_ACQUIRING;
			gNumEdges = 0;
			resetTimer();
      allLEDs(1);
			break;
			
		case S_ACQUIRING:
			if (gNumEdges < MAX_EDGES) {
				gEdgeDelta[gNumEdges++] = edgeTime - EDGE_START_TIME;
      }
			resetTimer();
			break;

		default:
			resetTimer();
			break;
	}
}

// This is called in an ISR to indicate that MS_BEFORE_IDLE milliseconds have elapsed
// thus we consider this incoming transmission complete.
static void handleOverflow(void)
{
	switch (gState) {
		case S_ACQUIRING:
			gState = S_DONE;
			resetTimer();
      allLEDs(0);
			break;

		default:
			resetTimer();
			break;
	}
}

/*********************************************************/
/*                                                       */
/* EEPROM Storage Management                             */
/*                                                       */
/*********************************************************/
#define EE_CRC_STARTVAL (0x1D0F) // Just some random non-zero value...or is it?

/* A codebook stored in EEPROM is simply a sequence of words
   indicating valid edge-to-edge intervals, ending with 0. For example

	 104, 35, 248, 512, 0

   Following the codebook come indices into the codebook, plus 1,
	 to indicate the encoding, terminating with 0. For example:

	     1, 2, 1, 3, 0

	 would map to edge-to-edge times of:

	     104, 35, 104, 248

   A codebook that begins with 0 indicates the end of used EEPROM. Following
	 this 0 there are 2 bytes that represent a CRC over everything from the
	 start of EEPROM to the last byte, including the 0 that indicates
	 the end of used EEPROM.

   This storage format could be much more efficient, using bit fields for
   example, at the penalty of increased complexity.
*/

// This compound variable is used to help move us through EEPROM
static struct {
	uint16_t addr;
	uint16_t crc;
} eeptr;

static void eeptr_init(void)
{
	eeptr.addr = 0;
	eeptr.crc = EE_CRC_STARTVAL;
}

static uint16_t crcupdate_byte(uint16_t crc, uint8_t val)
{
	crc = _crc16_update(crc, val);
	return crc;
}

static uint16_t crcupdate_word(uint16_t crc, uint16_t val)
{
	crc = _crc16_update(crc, (uint8_t)(val & 0xFF));
	crc = _crc16_update(crc, (uint8_t)((val>>8) & 0xFF));
	return crc;
}

static uint8_t ee_canread8(void)
{
	return eeptr.addr <= (uint16_t)E2END;
}

static uint8_t ee_read8(void)
{
	uint8_t val;

	val = eeprom_read_byte((const uint8_t *) (eeptr.addr));
	eeptr.crc = crcupdate_byte(eeptr.crc, val);
	eeptr.addr++;

	return val;
}

static void ee_write8(uint8_t val)
{
	eeprom_write_byte((uint8_t *)(eeptr.addr), val);
	eeptr.crc = crcupdate_byte(eeptr.crc, val);
	eeptr.addr++;
}

static uint8_t ee_canread16(void)
{
	return eeptr.addr < (uint16_t)E2END;
}

static uint16_t ee_read16(void)
{
	uint16_t val;

	val = eeprom_read_word((const uint16_t *) (eeptr.addr));
	eeptr.crc = crcupdate_word(eeptr.crc, val);
	eeptr.addr += 2;

	return val;
}

static void ee_write16(uint16_t val)
{
	eeprom_write_word((uint16_t *)(eeptr.addr), val);
	eeptr.crc = crcupdate_word(eeptr.crc, val);
	eeptr.addr += 2;
}

static uint8_t ee_iscrcok(void)
{
	if (ee_canread16()) {
		if (eeprom_read_word((const uint16_t *)(eeptr.addr)) == eeptr.crc) {
			return 1;
		}
	}
	return 0;
}

static uint16_t ee_getcrcto(uint16_t eeaddr)
{
	uint16_t addr;
	uint16_t crc;

	crc = EE_CRC_STARTVAL;
	for (addr=0; addr < eeaddr; addr++) {
		crc = crcupdate_byte(crc, eeprom_read_byte((const uint8_t *)addr));
	}
	return crc;
}

// Go through EEPROM and make sure it stores a valid dictionary.
// nextEntry will receive the EEPROM address where the next
// codebook can be stored. numEntries will receive the number of stored
// entries.
static uint8_t isEEPROMValid(uint16_t *nextEntry, uint16_t *numEntries)
{
	uint8_t count;

	eeptr_init();
	if (numEntries) *numEntries=0;

	do {
		/* First read the codebook */
	  count=0;
		while (ee_canread16()) {
			uint16_t val;

			val = ee_read16();
			count++;

			if (val==0) break; // End of codebook
		}

		if (count==1) {  // Single-entry codebook-->end of EEPROM
			if (ee_iscrcok()) {
				if (nextEntry) {
					*nextEntry = eeptr.addr-2;
				}
				return 1;
			}
			return 0;
		}

		/* Now read encoding */
		while (ee_canread8()) {
			uint8_t val;

			val = ee_read8();

			if (val==0) break; // End of encoding
		}

		if (numEntries) (*numEntries)++;

	} while (ee_canread16());

	return 0;
}

// Create a blank dictionary by simply storing the special marker entry.
static void initEEPROM(void)
{
	eeprom_write_word((uint16_t *)0, 0);
	eeprom_write_word((uint16_t *)2, crcupdate_word(EE_CRC_STARTVAL, 0));
}

static uint16_t eepromBytesFree(void)
{
	uint16_t nextEntry;

	if (isEEPROMValid(&nextEntry, 0)) {
		return E2END+1 - nextEntry;
	}
	return 0;
}

/*********************************************************/
/*                                                       */
/* Codebook Handling                                     */
/*                                                       */
/*********************************************************/
 typedef struct {
   uint16_t lb;
   uint16_t ub;
   uint16_t counts;
 } code_t;
 static code_t gCodebook[CODEBOOK_MAX_CODES];
 static uint8_t gNumCodes;
 
 // After gNumEdges are received, this function is called to identify
 // the filtered histogram of edge-to-edge times.
 static uint8_t constructCodebookFromEdges(void)
 {
   uint16_t edge;
   uint8_t code;
   code_t *ptr;
   
   gNumCodes=0;

	 for (edge=0; edge < gNumEdges; edge++) {
		 uint16_t delta;
		 
		 delta = gEdgeDelta[edge];

     // See if this edge-to-edge time already has a code assigned for it,
     // or whether an existing code can be stretched to accommodate it,
     // else create a new code.
		 for (code=0, ptr=gCodebook; code<gNumCodes; code++, ptr++) {
			 if (delta >= ptr->lb) {
				 if (delta <= ptr->ub) {
           // This edge-to-edge time fits into an existing code.
					 ptr->counts++;
					 break;
				 } else if ((delta - ptr->lb) <= CODEBOOK_CLUSTER_TOLERANCE) {
           // This edge-to-edge time can stretch the upper-bound of this code
					 ptr->ub = delta;
					 ptr->counts++;
					 break;
				 }
			 } else if (delta <= ptr->ub) {
				 if ((ptr->ub - delta) <= CODEBOOK_CLUSTER_TOLERANCE) {
           // This edge-to-edge time can stretch the lower-bound of this code
					 ptr->lb = delta;
					 ptr->counts++;
					 break;
				 }
			 }
		 }
		 
     // Did we assign this edge-to-edge time to an existing code, or do we
     // need to create a new code for it?
		 if (code >= gNumCodes) {
			 if (gNumCodes < CODEBOOK_MAX_CODES) {
				 ptr->lb = ptr->ub = delta;
				 ptr->counts=1;
				 gNumCodes++;
			 } else {
				 return 0; // Ran out of codebook space
			 }
		 }
	 }

	 // Now go back over codebook and expand each range to maximum cluster tolerance centered about mean
	 for (code=0; code < gNumCodes; code++) {
		 uint16_t mean;

		 mean = (gCodebook[code].lb + gCodebook[code].ub) / 2;
		 gCodebook[code].lb = mean - CODEBOOK_CLUSTER_TOLERANCE/2;
		 gCodebook[code].ub = mean + CODEBOOK_CLUSTER_TOLERANCE/2;
	 }

	 return 1;
 }
 
// Assuming a codebook has already been constructed, encode
// the latest edge-to-edge times using the codebook. This is done
// before storing the encoding to EEPROM. Alternatively,
// if 'match' is non-zero then verify latest edge-to-edge times
// against previously-constructed encoding. This is done when
// trying to compare an incoming reception to an existing encoding.
static uint8_t constructOrMatchEncodingFromEdges(uint8_t match)
{
	uint8_t edge;

  if (gNumCodes == 0) {
		return 0;
	}

	for (edge=0; edge < gNumEdges; edge++) {
		uint16_t edgetime;
		uint8_t code;

		edgetime = gEdgeDelta[edge];
		for (code=0; code < gNumCodes; code++) {
			if ( (edgetime>=gCodebook[code].lb) && (edgetime<=gCodebook[code].ub) ) {
				if (match) {
					if (gEncoding[edge] != code+1) {
						return 0; // Matches with respect to time duration but doesn't match stored entry
					}
				} else {
					gEncoding[edge] = code+1; // Found the codebook entry for this edge time
				}
				break;
			}
		}
		if (code >= gNumCodes) {
			if (match) {
				// Didn't find a code for this edge time. Not a match.
				;
			} else {
        // This really shouldn't happen if we're encoding edges based on a
        // codebook constructed from those edges!
				writestr(PSTR("Could not construct edge delta time ")); Serial.println(edgetime);
			}
			return 0;
		}
	}
	if (match) {
		if (gEncoding[edge] != 0) {
			return 0; // There are still more edges stored...not a match
		}
	} else {
		gEncoding[edge] = 0; // Mark the end of the encoding with a 0 entry
	}

	return 1;
}

// gEdgeDelta array is filled in. See if it matches any encodings stored in EEPROM
static uint8_t matchEncodingFromEdges(uint8_t *match)
{
	uint8_t entry;
	uint8_t count;

	entry = 0;
	eeptr_init();

	do {
		/* First read the codebook */
	  count=0;
		while (ee_canread16()) {
			uint16_t val;

			val = ee_read16();
			gCodebook[count].lb = val - CODEBOOK_CLUSTER_TOLERANCE/2;
			gCodebook[count].ub = val + CODEBOOK_CLUSTER_TOLERANCE/2;
			count++;

			if (val==0) break; // End of codebook
		}

		if (count==1) {  // Single-entry codebook-->end of EEPROM
			return 0;
		} else {
			gNumCodes = count-1;
		}

		/* Now read encoding */
		count = 0;
		while (ee_canread8()) {
			uint8_t val;

			val = ee_read8();
			gEncoding[count++] = val;

			if (val==0) break; // End of encoding
		}
		gNumEdges = count-1;

		if (constructOrMatchEncodingFromEdges(1)) {
			if (match) *match = entry;
			return 1;
		}

		entry++;
	} while (ee_canread16());
	return 0;
}

// Append recent encoding to stored dictionary in EEPROM
static uint16_t saveEncodingToEEPROM(void)
{
	uint16_t nextEntry;
	uint16_t requiredSpace;
	uint8_t code;
	uint16_t numEntries;

	if (isEEPROMValid(&nextEntry, &numEntries)) {
		requiredSpace = (gNumCodes+1)*sizeof(uint16_t) + (gNumEdges+1)*sizeof(uint8_t) + 4;
		if ((nextEntry + requiredSpace) > E2END) return 0;

		eeptr.addr = nextEntry;
		eeptr.crc = ee_getcrcto(nextEntry);

		// Write the codebook as 16-bit integers, followed by a 0 entry
		for (code=0; code<gNumCodes; code++) {
			ee_write16( (gCodebook[code].lb + gCodebook[code].ub)/2 );
		}
		ee_write16(0);

		// Write the encoding as 8-bit integers, followed by a 0 entry
		for (code=0; gEncoding[code] != 0; code++) {
			ee_write8(gEncoding[code]);
		}
		ee_write8(0);

		// Finally, write a 0 word (empty codebook) and CRC
    ee_write16(0);
		ee_write16(eeptr.crc);
	}
	return numEntries+1;
}

#if 0   // Debugging functions
static void dumpEncoding(void)
{
	uint8_t code;

	Serial.println("Codebook:");
	for (code=0; code < gNumCodes; code++) {
		Serial.print("["); Serial.print(gCodebook[code].lb); Serial.print(","); Serial.print(gCodebook[code].ub);
		Serial.print("]: "); Serial.println(gCodebook[code].counts);
	}

	Serial.println("Encoding:");
	for (code=0; code <= gNumEdges; code++) {
		Serial.print("  "); Serial.println((uint16_t)(gEncoding[code]));
	}
}

static void dumpEEPROM(void)
{
	uint8_t count;

	eeptr_init();

	do {
		Serial.println("Codebook:");

		/* First read the codebook */
	  count=0;
		while (ee_canread16()) {
			uint16_t val;

			val = ee_read16();
			count++;

			Serial.print("  "); Serial.println(val);
			if (val==0) break; // End of codebook
		}

		if (count==1) {  // Single-entry codebook-->end of EEPROM
			return;
		}

		Serial.println("Encoding");
		/* Now read encoding */
		while (ee_canread8()) {
			uint8_t val;

			val = ee_read8();
			Serial.print("  "); Serial.println((uint16_t)val);

			if (val==0) break; // End of encoding
		}
	} while (ee_canread16());
}

static void dumpEdges(void)
{
	uint8_t edge;

	for (edge=0; edge < gNumEdges; edge++) {
		Serial.println(gEdgeDelta[edge]);
	}
}
#endif

// In preparation for transmitting a stored encoding, find the starting
// address of this stored encoding in EEPROM.
static uint8_t getStoredEncoding(uint8_t num, uint16_t *addr) 
{
	uint8_t count;

	eeptr_init();

	do {
		if (num-- == 0) {
			if (addr) *addr = eeptr.addr;
			return 1;
		}

		/* First read the codebook */
	  count=0;
		while (ee_canread16()) {
			uint16_t val;

			val = ee_read16();
			count++;

			if (val==0) break; // End of codebook
		}

		if (count==1) {  // Single-entry codebook-->end of EEPROM
			return 0;
		}

		/* Now read encoding */
		while (ee_canread8()) {
			uint8_t val;

			val = ee_read8();

			if (val==0) break; // End of encoding
		}
	} while (ee_canread16());

	return 0;
}

// This array will store the number of 38 kHz (or whatever F_OUT is) pulses
// to generate for each codebook entry.
static uint16_t gPulseLengths[CODEBOOK_MAX_CODES];

// This function actually drives the output IR LED from a stored encoding
static uint8_t outputEncoding(uint8_t num)
{
  uint16_t addr;
	uint16_t length;
	uint8_t numCodes;
	uint8_t numEdges;
	uint8_t ledOnToggle;
	uint8_t edge, val;

  if (! getStoredEncoding(num, &addr)) return 0;

	for (numCodes=0; numCodes < CODEBOOK_MAX_CODES; numCodes++) {
		length = eeprom_read_word((const uint16_t *)addr);
		addr += 2;

		if (length==0) break;

    // Convert input pulse length in 4us periods to output length
    // in 1/F_OUT periods.
		gPulseLengths[numCodes] = convertOutputPulses(length);
	}

	// Shouldn't happen
	if (numCodes >= CODEBOOK_MAX_CODES) return 0;

	numEdges = 0;
	do {
		val = eeprom_read_byte((const uint8_t *)addr);
		addr++;

		if (val == 0) break;
		else val--; // Remember the -1 since we store +1  !!!!!!

		if (val < numCodes) {
			gEdgeDelta[numEdges++] = gPulseLengths[val];
		} else {
			; // Shouldn't happen
		}
	} while (1);
		
  // Now turn off the input timer and interrupt and turn on the output timer.
	configureIRInterrupt(0);
	configureInputTimer(0);
	configureOutputTimer(1);

  allLEDs(1);

  // First edge represents an LED ON condition. From there, every subsequent
  // edge alternates between OFF and ON conditions.
	ledOnToggle = 1;

	for (edge=0; edge < numEdges; edge++) {
		generateOutput(gEdgeDelta[edge] + (ledOnToggle ? EDGE_COUNT_COMPENSATION : -EDGE_COUNT_COMPENSATION), ledOnToggle);
		ledOnToggle = ! ledOnToggle;
	}

  // Go back to input timer and interrupt so we can match incoming transmissions.
  // First delay a little bit since our own receiver sees our transmission and 
  // we get a spurious incoming edge.	
  configureOutputTimer(0);
  resetTimer(); waitForInputTimerOverflow();
	configureIRInterrupt(1);
	configureInputTimer(1);

  allLEDs(0);

	return 1;
}

/*********************************************************/

static void writestr(PGM_P str)
{
  uint8_t c;

  while ((c=pgm_read_byte(str)) != 0) {
    Serial.print(c);
    str++;
  }
}

static void writestrln(PGM_P str)
{
  writestr(str);
  Serial.println();
}

static void printHelp(void)
{
  writestrln(PSTR(\
"Press a key:\n" \
"  0: Erase/initialize EEPROM\n" \
"  1: Learn a new encoding\n" \
"  2: Test last learned encoding\n" \
"  3: Save last learned encoding to EEPROM\n" \
"  M: Output stored encoding\n" \
"  V: Verify stored encodings in EEPROM\n" \
"  F: Print free EEPROM space\n" \
"  H: Print this help menu" \
         ));
}

// This variable is non-zero when the user pressed '1' to learn a new encoding
static uint8_t gLearning;

// This variable is non-zero when the user pressed '2' to test a new encoding
static uint8_t gTestEncoding;

void setup()
{
  GS.Setup();

	// Configure the interrupt on IR input
	configureIRInterrupt(1);

  // Disable IR output until user commands it
	configureOutputTimer(0);

	// Configure the timer to measure intervals
	configureInputTimer(1);

	gState = S_IDLE;
	gLearning = 0;
	gTestEncoding = 0;
	gNumCodes = 0;

	Serial.begin(19200);

	printHelp();

	if (!isEEPROMValid(0, 0)) {
		writestrln(PSTR("\nBegin by pressing '0' to prepare the EEPROM to store encodings"));
	}
}
 
void loop()
{
	uint8_t match;

	switch (gState) {
		case S_DONE:
			gState = S_IDLE;

			if (gLearning) {
				if (! constructCodebookFromEdges()) {
					writestrln(PSTR("Too many symbols. Either try again or increase CODEBOOK_MAX_CODES."));
				} else if (constructOrMatchEncodingFromEdges(0)) {
					writestrln(PSTR("Success...now press '2' to test the encoding"));
				} else {
					writestrln(PSTR("Could not encode transmission."));
				}
				printHelp();
				gLearning = 0;
			} else if (gTestEncoding) {
				if (constructOrMatchEncodingFromEdges(1)) {
					writestr(PSTR("Match"));
				} else {
					writestr(PSTR("Mismatch"));
				}
				writestrln(PSTR("...try again or press '2' to return to main menu"));
			} else if (matchEncodingFromEdges(&match)) {
					writestr(PSTR("Matched entry ")); Serial.println((uint16_t)match);
			} else {
				writestrln(PSTR("Unknown button pressed"));
			}
			break;

		case S_IDLE:
			if (Serial.available()) {
				if (gLearning) {
					switch (Serial.read()) {
						case '1':
							writestrln(PSTR("Cancelled"));
							Serial.println();
							printHelp();
							gLearning = 0;
							break;

						default:
							break;
					}
				} else if (gTestEncoding) {
					switch (Serial.read()) {
						case '2':
							writestrln(PSTR("If the encoding works well, press '3' to save to EEPROM"));
							printHelp();
							gTestEncoding=0;
							break;

						default:
							break;
					}
				} else {
					switch (Serial.read()) {
						case 'V': case 'v':
							{
								uint16_t numEntries;
								writestr(PSTR("EEPROM stored data is "));
								if (isEEPROMValid(0, &numEntries)) {
									writestr(PSTR("valid. There are ")); Serial.print(numEntries);
									writestrln(PSTR(" stored buttons."));
								} else {
									writestrln(PSTR("not valid. Press '0' to initialize EEPROM."));
								}
							}
							break;

						case '0':
							initEEPROM();
							writestrln(PSTR("Stored data erased. Press '1' to learn a new encoding."));
							break;

						case '1':
							writestrln(PSTR("Press a remote control button. Press '1' to cancel."));
							writestrln(PSTR("Try to press it as quickly as possible to avoid repeat codes."));
							gLearning = 1;
							break;

						case '3':
							if (! gNumCodes) {
								writestrln(PSTR("Please use '1' to learn a button first."));
							} else {
								uint16_t entryNum;
								if ((entryNum=saveEncodingToEEPROM()) != 0) {
									writestr(PSTR("Saved to EEPROM as entry ")); Serial.println(entryNum-1);
									gNumCodes = 0;
								} else {
									writestrln(PSTR("EEPROM is full -- could not save data."));
								}
							}
							break;

						case 'F': case 'f':
							{
								uint16_t percentage;

								writestr(PSTR("EEPROM is "));
								percentage = ((uint32_t)eepromBytesFree()*100 + (E2END+1)/2)/(E2END+1);
								Serial.print(percentage);
								writestrln(PSTR("% free"));
							}
							break;

						case '2':
							writestrln(PSTR("Press the same button again to test. Press '2' again to cancel."));
							gTestEncoding=1;
							break;

						case 'H': case 'h':
							printHelp();
							break;

						case 'M': case 'm':
							{
								uint8_t val;

								writestrln(PSTR("Type in an encoding number followed by #"));
								val = 0;
								do {
									if (Serial.available()) {
										int c;

										c = Serial.read();
										if (isdigit(c)) {
											val *= 10;
											val += c-'0';
										} else if (c == '#') break;
									}
								} while (1);
								writestr(PSTR("Output encoding ")); Serial.println((uint16_t)val);
								if (! outputEncoding(val)) {
									writestrln(PSTR("No such encoding"));
								}
								printHelp();
							}
							break;

						default:
							break;
					}
				}
			}

      // If user presses a pushbutton then output stored buttons 0 or 1
      if (GS.IsSwitch(0)) {
        outputEncoding(0);
        delay(100);
      } else if (GS.IsSwitch(1)) {
        outputEncoding(1);
        delay(100);
      }
			break;

		default: // S_ACQUIRING
			break;
	}
}

// vim: syntax=cpp ai cindent sw=2 ts=2 expandtab
