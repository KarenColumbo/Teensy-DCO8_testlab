#include <stdint.h>
#include <Arduino.h>
#include <MIDI.h>
#include <Adafruit_MCP4728.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include "TCA9548.h"
#include <EEPROM.h>
#include "Bounce2.h"

#define NUM_VOICES 8
#define MIDI_CHANNEL 1
#define PITCH_BEND_RANGE 2
#define PITCH_POS 2
#define PITCH_NEG -2
#define CC_TEMPO 5
#define A4 440
uint16_t benderValue = 0;
uint8_t midiTempo;
uint8_t midiController[10];
bool susOn = false;
uint8_t midiNote = 0;
uint8_t velocity = 0;
float pitchBend = 8192;
uint8_t aftertouch = 0;
uint8_t modulationWheel = 0;
uint8_t ccNumber = 0;
uint8_t ccValue = 0;
uint8_t sustainPedal = 0;
uint8_t knobNumber = 0;
uint8_t knobValue = 0;
int midiNoteVoltage = 0;
float midiNoteFrequency = 0;
float bentNoteFrequency = 0.00;



// ----------------------------- MIDI note frequencies C1-C7
float noteFrequency [73] = {
  32.7032, 34.6478, 36.7081, 38.8909, 41.2034, 43.6535, 46.2493, 48.9994, 51.9131, 55, 58.2705, 61.7354, 
  65.4064, 69.2957, 73.4162, 77.7817, 82.4069, 87.3071, 92.4986, 97.9989, 103.826, 110, 116.541, 123.471, 
  130.813, 138.591, 146.832, 155.563, 164.814, 174.614, 184.997, 195.998, 207.652, 220, 233.082, 246.942, 
  261.626, 277.183, 293.665, 311.127, 329.628, 349.228, 369.994, 391.995, 415.305, 440, 466.164, 493.883, 
  523.251, 554.365, 587.33, 622.254, 659.255, 698.456, 739.989, 783.991, 830.609, 880, 932.328, 987.767, 
  1046.5, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760, 1864.66, 1975.53, 
  2093
};

// ----------------------------- 14 bit note frequency voltages C1-C7
const unsigned int noteVolt[73] = {
  0, 15, 32, 49, 68, 87, 108, 130, 153, 177, 203, 231, 
  260, 291, 324, 358, 395, 434, 476, 519, 566, 615, 667, 722, 
  780, 842, 908, 977, 1051, 1129, 1211, 1299, 1391, 1489, 1593, 1704, 
  1820, 1944, 2075, 2214, 2361, 2517, 2682, 2857, 3043, 3239, 3447, 3667, 
  3901, 4148, 4411, 4688, 4982, 5294, 5625, 5974, 6345, 6738, 7154, 7595, 
  8062, 8557, 9081, 9637, 10225, 10849, 11509, 12209, 12950, 13736, 14568, 15450, 
  16383
  };

  struct Voice {
  unsigned long noteAge;
  uint8_t midiNote;
  bool noteOn;
  uint8_t velocity;
  uint16_t pitchBend;
  uint8_t channelPressure;
  uint8_t modulationWheel;
  uint8_t prevNote;
  uint16_t bentNote;
  uint16_t bentNoteFreq;
  unsigned long lastTime;
  bool clockState;
};

Voice voices[NUM_VOICES];

void initializeVoices() {
  for (int i = 0; i < NUM_VOICES; i++) {
    voices[i].noteAge = 0;
    voices[i].midiNote = 0;
    voices[i].noteOn = false;
    voices[i].velocity = 0;
    voices[i].pitchBend = 0x2000;
    voices[i].channelPressure = 0;
    voices[i].modulationWheel = 0;
    voices[i].prevNote = 0;
    voices[i].bentNote = 0x2000;
    voices[i].bentNoteFreq = 0x2000;
    voices[i].lastTime = 0;
    voices[i].clockState = false;
  }
}

// ------------------------ Voice buffer subroutines 
int findOldestVoice() {
  int oldestVoice = 0;
  unsigned long oldestAge = 0xFFFFFFFF;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (!voices[i].noteOn && voices[i].noteAge < oldestAge) {
      oldestVoice = i;
      oldestAge = voices[i].noteAge;
    }
  }
  return oldestVoice;
}

int findVoice(uint8_t midiNote) {
  int foundVoice = -1;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (voices[i].noteOn && voices[i].midiNote == midiNote) {
      foundVoice = i;
      break;
    }
  }
  return foundVoice;
}

/*void noteOn(uint8_t midiNote, uint8_t velocity) {
  int voice = findVoice(midiNote);
  if (voice == -1) {
    voice = findOldestVoice();
    voices[voice].prevNote = voices[voice].midiNote;
  }
  voices[voice].noteAge = millis();
  voices[voice].midiNote = midiNote;
  voices[voice].noteOn = true;
  voices[voice].velocity = velocity;
}
*/

void noteOn(uint8_t midiNote, uint8_t velocity) {
  int voice = findVoice(midiNote);
  if (voice == -1) {
    int numPlayingVoices = 0;
    for (int i = 0; i < NUM_VOICES; i++) {
      if (voices[i].noteOn) {
        numPlayingVoices++;
      }
    }

    if (numPlayingVoices >= NUM_VOICES) {
      unsigned long oldestAge = 0xFFFFFFFF;
      int oldestVoice = -1;
      for (int i = 0; i < NUM_VOICES; i++) {
        if (voices[i].noteAge < oldestAge) {
          oldestAge = voices[i].noteAge;
          oldestVoice = i;
        }
      }
      voice = oldestVoice;
    } else {
      for (int i = 0; i < NUM_VOICES; i++) {
        if (!voices[i].noteOn) {
          voice = i;
          break;
        }
      }
    }

    voices[voice].prevNote = voices[voice].midiNote;
  }
  voices[voice].noteAge = millis();
  voices[voice].midiNote = midiNote;
  voices[voice].noteOn = true;
  voices[voice].velocity = velocity;
}



void noteOff(uint8_t midiNote) {
  int voice = findVoice(midiNote);
  if (voice != -1) {
    voices[voice].noteOn = false;
    voices[voice].velocity = 0;
    voices[voice].midiNote = 0;
    voices[voice].noteAge = 0;
  }
}

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1,  MIDI);

// ************************************************
// ******************** SETUP *********************
// ************************************************

void setup() {
	Serial.begin(9600);
  MIDI.begin(MIDI_CHANNEL);
}

// ************************************************
// ******************** MAIN **********************
// ************************************************

void loop() {


if (MIDI.read()) {

    // -------------------- Note On/Off
    if (MIDI.getType() == midi::NoteOn && MIDI.getChannel() == MIDI_CHANNEL) {
      midiNote = MIDI.getData1();
      velocity = MIDI.getData2();
      noteOn(midiNote, velocity);
      for (int i = 0; i < NUM_VOICES; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(voices[i].midiNote);
        Serial.print(" | "+String(voices[i].noteAge));
        Serial.println("   --->    "+String(voices[i].noteOn));
      }
    }
    
    if (MIDI.getType() == midi::NoteOff && MIDI.getChannel() == MIDI_CHANNEL) {
      midiNote = MIDI.getData1();
      noteOff(midiNote);
      for (int i = 0; i < NUM_VOICES; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(voices[i].midiNote);
        Serial.print(" | "+String(voices[i].noteAge));
        Serial.println("   --->    "+String(voices[i].noteOn));
      }
    }

    //  float semitone_ratio = pow(2.0, 1.0/12.0);
    //  float bend_semitones = (pitchBend - 8192) / 8192.0 * PITCH_BEND_RANGE;
    //  float bend_factor = pow(semitone_ratio, bend_semitones);
    //  float bentNoteFrequency = noteFrequency[midiNote] * bend_factor;     

      
    
    
    // ------------------ Pitchbend 
    if (MIDI.getType() == midi::PitchBend && MIDI.getChannel() == MIDI_CHANNEL) {
      pitchBend = MIDI.getData1() | (MIDI.getData2() << 7);
    }

    // ------------------ Aftertouch 
    if (MIDI.getType() == midi::AfterTouchChannel && MIDI.getChannel() == MIDI_CHANNEL) {
      aftertouch = MIDI.getData1();
    }

    // ------------------ Modwheel 
    if (MIDI.getType() == midi::ControlChange && MIDI.getData1() == 1 && MIDI.getChannel() == MIDI_CHANNEL) {
      modulationWheel = MIDI.getData2();
		}

		// ------------------ Sustain
    if (MIDI.getType() == midi::ControlChange && MIDI.getData1() == 64 && MIDI.getChannel() == MIDI_CHANNEL) {
      sustainPedal = MIDI.getData2();
			if (sustainPedal > 63) {
        susOn = true;
      } else {
        susOn = false;
      }
      Serial.print(sustainPedal);
    }

    // ------------------ MIDI CC
    if (MIDI.getType() == midi::ControlChange && MIDI.getChannel() == MIDI_CHANNEL) {
      knobNumber = MIDI.getData1();
      knobValue = MIDI.getData2();
      if (knobNumber >69 && knobNumber <88) {
        
      }
    }
	}

	// ****************************************************************
	// ***************************** DEBUG ****************************
	// ****************************************************************
	
	
}
