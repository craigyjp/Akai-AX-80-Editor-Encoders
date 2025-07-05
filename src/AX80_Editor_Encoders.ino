/*
  AX80 Editor Encoders - Firmware Rev 1.1

  Includes code by:
    ElectroTechnique for general method of menus and updates.

  Arduino IDE
  Tools Settings:
  Board: "Teensy4,1"
  USB Type: "Serial + MIDI"
  CPU Speed: "600"
  Optimize: "Fastest"

  Performance Tests   CPU  Mem
  180Mhz Faster       81.6 44
  180Mhz Fastest      77.8 44
  180Mhz Fastest+PC   79.0 44
  180Mhz Fastest+LTO  76.7 44
  240MHz Fastest+LTO  55.9 44

  Additional libraries:
    Agileware CircularBuffer available in Arduino libraries manager
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <MIDI.h>
#include "MidiCC.h"
#include "Constants.h"
#include "Parameters.h"
#include "PatchMgr.h"
#include "Button.h"
#include "HWControls.h"
#include "EepromMgr.h"


#define PARAMETER 0      //The main page for displaying the current patch and control (parameter) changes
#define RECALL 1         //Patches list
#define SAVE 2           //Save patch page
#define REINITIALISE 3   // Reinitialise message
#define PATCH 4          // Show current patch bypassing PARAMETER
#define PATCHNAMING 5    // Patch naming page
#define DELETE 6         //Delete patch page
#define DELETEMSG 7      //Delete patch message page
#define SETTINGS 8       //Settings page
#define SETTINGSVALUE 9  //Settings page

unsigned int state = PARAMETER;

#include "ST7735Display.h"

boolean cardStatus = false;

//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial5, MIDI5);

byte ccType = 2;  //(EEPROM)

#include "Settings.h"

int count = 0;  //For MIDI Clk Sync
int DelayForSH3 = 12;
int patchNo = 1;               //Current patch no
int voiceToReturn = -1;        //Initialise
long earliestTime = millis();  //For voice allocation - initialise to now

void pollAllMCPs();

void initRotaryEncoders();

void initButtons();

int getEncoderSpeed(int id);

void setup() {
  SPI.begin();
  setupDisplay();
  Wire.begin();
  Wire.setClock(400000);  // Slow down I2C to 100kHz

  mcp1.begin(0);
  delay(10);
  mcp2.begin(1);
  delay(10);
  mcp3.begin(2);
  delay(10);
  mcp4.begin(3);
  delay(10);
  mcp5.begin(4);
  delay(10);
  mcp6.begin(5);
  delay(10);
  mcp7.begin(6);
  delay(10);

  //groupEncoders();
  initRotaryEncoders();
  initButtons();


  mcp1.pinMode(6, OUTPUT);  // pin 6 = GPA6 of MCP2301X
  mcp1.pinMode(7, OUTPUT);  // pin 7 = GPA7 of MCP2301X

  mcp1.pinMode(13, OUTPUT);  // pin 13 = GPB5 of MCP2301X
  mcp1.pinMode(14, OUTPUT);  // pin 14 = GPB6 of MCP2301X
  mcp1.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp2.pinMode(6, OUTPUT);   // pin 6 = GPA6 of MCP2301X
  mcp2.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp2.pinMode(13, OUTPUT);  // pin 13 = GPB5 of MCP2301X
  mcp2.pinMode(14, OUTPUT);  // pin 14 = GPB6 of MCP2301X
  mcp2.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp3.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp3.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp4.pinMode(6, OUTPUT);   // pin 6 = GPA6 of MCP2301X
  mcp4.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp4.pinMode(14, OUTPUT);  // pin 14 = GPB6 of MCP2301X
  mcp4.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp5.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp5.pinMode(14, OUTPUT);  // pin 14 = GPB6 of MCP2301X
  mcp5.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp6.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp6.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp7.pinMode(6, OUTPUT);   // pin 6 = GPA6 of MCP2301X
  mcp7.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp7.pinMode(14, OUTPUT);  // pin 14 = GPB6 of MCP2301X
  mcp7.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  for (int i = 0; i < NUM_ENCODERS; i++) {
    lastTransition[i] = millis();  // ✅ no warning now
  }

  setUpSettings();
  setupHardware();

  cardStatus = SD.begin(BUILTIN_SDCARD);
  if (cardStatus) {
    Serial.println("SD card is connected");
    //Get patch numbers and names from SD card
    loadPatches();
    if (patches.size() == 0) {
      //save an initialised patch to SD card
      savePatch("1", INITPATCH);
      loadPatches();
    }
  } else {
    Serial.println("SD card is not connected or unusable");
    reinitialiseToPanel();
    showPatchPage("No SD", "conn'd / usable");
  }

  //Read MIDI Channel from EEPROM
  midiChannel = getMIDIChannel();

  Serial.println("MIDI Ch:" + String(midiChannel) + " (0 is Omni On)");

  //Read UpdateParams type from EEPROM
  updateParams = getUpdateParams();
  ROMType = getROMType();

  //USB Client MIDI
  usbMIDI.setHandleControlChange(myConvertControlChange);
  usbMIDI.setHandleProgramChange(myProgramChange);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandlePitchChange(myPitchBend);
  usbMIDI.setHandleAfterTouchChannel(myAfterTouch);
  usbMIDI.setHandleSystemExclusive(handleSysexByte);
  Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(myConvertControlChange);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleNoteOn(myNoteOn);
  MIDI.setHandleNoteOff(myNoteOff);
  MIDI.setHandlePitchBend(myPitchBend);
  MIDI.setHandleAfterTouchChannel(myAfterTouch);
  MIDI.setHandleSystemExclusive(handleSysexByte);
  MIDI.turnThruOn(midi::Thru::Mode::Off);
  Serial.println("MIDI In DIN Listening");


  MIDI5.begin();
  MIDI5.setHandleSystemExclusive(handleSysexByte);
  //MIDI5.turnThruOn(midi::Thru::Mode::Off);
  Serial.println("MIDI5 In DIN Listening");

  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();

  //Read MIDI Out Channel from EEPROM
  midiOutCh = getMIDIOutCh();

  //Read Bank from EEPROM
  bankselect = getSetBank();

  // Read the encoders accelerate
  accelerate = getEncoderAccelerate();

  // read in aftertouch setting
  afterTouch = getAfterTouch();

  recallPatch(patchNo);  //Load first patch
  refreshScreen();
}

void initRotaryEncoders() {
  for (auto &rotaryEncoder : rotaryEncoders) {
    rotaryEncoder.init();
  }
}

void initButtons() {
  for (auto &button : allButtons) {
    button->begin();
  }
}

void myNoteOn(byte channel, byte note, byte velocity) {
  if (!recallPatchFlag) {
    MIDI.sendNoteOn(note, velocity, channel);
  }
}

void myNoteOff(byte channel, byte note, byte velocity) {
  if (!recallPatchFlag) {
    MIDI.sendNoteOff(note, velocity, channel);
  }
}

void handleSysexByte(byte *data, unsigned length) {
  // Only set up for the first call in a SysEx message
  if (!receivingSysEx) {
    receivingSysEx = true;
  }

  // Store incoming bytes in sysexData array across blocks
  for (unsigned i = 0; i < length; i++) {
    ramArray[currentBlock][byteIndex] = data[i];
    byteIndex++;

    // Move to the next block if the current block is full
    if (byteIndex >= 50) {
      byteIndex = 0;
      currentBlock++;
    }
  }

  // Check if we’ve received all 64 blocks
  if (currentBlock >= 64) {
    sysexComplete = true;
    receivingSysEx = false;  // Clear flag as SysEx message is complete
    currentBlock = 0;
  }
}

void printSysexData() {

  for (int i = 0; i < 64; i++) {
    for (int j = 0; j < 50; j++) {

      switch (j) {

        case 3:
          patchName = "SysexPatch " + String((ramArray[i][j] + 1));
          break;

        case 4:
          osc1_octave = ramArray[i][j];
          break;

        case 5:
          osc1_wave = ramArray[i][j];
          break;

        case 6:
          osc1_pw = ramArray[i][j];
          break;

        case 7:  // osc1_pwm
          osc1_pwm = ramArray[i][j];
          break;

        case 8:
          osc1_sub = ramArray[i][j];
          break;

        case 9:  // osc1_level
          osc1_level = ramArray[i][j];
          break;

        case 10:  // osc2_freq
          osc2_freq = ramArray[i][j];
          break;

        case 11:  // osc2_detune
          osc2_detune = ramArray[i][j];
          break;

        case 12:  // osc2_wave
          osc2_wave = ramArray[i][j];
          break;

        case 13:  // osc2_xmod
          osc2_xmod = ramArray[i][j];
          break;

        case 14:  // osc2_eg_depth
          osc2_eg_depth = ramArray[i][j];
          break;

        case 15:  // osc2_eg_select
          osc2_eg_select = ramArray[i][j];
          break;

        case 16:  // osc2_level
          osc2_level = ramArray[i][j];
          break;

        case 17:  // vcf_cutoff
          vcf_cutoff = ramArray[i][j];
          break;

        case 18:  // vcf_res
          vcf_res = ramArray[i][j];
          break;

        case 19:  // vcf_eg_depth
          vcf_eg_depth = ramArray[i][j];
          break;

        case 20:  // vcf_key_follow
          vcf_key_follow = ramArray[i][j];
          break;

        case 21:  // vcf_key_velocity
          vcf_key_velocity = ramArray[i][j];
          break;

        case 22:  // vcf_hpf
          vcf_hpf = ramArray[i][j];
          break;

        case 23:  // lfo1_depth
          lfo1_depth = ramArray[i][j];
          break;

        case 24:  // lfo1_speed
          lfo1_speed = ramArray[i][j];
          break;

        case 25:  // lfo1_delay
          lfo1_delay = ramArray[i][j];
          break;

        case 26:  // lfo1_wave
          lfo1_wave = ramArray[i][j];
          break;

        case 27:  // lfo_select
          lfo_select = ramArray[i][j];
          break;

        case 28:
          eg1_attack = ramArray[i][j];
          break;

        case 29:
          eg1_decay = ramArray[i][j];
          break;

        case 30:
          eg1_sustain = ramArray[i][j];
          break;

        case 31:
          eg1_release = ramArray[i][j];
          break;

        case 32:
          eg1_key_follow = ramArray[i][j];
          break;

        case 33:
          eg_select = ramArray[i][j];
          break;

        case 34:
          vca_key_velocity = ramArray[i][j];
          break;

        case 35:
          vca_level = ramArray[i][j];
          break;

        case 36:
          lfo2_depth = ramArray[i][j];
          break;

        case 37:
          lfo2_speed = ramArray[i][j];
          break;

        case 38:
          lfo2_delay = ramArray[i][j];
          break;

        case 39:
          lfo2_wave = ramArray[i][j];
          break;

        case 40:
          lfo3_depth = ramArray[i][j];
          break;

        case 41:
          lfo3_speed = ramArray[i][j];
          break;

        case 42:
          lfo3_delay = ramArray[i][j];
          break;

        case 43:
          lfo3_wave = ramArray[i][j];
          break;

        case 44:
          eg2_attack = ramArray[i][j];
          break;

        case 45:
          eg2_decay = ramArray[i][j];
          break;

        case 46:
          eg2_sustain = ramArray[i][j];
          break;

        case 47:
          eg2_release = ramArray[i][j];
          break;

        case 48:
          eg2_key_follow = ramArray[i][j];
          break;
      }
    }
    // Add a newline to separate rows (optional)
    switch (bankselect) {
      case 0:
        sprintf(buffer, "%d", i + 33);
        break;

      case 1:
        sprintf(buffer, "%d", i + 97);
        break;

      case 2:
        sprintf(buffer, "%d", i + 161);
        break;

      case 3:
        sprintf(buffer, "%d", i + 225);
        break;

      case 4:
        sprintf(buffer, "%d", i + 289);
        break;
    }
    savePatch(buffer, getCurrentPatchData());
    updatePatchname();
  }
  loadPatches();
  switch (bankselect) {
    case 0:
      recallPatch(33);
      break;

    case 1:
      recallPatch(97);
      break;

    case 2:
      recallPatch(161);
      break;

    case 3:
      recallPatch(225);
      break;

    case 4:
      recallPatch(289);
      break;
  }
}

void mySystemExclusiveChunk(byte *data, unsigned int length) {

  for (unsigned int n = 3; n < 50; n++) {
    recallPatchFlag = true;
    switch (n) {

      case 3:  // patch number
        received_patch = data[n];
        ramArray[received_patch][n] = data[n];
        break;

      case 4:  // osc1_octave
        ramArray[received_patch][n] = data[n];
        break;

      case 5:  // osc1_wave
        ramArray[received_patch][n] = data[n];
        break;

      case 6:  // osc1_pw
        ramArray[received_patch][n] = data[n];
        break;

      case 7:  // osc1_pwm
        ramArray[received_patch][n] = data[n];
        break;

      case 8:  // osc1_sub
        ramArray[received_patch][n] = data[n];
        break;

      case 9:  // osc1_level
        ramArray[received_patch][n] = data[n];
        break;

      case 10:  // osc2_freq
        ramArray[received_patch][n] = data[n];
        break;

      case 11:  // osc2_detune
        ramArray[received_patch][n] = data[n];
        break;

      case 12:  // osc2_wave
        ramArray[received_patch][n] = data[n];
        break;

      case 13:  // osc2_xmod
        ramArray[received_patch][n] = data[n];
        break;

      case 14:  // osc2_eg_depth
        ramArray[received_patch][n] = data[n];
        break;

      case 15:  // osc2_eg_select
        ramArray[received_patch][n] = data[n];
        break;

      case 16:  // osc2_level
        ramArray[received_patch][n] = data[n];
        break;

      case 17:  // vcf_cutoff
        ramArray[received_patch][n] = data[n];
        break;

      case 18:  // vcf_res
        ramArray[received_patch][n] = data[n];
        break;

      case 19:  // vcf_eg_depth
        ramArray[received_patch][n] = data[n];
        break;

      case 20:  // vcf_key_follow
        ramArray[received_patch][n] = data[n];
        break;

      case 21:  // vcf_key_velocity
        ramArray[received_patch][n] = data[n];
        break;

      case 22:  // vcf_hpf
        ramArray[received_patch][n] = data[n];
        break;

      case 23:  // lfo1_depth
        ramArray[received_patch][n] = data[n];
        break;

      case 24:  // lfo1_speed
        ramArray[received_patch][n] = data[n];
        break;

      case 25:  // lfo1_delay
        ramArray[received_patch][n] = data[n];
        break;

      case 26:  // lfo1_wave
        ramArray[received_patch][n] = data[n];

      case 27:  // lfo_select
        ramArray[received_patch][n] = data[n];
        break;

      case 28:  // eg1_attack
        ramArray[received_patch][n] = data[n];
        break;

      case 29:  // eg1_decay
        ramArray[received_patch][n] = data[n];
        break;

      case 30:  // eg1_sustain
        ramArray[received_patch][n] = data[n];
        break;

      case 31:  // eg1_release
        ramArray[received_patch][n] = data[n];
        break;

      case 32:  // eg1_key_follow
        ramArray[received_patch][n] = data[n];
        break;

      case 33:  // eg_select
        ramArray[received_patch][n] = data[n];
        break;

      case 34:  // vca_key_velocity
        ramArray[received_patch][n] = data[n];
        break;

      case 35:  // vca_level
        ramArray[received_patch][n] = data[n];
        break;

      case 36:  // lfo2_depth
        ramArray[received_patch][n] = data[n];
        break;

      case 37:  // lfo2_speed
        ramArray[received_patch][n] = data[n];
        break;

      case 38:  // lfo2_delay
        ramArray[received_patch][n] = data[n];
        break;

      case 39:  // lfo2_wave
        ramArray[received_patch][n] = data[n];
        break;

      case 40:  // lfo3_depth
        ramArray[received_patch][n] = data[n];
        break;

      case 41:  // lfo3_speed
        ramArray[received_patch][n] = data[n];
        break;

      case 42:  // lfo3_delay
        ramArray[received_patch][n] = data[n];
        break;

      case 43:  // lfo3_wave
        ramArray[received_patch][n] = data[n];
        break;

      case 44:  // eg2_attack
        ramArray[received_patch][n] = data[n];
        break;

      case 45:  // eg2_decay
        ramArray[received_patch][n] = data[n];
        break;

      case 46:  // eg2_sustain
        ramArray[received_patch][n] = data[n];
        break;

      case 47:  // eg2_release
        ramArray[received_patch][n] = data[n];
        break;

      case 48:  // eg2_key_follow
        ramArray[received_patch][n] = data[n];
        break;

      case 49:  // end of sysex
        ramArray[received_patch][n] = data[n];
        break;
    }
  }

  if (received_patch == 63) {
    for (unsigned int i = 0; i < 64; i++) {
      for (unsigned int n = 3; n < 49; n++) {
        Serial.print(ramArray[i][n]);
        Serial.print(", ");
      }
      Serial.println("");
    }
  }

  // recallPatchFlag = false;

  // patchName = "Patch ";
  // patchName += String(currentSendPatch + 1);

  // updatePatchname();
  // sprintf(buffer, "%d", currentSendPatch + 1);
  // savePatch(buffer, getCurrentPatchData());
  // currentSendPatch++;
  // delay(100);

  // if (currentSendPatch == 64) {
  //   loadPatches();
  //   loadFromDW = false;
  //   storeLoadFromDW(loadFromDW);
  //   settings::decrement_setting_value();
  //   settings::save_current_value();
  //   showSettingsPage();
  //   delay(100);
  //   state = PARAMETER;
  //   recallPatch(1);
  //   MIDI.sendProgramChange(0, midiOutCh);
  // }
  // dataInProgress = false;

  // }
}

void myConvertControlChange(byte channel, byte number, byte value) {
  if (!recallPatchFlag) {
    switch (number) {

      case 0:
        bankselect = value;
        break;

      case 1:
        MIDI.sendControlChange(number, value, midiOutCh);
        break;

      case 2:
        MIDI.sendControlChange(number, value, midiOutCh);
        break;

      case 7:
        MIDI.sendControlChange(96, value, midiOutCh);
        break;

      case 64:
        MIDI.sendControlChange(number, value, midiOutCh);
        break;

      default:
        int newvalue = value;
        myControlChange(channel, number, newvalue);
        break;
    }
  }
}

void myPitchBend(byte channel, int bend) {
  if (!recallPatchFlag) {
    MIDI.sendPitchBend(bend, midiOutCh);
  }
}

void myAfterTouch(byte channel, byte value) {
  if (!recallPatchFlag) {
    if (afterTouch) {
      MIDI.sendControlChange(1, value, midiOutCh);
    }
  }
}

void allNotesOff() {
}

void updateosc1_level() {
  if (!recallPatchFlag) {
    if (osc1_level == 0) {
      showCurrentParameterPage("Osc1 Level", String("Off"));
    } else {
      showCurrentParameterPage("Osc1 Level", String(osc1_level));
    }
    startParameterDisplay();
  }
  if (osc1_level != pre_osc1_level) {
    midiCCOut(CCosc1_level, osc1_level);
    pre_osc1_level = osc1_level;
  }
}

void updateosc1_PW() {
  if (!recallPatchFlag) {
    if (osc1_pw == 0) {
      showCurrentParameterPage("Osc1 PW", String("Off"));
    } else {
      showCurrentParameterPage("Osc1 PW", String(osc1_pw));
    }
    startParameterDisplay();
  }
  if (osc1_pw != pre_osc1_pw) {
    midiCCOut(CCosc1_PW, osc1_pw);
    pre_osc1_pw = osc1_pw;
  }
}

void updateosc1_PWM() {
  if (!recallPatchFlag) {
    if (osc1_pwm == 0) {
      showCurrentParameterPage("Osc1 PWM", String("Off"));
    } else {
      showCurrentParameterPage("Osc1 PWM", String(osc1_pwm));
    }
    startParameterDisplay();
  }
  if (osc1_pwm != pre_osc1_pwm) {
    midiCCOut(CCosc1_PWM, osc1_pwm);
    pre_osc1_pwm = osc1_pwm;
  }
}

void updateosc2_freq() {
  if (!recallPatchFlag) {
    switch (osc2_freq) {
      case 0:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot"));
        break;
      case 1:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +1"));
        break;
      case 2:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +2"));
        break;
      case 3:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +3"));
        break;
      case 4:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +4"));
        break;
      case 5:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +5"));
        break;
      case 6:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +6"));
        break;
      case 7:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +7"));
        break;
      case 8:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +8"));
        break;
      case 9:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +9"));
        break;
      case 10:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +10"));
        break;
      case 11:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot +11"));
        break;
      case 12:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot"));
        break;
      case 13:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +1"));
        break;
      case 14:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +2"));
        break;
      case 15:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +3"));
        break;
      case 16:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +4"));
        break;
      case 17:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +5"));
        break;
      case 18:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +6"));
        break;
      case 19:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +7"));
        break;
      case 20:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +8"));
        break;
      case 221:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +9"));
        break;
      case 22:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +10"));
        break;
      case 23:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot +11"));
        break;
      case 24:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot"));
        break;
      case 25:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +1"));
        break;
      case 26:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +2"));
        break;
      case 27:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +3"));
        break;
      case 28:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +4"));
        break;
      case 29:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +5"));
        break;
      case 30:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +6"));
        break;
      case 31:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +7"));
        break;
      case 32:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +8"));
        break;
      case 33:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +9"));
        break;
      case 34:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +10"));
        break;
      case 35:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot +11"));
        break;
      case 36:
        showCurrentParameterPage("Osc2 Freq", String("2 Foot"));
        break;
    }
    startParameterDisplay();
  }
  if (osc2_freq != pre_osc2_freq) {
    midiCCOut(CCosc2_freq, osc2_freq);
    pre_osc2_freq = osc2_freq;
  }
}

void updateosc2_eg_depth() {
  if (!recallPatchFlag) {
    if (osc2_eg_depth == 50) {
      showCurrentParameterPage("Osc2 EG Dep", "Off");
    } else {
      showCurrentParameterPage("Osc2 EG Dep", String(osc2_eg_depth));
    }
    startParameterDisplay();
  }
  if (osc2_eg_depth != pre_osc2_eg_depth) {
    midiCCOut(CCosc2_eg_depth, osc2_eg_depth);
    pre_osc2_eg_depth = osc2_eg_depth;
  }
}

void updateosc2_level() {
  if (!recallPatchFlag) {
    if (osc2_level == 0) {
      showCurrentParameterPage("Osc2 Level", String("Off"));
    } else {
      showCurrentParameterPage("Osc2 Level", String(osc2_level));
    }
    startParameterDisplay();
  }
  if (osc2_level != pre_osc2_level) {
    midiCCOut(CCosc2_level, osc2_level);
    pre_osc2_level = osc2_level;
  }
}

void updateosc2_detune() {
  if (!recallPatchFlag) {
    if (osc2_detune == 50) {
      showCurrentParameterPage("Osc2 Detune", "Off");
    } else {
      showCurrentParameterPage("Osc2 Detune", String(osc2_detune));
    }
    startParameterDisplay();
  }
  if (osc2_detune != pre_osc2_detune) {
    midiCCOut(CCosc2_detune, osc2_detune);
    pre_osc2_detune = osc2_detune;
  }
}

void updatevcf_cutoff() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("VCF Cutoff", String(vcf_cutoff));
    startParameterDisplay();
  }
  if (vcf_cutoff != pre_vcf_cutoff) {
    midiCCOut(CCvcf_cutoff, vcf_cutoff);
    pre_vcf_cutoff = vcf_cutoff;
  }
}

void updatevcf_res() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("VCF Res", String(vcf_res));
    startParameterDisplay();
  }
  if (vcf_res != pre_vcf_res) {
    midiCCOut(CCvcf_res, vcf_res);
    pre_vcf_res = vcf_res;
  }
}

void updatevcf_eg_depth() {
  if (!recallPatchFlag) {
    if (vcf_eg_depth == 50) {
      showCurrentParameterPage("VCF EG Depth", "Off");
    } else {
      showCurrentParameterPage("VCF EG Depth", String(vcf_eg_depth));
    }
    startParameterDisplay();
  }
  if (vcf_eg_depth != pre_vcf_eg_depth) {
    midiCCOut(CCvcf_eg_depth, vcf_eg_depth);
    pre_vcf_eg_depth = vcf_eg_depth;
  }
}

void updatevcf_key_follow() {
  if (!recallPatchFlag) {
    if (vcf_key_follow == 0) {
      showCurrentParameterPage("VCF K.Follow", String("Off"));
    } else {
      showCurrentParameterPage("VCF K.Follow", String(vcf_key_follow));
    }
    startParameterDisplay();
  }
  if (vcf_key_follow != pre_vcf_key_follow) {
    midiCCOut(CCvcf_key_follow, vcf_key_follow);
    pre_vcf_key_follow = vcf_key_follow;
  }
}

void updatevcf_key_velocity() {
  if (!recallPatchFlag) {
    if (vcf_key_velocity == 0) {
      showCurrentParameterPage("VCF K.Velocity", String("Off"));
    } else {
      showCurrentParameterPage("VCF K.Velocity", String(vcf_key_velocity));
    }
    startParameterDisplay();
  }
  if (vcf_key_velocity != pre_vcf_key_velocity) {
    midiCCOut(CCvcf_key_velocity, vcf_key_velocity);
    pre_vcf_key_velocity = vcf_key_velocity;
  }
}

void updatevcf_hpf() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("VCF High Pass", String(vcf_hpf));
    startParameterDisplay();
  }
  if (vcf_hpf != pre_vcf_hpf) {
    midiCCOut(CCvcf_hpf, vcf_hpf);
    pre_vcf_hpf = vcf_hpf;
  }
}

void updatelfo1_depth() {
  if (!recallPatchFlag) {
    if (lfo1_depth == 0) {
      showCurrentParameterPage("LFO1 Depth", String("Off"));
    } else {
      showCurrentParameterPage("LFO1 Depth", String(lfo1_depth));
    }
    startParameterDisplay();
  }

  if (lfo_select_temp != 0) {
    midiCCOut(CClfo_select, 0);
    delay(1);
    lfo_select_temp = 0;
  }
  if (lfo1_depth != pre_lfo1_depth) {
    midiCCOut(CClfo1_depth, lfo1_depth);
    pre_lfo1_depth = lfo1_depth;
  }
}

void updatelfo1_speed() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO1 Speed", String(lfo1_speed));
    startParameterDisplay();
  }

  if (lfo_select_temp != 0) {
    midiCCOut(CClfo_select, 0);
    delay(1);
    lfo_select_temp = 0;
  }
  if (lfo1_speed != pre_lfo1_speed) {
    midiCCOut(CClfo1_speed, lfo1_speed);
    pre_lfo1_speed = lfo1_speed;
  }
}

void updatelfo1_delay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO1 Delay", String(lfo1_delay));
    startParameterDisplay();
  }

  if (lfo_select_temp != 0) {
    midiCCOut(CClfo_select, 0);
    delay(1);
    lfo_select_temp = 0;
  }
  if (lfo1_delay != pre_lfo1_delay) {
    midiCCOut(CClfo1_delay, lfo1_delay);
    pre_lfo1_delay = lfo1_delay;
  }
}

void updatelfo2_depth() {
  if (!recallPatchFlag) {
    if (lfo2_depth == 0) {
      showCurrentParameterPage("LFO2 Depth", String("Off"));
    } else {
      showCurrentParameterPage("LFO2 Depth", String(lfo2_depth));
    }
    startParameterDisplay();
  }

  if (lfo_select_temp != 1) {
    midiCCOut(CClfo_select, 1);
    delay(1);
    lfo_select_temp = 1;
  }

  if (lfo2_depth != pre_lfo2_depth) {
    midiCCOut(CClfo2_depth, lfo2_depth);
    pre_lfo2_depth = lfo2_depth;
  }
}

void updatelfo2_speed() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO2 Speed", String(lfo2_speed));
    startParameterDisplay();
  }

  if (lfo_select_temp != 1) {
    midiCCOut(CClfo_select, 1);
    delay(1);
    lfo_select_temp = 1;
  }

  if (lfo2_speed != pre_lfo2_speed) {
    midiCCOut(CClfo2_speed, lfo2_speed);
    pre_lfo2_speed = lfo2_speed;
  }
}

void updatelfo2_delay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO2 Delay", String(lfo2_delay));
    startParameterDisplay();
  }

  if (lfo_select_temp != 1) {
    midiCCOut(CClfo_select, 1);
    delay(1);
    lfo_select_temp = 1;
  }

  if (lfo2_delay != pre_lfo2_delay) {
    midiCCOut(CClfo2_delay, lfo2_delay);
    pre_lfo2_delay = lfo2_delay;
  }
}

void updatelfo3_depth() {
  if (!recallPatchFlag) {
    if (lfo3_depth == 0) {
      showCurrentParameterPage("LFO3 Depth", String("Off"));
    } else {
      showCurrentParameterPage("LFO3 Depth", String(lfo3_depth));
    }
    startParameterDisplay();
  }

  if (lfo_select_temp != 2) {
    midiCCOut(CClfo_select, 2);
    delay(1);
    lfo_select_temp = 2;
  }

  if (lfo3_depth != pre_lfo3_depth) {
    midiCCOut(CClfo3_depth, lfo3_depth);
    pre_lfo3_depth = lfo3_depth;
  }
}

void updatelfo3_speed() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO3 Speed", String(lfo3_speed));
    startParameterDisplay();
  }

  if (lfo_select_temp != 2) {
    midiCCOut(CClfo_select, 2);
    delay(1);
    lfo_select_temp = 2;
  }

  if (lfo3_speed != pre_lfo3_speed) {
    midiCCOut(CClfo3_speed, lfo3_speed);
    pre_lfo3_speed = lfo3_speed;
  }
}

void updatelfo3_delay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO3 Delay", String(lfo3_delay));
    startParameterDisplay();
  }

  if (lfo_select_temp != 2) {
    midiCCOut(CClfo_select, 2);
    delay(1);
    lfo_select_temp = 2;
  }

  if (lfo3_delay != pre_lfo3_delay) {
    midiCCOut(CClfo3_delay, lfo3_delay);
    pre_lfo3_delay = lfo3_delay;
  }
}

void updateeg1_attack() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Attack", String(eg1_attack));
    startParameterDisplay();
  }

  if (eg_select_temp != 0) {
    midiCCOut(CCeg_select, 0);
    delay(1);
    eg_select_temp = 0;
  }
  if (eg1_attack != pre_eg1_attack) {
    midiCCOut(CCeg1_attack, eg1_attack);
    pre_eg1_attack = eg1_attack;
  }
}

void updateeg1_decay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Decay", String(eg1_decay));
    startParameterDisplay();
  }

  if (eg_select_temp != 0) {
    midiCCOut(CCeg_select, 0);
    delay(1);
    eg_select_temp = 0;
  }
  if (eg1_decay != pre_eg1_decay) {
    midiCCOut(CCeg1_decay, eg1_decay);
    pre_eg1_decay = eg1_decay;
  }
}

void updateeg1_release() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Release", String(eg1_release));
    startParameterDisplay();
  }

  if (eg_select_temp != 0) {
    midiCCOut(CCeg_select, 0);
    delay(1);
    eg_select_temp = 0;
  }
  if (eg1_release != pre_eg1_release) {
    midiCCOut(CCeg1_release, eg1_release);
    pre_eg1_release = eg1_release;
  }
}

void updateeg1_sustain() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Sustain", String(eg1_sustain));
    startParameterDisplay();
  }

  if (eg_select_temp != 0) {
    midiCCOut(CCeg_select, 0);
    delay(1);
    eg_select_temp = 0;
  }
  if (eg1_sustain != pre_eg1_sustain) {
    midiCCOut(CCeg1_sustain, eg1_sustain);
    pre_eg1_sustain = eg1_sustain;
  }
}

void updateeg1_key_follow() {
  if (!recallPatchFlag) {
    if (eg1_key_follow == 0) {
      showCurrentParameterPage("EG1 K.Follow", String("Off"));
    } else {
      showCurrentParameterPage("EG1 K.Follow", String(eg1_key_follow));
    }
    startParameterDisplay();
  }

  if (eg_select_temp != 0) {
    midiCCOut(CCeg_select, 0);
    delay(1);
    eg_select_temp = 0;
  }
  if (eg1_key_follow != pre_eg1_key_follow) {
    midiCCOut(CCeg1_key_follow, eg1_key_follow);
    pre_eg1_key_follow = eg1_key_follow;
  }
}

void updateeg2_attack() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Attack", String(eg2_attack));
    startParameterDisplay();
  }

  if (eg_select_temp != 2) {
    midiCCOut(CCeg_select, 2);
    delay(1);
    eg_select_temp = 2;
  }

  if (eg2_attack != pre_eg2_attack) {
    midiCCOut(CCeg2_attack, eg2_attack);
    pre_eg2_attack = eg2_attack;
  }
}

void updateeg2_decay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Decay", String(eg2_decay));
    startParameterDisplay();
  }

  if (eg_select_temp != 2) {
    midiCCOut(CCeg_select, 2);
    delay(1);
    eg_select_temp = 2;
  }

  if (eg2_decay != pre_eg2_decay) {
    midiCCOut(CCeg2_decay, eg2_decay);
    pre_eg2_decay = eg2_decay;
  }
}

void updateeg2_release() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Release", String(eg2_release));
    startParameterDisplay();
  }

  if (eg_select_temp != 2) {
    midiCCOut(CCeg_select, 2);
    delay(1);
    eg_select_temp = 2;
  }

  if (eg2_release != pre_eg2_release) {
    midiCCOut(CCeg2_release, eg2_release);
    pre_eg2_release = eg2_release;
  }
}

void updateeg2_sustain() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Sustain", String(eg2_sustain));
    startParameterDisplay();
  }

  if (eg_select_temp != 2) {
    midiCCOut(CCeg_select, 2);
    delay(1);
    eg_select_temp = 2;
  }
  if (eg2_sustain != pre_eg2_sustain) {
    midiCCOut(CCeg2_sustain, eg2_sustain);
    pre_eg2_sustain = eg2_sustain;
  }
}

void updateeg2_key_follow() {
  if (!recallPatchFlag) {
    if (eg2_key_follow == 0) {
      showCurrentParameterPage("EG2 K.Follow", String("Off"));
    } else {
      showCurrentParameterPage("EG2 K.Follow", String(eg2_key_follow));
    }
    startParameterDisplay();
  }

  if (eg_select_temp != 2) {
    midiCCOut(CCeg_select, 2);
    delay(1);
    eg_select_temp = 2;
  }
  if (eg2_key_follow != pre_eg2_key_follow) {
    midiCCOut(CCeg2_key_follow, eg2_key_follow);
    pre_eg2_key_follow = eg2_key_follow;
  }
}

void updatevca_key_velocity() {
  if (!recallPatchFlag) {
    if (vca_key_velocity == 0) {
      showCurrentParameterPage("VCA K.Velocity", String("Off"));
    } else {
      showCurrentParameterPage("VCA K.Velocity", String(vca_key_velocity));
    }
    startParameterDisplay();
  }
  if (vca_key_velocity != pre_vca_key_velocity) {
    midiCCOut(CCvca_key_velocity, vca_key_velocity);
    pre_vca_key_velocity = vca_key_velocity;
  }
}

void updatevca_level() {
  if (!recallPatchFlag) {
    if (vca_level == 0) {
      showCurrentParameterPage("VCA Level", String("Off"));
    } else {
      showCurrentParameterPage("VCA Level", String(vca_level));
    }
    startParameterDisplay();
  }
  if (vca_level != pre_vca_level) {
    midiCCOut(CCvca_level, vca_level);
    pre_vca_level = vca_level;
  }
}

// Buttons

void updateosc1_octave() {
  if (!recallPatchFlag) {
    switch (osc1_octave) {
      case 2:
        showCurrentParameterPage("Osc1 Octave", String("4 Foot"));
        break;
      case 1:
        showCurrentParameterPage("Osc1 Octave", String("8 Foot"));
        break;
      case 0:
        showCurrentParameterPage("Osc1 Octave", String("16 Foot"));
        break;
    }
    startParameterDisplay();
  }
  if (osc1_octave != pre_osc1_octave) {
    switch (osc1_octave) {
      case 2:
        mcp1.digitalWrite(OSC1_OCTAVE_LED_RED, HIGH);
        mcp1.digitalWrite(OSC1_OCTAVE_LED_GREEN, HIGH);
        break;
      case 1:
        mcp1.digitalWrite(OSC1_OCTAVE_LED_RED, LOW);
        mcp1.digitalWrite(OSC1_OCTAVE_LED_GREEN, HIGH);
        break;
      case 0:
        mcp1.digitalWrite(OSC1_OCTAVE_LED_RED, HIGH);
        mcp1.digitalWrite(OSC1_OCTAVE_LED_GREEN, LOW);
        break;
    }
    midiCCOut(CCosc1_octave, osc1_octave);
    pre_osc1_octave = osc1_octave;
  }
}

void updateosc1_wave() {
  if (!recallPatchFlag) {
    switch (osc1_wave) {
      case 0:
        showCurrentParameterPage("Osc1 Wave", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Osc1 Wave", String("Sawtooth"));
        break;
      case 2:
        showCurrentParameterPage("Osc1 Wave", String("Pulse"));
        break;
      case 3:
        showCurrentParameterPage("Osc1 Wave", String("Saw & Pulse"));
        break;
    }
    startParameterDisplay();
  }
  if (osc1_wave != pre_osc1_wave) {
    switch (osc1_wave) {
      case 0:
        mcp1.digitalWrite(OSC1_WAVE_LED_RED, LOW);
        mcp1.digitalWrite(OSC1_WAVE_LED_GREEN, LOW);
        break;
      case 1:
        mcp1.digitalWrite(OSC1_WAVE_LED_RED, HIGH);
        mcp1.digitalWrite(OSC1_WAVE_LED_GREEN, LOW);
        break;
      case 2:
        mcp1.digitalWrite(OSC1_WAVE_LED_RED, LOW);
        mcp1.digitalWrite(OSC1_WAVE_LED_GREEN, HIGH);
        break;
      case 3:
        mcp1.digitalWrite(OSC1_WAVE_LED_RED, HIGH);
        mcp1.digitalWrite(OSC1_WAVE_LED_GREEN, HIGH);
        break;
    }
    midiCCOut(CCosc1_wave, osc1_wave);
    pre_osc1_wave = osc1_wave;
  }
}

void updateosc1_sub() {
  if (!recallPatchFlag) {
    switch (osc1_sub) {
      case 0:
        showCurrentParameterPage("Osc1 Sub", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Osc1 Sub", String("On"));
        break;
    }
    startParameterDisplay();
  }
  if (osc1_sub != pre_osc1_sub) {
    switch (osc1_sub) {
      case 0:
        mcp1.digitalWrite(OSC1_SUB_LED, LOW);
        break;
      case 1:
        mcp1.digitalWrite(OSC1_SUB_LED, HIGH);
        break;
    }
    midiCCOut(CCosc1_sub, osc1_sub);
    pre_osc1_sub = osc1_sub;
  }
}

void updateosc2_wave() {
  if (!recallPatchFlag) {
    switch (osc2_wave) {
      case 0:
        showCurrentParameterPage("Osc2 Wave", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Osc2 Wave", String("Sawtooth"));
        break;
      case 2:
        showCurrentParameterPage("Osc2 Wave", String("Pulse"));
        break;
      case 3:
        showCurrentParameterPage("Osc2 Wave", String("Saw & Pulse"));
        break;
    }
    startParameterDisplay();
  }
  if (osc2_wave != pre_osc2_wave) {
    switch (osc2_wave) {
      case 0:
        mcp2.digitalWrite(OSC2_WAVE_LED_RED, LOW);
        mcp2.digitalWrite(OSC2_WAVE_LED_GREEN, LOW);
        break;
      case 1:
        mcp2.digitalWrite(OSC2_WAVE_LED_RED, HIGH);
        mcp2.digitalWrite(OSC2_WAVE_LED_GREEN, LOW);
        break;
      case 2:
        mcp2.digitalWrite(OSC2_WAVE_LED_RED, LOW);
        mcp2.digitalWrite(OSC2_WAVE_LED_GREEN, HIGH);
        break;
      case 3:
        mcp2.digitalWrite(OSC2_WAVE_LED_RED, HIGH);
        mcp2.digitalWrite(OSC2_WAVE_LED_GREEN, HIGH);
        break;
    }
    midiCCOut(CCosc2_wave, osc2_wave);
    pre_osc2_wave = osc2_wave;
  }
}

void updateosc2_xmod() {
  if (!recallPatchFlag) {
    switch (osc2_xmod) {
      case 0:
        showCurrentParameterPage("Cross Mod", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Cross Mod", String("Osc1 = Osc2"));
        break;
      case 2:
        showCurrentParameterPage("Cross Mod", String("Osc2 >> Osc1"));
        break;
    }
    startParameterDisplay();
  }
  if (osc2_xmod != pre_osc2_xmod) {
    switch (osc2_xmod) {
      case 0:
        mcp2.digitalWrite(OSC2_XMOD_LED_RED, LOW);
        mcp2.digitalWrite(OSC2_XMOD_LED_GREEN, LOW);
        break;
      case 1:
        mcp2.digitalWrite(OSC2_XMOD_LED_RED, HIGH);
        mcp2.digitalWrite(OSC2_XMOD_LED_GREEN, LOW);
        break;
      case 2:
        mcp2.digitalWrite(OSC2_XMOD_LED_RED, LOW);
        mcp2.digitalWrite(OSC2_XMOD_LED_GREEN, HIGH);
        break;
    }
    midiCCOut(CCosc2_xmod, osc2_xmod);
    pre_osc2_xmod = osc2_xmod;
  }
}

void updateosc2_eg_select() {
  if (!recallPatchFlag) {
    switch (osc2_eg_select) {
      case 0:
        showCurrentParameterPage("EG Select", String("VCF"));
        break;
      case 1:
        showCurrentParameterPage("EG Select", String("VCA"));
        break;
    }
    startParameterDisplay();
  }
  if (osc2_eg_select != pre_osc2_eg_select) {
    switch (osc2_eg_select) {
      case 0:
        mcp2.digitalWrite(OSC2_EG_SELECT_LED_RED, HIGH);
        mcp3.digitalWrite(OSC2_EG_SELECT_LED_GREEN, LOW);
        break;
      case 1:
        mcp2.digitalWrite(OSC2_EG_SELECT_LED_RED, LOW);
        mcp3.digitalWrite(OSC2_EG_SELECT_LED_GREEN, HIGH);
        break;
    }
    midiCCOut(CCosc2_eg_select, osc2_eg_select);
    pre_osc2_eg_select = osc2_eg_select;
  }
}

void updatelfo1_wave() {
  if (!recallPatchFlag) {
    switch (lfo1_wave) {
      case 0:
        showCurrentParameterPage("LFO1 Wave", String("Pulse"));
        break;
      case 1:
        showCurrentParameterPage("LFO1 Wave", String("Saw Down"));
        break;
      case 2:
        showCurrentParameterPage("LFO1 Wave", String("Saw Up"));
        break;
      case 3:
        showCurrentParameterPage("LFO1 Wave", String("Triangle"));
        break;
    }
    startParameterDisplay();
  }

  switch (lfo1_wave) {
    case 0:
      mcp7.digitalWrite(LFO1_WAVE_LED_RED, LOW);
      mcp7.digitalWrite(LFO1_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      mcp7.digitalWrite(LFO1_WAVE_LED_RED, HIGH);
      mcp7.digitalWrite(LFO1_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      mcp7.digitalWrite(LFO1_WAVE_LED_RED, LOW);
      mcp7.digitalWrite(LFO1_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      mcp7.digitalWrite(LFO1_WAVE_LED_RED, HIGH);
      mcp7.digitalWrite(LFO1_WAVE_LED_GREEN, HIGH);
      break;
  }

  if (lfo_select_temp != 0) {
    midiCCOut(CClfo_select, 0);
    delay(1);
    lfo_select_temp = 0;
  }
  if (lfo1_wave != pre_lfo1_wave) {
    midiCCOut(CClfo1_wave, lfo1_wave);
    pre_lfo1_wave = lfo1_wave;
  }
}

void updatelfo2_wave() {
  if (!recallPatchFlag) {
    switch (lfo2_wave) {
      case 0:
        showCurrentParameterPage("LFO2 Wave", String("Pulse"));
        break;
      case 1:
        showCurrentParameterPage("LFO2 Wave", String("Saw Down"));
        break;
      case 2:
        showCurrentParameterPage("LFO2 Wave", String("Saw Up"));
        break;
      case 3:
        showCurrentParameterPage("LFO2 Wave", String("Triangle"));
        break;
    }
    startParameterDisplay();
  }
  switch (lfo2_wave) {
    case 0:
      mcp7.digitalWrite(LFO2_WAVE_LED_RED, LOW);
      mcp7.digitalWrite(LFO2_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      mcp7.digitalWrite(LFO2_WAVE_LED_RED, HIGH);
      mcp7.digitalWrite(LFO2_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      mcp7.digitalWrite(LFO2_WAVE_LED_RED, LOW);
      mcp7.digitalWrite(LFO2_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      mcp7.digitalWrite(LFO2_WAVE_LED_RED, HIGH);
      mcp7.digitalWrite(LFO2_WAVE_LED_GREEN, HIGH);
      break;
  }

  if (lfo_select_temp != 1) {
    midiCCOut(CClfo_select, 1);
    delay(1);
    lfo_select_temp = 1;
  }

  if (lfo2_wave != pre_lfo2_wave) {
    midiCCOut(CClfo2_wave, lfo2_wave);
    pre_lfo2_wave = lfo2_wave;
  }
}

void updatelfo3_wave() {
  if (!recallPatchFlag) {
    switch (lfo3_wave) {
      case 0:
        showCurrentParameterPage("LFO3 Wave", String("Pulse"));
        break;
      case 1:
        showCurrentParameterPage("LFO3 Wave", String("Saw Down"));
        break;
      case 2:
        showCurrentParameterPage("LFO3 Wave", String("Saw Up"));
        break;
      case 3:
        showCurrentParameterPage("LFO3 Wave", String("Triangle"));
        break;
    }
    startParameterDisplay();
  }
  switch (lfo3_wave) {
    case 0:
      mcp4.digitalWrite(LFO3_WAVE_LED_RED, LOW);
      mcp4.digitalWrite(LFO3_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      mcp4.digitalWrite(LFO3_WAVE_LED_RED, HIGH);
      mcp4.digitalWrite(LFO3_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      mcp4.digitalWrite(LFO3_WAVE_LED_RED, LOW);
      mcp4.digitalWrite(LFO3_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      mcp4.digitalWrite(LFO3_WAVE_LED_RED, HIGH);
      mcp4.digitalWrite(LFO3_WAVE_LED_GREEN, HIGH);
      break;
  }

  if (lfo_select_temp != 2) {
    midiCCOut(CClfo_select, 2);
    delay(1);
    lfo_select_temp = 2;
  }

  if (lfo3_wave != pre_lfo3_wave) {
    midiCCOut(CClfo3_wave, lfo3_wave);
    pre_lfo3_wave = lfo3_wave;
  }
}

void updatelfo_select() {
  if (!recallPatchFlag) {
    switch (lfo_select) {
      case 0:
        showCurrentParameterPage("LFO Display", String("LFO 1"));
        break;
      case 1:
        showCurrentParameterPage("LFO Display", String("LFO 2"));
        break;
      case 2:
        showCurrentParameterPage("LFO Display", String("LFO 3"));
        break;
    }
    startParameterDisplay();
    lfo_select_temp = lfo_select;
  }
  if (!recallPatchFlag) {
    switch (lfo_select) {
      case 0:
        mcp4.digitalWrite(LFO_SELECT_LED_RED, HIGH);
        mcp4.digitalWrite(LFO_SELECT_LED_GREEN, LOW);
        break;
      case 1:
        mcp4.digitalWrite(LFO_SELECT_LED_RED, LOW);
        mcp4.digitalWrite(LFO_SELECT_LED_GREEN, HIGH);
        break;
      case 2:
        mcp4.digitalWrite(LFO_SELECT_LED_RED, HIGH);
        mcp4.digitalWrite(LFO_SELECT_LED_GREEN, HIGH);
        break;
    }
    midiCCOut(CClfo_select, lfo_select);
  }
}

void updateeg_select() {
  if (!recallPatchFlag) {
    switch (eg_select) {
      case 0:
        showCurrentParameterPage("EG Display", String("VCA"));
        break;
      case 1:
        showCurrentParameterPage("EG Display", String("VCA - VCF"));
        break;
      case 2:
        showCurrentParameterPage("EG Display", String("VCF"));
        break;
    }
    startParameterDisplay();
    eg_select_temp = eg_select;
  }
  if (!recallPatchFlag) {
    switch (eg_select) {
      case 0:
        mcp5.digitalWrite(EG_DEST_LED_RED, HIGH);
        mcp5.digitalWrite(EG_DEST_LED_GREEN, LOW);
        break;
      case 1:
        mcp5.digitalWrite(EG_DEST_LED_RED, HIGH);
        mcp5.digitalWrite(EG_DEST_LED_GREEN, HIGH);
        break;
      case 2:
        mcp5.digitalWrite(EG_DEST_LED_RED, LOW);
        mcp5.digitalWrite(EG_DEST_LED_GREEN, HIGH);
        break;
    }
    midiCCOut(CCeg_select, eg_select);
  }
}

void startParameterDisplay() {
  refreshScreen();

  lastDisplayTriggerTime = millis();
  waitingToUpdate = true;
}

void updatePatchname() {
  showPatchPage(String(patchNo), patchName);
}

void myControlChange(byte channel, byte control, int value) {
  switch (control) {

    case CCosc1_PW:
      osc1_pw = value;
      updateosc1_PW();
      break;

    case CCosc1_PWM:
      osc1_pwm = value;
      updateosc1_PWM();
      break;

    case CCosc1_level:
      osc1_level = value;
      updateosc1_level();
      break;

    case CCosc2_freq:
      osc2_freq = value;
      updateosc2_freq();
      break;

    case CCosc2_eg_depth:
      osc2_eg_depth = value;
      updateosc2_eg_depth();
      break;

    case CCosc2_detune:
      osc2_detune = value;
      updateosc2_detune();
      break;

    case CCosc2_level:
      osc2_level = value;
      updateosc2_level();
      break;

    case CCvcf_cutoff:
      vcf_cutoff = value;
      updatevcf_cutoff();
      break;

    case CCvcf_res:
      vcf_res = value;
      updatevcf_res();
      break;

    case CCvcf_eg_depth:
      vcf_eg_depth = value;
      updatevcf_eg_depth();
      break;

    case CCvcf_key_follow:
      vcf_key_follow = value;
      updatevcf_key_follow();
      break;

    case CCvcf_key_velocity:
      vcf_key_velocity = value;
      updatevcf_key_velocity();
      break;

    case CCvcf_hpf:
      vcf_hpf = value;
      updatevcf_hpf();
      break;

    case CClfo1_depth:
      lfo1_depth = value;
      updatelfo1_depth();
      break;

    case CClfo1_speed:
      lfo1_speed = value;
      updatelfo1_speed();
      break;

    case CClfo1_delay:
      lfo1_delay = value;
      updatelfo1_delay();
      break;

    case CClfo2_depth:
      lfo2_depth = value;
      updatelfo2_depth();
      break;

    case CClfo2_speed:
      lfo2_speed = value;
      updatelfo2_speed();
      break;

    case CClfo2_delay:
      lfo2_delay = value;
      updatelfo2_delay();
      break;

    case CClfo3_depth:
      lfo3_depth = value;
      updatelfo3_depth();
      break;

    case CClfo3_speed:
      lfo3_speed = value;
      updatelfo3_speed();
      break;

    case CClfo3_delay:
      lfo3_delay = value;
      updatelfo3_delay();
      break;

    case CCeg1_attack:
      eg1_attack = value;
      updateeg1_attack();
      break;

    case CCeg1_decay:
      eg1_decay = value;
      updateeg1_decay();
      break;

    case CCeg1_release:
      eg1_release = value;
      updateeg1_release();
      break;

    case CCeg1_sustain:
      eg1_sustain = value;
      updateeg1_sustain();
      break;

    case CCeg1_key_follow:
      eg1_key_follow = value;
      updateeg1_key_follow();
      break;

    case CCeg2_attack:
      eg2_attack = value;
      updateeg2_attack();
      break;

    case CCeg2_decay:
      eg2_decay = value;
      updateeg2_decay();
      break;

    case CCeg2_release:
      eg2_release = value;
      updateeg2_release();
      break;

    case CCeg2_sustain:
      eg2_sustain = value;
      updateeg2_sustain();
      break;

    case CCeg2_key_follow:
      eg2_key_follow = value;
      updateeg2_key_follow();
      break;

    case CCvca_key_velocity:
      vca_key_velocity = value;
      updatevca_key_velocity();
      break;

    case CCvca_level:
      vca_level = value;
      updatevca_level();
      break;

    case CCosc1_octave:
      osc1_octave = value;
      updateosc1_octave();
      break;

    case CCosc1_wave:
      osc1_wave = value;
      updateosc1_wave();
      break;

    case CCosc1_sub:
      value > 0 ? osc1_sub = 1 : osc1_sub = 0;
      updateosc1_sub();
      break;

    case CCosc2_wave:
      osc2_wave = value;
      updateosc2_wave();
      break;

    case CCosc2_xmod:
      osc2_xmod = value;
      updateosc2_xmod();
      break;

    case CCosc2_eg_select:
      osc2_eg_select = value;
      updateosc2_eg_select();
      break;

    case CClfo1_wave:
      lfo1_wave = value;
      updatelfo1_wave();
      break;

    case CClfo2_wave:
      lfo2_wave = value;
      updatelfo2_wave();
      break;

    case CClfo3_wave:
      lfo3_wave = value;
      updatelfo3_wave();
      break;

    case CClfo_select:
      lfo_select = value;
      updatelfo_select();
      break;

    case CCeg_select:
      eg_select = value;
      updateeg_select();
      break;

    case CCallnotesoff:
      allNotesOff();
      break;
  }
}

void myProgramChange(byte channel, byte program) {
  state = PATCH;
  patchNo = program + 1;
  recallPatch(patchNo);
  Serial.print("MIDI Pgm Change:");
  Serial.println(patchNo);
  state = PARAMETER;
}

void recallPatch(int patchNo) {
  allNotesOff();
  if (!updateParams) {
    MIDI.sendProgramChange(patchNo - 1, midiOutCh);
  }
  delay(50);
  recallPatchFlag = true;
  File patchFile = SD.open(String(patchNo).c_str());
  if (!patchFile) {
    Serial.println("File not found");
  } else {
    String data[NO_OF_PARAMS];  //Array of data read in
    recallPatchData(patchFile, data);
    setCurrentPatchData(data);
    patchFile.close();
  }
  recallPatchFlag = false;
}

void setCurrentPatchData(String data[]) {
  patchName = data[0];
  osc1_octave = data[1].toInt();
  osc1_wave = data[2].toInt();
  osc1_pw = data[3].toInt();
  osc1_pwm = data[4].toInt();
  osc1_sub = data[5].toInt();
  osc1_level = data[6].toInt();
  osc2_freq = data[7].toInt();
  osc2_detune = data[8].toInt();
  osc2_wave = data[9].toInt();
  osc2_xmod = data[10].toInt();
  osc2_eg_depth = data[11].toInt();
  osc2_eg_select = data[12].toInt();
  osc2_level = data[13].toInt();
  vcf_cutoff = data[14].toInt();
  vcf_res = data[15].toInt();
  vcf_eg_depth = data[16].toInt();
  vcf_key_follow = data[17].toInt();
  vcf_key_velocity = data[18].toInt();
  vcf_hpf = data[19].toInt();
  lfo1_depth = data[20].toInt();
  lfo1_speed = data[21].toInt();
  lfo1_delay = data[22].toInt();
  lfo1_wave = data[23].toInt();
  lfo_select = data[24].toInt();
  eg1_attack = data[25].toInt();
  eg1_decay = data[26].toInt();
  eg1_sustain = data[27].toInt();
  eg1_release = data[28].toInt();
  eg1_key_follow = data[29].toInt();
  eg_select = data[30].toInt();
  vca_key_velocity = data[31].toInt();
  vca_level = data[32].toInt();
  lfo2_depth = data[33].toInt();
  lfo2_speed = data[34].toInt();
  lfo2_delay = data[35].toInt();
  lfo2_wave = data[36].toInt();
  lfo3_depth = data[37].toInt();
  lfo3_speed = data[38].toInt();
  lfo3_delay = data[39].toInt();
  lfo3_wave = data[40].toInt();
  eg2_attack = data[41].toInt();
  eg2_decay = data[42].toInt();
  eg2_sustain = data[43].toInt();
  eg2_release = data[44].toInt();
  eg2_key_follow = data[45].toInt();

  updateosc1_sub();
  updateosc1_octave();
  updateosc1_wave();
  updateosc2_wave();
  updateosc2_xmod();
  updateosc2_eg_select();
  updatelfo1_wave();
  updatelfo2_wave();
  updatelfo3_wave();
  updatelfo_select();
  updateeg_select();

  //Patchname
  updatePatchname();

  Serial.print("Set Patch: ");
  Serial.println(patchName);
  if (updateParams) {
    sendToSynthData();
  }
}

void sendToSynthData() {

  updateosc1_octave();
  updateosc1_wave();
  updateosc1_PW();
  updateosc1_PWM();
  updateosc1_sub();
  updateosc1_level();
  updateosc2_freq();
  updateosc2_detune();
  updateosc2_wave();
  updateosc2_xmod();
  updateosc2_eg_depth();
  updateosc2_eg_select();
  updateosc2_level();
  updatevcf_cutoff();
  updatevcf_res();
  updatevcf_eg_depth();
  updatevcf_key_velocity();
  updatevcf_key_follow();
  updatevcf_hpf();
  updatelfo1_depth();
  updatelfo1_speed();
  updatelfo1_delay();
  updatelfo1_wave();
  updatelfo2_depth();
  updatelfo2_speed();
  updatelfo2_delay();
  updatelfo2_wave();
  updatelfo3_depth();
  updatelfo3_speed();
  updatelfo3_delay();
  updatelfo3_wave();
  updateeg1_attack();
  updateeg1_decay();
  updateeg1_sustain();
  updateeg1_release();
  updateeg1_key_follow();
  updateeg2_attack();
  updateeg2_decay();
  updateeg2_sustain();
  updateeg2_release();
  updateeg2_key_follow();
  updateeg_select();
  updatevca_key_velocity();
  updatevca_level();
  updatelfo_select();
}

String getCurrentPatchData() {
  return patchName + "," + String(osc1_octave) + "," + String(osc1_wave) + "," + String(osc1_pw) + "," + String(osc1_pwm) + "," + String(osc1_sub) + "," + String(osc1_level)
         + "," + String(osc2_freq) + "," + String(osc2_detune) + "," + String(osc2_wave) + "," + String(osc2_xmod) + "," + String(osc2_eg_depth) + "," + String(osc2_eg_select) + "," + String(osc2_level)
         + "," + String(vcf_cutoff) + "," + String(vcf_res) + "," + String(vcf_eg_depth) + "," + String(vcf_key_follow) + "," + String(vcf_key_velocity) + "," + String(vcf_hpf)
         + "," + String(lfo1_depth) + "," + String(lfo1_speed) + "," + String(lfo1_delay) + "," + String(lfo1_wave) + "," + String(lfo_select)
         + "," + String(eg1_attack) + "," + String(eg1_decay) + "," + String(eg1_sustain) + "," + String(eg1_release) + "," + String(eg1_key_follow) + "," + String(eg_select)
         + "," + String(vca_key_velocity) + "," + String(vca_level)
         + "," + String(lfo2_depth) + "," + String(lfo2_speed) + "," + String(lfo2_delay) + "," + String(lfo2_wave)
         + "," + String(lfo3_depth) + "," + String(lfo3_speed) + "," + String(lfo3_delay) + "," + String(lfo3_wave)
         + "," + String(eg2_attack) + "," + String(eg2_decay) + "," + String(eg2_sustain) + "," + String(eg2_release) + "," + String(eg2_key_follow);
}

void showSettingsPage() {
  showSettingsPage(settings::current_setting(), settings::current_setting_value(), state);
}

void midiCCOut(byte cc, byte value) {
  MIDI.sendControlChange(cc, value, midiOutCh);  //MIDI DIN is set to Out
  if (updateParams) {
    delay(2);
  }
}

void checkSwitches() {

  saveButton.update();
  if (saveButton.held()) {
    switch (state) {
      case PARAMETER:
      case PATCH:
        state = DELETE;
        break;
    }
    refreshScreen();
  } else if (saveButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        if (patches.size() < PATCHES_LIMIT) {
          resetPatchesOrdering();  //Reset order of patches from first patch
          patches.push({ patches.size() + 1, INITPATCHNAME });
          state = SAVE;
        }
        refreshScreen();
        break;
      case SAVE:
        //Save as new patch with INITIALPATCH name or overwrite existing keeping name - bypassing patch renaming
        patchName = patches.last().patchName;
        state = PATCH;
        savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
        showPatchPage(patches.last().patchNo, patches.last().patchName);
        patchNo = patches.last().patchNo;
        loadPatches();  //Get rid of pushed patch if it wasn't saved
        setPatchesOrdering(patchNo);
        renamedPatch = "";
        state = PARAMETER;
        refreshScreen();
        break;
      case PATCHNAMING:
        if (renamedPatch.length() > 0) patchName = renamedPatch;  //Prevent empty strings
        state = PATCH;
        savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
        showPatchPage(patches.last().patchNo, patchName);
        patchNo = patches.last().patchNo;
        loadPatches();  //Get rid of pushed patch if it wasn't saved
        setPatchesOrdering(patchNo);
        renamedPatch = "";
        state = PARAMETER;
        refreshScreen();
        break;
    }
  }

  settingsButton.update();
  if (settingsButton.held()) {
    //If recall held, set current patch to match current hardware state
    //Reinitialise all hardware values to force them to be re-read if different
    state = REINITIALISE;
    reinitialiseToPanel();
    refreshScreen();
  } else if (settingsButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;
      case SETTINGS:
        showSettingsPage();
        refreshScreen();
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;
    }
  }

  backButton.update();
  if (backButton.held()) {
    //If Back button held, Panic - all notes off
  } else if (backButton.numClicks() == 1) {
    switch (state) {
      case RECALL:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        refreshScreen();
        break;
      case SAVE:
        renamedPatch = "";
        state = PARAMETER;
        loadPatches();  //Remove patch that was to be saved
        setPatchesOrdering(patchNo);
        refreshScreen();
        break;
      case PATCHNAMING:
        charIndex = 0;
        renamedPatch = "";
        state = SAVE;
        refreshScreen();
        break;
      case DELETE:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        refreshScreen();
        break;
      case SETTINGS:
        state = PARAMETER;
        refreshScreen();
        break;
      case SETTINGSVALUE:
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;
    }
  }

  //Encoder switch
  recallButton.update();
  if (recallButton.held()) {
    //If Recall button held, return to current patch setting
    //which clears any changes made
    state = PATCH;
    //Recall the current patch
    patchNo = patches.first().patchNo;
    recallPatch(patchNo);
    state = PARAMETER;
    refreshScreen();
  } else if (recallButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = RECALL;  //show patch list
        refreshScreen();
        break;
      case RECALL:
        state = PATCH;
        //Recall the current patch
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        refreshScreen();
        break;
      case SAVE:
        showRenamingPage(patches.last().patchName);
        patchName = patches.last().patchName;
        state = PATCHNAMING;
        refreshScreen();
        break;
      case PATCHNAMING:
        if (renamedPatch.length() < 12)  //actually 12 chars
        {
          renamedPatch.concat(String(currentCharacter));
          charIndex = 0;
          currentCharacter = CHARACTERS[charIndex];
          showRenamingPage(renamedPatch);
        }
        refreshScreen();
        break;
      case DELETE:
        //Don't delete final patch
        if (patches.size() > 1) {
          state = DELETEMSG;
          patchNo = patches.first().patchNo;     //PatchNo to delete from SD card
          patches.shift();                       //Remove patch from circular buffer
          deletePatch(String(patchNo).c_str());  //Delete from SD card
          loadPatches();                         //Repopulate circular buffer to start from lowest Patch No
          renumberPatchesOnSD();
          loadPatches();                      //Repopulate circular buffer again after delete
          patchNo = patches.first().patchNo;  //Go back to 1
          recallPatch(patchNo);               //Load first patch
        }
        state = PARAMETER;
        refreshScreen();
        break;
      case SETTINGS:
        state = SETTINGSVALUE;
        showSettingsPage();
        refreshScreen();
        break;
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;
    }
  }
}

void reinitialiseToPanel() {

  osc1_octave = 0;
  osc1_wave = 1;
  osc1_pw = 50;
  osc1_pwm = 0;
  osc1_sub = 0;
  osc1_level = 99;

  osc2_freq = 0;
  osc2_detune = 50;
  osc2_wave = 1;
  osc2_xmod = 0;
  osc2_eg_depth = 50;
  osc2_eg_select = 0;
  osc2_level = 99;

  vcf_cutoff = 99;
  vcf_res = 0;
  vcf_eg_depth = 50;
  vcf_key_follow = 0;
  vcf_key_velocity = 0;
  vcf_hpf = 0;

  lfo1_depth = 0;
  lfo1_speed = 50;
  lfo1_delay = 0;
  lfo1_wave = 0;
  lfo_select = 0;

  eg1_attack = 0;
  eg1_decay = 0;
  eg1_sustain = 99;
  eg1_release = 0;
  eg1_key_follow = 0;
  eg_select = 0;

  vca_key_velocity = 0;
  vca_level = 99;

  lfo2_depth = 0;
  lfo2_speed = 50;
  lfo2_delay = 0;
  lfo2_wave = 0;

  lfo3_depth = 0;
  lfo3_speed = 50;
  lfo3_delay = 0;
  lfo3_wave = 0;

  eg2_attack = 0;
  eg2_decay = 0;
  eg2_sustain = 99;
  eg2_release = 0;
  eg2_key_follow = 0;

  recallPatchFlag = true;
  sendToSynthData();
  recallPatchFlag = false;
}

void checkEncoder() {
  //Encoder works with relative inc and dec values
  //Detent encoder goes up in 4 steps, hence +/-3

  long encRead = encoder.read();
  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        patches.push(patches.shift());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        refreshScreen();
        break;
      case RECALL:
        patches.push(patches.shift());
        refreshScreen();
        break;
      case SAVE:
        patches.push(patches.shift());
        refreshScreen();
        break;
      case PATCHNAMING:
        if (charIndex == TOTALCHARS) charIndex = 0;  //Wrap around
        currentCharacter = CHARACTERS[charIndex++];
        showRenamingPage(renamedPatch + currentCharacter);
        refreshScreen();
        break;
      case DELETE:
        patches.push(patches.shift());
        refreshScreen();
        break;
      case SETTINGS:
        settings::increment_setting();
        showSettingsPage();
        refreshScreen();
        break;
      case SETTINGSVALUE:
        settings::increment_setting_value();
        showSettingsPage();
        refreshScreen();
        break;
    }
    encPrevious = encRead;
  } else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        patches.unshift(patches.pop());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        refreshScreen();
        break;
      case RECALL:
        patches.unshift(patches.pop());
        refreshScreen();
        break;
      case SAVE:
        patches.unshift(patches.pop());
        refreshScreen();
        break;
      case PATCHNAMING:
        if (charIndex == -1)
          charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex--];
        showRenamingPage(renamedPatch + currentCharacter);
        refreshScreen();
        break;
      case DELETE:
        patches.unshift(patches.pop());
        refreshScreen();
        break;
      case SETTINGS:
        settings::decrement_setting();
        showSettingsPage();
        refreshScreen();
        break;
      case SETTINGSVALUE:
        settings::decrement_setting_value();
        showSettingsPage();
        refreshScreen();
        break;
    }
    encPrevious = encRead;
  }
}

void SaveCurrent() {
  // if (saveCurrent) {
  //   state = SETTINGS;
  //   if (midiOutCh > 0) {
  //     MIDI.sendSysEx(sizeof(saveRequest), saveRequest);
  //   }
  //   saveCurrent = false;
  //   storeSaveCurrent(saveCurrent);
  //   settings::decrement_setting_value();
  //   settings::save_current_value();
  //   showSettingsPage();
  //   delay(100);
  //   state = PARAMETER;
  //   //recallPatch(patchNo);
  // }
}

void SaveAll() {
  if (saveAll) {
    recallPatchFlag = true;
    updateParams = false;
    sysexData[0] = 0xF0;
    sysexData[1] = 0x47;
    sysexData[2] = 0x7E;
    sysexData[49] = 0xF7;
    state = SETTINGS;
    for (int row = 0; row < 64; row++) {
      recallPatch(row + 33);
      sysexData[3] = row;
      for (int i = 4; i < 49; i++) {
        switch (i) {
          case 4:
            sysexData[4] = osc1_octave;
            break;

          case 5:
            sysexData[5] = osc1_wave;
            break;

          case 6:
            sysexData[6] = osc1_pw;
            break;

          case 7:
            sysexData[7] = osc1_pwm;
            break;

          case 8:
            sysexData[8] = osc1_sub;
            break;

          case 9:
            sysexData[9] = osc1_level;
            break;

          case 10:
            sysexData[10] = osc2_freq;
            break;

          case 11:
            sysexData[11] = osc2_detune;
            break;

          case 12:
            sysexData[12] = osc2_wave;
            break;

          case 13:
            sysexData[13] = osc2_xmod;
            break;

          case 14:
            sysexData[14] = osc2_eg_depth;
            break;

          case 15:
            sysexData[15] = osc2_eg_select;
            break;

          case 16:
            sysexData[16] = osc2_level;
            break;

          case 17:
            sysexData[17] = vcf_cutoff;
            break;

          case 18:
            sysexData[18] = vcf_res;
            break;

          case 19:
            sysexData[19] = vcf_eg_depth;
            break;

          case 20:
            sysexData[20] = vcf_key_follow;
            break;

          case 21:
            sysexData[21] = vcf_key_velocity;
            break;

          case 22:
            sysexData[22] = vcf_hpf;
            break;

          case 23:
            sysexData[23] = lfo1_depth;
            break;

          case 24:
            sysexData[24] = lfo1_speed;
            break;

          case 25:
            sysexData[25] = lfo1_delay;
            break;

          case 26:
            sysexData[26] = lfo1_wave;
            break;

          case 27:
            sysexData[27] = lfo_select;
            break;

          case 28:
            sysexData[28] = eg1_attack;
            break;

          case 29:
            sysexData[29] = eg1_decay;
            break;

          case 30:
            sysexData[30] = eg1_sustain;
            break;

          case 31:
            sysexData[31] = eg1_release;
            break;

          case 32:
            sysexData[32] = eg1_key_follow;
            break;

          case 33:
            sysexData[33] = eg_select;
            break;

          case 34:
            sysexData[34] = vca_key_velocity;
            break;

          case 35:
            sysexData[35] = vca_level;
            break;

          case 36:
            sysexData[36] = lfo2_depth;
            break;

          case 37:
            sysexData[37] = lfo2_speed;
            break;

          case 38:
            sysexData[38] = lfo2_delay;
            break;

          case 39:
            sysexData[39] = lfo2_wave;
            break;

          case 40:
            sysexData[40] = lfo3_depth;
            break;

          case 41:
            sysexData[41] = lfo3_speed;
            break;

          case 42:
            sysexData[42] = lfo3_delay;
            break;

          case 43:
            sysexData[43] = lfo3_wave;
            break;

          case 44:
            sysexData[44] = eg2_attack;
            break;

          case 45:
            sysexData[45] = eg2_decay;
            break;

          case 46:
            sysexData[46] = eg2_sustain;
            break;

          case 47:
            sysexData[47] = eg2_release;
            break;

          case 48:
            sysexData[48] = eg2_key_follow;
            break;
        }
      }
      if (midiOutCh > 0) {
        MIDI.sendSysEx(sizeof(sysexData), sysexData);
      }
    }
    saveAll = false;
    storeSaveAll(saveAll);
    settings::decrement_setting_value();
    settings::save_current_value();
    showSettingsPage();
    delay(100);
    state = PARAMETER;
    recallPatch(patchNo);
  }
  recallPatchFlag = false;
  updateParams = true;
}

void checkLoadFactory() {
  if (loadFactory) {
    for (int row = 0; row < 32; row++) {
      if (ROMType) {
        currentRow = factoryI[row];
      } else {
        currentRow = factoryKL[row];
      }

      String values[46];   // Assuming you have 46 values per row
      int valueIndex = 0;  // Index for storing values
      for (unsigned int i = 0; i < currentRow.length(); i++) {
        char currentChar = currentRow.charAt(i);

        // Check for the delimiter (",") and move to the next value
        if (currentChar == ',') {
          valueIndex++;  // Move to the next value
          continue;      // Skip the delimiter
        }

        // Append the character to the current value
        values[valueIndex] += currentChar;
      }

      // Process the values
      int intValues[46];
      for (int i = 0; i < 46; i++) {  // Adjust the loop count based on the number of values per row
        switch (i) {

          case 0:
            patchName = values[i];
            break;

          case 1:
            intValues[i] = values[i].toInt();
            switch (intValues[i]) {
              case 16:
                osc1_octave = 0;
                break;
              case 8:
                osc1_octave = 1;
                break;
              case 4:
                osc1_octave = 2;
                break;
            }
            break;

          case 2:
            intValues[i] = values[i].toInt();
            osc1_wave = (intValues[i]);
            break;

          case 3:  // osc1_pw
            intValues[i] = values[i].toInt();
            osc1_pw = intValues[i];
            break;

          case 4:  // osc1_pwm
            intValues[i] = values[i].toInt();
            osc1_pwm = intValues[i];
            break;

          case 5:
            intValues[i] = values[i].toInt();
            osc1_sub = (intValues[i]);
            break;

          case 6:  // osc1_level
            intValues[i] = values[i].toInt();
            osc1_level = intValues[i];
            break;

          case 7:  // osc2_freq
            intValues[i] = values[i].toInt();
            osc2_freq = intValues[i];
            break;

          case 8:  // osc2_detune
            intValues[i] = values[i].toInt();
            osc2_detune = intValues[i];
            break;

          case 9:  // osc2_wave
            intValues[i] = values[i].toInt();
            osc2_wave = intValues[i];
            break;

          case 10:  // osc2_xmod
            intValues[i] = values[i].toInt();
            osc2_xmod = intValues[i];
            break;

          case 11:  // osc2_eg_depth
            intValues[i] = values[i].toInt();
            osc2_eg_depth = intValues[i];
            break;

          case 12:  // osc2_eg_select
            intValues[i] = values[i].toInt();
            osc2_eg_select = intValues[i];
            break;

          case 13:  // osc2_level
            intValues[i] = values[i].toInt();
            osc2_level = (intValues[i] - 1);
            break;

          case 14:  // vcf_cutoff
            intValues[i] = values[i].toInt();
            vcf_cutoff = intValues[i];
            break;

          case 15:  // vcf_res
            intValues[i] = values[i].toInt();
            vcf_res = intValues[i];
            break;

          case 16:  // vcf_eg_depth
            intValues[i] = values[i].toInt();
            vcf_eg_depth = intValues[i];
            break;

          case 17:  // vcf_key_follow
            intValues[i] = values[i].toInt();
            vcf_key_follow = intValues[i];
            break;

          case 18:  // vcf_key_velocity
            intValues[i] = values[i].toInt();
            vcf_key_velocity = intValues[i];
            break;

          case 19:  // vcf_hpf
            intValues[i] = values[i].toInt();
            vcf_hpf = intValues[i];
            break;

          case 20:  // lfo1_depth
            intValues[i] = values[i].toInt();
            lfo1_depth = intValues[i];
            break;

          case 21:  // lfo1_speed
            intValues[i] = values[i].toInt();
            lfo1_speed = intValues[i];
            break;

          case 22:  // lfo1_delay
            intValues[i] = values[i].toInt();
            lfo1_delay = intValues[i];
            break;

          case 23:  // lfo1_wave
            intValues[i] = values[i].toInt();
            switch (intValues[i]) {
              case 1:
                lfo1_wave = 0;
                break;
              case 2:
                lfo1_wave = 1;
                break;
              case 3:
                lfo1_wave = 2;
                break;
              case 4:
                lfo1_wave = 3;
                break;
            }
            break;

          case 24:
            intValues[i] = values[i].toInt();
            lfo2_depth = intValues[i];
            break;

          case 25:
            intValues[i] = values[i].toInt();
            lfo2_speed = intValues[i];
            break;

          case 26:
            intValues[i] = values[i].toInt();
            lfo2_delay = intValues[i];
            break;

          case 27:
            intValues[i] = values[i].toInt();
            switch (intValues[i]) {
              case 1:
                lfo2_wave = 0;
                break;
              case 2:
                lfo2_wave = 1;
                break;
              case 3:
                lfo2_wave = 2;
                break;
              case 4:
                lfo2_wave = 3;
                break;
            }
            break;

          case 28:
            intValues[i] = values[i].toInt();
            lfo3_depth = intValues[i];
            break;

          case 29:
            intValues[i] = values[i].toInt();
            lfo3_speed = intValues[i];
            break;

          case 30:
            intValues[i] = values[i].toInt();
            lfo3_delay = intValues[i];
            break;

          case 31:
            intValues[i] = values[i].toInt();
            switch (intValues[i]) {
              case 1:
                lfo3_wave = 0;
                break;
              case 2:
                lfo3_wave = 1;
                break;
              case 3:
                lfo3_wave = 2;
                break;
              case 4:
                lfo3_wave = 3;
                break;
            }
            break;

          case 32:
            intValues[i] = values[i].toInt();
            switch (intValues[i]) {
              case 1:
                lfo_select = 0;
                break;
              case 2:
                lfo_select = 1;
                break;
              case 3:
                lfo_select = 2;
                break;
            }
            break;

          case 33:
            intValues[i] = values[i].toInt();
            eg1_attack = intValues[i];
            break;

          case 34:
            intValues[i] = values[i].toInt();
            eg1_decay = intValues[i];
            break;

          case 35:
            intValues[i] = values[i].toInt();
            eg1_sustain = intValues[i];
            break;

          case 36:
            intValues[i] = values[i].toInt();
            eg1_release = intValues[i];
            break;

          case 37:
            intValues[i] = values[i].toInt();
            eg1_key_follow = intValues[i];
            break;

          case 38:
            intValues[i] = values[i].toInt();
            eg2_attack = intValues[i];
            break;

          case 39:
            intValues[i] = values[i].toInt();
            eg2_decay = intValues[i];
            break;

          case 40:
            intValues[i] = values[i].toInt();
            eg2_sustain = intValues[i];
            break;

          case 41:
            intValues[i] = values[i].toInt();
            eg2_release = intValues[i];
            break;

          case 42:
            intValues[i] = values[i].toInt();
            eg2_key_follow = intValues[i];
            break;

          case 43:
            intValues[i] = values[i].toInt();
            switch (intValues[i]) {
              case 1:
                eg_select = 0;
                break;
              case 2:
                eg_select = 1;
                break;
              case 3:
                eg_select = 2;
                break;
            }
            break;

          case 44:
            intValues[i] = values[i].toInt();
            vca_key_velocity = intValues[i];
            break;

          case 45:
            intValues[i] = values[i].toInt();
            vca_level = intValues[i];
            break;
        }
      }
      // Add a newline to separate rows (optional)
      sprintf(buffer, "%d", row + 1);
      savePatch(buffer, getCurrentPatchData());
      updatePatchname();
    }
    loadPatches();
    loadFactory = false;
    storeLoadFactory(loadFactory);
    settings::decrement_setting_value();
    settings::save_current_value();
    showSettingsPage();
    delay(100);
    state = PARAMETER;
    recallPatch(1);
    refreshScreen();
    //MIDI.sendProgramChange(0, midiOutCh);
  }
}

void checkLoadRAM() {
  if (loadRAM) {
    for (int row = 32; row < 96; row++) {
      if (ROMType) {
        currentRow = IRAM[row - 32];
      } else {
        currentRow = KLRAM[row - 32];
      }

      String values[47];   // Assuming you have 47 values per row
      int valueIndex = 0;  // Index for storing values
      for (unsigned int i = 0; i < currentRow.length(); i++) {
        char currentChar = currentRow.charAt(i);

        // Check for the delimiter (",") and move to the next value
        if (currentChar == ',') {
          valueIndex++;  // Move to the next value
          continue;      // Skip the delimiter
        }

        // Append the character to the current value
        values[valueIndex] += currentChar;
      }

      // Process the values
      int intValues[47];
      for (int i = 0; i < 46; i++) {  // Adjust the loop count based on the number of values per row
        switch (i) {

          case 0:
            patchName = values[i];
            break;

          case 2:
            intValues[i] = values[i].toInt();
            osc1_octave = (intValues[i]);
            break;

          case 3:
            intValues[i] = values[i].toInt();
            osc1_wave = (intValues[i]);
            break;

          case 4:  // osc1_pw
            intValues[i] = values[i].toInt();
            osc1_pw = intValues[i];
            break;

          case 5:  // osc1_pwm
            intValues[i] = values[i].toInt();
            osc1_pwm = intValues[i];
            break;

          case 6:
            intValues[i] = values[i].toInt();
            osc1_sub = (intValues[i]);
            break;

          case 7:  // osc1_level
            intValues[i] = values[i].toInt();
            osc1_level = intValues[i];
            break;

          case 8:  // osc2_freq
            intValues[i] = values[i].toInt();
            osc2_freq = intValues[i];
            break;

          case 9:  // osc2_detune
            intValues[i] = values[i].toInt();
            osc2_detune = intValues[i];
            break;

          case 10:  // osc2_wave
            intValues[i] = values[i].toInt();
            osc2_wave = intValues[i];
            break;

          case 11:  // osc2_xmod
            intValues[i] = values[i].toInt();
            osc2_xmod = intValues[i];
            break;

          case 12:  // osc2_eg_depth
            intValues[i] = values[i].toInt();
            osc2_eg_depth = intValues[i];
            break;

          case 13:  // osc2_eg_select
            intValues[i] = values[i].toInt();
            osc2_eg_select = intValues[i];
            break;

          case 14:  // osc2_level
            intValues[i] = values[i].toInt();
            osc2_level = (intValues[i] - 1);
            break;

          case 15:  // vcf_cutoff
            intValues[i] = values[i].toInt();
            vcf_cutoff = intValues[i];
            break;

          case 16:  // vcf_res
            intValues[i] = values[i].toInt();
            vcf_res = intValues[i];
            break;

          case 17:  // vcf_eg_depth
            intValues[i] = values[i].toInt();
            vcf_eg_depth = intValues[i];
            break;

          case 18:  // vcf_key_follow
            intValues[i] = values[i].toInt();
            vcf_key_follow = intValues[i];
            break;

          case 19:  // vcf_key_velocity
            intValues[i] = values[i].toInt();
            vcf_key_velocity = intValues[i];
            break;

          case 20:  // vcf_hpf
            intValues[i] = values[i].toInt();
            vcf_hpf = intValues[i];
            break;

          case 21:  // lfo1_depth
            intValues[i] = values[i].toInt();
            lfo1_depth = intValues[i];
            break;

          case 22:  // lfo1_speed
            intValues[i] = values[i].toInt();
            lfo1_speed = intValues[i];
            break;

          case 23:  // lfo1_delay
            intValues[i] = values[i].toInt();
            lfo1_delay = intValues[i];
            break;

          case 24:  // lfo1_wave
            intValues[i] = values[i].toInt();
            lfo1_wave = intValues[i];
            break;

          case 25:
            intValues[i] = values[i].toInt();
            lfo_select = intValues[i];
            break;

          case 26:
            intValues[i] = values[i].toInt();
            eg1_attack = intValues[i];
            break;

          case 27:
            intValues[i] = values[i].toInt();
            eg1_decay = intValues[i];
            break;

          case 28:
            intValues[i] = values[i].toInt();
            eg1_sustain = intValues[i];
            break;

          case 29:
            intValues[i] = values[i].toInt();
            eg1_release = intValues[i];
            break;

          case 30:
            intValues[i] = values[i].toInt();
            eg1_key_follow = intValues[i];
            break;

          case 31:
            intValues[i] = values[i].toInt();
            eg_select = intValues[i];
            break;

          case 32:
            intValues[i] = values[i].toInt();
            vca_key_velocity = intValues[i];
            break;

          case 33:
            intValues[i] = values[i].toInt();
            vca_level = intValues[i];
            break;

          case 34:
            intValues[i] = values[i].toInt();
            lfo2_depth = intValues[i];
            break;

          case 35:
            intValues[i] = values[i].toInt();
            lfo2_speed = intValues[i];
            break;

          case 36:
            intValues[i] = values[i].toInt();
            lfo2_delay = intValues[i];
            break;

          case 37:
            intValues[i] = values[i].toInt();
            lfo2_wave = intValues[i];
            break;

          case 38:
            intValues[i] = values[i].toInt();
            lfo3_depth = intValues[i];
            break;

          case 39:
            intValues[i] = values[i].toInt();
            lfo3_speed = intValues[i];
            break;

          case 40:
            intValues[i] = values[i].toInt();
            lfo3_delay = intValues[i];
            break;

          case 41:
            intValues[i] = values[i].toInt();
            lfo3_wave = intValues[i];
            break;

          case 42:
            intValues[i] = values[i].toInt();
            eg2_attack = intValues[i];
            break;

          case 43:
            intValues[i] = values[i].toInt();
            eg2_decay = intValues[i];
            break;

          case 44:
            intValues[i] = values[i].toInt();
            eg2_sustain = intValues[i];
            break;

          case 45:
            intValues[i] = values[i].toInt();
            eg2_release = intValues[i];
            break;

          case 46:
            intValues[i] = values[i].toInt();
            eg2_key_follow = intValues[i];
            break;
        }
      }
      // Add a newline to separate rows (optional)
      sprintf(buffer, "%d", row + 1);
      savePatch(buffer, getCurrentPatchData());
      updatePatchname();
    }
    loadPatches();
    loadRAM = false;
    storeLoadRAM(loadRAM);
    settings::decrement_setting_value();
    settings::save_current_value();
    showSettingsPage();
    delay(100);
    state = PARAMETER;
    recallPatch(1);
    refreshScreen();
  }
}

void RotaryEncoderChanged(bool clockwise, int id) {

  if (!accelerate) {
    speed = 1;
  } else {
    speed = getEncoderSpeed(id);
  }

  if (!clockwise) {
    speed = -speed;
  }

  switch (id) {
    case 1:
      osc1_pw = (osc1_pw + speed);
      osc1_pw = constrain(osc1_pw, 0, 99);
      updateosc1_PW();
      break;

    case 2:
      osc1_pwm = (osc1_pwm + speed);
      osc1_pwm = constrain(osc1_pwm, 0, 99);
      updateosc1_PWM();
      break;

    case 3:
      osc1_level = (osc1_level + speed);
      osc1_level = constrain(osc1_level, 0, 99);
      updateosc1_level();
      break;

    case 4:
      osc2_detune = (osc2_detune + speed);
      osc2_detune = constrain(osc2_detune, 0, 99);
      updateosc2_detune();
      break;

    case 5:
      osc2_freq = (osc2_freq + speed);
      osc2_freq = constrain(osc2_freq, 0, 36);
      updateosc2_freq();
      break;

    case 6:
      osc2_eg_depth = (osc2_eg_depth + speed);
      osc2_eg_depth = constrain(osc2_eg_depth, 0, 99);
      updateosc2_eg_depth();
      break;

    case 7:
      osc2_level = (osc2_level + speed);
      osc2_level = constrain(osc2_level, 0, 99);
      updateosc2_level();
      break;

    case 8:
      vcf_cutoff = (vcf_cutoff + speed);
      vcf_cutoff = constrain(vcf_cutoff, 0, 99);
      updatevcf_cutoff();
      break;

    case 9:
      vcf_res = (vcf_res + speed);
      vcf_res = constrain(vcf_res, 0, 99);
      updatevcf_res();
      break;

    case 10:
      vcf_eg_depth = (vcf_eg_depth + speed);
      vcf_eg_depth = constrain(vcf_eg_depth, 0, 99);
      updatevcf_eg_depth();
      break;

    case 11:
      vcf_key_follow = (vcf_key_follow + speed);
      vcf_key_follow = constrain(vcf_key_follow, 0, 99);
      updatevcf_key_follow();
      break;

    case 12:
      vcf_hpf = (vcf_hpf + speed);
      vcf_hpf = constrain(vcf_hpf, 0, 99);
      updatevcf_hpf();
      break;

    case 13:
      lfo1_depth = (lfo1_depth + speed);
      lfo1_depth = constrain(lfo1_depth, 0, 99);
      updatelfo1_depth();
      break;

    case 14:
      lfo2_depth = (lfo2_depth + speed);
      lfo2_depth = constrain(lfo2_depth, 0, 99);
      updatelfo2_depth();
      break;

    case 15:
      eg1_attack = (eg1_attack + speed);
      eg1_attack = constrain(eg1_attack, 0, 99);
      updateeg1_attack();
      break;

    case 16:
      eg1_decay = (eg1_decay + speed);
      eg1_decay = constrain(eg1_decay, 0, 99);
      updateeg1_decay();
      break;

    case 17:
      eg2_attack = (eg2_attack + speed);
      eg2_attack = constrain(eg2_attack, 0, 99);
      updateeg2_attack();
      break;

    case 18:
      eg2_decay = (eg2_decay + speed);
      eg2_decay = constrain(eg2_decay, 0, 99);
      updateeg2_decay();
      break;

    case 19:
      vcf_key_velocity = (vcf_key_velocity + speed);
      vcf_key_velocity = constrain(vcf_key_velocity, 0, 99);
      updatevcf_key_velocity();
      break;

    case 20:
      lfo3_depth = (lfo3_depth + speed);
      lfo3_depth = constrain(lfo3_depth, 0, 99);
      updatelfo3_depth();
      break;

    case 21:
      eg1_key_follow = (eg1_key_follow + speed);
      eg1_key_follow = constrain(eg1_key_follow, 0, 99);
      updateeg1_key_follow();
      break;

    case 22:
      eg2_key_follow = (eg2_key_follow + speed);
      eg2_key_follow = constrain(eg2_key_follow, 0, 99);
      updateeg2_key_follow();
      break;

    case 23:
      vca_key_velocity = (vca_key_velocity + speed);
      vca_key_velocity = constrain(vca_key_velocity, 0, 99);
      updatevca_key_velocity();
      break;

    case 24:
      vca_level = (vca_level + speed);
      vca_level = constrain(vca_level, 0, 99);
      updatevca_level();
      break;

    case 25:
      eg1_sustain = (eg1_sustain + speed);
      eg1_sustain = constrain(eg1_sustain, 0, 99);
      updateeg1_sustain();
      break;

    case 26:
      eg1_release = (eg1_release + speed);
      eg1_release = constrain(eg1_release, 0, 99);
      updateeg1_release();
      break;

    case 27:
      eg2_sustain = (eg2_sustain + speed);
      eg2_sustain = constrain(eg2_sustain, 0, 99);
      updateeg2_sustain();
      break;

    case 28:
      eg2_release = (eg2_release + speed);
      eg2_release = constrain(eg2_release, 0, 99);
      updateeg2_release();
      break;

    case 29:
      lfo1_speed = (lfo1_speed + speed);
      lfo1_speed = constrain(lfo1_speed, 0, 99);
      updatelfo1_speed();
      break;

    case 30:
      lfo2_speed = (lfo2_speed + speed);
      lfo2_speed = constrain(lfo2_speed, 0, 99);
      updatelfo2_speed();
      break;

    case 31:
      lfo3_speed = (lfo3_speed + speed);
      lfo3_speed = constrain(lfo3_speed, 0, 99);
      updatelfo3_speed();
      break;

    case 32:
      lfo1_delay = (lfo1_delay + speed);
      lfo1_delay = constrain(lfo1_delay, 0, 99);
      updatelfo1_delay();
      break;

    case 33:
      lfo2_delay = (lfo2_delay + speed);
      lfo2_delay = constrain(lfo2_delay, 0, 99);
      updatelfo2_delay();
      break;

    case 34:
      lfo3_delay = (lfo3_delay + speed);
      lfo3_delay = constrain(lfo3_delay, 0, 99);
      updatelfo3_delay();
      break;
  }


  //rotaryEncoderChanged(id, clockwise, speed);
}

int getEncoderSpeed(int id) {
  if (id < 1 || id > numEncoders) return 1;

  unsigned long now = millis();
  unsigned long revolutionTime = now - lastTransition[id];

  int speed = 1;
  if (revolutionTime < 50) {
    speed = 10;
  } else if (revolutionTime < 125) {
    speed = 5;
  } else if (revolutionTime < 250) {
    speed = 2;
  }

  // if (!accelerate) {
  //   speed = 1;
  // }

  lastTransition[id] = now;
  return speed;
}

void mainButtonChanged(Button *btn, bool released) {

  switch (btn->id) {
    case OSC1_OCT_BUTTON:
      if (!released) {
        osc1_octave = osc1_octave + 1;
        if (osc1_octave > 2) {
          osc1_octave = 0;
        }
        myControlChange(midiChannel, CCosc1_octave, osc1_octave);
      }
      break;

    case OSC1_WAVE_BUTTON:
      if (!released) {
        osc1_wave = osc1_wave + 1;
        if (osc1_wave > 3) {
          osc1_wave = 0;
        }
        myControlChange(midiChannel, CCosc1_wave, osc1_wave);
      }
      break;

    case OSC1_SUB_BUTTON:
      if (!released) {
        osc1_sub = !osc1_sub;
        myControlChange(midiChannel, CCosc1_sub, osc1_sub);
      }
      break;

    case OSC2_WAVE_BUTTON:
      if (!released) {
        osc2_wave = osc2_wave + 1;
        if (osc2_wave > 3) {
          osc2_wave = 0;
        }
        myControlChange(midiChannel, CCosc2_wave, osc2_wave);
      }
      break;

    case OSC2_XMOD_BUTTON:
      if (!released) {
        osc2_xmod = osc2_xmod + 1;
        if (osc2_xmod > 2) {
          osc2_xmod = 0;
        }
        myControlChange(midiChannel, CCosc2_xmod, osc2_xmod);
      }
      break;

    case OSC2_EG_BUTTON:
      if (!released) {
        osc2_eg_select = osc2_eg_select + 1;
        if (osc2_eg_select > 1) {
          osc2_eg_select = 0;
        }
        myControlChange(midiChannel, CCosc2_eg_select, osc2_eg_select);
      }
      break;

    case LFO1_WAVE_BUTTON:
      if (!released) {
        lfo1_wave = lfo1_wave + 1;
        if (lfo1_wave > 3) {
          lfo1_wave = 0;
        }
        myControlChange(midiChannel, CClfo1_wave, lfo1_wave);
      }
      break;

    case LFO2_WAVE_BUTTON:
      if (!released) {
        lfo2_wave = lfo2_wave + 1;
        if (lfo2_wave > 3) {
          lfo2_wave = 0;
        }
        myControlChange(midiChannel, CClfo2_wave, lfo2_wave);
      }
      break;

    case LFO3_WAVE_BUTTON:
      if (!released) {
        lfo3_wave = lfo3_wave + 1;
        if (lfo3_wave > 3) {
          lfo3_wave = 0;
        }
        myControlChange(midiChannel, CClfo3_wave, lfo3_wave);
      }
      break;

    case ENV_SEL_BUTTON:
      if (!released) {
        eg_select = eg_select + 1;
        if (eg_select > 2) {
          eg_select = 0;
        }
        myControlChange(midiChannel, CCeg_select, eg_select);
      }
      break;

    case LFO_SEL_BUTTON:
      if (!released) {
        lfo_select = lfo_select + 1;
        if (lfo_select > 2) {
          lfo_select = 0;
        }
        myControlChange(midiChannel, CClfo_select, lfo_select);
      }
      break;

    case OSC1_LEV_SW:
      if (!released) {
        if (!level1WasToggled) {
          // If it's already 0 and wasn't toggled, do nothing
          if (osc1_level == 0) break;

          // Save the current value and set to default
          lastlevel1Value = osc1_level;
          osc1_level = 0;
          level1WasToggled = true;
        } else {
          // Toggle back to previous value
          osc1_level = lastlevel1Value;
          level1WasToggled = false;
        }

        myControlChange(midiChannel, CCosc1_level, osc1_level);
      }
      break;

    case OSC2_DET_SW:
      if (!released) {
        if (!detuneWasToggled) {
          // If it's already 50 and wasn't toggled, do nothing
          if (osc2_detune == 50) break;

          // Save the current value and set to default
          lastDetuneValue = osc2_detune;
          osc2_detune = 50;
          detuneWasToggled = true;
        } else {
          // Toggle back to previous value
          osc2_detune = lastDetuneValue;
          detuneWasToggled = false;
        }

        myControlChange(midiChannel, CCosc2_detune, osc2_detune);
      }
      break;

    case OSC2_LEV_SW:
      if (!released) {
        if (!level2WasToggled) {
          // If it's already 0 and wasn't toggled, do nothing
          if (osc2_level == 0) break;

          // Save the current value and set to default
          lastlevel2Value = osc2_level;
          osc2_level = 0;
          level2WasToggled = true;
        } else {
          // Toggle back to previous value
          osc2_level = lastlevel2Value;
          level2WasToggled = false;
        }

        myControlChange(midiChannel, CCosc2_level, osc2_level);
      }
      break;

    case OSC2_EG_SW:
      if (!released) {
        if (!osc2EGWasToggled) {
          // If it's already 50 and wasn't toggled, do nothing
          if (osc2_eg_depth == 50) break;

          // Save the current value and set to default
          lastosc2EGValue = osc2_eg_depth;
          osc2_eg_depth = 50;
          osc2EGWasToggled = true;
        } else {
          // Toggle back to previous value
          osc2_eg_depth = lastosc2EGValue;
          osc2EGWasToggled = false;
        }

        myControlChange(midiChannel, CCosc2_eg_depth, osc2_eg_depth);
      }
      break;

    case VCF_EG_SW:
      if (!released) {
        if (!vcfEGWasToggled) {
          // If it's already 50 and wasn't toggled, do nothing
          if (vcf_eg_depth == 50) break;

          // Save the current value and set to default
          lastvcfEGValue = vcf_eg_depth;
          vcf_eg_depth = 50;
          vcfEGWasToggled = true;
        } else {
          // Toggle back to previous value
          vcf_eg_depth = lastvcfEGValue;
          vcfEGWasToggled = false;
        }

        myControlChange(midiChannel, CCvcf_eg_depth, vcf_eg_depth);
      }
      break;

    case VCF_KEYF_SW:
      if (!released) {
        if (!vcfKeyFWasToggled) {
          // If it's already 0 and wasn't toggled, do nothing
          if (vcf_key_follow == 0) break;

          // Save the current value and set to default
          lastvcfKeyFValue = vcf_key_follow;
          vcf_key_follow = 0;
          vcfKeyFWasToggled = true;
        } else {
          // Toggle back to previous value
          vcf_key_follow = lastvcfKeyFValue;
          vcfKeyFWasToggled = false;
        }

        myControlChange(midiChannel, CCvcf_key_follow, vcf_key_follow);
      }
      break;

    case VCF_VEL_SW:
      if (!released) {
        if (!vcfVelWasToggled) {
          // If it's already 0 and wasn't toggled, do nothing
          if (vcf_key_velocity == 0) break;

          // Save the current value and set to default
          lastvcfVelValue = vcf_key_velocity;
          vcf_key_velocity = 0;
          vcfVelWasToggled = true;
        } else {
          // Toggle back to previous value
          vcf_key_velocity = lastvcfVelValue;
          vcfVelWasToggled = false;
        }

        myControlChange(midiChannel, CCvcf_key_velocity, vcf_key_velocity);
      }
      break;

    case VCA_VEL_SW:
      if (!released) {
        if (!vcaVelWasToggled) {
          // If it's already 0 and wasn't toggled, do nothing
          if (vca_key_velocity == 0) break;

          // Save the current value and set to default
          lastvcaVelValue = vca_key_velocity;
          vca_key_velocity = 0;
          vcaVelWasToggled = true;
        } else {
          // Toggle back to previous value
          vca_key_velocity = lastvcfVelValue;
          vcaVelWasToggled = false;
        }

        myControlChange(midiChannel, CCvca_key_velocity, vca_key_velocity);
      }
      break;
  }
}

// build the groups in setup()
void groupEncoders() {
  for (auto &enc : rotaryEncoders) {
    for (size_t i = 0; i < NUM_MCP; ++i) {
      if (enc.getMCP() == allMCPs[i]) {
        encByMCP[i].push_back(&enc);
        break;
      }
    }
  }
}

void pollAllMCPs() {

  for (int j = 0; j < numMCPs; j++) {
    uint16_t gpioAB = allMCPs[j]->readGPIOAB();
    for (int i = 0; i < numEncoders; i++) {
      if (rotaryEncoders[i].getMCP() == allMCPs[j])
        rotaryEncoders[i].feedInput(gpioAB);
    }

    for (auto &button : allButtons) {
      if (button->getMcp() == allMCPs[j]) {
        button->feedInput(gpioAB);
      }
    }
  }
}

void loop() {

  if (!recallPatchFlag) {
    MIDI.read(midiChannel);
    usbMIDI.read(midiChannel);
    MIDI5.read(midiChannel);
  }

  if (!receivingSysEx) {

    pollAllMCPs();
    checkSwitches();
    checkEncoder();
    checkLoadFactory();
    checkLoadRAM();
    SaveCurrent();
    SaveAll();
  }

  if (waitingToUpdate && (millis() - lastDisplayTriggerTime >= displayTimeout)) {
    refreshScreen();  // retrigger
    waitingToUpdate = false;
  }

  // Print data if the entire SysEx message is complete
  if (sysexComplete) {
    printSysexData();
    sysexComplete = false;  // Reset for the next SysEx message
    currentBlock = 0;       // Reset to start filling from block 0 again
    byteIndex = 0;          // Reset byte index within the block
  }
}
