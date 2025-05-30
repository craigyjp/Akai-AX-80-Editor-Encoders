//Values below are just for initialising and will be changed when synth is initialised to current panel controls & EEPROM settings
byte midiChannel = MIDI_CHANNEL_OMNI;  //(EEPROM)
byte midiOutCh = 1;                    //(EEPROM)

// adding encoders
bool rotaryEncoderChanged(int id, bool clockwise, int speed);
#define NUM_ENCODERS 34
unsigned long lastTransition[NUM_ENCODERS + 1];
unsigned long lastDisplayTriggerTime = 0;
bool waitingToUpdate = false;
const unsigned long displayTimeout = 5000;  // e.g. 5 seconds

char buffer[10];

const int totalBytes = 3200;
byte ramArray[64][50]; // Array to store SysEx data
byte sysexBuff(50);
byte data(50);
byte sysexData[50];
bool sysexComplete = false;
bool receivingSysEx = false; // Flag to indicate if a SysEx message is in progress
int currentBlock = 0;
int byteIndex = 0;

int MIDIThru = midi::Thru::Off;  //(EEPROM)
String patchName = INITPATCHNAME;
String currentRow;
String bankdir = "/Bank";
boolean encCW = true;  //This is to set the encoder to increment when turned CW - Settings Option
boolean recallPatchFlag = true;
boolean loadFactory = false;
boolean loadRAM = false;
boolean loadFromDW = false;
boolean ROMType = false;
boolean dataInProgress = false;
int currentSendPatch = 0;
boolean saveCurrent = false;
boolean saveAll = false;
boolean updateParams = false;  //(EEPROM)
int bankselect = 0;
int old_value = 0;
int old_param_offset = 0;
int received_patch = 0;


int osc1_octave = 0;
int osc1_wave = 0;
int osc1_pw = 0;
int osc1_pwm = 0;
int osc1_sub = 0;
int osc1_level = 0;

int osc2_freq = 0;
int osc2_detune = 0;
int osc2_wave = 0;
int osc2_xmod = 0;
int osc2_eg_depth = 0;
int osc2_eg_select = 0;
int osc2_level = 0;

int vcf_cutoff = 0;
int vcf_res = 0;
int vcf_eg_depth = 0;
int vcf_key_follow = 0;
int vcf_key_velocity = 0;
int vcf_hpf = 0;

int lfo1_depth = 0;
int lfo1_speed = 0;
int lfo1_delay = 0;
int lfo1_wave = 0;
int lfo_select = 0;

int eg1_attack = 0;
int eg1_decay = 0;
int eg1_sustain = 0;
int eg1_release = 0;
int eg1_key_follow = 0;
int eg_select = 0;

int vca_key_velocity = 0;
int vca_level = 0;

int lfo2_depth = 0;
int lfo2_speed = 0;
int lfo2_delay = 0;
int lfo2_wave = 0;

int lfo3_depth = 0;
int lfo3_speed = 0;
int lfo3_delay = 0;
int lfo3_wave = 0;

int eg2_attack = 0;
int eg2_decay = 0;
int eg2_sustain = 0;
int eg2_release = 0;
int eg2_key_follow = 0;

int lfo_select_temp = -1;
int eg_select_temp = -1;

int returnvalue = 0;
