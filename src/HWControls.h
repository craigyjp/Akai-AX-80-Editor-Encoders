// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Bounce.h>
#include "TButton.h"

#include "Rotary.h"
#include "RotaryEncOverMCP.h"

#define OSC1_OCT_BUTTON 0
#define OSC1_WAVE_BUTTON 1
#define OSC1_SUB_BUTTON 2
#define OSC2_WAVE_BUTTON 3
#define OSC2_XMOD_BUTTON 4
#define OSC2_EG_BUTTON 5
#define LFO1_WAVE_BUTTON 6
#define LFO2_WAVE_BUTTON 7
#define LFO3_WAVE_BUTTON 8
#define ENV_SEL_BUTTON 9
#define LFO_SEL_BUTTON 10

// Pins for MCP23017
#define GPA0 0
#define GPA1 1
#define GPA2 2
#define GPA3 3
#define GPA4 4
#define GPA5 5
#define GPA6 6
#define GPA7 7
#define GPB0 8
#define GPB1 9
#define GPB2 10
#define GPB3 11
#define GPB4 12
#define GPB5 13
#define GPB6 14
#define GPB7 15

void RotaryEncoderChanged (bool clockwise, int id);

void mainButtonChanged(Button *btn, bool released);

// I2C MCP23017 GPIO expanders

Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;
Adafruit_MCP23017 mcp4;
Adafruit_MCP23017 mcp5;
Adafruit_MCP23017 mcp6;
Adafruit_MCP23017 mcp7;

//Array of pointers of all MCPs
Adafruit_MCP23017 *allMCPs[] = {&mcp1, &mcp2, &mcp3, &mcp4, &mcp5, &mcp6, &mcp7};

/* Array of all rotary encoders and their pins */
RotaryEncOverMCP rotaryEncoders[] = {
        RotaryEncOverMCP(&mcp1, 1, 0, &RotaryEncoderChanged, 1),
        RotaryEncOverMCP(&mcp1, 3, 2, &RotaryEncoderChanged, 2),
        RotaryEncOverMCP(&mcp1, 5, 4, &RotaryEncoderChanged, 3),
        RotaryEncOverMCP(&mcp1, 12, 11, &RotaryEncoderChanged, 4),
        RotaryEncOverMCP(&mcp2, 1, 0, &RotaryEncoderChanged, 5),
        RotaryEncOverMCP(&mcp2, 3, 2, &RotaryEncoderChanged, 6),
        RotaryEncOverMCP(&mcp2, 5, 4, &RotaryEncoderChanged, 7),
        RotaryEncOverMCP(&mcp2, 12, 11, &RotaryEncoderChanged, 8),
        RotaryEncOverMCP(&mcp3, 1, 0, &RotaryEncoderChanged, 9),
        RotaryEncOverMCP(&mcp3, 3, 2, &RotaryEncoderChanged, 10),
        RotaryEncOverMCP(&mcp3, 5, 4, &RotaryEncoderChanged, 11),
        RotaryEncOverMCP(&mcp3, 9, 8, &RotaryEncoderChanged, 12),
        RotaryEncOverMCP(&mcp3, 11, 10, &RotaryEncoderChanged, 13),
        RotaryEncOverMCP(&mcp3, 13, 12, &RotaryEncoderChanged, 14),
        RotaryEncOverMCP(&mcp4, 1, 0, &RotaryEncoderChanged, 15),
        RotaryEncOverMCP(&mcp4, 3, 2, &RotaryEncoderChanged, 16),
        RotaryEncOverMCP(&mcp4, 9, 8, &RotaryEncoderChanged, 17),
        RotaryEncOverMCP(&mcp4, 11, 10, &RotaryEncoderChanged, 18),
        RotaryEncOverMCP(&mcp5, 1, 0, &RotaryEncoderChanged, 19),
        RotaryEncOverMCP(&mcp5, 3, 2, &RotaryEncoderChanged, 20),
        RotaryEncOverMCP(&mcp5, 9, 8, &RotaryEncoderChanged, 21),
        RotaryEncOverMCP(&mcp5, 11, 10, &RotaryEncoderChanged, 22),
        RotaryEncOverMCP(&mcp6, 1, 0, &RotaryEncoderChanged, 23),
        RotaryEncOverMCP(&mcp6, 3, 2, &RotaryEncoderChanged, 24),
        RotaryEncOverMCP(&mcp6, 5, 4, &RotaryEncoderChanged, 25),
        RotaryEncOverMCP(&mcp6, 9, 8, &RotaryEncoderChanged, 26),
        RotaryEncOverMCP(&mcp6, 11, 10, &RotaryEncoderChanged, 27),
        RotaryEncOverMCP(&mcp6, 13, 12, &RotaryEncoderChanged, 28),
        RotaryEncOverMCP(&mcp7, 1, 0, &RotaryEncoderChanged, 29),
        RotaryEncOverMCP(&mcp7, 3, 2, &RotaryEncoderChanged, 30),
        RotaryEncOverMCP(&mcp7, 5, 4, &RotaryEncoderChanged, 31),
        RotaryEncOverMCP(&mcp7, 9, 8, &RotaryEncoderChanged, 32),
        RotaryEncOverMCP(&mcp7, 11, 10, &RotaryEncoderChanged, 33),
        RotaryEncOverMCP(&mcp7, 13, 12, &RotaryEncoderChanged, 34),
};

// after your rotaryEncoders[] definition
constexpr size_t NUM_MCP = sizeof(allMCPs) / sizeof(allMCPs[0]);
constexpr int numMCPs = (int)(sizeof(allMCPs) / sizeof(*allMCPs));
constexpr int numEncoders = (int)(sizeof(rotaryEncoders) / sizeof(*rotaryEncoders));

// an array of vectors to hold pointers to the encoders on each MCP
std::vector<RotaryEncOverMCP*> encByMCP[NUM_MCP];

Button osc1_oct_Button = Button(&mcp1, 8, OSC1_OCT_BUTTON, &mainButtonChanged);
Button osc1_wave_Button = Button(&mcp1, 9, OSC1_WAVE_BUTTON, &mainButtonChanged);
Button osc1_sub_Button = Button(&mcp1, 10, OSC1_SUB_BUTTON, &mainButtonChanged);
Button osc2_wave_Button = Button(&mcp2, 8, OSC2_WAVE_BUTTON, &mainButtonChanged);
Button osc2_xmod_Button = Button(&mcp2, 9, OSC2_XMOD_BUTTON, &mainButtonChanged);
Button osc2_eg_Button = Button(&mcp2, 10, OSC2_EG_BUTTON, &mainButtonChanged);
Button lfo1_wave_Button = Button(&mcp4, 4, LFO1_WAVE_BUTTON, &mainButtonChanged);
Button lfo2_wave_Button = Button(&mcp4, 5, LFO2_WAVE_BUTTON, &mainButtonChanged);
Button lfo3_wave_Button = Button(&mcp4, 12, LFO3_WAVE_BUTTON, &mainButtonChanged);
Button env_sel_Button = Button(&mcp5, 13, ENV_SEL_BUTTON, &mainButtonChanged);
Button lfo_sel_Button = Button(&mcp5, 12, LFO_SEL_BUTTON, &mainButtonChanged);

Button *mainButtons[] = {
        &osc1_oct_Button, &osc1_wave_Button, &osc1_sub_Button, &osc2_wave_Button, &osc2_xmod_Button, &osc2_eg_Button, &lfo1_wave_Button, &lfo2_wave_Button, &lfo3_wave_Button, &env_sel_Button, &lfo_sel_Button,
};

Button *allButtons[] = {
        &osc1_oct_Button, &osc1_wave_Button, &osc1_sub_Button, &osc2_wave_Button, &osc2_xmod_Button, &osc2_eg_Button,
        &lfo1_wave_Button, &lfo2_wave_Button, &lfo3_wave_Button, &env_sel_Button, &lfo_sel_Button
};

// GP1
#define osc1_PW_A 0
#define osc1_PW_B 1
#define osc1_PWM_A 2
#define osc1_PWM_B 3
#define osc1_level_A 4
#define osc1_level_B 5
#define OSC1_OCTAVE_LED_RED 6
#define OSC1_OCTAVE_LED_GREEN 7
#define OSC1_OCTAVE 8
#define OSC1_WAVE 9
#define OSC1_SUB 10
#define osc2_detune_A 11
#define osc2_detune_B 12
#define OSC1_SUB_LED 13
#define OSC1_WAVE_LED_RED 14
#define OSC1_WAVE_LED_GREEN 15

// GP2
#define osc2_freq_A 0
#define osc2_freq_B 1
#define osc2_eg_depth_A 2
#define osc2_eg_depth_B 3
#define osc2_level_A 4
#define osc2_level_B 5
#define OSC2_WAVE_LED_RED 6
#define OSC2_WAVE_LED_GREEN 7
#define OSC2_WAVE 8
#define OSC2_XMOD 9
#define OSC2_EG_SELECT 10
#define vcf_cutoff_A 11
#define vcf_cutoff_B 12
#define OSC2_XMOD_LED_RED 13
#define OSC2_XMOD_LED_GREEN 14
#define OSC2_EG_SELECT_LED_RED 15

// GP3
#define vcf_res_A 0
#define vcf_res_B 1
#define vcf_eg_depth_A 2
#define vcf_eg_depth_B 3
#define vcf_key_follow_A 4
#define vcf_key_follow_B 5
// 6 unused
// 7 unused
#define vcf_hpf_A 8
#define vcf_hpf_B 9
#define lfo1_depth_A 10
#define lfo1_depth_B 11
#define lfo2_depth_A 12
#define lfo2_depth_B 13
// 14 unused
#define OSC2_EG_SELECT_LED_GREEN 15

// GP4
#define eg1_attack_A 0
#define eg1_attack_B 1
#define eg1_decay_A 2
#define eg1_decay_B 3
#define LFO1_WAVE 4
#define LFO2_WAVE 5
#define LFO3_WAVE_LED_RED 6
#define LFO3_WAVE_LED_GREEN 7
#define eg2_attack_A 8
#define eg2_attack_B 9
#define eg2_decay_A 10
#define eg2_decay_B 11
#define LFO3_WAVE 12
// 13 unused
#define LFO_SELECT_LED_RED 14
#define LFO_SELECT_LED_GREEN 15

// GP5
#define vcf_key_velocity_A 0
#define vcf_key_velocity_B 1
#define lfo3_depth_A 2
#define lfo3_depth_B 3
// 4 unused
// 5 unused
// 6 unused
// 7 unused
#define eg1_key_follow_A 8
#define eg1_key_follow_B 9
#define eg2_key_follow_A 10
#define eg2_key_follow_B 11
#define EG_SELECT 12
#define LFO_SELECT 13
#define EG_DEST_LED_RED 14
#define EG_DEST_LED_GREEN 15

// GP6
#define vca_key_velocity_A 0
#define vca_key_velocity_B 1
#define vca_level_A 2
#define vca_level_B 3
#define eg1_sus_A 4
#define eg1_sus_B 5
// 6 unused
// 7 unused
#define eg1_rel_A 8
#define eg1_rel_B 9
#define eg2_sus_A 10
#define eg2_sus_B 11
#define eg2_rel_A 12
#define eg2_rel_B 13
// 14 unused
// 15 unused

// GP7
#define lfo1_speed_A 0
#define lfo1_speed_B 1
#define lfo2_speed_A 2
#define lfo2_speed_B 3
#define lfo3_speed_A 4
#define lfo3_speed_B 5
#define LFO1_WAVE_LED_RED 6
#define LFO1_WAVE_LED_GREEN 7
#define lfo1_delay_A 8
#define lfo1_delay_B 9
#define lfo2_delay_A 10
#define lfo2_delay_B 11
#define lfo3_delay_A 12
#define lfo3_delay_B 13
#define LFO2_WAVE_LED_RED 14
#define LFO2_WAVE_LED_GREEN 15

//Teensy 4.1 Pins

#define RECALL_SW 40
#define SAVE_SW 41
#define SETTINGS_SW 39
#define BACK_SW 38

#define ENCODER_PINA 4
#define ENCODER_PINB 5

#define MUXCHANNELS 16
#define QUANTISE_FACTOR 31

#define DEBOUNCE 30

static byte muxInput = 0;
static int mux1ValuesPrev[MUXCHANNELS] = {};
static int mux2ValuesPrev[MUXCHANNELS] = {};
static int mux3ValuesPrev[MUXCHANNELS] = {};
static int mux4ValuesPrev[MUXCHANNELS] = {};
static int mux5ValuesPrev[MUXCHANNELS] = {};

static long encPrevious = 0;

//These are pushbuttons and require debouncing

TButton saveButton{ SAVE_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton settingsButton{ SETTINGS_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton backButton{ BACK_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton recallButton{ RECALL_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION }; // on encoder

Encoder encoder(ENCODER_PINB, ENCODER_PINA);  //This often needs the pins swapping depending on the encoder

void setupHardware() {

  //Switches
  pinMode(RECALL_SW, INPUT_PULLUP);  //On encoder
  pinMode(SAVE_SW, INPUT_PULLUP);
  pinMode(SETTINGS_SW, INPUT_PULLUP);
  pinMode(BACK_SW, INPUT_PULLUP);

}
