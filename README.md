A second build of this midi controller for the AX-80, all 45 controls are presented with buttons and LEDs for the switch functions. Encoders for the 0-99 controls. Encoders give a more accurate edit of the current parameter and can be accelerated to transition quickly of slowly across the range.

![Synth](photos/synth.jpg)

It will have 999 patch memories and names and can be used as a master for the AX-80 which means you don't store any sounds on the AX and the editor is in charge. Or normal mode where it just edits parameters.

This version uses MCP23017 chips to read the encoders and buttons and also to drive the LED's. This reduces chip count, analogue mux jitter etc.

It also acts as a usb to MIDI converter so you can play the AX over USB from your DAW.

Several encoder buttons are used as default quick buttons, a press of the OSC1 level encoder sets the volume to 0 and press again and it returns to the previous value.

Aftertouch is now translated to the Modulation Wheel, this can be turned off or on the settings menu.

I've also mapped CC 7 for volume to CC 96 (VCA level) to act as a volume control.
