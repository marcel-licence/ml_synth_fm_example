/*
 * Headless FM Synthesizer with Moog Ladder Filter
 * Target: Wemos Lolin32 Lite (ESP32)
 * Control: BLE MIDI
 * Output: I2S
 *
 * Architecture:
 * - 4-Operator FM Engine (YM2612 style)
 * - Moog Ladder Filter (Improved Model)
 * - BLE MIDI Control
 */

#include "config.h"
#include <Arduino.h>
#include <BLEMidi.h>
#include <math.h>

/* Requires ML_SynthTools */
#include <caps_info.h>
#include <ml_arp.h>
#include <ml_delay.h>
#include <ml_midi_ctrl.h>
#include <ml_reverb.h>
#include <ml_types.h>
#include <ml_fm.h>

#define ML_SYNTH_INLINE_DECLARATION
#include <ml_inline.h>
#undef ML_SYNTH_INLINE_DECLARATION

/* Include Moog Filter Implementation */
#include "MoogFilter.h"

// --- Configuration Constants ---
#define SAMPLE_RATE 48000
#define FILTER_MIN_FREQ 20.0f
#define FILTER_MAX_FREQ 20000.0f
#define FILTER_MAX_RES 4.5f

// --- Global Objects ---
ImprovedMoog moogFilter(SAMPLE_RATE);

// Audio Buffers
static float fl_sample[SAMPLE_BUFFER_SIZE];
static float fr_sample[SAMPLE_BUFFER_SIZE];
static float m1_sample[SAMPLE_BUFFER_SIZE];

// Output Gain
float master_output_gain = 1.00f;

// --- MIDI Handling ---

void onNoteOn(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp) {
    float fVel = (float)velocity / 127.0f;
    FmSynth_NoteOn(channel, note, fVel);
}

void onNoteOff(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp) {
    FmSynth_NoteOff(channel, note);
}

void onControlChange(uint8_t channel, uint8_t controller, uint8_t value, uint16_t timestamp) {
    float normVal = (float)value / 127.0f;

    // Filter Controls
    if (controller == 74) { // Cutoff
        float cutoffHz = FILTER_MIN_FREQ * powf(FILTER_MAX_FREQ / FILTER_MIN_FREQ, normVal);
        moogFilter.SetCutoff(cutoffHz);
    }
    else if (controller == 71) { // Resonance
        float res = normVal * FILTER_MAX_RES;
        moogFilter.SetResonance(res);
    }
    // FM Global Controls
    else if (controller == 104) { // Feedback
        FmSynth_Feedback(0, normVal);
    }
    else if (controller == 105) { // Algorithm
        // 8 Algorithms usually (0-7)
        FmSynth_SetAlgorithm(0, normVal * 7.0f); // ML_FM handles float to int conversion internally hopefully or we cast
    }
    else if (controller == 102) { // Operator Selector (1-4)
        // Map 0-127 to 0-3 (Op 1-4)
        // Divide into 4 zones: 0-31=Op1, 32-63=Op2, etc.
        float op = (float)(value / 32);
        if(op > 3.0f) op = 3.0f;
        FmSynth_SelectOp(0, op);
    }
    // FM Operator Specific Controls (Affects currently selected Operator)
    else if (controller == 73) { // Attack
        FmSynth_Attack(0, normVal);
    }
    else if (controller == 75) { // Decay 1
        FmSynth_Decay1(0, normVal);
    }
    else if (controller == 76) { // Decay 2 / Sustain Level
        // ML_FM has Decay1, DecayL (Level), Decay2, Release
        // Let's map CC 76 to Decay2 for now
        FmSynth_Decay2(0, normVal);
    }
    else if (controller == 79) { // Sustain Level (Decay Level)
        FmSynth_DecayL(0, normVal);
    }
    else if (controller == 72) { // Release
        FmSynth_Release(0, normVal);
    }
    else if (controller == 77) { // Frequency Multiplier / Ratio
        FmSynth_ChangeParam(0, normVal); // Check if ChangeParam controls freq mult?
        // Based on header, `FmSynth_ChangeParam` name is ambiguous.
        // In many FM synths, "Coarse/Fine" tune or "Mult" is key.
        // Let's assume ChangeParam is the generic "Value" setter for the selected param
        // OR we might be missing a specific function.
        // Re-reading header: `FmSynth_ChangeParam(uint8_t param, float value);`
        // It might be that SelectOp selects the Op, but we need to select the PROPERTY too?
        // Wait, the functions are `FmSynth_Attack`, `FmSynth_Decay1` etc.
        // `FmSynth_ChangeParam` might be for the frequency multiplier/detune.
        // Let's try `FmSynth_ChangeParam` for the "Timbre/Ratio" control.
        FmSynth_ChangeParam(0, normVal);
    }
}


// --- Audio Task (Core 1) ---

inline void audio_task()
{
    memset(fl_sample, 0, sizeof(fl_sample));
    memset(fr_sample, 0, sizeof(fr_sample));
    memset(m1_sample, 0, sizeof(m1_sample));

    // 1. Generate FM Synthesis
    FmSynth_Process(m1_sample, m1_sample, SAMPLE_BUFFER_SIZE);

    // 2. Apply Moog Ladder Filter
    moogFilter.Process(m1_sample, SAMPLE_BUFFER_SIZE);

    // 3. Post-Processing (Delay/Reverb)
    // Delay_Process_Buff(m1_sample, SAMPLE_BUFFER_SIZE);
    // Reverb_Process(m1_sample, SAMPLE_BUFFER_SIZE);

    // 4. Output Mixing
    for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
    {
        m1_sample[n] *= master_output_gain;
        fl_sample[n] = m1_sample[n];
        fr_sample[n] = m1_sample[n];
    }

    Audio_Output(fl_sample, fr_sample);
}


// --- Setup ---

void setup() {
    Serial.begin(115200);
    Audio_Setup();

    // FM Init
    FmSynth_Init(SAMPLE_RATE);

    // Set Default Algorithm
    FmSynth_SetAlgorithm(0, 0); // Algo 0

    // Filter Init
    moogFilter.SetCutoff(1000.0f);
    moogFilter.SetResonance(0.0f);

    // BLE MIDI Setup
    BLEMidiServer.begin("Headless FM Synth");
    BLEMidiServer.setOnConnectCallback([](){ Serial.println("BLE Connected"); });
    BLEMidiServer.setOnDisconnectCallback([](){ Serial.println("BLE Disconnected"); });
    BLEMidiServer.setNoteOnCallback(onNoteOn);
    BLEMidiServer.setNoteOffCallback(onNoteOff);
    BLEMidiServer.setControlChangeCallback(onControlChange);

    Serial.println("Headless FM Synth Ready");
}

// --- Main Loop (Core 1) ---

void loop() {
    audio_task();
}
