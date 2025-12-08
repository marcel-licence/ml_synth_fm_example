# Headless FM Synth (ESP32)

A fully headless, BLE-MIDI controlled FM Synthesizer running on the Wemos Lolin32 Lite (ESP32).

## Architecture

*   **Platform**: ESP32 (Wemos Lolin32 Lite)
*   **Audio Engine**: 4-Operator FM Synthesis (YM2612 style) using `ML_SynthTools`.
*   **Filter**: Moog Ladder Filter (Improved Model) for analog-style warmth and self-oscillation.
*   **Control**: strictly BLE MIDI. No physical potentiometers.
*   **Output**: I2S DAC (PCM5102 / PT8211).

## Features

*   **4-Operator FM**: Full control over Attack, Decay1, Decay2, Sustain Level, Release, and Frequency Ratio for each operator.
*   **Algorithms**: 8 Selectable FM Algorithms (CC 105).
*   **Feedback**: Adjustable Feedback intensity (CC 104).
*   **Moog Filter**:
    *   **Cutoff**: Logarithmic mapping (20Hz - 20kHz) via CC 74.
    *   **Resonance**: Linear mapping with self-oscillation range (0 - 4.5) via CC 71.
*   **Web App**: Includes a PWA (`index.html`) for control via Chrome/Edge Web Bluetooth.

## MIDI Implementation Chart

| Parameter | CC Number | Value Range | Description |
| :--- | :--- | :--- | :--- |
| **Filter Cutoff** | 74 | 0-127 | Logarithmic (20Hz - 20kHz) |
| **Filter Resonance** | 71 | 0-127 | 0.0 - 4.5 (Self-Oscillates > 4.0) |
| **Operator Select** | 102 | 0-127 | 0-31: Op1, 32-63: Op2, 64-95: Op3, 96-127: Op4 |
| **FM Algorithm** | 105 | 0-127 | Selects global FM algorithm (0-7) |
| **FM Feedback** | 104 | 0-127 | Global Feedback Amount |
| **Op Attack** | 73 | 0-127 | Selected Operator Attack Time |
| **Op Decay 1** | 75 | 0-127 | Selected Operator Decay 1 Time |
| **Op Decay 2** | 76 | 0-127 | Selected Operator Decay 2 Time |
| **Op Sustain Lvl** | 79 | 0-127 | Selected Operator Sustain Level |
| **Op Release** | 72 | 0-127 | Selected Operator Release Time |
| **Op Ratio/Mult** | 77 | 0-127 | Selected Operator Frequency Multiplier |

## Hardware Setup

*   **Board**: Wemos Lolin32 Lite
*   **I2S DAC**: Connect to standard I2S pins defined in `ML_Boards` (typically bck, ws, data).
*   **Control**: None required. Use a Smartphone or PC with Bluetooth LE.

## Usage

1.  Flash `HeadlessFMSynth.ino` to the ESP32.
2.  Open `index.html` in a Web Bluetooth compatible browser (Chrome, Edge).
3.  Click "Connect BLE MIDI" and select "Headless FM Synth".
4.  Select an Operator (1-4) to edit its specific envelope and tuning.
5.  Adjust Global FM and Filter settings.
