/*
 * Copyright (c) 2024 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/**
 * @file config.h
 * @author Marcel Licence
 * @date 12.05.2021
 *
 * @brief This file contains the project configuration
 *
 * All definitions are visible in the entire project
 *
 * Put all your project settings here (defines, numbers, etc.)
 * configurations which are requiring knowledge of types etc.
 * shall be placed in z_config.ino (will be included at the end)
 */


#ifndef CONFIG_H_
#define CONFIG_H_


#define STATUS_SIMPLE /* use local definitions */

#ifdef __CDT_PARSER__
#include <cdt.h>
#endif

#define SERIAL_BAUDRATE 115200
#define MIDI_SERIAL_BAUDRATE SERIAL_BAUDRATE

#define MIDI_CHANNEL_PRESSURE_ENABLED

#ifdef ESP32
//#define BOARD_ML_V1 /* activate this when using the ML PCB V1 */
#define BOARD_ML_SYNTH_V2
//#define BOARD_ESP32_AUDIO_KIT_AC101 /* activate this when using the ESP32 Audio Kit v2.2 with the AC101 codec */
//#define BOARD_ESP32_AUDIO_KIT_ES8388 /* activate this when using the ESP32 Audio Kit v2.2 with the ES8388 codec */
//#define BOARD_ESP32_DOIT /* activate this when using the DOIT ESP32 DEVKIT V1 board */
//#define BOARD_SAA_S2132
#endif

/* can be used to pass line in through audio processing to output */
//#define AUDIO_PASS_THROUGH

#define SAMPLE_BUFFER_SIZE  48

/* max delay can be changed but changes also the memory consumption */
#define MAX_DELAY   (SAMPLE_RATE/3) /* 1/2s */

//#define LED_MATRIX_ENABLED
//#define ADC_TO_MIDI_ENABLED

#define MIDI_RECV_FROM_SERIAL

//#define VT100_ENABLED

/* use this to display a scope on the oled display */
//#define OLED_OSC_DISP_ENABLED

//#define ARP_MODULE_ENABLED
#define MIDI_SYNC_MASTER


/*
 * include the board configuration
 * there you will find the most hardware depending pin settings
 */
#include <ml_boards.h> /* requires library ML_SynthTools: https://github.com/marcel-licence/ML_SynthTools */

#ifdef BOARD_ML_V1

#define OLED_OSC_DISP_ENABLED
#define SPI_DISP_ENABLED
#define SPI_DISP_ALGO_ENABLED
#define ADC_MCP_CTRL_ENABLED
#define DISPLAY_160x80
#define ADC_TO_MIDI_ENABLED
#define ADC_TO_MIDI_LOOKUP_SIZE 8
#define ADC_INPUTS 8
#define DISPLAY_FROM_STATUS_ENABLED
#elif (defined BOARD_ML_SYNTH_V2)
#define OLED_OSC_DISP_ENABLED
#elif (defined BOARD_ESP32_AUDIO_KIT_AC101)
#elif (defined BOARD_ESP32_AUDIO_KIT_ES8388)
#elif (defined BOARD_ESP32_DOIT)
#elif (defined BOARD_SAA_S2132)
#else

/*
 * DIN MIDI Pinout
 */
#if 0
#define MIDI_PORT2_ACTIVE
#define MIDI_RX_PIN 16 /* U2RRXD */
#define MIDI_TX_PIN 17
#endif

#endif


#ifndef I2C_SPEED
//#define I2C_SPEED 100000
#endif

/*
 * You can modify the sample rate as you want
 */
#ifdef ESP32_AUDIO_KIT
#define SAMPLE_RATE 48000
#define SAMPLE_SIZE_16BIT
#else
#define SAMPLE_RATE 48000
#define SAMPLE_SIZE_16BIT /* 32 bit seems not to work at the moment */
#endif


/*
 * configuration for the Raspberry Pi Pico 2
 * BOARD: Raspberry Pi RP2040 (4.0.1)
 * Device: Raspberry Pi Pico 2
 */
#ifdef ARDUINO_ARCH_RP2040
#ifdef __ARM_FEATURE_DSP
//#define MAX_DELAY_Q 8096
#define PICO_AUDIO_I2S
#define PICO_AUDIO_I2S_DATA_PIN 26
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 27
#define MIDI_RX1_PIN    13
#define MIDI_TX1_PIN    12
//#define WS2812_PIN  3
//#define LED_COUNT 4
#define LED_PIN LED_BUILTIN
#define BLINK_LED_PIN LED_BUILTIN
//#define WS2812_PIN 3
#endif
#endif


/*
 * Configuration for
 * Board: "Generic STM32F4 Series"
 * Board part number: "Generic F407VG"
 *
 * does not work at the moment
 */
#ifdef ARDUINO_DISCO_F407VG

#include <ml_boards.h> /* requires the ML_Synth library:  https://github.com/marcel-licence/ML_SynthTools */

#define BLINK_LED_PIN LED_USER_RED
#define LED_PIN LED_USER_GREEN

//#define SAMPLE_BUFFER_SIZE  48
//#define SAMPLE_RATE  48000

#define MIDI_PORT2_ACTIVE

//#define AUDIO_BLINK_GPIO GPIOD
//#define AUDIO_BLINK_Pin LD4_Pin

#endif /* ARDUINO_DISCO_F407VG */
#endif /* CONFIG_H_ */

