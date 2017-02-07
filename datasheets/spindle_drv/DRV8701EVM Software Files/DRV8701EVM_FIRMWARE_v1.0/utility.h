/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*******************************************************************************
 *
 *  utility.h - Declaration file for utility functions and global variables
 *  DRV8701EVM_FIRMWARE
 *  1/16/2015
 *
 ******************************************************************************/

#ifndef UTILITY_H_
#define UTILITY_H_

 /*****************************************************************************/

// MSP430 Clk Frequencies
#define MCLK_MHz    8       // Main Clock Frequency
#define SMCLK_MHz   2       // Sub Main Clock Frequency

// GPIO Port 1 Definitions
#define STATUS      BIT0    // P1.0
#define PUSH        BIT3    // P1.3
#define POT 		BIT4    // P1.4
#define SO          BIT5    // P1.5
#define NC1         BIT6    // P1.6
#define NC2         BIT7    // P1.7

// GPIO Port 2 Definitions
#define NC3 		BIT0    // P2.0
#define EN_IN2	    BIT1    // P2.1
#define NC4 	    BIT2    // P2.2
#define nFAULT      BIT3    // P2.3
#define nSLEEP		BIT4    // P2.4
#define	PH_IN1		BIT5    // P2.5
#define VREF		BIT6    // P2.6
#define SNSOUT		BIT7    // P2.7

// Custom Types
typedef enum {false, true} boolean;
typedef enum {low, high} gpio;

/*****************************************************************************/

// Declare Global Variables

// LED Timer
extern unsigned int G_LED_COUNT;            // Counter for LED

// GUI Variables
extern float G_FIRMWARE_VERSION;            // Version number of the firmware

// DRV8701 GPIO
extern gpio G_nFAULT;                       // Logic low when in FAULT condition
extern gpio G_nSLEEP;                       // Logic low to enter low-power sleep mode
extern gpio G_PH_IN1;                       // PH -> Determines the direction of the motor
extern gpio G_SNSOUT;                       // Logic low when drive current hits threshold

// DRV8701 VREF and EN_IN2
extern unsigned int G_VREF;                 // VREF duty cycle %
extern unsigned int G_VREF_TEMP;            // Storage value for G_VREF to determine when it changes
extern unsigned int G_EN_IN2;               // EN -> PWM duty cycle determines the speed of the motor
extern unsigned int G_EN_IN2_TEMP;          // Storage value for G_EN_IN2 to determine when it changes

// DRV8701 SO (MOTOR CURRENT)
extern unsigned int G_MOTOR_CURRENT;        // Current through the motor, scaled 0-1023
extern unsigned int G_SO_OFFSET;            // Offset of the SO amplifier

/*****************************************************************************/

void Initialize();                          // Setup the MCU peripherals and default values
void UpdateGPIO();                          // Update the MCU GPIO from GUI values
void UpdatePWM();                           // Update the PWM for motor speed from GUI value
void UpdateVREF();                          // Update the VREF for current regulation from GUI value
void UpdateCUR();                           // Update drive current from the DRV8701 SO pin

#endif /* UTILITY_H_ */
