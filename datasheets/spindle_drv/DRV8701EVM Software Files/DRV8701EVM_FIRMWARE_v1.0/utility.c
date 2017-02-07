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
 *  utility.c - Definition file for utility functions
 *  DRV8701EVM_FIRMWARE
 *  1/16/2015
 *
 ******************************************************************************/

#include "msp430.h"
#include "utility.h"
#include "monitor.h"
#include "uart.h"

 /*****************************************************************************/

// Define Global Variables

// LED Timer
unsigned int G_LED_COUNT = 0;

// GUI Variables
float G_FIRMWARE_VERSION = 1.0;

// DRV8701 GPIO
gpio G_nFAULT = low;
gpio G_nSLEEP = low;
gpio G_PH_IN1 = low;
gpio G_SNSOUT = low;

// DRV8701 VREF and EN_IN2
unsigned int G_VREF = 0;
unsigned int G_VREF_TEMP = 0;
unsigned int G_EN_IN2 = 0;
unsigned int G_EN_IN2_TEMP = 0;

// DRV8701 SO (MOTOR CURRENT)
unsigned int G_MOTOR_CURRENT = 0;
unsigned int G_SO_OFFSET = 0;

// Enforce the variable locations in memory for GUI compatibility across version
#pragma DATA_SECTION(G_LED_COUNT,".mysect01")
#pragma DATA_SECTION(G_FIRMWARE_VERSION,".mysect02")
#pragma DATA_SECTION(G_nFAULT,".mysect03")
#pragma DATA_SECTION(G_nSLEEP,".mysect04")
#pragma DATA_SECTION(G_PH_IN1,".mysect05")
#pragma DATA_SECTION(G_SNSOUT,".mysect06")
#pragma DATA_SECTION(G_VREF,".mysect07")
#pragma DATA_SECTION(G_VREF_TEMP,".mysect08")
#pragma DATA_SECTION(G_EN_IN2,".mysect09")
#pragma DATA_SECTION(G_EN_IN2_TEMP,".mysect10")
#pragma DATA_SECTION(G_MOTOR_CURRENT,".mysect11")
#pragma DATA_SECTION(G_SO_OFFSET,".mysect12")


/*****************************************************************************/

void Initialize()
{
    // Set firmware version
    G_FIRMWARE_VERSION = 1.0;

    // Setup CLKs

	// Stop Watchdog Timer
	WDTCTL = WDTPW | WDTHOLD;
	// 8MHz DCO required for 2MHz SMCLK
	DCOCTL = 0x00;
    DCOCTL = CALDCO_8MHZ;
	BCSCTL1 = CALBC1_8MHZ;
	// 2MHz SMCLK required for PWM frequency
	BCSCTL2 |= DIVS_2;
    // ACLK = VLO
    BCSCTL3 |= LFXT1S_2;

	// Configure Port Directions and Peripherals as Needed

	// Configure GPIO
	P1SEL  &= ~(STATUS | PUSH | POT | SO | NC1 | NC2);
	P1SEL2 &= ~(STATUS | PUSH | POT | SO | NC1 | NC2);

	P1DIR |= (STATUS | NC1 | NC2);
	P1OUT &= ~(STATUS | NC1 | NC2);

	P1DIR &= ~(PUSH | POT | SO);

	P2SEL  &= ~(NC3 | EN_IN2 | NC4 | nFAULT | nSLEEP | PH_IN1 | VREF | SNSOUT);
	P2SEL2 &= ~(NC3 | EN_IN2 | NC4 | nFAULT | nSLEEP | PH_IN1 | VREF | SNSOUT);

	P2DIR |= (NC3 | EN_IN2 | NC4 | nSLEEP | PH_IN1 | VREF);
	P2OUT &= ~(NC3 | EN_IN2 | NC4 | nSLEEP | PH_IN1 | VREF);

	P2DIR &= ~(nFAULT | SNSOUT);
	P2OUT |= (nFAULT | SNSOUT);
	P2REN |= (nFAULT | SNSOUT);

	P3DIR |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
	P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);

	// UART Initialization
	uartInit();

	// Enables LPM Interrupts
	__bis_SR_register(GIE);

	// GUI Composer Monitor Initialization
	ClearBufferRelatedParam();

	// Set Default GPIO Settings
	G_nFAULT = low;
	G_nSLEEP = high;
	G_PH_IN1 = low;
	G_SNSOUT = low;

	// Send GPIO Settings to wake DRV8701
	UpdateGPIO();

	// Delay to allow for device wake up (~1ms)
	_delay_cycles(8000);

    // Setup ADC10
    ADC10CTL0 = ADC10SHT_0 | ADC10ON;
    ADC10CTL1 = INCH_5;
    ADC10AE0 |= SO;

    // Calibrate for DRV8701 amplifier offset voltage
    ADC10CTL0 |= ENC | ADC10SC;
    while (ADC10CTL1 & ADC10BUSY);
    ADC10CTL0 &= ~ENC;
    G_SO_OFFSET = ADC10MEM;

    // Set Default VREF Setting (~1V/5A)
    G_VREF = 30;

    // Setup LED Timer
    // Setup Watchdog Timer as an Interval Timer
    WDTCTL = WDT_MDLY_32;
    IE1 |= WDTIE;

    // Setup VREF Timer
    TA0CTL = TASSEL_2 | MC_0 | TACLR;
    TA0CCTL1 = OUTMOD_7;
    TA0CCR0 = 99;
    TA0CCR1 = 0;
    P2SEL |= VREF;
    TA0CTL = TASSEL_2 | MC_1;

    // Setup EN_IN2 Timer
    TA1CTL = TASSEL_2 | MC_0 | TACLR | TAIE;
    TA1CCTL1 = OUTMOD_7;
    TA1CCTL2 = CCIE;
    TA1CCR0 = 99;
    TA1CCR1 = 0;
    // TA1.2 used as a trigger for the SO ADC measurement
    // There is a delay from EN going high to valid SO measurement related to turn-off, turn-on delay of external MOSFETs
    // Each TA tick is 500ns (1/2MHz)
    // TA1CCR2 = 2 will delay the SO measurement (500ns * 2 = 1us delay) until after the rising PWM edge
    TA1CCR2 = 2;
    P2SEL |= EN_IN2;
    TA1CTL = TASSEL_2 | MC_1;
}

void UpdateGPIO()
{
	// Update the GPIO pins from global variables

    // nFAULT
    if ((nFAULT & P2IN) == 0)
        G_nFAULT = low;
    else
        G_nFAULT = high;

    // nSLEEP
    if (G_nSLEEP == high)
        P2OUT |= nSLEEP;
    else
        P2OUT &= ~nSLEEP;

    // PH_IN1
    if (G_PH_IN1 == high)
        P2OUT |= PH_IN1;
    else
        P2OUT &= ~PH_IN1;

    // SNSOUT
    if ((SNSOUT & P2IN) == 0)
        G_SNSOUT = low;
    else
        G_SNSOUT = high;
}

void UpdateVREF()
{
    // Determine if the VREF value has changed
    if (G_VREF != G_VREF_TEMP)
    {
        // Update the VREF duty cycle from GUI value
        G_VREF_TEMP = G_VREF;
        TA0CCR1 = G_VREF;
    }
}

void UpdatePWM()
{
    // Determine if the EN_PH2 value has changed
    if (G_EN_IN2 != G_EN_IN2_TEMP)
    {
        // Update the EN_PH2 duty cycle from GUI value
        G_EN_IN2_TEMP = G_EN_IN2;
        TA1CCR1 = G_EN_IN2;
    }
}

void UpdateCUR()
{
    // Check that the device not current limiting (SNSOUT == 0)
    // Check that the device is in drive (EN == 1)
    // Check that the device is not asleep (nSLEEP == 1)
    if (((SNSOUT & P2IN) != 0) && (G_EN_IN2 != 0) && (G_nSLEEP != 0))
    {
        ADC10CTL0 |= ENC | ADC10SC;
        while (ADC10CTL1 & ADC10BUSY);
        ADC10CTL0 &= ~ENC;

        if (ADC10MEM < G_SO_OFFSET)
            G_MOTOR_CURRENT = 0;
        else
            G_MOTOR_CURRENT = ADC10MEM - G_SO_OFFSET;
    }
    else
        G_MOTOR_CURRENT = 0;
}
