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
 *  main.c - c file for main function
 *  DRV8701EVM_FIRMWARE
 *  1/16/2015
 *
 ******************************************************************************/

/* 				Program Structure
 *
 *					 main.c
 *					  ^^^^
 *			   utility.h->utility.c		(Global Variables, Work Functions)
 *					  ^^^^
 *			   monitor.h->monitor.c		(Serial Monitor for Host PC Connection)
 *					  ^^^^
 *			      uart.h->uart.c		(MSP430 Hardware/Software UART)
 *
 */

#include "msp430.h"
#include "utility.h"

int main(void)
{
	Initialize();

	while (1)
	{
		// Update functions
		UpdateGPIO();
		UpdateVREF();
		UpdatePWM();
        // Enter low power mode and wake up when signaled by the UART
        __bis_SR_register(LPM0_bits + GIE);
	}
}

/****************************Interrupt Service Routines*****************************/

#pragma vector=PORT1_VECTOR, PORT2_VECTOR, ADC10_VECTOR, \
        USCIAB0TX_VECTOR, TIMER0_A0_VECTOR, TIMER0_A1_VECTOR, \
        TIMER1_A0_VECTOR, COMPARATORA_VECTOR, \
        NMI_VECTOR
__interrupt void Trap_ISR(void) {}

// Status LED derived from the watchdog timer
#pragma vector=WDT_VECTOR
__interrupt void WatchDog_Timer(void)
{
    G_LED_COUNT++;
    if (G_LED_COUNT == 10)
    {
        G_LED_COUNT = 0;
        P1OUT ^= STATUS;
    }
}

// Timer1_A1 ISR: Read value of SO (curret shunt amplifier)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1(void)
{
    switch (TA1IV)
    {
        case TA1IV_NONE: break;         // Vector 0: No Interrupt
        case TA1IV_TACCR1:              // Vector 2: CCR1 CCIFG
        {
            TA1CCTL1 &= ~CCIFG;
            break;
        }
        case TA1IV_TACCR2:              // Vector 4: CCR2 CCIFG
        {
            UpdateCUR();
            TA1CCTL2 &= ~CCIFG;
            break;
        }
        case TA1IV_6: break;            // Vector 6: Reserved CCIFG
        case TA1IV_8: break;            // Vector 8: Reserved CCIFG
        case TA1IV_TAIFG:               // Vector 10: Overflow
        {
            TACTL &= ~TAIFG;
            break;
        }
        default: break;
    }
}
