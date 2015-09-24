/*
 * --COPYRIGHT--,BSD Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved. Redistribution and use in source and binary
 * forms, with or without modification, are permitted provided that the
 * following conditions are met: * Redistributions of source code must
 * retain the above copyright notice, this list of conditions and the
 * following disclaimer. * Redistributions in binary form must reproduce
 * the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with
 * the distribution. * Neither the name of Texas Instruments Incorporated 
 * nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE. --/COPYRIGHT--
 */


/*******************************************************************************
 *
 *  main.c - c file for main function
 *  DRV8880_FIRMWARE
 *  1/30/2015
 *
 ******************************************************************************/

/*
 * Program Structure main.c ^^^^ utility.h->utility.c (Global Variables,
 * Work Functions, SPI) ^^^^ ^^^^ ^^^^ monitor.h->monitor.c (Serial
 * Monitor for Host PC Connection) ^^^^ uart.h->uart.c (MSP430
 * Hardware/Software UART) 
 */

#include <msp430f2617.h>
#include "utility.h"

int
main(void)
{
    Initialize();		// Set up initial states of micro

    while (1) {
	// Enter LPM and wake up periodically
	// __bis_SR_register(LPM0_bits + GIE);
	__bis_SR_register(GIE);
	// Update Functions
	UpdateDeviceControls();	// read the GUI values and pass to device
	UpdateGUIControls();	// read device values and pass to GUI
	UpdateSpeed();		// control the motor based on info
				// captured above
    }
}
