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


#include <msp430.h>
#include "utility.h"



#pragma vector= DAC12_VECTOR, DMA_VECTOR, USCIAB1TX_VECTOR, USCIAB1RX_VECTOR, \
	            PORT1_VECTOR, PORT2_VECTOR,ADC12_VECTOR, COMPARATORA_VECTOR, \
	            TIMERA0_VECTOR, TIMERB0_VECTOR, NMI_VECTOR
__interrupt void
Trap_ISR(void)
{
}

#pragma vector=TIMERB1_VECTOR
__interrupt void
TimerB1(void)
{
    switch (TBIV) {
    case TBIV_TBCCR1:
	// increment or decrement to the new timer B value
	if (AccelerateState == ACCEL
	    && (SteppingRate <= DesiredTargetSpeed)) {
	    // increase pps and calculate new pps
	    if (SteppingRate + SteppingRateUpdate >= DesiredTargetSpeed) {
		// if (MoveSteps == false && SteppingRate ==
		// DesiredTargetSpeed)
		AccelerateState = RUN;	// No longer accelerate
		SteppingRate = DesiredTargetSpeed;
	    } else {
		SteppingRate += SteppingRateUpdate;	// Increment
		AccelerateState = ACCEL;
	    }
	}
	if (AccelerateState == DECEL) {
	    if (SteppingRate >= StoppingSpeed) {
		// decrease pps and calculate new pps
		if (SteppingRate - SteppingRateUpdate <= StoppingSpeed) {
		    if (MoveSteps == false
			&& SteppingRate == StoppingSpeed)
			AccelerateState = IDLE;	// Stop motor when speed
						// is reached
		    SteppingRate = StoppingSpeed;
		} else {
		    SteppingRate -= SteppingRateUpdate;	// Decrement
		    AccelerateState = DECEL;
		}
	    } else		// already less than stopping speed
	    {
		if (MoveSteps == false)
		    AccelerateState = IDLE;	// Stop motor when speed
						// is reached
	    }
	}
	// this allows for the motor to spin down to new speed without
	// stopping
	if (AccelerateState == DECEL2NEWTARGET) {
	    if (SteppingRate >= DesiredTargetSpeed) {
		// decrease pps and calculate new pps
		if (SteppingRate - SteppingRateUpdate <=
		    DesiredTargetSpeed) {
		    if (SteppingRate == DesiredTargetSpeed)
			AccelerateState = RUN;	// switch to RUN when
						// speed is reached
		    SteppingRate = DesiredTargetSpeed;
		} else {
		    SteppingRate -= SteppingRateUpdate;	// Decrement
		    AccelerateState = DECEL2NEWTARGET;
		}
	    }
	}
	// do not perform the division here -- only update the stepping
	// rate
	// if the stepping rate has been updated, set a flag
	if (SteppingRate != PriorSteppingRate) {
	    UpdateSteppingRateTMR = true;	// Set flag to
						// re-calculate stepping
						// rate TMR
	    PriorSteppingRate = SteppingRate;
	}
	break;
    case TBIV_TBCCR2:
	break;
    case TBIV_TBCCR3:
	break;
    case TBIV_TBCCR4:
	break;
    case TBIV_TBCCR5:
	break;
    case TBIV_TBCCR6:
	break;
    case TBIV_TBIFG:
	break;
    }
}

#pragma vector=TIMERA1_VECTOR
__interrupt void
TimerA1(void)
{
    switch (TAIV) {
    case TAIV_TACCR1:
	break;
    case TAIV_TACCR2:
	if (MoveSteps)		// if manual advance count steps
	{			// when complete
	    tmpStepsToMove++;	// increment step counter
	    if (tmpStepsToMove >= Steps2Decel) {
		AccelerateState = DECEL;	// start deceleration
	    }
	    if (tmpStepsToMove >= StepsToMove)	// stop here when equal
	    {
		AccelerateState = IDLE;
		temp_reciprocation = false;
		if ((G_RECIPROCATION == true) && (temp_delay == false)) {
		    if (G_DIR == high)	// change direction when finished
					// the steps
		    {
			G_DIR = low;
		    } else
			G_DIR = high;
		    temp_delay = true;
		}
		if (G_RECIPROCATION == false) {
		    if (last_reciprocation_state == 1) {

			if (G_DIR == high)	// reverse the direction
						// when the motor is
						// stopped
			{
			    G_DIR = low;
			} else
			    G_DIR = high;
			last_reciprocation_state = 0;
		    }
		    // TACTL &= ~MC_3; // Stop timerA
		    // TBCTL &= ~MC_3; // Stop timerB
		    MoveSteps = false;
		}
		TACTL &= ~MC_3;	// Stop timerA
		TBCTL &= ~MC_3;	// Stop timerB
	    }

	}
	// Reload the timers below
	TACCR0 = SteppingRateTMR;	// 
	TACCR2 = SteppingRateTMR >> 1;	// set at 50%
	break;
    case TAIV_TAIFG:
	break;
    }
}


// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void
watchdog_timer(void)
{
    delay_cycles++;
}
