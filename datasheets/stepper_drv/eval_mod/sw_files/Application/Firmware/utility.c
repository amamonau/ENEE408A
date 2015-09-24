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
 *  utility.c - c file for utility functions
 *  DRV8880EVM_FIRMWARE
 *  1/30/2015
 *
 ******************************************************************************/

#include "msp430f2617.h"
#include "utility.h"
#include "Serial_Cmd_Monitor.h"
#include "uart.h"

/*****************************************************************************/

// Define Global Variables in utility.h

/*****************************************************************************/

// variables to calculate ifs
unsigned int    temp_torque_val = 500;	// out of range values to force
					// recalculation
unsigned int    temp_vref_val = 32000;	// out of range values to force
					// recalculation
unsigned int    temp_step_mode = 32000;	// out of range values to force
					// recalculation
unsigned int    temp_decay_mode = 32000;	// out of range to force
						// recalculation
unsigned int    temp_pwm_off_time = 32000;	// out of range to force
						// recalculation
float           temp_irsense = 100;	// out of range values to force
					// recalculation


// Function Definitions
void
Initialize()
{
    // Setup clocks

    DCOCTL = CALDCO_8MHZ;	// Set DCO to 8MHz
    BCSCTL1 = CALBC1_8MHZ | XTS;
    BCSCTL3 = LFXT1S_2;		// ACLK = VLO

    WDTCTL = WDT_MDLY_32;	// Set Watchdog Timer interval to 4ms
    // WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    // Configure port directions and peripherals as needed
    // Note: All unused ports should be set to either output or input with
    // pullup/down enabled

    // 
    P1DIR = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high
    P1OUT = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high

    P3DIR = BIT7 | BIT6 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All
								// high
								// except
								// bit 5
								// (RXD)
    P3OUT = BIT7 | BIT6 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All
								// high

    P5DIR = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high
    P5OUT = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high

    P6DIR = ADEC | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All
								// high
								// except
								// bit 7
    P6OUT = BIT7 | ADEC | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All
								// high
								// except
								// bit 5
								// (status 
								// led)

    P7DIR = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high
    P7OUT = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high

    P8DIR = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high
    P8OUT = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high

    PADIR = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high
    PAOUT = BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0;	// All 
									// high

    P2DIR = TOFF | USM0 | TRQ0 | STEP | USM1 | TRQ1 | DECAY0 | DECAY1;	// Bits 
									// 6:0
    P4OUT = 0x00;
    P4DIR = V3P3_GPIO | WAKE | BIT4 | BIT3 | DIR | ENABLE | BIT0;	// FAULT 
									// is 
									// input
    P4REN |= FAULT;		// SET FAULT as input pullup


    // Configure GPIO
    P4OUT &= ~ENABLE;		// ENABLE should be low, DIR, ADEC,
    P4OUT |= FAULT;		// set pullup on FAULT
    // STEP, WAKE should be low for default.
    // set Current to 100%, Mixed/Mixed and 1/16 rising edge only
    P2OUT = USM0;
    CAPD = CAPD_DECAY1 | CAPD_USM1;

    // Configure DAC
    // if DAC12AMP > 0, DAC is routed to pin automatically
    // TO BE REMOVED
    // ADC12CTL0 = REF2_5V | REFON; // remove this line and next once part 
    // is soldered on
    DAC12_1CTL = DAC12SREF_3 | DAC12IR | DAC12AMP_7;	// Select VeREF,
							// 1x, and Amp
							// high speed/high 
							// current

    // ADC is unused, no need to configure

    // UART Initialization
    uartInit();

    // Enables LPM Interrupts
    __bis_SR_register(GIE);

    // GUI Composer Monitor Initialization
    ClearBufferRelatedParam();

    // Setup MSP430 Timer
    // Configure Timer B to run up to FFFF (16 bit FRC)
    P2SEL |= STEP;

    // timer setup
    // select individual, 16bit counter length, SMCLK, divide by 1, stop

    // configure timer A
    TACTL = CNTL_0 | TASSEL_2 | ID_0 | MC_0;

    // configure timer A
    // select SMCLK, divide by 1, stop
    TBCTL = TBCLGRP_0 | TBSSEL_2 | ID_3 | MC_0;

    // Set Default GUI Variables Values
    SetDefaultGUIVals();

    // Set Default GPIO Values
    SetDefaultGPIOVals();

    // Load GPIO Values
    UpdateDeviceControls();
    UpdateGUIControls();

    // set boolean defaults
    Block_GUI_FAULT_button = false;
    ForceUpdate = false;
    LastCommandWasRunSteps = false;
    MoveSteps = false;
    UpdateChoppingCurrent = false;
    UpdateSteppingRateTMR = false;
    temp_reciprocation = false;
}

// 
// routine to communicate between GUI and device
// reads GUI variables and updates the GPIOs

void
UpdateDeviceControls()
{
    LED_TOGGLE_CNT++;
    // toggle status if count is reached
    if (LED_TOGGLE_CNT == LED_TIMEOUT) {
	toggle_status_led;
	LED_TOGGLE_CNT = 0;
	ForceUpdate = true;	// force an update of the chopping current
    }
    // From GUI to Micro
    // Update the GPIO Pins from Global Variables
    // WAKE
    if (G_WAKE == high)
	set_WAKE_hi;
    else
	set_WAKE_lo;

    // DIR
    if (G_DIR == high)
	set_DIR_hi;
    else
	set_DIR_lo;

    // ENABLE -- set port high to enable device
    if (G_ENABLE == high)
	if (set_ENABLE_as_opendrain == true) {
	    disable_ENABLE_output;
	    enable_V3P3_GPIO;
	} else {
	    set_ENABLE_hi;
	    enable_ENABLE_output;
    } else {
	enable_nFAULT_pullup;
	set_ENABLE_lo;
	enable_ENABLE_output;
	// add a check when wake is low to determine if
	// the FAULT button should toggle
	// first read the nFAULT pin (it should be high if
	// the RETRY jumper is not populated
	if (read_FAULT_pin == 0) {
	    Block_GUI_FAULT_button = true;
	    // set ENABLE as open drain output
	    set_ENABLE_as_opendrain = true;
	    disable_V3P3_GPIO;
	} else {
	    Block_GUI_FAULT_button = false;
	    // set ENABLE as push pull output
	    set_ENABLE_as_opendrain = false;
	    disable_nFAULT_pullup;
	    enable_V3P3_GPIO;
	}
    }

    // ADEC -- set ADEC port
    if (G_ADEC == high)
	set_ADEC_hi;
    else
	set_ADEC_lo;
    if (tmp_adec != G_ADEC)
	// must reset wake to a logic low if ADEC has been changed
	// override wake setting above if G_ADEC has changed
    {
	G_WAKE = low;
	set_WAKE_lo;
	tmp_adec = G_ADEC;
	if (G_ADEC == high)
	    PriorDecayMode = G_DECAY_MODE;	// save previous decay
						// mode to restore
	else
	    G_DECAY_MODE = PriorDecayMode;	// restore previous decay
						// mode
    }
    // override DECAY mode if using ADEC
    // DECAY_MODE - DECAY1 CNTL
    if (G_ADEC == high) {
	// set DECAY1, DECAY0 to 00
	// DECAY1,0 DIR
	enable_DECAY1_output;
	enable_DECAY0_output;
	// set DECAY1,0 OUT to 00
	set_DECAY1_lo;
	set_DECAY0_lo;
	// enable DECAY1,DECAY0 inputs
	enable_DECAY1_input;
	enable_DECAY0_input;
	G_DECAY_MODE = 9;	// Set decay mode to 9, display ADEC
				// images
    }
    // if G_ADEC is not high use normal decay settings
    else if (temp_decay_mode != G_DECAY_MODE || ForceUpdate == true) {
	switch (G_DECAY_MODE) {
	case 0:
	case 1:
	case 4:		// 0
	    enable_DECAY1_output;
	    set_DECAY1_lo;	// DECAY1 OUT
	    enable_DECAY1_input;	// DECAY1 CAP CNTL
	    break;
	case 2:
	case 3:
	case 5:		// 1
	    enable_DECAY1_output;
	    set_DECAY1_hi;	// DECAY1 OUT
	    enable_DECAY1_input;	// DECAY1 CAP CNTL
	    break;
	case 6:
	case 7:
	case 8:		// Z
	    disable_DECAY1_output;
	    set_DECAY1_lo;	// DECAY1 OUT
	    disable_DECAY1_input;	// DECAY1 CAP CNTL
	    break;
	}
	// DECAY_MODE - DECAY0 CNTL
	switch (G_DECAY_MODE) {
	case 0:
	case 2:
	case 6:		// 0
	    enable_DECAY0_output;
	    set_DECAY0_lo;	// DECAY0 OUT
	    enable_DECAY0_input;	// DECAY0 CAP CNTL
	    break;
	case 1:
	case 3:
	case 7:		// 1
	    enable_DECAY0_output;
	    set_DECAY0_hi;	// DECAY0 OUT
	    enable_DECAY0_input;	// DECAY0 CAP CNTL
	    break;
	case 4:
	case 5:
	case 8:		// Z
	    disable_DECAY0_output;
	    set_DECAY0_lo;	// DECAY0 OUT
	    disable_DECAY0_input;	// DECAY0 CAP CNTL
	    break;
	}
	temp_decay_mode = G_DECAY_MODE;
    }
    if (temp_step_mode != G_STEP_MODE || ForceUpdate == true) {
	// STEP CONTROL
	// STEP_MODE -- USM1
	switch (G_STEP_MODE) {
	case 0:
	case 1:
	case 4:		// 0
	    enable_USM1_output;
	    set_USM1_lo;	// USM1 OUT
	    enable_USM1_input;	// USM1 CAP CNTL
	    break;
	case 6:
	case 7:
	case 8:		// Z -- reserved
	    disable_USM1_output;
	    set_USM1_lo;	// USM1 OUT
	    disable_USM1_input;	// USM1 CAP CNTL
	    break;
	case 2:
	case 3:
	case 5:		// 1
	    enable_USM1_output;
	    set_USM1_hi;	// USM1 OUT
	    enable_USM1_input;	// USM1 CAP CNTL
	    break;
	}

	// STEP_MODE - USM0 CNTL
	switch (G_STEP_MODE) {
	case 0:
	case 2:
	case 6:		// 0
	    enable_USM0_output;
	    set_USM0_lo;	// USM0 OUT
	    enable_USM0_input;	// USM0 CAP CNTL
	    break;
	case 4:
	case 5:
	case 8:		// Z
	    disable_USM0_output;
	    set_USM0_lo;	// USM0 OUT
	    disable_USM0_input;	// USM0 CAP CNTL
	    break;
	case 1:
	case 3:
	case 7:		// 1
	    enable_USM0_output;
	    set_USM0_hi;	// USM0 OUT
	    enable_USM0_input;	// USM0 CAP CNTL
	    break;
	}
    }

    if (temp_torque_val != G_TORQUE || ForceUpdate == true) {
	// TORQUE CONTROL
	// TORQUE CONTROL -- TRQ1 CNTL
	switch (G_TORQUE) {
	case 0:		// 00
	    set_TRQ1_lo;
	    set_TRQ0_lo;
	    break;
	case 1:		// 01
	    set_TRQ1_lo;
	    set_TRQ0_hi;
	    break;
	case 2:		// 10
	    set_TRQ1_hi;
	    set_TRQ0_lo;
	    break;
	case 3:		// 11
	    set_TRQ1_hi;
	    set_TRQ0_hi;
	    break;
	}
    }

    if (temp_pwm_off_time != G_PWM_OFF_TIME || ForceUpdate == true) {
	// PWM OFF TIME -- 14Apr -- Corrected mismatch between pulldown
	// menu and selection
	switch (G_PWM_OFF_TIME) {
	case 0:		// 0, 20us
	    enable_TOFF_output;
	    set_TOFF_lo;	// PWM_OFF_TIME OUT
	    enable_TOFF_input;	// PWM_OFF_TIME CAP CNTL
	    break;
	case 1:		// 1, 30us
	    enable_TOFF_output;
	    set_TOFF_hi;	// PWM_OFF_TIME OUT
	    enable_TOFF_input;	// PWM_OFF_TIME CAP CNTL
	    break;
	case 2:		// Z, 10us
	    disable_TOFF_output;
	    set_TOFF_lo;	// PWM_OFF_TIME OUT
	    disable_TOFF_input;	// PWM_OFF_TIME CAP CNTL
	    break;
	}
	temp_pwm_off_time = G_PWM_OFF_TIME;
    }
}

// 
// routine to communicate between device and GUI
// reads GPIOs and updates the GUI

void
UpdateGUIControls()
{
    // From Micro to GUI
    // FAULT -- block GUI FAULT button if the RETRY jumper is inserted
    if (Block_GUI_FAULT_button == true)
	G_FAULT = high;
    else {
	if (read_FAULT_pin == 0)
	    G_FAULT = low;
	else
	    G_FAULT = high;
    }
    // Enable Motor control buttons
    if (G_WAKE == high && G_ENABLE == high) {
	if (G_RUN_MOTOR == false && G_MANUAL_ADVANCE == false
	    && G_RECIPROCATION == false) {
	    G_ENBL_START_STOP = false;	// enabled with false // only
					// reset if not running
	    G_ENBL_MOVE_STEPS = false;	// enabled with false
	    G_ENBL_RECIPROCATION_BUTTON = false;
	    G_ENABLE_MTR_CONTROL = false;	// pop up message to
						// select wake and enable
	}
    } else {
	G_ENABLE_MTR_CONTROL = true;	// pop up message to select wake
					// and enable
	G_ENBL_START_STOP = true;	// disable with true
	G_ENBL_MOVE_STEPS = true;	// disable with true
	G_ENBL_RECIPROCATION_BUTTON = true;	// disable with true
	// when disabled reset the indicators in the motor operation
	G_RUN_MOTOR = false;
	G_MANUAL_ADVANCE = false;
	G_RECIPROCATION = false;
	G_MOTOR_CMD_ACTIVE1 = false;
	G_MOTOR_CMD_ACTIVE2 = false;
	G_MOTOR_CMD_ACTIVE3 = false;
	// stop the timer also
	TACTL &= ~MC_3;		// Stop timer by setting mc_0
	TACCR0 = 0x0000;	// once stopped reset the timer value
	// clear temp_reciprocation and MoveSteps so next movement is
	// correct
	temp_reciprocation = false;	// reset reciprocation
	MoveSteps = false;
	AccelerateState = IDLE;	// set AccelerateState to idle when
				// sleeping or disabled
    }

    if (AccelerateState == IDLE)
	G_IDLE = true;		// set to 1 if IDLE
    else
	G_IDLE = false;

    // Set DAC value by multiplying by 4
    DAC12_1DAT = G_VREF_VAL << 2;	// take the G_VREF_VAL and
					// multiply by 4

    // update the full scale chopping current if necessary
    if ((temp_torque_val != G_TORQUE) || ForceUpdate == true) {
	temp_torque_val = G_TORQUE;
	torque_multiplier = (4 - temp_torque_val) * TORQUE_STEP;
	UpdateChoppingCurrent = true;
    }
    if ((temp_vref_val != G_VREF_VAL) || ForceUpdate == true) {
	temp_vref_val = G_VREF_VAL;
	vref_multiplier = temp_vref_val * VREF_BIT;	// vref bit =
							// 1/1023
	UpdateChoppingCurrent = true;
    }
    if ((temp_step_mode != G_STEP_MODE) || ForceUpdate == true) {
	temp_step_mode = G_STEP_MODE;
	if (temp_step_mode == 0)
	    step_multiplier = .71;	// full step, use 71%
	else
	    step_multiplier = 1;	// microstepping use 100%
	UpdateChoppingCurrent = true;
    }
    if ((temp_irsense != G_IRSENSE) || ForceUpdate == true) {
	temp_irsense = G_IRSENSE;
	irsense_multiplier = 1 / temp_irsense;	// Could not avoid
						// division here, but it
						// is the only place
	UpdateChoppingCurrent = true;
    }
    if (UpdateChoppingCurrent == true)	// recalculate chopping current
					// based on multipliers
    {
	G_IFS =
	    V3P3_TIMES_GAIN * vref_multiplier * irsense_multiplier *
	    step_multiplier * torque_multiplier;
	UpdateChoppingCurrent = false;	// reset the flag
	ForceUpdate = false;
    }
}


void
SetDefaultGUIVals()
{
    temp_delay = false;
    last_reciprocation_state = 0;
    delay_cycles = 0;
    G_FIRMWARE_VERSION = FW_VERSION;	// 
    G_MOTOR_CMD_ACTIVE1 = false;
    G_MOTOR_CMD_ACTIVE2 = false;
    G_MOTOR_CMD_ACTIVE3 = false;
    G_STARTING_SPEED = DEFAULT_START_SPEED;	// initial speed in pulses 
						// per second when
						// starting motor
    G_TARGET_SPEED = DEFAULT_TARGET_SPEED;	// speed of motor once
						// running
    G_ACCEL_RATE = DEFAULT_ACCEL_RATE;	// acceleration rate from starting 
					// speed to target speed
    G_STOPPING_SPEED = DEFAULT_STOPPING_SPEED;	// speed to stop motor
						// once deceleration has
						// reached this point
    G_NUM_STEPS = DEFAULT_NUM_STEPS;	// number of steps to move when
					// manually stepping
    G_STEPS_2_STOP = DEFAULT_STEPS_2_STOP;	// number of steps used to 
						// stop the motor when
						// manually stepping
    G_DECAY_MODE = DEFAULT_DECAY;	// Mixed 1tblank / Mixed 1 tblank
    G_STEP_MODE = DEFAULT_STEP_MODE;	// 8 rising edge only
    G_TORQUE = DEFAULT_TORQUE;	// 100%
    G_PWM_OFF_TIME = DEFAULT_PWM_OFF_TIME;	// 20us off
    G_IFS = DEFAULT_IFS;
    G_IRSENSE = DEFAULT_SENSE_RESISTOR;
    G_VREF_VAL = DEFAULT_VREF_VAL;

    G_ENBL_START_STOP = false;
    G_ENBL_MOVE_STEPS = false;
    G_ENBL_RECIPROCATION_BUTTON = false;
    G_RUN_MOTOR = false;	// Auto run enabled
    G_MANUAL_ADVANCE = false;	// Manually advance motor
    G_RECIPROCATION = false;
    G_ENABLE_MTR_CONTROL = true;	// pop up message
}


// 
// this routine sets the defaults in the gui
// 
void
SetDefaultGPIOVals()
{
    G_WAKE = low;
    G_DIR = low;
    G_ENABLE = low;
    G_ADEC = high;		// G_ADEC is ADEC
    G_FAULT = high;
}

// 
// routine to determine if any changes are in motor operation are required
// and pass the information to the motor state machine
// 
void
UpdateSpeed()
{
    StartingSpeed = (unsigned long) G_STARTING_SPEED << use_one_edge;	// Configure 
									// the 
									// starting 
									// speed
    StoppingSpeed = (unsigned long) G_STOPPING_SPEED << use_one_edge;	// Configure 
									// the 
									// stopping 
									// speed
    StepsToMove = (unsigned long) G_NUM_STEPS << use_one_edge;	// adjust
								// for
								// both
								// edges
    StepsToStop = (unsigned long) G_STEPS_2_STOP << use_one_edge;	// adjust 
									// for 
									// both 
									// edges
    DesiredTargetSpeed = (unsigned long) G_TARGET_SPEED << use_one_edge;	// Configure 
										// the 
										// target 
										// speed
    AccelRate = (unsigned long) G_ACCEL_RATE << use_one_edge;	// Configure 
								// the
								// acceleration 
								// rate
								// here

    DetermineMotorMovement();	// Once type of motor movement is
				// determined
    MotorStateMachine();	// Command the motor to move
}

// 
// Routine to determine desired motor operation
// direction, acceleration, deceleration, steps to move
// reciprication
// 
void
DetermineMotorMovement()
{
    // calculate steps to target and steps to stop prior to starting motor
    if ((G_RUN_MOTOR == true || G_MANUAL_ADVANCE == true
	 || G_RECIPROCATION == true) && G_MOTOR_CMD_ACTIVE2 == false
	&& G_MOTOR_CMD_ACTIVE1 == false && G_MOTOR_CMD_ACTIVE3 == false) {
	// update steps to speed and steps to stop calculations
	// only calculate when the motor is idle and commanded to move
	time = (float) (DesiredTargetSpeed - StartingSpeed) / AccelRate;
	time_squared = time * time;
	// calculate steps to target
	Steps2Target =
	    ((AccelRate >> 1) * (time_squared)) + (StartingSpeed * time);
	// calculate Steps2Stop if:
	// StepsToStop in GUI is >1 and
	// StoppingSpeed is not equal Starting Speed
	// 
	if ((StepsToStop == 2 && use_one_edge == 1) || StepsToStop == 1) {
	    if (StartingSpeed != StoppingSpeed) {	// recalculate
							// time if
							// necessary
		time =
		    (float) (DesiredTargetSpeed -
			     StoppingSpeed) / AccelRate;
		time_squared = time * time;
	    }
	    // calculate steps required to stop
	    Steps2Stop =
		((AccelRate >> 1) * (time_squared)) +
		(StartingSpeed * time);
	} else			// defined number of steps to stop
	{
	    Steps2Stop = StepsToStop;
	}
	// calculate steppingrate update (integer value divide by 32)
	if (AccelRate > 31 || AccelRate == 0)	// if >=32 or 0
	{
	    SteppingRateUpdate = AccelRate >> 5;	// divide by 32 -
							// right shift 5
	} else {
	    SteppingRateUpdate = 1;	// increment by 1 to get correct
					// value
	}
	if ((StartingSpeed >= DesiredTargetSpeed)
	    || (StoppingSpeed >= DesiredTargetSpeed))
	    // Special case -- override prior settings
	    // allow motor to attempt to run only at starting speed
	{
	    SteppingRateUpdate = 0;
	    Steps2Target = 0;
	    Steps2Stop = 0;
	}
	// transition from ACCEL to DECEL when count equals (StepsToMove - 
	// Steps2Stop) <=0
	if ((Steps2Stop > (StepsToMove >> 1))
	    && (StartingSpeed == StoppingSpeed))
	    Steps2Decel = StepsToMove >> 1;	// decel at midpoint
	else
	    Steps2Decel = StepsToMove - Steps2Stop;
    }
    // start running or stopping the motor if commanded and not manually
    // advancing
    if (G_RUN_MOTOR == true && G_MOTOR_CMD_ACTIVE1 == false
	&& G_MOTOR_CMD_ACTIVE2 == false && G_MOTOR_CMD_ACTIVE3 == false) {
	// START STEPPER
	G_MOTOR_CMD_ACTIVE1 = true;	// Indicate that the motor is
					// running
	G_ENBL_MOVE_STEPS = true;	// disable with true; lock out
					// Move Steps
	G_ENBL_RECIPROCATION_BUTTON = true;


	LastCommandWasRunSteps = true;	// flag to determine if the timer
					// requires clearing

	DesiredStepperSpeed = DesiredTargetSpeed;	// Configure the
							// final stepping
							// rate in PPS

	SteppingRate = StartingSpeed;	// This is the starting speed from 
					// the GUI
	// adjusted by the edge indicator
	SteppingRateTMR = SMCLK_FREQ / StartingSpeed;	// Convert here to 
							// the timer value 
							// -- xxTMR means
							// the
	// value to be loaded into the timer
	AccelerateState = ACCEL;	// signal to start state machine
	// Configure TimerA
	TACCTL2 &= 0xFF1F;	// Clear OUTMODx bits; 3 MSB on lower
				// byte; Timer configured to output mode
	TACCTL2 |= OUTMOD_4 + CCIE;	// Configure the timer as TOGGLE.
					// TBCCR0 does the reset
	TACCR0 = SteppingRateTMR;	// Start the timer here with
					// initial value
	TACCR2 = SteppingRateTMR >> 1;	// set TBCCR1 to the midpoint of
					// the TBCCR0 for toggle
	// configure TimerB
	TBCCTL1 &= 0xFF1F;	// Clear OUTMODx bits; 3 MSB on lower
				// byte; Timer configured to output mode
	TBCCTL1 |= OUTMOD_4 + CCIE;
	TBCCR0 = TIME_32ms;	// set timer to 32ms
	TBCCR1 = TIME_32ms >> 1;
	// start timers
	TACTL |= MC_1;		// Start timerA by selecting up mode
	TBCTL |= MC_1;		// start timerB
    }
    if ((G_RUN_MOTOR == false && G_MOTOR_CMD_ACTIVE1 == true))
	// motor is running and stop is issued
    {
	// STOP STEPPER
	G_MOTOR_CMD_ACTIVE1 = false;	// must set low when stopping
	G_ENBL_MOVE_STEPS = false;	// enable with false, enable Move
					// Steps
	G_ENBL_RECIPROCATION_BUTTON = false;
	AccelerateState = DECEL;
	DesiredStepperSpeed = StoppingSpeed;	// Speed to stop at
    }
    // MOVE_STEPS
    // Manually advance steps if commanded
    // only allow starting if not automatically running
    if (G_MANUAL_ADVANCE == true && G_MOTOR_CMD_ACTIVE1 == false
	&& G_MOTOR_CMD_ACTIVE2 == false && G_MOTOR_CMD_ACTIVE3 == false) {
	G_MOTOR_CMD_ACTIVE2 = true;
	G_ENBL_START_STOP = true;	// disable with true, lock out
					// Start Stop
	G_ENBL_RECIPROCATION_BUTTON = true;	// disable with true, lock 
						// out reciprocation
						// function
	if (LastCommandWasRunSteps == true || use_one_edge == true)	// 15APR 
									// -- 
									// added 
									// control 
									// to 
									// avoid 
									// resetting
	{
	    // reset timer only when starting or when using one edge
	    TACTL |= TACLR;	// reset timerA
	    TACTL = CNTL_0 | TASSEL_2 | ID_0 | MC_0;
	    TACCTL2 &= 0xFF1F;	// Clear OUTMODx bits; 3 MSB on lower
				// byte; Timer configured to output mode
	    TACCTL2 |= OUTMOD_4 + CCIE;	// Configure the timer as
					// SET_RESET. TBCCR0 does the
					// reset
	}
	LastCommandWasRunSteps = false;	// reset flag to indicate move
					// steps was last commanded
	DesiredStepperSpeed = DesiredTargetSpeed;	// Configure the
							// Frequency Rate

	SteppingRate = StartingSpeed;
	SteppingRateTMR = SMCLK_FREQ / StartingSpeed;	// convert to
							// timer value
	AccelerateState = ACCEL;	// signal to start state machine
	tmpStepsToMove = 0;	// clear counter
	MoveSteps = true;	// set flag to indicate manual advance
	// configure Timer A
	TACCR0 = SteppingRateTMR;	// start the timer
	TACCR2 = SteppingRateTMR >> 1;	// now using count up
	// configure TimerB
	TBCCTL1 &= 0xFF1F;	// Clear OUTMODx bits; 3 MSB on lower
				// byte; Timer configured to output mode
	TBCCTL1 |= OUTMOD_4 + CCIE;
	TBCCR0 = TIME_32ms;	// set timer to 32ms
	TBCCR1 = TIME_32ms >> 1;
	// start timers
	TACTL |= MC_1;		// Start timer by selecting up mode
	TBCTL |= MC_1;
    }
    if (temp_delay == true) {
	IE1 |= WDTIE;		// Enable WDT interrupt
	if (delay_cycles > 100)	// 4ms*100 = 0.4s
	{
	    IE1 &= ~WDTIE;
	    delay_cycles = 0;
	    temp_delay = false;
	}
    }
    if (G_RECIPROCATION == true && temp_reciprocation == false
	&& temp_delay == false) {
	last_reciprocation_state = 1;
	temp_reciprocation = true;
	G_MOTOR_CMD_ACTIVE3 = true;
	G_ENBL_MOVE_STEPS = true;
	G_ENBL_START_STOP = true;
	if (LastCommandWasRunSteps == true || use_one_edge == true)	// 15APR 
									// -- 
									// added 
									// control 
									// to 
									// avoid 
									// resetting
	{
	    // reset timer only when starting or when using one edge
	    TACTL |= TACLR;	// reset timerA
	    TACTL = CNTL_0 | TASSEL_2 | ID_0 | MC_0;
	    TACCTL2 &= 0xFF1F;	// Clear OUTMODx bits; 3 MSB on lower
				// byte; Timer configured to output mode
	    TACCTL2 |= OUTMOD_4 + CCIE;	// Configure the timer as
					// SET_RESET. TBCCR0 does the
					// reset
	}
	LastCommandWasRunSteps = false;	// reset flag to indicate move
					// steps was last commanded
	DesiredStepperSpeed = DesiredTargetSpeed;	// Configure the
							// Frequency Rate

	SteppingRate = StartingSpeed;
	SteppingRateTMR = SMCLK_FREQ / StartingSpeed;	// convert to
							// timer value
	AccelerateState = ACCEL;	// signal to start state machine
	tmpStepsToMove = 0;	// clear counter
	MoveSteps = true;	// set flag to indicate manual advance
	// configure Timer A
	TACCR0 = SteppingRateTMR;	// start the timer
	TACCR2 = SteppingRateTMR >> 1;	// now using count up
	// configure TimerB
	TBCCTL1 &= 0xFF1F;	// Clear OUTMODx bits; 3 MSB on lower
				// byte; Timer configured to output mode
	TBCCTL1 |= OUTMOD_4 + CCIE;
	TBCCR0 = TIME_32ms;	// set timer to 32ms
	TBCCR1 = TIME_32ms >> 1;
	// start timers
	TACTL |= MC_1;		// Start timer by selecting up mode
	TBCTL |= MC_1;
    }
    // Special case of updating speed during running
    // if the Desired Target Speed has been updated, then the stepper
    // speed should be increased or decreased
    // 
    if ((G_RUN_MOTOR == true)
	&& (DesiredStepperSpeed != DesiredTargetSpeed)) {
	if (DesiredTargetSpeed > DesiredStepperSpeed)
	    AccelerateState = ACCEL;
	else
	    AccelerateState = DECEL2NEWTARGET;
	// reset DesiredStepperSpeed
	DesiredStepperSpeed = DesiredTargetSpeed;
    }
}

// Stepper State machine
// This is used to move the motor

void
MotorStateMachine()
{
    // update the SteppingRateTMR if required
    // reduce the number of calculations
    if (UpdateSteppingRateTMR == true) {
	SteppingRateTMR = SMCLK_FREQ / SteppingRate;	// calculate new
							// value
	UpdateSteppingRateTMR = false;	// clear flag
    }
    switch (AccelerateState) {
    case (IDLE):
	if (G_RECIPROCATION == false && G_WAKE == high && G_ENABLE == high) {
	    G_MOTOR_CMD_ACTIVE1 = false;
	    G_MOTOR_CMD_ACTIVE2 = false;
	    G_MOTOR_CMD_ACTIVE3 = false;
	    G_ENBL_START_STOP = false;	// enable with false, allow Start
					// Stop
	    G_MANUAL_ADVANCE = false;	// clear the flags once stops
	    G_ENBL_RECIPROCATION_BUTTON = false;
	    G_RUN_MOTOR = false;
	    G_ENBL_MOVE_STEPS = false;
	    MoveSteps = false;	// clear the MoveSteps flag here
	    // reset the timers if the last command was run motor
	    if (LastCommandWasRunSteps == true) {
		TACTL &= ~MC_3;	// Stop timerA
		TBCTL &= ~MC_3;	// Stop timerB
	    }
	}
	break;
    case (ACCEL):
	{
	    // do nothing in ACCEL mode, wait for external input to change 
	    // state
	}
	break;
    case (DECEL):
	{
	    // do nothing in DECEL mode, wait for external input to change 
	    // state
	}
	break;
    case (RUN):
	{
	    // do nothing in RUN mode, wait for external input to change
	    // state
	}
	break;
    case (DECEL2NEWTARGET):
	{
	    // do nothing in DECEL mode, wait for external input to change 
	    // state
	}
    }
}
