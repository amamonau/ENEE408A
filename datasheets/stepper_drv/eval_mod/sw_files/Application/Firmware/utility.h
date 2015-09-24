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
 *  utility.h - Definition file for utility functions
 *  DRV8880EVM_FIRMWARE
 *  1/30/2015
 *
 ******************************************************************************/

#ifndef DRV8880EVM_V1P1_UTILITY_H_
#define DRV8880EVM_V1P1_UTILITY_H_

/*****************************************************************************/

// change to reflect firmware revision
#define FW_VERSION   1.1	// Version 1.1

#define LED_TIMEOUT 10000	// Count to toggle LED
#define SMCLK_FREQ  8000000	// SMCLK frequency in Hz
#define TIME_32ms	31250	// TimerA interrupt value, (31.25ms or 32
				// updates/s)

// IFS defines
// this are used to calculate the chopping current
#define V3P3_TIMES_GAIN			0.5	// 3.3 divide by 6.6, use
						// constant
#define VREF_BIT				0.0009775	// 1/1023
								// per
								// step
#define TORQUE_STEP				0.25	// 1/4 per step

// 

#define	DEFAULT_START_SPEED		200	// initial speed in pulses 
						// per second when
						// starting motor
#define DEFAULT_TARGET_SPEED    1000	// speed of motor once running
#define DEFAULT_ACCEL_RATE		50	// acceleration rate from
						// starting speed to
						// target speed
#define DEFAULT_STOPPING_SPEED  200	// speed to stop motor once
					// deceleration has reached this
					// point
#define DEFAULT_NUM_STEPS		1	// number of steps to move 
						// when manually stepping
#define DEFAULT_STEPS_2_STOP 	1	// number of steps used to stop
					// the motor when manually
					// stepping
#define	DEFAULT_DECAY			6	// Mixed 1tblank / Mixed 1 
						// tblank (use index value 
						// 0 - 8)
#define	DEFAULT_STEP_MODE		3	// 8 rising edge only (use 
						// index value 0 - 8)
#define	DEFAULT_TORQUE			0	// 100% (use index value 0 
						// - 3)
#define	DEFAULT_PWM_OFF_TIME	0	// 20us off (use index value 0 -
					// 2)
#define	DEFAULT_IFS				0
#define	DEFAULT_SENSE_RESISTOR	0.25
#define	DEFAULT_VREF_VAL		512

#define use_one_edge			1	// multiplies all step
						// calculations by 2
											// this 
											// is 
											// legacy 
											// from 
											// the 
											// DRV8846

// defines for motor state machine
#define     IDLE        	1
#define     ACCEL       	2
#define     DECEL       	4
#define     RUN         	8
#define		DECEL2NEWTARGET	16

#define		ACCEL_RATE_COUNT 2000

// GPIO Port 2 Definitions
#define DECAY1		BIT0	// P2.0
#define DECAY0		BIT1	// P2.1
#define STEP		BIT4	// P2.4
#define TOFF		BIT7	// P2.7
#define TRQ1		BIT2	// P2.2
#define TRQ0		BIT5	// P2.5
#define USM1		BIT3	// P2.3
#define USM0		BIT6	// P2.6

#define CAPD_DECAY1	BIT0
#define CAPD_DECAY0	BIT1
#define CAPD_TRQ1	BIT2
#define CAPD_TRQ0	BIT5
#define CAPD_TOFF	BIT7
#define CAPD_USM1	BIT3
#define CAPD_USM0	BIT6

// GPIO Port 4 Definitions
#define WAKE		BIT5	// P4.5
#define DIR			BIT2	// P4.2
#define ENABLE		BIT1	// P4.1
#define FAULT		BIT6	// P4.6
#define V3P3_GPIO   BIT7	// P4.7

// GPIO Port 6 Definitions
#define ADEC		BIT6	// P6.6
#define STATUS		BIT5	// P6.5

// 
// defines to make code more readable, and easier to fix
// 

#define toggle_status_led 		P6OUT ^=  STATUS

#define set_ADEC_hi				P6OUT |=  ADEC
#define set_ADEC_lo				P6OUT &= ~ADEC

#define set_DECAY1_hi			P2OUT |=  DECAY1
#define set_DECAY1_lo			P2OUT &= ~DECAY1
#define set_DECAY0_hi			P2OUT |=  DECAY0
#define set_DECAY0_lo			P2OUT &= ~DECAY0

#define set_DIR_hi				P4OUT |=  DIR
#define set_DIR_lo				P4OUT &= ~DIR

#define set_ENABLE_hi			P4OUT |=  ENABLE
#define set_ENABLE_lo			P4OUT &= ~ENABLE

#define enable_nFAULT_pullup	P4REN |=  FAULT; P4OUT |= FAULT
#define disable_nFAULT_pullup	P4REN &= ~FAULT; P4OUT &= ~FAULT

#define set_TOFF_hi				P2OUT |=  TOFF
#define set_TOFF_lo				P2OUT &= ~TOFF

#define set_TRQ1_hi				P2OUT |=  TRQ1
#define set_TRQ1_lo				P2OUT &= ~TRQ1
#define set_TRQ0_hi				P2OUT |=  TRQ0
#define set_TRQ0_lo				P2OUT &= ~TRQ0

#define set_USM1_hi				P2OUT |=  USM1
#define set_USM1_lo				P2OUT &= ~USM1
#define set_USM0_hi				P2OUT |=  USM0
#define set_USM0_lo				P2OUT &= ~USM0

#define set_WAKE_hi				P4OUT |=  WAKE
#define set_WAKE_lo				P4OUT &= ~WAKE

#define enable_V3P3_GPIO		P4OUT |= V3P3_GPIO; P4DIR |=  V3P3_GPIO
#define disable_V3P3_GPIO		P4DIR &= ~V3P3_GPIO

// enable/disable output

#define enable_DECAY1_output	P2DIR |=  DECAY1
#define disable_DECAY1_output	P2DIR &= ~DECAY1
#define enable_DECAY0_output	P2DIR |=  DECAY0
#define disable_DECAY0_output	P2DIR &= ~DECAY0

#define disable_DECAY1_input	CAPD  |=  CAPD_DECAY1
#define enable_DECAY1_input		CAPD  &= ~CAPD_DECAY1
#define disable_DECAY0_input	CAPD  |=  CAPD_DECAY0
#define enable_DECAY0_input		CAPD  &= ~CAPD_DECAY0

#define enable_TOFF_output		P2DIR |=  TOFF
#define disable_TOFF_output		P2DIR &= ~TOFF

#define disable_TOFF_input		CAPD  |=  CAPD_TOFF
#define enable_TOFF_input		CAPD  &= ~CAPD_TOFF

#define enable_USM1_output		P2DIR |=  USM1
#define disable_USM1_output		P2DIR &= ~USM1
#define enable_USM0_output		P2DIR |=  USM0
#define disable_USM0_output		P2DIR &= ~USM0

#define disable_USM1_input		CAPD  |=  CAPD_USM1
#define enable_USM1_input		CAPD  &= ~CAPD_USM1
#define disable_USM0_input		CAPD  |=  CAPD_USM0
#define enable_USM0_input		CAPD  &= ~CAPD_USM0

#define read_FAULT_pin			(P4IN & FAULT)

#define disable_ENABLE_output   P4DIR &= ~ENABLE
#define enable_ENABLE_output    P4DIR |= ENABLE

// Custom Types
typedef enum { false, true } boolean;
typedef enum { low, high } gpio;

/*****************************************************************************/

// Declare Global Variables

// ***********************************************************************************
// Global Variables for GUI Composer *
// ***********************************************************************************

extern boolean  G_ENBL_START_STOP;	// Allow Motor to Move 0 -
					// Disabled 1 - Enabled
extern boolean  G_ENBL_MOVE_STEPS;	// Allow Motor to Move 0 -
					// Disabled 1 - Enabled
extern boolean  G_MOTOR_CMD_ACTIVE1;	// Motor in Motion (Auto) 0 - No
					// Motion 1 - In Motion
extern boolean  G_MOTOR_CMD_ACTIVE2;	// Motor in Motion (Man) 0 - No
					// Motion 1 - In Motion
extern boolean  G_MOTOR_CMD_ACTIVE3;
extern boolean  G_RUN_MOTOR;	// Auto run enabled
extern boolean  G_MANUAL_ADVANCE;	// Manually advance motor
extern boolean  G_ENABLE_MTR_CONTROL;	// message for user to select
					// enable and wake
extern boolean  G_RECIPROCATION;
extern boolean  G_ENBL_RECIPROCATION_BUTTON;
extern boolean  G_IDLE;		// AccelerateState = 1

// GPIO
extern gpio     G_WAKE;		// Wake device 0 - Sleep 1 - Wake
extern gpio     G_DIR;		// Direction 0 - Forward 1 - Reverse
extern gpio     G_ENABLE;	// Enable device 0 - Disabled 1 - Enabled
extern gpio     G_FAULT;	// Fault Status 0 - Fault 1 - OK
extern gpio     G_ADEC;		// Unused Spare 0 - Default 1 - Unused

// PWM Generation
extern unsigned int G_DECAY_MODE;	// Decay Mode (1 of 9)
extern unsigned int G_STEP_MODE;	// Step Mode (1 of 9)
extern unsigned int G_TORQUE;	// Torque (1 of 4)
extern unsigned int G_PWM_OFF_TIME;	// PWM off time (1 of 3)

extern unsigned int G_STARTING_SPEED;	// initial speed in pulses per
					// second when starting motor
extern unsigned int G_TARGET_SPEED;	// speed of motor once running
extern unsigned int G_ACCEL_RATE;	// acceleration rate from starting 
					// speed to target speed
extern unsigned int G_STOPPING_SPEED;	// speed to stop motor once
					// deceleration has reached this
					// point
extern unsigned int G_NUM_STEPS;	// number of steps to move when
					// manually stepping
extern unsigned int G_STEPS_2_STOP;	// number of steps used to stop
					// the motor when manually
					// stepping
extern unsigned long int G_VREF_VAL;	// scale for current

extern float    G_IFS;		// Full scale current
extern float    G_IRSENSE;	// sense value resistor
extern float    G_FIRMWARE_VERSION;	// keep track of firmware

// ///////////////////////
// variables for utility
// ///////////////////////

// variable for the motor state machine
unsigned int    AccelerateState;
unsigned long int SteppingRateTMR;
unsigned long int SteppingRate;
unsigned long int SteppingRateUpdate;
unsigned long int DesiredStepperSpeed;
unsigned long int DesiredTargetSpeed;
unsigned long int StartingSpeed;
unsigned long int StoppingSpeed;
unsigned long int AccelRate;
unsigned long int AccelTimeBase;
unsigned long int AccelerationIncrease;
unsigned long int StepsToStop;
unsigned long int StepsToMove;
unsigned long int tmpStepsToMove;
unsigned long int PriorSteppingRate;

unsigned int    PriorDecayMode;	// return to previous decay mode
											// when 
											// exiting 
											// autodecay
boolean         Block_GUI_FAULT_button;
boolean         set_ENABLE_as_opendrain;
boolean         temp_delay;
boolean         temp_reciprocation;
boolean         ForceUpdate;
boolean         LastCommandWasRunSteps;
boolean         MoveSteps;
boolean         UpdateChoppingCurrent;
boolean         UpdateSteppingRateTMR;
gpio            tmp_adec;

float           torque_multiplier;
float           vref_multiplier;
float           step_multiplier;
float           irsense_multiplier;
float           time;
float           time_squared;

unsigned int    last_reciprocation_state;	// reciprocation flag, =1
						// last action is
						// reciprocation

unsigned int    LED_TOGGLE_CNT;	// count to determine if LED should be
				// toggled
unsigned long int Steps2Target;	// number of steps to reach target speed
										// from 
										// starting 
										// speed 
										// using 
										// acceleration 
										// rate
unsigned long int Steps2Stop;	// number of steps to stop
										// from 
										// target 
										// speed 
										// using 
										// acceleration 
										// rate
unsigned long int Steps2Decel;	// step number to change from accel to
				// decel
										// in 
										// manual 
										// mode
unsigned int    delay_cycles;


/*****************************************************************************/

// Function Declarations

// Main Functions
void            Initialize();
void            UpdateDeviceControls();
void            UpdateGUIControls();
void            UpdateRegisters();
void            UpdateSpeed();

// GPIO/GUI Functions
void            SetDefaultGUIVals();
void            SetDefaultGPIOVals();

// Motor Functions
void            DetermineMotorMovement();
void            MotorStateMachine();

#endif				/* DRV8880EVM_V1P1_UTILITY_H_ */
