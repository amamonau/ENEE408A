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
/*
 * Pragmas.c
 *
 *  Created on: Apr 14, 2015
 *      Author: a0271951
 */

#include "utility.h"


// pragma declarations to force variable to specific locations

#pragma DATA_SECTION(G_FIRMWARE_VERSION,".mysect01")
float           G_FIRMWARE_VERSION;

#pragma DATA_SECTION(G_IFS,".mysect02")
float           G_IFS;

#pragma DATA_SECTION(G_IRSENSE,".mysect03")
float           G_IRSENSE;

#pragma DATA_SECTION(G_VREF_VAL,".mysect04")
unsigned long   G_VREF_VAL;

#pragma DATA_SECTION(G_ACCEL_RATE,".mysect05")
unsigned int    G_ACCEL_RATE;

#pragma DATA_SECTION(G_DECAY_MODE,".mysect06")
unsigned int    G_DECAY_MODE;

#pragma DATA_SECTION(G_NUM_STEPS,".mysect07")
unsigned int    G_NUM_STEPS;

#pragma DATA_SECTION(G_PWM_OFF_TIME,".mysect08")
unsigned int    G_PWM_OFF_TIME;

#pragma DATA_SECTION(G_STARTING_SPEED,".mysect09")
unsigned int    G_STARTING_SPEED;

#pragma DATA_SECTION(G_STEPS_2_STOP,".mysect10")
unsigned int    G_STEPS_2_STOP;

#pragma DATA_SECTION(G_STEP_MODE,".mysect11")
unsigned int    G_STEP_MODE;

#pragma DATA_SECTION(G_STOPPING_SPEED,".mysect12")
unsigned int    G_STOPPING_SPEED;

#pragma DATA_SECTION(G_TARGET_SPEED,".mysect13")
unsigned int    G_TARGET_SPEED;

#pragma DATA_SECTION(G_TORQUE,".mysect14")
unsigned int    G_TORQUE;

#pragma DATA_SECTION(G_DIR,".mysect15")
gpio            G_DIR;

#pragma DATA_SECTION(G_ENABLE,".mysect16")
gpio            G_ENABLE;

#pragma DATA_SECTION(G_ENABLE_MTR_CONTROL,".mysect17")
boolean         G_ENABLE_MTR_CONTROL;

#pragma DATA_SECTION(G_ENBL_MOVE_STEPS,".mysect18")
boolean         G_ENBL_MOVE_STEPS;

#pragma DATA_SECTION(G_ENBL_START_STOP,".mysect19")
boolean         G_ENBL_START_STOP;

#pragma DATA_SECTION(G_FAULT,".mysect20")
gpio            G_FAULT;

#pragma DATA_SECTION(G_MANUAL_ADVANCE,".mysect21")
boolean         G_MANUAL_ADVANCE;

#pragma DATA_SECTION(G_MOTOR_CMD_ACTIVE1,".mysect22")
boolean         G_MOTOR_CMD_ACTIVE1;

#pragma DATA_SECTION(G_MOTOR_CMD_ACTIVE2,".mysect23")
boolean         G_MOTOR_CMD_ACTIVE2;

#pragma DATA_SECTION(G_ADEC,".mysect24")
gpio            G_ADEC;

#pragma DATA_SECTION(G_RUN_MOTOR,".mysect25")
boolean         G_RUN_MOTOR;

#pragma DATA_SECTION(G_WAKE,".mysect26")
gpio            G_WAKE;

#pragma DATA_SECTION(G_MOTOR_CMD_ACTIVE3,".mysect27")
boolean         G_MOTOR_CMD_ACTIVE3;

#pragma DATA_SECTION(G_RECIPROCATION,".mysect28")
boolean         G_RECIPROCATION;

#pragma DATA_SECTION(G_ENBL_RECIPROCATION_BUTTON,".mysect29")
boolean         G_ENBL_RECIPROCATION_BUTTON;

#pragma DATA_SECTION(G_IDLE,".mysect30")
boolean         G_IDLE;
