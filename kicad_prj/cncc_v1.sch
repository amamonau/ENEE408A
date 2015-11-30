EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:LIB_cncc_v1
LIBS:cncc_v1-cache
EELAYER 25 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 11
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 8550 2550 900  700 
U 56526EA5
F0 "StepperDriverX" 60
F1 "stp_drv.sch" 60
F2 "VM" U R 9450 2700 60 
F3 "GND" U R 9450 2800 60 
F4 "STEP" I L 8550 2600 60 
F5 "DIR" I L 8550 2700 60 
F6 "EN" I L 8550 2800 60 
F7 "~SLEEP" I L 8550 2900 60 
F8 "VREF" I L 8550 3200 60 
F9 "~FAULT" O L 8550 3050 60 
$EndSheet
$Sheet
S 8550 3450 900  700 
U 5653D417
F0 "StepperDriverY" 60
F1 "stp_drv.sch" 60
F2 "VM" U R 9450 3600 60 
F3 "GND" U R 9450 3700 60 
F4 "STEP" I L 8550 3500 60 
F5 "DIR" I L 8550 3600 60 
F6 "EN" I L 8550 3700 60 
F7 "~SLEEP" I L 8550 3800 60 
F8 "VREF" I L 8550 4100 60 
F9 "~FAULT" O L 8550 3950 60 
$EndSheet
$Sheet
S 8550 4350 900  700 
U 56542384
F0 "StepperDriverZ" 60
F1 "stp_drv.sch" 60
F2 "VM" U R 9450 4500 60 
F3 "GND" U R 9450 4600 60 
F4 "STEP" I L 8550 4400 60 
F5 "DIR" I L 8550 4500 60 
F6 "EN" I L 8550 4600 60 
F7 "~SLEEP" I L 8550 4700 60 
F8 "VREF" I L 8550 5000 60 
F9 "~FAULT" O L 8550 4850 60 
$EndSheet
$Sheet
S 8550 5250 900  700 
U 5654C1EE
F0 "StepperDriverA" 60
F1 "stp_drv.sch" 60
F2 "VM" U R 9450 5400 60 
F3 "GND" U R 9450 5500 60 
F4 "STEP" I L 8550 5300 60 
F5 "DIR" I L 8550 5400 60 
F6 "EN" I L 8550 5500 60 
F7 "~SLEEP" I L 8550 5600 60 
F8 "VREF" I L 8550 5900 60 
F9 "~FAULT" O L 8550 5750 60 
$EndSheet
$Sheet
S 6900 3300 900  750 
U 5653A94A
F0 "BrushedDCMotorDriver" 60
F1 "dc_motor_drv.sch" 60
F2 "IN1" I L 6900 3350 60 
F3 "IN2" I L 6900 3450 60 
F4 "~SLEEP" I L 6900 3550 60 
F5 "~FAULT" O L 6900 3650 60 
F6 "SNSOUT" O L 6900 3750 60 
F7 "SDA" B L 6900 3900 60 
F8 "SCL" I L 6900 4000 60 
F9 "VM" U R 7800 3450 60 
F10 "VDD" U R 7800 3550 60 
F11 "GND" U R 7800 3650 60 
F12 "SO" O R 7800 3900 60 
F13 "+5V_REF" O R 7800 4000 60 
$EndSheet
$Sheet
S 6900 4250 900  1800
U 5659B5AB
F0 "SensorsBreakOut" 60
F1 "sns_sch.sch" 60
F2 "VS" U R 7800 4600 60 
F3 "VE" U R 7800 4700 60 
F4 "GND" U R 7800 4800 60 
F5 "X_L+" O L 6900 4300 60 
F6 "X_L-" O L 6900 4400 60 
F7 "X_HM" O L 6900 4500 60 
F8 "Y_LM+" O L 6900 4600 60 
F9 "Y_LM-" O L 6900 4700 60 
F10 "Y_HM" O L 6900 4800 60 
F11 "Z_LM+" O L 6900 4900 60 
F12 "Z_LM-" O L 6900 5000 60 
F13 "Z_HM" O L 6900 5100 60 
F14 "A_LM+" O L 6900 5200 60 
F15 "A_LM-" O L 6900 5300 60 
F16 "A_HM" O L 6900 5400 60 
F17 "E_A" O L 6900 5550 60 
F18 "E_B" O L 6900 5650 60 
F19 "E_IN" O L 6900 5750 60 
F20 "+5V_REF" I R 7800 4300 60 
F21 "SDA" B L 6900 5900 60 
F22 "SCL" I L 6900 6000 60 
F23 "CH7_IN" I R 7800 4400 60 
$EndSheet
$Sheet
S 6900 2400 900  500 
U 565650ED
F0 "DAC" 60
F1 "dac_out.sch" 60
F2 "VDD" U L 6900 2450 60 
F3 "GND" U L 6900 2550 60 
F4 "SDA" B L 6900 2750 60 
F5 "SCL" I L 6900 2850 60 
F6 "OUT" O R 7800 2550 60 
$EndSheet
$Sheet
S 4700 1550 1250 2200
U 565A3966
F0 "Isolation" 60
F1 "iso.sch" 60
F2 "STEP[0..3]" O R 5950 1900 60 
F3 "DIR[0..3]" O R 5950 2000 60 
F4 "EN[0..3]" O R 5950 2100 60 
F5 "SLEEP[0..3]" O R 5950 2200 60 
F6 "FAULT[0..3]" I R 5950 2300 60 
F7 "DSTP[0..3]" I L 4700 1900 60 
F8 "DDIR[0..3]" I L 4700 2000 60 
F9 "DEN[0..3]" I L 4700 2100 60 
F10 "DSLP[0..3]" I L 4700 2200 60 
F11 "DFLT[0..3]" O L 4700 2300 60 
F12 "IN2" O R 5950 2600 60 
F13 "IN1" O R 5950 2500 60 
F14 "EN" O R 5950 2700 60 
F15 "DFLT" O L 4700 2800 60 
F16 "DSOUT" O L 4700 2900 60 
F17 "DIN1" I L 4700 2500 60 
F18 "DIN2" I L 4700 2600 60 
F19 "DEN" I L 4700 2700 60 
F20 "FAULT" I R 5950 2800 60 
F21 "SOUT" I R 5950 2900 60 
F22 "SDA" B R 5950 3100 60 
F23 "SCL" O R 5950 3200 60 
F24 "DSDA" B L 4700 3100 60 
F25 "DSCL" I L 4700 3200 60 
F26 "DSCL2" I L 4700 3400 60 
F27 "DSDA2" B L 4700 3300 60 
F28 "DO[0..3]" I L 4700 3600 60 
F29 "DI[0..3]" O L 4700 3700 60 
F30 "VDD_LP" U L 4700 1600 60 
F31 "VDD_HP" U R 5950 1600 60 
F32 "GND_LP" U L 4700 1700 60 
F33 "GND_HP" U R 5950 1700 60 
$EndSheet
Wire Wire Line
	7800 4000 8050 4000
Wire Wire Line
	8050 4000 8050 4300
Wire Wire Line
	8050 4300 7800 4300
Wire Wire Line
	7800 3900 8100 3900
Wire Wire Line
	8100 3900 8100 4400
Wire Wire Line
	8100 4400 7800 4400
$Sheet
S 4700 3950 1250 2350
U 565FD8F0
F0 "IsolationInputs&SPIBus" 60
F1 "iso_spi.sch" 60
F2 "DSS[0..4]" I L 4700 6250 60 
F3 "DSCK" I L 4700 6150 60 
F4 "DMISO" O L 4700 6050 60 
F5 "DMOSI" I L 4700 5950 60 
F6 "VDD_LP" U L 4700 4000 60 
F7 "GND_LP" U L 4700 4100 60 
F8 "VDD_HP" U R 5950 4000 60 
F9 "GND_HP" U R 5950 4100 60 
F10 "DX_L+" O L 4700 4300 60 
F11 "DX_L-" O L 4700 4400 60 
F12 "DX_HM" O L 4700 4500 60 
F13 "DY_LM+" O L 4700 4600 60 
F14 "DY_LM-" O L 4700 4700 60 
F15 "DY_HM" O L 4700 4800 60 
F16 "DZ_LM+" O L 4700 4900 60 
F17 "DZ_LM-" O L 4700 5000 60 
F18 "DZ_HM" O L 4700 5100 60 
F19 "DA_LM+" O L 4700 5200 60 
F20 "DA_LM-" O L 4700 5300 60 
F21 "DA_HM" O L 4700 5400 60 
F22 "DE_A" O L 4700 5550 60 
F23 "DE_B" O L 4700 5650 60 
F24 "DE_IN" O L 4700 5750 60 
F25 "X_L+" I R 5950 4300 60 
F26 "X_L-" I R 5950 4400 60 
F27 "X_HM" I R 5950 4500 60 
F28 "Y_LM+" I R 5950 4600 60 
F29 "Y_LM-" I R 5950 4700 60 
F30 "Y_HM" I R 5950 4800 60 
F31 "Z_LM+" I R 5950 4900 60 
F32 "Z_LM-" I R 5950 5000 60 
F33 "Z_HM" I R 5950 5100 60 
F34 "A_LM+" I R 5950 5200 60 
F35 "A_LM-" I R 5950 5300 60 
F36 "A_HM" I R 5950 5400 60 
F37 "E_A" I R 5950 5550 60 
F38 "E_B" I R 5950 5650 60 
F39 "E_IN" I R 5950 5750 60 
$EndSheet
$Sheet
S 6600 700  1250 850 
U 565E910C
F0 "PowerInput" 60
F1 "pow_in.sch" 60
$EndSheet
$EndSCHEMATC
