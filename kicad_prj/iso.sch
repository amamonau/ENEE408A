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
Sheet 9 10
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ISO7140FCC U?
U 1 1 565BF0C9
P 2550 1150
AR Path="/565BF0C9" Ref="U?"  Part="1" 
AR Path="/565A3966/565BF0C9" Ref="U?"  Part="1" 
F 0 "U?" H 2350 650 60  0000 C CNN
F 1 "ISO7140FCC" H 2550 750 60  0000 C CNN
F 2 "" H 2550 1150 60  0000 C CNN
F 3 "" H 2550 1150 60  0000 C CNN
	1    2550 1150
	1    0    0    -1  
$EndComp
$Comp
L ISO7140FCC U?
U 1 1 565BF19D
P 2550 2350
AR Path="/565BF19D" Ref="U?"  Part="1" 
AR Path="/565A3966/565BF19D" Ref="U?"  Part="1" 
F 0 "U?" H 2350 1850 60  0000 C CNN
F 1 "ISO7140FCC" H 2550 1950 60  0000 C CNN
F 2 "" H 2550 2350 60  0000 C CNN
F 3 "" H 2550 2350 60  0000 C CNN
	1    2550 2350
	1    0    0    -1  
$EndComp
$Comp
L ISO7140FCC U?
U 1 1 565BF25F
P 2550 3550
AR Path="/565BF25F" Ref="U?"  Part="1" 
AR Path="/565A3966/565BF25F" Ref="U?"  Part="1" 
F 0 "U?" H 2350 3050 60  0000 C CNN
F 1 "ISO7140FCC" H 2550 3150 60  0000 C CNN
F 2 "" H 2550 3550 60  0000 C CNN
F 3 "" H 2550 3550 60  0000 C CNN
	1    2550 3550
	1    0    0    -1  
$EndComp
$Comp
L ISO7140FCC U?
U 1 1 565BF2C5
P 2550 4750
AR Path="/565BF2C5" Ref="U?"  Part="1" 
AR Path="/565A3966/565BF2C5" Ref="U?"  Part="1" 
F 0 "U?" H 2350 4250 60  0000 C CNN
F 1 "ISO7140FCC" H 2550 4350 60  0000 C CNN
F 2 "" H 2550 4750 60  0000 C CNN
F 3 "" H 2550 4750 60  0000 C CNN
	1    2550 4750
	1    0    0    -1  
$EndComp
$Comp
L ISO7140FCC U?
U 1 1 565BF357
P 2550 5950
AR Path="/565BF357" Ref="U?"  Part="1" 
AR Path="/565A3966/565BF357" Ref="U?"  Part="1" 
F 0 "U?" H 2750 5450 60  0000 C CNN
F 1 "ISO7140FCC" H 2550 5550 60  0000 C CNN
F 2 "" H 2550 5950 60  0000 C CNN
F 3 "" H 2550 5950 60  0000 C CNN
	1    2550 5950
	-1   0    0    -1  
$EndComp
Entry Wire Line
	3450 750  3350 850 
Entry Wire Line
	3450 850  3350 950 
Entry Wire Line
	3450 950  3350 1050
Entry Wire Line
	3450 1050 3350 1150
Text Label 3050 850  0    60   ~ 0
STEP0
Text Label 3450 700  0    60   ~ 0
STEP[0..3]
Text Label 3050 950  0    60   ~ 0
STEP1
Text Label 3050 1050 0    60   ~ 0
STEP2
Text Label 3050 1150 0    60   ~ 0
STEP3
Text HLabel 3950 700  2    60   Output ~ 0
STEP[0..3]
Entry Wire Line
	3450 1950 3350 2050
Entry Wire Line
	3450 2050 3350 2150
Entry Wire Line
	3450 2150 3350 2250
Entry Wire Line
	3450 2250 3350 2350
Text Label 3050 2050 0    60   ~ 0
DIR0
Text Label 3450 1900 0    60   ~ 0
DIR[0..3]
Text Label 3050 2150 0    60   ~ 0
DIR1
Text Label 3050 2250 0    60   ~ 0
DIR2
Text Label 3050 2350 0    60   ~ 0
DIR3
Text HLabel 3950 1900 2    60   Output ~ 0
DIR[0..3]
Entry Wire Line
	3450 3150 3350 3250
Entry Wire Line
	3450 3250 3350 3350
Entry Wire Line
	3450 3350 3350 3450
Entry Wire Line
	3450 3450 3350 3550
Text Label 3050 3250 0    60   ~ 0
EN0
Text Label 3450 3100 0    60   ~ 0
EN[0..3]
Text Label 3050 3350 0    60   ~ 0
EN1
Text Label 3050 3450 0    60   ~ 0
EN2
Text Label 3050 3550 0    60   ~ 0
EN3
Text HLabel 3950 3100 2    60   Output ~ 0
EN[0..3]
Entry Wire Line
	3450 4350 3350 4450
Entry Wire Line
	3450 4450 3350 4550
Entry Wire Line
	3450 4550 3350 4650
Entry Wire Line
	3450 4650 3350 4750
Text Label 3050 4450 0    60   ~ 0
SLEEP0
Text Label 3450 4300 0    60   ~ 0
SLEEP[0..3]
Text Label 3050 4550 0    60   ~ 0
SLEEP1
Text Label 3050 4650 0    60   ~ 0
SLEEP2
Text Label 3050 4750 0    60   ~ 0
SLEEP3
Text HLabel 3950 4300 2    60   Output ~ 0
SLEEP[0..3]
Entry Wire Line
	3450 5550 3350 5650
Entry Wire Line
	3450 5650 3350 5750
Entry Wire Line
	3450 5750 3350 5850
Entry Wire Line
	3450 5850 3350 5950
Text Label 3050 5650 0    60   ~ 0
FAULT0
Text Label 3450 5500 0    60   ~ 0
FAULT[0..3]
Text Label 3050 5750 0    60   ~ 0
FAULT1
Text Label 3050 5850 0    60   ~ 0
FAULT2
Text Label 3050 5950 0    60   ~ 0
FAULT3
Text HLabel 3950 5500 2    60   Input ~ 0
FAULT[0..3]
Entry Wire Line
	1650 750  1750 850 
Entry Wire Line
	1650 850  1750 950 
Entry Wire Line
	1650 950  1750 1050
Entry Wire Line
	1650 1050 1750 1150
Text Label 2050 850  2    60   ~ 0
DSTP0
Text Label 1650 700  2    60   ~ 0
DSTP[0..3]
Text Label 2050 950  2    60   ~ 0
DSTP1
Text Label 2050 1050 2    60   ~ 0
DSTP2
Text Label 2050 1150 2    60   ~ 0
DSTP3
Text HLabel 1150 700  0    60   Input ~ 0
DSTP[0..3]
Entry Wire Line
	1650 1950 1750 2050
Entry Wire Line
	1650 2050 1750 2150
Entry Wire Line
	1650 2150 1750 2250
Entry Wire Line
	1650 2250 1750 2350
Text Label 2050 2050 2    60   ~ 0
DDIR0
Text Label 1650 1900 2    60   ~ 0
DDIR[0..3]
Text Label 2050 2150 2    60   ~ 0
DDIR1
Text Label 2050 2250 2    60   ~ 0
DDIR2
Text Label 2050 2350 2    60   ~ 0
DDIR3
Text HLabel 1150 1900 0    60   Input ~ 0
DDIR[0..3]
Entry Wire Line
	1650 3150 1750 3250
Entry Wire Line
	1650 3250 1750 3350
Entry Wire Line
	1650 3350 1750 3450
Entry Wire Line
	1650 3450 1750 3550
Text Label 1650 3100 2    60   ~ 0
DEN[0..3]
Text HLabel 1150 3100 0    60   Input ~ 0
DEN[0..3]
Entry Wire Line
	1650 4350 1750 4450
Entry Wire Line
	1650 4450 1750 4550
Entry Wire Line
	1650 4550 1750 4650
Entry Wire Line
	1650 4650 1750 4750
Text Label 2050 4450 2    60   ~ 0
DSLP0
Text Label 1650 4300 2    60   ~ 0
DSLP[0..3]
Text Label 2050 4550 2    60   ~ 0
DSLP1
Text Label 2050 4650 2    60   ~ 0
DSLP2
Text Label 2050 4750 2    60   ~ 0
DSLP3
Text HLabel 1150 4300 0    60   Input ~ 0
DSLP[0..3]
Entry Wire Line
	1650 5550 1750 5650
Entry Wire Line
	1650 5650 1750 5750
Entry Wire Line
	1650 5750 1750 5850
Entry Wire Line
	1650 5850 1750 5950
Text Label 2050 5650 2    60   ~ 0
DFLT0
Text Label 1650 5500 2    60   ~ 0
DFLT[0..3]
Text Label 2050 5750 2    60   ~ 0
DFLT1
Text Label 2050 5850 2    60   ~ 0
DFLT2
Text Label 2050 5950 2    60   ~ 0
DFLT3
Text HLabel 1150 5500 0    60   Output ~ 0
DFLT[0..3]
Text Label 2050 3250 2    60   ~ 0
DEN0
Text Label 2050 3350 2    60   ~ 0
DEN1
Text Label 2050 3450 2    60   ~ 0
DEN2
Text Label 2050 3550 2    60   ~ 0
DEN3
$Comp
L GND #PWR?
U 1 1 565908F2
P 3100 1500
F 0 "#PWR?" H 3100 1250 50  0001 C CNN
F 1 "GND" H 3100 1350 50  0000 C CNN
F 2 "" H 3100 1500 60  0000 C CNN
F 3 "" H 3100 1500 60  0000 C CNN
	1    3100 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56590963
P 3100 2700
F 0 "#PWR?" H 3100 2450 50  0001 C CNN
F 1 "GND" H 3100 2550 50  0000 C CNN
F 2 "" H 3100 2700 60  0000 C CNN
F 3 "" H 3100 2700 60  0000 C CNN
	1    3100 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565909AF
P 3100 3900
F 0 "#PWR?" H 3100 3650 50  0001 C CNN
F 1 "GND" H 3100 3750 50  0000 C CNN
F 2 "" H 3100 3900 60  0000 C CNN
F 3 "" H 3100 3900 60  0000 C CNN
	1    3100 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56590A55
P 3100 5100
F 0 "#PWR?" H 3100 4850 50  0001 C CNN
F 1 "GND" H 3100 4950 50  0000 C CNN
F 2 "" H 3100 5100 60  0000 C CNN
F 3 "" H 3100 5100 60  0000 C CNN
	1    3100 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56590AAB
P 3100 6300
F 0 "#PWR?" H 3100 6050 50  0001 C CNN
F 1 "GND" H 3100 6150 50  0000 C CNN
F 2 "" H 3100 6300 60  0000 C CNN
F 3 "" H 3100 6300 60  0000 C CNN
	1    3100 6300
	1    0    0    -1  
$EndComp
Text Label 6500 700  0    60   ~ 0
+5V
Text Label 3050 1250 0    60   ~ 0
+5V
Text Label 3050 1900 0    60   ~ 0
+5V
Text Label 3050 2450 0    60   ~ 0
+5V
Text Label 3050 3100 0    60   ~ 0
+5V
Text Label 3050 3650 0    60   ~ 0
+5V
Text Label 3050 4300 0    60   ~ 0
+5V
Text Label 3050 4850 0    60   ~ 0
+5V
Text Label 3050 5500 0    60   ~ 0
+5V
Text Label 1800 6050 0    60   ~ 0
+3.3V
Text Label 1800 5500 0    60   ~ 0
+3.3V
Text Label 1800 4300 0    60   ~ 0
+3.3V
Text Label 1800 3100 0    60   ~ 0
+3.3V
Text Label 1800 1900 0    60   ~ 0
+3.3V
Text Label 1800 700  0    60   ~ 0
+3.3V
$Comp
L GNDD #PWR?
U 1 1 56592391
P 2000 1500
F 0 "#PWR?" H 2000 1250 50  0001 C CNN
F 1 "GNDD" H 2000 1350 50  0000 C CNN
F 2 "" H 2000 1500 60  0000 C CNN
F 3 "" H 2000 1500 60  0000 C CNN
	1    2000 1500
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 56592411
P 2000 2700
F 0 "#PWR?" H 2000 2450 50  0001 C CNN
F 1 "GNDD" H 2000 2550 50  0000 C CNN
F 2 "" H 2000 2700 60  0000 C CNN
F 3 "" H 2000 2700 60  0000 C CNN
	1    2000 2700
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 56592467
P 2000 3900
F 0 "#PWR?" H 2000 3650 50  0001 C CNN
F 1 "GNDD" H 2000 3750 50  0000 C CNN
F 2 "" H 2000 3900 60  0000 C CNN
F 3 "" H 2000 3900 60  0000 C CNN
	1    2000 3900
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 565924D1
P 2000 5100
F 0 "#PWR?" H 2000 4850 50  0001 C CNN
F 1 "GNDD" H 2000 4950 50  0000 C CNN
F 2 "" H 2000 5100 60  0000 C CNN
F 3 "" H 2000 5100 60  0000 C CNN
	1    2000 5100
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 565924EB
P 2000 6300
F 0 "#PWR?" H 2000 6050 50  0001 C CNN
F 1 "GNDD" H 2000 6150 50  0000 C CNN
F 2 "" H 2000 6300 60  0000 C CNN
F 3 "" H 2000 6300 60  0000 C CNN
	1    2000 6300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565931E4
P 1250 6950
F 0 "C?" H 1275 7050 50  0000 L CNN
F 1 "0.1uF" H 1275 6850 50  0000 L CNN
F 2 "" H 1288 6800 30  0000 C CNN
F 3 "" H 1250 6950 60  0000 C CNN
	1    1250 6950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56593BC5
P 1500 6950
F 0 "C?" H 1525 7050 50  0000 L CNN
F 1 "0.1uF" H 1525 6850 50  0000 L CNN
F 2 "" H 1538 6800 30  0000 C CNN
F 3 "" H 1500 6950 60  0000 C CNN
	1    1500 6950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56593BF4
P 1750 6950
F 0 "C?" H 1775 7050 50  0000 L CNN
F 1 "0.1uF" H 1775 6850 50  0000 L CNN
F 2 "" H 1788 6800 30  0000 C CNN
F 3 "" H 1750 6950 60  0000 C CNN
	1    1750 6950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56593C22
P 2000 6950
F 0 "C?" H 2025 7050 50  0000 L CNN
F 1 "0.1uF" H 2025 6850 50  0000 L CNN
F 2 "" H 2038 6800 30  0000 C CNN
F 3 "" H 2000 6950 60  0000 C CNN
	1    2000 6950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56593C53
P 2250 6950
F 0 "C?" H 2275 7050 50  0000 L CNN
F 1 "0.1uF" H 2275 6850 50  0000 L CNN
F 2 "" H 2288 6800 30  0000 C CNN
F 3 "" H 2250 6950 60  0000 C CNN
	1    2250 6950
	1    0    0    -1  
$EndComp
Text Label 1100 6700 1    60   ~ 0
+3.3V
$Comp
L GNDD #PWR?
U 1 1 56594C32
P 1100 7200
F 0 "#PWR?" H 1100 6950 50  0001 C CNN
F 1 "GNDD" H 1100 7050 50  0000 C CNN
F 2 "" H 1100 7200 60  0000 C CNN
F 3 "" H 1100 7200 60  0000 C CNN
	1    1100 7200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56595498
P 3850 6950
F 0 "C?" H 3875 7050 50  0000 L CNN
F 1 "0.1uF" H 3875 6850 50  0000 L CNN
F 2 "" H 3888 6800 30  0000 C CNN
F 3 "" H 3850 6950 60  0000 C CNN
	1    3850 6950
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5659549E
P 3600 6950
F 0 "C?" H 3625 7050 50  0000 L CNN
F 1 "0.1uF" H 3625 6850 50  0000 L CNN
F 2 "" H 3638 6800 30  0000 C CNN
F 3 "" H 3600 6950 60  0000 C CNN
	1    3600 6950
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565954A4
P 3350 6950
F 0 "C?" H 3375 7050 50  0000 L CNN
F 1 "0.1uF" H 3375 6850 50  0000 L CNN
F 2 "" H 3388 6800 30  0000 C CNN
F 3 "" H 3350 6950 60  0000 C CNN
	1    3350 6950
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565954AA
P 3100 6950
F 0 "C?" H 3125 7050 50  0000 L CNN
F 1 "0.1uF" H 3125 6850 50  0000 L CNN
F 2 "" H 3138 6800 30  0000 C CNN
F 3 "" H 3100 6950 60  0000 C CNN
	1    3100 6950
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565954B0
P 2850 6950
F 0 "C?" H 2875 7050 50  0000 L CNN
F 1 "0.1uF" H 2875 6850 50  0000 L CNN
F 2 "" H 2888 6800 30  0000 C CNN
F 3 "" H 2850 6950 60  0000 C CNN
	1    2850 6950
	-1   0    0    -1  
$EndComp
Text Label 4000 6700 1    60   ~ 0
+5V
$Comp
L GND #PWR?
U 1 1 56595647
P 4000 7200
F 0 "#PWR?" H 4000 6950 50  0001 C CNN
F 1 "GND" H 4000 7050 50  0000 C CNN
F 2 "" H 4000 7200 60  0000 C CNN
F 3 "" H 4000 7200 60  0000 C CNN
	1    4000 7200
	1    0    0    -1  
$EndComp
Text Notes 3250 7650 0    60   ~ 0
STEPPER DRIVERS ISOLATION
$Comp
L ISO7330FC U?
U 1 1 565B5285
P 6000 1050
F 0 "U?" H 5850 550 60  0000 C CNN
F 1 "ISO7330FC" H 6000 650 60  0000 C CNN
F 2 "" H 6000 950 60  0000 C CNN
F 3 "" H 6000 950 60  0000 C CNN
	1    6000 1050
	1    0    0    -1  
$EndComp
$Comp
L ISO7520C U?
U 1 1 565B53EC
P 6000 2050
F 0 "U?" H 5750 1600 60  0000 C CNN
F 1 "ISO7520C" H 5900 1700 60  0000 C CNN
F 2 "" H 6000 1850 60  0000 C CNN
F 3 "" H 6000 1850 60  0000 C CNN
	1    6000 2050
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565B6B3D
P 6550 1400
F 0 "#PWR?" H 6550 1150 50  0001 C CNN
F 1 "GND" H 6550 1250 50  0000 C CNN
F 2 "" H 6550 1400 60  0000 C CNN
F 3 "" H 6550 1400 60  0000 C CNN
	1    6550 1400
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 565B7594
P 5450 2350
F 0 "#PWR?" H 5450 2100 50  0001 C CNN
F 1 "GNDD" H 5450 2200 50  0000 C CNN
F 2 "" H 5450 2350 60  0000 C CNN
F 3 "" H 5450 2350 60  0000 C CNN
	1    5450 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565B765F
P 6550 2350
F 0 "#PWR?" H 6550 2100 50  0001 C CNN
F 1 "GND" H 6550 2200 50  0000 C CNN
F 2 "" H 6550 2350 60  0000 C CNN
F 3 "" H 6550 2350 60  0000 C CNN
	1    6550 2350
	1    0    0    -1  
$EndComp
Text Label 6500 1800 0    60   ~ 0
+5V
$Comp
L GNDD #PWR?
U 1 1 565B8409
P 5450 1400
F 0 "#PWR?" H 5450 1150 50  0001 C CNN
F 1 "GNDD" H 5450 1250 50  0000 C CNN
F 2 "" H 5450 1400 60  0000 C CNN
F 3 "" H 5450 1400 60  0000 C CNN
	1    5450 1400
	1    0    0    -1  
$EndComp
Text Label 5250 700  0    60   ~ 0
+3.3V
Text Label 5250 1800 0    60   ~ 0
+3.3V
Text Label 5300 850  0    60   ~ 0
DIN1
Text Label 5300 950  0    60   ~ 0
DIN2
Text Label 5300 1050 0    60   ~ 0
DEN
Text Label 6550 850  0    60   ~ 0
IN1
Text Label 6550 950  0    60   ~ 0
IN2
Text Label 6550 1050 0    60   ~ 0
EN
Text Label 6500 1150 0    60   ~ 0
+5V
Text Label 6500 1950 0    60   ~ 0
FAULT
Text Label 6500 2050 0    60   ~ 0
SOUT
Text Label 5250 1950 0    60   ~ 0
DFLT
Text Label 5250 2050 0    60   ~ 0
DSOUT
Text HLabel 6750 950  2    60   Output ~ 0
IN2
Text HLabel 6750 850  2    60   Output ~ 0
IN1
Text HLabel 6750 1050 2    60   Output ~ 0
EN
Text HLabel 5250 1950 0    60   Output ~ 0
DFLT
Text HLabel 5250 2050 0    60   Output ~ 0
DSOUT
Text HLabel 5250 850  0    60   Input ~ 0
DIN1
Text HLabel 5250 950  0    60   Input ~ 0
DIN2
Text HLabel 5250 1050 0    60   Input ~ 0
DEN
Text HLabel 6750 1950 2    60   Input ~ 0
FAULT
Text HLabel 6750 2050 2    60   Input ~ 0
SOUT
$Comp
L C C?
U 1 1 565C098A
P 6650 3050
F 0 "C?" H 6675 3150 50  0000 L CNN
F 1 "0.1uF" H 6675 2950 50  0000 L CNN
F 2 "" H 6688 2900 30  0000 C CNN
F 3 "" H 6650 3050 60  0000 C CNN
	1    6650 3050
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565C0990
P 6400 3050
F 0 "C?" H 6425 3150 50  0000 L CNN
F 1 "0.1uF" H 6425 2950 50  0000 L CNN
F 2 "" H 6438 2900 30  0000 C CNN
F 3 "" H 6400 3050 60  0000 C CNN
	1    6400 3050
	-1   0    0    -1  
$EndComp
Text Label 6800 2800 1    60   ~ 0
+5V
$Comp
L GND #PWR?
U 1 1 565C09A3
P 6800 3300
F 0 "#PWR?" H 6800 3050 50  0001 C CNN
F 1 "GND" H 6800 3150 50  0000 C CNN
F 2 "" H 6800 3300 60  0000 C CNN
F 3 "" H 6800 3300 60  0000 C CNN
	1    6800 3300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565C1CDA
P 5350 3050
F 0 "C?" H 5375 3150 50  0000 L CNN
F 1 "0.1uF" H 5375 2950 50  0000 L CNN
F 2 "" H 5388 2900 30  0000 C CNN
F 3 "" H 5350 3050 60  0000 C CNN
	1    5350 3050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565C1CE0
P 5600 3050
F 0 "C?" H 5625 3150 50  0000 L CNN
F 1 "0.1uF" H 5625 2950 50  0000 L CNN
F 2 "" H 5638 2900 30  0000 C CNN
F 3 "" H 5600 3050 60  0000 C CNN
	1    5600 3050
	1    0    0    -1  
$EndComp
Text Label 5200 2800 1    60   ~ 0
+3.3V
$Comp
L GNDD #PWR?
U 1 1 565C1CEA
P 5200 3300
F 0 "#PWR?" H 5200 3050 50  0001 C CNN
F 1 "GNDD" H 5200 3150 50  0000 C CNN
F 2 "" H 5200 3300 60  0000 C CNN
F 3 "" H 5200 3300 60  0000 C CNN
	1    5200 3300
	1    0    0    -1  
$EndComp
Text Label 3050 700  0    60   ~ 0
+5V
Text Notes 5800 3750 0    60   ~ 0
DC MOTOR DRIVER ISOLATION
$Comp
L ISO1541 U?
U 1 1 565C69D0
P 8650 1400
F 0 "U?" H 8550 1050 60  0000 C CNN
F 1 "ISO1541" H 8650 1150 60  0000 C CNN
F 2 "" H 8630 1400 60  0000 C CNN
F 3 "" H 8630 1400 60  0000 C CNN
	1    8650 1400
	1    0    0    -1  
$EndComp
$Comp
L ISO1541 U?
U 1 1 565C7141
P 8650 2200
F 0 "U?" H 8550 1850 60  0000 C CNN
F 1 "ISO1541" H 8650 1950 60  0000 C CNN
F 2 "" H 8630 2200 60  0000 C CNN
F 3 "" H 8630 2200 60  0000 C CNN
	1    8650 2200
	1    0    0    -1  
$EndComp
$Comp
L JMPR_01X03 P?
U 1 1 565C79FB
P 9950 1250
F 0 "P?" H 9950 1450 50  0000 C CNN
F 1 "JMPR_01X03" V 10050 1250 50  0000 C CNN
F 2 "" H 9950 1250 60  0000 C CNN
F 3 "" H 9950 1250 60  0000 C CNN
	1    9950 1250
	1    0    0    -1  
$EndComp
$Comp
L JMPR_01X02 P?
U 1 1 565CC173
P 9650 700
F 0 "P?" H 9650 850 50  0000 C CNN
F 1 "JMPR_01X02" V 9750 700 50  0000 C CNN
F 2 "" H 9650 700 60  0000 C CNN
F 3 "" H 9650 700 60  0000 C CNN
	1    9650 700 
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 565CD752
P 9600 1000
F 0 "#PWR?" H 9600 750 50  0001 C CNN
F 1 "GND" H 9600 850 50  0000 C CNN
F 2 "" H 9600 1000 60  0000 C CNN
F 3 "" H 9600 1000 60  0000 C CNN
	1    9600 1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565CEF68
P 9200 2450
F 0 "#PWR?" H 9200 2200 50  0001 C CNN
F 1 "GND" H 9200 2300 50  0000 C CNN
F 2 "" H 9200 2450 60  0000 C CNN
F 3 "" H 9200 2450 60  0000 C CNN
	1    9200 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565CEFBE
P 9200 1650
F 0 "#PWR?" H 9200 1400 50  0001 C CNN
F 1 "GND" H 9200 1500 50  0000 C CNN
F 2 "" H 9200 1650 60  0000 C CNN
F 3 "" H 9200 1650 60  0000 C CNN
	1    9200 1650
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 565CF68B
P 8100 1650
F 0 "#PWR?" H 8100 1400 50  0001 C CNN
F 1 "GNDD" H 8100 1500 50  0000 C CNN
F 2 "" H 8100 1650 60  0000 C CNN
F 3 "" H 8100 1650 60  0000 C CNN
	1    8100 1650
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 565CFAC9
P 8100 2450
F 0 "#PWR?" H 8100 2200 50  0001 C CNN
F 1 "GNDD" H 8100 2300 50  0000 C CNN
F 2 "" H 8100 2450 60  0000 C CNN
F 3 "" H 8100 2450 60  0000 C CNN
	1    8100 2450
	1    0    0    -1  
$EndComp
Text Label 7900 1250 0    60   ~ 0
+3.3V
Text Label 7900 2050 0    60   ~ 0
+3.3V
Text Label 9550 1350 0    60   ~ 0
+5V
Text Label 9150 2050 0    60   ~ 0
+5V
$Comp
L C C?
U 1 1 565D61AF
P 8700 3250
F 0 "C?" H 8725 3350 50  0000 L CNN
F 1 "0.1uF" H 8725 3150 50  0000 L CNN
F 2 "" H 8738 3100 30  0000 C CNN
F 3 "" H 8700 3250 60  0000 C CNN
	1    8700 3250
	-1   0    0    -1  
$EndComp
Text Label 8700 3000 1    60   ~ 0
+5V
$Comp
L GND #PWR?
U 1 1 565D61BE
P 8700 3500
F 0 "#PWR?" H 8700 3250 50  0001 C CNN
F 1 "GND" H 8700 3350 50  0000 C CNN
F 2 "" H 8700 3500 60  0000 C CNN
F 3 "" H 8700 3500 60  0000 C CNN
	1    8700 3500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565D61C4
P 7800 3250
F 0 "C?" H 7825 3350 50  0000 L CNN
F 1 "0.1uF" H 7825 3150 50  0000 L CNN
F 2 "" H 7838 3100 30  0000 C CNN
F 3 "" H 7800 3250 60  0000 C CNN
	1    7800 3250
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565D61CA
P 8050 3250
F 0 "C?" H 8075 3350 50  0000 L CNN
F 1 "0.1uF" H 8075 3150 50  0000 L CNN
F 2 "" H 8088 3100 30  0000 C CNN
F 3 "" H 8050 3250 60  0000 C CNN
	1    8050 3250
	1    0    0    -1  
$EndComp
Text Label 7650 3000 1    60   ~ 0
+3.3V
$Comp
L GNDD #PWR?
U 1 1 565D61D3
P 7650 3500
F 0 "#PWR?" H 7650 3250 50  0001 C CNN
F 1 "GNDD" H 7650 3350 50  0000 C CNN
F 2 "" H 7650 3500 60  0000 C CNN
F 3 "" H 7650 3500 60  0000 C CNN
	1    7650 3500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565DA3AE
P 9000 3250
F 0 "C?" H 9025 3350 50  0000 L CNN
F 1 "0.1uF" H 9025 3150 50  0000 L CNN
F 2 "" H 9038 3100 30  0000 C CNN
F 3 "" H 9000 3250 60  0000 C CNN
	1    9000 3250
	-1   0    0    -1  
$EndComp
Text Label 9000 3000 1    60   ~ 0
VEXT
$Comp
L GND #PWR?
U 1 1 565DA3B7
P 9000 3500
F 0 "#PWR?" H 9000 3250 50  0001 C CNN
F 1 "GND" H 9000 3350 50  0000 C CNN
F 2 "" H 9000 3500 60  0000 C CNN
F 3 "" H 9000 3500 60  0000 C CNN
	1    9000 3500
	1    0    0    -1  
$EndComp
Text Label 9200 1250 0    60   ~ 0
VEXT
Text HLabel 9200 2150 2    60   BiDi ~ 0
SDA
Text HLabel 9200 2250 2    60   Output ~ 0
SCL
Text HLabel 7900 2150 0    60   BiDi ~ 0
DSDA
Text HLabel 7900 2250 0    60   Input ~ 0
DSCL
Text HLabel 8100 1450 0    60   Input ~ 0
DSCL2
Text HLabel 8100 1350 0    60   BiDi ~ 0
DSDA2
Text Label 9200 1350 0    60   ~ 0
SDA2
Text Label 9200 1450 0    60   ~ 0
SCL2
Text Notes 7900 1100 0    50   Italic 0
MISCELLANEOUS I2C BUS\nW/ EXTERNAL OPERATIONAL VOLTAGE
Text Notes 9450 3750 0    60   ~ 0
I2C BUS ISOLATION
$Comp
L JMPR_01X04 P?
U 1 1 565A7C03
P 9950 1800
F 0 "P?" H 9950 2050 50  0000 C CNN
F 1 "JMPR_01X04" V 10050 1800 50  0000 C CNN
F 2 "" H 9950 1800 60  0000 C CNN
F 3 "" H 9950 1800 60  0000 C CNN
	1    9950 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565A8333
P 9700 2050
F 0 "#PWR?" H 9700 1800 50  0001 C CNN
F 1 "GND" H 9700 1900 50  0000 C CNN
F 2 "" H 9700 2050 60  0000 C CNN
F 3 "" H 9700 2050 60  0000 C CNN
	1    9700 2050
	1    0    0    -1  
$EndComp
Text Label 9550 1650 0    60   ~ 0
VEXT
Text Label 9550 1750 0    60   ~ 0
SDA2
Text Label 9550 1850 0    60   ~ 0
SCL2
$Comp
L ISO7140FCC U?
U 1 1 565AC178
P 6600 4650
F 0 "U?" H 6400 4150 60  0000 C CNN
F 1 "ISO7140FCC" H 6600 4250 60  0000 C CNN
F 2 "" H 6600 4650 60  0000 C CNN
F 3 "" H 6600 4650 60  0000 C CNN
	1    6600 4650
	1    0    0    -1  
$EndComp
$Comp
L ISO7140FCC U?
U 1 1 565AC17E
P 6600 5850
F 0 "U?" H 6800 5350 60  0000 C CNN
F 1 "ISO7140FCC" H 6600 5450 60  0000 C CNN
F 2 "" H 6600 5850 60  0000 C CNN
F 3 "" H 6600 5850 60  0000 C CNN
	1    6600 5850
	-1   0    0    -1  
$EndComp
Text Label 7100 4350 0    60   ~ 0
OUT0
Text Label 7100 4450 0    60   ~ 0
OUT1
Text Label 7100 4550 0    60   ~ 0
OUT2
Text Label 7100 4650 0    60   ~ 0
OUT3
Text Label 7100 5550 0    60   ~ 0
IN0
Text Label 7100 5650 0    60   ~ 0
IN1
Text Label 7100 5750 0    60   ~ 0
IN2
Text Label 7100 5850 0    60   ~ 0
IN3
Entry Wire Line
	5800 4250 5900 4350
Entry Wire Line
	5800 4350 5900 4450
Entry Wire Line
	5800 4450 5900 4550
Entry Wire Line
	5800 4550 5900 4650
Text Label 6100 4350 2    60   ~ 0
DO0
Text Label 5800 4200 2    60   ~ 0
DO[0..3]
Text Label 6100 4450 2    60   ~ 0
DO1
Text Label 6100 4550 2    60   ~ 0
DO2
Text Label 6100 4650 2    60   ~ 0
DO3
Text HLabel 5400 4200 0    60   Input ~ 0
DO[0..3]
Entry Wire Line
	5800 5450 5900 5550
Entry Wire Line
	5800 5550 5900 5650
Entry Wire Line
	5800 5650 5900 5750
Entry Wire Line
	5800 5750 5900 5850
Text Label 6100 5550 2    60   ~ 0
DI0
Text Label 5800 5400 2    60   ~ 0
DI[0..3]
Text Label 6100 5650 2    60   ~ 0
DI1
Text Label 6100 5750 2    60   ~ 0
DI2
Text Label 6100 5850 2    60   ~ 0
DI3
Text HLabel 5400 5400 0    60   Output ~ 0
DI[0..3]
$Comp
L GND #PWR?
U 1 1 565AC1AC
P 7150 5000
F 0 "#PWR?" H 7150 4750 50  0001 C CNN
F 1 "GND" H 7150 4850 50  0000 C CNN
F 2 "" H 7150 5000 60  0000 C CNN
F 3 "" H 7150 5000 60  0000 C CNN
	1    7150 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565AC1B2
P 7150 6200
F 0 "#PWR?" H 7150 5950 50  0001 C CNN
F 1 "GND" H 7150 6050 50  0000 C CNN
F 2 "" H 7150 6200 60  0000 C CNN
F 3 "" H 7150 6200 60  0000 C CNN
	1    7150 6200
	1    0    0    -1  
$EndComp
Text Label 7100 4200 0    60   ~ 0
+5V
Text Label 7100 4750 0    60   ~ 0
+5V
Text Label 7100 5400 0    60   ~ 0
+5V
Text Label 5850 5950 0    60   ~ 0
+3.3V
Text Label 5850 5400 0    60   ~ 0
+3.3V
Text Label 5850 4200 0    60   ~ 0
+3.3V
$Comp
L GNDD #PWR?
U 1 1 565AC1EA
P 6050 5000
F 0 "#PWR?" H 6050 4750 50  0001 C CNN
F 1 "GNDD" H 6050 4850 50  0000 C CNN
F 2 "" H 6050 5000 60  0000 C CNN
F 3 "" H 6050 5000 60  0000 C CNN
	1    6050 5000
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 565AC1F0
P 6050 6200
F 0 "#PWR?" H 6050 5950 50  0001 C CNN
F 1 "GNDD" H 6050 6050 50  0000 C CNN
F 2 "" H 6050 6200 60  0000 C CNN
F 3 "" H 6050 6200 60  0000 C CNN
	1    6050 6200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565BBDBF
P 8950 4700
F 0 "C?" H 8975 4800 50  0000 L CNN
F 1 "0.1uF" H 8975 4600 50  0000 L CNN
F 2 "" H 8988 4550 30  0000 C CNN
F 3 "" H 8950 4700 60  0000 C CNN
	1    8950 4700
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565BBDC5
P 8700 4700
F 0 "C?" H 8725 4800 50  0000 L CNN
F 1 "0.1uF" H 8725 4600 50  0000 L CNN
F 2 "" H 8738 4550 30  0000 C CNN
F 3 "" H 8700 4700 60  0000 C CNN
	1    8700 4700
	-1   0    0    -1  
$EndComp
Text Label 9100 4450 1    60   ~ 0
+5V
$Comp
L GND #PWR?
U 1 1 565BBDD6
P 9100 4950
F 0 "#PWR?" H 9100 4700 50  0001 C CNN
F 1 "GND" H 9100 4800 50  0000 C CNN
F 2 "" H 9100 4950 60  0000 C CNN
F 3 "" H 9100 4950 60  0000 C CNN
	1    9100 4950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565BBDDC
P 8850 5950
F 0 "C?" H 8875 6050 50  0000 L CNN
F 1 "0.1uF" H 8875 5850 50  0000 L CNN
F 2 "" H 8888 5800 30  0000 C CNN
F 3 "" H 8850 5950 60  0000 C CNN
	1    8850 5950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565BBDE2
P 9100 5950
F 0 "C?" H 9125 6050 50  0000 L CNN
F 1 "0.1uF" H 9125 5850 50  0000 L CNN
F 2 "" H 9138 5800 30  0000 C CNN
F 3 "" H 9100 5950 60  0000 C CNN
	1    9100 5950
	1    0    0    -1  
$EndComp
Text Label 8700 5700 1    60   ~ 0
+3.3V
$Comp
L GNDD #PWR?
U 1 1 565BBDEC
P 8700 6200
F 0 "#PWR?" H 8700 5950 50  0001 C CNN
F 1 "GNDD" H 8700 6050 50  0000 C CNN
F 2 "" H 8700 6200 60  0000 C CNN
F 3 "" H 8700 6200 60  0000 C CNN
	1    8700 6200
	1    0    0    -1  
$EndComp
Text Notes 8750 6650 0    60   ~ 0
GENERAL PURPOSE ISOLATED I/Os
$Comp
L JMPR_02X04 P?
U 1 1 565AADAD
P 7750 4500
F 0 "P?" H 7750 4750 50  0000 C CNN
F 1 "JMPR_02X04" H 7750 4250 50  0000 C CNN
F 2 "" H 7750 3300 60  0000 C CNN
F 3 "" H 7750 3300 60  0000 C CNN
	1    7750 4500
	-1   0    0    -1  
$EndComp
$Comp
L JMPR_02X04 P?
U 1 1 565AB2EE
P 7750 5700
F 0 "P?" H 7750 5950 50  0000 C CNN
F 1 "JMPR_02X04" H 7750 5450 50  0000 C CNN
F 2 "" H 7750 4500 60  0000 C CNN
F 3 "" H 7750 4500 60  0000 C CNN
	1    7750 5700
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565AC29B
P 8100 6000
F 0 "#PWR?" H 8100 5750 50  0001 C CNN
F 1 "GND" H 8100 5850 50  0000 C CNN
F 2 "" H 8100 6000 60  0000 C CNN
F 3 "" H 8100 6000 60  0000 C CNN
	1    8100 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565AC494
P 8100 4800
F 0 "#PWR?" H 8100 4550 50  0001 C CNN
F 1 "GND" H 8100 4650 50  0000 C CNN
F 2 "" H 8100 4800 60  0000 C CNN
F 3 "" H 8100 4800 60  0000 C CNN
	1    8100 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 850  3350 850 
Wire Wire Line
	3050 950  3350 950 
Wire Wire Line
	3050 1050 3350 1050
Wire Wire Line
	3050 1150 3350 1150
Wire Bus Line
	3450 700  3450 1050
Wire Bus Line
	3450 700  3950 700 
Wire Wire Line
	3050 2050 3350 2050
Wire Wire Line
	3050 2150 3350 2150
Wire Wire Line
	3050 2250 3350 2250
Wire Wire Line
	3050 2350 3350 2350
Wire Bus Line
	3450 1900 3450 2250
Wire Bus Line
	3450 1900 3950 1900
Wire Wire Line
	3050 3250 3350 3250
Wire Wire Line
	3050 3350 3350 3350
Wire Wire Line
	3050 3450 3350 3450
Wire Wire Line
	3050 3550 3350 3550
Wire Bus Line
	3450 3100 3450 3450
Wire Bus Line
	3450 3100 3950 3100
Wire Wire Line
	3050 4450 3350 4450
Wire Wire Line
	3050 4550 3350 4550
Wire Wire Line
	3050 4650 3350 4650
Wire Wire Line
	3050 4750 3350 4750
Wire Bus Line
	3450 4300 3450 4650
Wire Bus Line
	3450 4300 3950 4300
Wire Wire Line
	3050 5650 3350 5650
Wire Wire Line
	3050 5750 3350 5750
Wire Wire Line
	3050 5850 3350 5850
Wire Wire Line
	3050 5950 3350 5950
Wire Bus Line
	3450 5500 3450 5850
Wire Bus Line
	3450 5500 3950 5500
Wire Wire Line
	2050 850  1750 850 
Wire Wire Line
	2050 950  1750 950 
Wire Wire Line
	2050 1050 1750 1050
Wire Wire Line
	2050 1150 1750 1150
Wire Bus Line
	1650 700  1650 1050
Wire Bus Line
	1650 700  1150 700 
Wire Wire Line
	2050 2050 1750 2050
Wire Wire Line
	2050 2150 1750 2150
Wire Wire Line
	2050 2250 1750 2250
Wire Wire Line
	2050 2350 1750 2350
Wire Bus Line
	1650 1900 1650 2250
Wire Bus Line
	1650 1900 1150 1900
Wire Wire Line
	2050 3250 1750 3250
Wire Wire Line
	2050 3350 1750 3350
Wire Wire Line
	2050 3450 1750 3450
Wire Wire Line
	2050 3550 1750 3550
Wire Bus Line
	1650 3100 1650 3450
Wire Bus Line
	1650 3100 1150 3100
Wire Wire Line
	2050 4450 1750 4450
Wire Wire Line
	2050 4550 1750 4550
Wire Wire Line
	2050 4650 1750 4650
Wire Wire Line
	2050 4750 1750 4750
Wire Bus Line
	1650 4300 1650 4650
Wire Bus Line
	1650 4300 1150 4300
Wire Wire Line
	2050 5650 1750 5650
Wire Wire Line
	2050 5750 1750 5750
Wire Wire Line
	2050 5850 1750 5850
Wire Wire Line
	2050 5950 1750 5950
Wire Bus Line
	1650 5500 1650 5850
Wire Bus Line
	1650 5500 1150 5500
Wire Wire Line
	3050 4300 3350 4300
Wire Wire Line
	3050 5500 3350 5500
Wire Wire Line
	3050 3100 3350 3100
Wire Wire Line
	3050 1900 3350 1900
Wire Wire Line
	6500 700  6750 700 
Wire Wire Line
	3050 4850 3350 4850
Wire Wire Line
	3050 3650 3350 3650
Wire Wire Line
	3050 1250 3350 1250
Wire Wire Line
	3050 2450 3350 2450
Wire Wire Line
	3050 6150 3100 6150
Wire Wire Line
	3100 6150 3100 6300
Wire Wire Line
	3050 6250 3100 6250
Connection ~ 3100 6250
Wire Wire Line
	3050 4950 3100 4950
Wire Wire Line
	3100 4950 3100 5100
Wire Wire Line
	3050 5050 3100 5050
Connection ~ 3100 5050
Wire Wire Line
	3050 3750 3100 3750
Wire Wire Line
	3100 3750 3100 3900
Wire Wire Line
	3050 3850 3100 3850
Connection ~ 3100 3850
Wire Wire Line
	3050 2550 3100 2550
Wire Wire Line
	3100 2550 3100 2700
Wire Wire Line
	3050 2650 3100 2650
Connection ~ 3100 2650
Wire Wire Line
	3050 1350 3100 1350
Wire Wire Line
	3100 1350 3100 1500
Wire Wire Line
	3050 1450 3100 1450
Connection ~ 3100 1450
Wire Wire Line
	2050 700  1800 700 
Wire Wire Line
	2050 1900 1800 1900
Wire Wire Line
	2050 3100 1800 3100
Wire Wire Line
	2050 4300 1800 4300
Wire Wire Line
	2050 5500 1800 5500
Wire Wire Line
	2050 6050 1800 6050
Wire Wire Line
	2050 6150 2000 6150
Wire Wire Line
	2000 6150 2000 6300
Wire Wire Line
	2050 6250 2000 6250
Connection ~ 2000 6250
Wire Wire Line
	2050 4950 2000 4950
Wire Wire Line
	2000 4950 2000 5100
Wire Wire Line
	2050 5050 2000 5050
Connection ~ 2000 5050
Wire Wire Line
	2050 3750 2000 3750
Wire Wire Line
	2000 3750 2000 3900
Wire Wire Line
	2050 3850 2000 3850
Connection ~ 2000 3850
Wire Wire Line
	2050 2550 2000 2550
Wire Wire Line
	2000 2550 2000 2700
Wire Wire Line
	2050 2650 2000 2650
Connection ~ 2000 2650
Wire Wire Line
	2050 1350 2000 1350
Wire Wire Line
	2000 1350 2000 1500
Wire Wire Line
	2050 1450 2000 1450
Connection ~ 2000 1450
Wire Wire Line
	2250 6750 2250 6800
Wire Wire Line
	1100 6750 2250 6750
Wire Wire Line
	1100 6750 1100 6450
Wire Wire Line
	2250 7150 2250 7100
Wire Wire Line
	1100 7150 2250 7150
Wire Wire Line
	1100 7150 1100 7200
Wire Wire Line
	1250 6800 1250 6750
Connection ~ 1250 6750
Wire Wire Line
	1250 7100 1250 7150
Connection ~ 1250 7150
Wire Wire Line
	1500 7100 1500 7150
Connection ~ 1500 7150
Wire Wire Line
	1750 7100 1750 7150
Connection ~ 1750 7150
Wire Wire Line
	2000 7100 2000 7150
Connection ~ 2000 7150
Wire Wire Line
	2000 6800 2000 6750
Connection ~ 2000 6750
Wire Wire Line
	1750 6800 1750 6750
Connection ~ 1750 6750
Wire Wire Line
	1500 6800 1500 6750
Connection ~ 1500 6750
Wire Wire Line
	2850 6800 2850 6750
Wire Wire Line
	2850 6750 4000 6750
Wire Wire Line
	4000 6750 4000 6450
Wire Wire Line
	2850 7100 2850 7150
Wire Wire Line
	2850 7150 4000 7150
Wire Wire Line
	4000 7150 4000 7200
Wire Wire Line
	3850 6800 3850 6750
Connection ~ 3850 6750
Wire Wire Line
	3850 7100 3850 7150
Connection ~ 3850 7150
Wire Wire Line
	3600 7100 3600 7150
Connection ~ 3600 7150
Wire Wire Line
	3350 7100 3350 7150
Connection ~ 3350 7150
Wire Wire Line
	3100 7100 3100 7150
Connection ~ 3100 7150
Wire Wire Line
	3100 6800 3100 6750
Connection ~ 3100 6750
Wire Wire Line
	3350 6800 3350 6750
Connection ~ 3350 6750
Wire Wire Line
	3600 6800 3600 6750
Connection ~ 3600 6750
Wire Notes Line
	500  500  4600 500 
Wire Notes Line
	4600 500  4600 7700
Wire Notes Line
	4600 7700 500  7700
Wire Notes Line
	500  7700 500  500 
Wire Wire Line
	6500 850  6750 850 
Wire Wire Line
	6500 950  6750 950 
Wire Wire Line
	6500 1050 6750 1050
Wire Wire Line
	6500 1150 6750 1150
Wire Wire Line
	6500 1250 6550 1250
Wire Wire Line
	6550 1250 6550 1400
Wire Wire Line
	6500 1350 6550 1350
Connection ~ 6550 1350
Wire Wire Line
	5500 2200 5450 2200
Wire Wire Line
	5450 2200 5450 2350
Wire Wire Line
	5500 2300 5450 2300
Connection ~ 5450 2300
Wire Wire Line
	6500 2200 6550 2200
Wire Wire Line
	6550 2200 6550 2350
Wire Wire Line
	6500 2300 6550 2300
Connection ~ 6550 2300
Wire Wire Line
	6500 1800 6750 1800
Wire Wire Line
	6500 1950 6750 1950
Wire Wire Line
	6500 2050 6750 2050
Wire Wire Line
	5250 700  5500 700 
Wire Wire Line
	5500 850  5250 850 
Wire Wire Line
	5500 950  5250 950 
Wire Wire Line
	5500 1050 5250 1050
Wire Wire Line
	5500 1250 5450 1250
Wire Wire Line
	5450 1250 5450 1400
Wire Wire Line
	5500 1350 5450 1350
Connection ~ 5450 1350
Wire Wire Line
	5250 1800 5500 1800
Wire Wire Line
	5500 1950 5250 1950
Wire Wire Line
	5500 2050 5250 2050
Wire Wire Line
	6400 2850 6800 2850
Wire Wire Line
	6800 2850 6800 2550
Wire Wire Line
	6400 3250 6800 3250
Wire Wire Line
	6800 3250 6800 3300
Wire Wire Line
	6650 2900 6650 2850
Connection ~ 6650 2850
Wire Wire Line
	6650 3200 6650 3250
Connection ~ 6650 3250
Wire Wire Line
	5200 2850 5600 2850
Wire Wire Line
	5200 2850 5200 2550
Wire Wire Line
	5200 3250 5600 3250
Wire Wire Line
	5200 3250 5200 3300
Wire Wire Line
	5350 2900 5350 2850
Connection ~ 5350 2850
Wire Wire Line
	5350 3200 5350 3250
Connection ~ 5350 3250
Wire Wire Line
	5600 2850 5600 2900
Wire Wire Line
	5600 3250 5600 3200
Wire Wire Line
	6400 3200 6400 3250
Wire Wire Line
	6400 2900 6400 2850
Wire Wire Line
	3050 700  3350 700 
Wire Notes Line
	4700 500  7200 500 
Wire Notes Line
	7200 500  7200 3800
Wire Notes Line
	7200 3800 4700 3800
Wire Notes Line
	4700 3800 4700 500 
Wire Notes Line
	7300 500  10400 500 
Wire Notes Line
	10400 500  10400 3800
Wire Wire Line
	9700 900  9700 1150
Wire Wire Line
	9700 1150 9750 1150
Wire Wire Line
	9600 900  9600 1000
Wire Wire Line
	9150 1550 9200 1550
Wire Wire Line
	9200 1550 9200 1650
Wire Wire Line
	9150 2350 9200 2350
Wire Wire Line
	9200 2350 9200 2450
Wire Wire Line
	8150 1550 8100 1550
Wire Wire Line
	8100 1550 8100 1650
Wire Wire Line
	8150 2350 8100 2350
Wire Wire Line
	8100 2350 8100 2450
Wire Wire Line
	7900 1250 8150 1250
Wire Wire Line
	7900 2050 8150 2050
Wire Wire Line
	9550 1350 9750 1350
Wire Wire Line
	9150 2050 9400 2050
Wire Wire Line
	9150 2150 9200 2150
Wire Wire Line
	9150 2250 9200 2250
Wire Wire Line
	8150 2150 7900 2150
Wire Wire Line
	8150 2250 7900 2250
Wire Wire Line
	9150 1350 9400 1350
Wire Wire Line
	9150 1450 9400 1450
Wire Wire Line
	8150 1350 8100 1350
Wire Wire Line
	8150 1450 8100 1450
Wire Wire Line
	8700 3100 8700 2750
Wire Wire Line
	8700 3400 8700 3500
Wire Wire Line
	7650 3050 8050 3050
Wire Wire Line
	7650 3050 7650 2750
Wire Wire Line
	7650 3450 8050 3450
Wire Wire Line
	7650 3450 7650 3500
Wire Wire Line
	7800 3100 7800 3050
Connection ~ 7800 3050
Wire Wire Line
	7800 3400 7800 3450
Connection ~ 7800 3450
Wire Wire Line
	8050 3050 8050 3100
Wire Wire Line
	8050 3450 8050 3400
Wire Notes Line
	7300 500  7300 3800
Wire Notes Line
	7300 3800 10400 3800
Wire Notes Line
	9900 3100 9900 3100
Wire Wire Line
	9000 3100 9000 2750
Wire Wire Line
	9000 3400 9000 3500
Wire Wire Line
	9150 1250 9750 1250
Wire Wire Line
	9750 1650 9550 1650
Wire Wire Line
	9750 1750 9550 1750
Wire Wire Line
	9750 1850 9550 1850
Wire Wire Line
	9750 1950 9700 1950
Wire Wire Line
	9700 1950 9700 2050
Wire Wire Line
	7100 4350 7500 4350
Wire Wire Line
	7100 4450 7500 4450
Wire Wire Line
	7100 4550 7500 4550
Wire Wire Line
	7100 4650 7500 4650
Wire Wire Line
	7100 5550 7500 5550
Wire Wire Line
	7100 5650 7500 5650
Wire Wire Line
	7100 5750 7500 5750
Wire Wire Line
	7100 5850 7500 5850
Wire Wire Line
	6100 4350 5900 4350
Wire Wire Line
	6100 4450 5900 4450
Wire Wire Line
	6100 4550 5900 4550
Wire Wire Line
	6100 4650 5900 4650
Wire Bus Line
	5800 4200 5800 4550
Wire Bus Line
	5800 4200 5400 4200
Wire Wire Line
	6100 5550 5900 5550
Wire Wire Line
	6100 5650 5900 5650
Wire Wire Line
	6100 5750 5900 5750
Wire Wire Line
	6100 5850 5900 5850
Wire Bus Line
	5800 5400 5800 5750
Wire Bus Line
	5800 5400 5400 5400
Wire Wire Line
	7100 4200 7350 4200
Wire Wire Line
	7100 5400 7350 5400
Wire Wire Line
	7100 4750 7350 4750
Wire Wire Line
	7100 6050 7150 6050
Wire Wire Line
	7150 6050 7150 6200
Wire Wire Line
	7100 6150 7150 6150
Connection ~ 7150 6150
Wire Wire Line
	7100 4850 7150 4850
Wire Wire Line
	7150 4850 7150 5000
Wire Wire Line
	7100 4950 7150 4950
Connection ~ 7150 4950
Wire Wire Line
	6100 4200 5850 4200
Wire Wire Line
	6100 5400 5850 5400
Wire Wire Line
	6100 5950 5850 5950
Wire Wire Line
	6100 6050 6050 6050
Wire Wire Line
	6050 6050 6050 6200
Wire Wire Line
	6100 6150 6050 6150
Connection ~ 6050 6150
Wire Wire Line
	6100 4850 6050 4850
Wire Wire Line
	6050 4850 6050 5000
Wire Wire Line
	6100 4950 6050 4950
Connection ~ 6050 4950
Wire Notes Line
	4700 3900 10400 3900
Wire Notes Line
	10400 3900 10400 6700
Wire Notes Line
	4700 6700 4700 3900
Wire Wire Line
	8700 4500 9100 4500
Wire Wire Line
	9100 4500 9100 4200
Wire Wire Line
	8700 4900 9100 4900
Wire Wire Line
	9100 4900 9100 4950
Wire Wire Line
	8950 4550 8950 4500
Connection ~ 8950 4500
Wire Wire Line
	8950 4850 8950 4900
Connection ~ 8950 4900
Wire Wire Line
	8700 5750 9100 5750
Wire Wire Line
	8700 5750 8700 5450
Wire Wire Line
	8700 6150 9100 6150
Wire Wire Line
	8700 6150 8700 6200
Wire Wire Line
	8850 5800 8850 5750
Connection ~ 8850 5750
Wire Wire Line
	8850 6100 8850 6150
Connection ~ 8850 6150
Wire Wire Line
	9100 5750 9100 5800
Wire Wire Line
	9100 6150 9100 6100
Wire Wire Line
	8700 4850 8700 4900
Wire Wire Line
	8700 4550 8700 4500
Wire Wire Line
	8000 4350 8100 4350
Wire Wire Line
	8100 4350 8100 4800
Wire Wire Line
	8000 4450 8100 4450
Connection ~ 8100 4450
Wire Wire Line
	8000 4550 8100 4550
Connection ~ 8100 4550
Wire Wire Line
	8000 4650 8100 4650
Connection ~ 8100 4650
Wire Wire Line
	8000 5550 8100 5550
Wire Wire Line
	8100 5550 8100 6000
Wire Wire Line
	8000 5650 8100 5650
Connection ~ 8100 5650
Wire Wire Line
	8000 5750 8100 5750
Connection ~ 8100 5750
Wire Wire Line
	8000 5850 8100 5850
Connection ~ 8100 5850
Wire Notes Line
	10400 6700 4700 6700
Text HLabel 5150 7050 0    60   UnSpc ~ 0
VDD_LP
Text HLabel 5800 7050 2    60   UnSpc ~ 0
VDD_HP
Text HLabel 5150 7150 0    60   UnSpc ~ 0
GND_LP
$Comp
L GNDD #PWR?
U 1 1 565BB61A
P 5250 7250
F 0 "#PWR?" H 5250 7000 50  0001 C CNN
F 1 "GNDD" H 5250 7100 50  0000 C CNN
F 2 "" H 5250 7250 60  0000 C CNN
F 3 "" H 5250 7250 60  0000 C CNN
	1    5250 7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565BBE4F
P 5700 7250
F 0 "#PWR?" H 5700 7000 50  0001 C CNN
F 1 "GND" H 5700 7100 50  0000 C CNN
F 2 "" H 5700 7250 60  0000 C CNN
F 3 "" H 5700 7250 60  0000 C CNN
	1    5700 7250
	1    0    0    -1  
$EndComp
Text HLabel 5800 7150 2    60   UnSpc ~ 0
GND_HP
Wire Wire Line
	5150 7050 5400 7050
Wire Wire Line
	5150 7150 5250 7150
Wire Wire Line
	5250 7150 5250 7250
Wire Wire Line
	5800 7050 5600 7050
Wire Wire Line
	5800 7150 5700 7150
Wire Wire Line
	5700 7150 5700 7250
Text Label 5750 7050 2    60   ~ 0
+5V
Text Label 5150 7050 0    60   ~ 0
+3.3V
$EndSCHEMATC
