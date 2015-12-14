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
Sheet 5 13
Title "Stepper Driver"
Date "2015-11-22"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L DRV8880 U?
U 1 1 56526EB2
P 5800 4000
AR Path="/56526EA5/56526EB2" Ref="U?"  Part="1" 
AR Path="/5653D417/56526EB2" Ref="U?"  Part="1" 
AR Path="/5654C1EE/56526EB2" Ref="U?"  Part="1" 
AR Path="/56542384/56526EB2" Ref="U?"  Part="1" 
F 0 "U?" H 5300 3000 60  0000 C CNN
F 1 "DRV8880" H 5450 3100 60  0000 C CNN
F 2 "" H 5900 3700 60  0000 C CNN
F 3 "" H 5900 3700 60  0000 C CNN
	1    5800 4000
	1    0    0    -1  
$EndComp
$Comp
L MX_01X04_SPRING P?
U 1 1 56526F71
P 8150 2550
AR Path="/56526EA5/56526F71" Ref="P?"  Part="1" 
AR Path="/5653D417/56526F71" Ref="P?"  Part="1" 
AR Path="/5654C1EE/56526F71" Ref="P?"  Part="1" 
AR Path="/56542384/56526F71" Ref="P?"  Part="1" 
F 0 "P?" H 8150 2800 50  0000 C CNN
F 1 "MX_01X04_SPRING" V 8250 2550 50  0000 C CNN
F 2 "" H 8150 2550 60  0000 C CNN
F 3 "" H 8150 2550 60  0000 C CNN
	1    8150 2550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 565270AE
P 6900 4200
AR Path="/56526EA5/565270AE" Ref="R?"  Part="1" 
AR Path="/5653D417/565270AE" Ref="R?"  Part="1" 
AR Path="/5654C1EE/565270AE" Ref="R?"  Part="1" 
AR Path="/56542384/565270AE" Ref="R?"  Part="1" 
F 0 "R?" V 6980 4200 50  0000 C CNN
F 1 "0.1" V 6900 4200 50  0000 C CNN
F 2 "" V 6830 4200 30  0000 C CNN
F 3 "" H 6900 4200 30  0000 C CNN
	1    6900 4200
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5652718B
P 6900 4000
AR Path="/56526EA5/5652718B" Ref="R?"  Part="1" 
AR Path="/5653D417/5652718B" Ref="R?"  Part="1" 
AR Path="/5654C1EE/5652718B" Ref="R?"  Part="1" 
AR Path="/56542384/5652718B" Ref="R?"  Part="1" 
F 0 "R?" V 6980 4000 50  0000 C CNN
F 1 "0.1" V 6900 4000 50  0000 C CNN
F 2 "" V 6830 4000 30  0000 C CNN
F 3 "" H 6900 4000 30  0000 C CNN
	1    6900 4000
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 565273EE
P 5300 2750
AR Path="/56526EA5/565273EE" Ref="C?"  Part="1" 
AR Path="/5653D417/565273EE" Ref="C?"  Part="1" 
AR Path="/5654C1EE/565273EE" Ref="C?"  Part="1" 
AR Path="/56542384/565273EE" Ref="C?"  Part="1" 
F 0 "C?" H 5325 2850 50  0000 L CNN
F 1 "0.47uF" H 5325 2650 50  0000 L CNN
F 2 "" H 5338 2600 30  0000 C CNN
F 3 "" H 5300 2750 60  0000 C CNN
	1    5300 2750
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56527B5A
P 5900 2750
AR Path="/56526EA5/56527B5A" Ref="C?"  Part="1" 
AR Path="/5653D417/56527B5A" Ref="C?"  Part="1" 
AR Path="/5654C1EE/56527B5A" Ref="C?"  Part="1" 
AR Path="/56542384/56527B5A" Ref="C?"  Part="1" 
F 0 "C?" H 5925 2850 50  0000 L CNN
F 1 "0.47uF" H 5925 2650 50  0000 L CNN
F 2 "" H 5938 2600 30  0000 C CNN
F 3 "" H 5900 2750 60  0000 C CNN
	1    5900 2750
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56527B86
P 6300 2750
AR Path="/56526EA5/56527B86" Ref="C?"  Part="1" 
AR Path="/5653D417/56527B86" Ref="C?"  Part="1" 
AR Path="/5654C1EE/56527B86" Ref="C?"  Part="1" 
AR Path="/56542384/56527B86" Ref="C?"  Part="1" 
F 0 "C?" H 6325 2850 50  0000 L CNN
F 1 "0.1uF" H 6325 2650 50  0000 L CNN
F 2 "" H 6338 2600 30  0000 C CNN
F 3 "" H 6300 2750 60  0000 C CNN
	1    6300 2750
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56527BB1
P 4800 2750
AR Path="/56526EA5/56527BB1" Ref="C?"  Part="1" 
AR Path="/5653D417/56527BB1" Ref="C?"  Part="1" 
AR Path="/5654C1EE/56527BB1" Ref="C?"  Part="1" 
AR Path="/56542384/56527BB1" Ref="C?"  Part="1" 
F 0 "C?" H 4825 2850 50  0000 L CNN
F 1 "0.1uF" H 4825 2650 50  0000 L CNN
F 2 "" H 4838 2600 30  0000 C CNN
F 3 "" H 4800 2750 60  0000 C CNN
	1    4800 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 2400 5600 3000
$Comp
L C C?
U 1 1 56527EEA
P 5050 2750
AR Path="/56526EA5/56527EEA" Ref="C?"  Part="1" 
AR Path="/5653D417/56527EEA" Ref="C?"  Part="1" 
AR Path="/5654C1EE/56527EEA" Ref="C?"  Part="1" 
AR Path="/56542384/56527EEA" Ref="C?"  Part="1" 
F 0 "C?" H 5075 2850 50  0000 L CNN
F 1 "0.1uF" H 5075 2650 50  0000 L CNN
F 2 "" H 5088 2600 30  0000 C CNN
F 3 "" H 5050 2750 60  0000 C CNN
	1    5050 2750
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 565280CA
P 4500 2750
AR Path="/56526EA5/565280CA" Ref="C?"  Part="1" 
AR Path="/5653D417/565280CA" Ref="C?"  Part="1" 
AR Path="/5654C1EE/565280CA" Ref="C?"  Part="1" 
AR Path="/56542384/565280CA" Ref="C?"  Part="1" 
F 0 "C?" H 4525 2850 50  0000 L CNN
F 1 "220uF" H 4525 2650 50  0000 L CNN
F 2 "" H 4538 2600 30  0000 C CNN
F 3 "" H 4500 2750 60  0000 C CNN
	1    4500 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2400 4800 2600
Connection ~ 4800 2400
Wire Wire Line
	5050 2400 5050 2600
Connection ~ 5050 2400
Text Label 4250 2400 0    60   ~ 0
+VM
Wire Wire Line
	5700 2400 5700 3000
Connection ~ 5600 2400
Wire Wire Line
	4500 2600 4500 2400
Connection ~ 4500 2400
Wire Wire Line
	5900 2400 5900 2600
Connection ~ 5700 2400
Wire Wire Line
	5900 3000 5900 2900
Wire Wire Line
	6100 3000 6100 2950
Wire Wire Line
	6100 2950 6300 2950
Wire Wire Line
	6300 2950 6300 2900
Wire Wire Line
	6300 2600 6300 2400
Wire Wire Line
	6300 2400 6000 2400
Wire Wire Line
	6000 2400 6000 3000
Wire Wire Line
	5300 2600 5300 2550
Wire Wire Line
	5300 2550 5500 2550
Wire Wire Line
	5500 2550 5500 3000
$Comp
L GNDA #PWR01
U 1 1 5652A0D1
P 4300 3100
AR Path="/56526EA5/5652A0D1" Ref="#PWR01"  Part="1" 
AR Path="/5653D417/5652A0D1" Ref="#PWR08"  Part="1" 
AR Path="/5654C1EE/5652A0D1" Ref="#PWR022"  Part="1" 
AR Path="/56542384/5652A0D1" Ref="#PWR015"  Part="1" 
F 0 "#PWR022" H 4300 2850 50  0001 C CNN
F 1 "GNDA" H 4300 2950 50  0000 C CNN
F 2 "" H 4300 3100 60  0000 C CNN
F 3 "" H 4300 3100 60  0000 C CNN
	1    4300 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3100 4300 3000
Wire Wire Line
	4300 3000 5300 3000
Wire Wire Line
	5300 3000 5300 2900
Wire Wire Line
	5050 2900 5050 3000
Connection ~ 5050 3000
Wire Wire Line
	4800 2900 4800 3000
Connection ~ 4800 3000
Wire Wire Line
	4500 2900 4500 3000
Connection ~ 4500 3000
$Comp
L GNDA #PWR02
U 1 1 5652AD3D
P 7100 4300
AR Path="/56526EA5/5652AD3D" Ref="#PWR02"  Part="1" 
AR Path="/5653D417/5652AD3D" Ref="#PWR09"  Part="1" 
AR Path="/5654C1EE/5652AD3D" Ref="#PWR023"  Part="1" 
AR Path="/56542384/5652AD3D" Ref="#PWR016"  Part="1" 
F 0 "#PWR023" H 7100 4050 50  0001 C CNN
F 1 "GNDA" H 7100 4150 50  0000 C CNN
F 2 "" H 7100 4300 60  0000 C CNN
F 3 "" H 7100 4300 60  0000 C CNN
	1    7100 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4000 6750 4000
Wire Wire Line
	6550 4100 6700 4100
Wire Wire Line
	6700 4100 6700 4200
Wire Wire Line
	6700 4200 6750 4200
Wire Wire Line
	7050 4000 7100 4000
Wire Wire Line
	7100 4000 7100 4300
Wire Wire Line
	7050 4200 7100 4200
Connection ~ 7100 4200
Wire Wire Line
	7750 2400 7950 2400
Wire Wire Line
	7750 2500 7950 2500
Wire Wire Line
	7750 2600 7950 2600
Wire Wire Line
	7750 2700 7950 2700
Text Label 7750 2400 0    60   ~ 0
A1
Text Label 7750 2500 0    60   ~ 0
A2
Text Label 7750 2600 0    60   ~ 0
B2
Text Label 7750 2700 0    60   ~ 0
B1
Wire Wire Line
	6550 3500 6800 3500
Wire Wire Line
	6550 3600 6800 3600
Wire Wire Line
	6550 3700 6800 3700
Wire Wire Line
	6550 3800 6800 3800
Text Label 6700 3500 0    60   ~ 0
A1
Text Label 6700 3600 0    60   ~ 0
A2
Text Label 6700 3700 0    60   ~ 0
B1
Text Label 6700 3800 0    60   ~ 0
B2
$Comp
L GNDA #PWR03
U 1 1 5652CBA3
P 6100 5250
AR Path="/56526EA5/5652CBA3" Ref="#PWR03"  Part="1" 
AR Path="/5653D417/5652CBA3" Ref="#PWR010"  Part="1" 
AR Path="/5654C1EE/5652CBA3" Ref="#PWR024"  Part="1" 
AR Path="/56542384/5652CBA3" Ref="#PWR017"  Part="1" 
F 0 "#PWR024" H 6100 5000 50  0001 C CNN
F 1 "GNDA" H 6100 5100 50  0000 C CNN
F 2 "" H 6100 5250 60  0000 C CNN
F 3 "" H 6100 5250 60  0000 C CNN
	1    6100 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 5050 5800 5150
Wire Wire Line
	5800 5150 6100 5150
Wire Wire Line
	6100 5050 6100 5250
Wire Wire Line
	5900 5050 5900 5150
Connection ~ 5900 5150
Connection ~ 6100 5150
Text Label 5300 2550 0    60   ~ 0
V3P3
Wire Wire Line
	4250 2400 5900 2400
Wire Wire Line
	6550 4350 6800 4350
Text Label 6600 4350 0    60   ~ 0
V3P3
Wire Wire Line
	5050 4300 4800 4300
Wire Wire Line
	4800 4300 4800 4500
Wire Wire Line
	5050 4400 4800 4400
Connection ~ 4800 4400
$Comp
L GNDD #PWR04
U 1 1 56530DCA
P 4800 4500
AR Path="/56526EA5/56530DCA" Ref="#PWR04"  Part="1" 
AR Path="/5653D417/56530DCA" Ref="#PWR011"  Part="1" 
AR Path="/5654C1EE/56530DCA" Ref="#PWR025"  Part="1" 
AR Path="/56542384/56530DCA" Ref="#PWR018"  Part="1" 
F 0 "#PWR025" H 4800 4250 50  0001 C CNN
F 1 "GNDD" H 4800 4350 50  0000 C CNN
F 2 "" H 4800 4500 60  0000 C CNN
F 3 "" H 4800 4500 60  0000 C CNN
	1    4800 4500
	1    0    0    -1  
$EndComp
$Comp
L JMPR_02X03 P?
U 1 1 565318DF
P 7950 3300
AR Path="/56526EA5/565318DF" Ref="P?"  Part="1" 
AR Path="/5653D417/565318DF" Ref="P?"  Part="1" 
AR Path="/5654C1EE/565318DF" Ref="P?"  Part="1" 
AR Path="/56542384/565318DF" Ref="P?"  Part="1" 
F 0 "P?" H 7950 3500 50  0000 C CNN
F 1 "JMPR_02X03" H 7950 3100 50  0000 C CNN
F 2 "" H 7950 2100 60  0000 C CNN
F 3 "" H 7950 2100 60  0000 C CNN
	1    7950 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3300 7700 3300
Wire Wire Line
	8200 3300 8350 3300
Text Label 7550 3300 0    60   ~ 0
M0
Text Label 8250 3300 0    60   ~ 0
M1
Wire Wire Line
	8200 3000 8200 3200
Wire Wire Line
	7550 3000 8200 3000
Wire Wire Line
	7700 3200 7700 3000
Connection ~ 7700 3000
Text Label 7550 3000 0    60   ~ 0
V3P3
$Comp
L GNDD #PWR05
U 1 1 56533262
P 8200 3650
AR Path="/56526EA5/56533262" Ref="#PWR05"  Part="1" 
AR Path="/5653D417/56533262" Ref="#PWR012"  Part="1" 
AR Path="/5654C1EE/56533262" Ref="#PWR026"  Part="1" 
AR Path="/56542384/56533262" Ref="#PWR019"  Part="1" 
F 0 "#PWR026" H 8200 3400 50  0001 C CNN
F 1 "GNDD" H 8200 3500 50  0000 C CNN
F 2 "" H 8200 3650 60  0000 C CNN
F 3 "" H 8200 3650 60  0000 C CNN
	1    8200 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3400 8200 3650
Wire Wire Line
	7700 3400 7700 3550
Wire Wire Line
	7700 3550 8200 3550
Connection ~ 8200 3550
Wire Wire Line
	5050 3500 4900 3500
Wire Wire Line
	5050 3600 4900 3600
Text Label 4900 3500 0    60   ~ 0
M0
Text Label 4900 3600 0    60   ~ 0
M1
$Comp
L R R?
U 1 1 56534F8B
P 3200 3400
AR Path="/56526EA5/56534F8B" Ref="R?"  Part="1" 
AR Path="/5653D417/56534F8B" Ref="R?"  Part="1" 
AR Path="/5654C1EE/56534F8B" Ref="R?"  Part="1" 
AR Path="/56542384/56534F8B" Ref="R?"  Part="1" 
F 0 "R?" V 3280 3400 50  0000 C CNN
F 1 "10k" V 3200 3400 50  0000 C CNN
F 2 "" V 3130 3400 30  0000 C CNN
F 3 "" H 3200 3400 30  0000 C CNN
	1    3200 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4450 6800 4450
Wire Wire Line
	6550 4550 6800 4550
Text Label 6600 4450 0    60   ~ 0
VREF
Text Label 6600 4550 0    60   ~ 0
~FLT
Wire Wire Line
	5050 3800 4750 3800
Wire Wire Line
	5050 3900 4750 3900
Wire Wire Line
	5050 4000 4750 4000
Wire Wire Line
	5050 4100 4750 4100
Text Label 4750 3800 0    60   ~ 0
STEP
Text Label 4750 3900 0    60   ~ 0
DIR
Text Label 4750 4000 0    60   ~ 0
EN
Text Label 4750 4100 0    60   ~ 0
~SLP
Text HLabel 2600 3000 0    60   Input ~ 0
STEP
Text HLabel 2600 3100 0    60   Input ~ 0
DIR
Text HLabel 2600 3200 0    60   Input ~ 0
EN
Text HLabel 2600 3300 0    60   Input ~ 0
~SLEEP
Text HLabel 2600 3750 0    60   Input ~ 0
VREF
Text HLabel 2600 3550 0    60   Output ~ 0
~FAULT
Wire Wire Line
	2600 3000 2900 3000
Wire Wire Line
	2600 3100 2900 3100
Wire Wire Line
	2600 3200 2900 3200
Wire Wire Line
	2600 3300 2900 3300
Wire Wire Line
	2600 3550 3550 3550
Wire Wire Line
	2600 3750 2900 3750
Connection ~ 3200 3550
Wire Wire Line
	3200 3250 3200 3000
Text Label 3200 3200 1    60   ~ 0
V3P3
Text Label 3400 3550 0    60   ~ 0
~FLT
Text Label 2700 3000 0    60   ~ 0
STEP
Text Label 2700 3100 0    60   ~ 0
DIR
Text Label 2700 3200 0    60   ~ 0
EN
Text Label 2700 3300 0    60   ~ 0
~SLP
Text Label 2700 3750 0    60   ~ 0
VREF
$Comp
L GNDD #PWR06
U 1 1 565469D2
P 7700 5550
AR Path="/56526EA5/565469D2" Ref="#PWR06"  Part="1" 
AR Path="/5653D417/565469D2" Ref="#PWR013"  Part="1" 
AR Path="/5654C1EE/565469D2" Ref="#PWR027"  Part="1" 
AR Path="/56542384/565469D2" Ref="#PWR020"  Part="1" 
F 0 "#PWR027" H 7700 5300 50  0001 C CNN
F 1 "GNDD" H 7700 5400 50  0000 C CNN
F 2 "" H 7700 5550 60  0000 C CNN
F 3 "" H 7700 5550 60  0000 C CNN
	1    7700 5550
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR028
U 1 1 56546E45
P 8200 5550
AR Path="/5654C1EE/56546E45" Ref="#PWR028"  Part="1" 
AR Path="/56526EA5/56546E45" Ref="#PWR07"  Part="1" 
AR Path="/5653D417/56546E45" Ref="#PWR014"  Part="1" 
AR Path="/56542384/56546E45" Ref="#PWR021"  Part="1" 
F 0 "#PWR028" H 8200 5300 50  0001 C CNN
F 1 "GNDA" H 8200 5400 50  0000 C CNN
F 2 "" H 8200 5550 60  0000 C CNN
F 3 "" H 8200 5550 60  0000 C CNN
	1    8200 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 5550 7700 5450
Wire Wire Line
	7350 5450 8200 5450
Wire Wire Line
	8200 5450 8200 5550
Connection ~ 7700 5450
Text Label 7350 5450 0    60   ~ 0
GND
Text HLabel 2600 2400 0    60   UnSpc ~ 0
VM
Text HLabel 2600 2500 0    60   UnSpc ~ 0
GND
Wire Wire Line
	2600 2400 2900 2400
Wire Wire Line
	2600 2500 2900 2500
Text Label 2700 2400 0    60   ~ 0
+VM
Text Label 2700 2500 0    60   ~ 0
GND
$EndSCHEMATC
