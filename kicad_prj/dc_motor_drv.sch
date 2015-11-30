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
Sheet 6 11
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
L DRV8701 U?
U 1 1 5653CF04
P 6200 4600
F 0 "U?" H 5850 3800 60  0000 C CNN
F 1 "DRV8701" H 5950 3900 60  0000 C CNN
F 2 "" H 6100 4650 60  0000 C CNN
F 3 "" H 6100 4650 60  0000 C CNN
	1    6200 4600
	1    0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D088
P 8100 3950
F 0 "Q?" H 8390 3960 50  0000 R CNN
F 1 "CSD18532Q5B" H 8830 4030 50  0000 R CNN
F 2 "" H 8300 4050 29  0000 C CNN
F 3 "" H 8100 3950 60  0000 C CNN
	1    8100 3950
	1    0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D137
P 8100 4650
F 0 "Q?" H 8390 4660 50  0000 R CNN
F 1 "CSD18532Q5B" H 8830 4730 50  0000 R CNN
F 2 "" H 8300 4750 29  0000 C CNN
F 3 "" H 8100 4650 60  0000 C CNN
	1    8100 4650
	1    0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D1B9
P 9650 3950
F 0 "Q?" H 9940 3960 50  0000 R CNN
F 1 "CSD18532Q5B" H 10380 4030 50  0000 R CNN
F 2 "" H 9850 4050 29  0000 C CNN
F 3 "" H 9650 3950 60  0000 C CNN
	1    9650 3950
	-1   0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D22C
P 9650 4650
F 0 "Q?" H 9940 4660 50  0000 R CNN
F 1 "CSD18532Q5B" H 10380 4730 50  0000 R CNN
F 2 "" H 9850 4750 29  0000 C CNN
F 3 "" H 9650 4650 60  0000 C CNN
	1    9650 4650
	-1   0    0    -1  
$EndComp
$Comp
L PC_01X02_SPRING_HP P?
U 1 1 5653D47B
P 8900 4100
F 0 "P?" H 8900 4250 50  0000 C CNN
F 1 "PC_01X02_SPRING_HP" V 9000 4100 50  0000 C CNN
F 2 "" H 8900 4100 60  0000 C CNN
F 3 "" H 8900 4100 60  0000 C CNN
	1    8900 4100
	0    -1   -1   0   
$EndComp
Text Label 8900 3050 1    60   ~ 0
+VM
$Comp
L GNDA #PWR?
U 1 1 5653DA4C
P 8900 5500
F 0 "#PWR?" H 8900 5250 50  0001 C CNN
F 1 "GNDA" H 8900 5350 50  0000 C CNN
F 2 "" H 8900 5500 60  0000 C CNN
F 3 "" H 8900 5500 60  0000 C CNN
	1    8900 5500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5653DC0F
P 9800 3350
F 0 "C?" H 9825 3450 50  0000 L CNN
F 1 "1uF" H 9825 3250 50  0000 L CNN
F 2 "" H 9838 3200 30  0000 C CNN
F 3 "" H 9800 3350 60  0000 C CNN
	1    9800 3350
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 5653DC9E
P 10050 3350
F 0 "C?" H 10075 3450 50  0000 L CNN
F 1 "470uF" H 10075 3250 50  0000 L CNN
F 2 "" H 10088 3200 30  0000 C CNN
F 3 "" H 10050 3350 60  0000 C CNN
	1    10050 3350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5653E555
P 7950 3350
F 0 "C?" H 7975 3450 50  0000 L CNN
F 1 "1uF" H 7975 3250 50  0000 L CNN
F 2 "" H 7988 3200 30  0000 C CNN
F 3 "" H 7950 3350 60  0000 C CNN
	1    7950 3350
	-1   0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 5653E55B
P 7700 3350
F 0 "C?" H 7725 3450 50  0000 L CNN
F 1 "470uF" H 7725 3250 50  0000 L CNN
F 2 "" H 7738 3200 30  0000 C CNN
F 3 "" H 7700 3350 60  0000 C CNN
	1    7700 3350
	-1   0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5653EA18
P 7700 3600
F 0 "#PWR?" H 7700 3350 50  0001 C CNN
F 1 "GNDA" H 7700 3450 50  0000 C CNN
F 2 "" H 7700 3600 60  0000 C CNN
F 3 "" H 7700 3600 60  0000 C CNN
	1    7700 3600
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5653EA41
P 10050 3600
F 0 "#PWR?" H 10050 3350 50  0001 C CNN
F 1 "GNDA" H 10050 3450 50  0000 C CNN
F 2 "" H 10050 3600 60  0000 C CNN
F 3 "" H 10050 3600 60  0000 C CNN
	1    10050 3600
	1    0    0    -1  
$EndComp
Text Label 7650 3950 0    60   ~ 0
GH1
Text Label 7650 4650 0    60   ~ 0
GL1
Text Label 7650 4300 0    60   ~ 0
SH1
Text Label 9950 3950 0    60   ~ 0
GH2
Text Label 9950 4300 0    60   ~ 0
SH2
Text Label 9950 4650 0    60   ~ 0
GL2
Text Label 6950 4150 0    60   ~ 0
GH1
Text Label 6950 4250 0    60   ~ 0
SH1
Text Label 6950 4350 0    60   ~ 0
GL1
Text Label 6950 4450 0    60   ~ 0
GH2
Text Label 6950 4550 0    60   ~ 0
SH2
Text Label 6950 4650 0    60   ~ 0
GL2
$Comp
L R R?
U 1 1 56540BC2
P 8800 5250
F 0 "R?" V 8880 5250 50  0000 C CNN
F 1 "0.04" V 8800 5250 50  0000 C CNN
F 2 "" V 8730 5250 30  0000 C CNN
F 3 "" H 8800 5250 30  0000 C CNN
	1    8800 5250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56540C3E
P 9000 5250
F 0 "R?" V 9080 5250 50  0000 C CNN
F 1 "0.04" V 9000 5250 50  0000 C CNN
F 2 "" V 8930 5250 30  0000 C CNN
F 3 "" H 9000 5250 30  0000 C CNN
	1    9000 5250
	1    0    0    -1  
$EndComp
Text Label 7650 5050 0    60   ~ 0
SP
Text Label 7650 5450 0    60   ~ 0
SN
$Comp
L C C?
U 1 1 565413CB
P 8400 5250
F 0 "C?" H 8425 5350 50  0000 L CNN
F 1 "1000pF" H 8425 5150 50  0000 L CNN
F 2 "" H 8438 5100 30  0000 C CNN
F 3 "" H 8400 5250 60  0000 C CNN
	1    8400 5250
	1    0    0    -1  
$EndComp
Text Label 7000 4850 0    60   ~ 0
SP
Text Label 7000 4950 0    60   ~ 0
SN
$Comp
L C C?
U 1 1 56542CB1
P 5400 3550
F 0 "C?" H 5425 3650 50  0000 L CNN
F 1 "1uF" H 5425 3450 50  0000 L CNN
F 2 "" H 5438 3400 30  0000 C CNN
F 3 "" H 5400 3550 60  0000 C CNN
	1    5400 3550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56542D35
P 5650 3550
F 0 "C?" H 5675 3650 50  0000 L CNN
F 1 "1uF" H 5675 3450 50  0000 L CNN
F 2 "" H 5688 3400 30  0000 C CNN
F 3 "" H 5650 3550 60  0000 C CNN
	1    5650 3550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56542D78
P 5100 3550
F 0 "C?" H 5125 3650 50  0000 L CNN
F 1 "0.1uF" H 5125 3450 50  0000 L CNN
F 2 "" H 5138 3400 30  0000 C CNN
F 3 "" H 5100 3550 60  0000 C CNN
	1    5100 3550
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 56542E4E
P 4850 3550
F 0 "C?" H 4875 3650 50  0000 L CNN
F 1 "10uF" H 4875 3450 50  0000 L CNN
F 2 "" H 4888 3400 30  0000 C CNN
F 3 "" H 4850 3550 60  0000 C CNN
	1    4850 3550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56542FAF
P 6650 3400
F 0 "C?" H 6675 3500 50  0000 L CNN
F 1 "0.1uF" H 6675 3300 50  0000 L CNN
F 2 "" H 6688 3250 30  0000 C CNN
F 3 "" H 6650 3400 60  0000 C CNN
	1    6650 3400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565434D1
P 6300 3400
F 0 "C?" H 6325 3500 50  0000 L CNN
F 1 "1uF" H 6325 3300 50  0000 L CNN
F 2 "" H 6338 3250 30  0000 C CNN
F 3 "" H 6300 3400 60  0000 C CNN
	1    6300 3400
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR?
U 1 1 56543BD9
P 4850 3850
F 0 "#PWR?" H 4850 3600 50  0001 C CNN
F 1 "GNDD" H 4850 3700 50  0000 C CNN
F 2 "" H 4850 3850 60  0000 C CNN
F 3 "" H 4850 3850 60  0000 C CNN
	1    4850 3850
	1    0    0    -1  
$EndComp
Text Label 4500 3150 0    60   ~ 0
+VM
$Comp
L GNDD #PWR?
U 1 1 56545BB2
P 6550 5600
F 0 "#PWR?" H 6550 5350 50  0001 C CNN
F 1 "GNDD" H 6550 5450 50  0000 C CNN
F 2 "" H 6550 5600 60  0000 C CNN
F 3 "" H 6550 5600 60  0000 C CNN
	1    6550 5600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5654705B
P 5050 4550
F 0 "R?" V 5130 4550 50  0000 C CNN
F 1 "68k" V 5050 4550 50  0000 C CNN
F 2 "" V 4980 4550 30  0000 C CNN
F 3 "" H 5050 4550 30  0000 C CNN
	1    5050 4550
	0    1    1    0   
$EndComp
Text Label 4600 4550 0    60   ~ 0
AVDD
Text Label 5700 3350 0    60   ~ 0
AVDD
Text HLabel 1450 1950 0    60   Input ~ 0
IN1
Text HLabel 1450 2050 0    60   Input ~ 0
IN2
Text HLabel 1450 2150 0    60   Input ~ 0
~SLEEP
Text HLabel 1450 2400 0    60   Output ~ 0
~FAULT
Text HLabel 1450 2500 0    60   Output ~ 0
SNSOUT
Text Label 1600 1950 0    60   ~ 0
IN1
Text Label 1600 2050 0    60   ~ 0
IN2
Text Label 1600 2150 0    60   ~ 0
~SLP
Text Label 5250 4150 0    60   ~ 0
IN1
Text Label 5250 4250 0    60   ~ 0
IN2
Text Label 5250 4450 0    60   ~ 0
~SLP
$Comp
L R R?
U 1 1 5655BD62
P 2100 2150
F 0 "R?" V 2180 2150 50  0000 C CNN
F 1 "10k" V 2100 2150 50  0000 C CNN
F 2 "" V 2030 2150 30  0000 C CNN
F 3 "" H 2100 2150 30  0000 C CNN
	1    2100 2150
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5655BDF1
P 2300 2150
F 0 "R?" V 2380 2150 50  0000 C CNN
F 1 "10k" V 2300 2150 50  0000 C CNN
F 2 "" V 2230 2150 30  0000 C CNN
F 3 "" H 2300 2150 30  0000 C CNN
	1    2300 2150
	-1   0    0    1   
$EndComp
Text Label 2400 2400 0    60   ~ 0
~FLT
Text Label 2400 2500 0    60   ~ 0
SNS
Text Label 2300 1850 1    60   ~ 0
DVDD
Text Label 5700 3250 0    60   ~ 0
DVDD
Text Label 5250 4650 0    60   ~ 0
~FLT
Text Label 5250 4750 0    60   ~ 0
SNS
$Comp
L DAC7571 U?
U 1 1 5655F9FA
P 3900 4950
F 0 "U?" H 3750 4600 60  0000 C CNN
F 1 "DAC7571" H 3850 4700 60  0000 C CNN
F 2 "" H 3900 4950 60  0000 C CNN
F 3 "" H 3900 4950 60  0000 C CNN
	1    3900 4950
	1    0    0    -1  
$EndComp
$Comp
L REF02 U?
U 1 1 5655FD49
P 2100 4250
F 0 "U?" H 1850 3775 60  0000 C CNN
F 1 "REF02" H 1925 3850 60  0000 C CNN
F 2 "" H 2100 4250 60  0000 C CNN
F 3 "" H 2100 4250 60  0000 C CNN
	1    2100 4250
	1    0    0    -1  
$EndComp
$Comp
L RTRM R?
U 1 1 5654BAF5
P 2900 4400
F 0 "R?" V 2980 4350 50  0000 C CNN
F 1 "10k" V 2820 4530 50  0000 C CNN
F 2 "" H 2900 4400 60  0000 C CNN
F 3 "" H 2900 4400 60  0000 C CNN
	1    2900 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 4150 8200 4450
Wire Wire Line
	9550 4150 9550 4450
Wire Wire Line
	7600 4300 8850 4300
Connection ~ 8200 4300
Wire Wire Line
	8950 4300 10150 4300
Connection ~ 9550 4300
Wire Wire Line
	8200 3750 8200 3650
Wire Wire Line
	8200 3650 9550 3650
Wire Wire Line
	9550 3650 9550 3750
Wire Wire Line
	8900 2850 8900 3650
Connection ~ 8900 3650
Wire Wire Line
	8200 4850 8200 4950
Wire Wire Line
	8200 4950 9550 4950
Wire Wire Line
	9550 4950 9550 4850
Wire Wire Line
	8900 5050 8900 4950
Connection ~ 8900 4950
Wire Wire Line
	7700 3200 7700 3150
Wire Wire Line
	7700 3150 10050 3150
Wire Wire Line
	10050 3150 10050 3200
Connection ~ 8900 3150
Wire Wire Line
	7950 3200 7950 3150
Connection ~ 7950 3150
Wire Wire Line
	9800 3200 9800 3150
Connection ~ 9800 3150
Wire Wire Line
	7700 3500 7700 3600
Wire Wire Line
	7950 3500 7950 3550
Wire Wire Line
	7950 3550 7700 3550
Connection ~ 7700 3550
Wire Wire Line
	10050 3500 10050 3600
Wire Wire Line
	9800 3500 9800 3550
Wire Wire Line
	9800 3550 10050 3550
Connection ~ 10050 3550
Wire Wire Line
	7900 3950 7600 3950
Wire Wire Line
	7900 4650 7600 4650
Wire Wire Line
	9850 3950 10150 3950
Wire Wire Line
	9850 4650 10150 4650
Wire Wire Line
	6850 4150 7150 4150
Wire Wire Line
	6850 4250 7150 4250
Wire Wire Line
	6850 4350 7150 4350
Wire Wire Line
	6850 4450 7150 4450
Wire Wire Line
	6850 4550 7150 4550
Wire Wire Line
	6850 4650 7150 4650
Wire Wire Line
	7600 5050 9000 5050
Wire Wire Line
	8800 5050 8800 5100
Wire Wire Line
	9000 5050 9000 5100
Connection ~ 8900 5050
Wire Wire Line
	8800 5450 8800 5400
Wire Wire Line
	9000 5450 9000 5400
Wire Wire Line
	8900 5450 8900 5500
Connection ~ 8900 5450
Connection ~ 8800 5050
Connection ~ 8800 5450
Connection ~ 8400 5050
Connection ~ 8400 5450
Wire Wire Line
	7600 5450 9000 5450
Wire Wire Line
	8400 5450 8400 5400
Wire Wire Line
	8400 5100 8400 5050
Wire Wire Line
	6850 4850 7150 4850
Wire Wire Line
	6850 4950 7150 4950
Wire Wire Line
	6300 3650 6300 3550
Wire Wire Line
	6400 3650 6400 3150
Wire Wire Line
	6400 3150 6650 3150
Wire Wire Line
	6650 3150 6650 3250
Wire Wire Line
	6650 3550 6650 3600
Wire Wire Line
	6650 3600 6500 3600
Wire Wire Line
	6500 3600 6500 3650
Wire Wire Line
	6300 3150 6300 3250
Wire Wire Line
	4500 3150 6300 3150
Wire Wire Line
	5900 3650 5900 3350
Wire Wire Line
	5900 3350 5650 3350
Connection ~ 5100 3150
Wire Wire Line
	6000 3650 6000 3250
Wire Wire Line
	6000 3250 5400 3250
Wire Wire Line
	6100 3650 6100 3150
Connection ~ 6100 3150
Wire Wire Line
	5400 3250 5400 3400
Wire Wire Line
	5650 3350 5650 3400
Connection ~ 4850 3150
Wire Wire Line
	4850 3400 4850 3150
Wire Wire Line
	5100 3400 5100 3150
Wire Wire Line
	4850 3750 5650 3750
Connection ~ 4850 3750
Connection ~ 5400 3750
Connection ~ 5100 3750
Wire Wire Line
	6250 5450 6250 5500
Wire Wire Line
	6250 5500 6550 5500
Wire Wire Line
	6550 5450 6550 5600
Connection ~ 6550 5500
Wire Wire Line
	6350 5450 6350 5500
Connection ~ 6350 5500
Wire Wire Line
	5200 4550 5550 4550
Wire Wire Line
	4900 4550 4600 4550
Wire Wire Line
	5650 3750 5650 3700
Wire Wire Line
	5400 3700 5400 3750
Wire Wire Line
	5100 3700 5100 3750
Wire Wire Line
	4850 3700 4850 3850
Wire Wire Line
	1450 1950 1750 1950
Wire Wire Line
	1450 2050 1750 2050
Wire Wire Line
	1450 2150 1750 2150
Wire Wire Line
	5550 4150 5250 4150
Wire Wire Line
	5550 4250 5250 4250
Wire Wire Line
	5550 4450 5250 4450
Wire Wire Line
	2100 2000 2100 1950
Wire Wire Line
	2100 1950 2300 1950
Wire Wire Line
	2300 1650 2300 2000
Connection ~ 2300 1950
Wire Wire Line
	1450 2400 2550 2400
Wire Wire Line
	2100 2300 2100 2400
Connection ~ 2100 2400
Wire Wire Line
	1450 2500 2550 2500
Wire Wire Line
	2300 2300 2300 2500
Connection ~ 2300 2500
Wire Wire Line
	5550 4650 5250 4650
Wire Wire Line
	5550 4750 5250 4750
Wire Wire Line
	2610 4100 4400 4100
Wire Wire Line
	2900 4100 2900 4150
Wire Wire Line
	2610 4400 2660 4400
Wire Wire Line
	2900 4650 2900 4850
$Comp
L C C?
U 1 1 565577FF
P 1450 3800
F 0 "C?" H 1475 3900 50  0000 L CNN
F 1 "0.1uF" H 1475 3700 50  0000 L CNN
F 2 "" H 1488 3650 30  0000 C CNN
F 3 "" H 1450 3800 60  0000 C CNN
	1    1450 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3600 2100 3740
Wire Wire Line
	900  3600 2100 3600
Wire Wire Line
	1450 3650 1450 3600
Connection ~ 1450 3600
Text Label 900  3600 0    60   ~ 0
+12V
$Comp
L GNDD #PWR?
U 1 1 5655A17D
P 2100 4900
F 0 "#PWR?" H 2100 4650 50  0001 C CNN
F 1 "GNDD" H 2100 4750 50  0000 C CNN
F 2 "" H 2100 4900 60  0000 C CNN
F 3 "" H 2100 4900 60  0000 C CNN
	1    2100 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 4810 2100 4900
Wire Wire Line
	2900 4850 2100 4850
Connection ~ 2100 4850
$Comp
L GNDD #PWR?
U 1 1 5655ADA8
P 1450 4000
F 0 "#PWR?" H 1450 3750 50  0001 C CNN
F 1 "GNDD" H 1450 3850 50  0000 C CNN
F 2 "" H 1450 4000 60  0000 C CNN
F 3 "" H 1450 4000 60  0000 C CNN
	1    1450 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 3950 1450 4000
Wire Wire Line
	4350 4950 5550 4950
Wire Wire Line
	4400 4100 4400 4850
Wire Wire Line
	4400 4850 4350 4850
Connection ~ 2900 4100
$Comp
L GNDD #PWR?
U 1 1 5655DEA6
P 4400 5150
F 0 "#PWR?" H 4400 4900 50  0001 C CNN
F 1 "GNDD" H 4400 5000 50  0000 C CNN
F 2 "" H 4400 5150 60  0000 C CNN
F 3 "" H 4400 5150 60  0000 C CNN
	1    4400 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 5050 4400 5050
Wire Wire Line
	4400 5050 4400 5150
Wire Wire Line
	3450 4850 3150 4850
Wire Wire Line
	3450 4950 3150 4950
Text Label 3150 4850 0    60   ~ 0
SDA
Text Label 3150 4950 0    60   ~ 0
SCL
$Comp
L GNDD #PWR?
U 1 1 56564EE8
P 1400 5850
F 0 "#PWR?" H 1400 5600 50  0001 C CNN
F 1 "GNDD" H 1400 5700 50  0000 C CNN
F 2 "" H 1400 5850 60  0000 C CNN
F 3 "" H 1400 5850 60  0000 C CNN
	1    1400 5850
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 56564F41
P 1750 5850
F 0 "#PWR?" H 1750 5600 50  0001 C CNN
F 1 "GNDA" H 1750 5700 50  0000 C CNN
F 2 "" H 1750 5850 60  0000 C CNN
F 3 "" H 1750 5850 60  0000 C CNN
	1    1750 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 5850 1400 5750
Wire Wire Line
	1150 5750 1750 5750
Wire Wire Line
	1750 5750 1750 5850
Connection ~ 1400 5750
Text Label 1150 5750 0    60   ~ 0
GND
Text HLabel 1450 2900 0    60   BiDi ~ 0
SDA
Text HLabel 1450 3000 0    60   Input ~ 0
SCL
Wire Wire Line
	1450 2900 1750 2900
Wire Wire Line
	1450 3000 1750 3000
Text Label 1600 2900 0    60   ~ 0
SDA
Text Label 1600 3000 0    60   ~ 0
SCL
Text HLabel 1450 1400 0    60   UnSpc ~ 0
VM
Text HLabel 1450 1500 0    60   UnSpc ~ 0
VDD
Text HLabel 1450 1600 0    60   UnSpc ~ 0
GND
Wire Wire Line
	1450 1400 1750 1400
Wire Wire Line
	1450 1500 1750 1500
Wire Wire Line
	1450 1600 1750 1600
Text Label 1600 1400 0    60   ~ 0
+VM
Text Label 1600 1500 0    60   ~ 0
+12V
Text Label 1600 1600 0    60   ~ 0
GND
Text HLabel 1450 2600 0    60   Output ~ 0
SO
Wire Wire Line
	1450 2600 1750 2600
Text Label 1600 2600 0    60   ~ 0
SO
Wire Wire Line
	5550 5050 5250 5050
Text Label 5250 5050 0    60   ~ 0
SO
$Comp
L R R?
U 1 1 56576226
P 3250 5050
F 0 "R?" V 3330 5050 50  0000 C CNN
F 1 "10k" V 3250 5050 50  0000 C CNN
F 2 "" V 3180 5050 30  0000 C CNN
F 3 "" H 3250 5050 30  0000 C CNN
	1    3250 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	3450 5050 3400 5050
Wire Wire Line
	3100 5050 2850 5050
Text Label 3150 4100 0    60   ~ 0
+5V
Text Label 2850 5050 0    60   ~ 0
+5V
Text HLabel 1450 2700 0    60   Output ~ 0
+5V_REF
Wire Wire Line
	1450 2700 1750 2700
Text Label 1550 2700 0    60   ~ 0
+5V
$Comp
L GNDD #PWR?
U 1 1 565A13C9
P 4100 4500
F 0 "#PWR?" H 4100 4250 50  0001 C CNN
F 1 "GNDD" H 4100 4350 50  0000 C CNN
F 2 "" H 4100 4500 60  0000 C CNN
F 3 "" H 4100 4500 60  0000 C CNN
	1    4100 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4150 4100 4100
Connection ~ 4100 4100
Wire Wire Line
	4100 4500 4100 4450
Wire Wire Line
	3850 4150 3850 4100
Connection ~ 3850 4100
$Comp
L CP C?
U 1 1 565FCB4C
P 1200 3800
F 0 "C?" H 1225 3900 50  0000 L CNN
F 1 "10uF" H 1225 3700 50  0000 L CNN
F 2 "" H 1238 3650 30  0000 C CNN
F 3 "" H 1200 3800 60  0000 C CNN
	1    1200 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 3650 1200 3600
Connection ~ 1200 3600
Wire Wire Line
	1200 3950 1450 3950
Connection ~ 1450 3950
$Comp
L CP C?
U 1 1 565B9620
P 3850 4300
F 0 "C?" H 3875 4400 50  0000 L CNN
F 1 "1uF" H 3875 4200 50  0000 L CNN
F 2 "" H 3888 4150 30  0000 C CNN
F 3 "" H 3850 4300 60  0000 C CNN
	1    3850 4300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565B96BD
P 4100 4300
F 0 "C?" H 4125 4400 50  0000 L CNN
F 1 "0.1uF" H 4125 4200 50  0000 L CNN
F 2 "" H 4138 4150 30  0000 C CNN
F 3 "" H 4100 4300 60  0000 C CNN
	1    4100 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4450 3850 4450
Connection ~ 4100 4450
$EndSCHEMATC
