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
$Descr A4 11693 8268
encoding utf-8
Sheet 6 6
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
P 4950 3700
F 0 "U?" H 4600 2900 60  0000 C CNN
F 1 "DRV8701" H 4700 3000 60  0000 C CNN
F 2 "" H 4850 3750 60  0000 C CNN
F 3 "" H 4850 3750 60  0000 C CNN
	1    4950 3700
	1    0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D088
P 6900 3100
F 0 "Q?" H 7190 3110 50  0000 R CNN
F 1 "CSD18532Q5B" H 7630 3180 50  0000 R CNN
F 2 "" H 7100 3200 29  0000 C CNN
F 3 "" H 6900 3100 60  0000 C CNN
	1    6900 3100
	1    0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D137
P 6900 3800
F 0 "Q?" H 7190 3810 50  0000 R CNN
F 1 "CSD18532Q5B" H 7630 3880 50  0000 R CNN
F 2 "" H 7100 3900 29  0000 C CNN
F 3 "" H 6900 3800 60  0000 C CNN
	1    6900 3800
	1    0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D1B9
P 8450 3100
F 0 "Q?" H 8740 3110 50  0000 R CNN
F 1 "CSD18532Q5B" H 9180 3180 50  0000 R CNN
F 2 "" H 8650 3200 29  0000 C CNN
F 3 "" H 8450 3100 60  0000 C CNN
	1    8450 3100
	-1   0    0    -1  
$EndComp
$Comp
L CSD18532Q5B Q?
U 1 1 5653D22C
P 8450 3800
F 0 "Q?" H 8740 3810 50  0000 R CNN
F 1 "CSD18532Q5B" H 9180 3880 50  0000 R CNN
F 2 "" H 8650 3900 29  0000 C CNN
F 3 "" H 8450 3800 60  0000 C CNN
	1    8450 3800
	-1   0    0    -1  
$EndComp
$Comp
L PC_01X02_SPRING_HP P?
U 1 1 5653D47B
P 7700 3250
F 0 "P?" H 7700 3400 50  0000 C CNN
F 1 "PC_01X02_SPRING_HP" V 7800 3250 50  0000 C CNN
F 2 "" H 7700 3250 60  0000 C CNN
F 3 "" H 7700 3250 60  0000 C CNN
	1    7700 3250
	0    -1   -1   0   
$EndComp
Text Label 7700 2200 1    60   ~ 0
+VM
$Comp
L GNDA #PWR?
U 1 1 5653DA4C
P 7700 4650
F 0 "#PWR?" H 7700 4400 50  0001 C CNN
F 1 "GNDA" H 7700 4500 50  0000 C CNN
F 2 "" H 7700 4650 60  0000 C CNN
F 3 "" H 7700 4650 60  0000 C CNN
	1    7700 4650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5653DC0F
P 8600 2500
F 0 "C?" H 8625 2600 50  0000 L CNN
F 1 "1uF" H 8625 2400 50  0000 L CNN
F 2 "" H 8638 2350 30  0000 C CNN
F 3 "" H 8600 2500 60  0000 C CNN
	1    8600 2500
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 5653DC9E
P 8850 2500
F 0 "C?" H 8875 2600 50  0000 L CNN
F 1 "470uF" H 8875 2400 50  0000 L CNN
F 2 "" H 8888 2350 30  0000 C CNN
F 3 "" H 8850 2500 60  0000 C CNN
	1    8850 2500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5653E555
P 6750 2500
F 0 "C?" H 6775 2600 50  0000 L CNN
F 1 "1uF" H 6775 2400 50  0000 L CNN
F 2 "" H 6788 2350 30  0000 C CNN
F 3 "" H 6750 2500 60  0000 C CNN
	1    6750 2500
	-1   0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 5653E55B
P 6500 2500
F 0 "C?" H 6525 2600 50  0000 L CNN
F 1 "470uF" H 6525 2400 50  0000 L CNN
F 2 "" H 6538 2350 30  0000 C CNN
F 3 "" H 6500 2500 60  0000 C CNN
	1    6500 2500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7000 3300 7000 3450
Wire Wire Line
	7000 3450 7000 3600
Wire Wire Line
	8350 3300 8350 3450
Wire Wire Line
	8350 3450 8350 3600
Wire Wire Line
	6400 3450 7000 3450
Wire Wire Line
	7000 3450 7650 3450
Connection ~ 7000 3450
Wire Wire Line
	7750 3450 8350 3450
Wire Wire Line
	8350 3450 8950 3450
Connection ~ 8350 3450
Wire Wire Line
	7000 2900 7000 2800
Wire Wire Line
	7000 2800 7700 2800
Wire Wire Line
	7700 2800 8350 2800
Wire Wire Line
	8350 2800 8350 2900
Wire Wire Line
	7700 2000 7700 2300
Wire Wire Line
	7700 2300 7700 2800
Connection ~ 7700 2800
Wire Wire Line
	7000 4000 7000 4100
Wire Wire Line
	7000 4100 7700 4100
Wire Wire Line
	7700 4100 8350 4100
Wire Wire Line
	8350 4100 8350 4000
Wire Wire Line
	7700 4200 7700 4100
Connection ~ 7700 4100
Wire Wire Line
	6500 2350 6500 2300
Wire Wire Line
	6500 2300 6750 2300
Wire Wire Line
	6750 2300 7700 2300
Wire Wire Line
	7700 2300 8600 2300
Wire Wire Line
	8600 2300 8850 2300
Wire Wire Line
	8850 2300 8850 2350
Connection ~ 7700 2300
Wire Wire Line
	6750 2350 6750 2300
Connection ~ 6750 2300
Wire Wire Line
	8600 2350 8600 2300
Connection ~ 8600 2300
$Comp
L GNDA #PWR?
U 1 1 5653EA18
P 6500 2750
F 0 "#PWR?" H 6500 2500 50  0001 C CNN
F 1 "GNDA" H 6500 2600 50  0000 C CNN
F 2 "" H 6500 2750 60  0000 C CNN
F 3 "" H 6500 2750 60  0000 C CNN
	1    6500 2750
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5653EA41
P 8850 2750
F 0 "#PWR?" H 8850 2500 50  0001 C CNN
F 1 "GNDA" H 8850 2600 50  0000 C CNN
F 2 "" H 8850 2750 60  0000 C CNN
F 3 "" H 8850 2750 60  0000 C CNN
	1    8850 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2650 6500 2700
Wire Wire Line
	6500 2700 6500 2750
Wire Wire Line
	6750 2650 6750 2700
Wire Wire Line
	6750 2700 6500 2700
Connection ~ 6500 2700
Wire Wire Line
	8850 2650 8850 2700
Wire Wire Line
	8850 2700 8850 2750
Wire Wire Line
	8600 2650 8600 2700
Wire Wire Line
	8600 2700 8850 2700
Connection ~ 8850 2700
Wire Wire Line
	6700 3100 6400 3100
Wire Wire Line
	6700 3800 6400 3800
Wire Wire Line
	8650 3100 8950 3100
Wire Wire Line
	8650 3800 8950 3800
Text Label 6450 3100 0    60   ~ 0
GH1
Text Label 6450 3800 0    60   ~ 0
GL1
Text Label 6450 3450 0    60   ~ 0
SH1
Text Label 8750 3100 0    60   ~ 0
GH2
Text Label 8750 3450 0    60   ~ 0
SH2
Text Label 8750 3800 0    60   ~ 0
GL2
Wire Wire Line
	5600 3250 5900 3250
Wire Wire Line
	5600 3350 5900 3350
Wire Wire Line
	5600 3450 5900 3450
Wire Wire Line
	5600 3550 5900 3550
Wire Wire Line
	5600 3650 5900 3650
Wire Wire Line
	5600 3750 5900 3750
Text Label 5700 3250 0    60   ~ 0
GH1
Text Label 5700 3350 0    60   ~ 0
SH1
Text Label 5700 3450 0    60   ~ 0
GL1
Text Label 5700 3550 0    60   ~ 0
GH2
Text Label 5700 3650 0    60   ~ 0
SH2
Text Label 5700 3750 0    60   ~ 0
GL2
$Comp
L R R?
U 1 1 56540BC2
P 7600 4400
F 0 "R?" V 7680 4400 50  0000 C CNN
F 1 "0.04" V 7600 4400 50  0000 C CNN
F 2 "" V 7530 4400 30  0000 C CNN
F 3 "" H 7600 4400 30  0000 C CNN
	1    7600 4400
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56540C3E
P 7800 4400
F 0 "R?" V 7880 4400 50  0000 C CNN
F 1 "0.04" V 7800 4400 50  0000 C CNN
F 2 "" V 7730 4400 30  0000 C CNN
F 3 "" H 7800 4400 30  0000 C CNN
	1    7800 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4200 7200 4200
Wire Wire Line
	7200 4200 7600 4200
Wire Wire Line
	7600 4200 7700 4200
Wire Wire Line
	7700 4200 7800 4200
Wire Wire Line
	7600 4200 7600 4250
Wire Wire Line
	7800 4200 7800 4250
Connection ~ 7700 4200
Wire Wire Line
	7600 4600 7600 4550
Wire Wire Line
	7800 4600 7800 4550
Wire Wire Line
	7700 4600 7700 4650
Connection ~ 7700 4600
Connection ~ 7600 4200
Connection ~ 7600 4600
Text Label 6450 4200 0    60   ~ 0
SP
Text Label 6450 4600 0    60   ~ 0
SN
$Comp
L C C?
U 1 1 565413CB
P 7200 4400
F 0 "C?" H 7225 4500 50  0000 L CNN
F 1 "1000pF" H 7225 4300 50  0000 L CNN
F 2 "" H 7238 4250 30  0000 C CNN
F 3 "" H 7200 4400 60  0000 C CNN
	1    7200 4400
	1    0    0    -1  
$EndComp
Connection ~ 7200 4200
Connection ~ 7200 4600
Wire Wire Line
	6400 4600 7200 4600
Wire Wire Line
	7200 4600 7600 4600
Wire Wire Line
	7600 4600 7700 4600
Wire Wire Line
	7700 4600 7800 4600
Wire Wire Line
	7200 4600 7200 4550
Wire Wire Line
	7200 4250 7200 4200
Wire Wire Line
	5600 3950 5900 3950
Wire Wire Line
	5600 4050 5900 4050
Text Label 5750 3950 0    60   ~ 0
SP
Text Label 5750 4050 0    60   ~ 0
SN
$Comp
L C C?
U 1 1 56542CB1
P 4150 2650
F 0 "C?" H 4175 2750 50  0000 L CNN
F 1 "1uF" H 4175 2550 50  0000 L CNN
F 2 "" H 4188 2500 30  0000 C CNN
F 3 "" H 4150 2650 60  0000 C CNN
	1    4150 2650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56542D35
P 4400 2650
F 0 "C?" H 4425 2750 50  0000 L CNN
F 1 "1uF" H 4425 2550 50  0000 L CNN
F 2 "" H 4438 2500 30  0000 C CNN
F 3 "" H 4400 2650 60  0000 C CNN
	1    4400 2650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56542D78
P 3850 2650
F 0 "C?" H 3875 2750 50  0000 L CNN
F 1 "0.1uF" H 3875 2550 50  0000 L CNN
F 2 "" H 3888 2500 30  0000 C CNN
F 3 "" H 3850 2650 60  0000 C CNN
	1    3850 2650
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 56542E4E
P 3600 2650
F 0 "C?" H 3625 2750 50  0000 L CNN
F 1 "10uF" H 3625 2550 50  0000 L CNN
F 2 "" H 3638 2500 30  0000 C CNN
F 3 "" H 3600 2650 60  0000 C CNN
	1    3600 2650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56542FAF
P 5400 2500
F 0 "C?" H 5425 2600 50  0000 L CNN
F 1 "0.1uF" H 5425 2400 50  0000 L CNN
F 2 "" H 5438 2350 30  0000 C CNN
F 3 "" H 5400 2500 60  0000 C CNN
	1    5400 2500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565434D1
P 5050 2500
F 0 "C?" H 5075 2600 50  0000 L CNN
F 1 "1uF" H 5075 2400 50  0000 L CNN
F 2 "" H 5088 2350 30  0000 C CNN
F 3 "" H 5050 2500 60  0000 C CNN
	1    5050 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2750 5050 2650
Wire Wire Line
	5150 2750 5150 2300
Wire Wire Line
	5150 2300 5400 2300
Wire Wire Line
	5400 2300 5400 2350
Wire Wire Line
	5400 2650 5400 2700
Wire Wire Line
	5400 2700 5250 2700
Wire Wire Line
	5250 2700 5250 2750
Wire Wire Line
	5050 2300 5050 2350
Wire Wire Line
	3250 2300 3600 2300
Wire Wire Line
	3600 2300 3850 2300
Wire Wire Line
	3850 2300 4850 2300
Wire Wire Line
	4850 2300 5050 2300
$Comp
L GNDD #PWR?
U 1 1 56543BD9
P 3600 2950
F 0 "#PWR?" H 3600 2700 50  0001 C CNN
F 1 "GNDD" H 3600 2800 50  0000 C CNN
F 2 "" H 3600 2950 60  0000 C CNN
F 3 "" H 3600 2950 60  0000 C CNN
	1    3600 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2750 4650 2450
Wire Wire Line
	4650 2450 4400 2450
Connection ~ 3850 2300
Wire Wire Line
	4750 2750 4750 2350
Wire Wire Line
	4750 2350 4150 2350
Wire Wire Line
	4850 2750 4850 2300
Connection ~ 4850 2300
Wire Wire Line
	4150 2350 4150 2500
Wire Wire Line
	4400 2450 4400 2500
Connection ~ 3600 2300
Text Label 3250 2300 0    60   ~ 0
+VM
Wire Wire Line
	3600 2500 3600 2300
Wire Wire Line
	3850 2500 3850 2300
Wire Wire Line
	3600 2850 3850 2850
Wire Wire Line
	3850 2850 4150 2850
Wire Wire Line
	4150 2850 4400 2850
Connection ~ 3600 2850
Connection ~ 4150 2850
Connection ~ 3850 2850
$Comp
L GNDD #PWR?
U 1 1 56545BB2
P 5300 4700
F 0 "#PWR?" H 5300 4450 50  0001 C CNN
F 1 "GNDD" H 5300 4550 50  0000 C CNN
F 2 "" H 5300 4700 60  0000 C CNN
F 3 "" H 5300 4700 60  0000 C CNN
	1    5300 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 4550 5000 4600
Wire Wire Line
	5000 4600 5100 4600
Wire Wire Line
	5100 4600 5300 4600
Wire Wire Line
	5300 4550 5300 4600
Wire Wire Line
	5300 4600 5300 4700
Connection ~ 5300 4600
Wire Wire Line
	5100 4550 5100 4600
Connection ~ 5100 4600
$Comp
L R R?
U 1 1 5654705B
P 4000 3650
F 0 "R?" V 4080 3650 50  0000 C CNN
F 1 "68k" V 4000 3650 50  0000 C CNN
F 2 "" V 3930 3650 30  0000 C CNN
F 3 "" H 4000 3650 30  0000 C CNN
	1    4000 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	4150 3650 4300 3650
Wire Wire Line
	3850 3650 3550 3650
Text Label 3550 3650 0    60   ~ 0
AVDD
Text Label 4450 2450 0    60   ~ 0
AVDD
Wire Wire Line
	4400 2850 4400 2800
Wire Wire Line
	4150 2800 4150 2850
Wire Wire Line
	3850 2800 3850 2850
Wire Wire Line
	3600 2800 3600 2850
Wire Wire Line
	3600 2850 3600 2950
$EndSCHEMATC
