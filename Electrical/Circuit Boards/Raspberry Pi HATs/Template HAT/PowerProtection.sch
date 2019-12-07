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
LIBS:opto
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
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
L Q_PMOS_GSD Q2
U 1 1 5939A4E1
P 3450 2000
F 0 "Q2" V 3400 2100 50  0000 L CNN
F 1 "DMG2305UX" V 3700 1750 50  0000 L CNN
F 2 "modules:SOT-23" V 3650 2100 50  0001 C CNN
F 3 "" H 3450 2000 50  0001 C CNN
	1    3450 2000
	0    -1   -1   0   
$EndComp
$Comp
L DMMT5401 Q1
U 1 1 5939C269
P 3000 2350
F 0 "Q1" H 3400 2250 50  0000 L CNN
F 1 "DMMT5401" H 3250 2350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6_Handsoldering" H 3075 2200 50  0001 C CNN
F 3 "" H 3275 2300 50  0001 C CNN
	1    3000 2350
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5939C46F
P 3150 2850
F 0 "R4" V 3230 2850 50  0000 C CNN
F 1 "10K" V 3150 2850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3080 2850 50  0001 C CNN
F 3 "" H 3150 2850 50  0001 C CNN
	1    3150 2850
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5939C4DE
P 3750 2850
F 0 "R5" V 3830 2850 50  0000 C CNN
F 1 "47K" V 3750 2850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3680 2850 50  0001 C CNN
F 3 "" H 3750 2850 50  0001 C CNN
	1    3750 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR8
U 1 1 5939C545
P 3450 3000
F 0 "#PWR8" H 3450 2750 50  0001 C CNN
F 1 "GND" H 3450 2850 50  0000 C CNN
F 2 "" H 3450 3000 50  0001 C CNN
F 3 "" H 3450 3000 50  0001 C CNN
	1    3450 3000
	1    0    0    -1  
$EndComp
Wire Notes Line
	3400 1500 3400 1700
Wire Notes Line
	3200 1600 3400 1600
Connection ~ 3300 2650
Connection ~ 3150 2650
Wire Wire Line
	3300 2650 3300 2600
Wire Wire Line
	3150 2650 3300 2650
Wire Wire Line
	3300 2700 3600 2700
Wire Wire Line
	3600 2650 3600 2600
Wire Wire Line
	3450 2700 3750 2700
Wire Wire Line
	3450 2200 3450 2700
Connection ~ 3450 3000
Wire Wire Line
	3150 3000 3450 3000
Wire Wire Line
	3450 3000 3750 3000
Wire Wire Line
	3750 2700 3750 2600
Connection ~ 3750 1900
Wire Wire Line
	3750 2200 3750 1900
Connection ~ 3150 1900
Wire Wire Line
	3150 2200 3150 1900
Wire Notes Line
	3400 1500 3600 1600
Wire Notes Line
	3600 1600 3400 1700
Wire Notes Line
	3600 1600 3750 1600
Text Notes 3500 1500 2    60   ~ 0
~~0V
Text HLabel 3000 1900 0    60   Input ~ 0
Source
Text HLabel 3900 1900 2    60   Output ~ 0
Target
Wire Wire Line
	3650 1900 3750 1900
Wire Wire Line
	3750 1900 3900 1900
Wire Wire Line
	3000 1900 3150 1900
Wire Wire Line
	3150 1900 3250 1900
Wire Notes Line
	3600 1500 3600 1700
Wire Wire Line
	3150 2600 3150 2700
$EndSCHEMATC
