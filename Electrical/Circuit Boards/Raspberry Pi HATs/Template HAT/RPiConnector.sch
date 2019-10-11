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
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 1400 1500 0    60   ~ 0
Pi3
Text Label 1400 1600 0    60   ~ 0
Pi5
Text Label 1400 1700 0    60   ~ 0
Pi7
Text Label 1400 1900 0    60   ~ 0
Pi11
Text Label 1400 2000 0    60   ~ 0
Pi13
Text Label 1400 2100 0    60   ~ 0
Pi15
Text Label 1400 2300 0    60   ~ 0
Pi19
Text Label 1400 2400 0    60   ~ 0
Pi21
Text Label 1400 2500 0    60   ~ 0
Pi23
$Comp
L GND #PWR04
U 1 1 59374B19
P 2900 3000
F 0 "#PWR04" H 2900 2750 50  0001 C CNN
F 1 "GND" H 2900 2850 50  0000 C CNN
F 2 "" H 2900 3000 50  0000 C CNN
F 3 "" H 2900 3000 50  0000 C CNN
	1    2900 3000
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X20 J2
U 1 1 59374B1F
P 2100 2350
F 0 "J2" H 2100 3400 50  0000 C CNN
F 1 "CONN_02X20" V 2100 2350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x20_Pitch2.54mm" V 2100 1400 50  0001 C CNN
F 3 "" H 2100 1400 50  0001 C CNN
	1    2100 2350
	1    0    0    -1  
$EndComp
Text Label 1400 2700 0    60   ~ 0
Pi27
Text Label 1400 2800 0    60   ~ 0
Pi29
Text Label 1400 2900 0    60   ~ 0
Pi31
Text Label 1400 3000 0    60   ~ 0
Pi33
Text Label 1400 3100 0    60   ~ 0
Pi35
Text Label 1400 3200 0    60   ~ 0
Pi37
Text Label 2750 1700 2    60   ~ 0
Pi8
Text Label 2750 1900 2    60   ~ 0
Pi12
Text Label 2750 2100 2    60   ~ 0
Pi16
Text Label 2750 2400 2    60   ~ 0
Pi22
Text Label 2750 2500 2    60   ~ 0
Pi24
Text Label 2750 1800 2    60   ~ 0
Pi10
Text Label 2750 2200 2    60   ~ 0
Pi18
Text Label 2750 2600 2    60   ~ 0
Pi26
Text Label 2750 2700 2    60   ~ 0
Pi28
Text Label 2750 2900 2    60   ~ 0
Pi32
Text Label 2750 3100 2    60   ~ 0
Pi36
Text Label 2750 3200 2    60   ~ 0
Pi38
Text Label 2750 3300 2    60   ~ 0
Pi40
$Comp
L GND #PWR05
U 1 1 59374B39
P 1350 3300
F 0 "#PWR05" H 1350 3050 50  0001 C CNN
F 1 "GND" H 1350 3150 50  0000 C CNN
F 2 "" H 1350 3300 50  0000 C CNN
F 3 "" H 1350 3300 50  0000 C CNN
	1    1350 3300
	1    0    0    -1  
$EndComp
Text Label 3550 1500 0    60   ~ 0
Pi3
Text Label 3550 1750 0    60   ~ 0
Pi5
Text Label 3550 2000 0    60   ~ 0
Pi7
Text Label 3550 2500 0    60   ~ 0
Pi11
Text Label 3550 2750 0    60   ~ 0
Pi13
Text Label 3550 3000 0    60   ~ 0
Pi15
Text Label 3550 3500 0    60   ~ 0
Pi19
Text Label 3550 3750 0    60   ~ 0
Pi21
Text Label 3550 4000 0    60   ~ 0
Pi23
Text Label 3550 4500 0    60   ~ 0
Pi27
Text Label 3550 4750 0    60   ~ 0
Pi29
Text Label 3550 5000 0    60   ~ 0
Pi31
Text Label 3550 5250 0    60   ~ 0
Pi33
Text Label 3550 5500 0    60   ~ 0
Pi35
Text Label 3550 5750 0    60   ~ 0
Pi37
Text HLabel 4050 1500 2    60   BiDi ~ 0
GPIO2
Text HLabel 4050 1600 2    60   BiDi ~ 0
SDA1
Text HLabel 4050 1750 2    60   BiDi ~ 0
GPIO3
Text HLabel 4050 1850 2    60   BiDi ~ 0
SCL1
Text HLabel 4050 2000 2    60   BiDi ~ 0
GPIO4
Text HLabel 4050 2100 2    60   Output ~ 0
GCLK
Text HLabel 4050 2500 2    60   BiDi ~ 0
GPIO17
Text HLabel 4050 2600 2    60   Output ~ 0
GEN0
Text HLabel 4050 2750 2    60   BiDi ~ 0
GPIO27
Text HLabel 4050 2850 2    60   Output ~ 0
GEN2
Text HLabel 4050 3100 2    60   Output ~ 0
GEN3
Text HLabel 4050 3000 2    60   BiDi ~ 0
GPIO22
Text HLabel 4050 3500 2    60   BiDi ~ 0
GPIO10
Text HLabel 4050 3600 2    60   Output ~ 0
MOSI
Text HLabel 4050 3850 2    60   Input ~ 0
MISO
Text HLabel 4050 3750 2    60   BiDi ~ 0
GPIO9
Text HLabel 4050 4000 2    60   BiDi ~ 0
GPIO11
Text HLabel 4050 4100 2    60   Output ~ 0
SCLK
Text HLabel 4050 4750 2    60   BiDi ~ 0
GPIO5
Text HLabel 4050 5000 2    60   BiDi ~ 0
GPIO6
Text HLabel 4050 5250 2    60   BiDi ~ 0
GPIO13
Text HLabel 4050 5500 2    60   BiDi ~ 0
GPIO19
Text HLabel 4050 5750 2    60   BiDi ~ 0
GPIO26
Text Label 4700 2000 0    60   ~ 0
Pi8
Text Label 4700 2250 0    60   ~ 0
Pi10
Text Label 4700 2500 0    60   ~ 0
Pi12
Text Label 4700 3000 0    60   ~ 0
Pi16
Text Label 4700 3250 0    60   ~ 0
Pi18
Text Label 4700 3750 0    60   ~ 0
Pi22
Text Label 4700 4000 0    60   ~ 0
Pi24
Text Label 4700 4250 0    60   ~ 0
Pi26
Text Label 4700 4500 0    60   ~ 0
Pi28
Text Label 4700 5000 0    60   ~ 0
Pi32
Text Label 4700 5750 0    60   ~ 0
Pi38
Text Label 4700 6000 0    60   ~ 0
Pi40
Text HLabel 5200 6000 2    60   BiDi ~ 0
GPIO21
Text HLabel 5200 5750 2    60   BiDi ~ 0
GPIO20
Text HLabel 5200 5500 2    60   BiDi ~ 0
GPIO16
Text HLabel 5200 5000 2    60   BiDi ~ 0
GPIO12
Text HLabel 5200 4250 2    60   BiDi ~ 0
GPIO7
Text HLabel 5200 4000 2    60   BiDi ~ 0
GPIO8
Text HLabel 5200 3750 2    60   BiDi ~ 0
GPIO25
Text HLabel 5200 3250 2    60   BiDi ~ 0
GPIO24
Text HLabel 5200 3000 2    60   BiDi ~ 0
GPIO23
Text HLabel 5200 2500 2    60   BiDi ~ 0
GPIO18
Text HLabel 5200 2250 2    60   BiDi ~ 0
GPIO15
Text HLabel 5200 2000 2    60   BiDi ~ 0
GPIO14
Text Label 4700 5500 0    60   ~ 0
Pi36
Text HLabel 5200 4350 2    60   BiDi ~ 0
~CS1
Text HLabel 5200 4100 2    60   BiDi ~ 0
~CS0
Text HLabel 5200 3850 2    60   BiDi ~ 0
GEN6
Text HLabel 5200 3350 2    60   BiDi ~ 0
GEN5
Text HLabel 5200 3100 2    60   BiDi ~ 0
GEN4
Text HLabel 5200 2600 2    60   BiDi ~ 0
GEN1
Text HLabel 5200 2350 2    60   BiDi ~ 0
RXD0
Text HLabel 5200 2100 2    60   BiDi ~ 0
TXD0
Wire Wire Line
	1750 1250 1750 2200
Wire Wire Line
	1750 1400 1850 1400
Wire Wire Line
	2500 1250 2500 1500
Connection ~ 2500 1400
Wire Wire Line
	2500 1400 2350 1400
Wire Wire Line
	2500 1500 2350 1500
Wire Wire Line
	2750 1700 2350 1700
Wire Wire Line
	2750 1800 2350 1800
Wire Wire Line
	2750 1900 2350 1900
Wire Wire Line
	2350 2000 2800 2000
Wire Wire Line
	2750 2100 2350 2100
Wire Wire Line
	2750 2200 2350 2200
Wire Wire Line
	2800 2300 2350 2300
Wire Wire Line
	2750 2400 2350 2400
Wire Wire Line
	2750 2500 2350 2500
Wire Wire Line
	2750 2600 2350 2600
Wire Wire Line
	2750 2700 2350 2700
Wire Wire Line
	2800 2800 2350 2800
Wire Wire Line
	2750 2900 2350 2900
Wire Wire Line
	2350 3000 2900 3000
Wire Wire Line
	2750 3100 2350 3100
Wire Wire Line
	2750 3200 2350 3200
Wire Wire Line
	2750 3300 2350 3300
Wire Wire Line
	1400 1500 1850 1500
Wire Wire Line
	1850 1600 1400 1600
Wire Wire Line
	1400 1700 1850 1700
Wire Wire Line
	1350 1800 1850 1800
Wire Wire Line
	1400 1900 1850 1900
Wire Wire Line
	1850 2000 1400 2000
Wire Wire Line
	1400 2100 1850 2100
Wire Wire Line
	1850 2300 1400 2300
Wire Wire Line
	1400 2400 1850 2400
Wire Wire Line
	1850 2500 1400 2500
Wire Wire Line
	1350 2600 1850 2600
Wire Wire Line
	1850 2700 1400 2700
Wire Wire Line
	1850 2800 1400 2800
Wire Wire Line
	1400 2900 1850 2900
Wire Wire Line
	1850 3000 1400 3000
Wire Wire Line
	1400 3100 1850 3100
Wire Wire Line
	1850 3200 1400 3200
Wire Wire Line
	1350 3300 1850 3300
Wire Wire Line
	1350 1800 1350 3300
Connection ~ 1350 2600
Wire Wire Line
	2800 1600 2800 3000
Connection ~ 2800 2800
Connection ~ 2800 2300
Wire Wire Line
	2800 1600 2350 1600
Connection ~ 2800 2000
Connection ~ 2800 3000
Wire Wire Line
	3550 1500 4050 1500
Wire Wire Line
	3750 1500 3750 1600
Wire Wire Line
	3750 1600 4050 1600
Connection ~ 3750 1500
Wire Wire Line
	3550 1750 4050 1750
Wire Wire Line
	3750 1750 3750 1850
Wire Wire Line
	3750 1850 4050 1850
Connection ~ 3750 1750
Wire Wire Line
	3550 2000 4050 2000
Wire Wire Line
	3750 2000 3750 2100
Wire Wire Line
	3750 2100 4050 2100
Connection ~ 3750 2000
Wire Wire Line
	3550 2500 4050 2500
Wire Wire Line
	3750 2500 3750 2600
Wire Wire Line
	3750 2600 4050 2600
Connection ~ 3750 2500
Wire Wire Line
	3550 2750 4050 2750
Wire Wire Line
	3750 2750 3750 2850
Wire Wire Line
	3750 2850 4050 2850
Connection ~ 3750 2750
Wire Wire Line
	3550 3000 4050 3000
Wire Wire Line
	3750 3000 3750 3100
Wire Wire Line
	3750 3100 4050 3100
Connection ~ 3750 3000
Wire Wire Line
	4700 2000 5200 2000
Wire Wire Line
	4900 2000 4900 2100
Wire Wire Line
	4900 2100 5200 2100
Connection ~ 4900 2000
Wire Wire Line
	4700 2250 5200 2250
Wire Wire Line
	4900 2250 4900 2350
Wire Wire Line
	4900 2350 5200 2350
Connection ~ 4900 2250
Wire Wire Line
	4700 2500 5200 2500
Wire Wire Line
	4900 2500 4900 2600
Wire Wire Line
	4900 2600 5200 2600
Connection ~ 4900 2500
Wire Wire Line
	4700 3000 5200 3000
Wire Wire Line
	4900 3000 4900 3100
Wire Wire Line
	4900 3100 5200 3100
Connection ~ 4900 3000
Wire Wire Line
	4700 3250 5200 3250
Wire Wire Line
	4900 3250 4900 3350
Wire Wire Line
	4900 3350 5200 3350
Connection ~ 4900 3250
Wire Wire Line
	3550 3500 4050 3500
Wire Wire Line
	3750 3500 3750 3600
Wire Wire Line
	3750 3600 4050 3600
Connection ~ 3750 3500
Wire Wire Line
	3550 3750 4050 3750
Wire Wire Line
	3750 3750 3750 3850
Wire Wire Line
	3750 3850 4050 3850
Connection ~ 3750 3750
Wire Wire Line
	3550 4000 4050 4000
Wire Wire Line
	3750 4000 3750 4100
Wire Wire Line
	3750 4100 4050 4100
Connection ~ 3750 4000
Wire Wire Line
	3550 4500 4050 4500
Wire Wire Line
	3550 4750 4050 4750
Wire Wire Line
	3550 5000 4050 5000
Wire Wire Line
	3550 5250 4050 5250
Wire Wire Line
	3550 5500 4050 5500
Wire Wire Line
	3550 5750 4050 5750
Wire Wire Line
	4700 3750 5200 3750
Wire Wire Line
	4900 3750 4900 3850
Wire Wire Line
	4900 3850 5200 3850
Connection ~ 4900 3750
Wire Wire Line
	4700 4000 5200 4000
Wire Wire Line
	4900 4000 4900 4100
Wire Wire Line
	4900 4100 5200 4100
Connection ~ 4900 4000
Wire Wire Line
	4700 4250 5200 4250
Wire Wire Line
	4900 4250 4900 4350
Wire Wire Line
	4900 4350 5200 4350
Connection ~ 4900 4250
Wire Wire Line
	4700 4500 5200 4500
Wire Wire Line
	4700 5000 5200 5000
Wire Wire Line
	4700 5500 5200 5500
Wire Wire Line
	4700 5750 5200 5750
Wire Wire Line
	4700 6000 5200 6000
Wire Wire Line
	1750 2200 1850 2200
Connection ~ 1750 1400
Wire Bus Line
	1800 4150 1800 4350
Wire Bus Line
	1800 4350 2050 4350
Text HLabel 2050 4350 2    60   BiDi ~ 0
I2C1
Entry Wire Line
	1700 4050 1800 4150
Wire Wire Line
	1700 4050 1450 4050
Entry Wire Line
	1700 4200 1800 4300
Wire Wire Line
	1700 4200 1450 4200
Text Label 1450 4050 0    60   ~ 0
SCL1
Text Label 1450 4200 0    60   ~ 0
SDA1
Wire Bus Line
	1800 4700 1800 4900
Wire Bus Line
	1800 4900 2050 4900
Text HLabel 2050 4900 2    60   BiDi ~ 0
I2C_ID
Entry Wire Line
	1700 4600 1800 4700
Wire Wire Line
	1700 4600 1450 4600
Entry Wire Line
	1700 4750 1800 4850
Wire Wire Line
	1700 4750 1450 4750
Text Label 1450 4600 0    60   ~ 0
ID_SC
Text Label 1450 4750 0    60   ~ 0
ID_SD
Text HLabel 1750 1250 1    60   Input ~ 0
+3.3_Pi
Text HLabel 2500 1250 1    60   Input ~ 0
+5V_Pi
Text Label 5200 4500 0    60   ~ 0
ID_SC
Text Label 4050 4500 0    60   ~ 0
ID_SD
$EndSCHEMATC
