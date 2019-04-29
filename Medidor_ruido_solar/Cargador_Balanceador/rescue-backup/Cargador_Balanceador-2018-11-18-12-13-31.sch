EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:Cargador_Balanceador-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Cargador_Balanceador"
Date "2018-11-03"
Rev "1"
Comp "EA5IOT"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 5650 3350 2    60   Input ~ 0
CELDA1+
Text GLabel 5650 4450 2    60   Input ~ 0
CELDA2+
Text GLabel 6650 3350 0    60   Input ~ 0
CELDA1-
Text GLabel 6650 4450 0    60   Input ~ 0
CELDA2-
$Comp
L D_ALT D1
U 1 1 5BDD9DC4
P 4650 2650
F 0 "D1" H 4650 2750 50  0000 C CNN
F 1 "1N400X" H 4650 2550 50  0000 C CNN
F 2 "" H 4650 2650 50  0001 C CNN
F 3 "" H 4650 2650 50  0001 C CNN
	1    4650 2650
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 5BDD9E33
P 4650 3000
F 0 "R1" V 4730 3000 50  0000 C CNN
F 1 "10 1W" V 4550 3000 50  0000 C CNN
F 2 "" V 4580 3000 50  0001 C CNN
F 3 "" H 4650 3000 50  0001 C CNN
	1    4650 3000
	-1   0    0    1   
$EndComp
$Comp
L SW_SPDT SW1
U 1 1 5BDD9E9E
P 5200 3350
F 0 "SW1" H 5200 3520 50  0000 C CNN
F 1 "SW_SPDT" H 5200 3150 50  0001 C CNN
F 2 "" H 5200 3350 50  0001 C CNN
F 3 "" H 5200 3350 50  0001 C CNN
	1    5200 3350
	-1   0    0    -1  
$EndComp
$Comp
L SW_SPDT SW2
U 1 1 5BDD9F67
P 5200 4450
F 0 "SW2" H 5200 4620 50  0000 C CNN
F 1 "SW_SPDT" H 5200 4250 50  0001 C CNN
F 2 "" H 5200 4450 50  0001 C CNN
F 3 "" H 5200 4450 50  0001 C CNN
	1    5200 4450
	-1   0    0    -1  
$EndComp
$Comp
L SW_SPDT SW3
U 1 1 5BDD9F9E
P 7100 3350
F 0 "SW3" H 7100 3520 50  0000 C CNN
F 1 "SW_SPDT" H 7100 3150 50  0001 C CNN
F 2 "" H 7100 3350 50  0001 C CNN
F 3 "" H 7100 3350 50  0001 C CNN
	1    7100 3350
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR2
U 1 1 5BDD9FFF
P 7100 4850
F 0 "#PWR2" H 7100 4600 50  0001 C CNN
F 1 "GNDREF" H 7100 4700 50  0001 C CNN
F 2 "" H 7100 4850 50  0001 C CNN
F 3 "" H 7100 4850 50  0001 C CNN
	1    7100 4850
	1    0    0    -1  
$EndComp
$Comp
L D_ALT D2
U 1 1 5BDDA28D
P 3850 3450
F 0 "D2" H 3850 3550 50  0000 C CNN
F 1 "1N400X" H 3850 3350 50  0000 C CNN
F 2 "" H 3850 3450 50  0001 C CNN
F 3 "" H 3850 3450 50  0001 C CNN
	1    3850 3450
	1    0    0    -1  
$EndComp
Text Notes 5000 5300 0    60   ~ 0
POSICION 1: CARGADOR ON Y ALIMENTACION EXTERNA ON\nPOSICION 2: BATERIA ON
$Comp
L L7805 UX
U 1 1 5BDDA811
P 2000 2700
F 0 "UX" H 1850 2825 50  0000 C CNN
F 1 "L7805" H 2000 2825 50  0000 L CNN
F 2 "" H 2025 2550 50  0001 L CIN
F 3 "" H 2000 2650 50  0001 C CNN
	1    2000 2700
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR1
U 1 1 5BDDA92E
P 2000 3150
F 0 "#PWR1" H 2000 2900 50  0001 C CNN
F 1 "GNDREF" H 2000 3000 50  0001 C CNN
F 2 "" H 2000 3150 50  0001 C CNN
F 3 "" H 2000 3150 50  0001 C CNN
	1    2000 3150
	1    0    0    -1  
$EndComp
Text Notes 1800 2450 0    60   ~ 0
7805 EN EL\nPCB DEL\nMEDIDOR
Wire Wire Line
	6650 3350 6900 3350
Wire Wire Line
	6650 4450 7650 4450
Wire Wire Line
	7100 4450 7100 4850
Wire Wire Line
	7300 3450 7450 3450
Wire Wire Line
	7450 3450 7450 3900
Wire Wire Line
	7450 3900 4850 3900
Wire Wire Line
	4850 3900 4850 4550
Wire Wire Line
	4850 4550 5000 4550
Wire Wire Line
	5400 4450 5650 4450
Wire Wire Line
	5400 3350 5650 3350
Wire Wire Line
	7300 3250 7650 3250
Wire Wire Line
	7650 3250 7650 4450
Connection ~ 7100 4450
Wire Wire Line
	4000 3450 5000 3450
Wire Wire Line
	1300 3450 3700 3450
Wire Wire Line
	4650 3250 5000 3250
Wire Wire Line
	4350 4350 5000 4350
Wire Notes Line
	5200 3300 5200 4400
Wire Notes Line
	5200 3650 7100 3650
Wire Notes Line
	7100 3650 7100 3300
Wire Wire Line
	1300 3450 1300 2700
Wire Wire Line
	1300 2700 1700 2700
Wire Wire Line
	2000 3000 2000 3150
Wire Notes Line
	1300 2000 3000 2000
Wire Notes Line
	3000 2000 3000 4000
Wire Notes Line
	3000 4000 1300 4000
$Comp
L R R2
U 1 1 5BDDAC66
P 4350 3000
F 0 "R2" V 4430 3000 50  0000 C CNN
F 1 "10 1W" V 4250 3000 50  0000 C CNN
F 2 "" V 4280 3000 50  0001 C CNN
F 3 "" H 4350 3000 50  0001 C CNN
	1    4350 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2800 4650 2850
Wire Wire Line
	4350 3150 4350 4350
Wire Wire Line
	4650 3150 4650 4350
Connection ~ 4650 4350
Connection ~ 4650 3250
$Comp
L D_ALT D3
U 1 1 5BDD9685
P 4350 2650
F 0 "D3" H 4350 2750 50  0000 C CNN
F 1 "1N400X" H 4350 2550 50  0000 C CNN
F 2 "" H 4350 2650 50  0001 C CNN
F 3 "" H 4350 2650 50  0001 C CNN
	1    4350 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4350 2850 4350 2800
Wire Wire Line
	4650 2500 4650 2300
Wire Wire Line
	4650 2300 3850 2300
Wire Wire Line
	3850 2300 3850 2700
Wire Wire Line
	3850 2700 2300 2700
Wire Wire Line
	4350 2300 4350 2500
Connection ~ 4350 2300
$EndSCHEMATC
