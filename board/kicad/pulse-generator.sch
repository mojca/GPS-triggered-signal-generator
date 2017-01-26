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
LIBS:arduino
LIBS:adafruit_ultimate_gps_breakout_v3
LIBS:2N7000
LIBS:pulse-generator-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L arduino_mini U1
U 1 1 5888B2BC
P 2050 2100
F 0 "U1" H 2550 1150 70  0000 C CNN
F 1 "Arduino nano" H 2800 1050 70  0000 C CNN
F 2 "arduino:arduino_mini" H 2050 2050 60  0000 C CNN
F 3 "" H 2050 2100 60  0000 C CNN
	1    2050 2100
	1    0    0    -1  
$EndComp
NoConn ~ 1900 950 
NoConn ~ 2750 2900
$Comp
L GND #PWR01
U 1 1 5888B37B
P 2050 3650
F 0 "#PWR01" H 2050 3400 50  0001 C CNN
F 1 "GND" H 2050 3500 50  0000 C CNN
F 2 "" H 2050 3650 50  0000 C CNN
F 3 "" H 2050 3650 50  0000 C CNN
	1    2050 3650
	1    0    0    -1  
$EndComp
NoConn ~ 2200 950 
NoConn ~ 2050 950 
$Comp
L CONN_02X05 P2
U 1 1 5888B3E0
P 1800 4600
F 0 "P2" H 1800 4900 50  0000 C CNN
F 1 "CONN_02X05" H 1800 4300 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_2x05" H 1800 3400 50  0001 C CNN
F 3 "" H 1800 3400 50  0000 C CNN
	1    1800 4600
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P1
U 1 1 5888B4C7
P 900 800
F 0 "P1" H 900 1050 50  0000 C CNN
F 1 "I2C" V 1000 800 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x04" H 900 800 50  0001 C CNN
F 3 "" H 900 800 50  0000 C CNN
	1    900  800 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR02
U 1 1 5888B598
P 750 1250
F 0 "#PWR02" H 750 1000 50  0001 C CNN
F 1 "GND" H 750 1100 50  0000 C CNN
F 2 "" H 750 1250 50  0000 C CNN
F 3 "" H 750 1250 50  0000 C CNN
	1    750  1250
	1    0    0    -1  
$EndComp
Text Label 950  1200 1    60   ~ 0
SDA
Text Label 1050 1200 1    60   ~ 0
SCL
Text Label 1050 2300 0    60   ~ 0
SDA
Text Label 1050 2400 0    60   ~ 0
SCL
NoConn ~ 1350 2200
NoConn ~ 1350 2100
NoConn ~ 1350 2000
NoConn ~ 1350 1900
NoConn ~ 1350 1700
Text Label 2950 1450 0    60   ~ 0
CTRLa
NoConn ~ 2750 1550
NoConn ~ 2750 1650
NoConn ~ 2750 1750
NoConn ~ 2750 1850
NoConn ~ 2750 1950
Text Label 3000 2400 0    60   ~ 0
BUTTON1b
Text Label 3000 2150 0    60   ~ 0
BUTTON2b
Text Label 3000 2050 0    60   ~ 0
BUTTON3b
NoConn ~ 2750 2500
NoConn ~ 2750 2700
Text Label 3000 2600 0    60   ~ 0
PPS
$Comp
L 74HC74 U2
U 1 1 5888C041
P 4300 4450
F 0 "U2" H 4450 4750 50  0000 C CNN
F 1 "74HC74" H 4600 4155 50  0000 C CNN
F 2 "" H 4300 4450 50  0001 C CNN
F 3 "" H 4300 4450 50  0000 C CNN
	1    4300 4450
	1    0    0    -1  
$EndComp
Text Label 3400 4250 0    60   ~ 0
CTRLa
Text Label 3400 4450 0    60   ~ 0
PPS
NoConn ~ 4900 4650
$Comp
L 74HC74 U2
U 2 1 5888D95E
P 5300 5700
F 0 "U2" H 5450 6000 50  0000 C CNN
F 1 "74HC74" H 5600 5405 50  0000 C CNN
F 2 "" H 5300 5700 50  0001 C CNN
F 3 "" H 5300 5700 50  0000 C CNN
	2    5300 5700
	1    0    0    -1  
$EndComp
NoConn ~ 5900 5900
NoConn ~ 5900 5500
NoConn ~ 5300 5150
$Comp
L GND #PWR03
U 1 1 5888D9AA
P 4600 5700
F 0 "#PWR03" H 4600 5450 50  0001 C CNN
F 1 "GND" H 4600 5550 50  0000 C CNN
F 2 "" H 4600 5700 50  0000 C CNN
F 3 "" H 4600 5700 50  0000 C CNN
	1    4600 5700
	0    1    1    0   
$EndComp
$Comp
L GND #PWR04
U 1 1 5888D9D0
P 4600 5500
F 0 "#PWR04" H 4600 5250 50  0001 C CNN
F 1 "GND" H 4600 5350 50  0000 C CNN
F 2 "" H 4600 5500 50  0000 C CNN
F 3 "" H 4600 5500 50  0000 C CNN
	1    4600 5500
	0    1    1    0   
$EndComp
NoConn ~ 5300 6250
$Comp
L GND #PWR05
U 1 1 58890667
P 1400 4800
F 0 "#PWR05" H 1400 4550 50  0001 C CNN
F 1 "GND" H 1400 4650 50  0000 C CNN
F 2 "" H 1400 4800 50  0000 C CNN
F 3 "" H 1400 4800 50  0000 C CNN
	1    1400 4800
	0    1    1    0   
$EndComp
Text Label 1300 4600 0    60   ~ 0
SDA
Text Label 1300 4500 0    60   ~ 0
SCL
Text Label 2150 4600 0    60   ~ 0
BUTTON3b
Text Label 2150 4700 0    60   ~ 0
BUTTON2b
Text Label 2150 4800 0    60   ~ 0
BUTTON1b
$Comp
L Adafruit_Ultimate_GPS_Breakout_v3 GPS1
U 1 1 588911BA
P 4950 1850
F 0 "GPS1" H 4350 2500 60  0000 C CNN
F 1 "Adafruit_Ultimate_GPS_Breakout_v3" V 4800 1800 60  0000 C CNN
F 2 "" H 4800 1800 60  0001 C CNN
F 3 "" H 4800 1800 60  0000 C CNN
	1    4950 1850
	1    0    0    -1  
$EndComp
NoConn ~ 4050 1450
NoConn ~ 4050 1550
NoConn ~ 4050 1650
NoConn ~ 4050 1750
NoConn ~ 4050 1950
$Comp
L GND #PWR06
U 1 1 5889126D
P 3900 2050
F 0 "#PWR06" H 3900 1800 50  0001 C CNN
F 1 "GND" H 3900 1900 50  0000 C CNN
F 2 "" H 3900 2050 50  0000 C CNN
F 3 "" H 3900 2050 50  0000 C CNN
	1    3900 2050
	0    1    1    0   
$EndComp
Text Label 3800 2250 0    60   ~ 0
PPS
Text Label 3000 2800 0    60   ~ 0
NEMA
Text Label 3800 1850 0    60   ~ 0
NEMA
$Comp
L R R1
U 1 1 58891AEE
P 5800 3600
F 0 "R1" V 5880 3600 50  0000 C CNN
F 1 "R" V 5800 3600 50  0000 C CNN
F 2 "" V 5730 3600 50  0001 C CNN
F 3 "" H 5800 3600 50  0000 C CNN
	1    5800 3600
	0    1    1    0   
$EndComp
$Comp
L 2N7000 S1
U 1 1 58891C02
P 6950 3500
F 0 "S1" H 7000 3600 50  0000 L CNN
F 1 "2N7000" H 6950 3500 50  0001 L CNN
F 2 "TO-92EXPANDED" H 6950 3500 50  0001 L CNN
F 3 "TO-92 STMicroelectronics" H 6950 3500 50  0001 L CNN
F 4 "Unavailable" H 6950 3500 50  0001 L CNN "Availability"
F 5 "None" H 6950 3500 50  0001 L CNN "Price"
F 6 "2N7000" H 6950 3500 50  0001 L CNN "MP"
F 7 "N-channel 60 V, 1.8 Ohm, 0.35 A STripFET(TM) II Power MOSFET in a TO-92 package" H 6950 3500 50  0001 L CNN "Description"
F 8 "STMicroelectronics" H 6950 3500 50  0001 L CNN "MF"
	1    6950 3500
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 58891C93
P 6950 3950
F 0 "R3" V 7030 3950 50  0000 C CNN
F 1 "1R" V 6950 3950 50  0000 C CNN
F 2 "" V 6880 3950 50  0001 C CNN
F 3 "" H 6950 3950 50  0000 C CNN
	1    6950 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 58891D15
P 6950 4250
F 0 "#PWR07" H 6950 4000 50  0001 C CNN
F 1 "GND" H 6950 4100 50  0000 C CNN
F 2 "" H 6950 4250 50  0000 C CNN
F 3 "" H 6950 4250 50  0000 C CNN
	1    6950 4250
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 58891D71
P 7200 2850
F 0 "D1" H 7200 2950 50  0000 C CNN
F 1 "D" H 7200 2750 50  0000 C CNN
F 2 "" H 7200 2850 50  0001 C CNN
F 3 "" H 7200 2850 50  0000 C CNN
	1    7200 2850
	0    1    1    0   
$EndComp
$Comp
L CONN_01X01 P4
U 1 1 58891DCF
P 7600 3000
F 0 "P4" H 7600 3100 50  0000 C CNN
F 1 "CONN_01X01" V 7700 3000 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01" H 7600 3000 50  0001 C CNN
F 3 "" H 7600 3000 50  0000 C CNN
	1    7600 3000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P3
U 1 1 58891E11
P 7600 2600
F 0 "P3" H 7600 2700 50  0000 C CNN
F 1 "CONN_01X01" V 7700 2600 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01" H 7600 2600 50  0001 C CNN
F 3 "" H 7600 2600 50  0000 C CNN
	1    7600 2600
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 58891EE9
P 6850 2600
F 0 "R2" V 6930 2600 50  0000 C CNN
F 1 "33R" V 6850 2600 50  0000 C CNN
F 2 "" V 6780 2600 50  0001 C CNN
F 3 "" H 6850 2600 50  0000 C CNN
	1    6850 2600
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 58892135
P 6300 2750
F 0 "C1" H 6325 2850 50  0000 L CNN
F 1 "100n" H 6325 2650 50  0000 L CNN
F 2 "" H 6338 2600 50  0001 C CNN
F 3 "" H 6300 2750 50  0000 C CNN
	1    6300 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 588921A2
P 6300 3000
F 0 "#PWR08" H 6300 2750 50  0001 C CNN
F 1 "GND" H 6300 2850 50  0000 C CNN
F 2 "" H 6300 3000 50  0000 C CNN
F 3 "" H 6300 3000 50  0000 C CNN
	1    6300 3000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR09
U 1 1 5889231E
P 6300 2450
F 0 "#PWR09" H 6300 2300 50  0001 C CNN
F 1 "+3.3V" H 6300 2590 50  0000 C CNN
F 2 "" H 6300 2450 50  0000 C CNN
F 3 "" H 6300 2450 50  0000 C CNN
	1    6300 2450
	1    0    0    -1  
$EndComp
NoConn ~ 1350 2500
NoConn ~ 1350 2600
$Comp
L PWR_FLAG #FLG010
U 1 1 58892F73
P 6150 1050
F 0 "#FLG010" H 6150 1145 50  0001 C CNN
F 1 "PWR_FLAG" H 6150 1230 50  0000 C CNN
F 2 "" H 6150 1050 50  0000 C CNN
F 3 "" H 6150 1050 50  0000 C CNN
	1    6150 1050
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG011
U 1 1 588930C9
P 6600 1050
F 0 "#FLG011" H 6600 1145 50  0001 C CNN
F 1 "PWR_FLAG" H 6600 1230 50  0000 C CNN
F 2 "" H 6600 1050 50  0000 C CNN
F 3 "" H 6600 1050 50  0000 C CNN
	1    6600 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 588930F9
P 6150 1150
F 0 "#PWR012" H 6150 900 50  0001 C CNN
F 1 "GND" H 6150 1000 50  0000 C CNN
F 2 "" H 6150 1150 50  0000 C CNN
F 3 "" H 6150 1150 50  0000 C CNN
	1    6150 1150
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR013
U 1 1 5889388B
P 6600 1150
F 0 "#PWR013" H 6600 1000 50  0001 C CNN
F 1 "VCC" H 6600 1300 50  0000 C CNN
F 2 "" H 6600 1150 50  0000 C CNN
F 3 "" H 6600 1150 50  0000 C CNN
	1    6600 1150
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR014
U 1 1 58893964
P 850 1250
F 0 "#PWR014" H 850 1100 50  0001 C CNN
F 1 "VCC" H 850 1400 50  0000 C CNN
F 2 "" H 850 1250 50  0000 C CNN
F 3 "" H 850 1250 50  0000 C CNN
	1    850  1250
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR015
U 1 1 588939A3
P 3900 2150
F 0 "#PWR015" H 3900 2000 50  0001 C CNN
F 1 "VCC" H 3900 2300 50  0000 C CNN
F 2 "" H 3900 2150 50  0000 C CNN
F 3 "" H 3900 2150 50  0000 C CNN
	1    3900 2150
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR016
U 1 1 58893A8A
P 4300 3800
F 0 "#PWR016" H 4300 3650 50  0001 C CNN
F 1 "VCC" H 4300 3950 50  0000 C CNN
F 2 "" H 4300 3800 50  0000 C CNN
F 3 "" H 4300 3800 50  0000 C CNN
	1    4300 3800
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR017
U 1 1 58893AC9
P 1400 4700
F 0 "#PWR017" H 1400 4550 50  0001 C CNN
F 1 "VCC" H 1400 4850 50  0000 C CNN
F 2 "" H 1400 4700 50  0000 C CNN
F 3 "" H 1400 4700 50  0000 C CNN
	1    1400 4700
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR018
U 1 1 58893D21
P 4300 5050
F 0 "#PWR018" H 4300 4900 50  0001 C CNN
F 1 "VCC" H 4300 5200 50  0000 C CNN
F 2 "" H 4300 5050 50  0000 C CNN
F 3 "" H 4300 5050 50  0000 C CNN
	1    4300 5050
	-1   0    0    1   
$EndComp
NoConn ~ 1350 3400
NoConn ~ 1350 3300
$Comp
L VCC #PWR019
U 1 1 5889A688
P 6100 1600
F 0 "#PWR019" H 6100 1450 50  0001 C CNN
F 1 "VCC" H 6100 1750 50  0000 C CNN
F 2 "" H 6100 1600 50  0000 C CNN
F 3 "" H 6100 1600 50  0000 C CNN
	1    6100 1600
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR020
U 1 1 5889A763
P 6900 1600
F 0 "#PWR020" H 6900 1450 50  0001 C CNN
F 1 "+3.3V" H 6900 1740 50  0000 C CNN
F 2 "" H 6900 1600 50  0000 C CNN
F 3 "" H 6900 1600 50  0000 C CNN
	1    6900 1600
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 5889A8A3
P 6150 1800
F 0 "C2" H 6175 1900 50  0000 L CNN
F 1 "C" H 6175 1700 50  0000 L CNN
F 2 "" H 6188 1650 50  0000 C CNN
F 3 "" H 6150 1800 50  0000 C CNN
	1    6150 1800
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5889A8DB
P 6850 1800
F 0 "C3" H 6875 1900 50  0000 L CNN
F 1 "C" H 6875 1700 50  0000 L CNN
F 2 "" H 6888 1650 50  0000 C CNN
F 3 "" H 6850 1800 50  0000 C CNN
	1    6850 1800
	1    0    0    -1  
$EndComp
$Comp
L MCP1754ST-3302E/MB U3
U 1 1 5889A92A
P 6500 1600
F 0 "U3" H 6600 1450 50  0000 C CNN
F 1 "MCP1754ST-3302E/MB" H 6500 1750 50  0000 C CNN
F 2 "" H 6500 1600 50  0000 C CNN
F 3 "" H 6500 1600 50  0000 C CNN
	1    6500 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 5889A9C2
P 6150 2000
F 0 "#PWR021" H 6150 1750 50  0001 C CNN
F 1 "GND" H 6150 1850 50  0000 C CNN
F 2 "" H 6150 2000 50  0000 C CNN
F 3 "" H 6150 2000 50  0000 C CNN
	1    6150 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5889A9F8
P 6500 2000
F 0 "#PWR022" H 6500 1750 50  0001 C CNN
F 1 "GND" H 6500 1850 50  0000 C CNN
F 2 "" H 6500 2000 50  0000 C CNN
F 3 "" H 6500 2000 50  0000 C CNN
	1    6500 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 5889AA2E
P 6850 2000
F 0 "#PWR023" H 6850 1750 50  0001 C CNN
F 1 "GND" H 6850 1850 50  0000 C CNN
F 2 "" H 6850 2000 50  0000 C CNN
F 3 "" H 6850 2000 50  0000 C CNN
	1    6850 2000
	1    0    0    -1  
$EndComp
Text Notes 7200 1700 0    60   ~ 0
Zamenjaj komponento za\nMCP1700-3302E/TO
Text Notes 4400 5000 0    60   ~ 0
There should be capacitor\nbetween VCC and GND
Text Notes 700  4650 0    60   ~ 0
LED:\n1 - power\n2 - pps\n3 - coil
$Comp
L R R5
U 1 1 5889E51F
P 2300 4500
F 0 "R5" V 2380 4500 50  0000 C CNN
F 1 "75R" V 2300 4500 50  0000 C CNN
F 2 "" V 2230 4500 50  0000 C CNN
F 3 "" H 2300 4500 50  0000 C CNN
	1    2300 4500
	0    1    1    0   
$EndComp
Text Label 2550 4500 0    60   ~ 0
PPS
$Comp
L R R4
U 1 1 5889E757
P 2300 4400
F 0 "R4" V 2380 4400 50  0000 C CNN
F 1 "180R" V 2300 4400 50  0000 C CNN
F 2 "" V 2230 4400 50  0000 C CNN
F 3 "" H 2300 4400 50  0000 C CNN
	1    2300 4400
	0    1    1    0   
$EndComp
Text Label 5300 4250 0    60   ~ 0
LED_COIL
Text Label 2550 4400 0    60   ~ 0
LED_COIL
Connection ~ 7200 2600
Wire Wire Line
	7200 2600 7200 2700
Wire Wire Line
	7000 2600 7400 2600
Connection ~ 7200 3000
Wire Wire Line
	6950 3000 7400 3000
Wire Wire Line
	6950 3300 6950 3000
Wire Wire Line
	6950 4250 6950 4100
Wire Wire Line
	6950 3700 6950 3800
Wire Wire Line
	5950 3600 6750 3600
Wire Wire Line
	5100 3600 5650 3600
Wire Wire Line
	2750 2800 3000 2800
Wire Wire Line
	3800 1850 4050 1850
Wire Wire Line
	3900 2050 4050 2050
Wire Wire Line
	3900 2150 4050 2150
Wire Wire Line
	3800 2250 4050 2250
Wire Wire Line
	2050 4600 2600 4600
Wire Wire Line
	2050 4700 2600 4700
Wire Wire Line
	2050 4800 2600 4800
Wire Wire Line
	1300 4600 1550 4600
Wire Wire Line
	1300 4500 1550 4500
Wire Wire Line
	1400 4800 1550 4800
Wire Wire Line
	1400 4700 1550 4700
Wire Wire Line
	4600 5700 4700 5700
Wire Wire Line
	4600 5500 4700 5500
Wire Wire Line
	4300 5050 4300 5000
Wire Wire Line
	4300 3800 4300 3900
Wire Wire Line
	4900 4250 5300 4250
Wire Wire Line
	3400 4450 3700 4450
Wire Wire Line
	3400 4250 3700 4250
Wire Wire Line
	2750 2600 3000 2600
Wire Wire Line
	2750 2400 3000 2400
Wire Wire Line
	2750 2150 3000 2150
Wire Wire Line
	2750 2050 3000 2050
Wire Wire Line
	2950 1450 2750 1450
Wire Wire Line
	1050 2300 1350 2300
Wire Wire Line
	1050 2400 1350 2400
Wire Wire Line
	1050 1000 1050 1200
Wire Wire Line
	950  1000 950  1200
Wire Wire Line
	850  1250 850  1000
Wire Wire Line
	750  1000 750  1250
Wire Wire Line
	5100 3600 5100 4250
Wire Wire Line
	6300 3000 6300 2900
Wire Wire Line
	6300 2600 6700 2600
Connection ~ 6300 2600
Wire Wire Line
	6300 2450 6300 2600
Wire Wire Line
	6400 3600 6400 3600
Wire Wire Line
	6600 1050 6600 1150
Wire Wire Line
	6150 1050 6150 1150
Wire Wire Line
	6850 2000 6850 1950
Wire Wire Line
	6500 2000 6500 1800
Wire Wire Line
	6150 2000 6150 1950
Wire Wire Line
	6100 1600 6200 1600
Wire Wire Line
	6800 1600 6900 1600
Connection ~ 6850 1600
Connection ~ 6150 1600
Wire Wire Line
	6850 1600 6850 1650
Wire Wire Line
	6150 1600 6150 1650
Wire Wire Line
	2050 4500 2150 4500
Wire Wire Line
	2450 4500 2550 4500
Wire Wire Line
	2050 4400 2150 4400
Wire Wire Line
	2450 4400 2550 4400
Connection ~ 5100 4250
$Comp
L CONN_02X05 P6
U 1 1 5889ED20
P 1800 6650
F 0 "P6" H 1800 6950 50  0000 C CNN
F 1 "CONN_02X05" H 1800 6350 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_2x05" H 1800 5450 50  0001 C CNN
F 3 "" H 1800 5450 50  0000 C CNN
	1    1800 6650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 5889ED8F
P 1400 4400
F 0 "#PWR024" H 1400 4150 50  0001 C CNN
F 1 "GND" H 1400 4250 50  0000 C CNN
F 2 "" H 1400 4400 50  0000 C CNN
F 3 "" H 1400 4400 50  0000 C CNN
	1    1400 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 4400 1550 4400
$Comp
L GND #PWR025
U 1 1 5889EEEF
P 1100 6450
F 0 "#PWR025" H 1100 6200 50  0001 C CNN
F 1 "GND" H 1100 6300 50  0000 C CNN
F 2 "" H 1100 6450 50  0000 C CNN
F 3 "" H 1100 6450 50  0000 C CNN
	1    1100 6450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR026
U 1 1 5889EF72
P 1500 6950
F 0 "#PWR026" H 1500 6700 50  0001 C CNN
F 1 "GND" H 1500 6800 50  0000 C CNN
F 2 "" H 1500 6950 50  0000 C CNN
F 3 "" H 1500 6950 50  0000 C CNN
	1    1500 6950
	1    0    0    -1  
$EndComp
$Comp
L LED D3
U 1 1 5889EFD0
P 2350 6550
F 0 "D3" H 2350 6650 50  0000 C CNN
F 1 "LED_PPS" H 2350 6450 50  0000 C CNN
F 2 "" H 2350 6550 50  0000 C CNN
F 3 "" H 2350 6550 50  0000 C CNN
	1    2350 6550
	-1   0    0    1   
$EndComp
$Comp
L LED D2
U 1 1 5889F03A
P 2350 6450
F 0 "D2" H 2350 6550 50  0000 C CNN
F 1 "LED_COIL" H 2350 6350 50  0000 C CNN
F 2 "" H 2350 6450 50  0000 C CNN
F 3 "" H 2350 6450 50  0000 C CNN
	1    2350 6450
	-1   0    0    1   
$EndComp
Wire Wire Line
	2050 6450 2150 6450
Wire Wire Line
	2050 6550 2150 6550
$Comp
L GND #PWR027
U 1 1 5889F262
P 2700 6450
F 0 "#PWR027" H 2700 6200 50  0001 C CNN
F 1 "GND" H 2700 6300 50  0000 C CNN
F 2 "" H 2700 6450 50  0000 C CNN
F 3 "" H 2700 6450 50  0000 C CNN
	1    2700 6450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR028
U 1 1 5889F2AF
P 2700 6550
F 0 "#PWR028" H 2700 6300 50  0001 C CNN
F 1 "GND" H 2700 6400 50  0000 C CNN
F 2 "" H 2700 6550 50  0000 C CNN
F 3 "" H 2700 6550 50  0000 C CNN
	1    2700 6550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2550 6450 2700 6450
Wire Wire Line
	2550 6550 2700 6550
$Comp
L SW_PUSH SW3
U 1 1 5889F39B
P 2450 6850
F 0 "SW3" H 2600 6960 50  0000 C CNN
F 1 "BUTTON1" H 2450 6770 50  0000 C CNN
F 2 "" H 2450 6850 50  0000 C CNN
F 3 "" H 2450 6850 50  0000 C CNN
	1    2450 6850
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 5889F466
P 2450 6750
F 0 "SW2" H 2600 6860 50  0000 C CNN
F 1 "BUTTON2" H 2450 6670 50  0000 C CNN
F 2 "" H 2450 6750 50  0000 C CNN
F 3 "" H 2450 6750 50  0000 C CNN
	1    2450 6750
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 5889F500
P 2450 6650
F 0 "SW1" H 2600 6760 50  0000 C CNN
F 1 "BUTTON3" H 2450 6570 50  0000 C CNN
F 2 "" H 2450 6650 50  0000 C CNN
F 3 "" H 2450 6650 50  0000 C CNN
	1    2450 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 6650 2150 6650
Wire Wire Line
	2050 6750 2150 6750
Wire Wire Line
	2050 6850 2150 6850
Wire Wire Line
	1100 6450 1550 6450
$Comp
L CONN_01X04 P5
U 1 1 5889F780
P 1150 6700
F 0 "P5" H 1150 6950 50  0000 C CNN
F 1 "CONN_LCD" V 1250 6700 50  0000 C CNN
F 2 "" H 1150 6700 50  0000 C CNN
F 3 "" H 1150 6700 50  0000 C CNN
	1    1150 6700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1350 6550 1550 6550
Wire Wire Line
	1350 6650 1550 6650
Wire Wire Line
	1350 6750 1550 6750
Wire Wire Line
	1350 6850 1550 6850
Connection ~ 1500 6850
Wire Wire Line
	1500 6850 1500 6950
Wire Wire Line
	2850 6650 2850 7200
Connection ~ 1400 6750
Wire Wire Line
	1400 6750 1400 7200
Wire Wire Line
	1400 7200 2850 7200
Text Notes 2700 7150 0    60   ~ 0
5V
Connection ~ 2150 4600
Connection ~ 2150 4700
Connection ~ 2150 4800
$Comp
L R R6
U 1 1 588A001E
P 2750 4600
F 0 "R6" V 2830 4600 50  0000 C CNN
F 1 "10k" V 2750 4600 50  0000 C CNN
F 2 "" V 2680 4600 50  0000 C CNN
F 3 "" H 2750 4600 50  0000 C CNN
	1    2750 4600
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 588A014B
P 2750 4700
F 0 "R7" V 2830 4700 50  0000 C CNN
F 1 "10k" V 2750 4700 50  0000 C CNN
F 2 "" V 2680 4700 50  0000 C CNN
F 3 "" H 2750 4700 50  0000 C CNN
	1    2750 4700
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 588A01AD
P 2750 4800
F 0 "R8" V 2830 4800 50  0000 C CNN
F 1 "10k" V 2750 4800 50  0000 C CNN
F 2 "" V 2680 4800 50  0000 C CNN
F 3 "" H 2750 4800 50  0000 C CNN
	1    2750 4800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR029
U 1 1 588A037A
P 2950 4600
F 0 "#PWR029" H 2950 4350 50  0001 C CNN
F 1 "GND" H 2950 4450 50  0000 C CNN
F 2 "" H 2950 4600 50  0000 C CNN
F 3 "" H 2950 4600 50  0000 C CNN
	1    2950 4600
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR030
U 1 1 588A03DC
P 2950 4700
F 0 "#PWR030" H 2950 4450 50  0001 C CNN
F 1 "GND" H 2950 4550 50  0000 C CNN
F 2 "" H 2950 4700 50  0000 C CNN
F 3 "" H 2950 4700 50  0000 C CNN
	1    2950 4700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR031
U 1 1 588A043E
P 2950 4800
F 0 "#PWR031" H 2950 4550 50  0001 C CNN
F 1 "GND" H 2950 4650 50  0000 C CNN
F 2 "" H 2950 4800 50  0000 C CNN
F 3 "" H 2950 4800 50  0000 C CNN
	1    2950 4800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2900 4600 2950 4600
Wire Wire Line
	2900 4700 2950 4700
Wire Wire Line
	2900 4800 2950 4800
Wire Wire Line
	2850 6750 2750 6750
Connection ~ 2850 6750
Wire Wire Line
	2850 6850 2750 6850
Connection ~ 2850 6850
Connection ~ 2850 6650
$Comp
L LED D4
U 1 1 588A0B8F
P 3150 6650
F 0 "D4" H 3150 6750 50  0000 C CNN
F 1 "LED_PWR" H 3150 6550 50  0000 C CNN
F 2 "" H 3150 6650 50  0000 C CNN
F 3 "" H 3150 6650 50  0000 C CNN
	1    3150 6650
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR032
U 1 1 588A0C01
P 3850 6650
F 0 "#PWR032" H 3850 6400 50  0001 C CNN
F 1 "GND" H 3850 6500 50  0000 C CNN
F 2 "" H 3850 6650 50  0000 C CNN
F 3 "" H 3850 6650 50  0000 C CNN
	1    3850 6650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 6650 3750 6650
$Comp
L R R9
U 1 1 588A0E38
P 3600 6650
F 0 "R9" V 3680 6650 50  0000 C CNN
F 1 "R" V 3600 6650 50  0000 C CNN
F 2 "" V 3530 6650 50  0000 C CNN
F 3 "" H 3600 6650 50  0000 C CNN
	1    3600 6650
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 6650 2950 6650
Wire Wire Line
	3450 6650 3350 6650
Text Notes 1000 6200 0    60   ~ 0
Cover
$EndSCHEMATC
