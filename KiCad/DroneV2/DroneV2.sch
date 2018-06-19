EESchema Schematic File Version 2
LIBS:DroneV2-rescue
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
LIBS:arduino_micro_shield
LIBS:gy-80
LIBS:esc_afro_30a
LIBS:Turnigy_8CH_Rx
LIBS:DroneV2-cache
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
L ARDUINO_MICRO_SHIELD-RESCUE-DroneV2 U2
U 1 1 56DF1795
P 5525 4200
F 0 "U2" V 6450 4200 60  0000 C CNN
F 1 "ARDUINO_MICRO_SHIELD" V 4575 4200 60  0000 C CNN
F 2 "drone-custom:ARDUINO_MICRO_SHIELD_custom" H 5925 4150 60  0001 C CNN
F 3 "" H 5925 4150 60  0000 C CNN
	1    5525 4200
	0    1    1    0   
$EndComp
$Comp
L GY-80 U1
U 1 1 56DF1AF9
P 3225 2400
F 0 "U1" H 3225 3000 60  0000 C CNN
F 1 "GY-80" H 3225 1775 60  0000 C CNN
F 2 "drone-custom:GY-80-Footprint_wider_pad" H 3025 2900 60  0001 C CNN
F 3 "" H 3025 2900 60  0000 C CNN
	1    3225 2400
	1    0    0    1   
$EndComp
$Comp
L ESC_Connecor ESC1
U 1 1 56DF1DA7
P 7775 3800
F 0 "ESC1" H 7475 3800 60  0000 C CNN
F 1 "ESC_Connecor" H 8350 3800 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 7775 3800 60  0001 C CNN
F 3 "" H 7775 3800 60  0000 C CNN
	1    7775 3800
	1    0    0    -1  
$EndComp
$Comp
L ESC_Connecor ESC2
U 1 1 56DF1E63
P 7775 4350
F 0 "ESC2" H 7475 4350 60  0000 C CNN
F 1 "ESC_Connecor" H 8350 4350 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 7775 4350 60  0001 C CNN
F 3 "" H 7775 4350 60  0000 C CNN
	1    7775 4350
	1    0    0    -1  
$EndComp
$Comp
L ESC_Connecor ESC3
U 1 1 56DF1EA7
P 7775 4900
F 0 "ESC3" H 7475 4900 60  0000 C CNN
F 1 "ESC_Connecor" H 8350 4900 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 7775 4900 60  0001 C CNN
F 3 "" H 7775 4900 60  0000 C CNN
	1    7775 4900
	1    0    0    -1  
$EndComp
$Comp
L ESC_Connecor ESC4
U 1 1 56DF1EF0
P 7775 5450
F 0 "ESC4" H 7475 5450 60  0000 C CNN
F 1 "ESC_Connecor" H 8350 5450 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 7775 5450 60  0001 C CNN
F 3 "" H 7775 5450 60  0000 C CNN
	1    7775 5450
	1    0    0    -1  
$EndComp
$Comp
L LM7809CT U3
U 1 1 56DF2096
P 7175 2275
F 0 "U3" H 6975 2475 50  0000 C CNN
F 1 "LM7809CT" H 7175 2475 50  0000 L CNN
F 2 "drone-custom:TO-220_Neutral123_Vertical_custom" H 7175 2375 50  0001 C CIN
F 3 "" H 7175 2275 50  0000 C CNN
	1    7175 2275
	-1   0    0    -1  
$EndComp
$Comp
L Rx_Pin CH1
U 1 1 56DF22BB
P 3200 3800
F 0 "CH1" H 3450 3800 60  0000 C CNN
F 1 "Rx_Pin" H 2900 3800 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 3200 3800 60  0001 C CNN
F 3 "" H 3200 3800 60  0000 C CNN
	1    3200 3800
	1    0    0    -1  
$EndComp
$Comp
L Rx_Pin CH2
U 1 1 56DF230E
P 3200 4350
F 0 "CH2" H 3450 4350 60  0000 C CNN
F 1 "Rx_Pin" H 2900 4350 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 3200 4350 60  0001 C CNN
F 3 "" H 3200 4350 60  0000 C CNN
	1    3200 4350
	1    0    0    -1  
$EndComp
$Comp
L Rx_Pin CH3
U 1 1 56DF23AF
P 3200 4900
F 0 "CH3" H 3450 4900 60  0000 C CNN
F 1 "Rx_Pin" H 2900 4900 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 3200 4900 60  0001 C CNN
F 3 "" H 3200 4900 60  0000 C CNN
	1    3200 4900
	1    0    0    -1  
$EndComp
$Comp
L Rx_Pin CH4
U 1 1 56DF240D
P 3200 5450
F 0 "CH4" H 3450 5450 60  0000 C CNN
F 1 "Rx_Pin" H 2900 5450 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Straight_1x03_custom" H 3200 5450 60  0001 C CNN
F 3 "" H 3200 5450 60  0000 C CNN
	1    3200 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3600 3300 3550
Wire Wire Line
	3300 3550 3775 3550
Wire Wire Line
	3775 3550 3775 4600
Wire Wire Line
	3775 4600 4825 4600
Wire Wire Line
	3300 4150 3300 4100
Wire Wire Line
	3300 4100 3725 4100
Wire Wire Line
	3725 4100 3725 4700
Wire Wire Line
	3725 4700 4825 4700
Wire Wire Line
	3300 4700 3300 4650
Wire Wire Line
	3300 4650 3675 4650
Wire Wire Line
	3675 4650 3675 4800
Wire Wire Line
	3675 4800 4825 4800
Wire Wire Line
	3300 5250 3300 5200
Wire Wire Line
	3300 5200 3675 5200
Wire Wire Line
	3675 5200 3675 4900
Wire Wire Line
	3675 4900 4825 4900
Wire Wire Line
	3100 5250 3100 5200
Wire Wire Line
	3100 5200 2425 5200
Wire Wire Line
	2425 5200 2425 3375
Wire Wire Line
	2425 3550 3100 3550
Wire Wire Line
	3100 3550 3100 3600
Wire Wire Line
	3200 3600 3200 3500
Wire Wire Line
	3200 3500 2525 3500
Wire Wire Line
	2525 3375 2525 5150
Wire Wire Line
	2525 5150 3200 5150
Wire Wire Line
	3200 5150 3200 5250
Wire Wire Line
	3200 4700 3200 4600
Wire Wire Line
	3200 4600 2525 4600
Connection ~ 2525 4600
Wire Wire Line
	3100 4700 3100 4650
Wire Wire Line
	3100 4650 2425 4650
Connection ~ 2425 4650
Wire Wire Line
	3200 4150 3200 4050
Wire Wire Line
	3200 4050 2525 4050
Connection ~ 2525 4050
Wire Wire Line
	3100 4150 3100 4100
Wire Wire Line
	3100 4100 2425 4100
Connection ~ 2425 4100
Connection ~ 2425 3550
Connection ~ 2525 3500
Wire Wire Line
	6175 4700 7300 4700
Wire Wire Line
	7300 4700 7300 5200
Wire Wire Line
	7300 5200 7675 5200
Wire Wire Line
	7675 5200 7675 5250
Wire Wire Line
	7675 4700 7675 4650
Wire Wire Line
	7675 4650 7300 4650
Wire Wire Line
	7300 4650 7300 4600
Wire Wire Line
	7300 4600 6175 4600
Wire Wire Line
	7675 4150 7675 4100
Wire Wire Line
	7675 4100 7300 4100
Wire Wire Line
	7300 4100 7300 4500
Wire Wire Line
	7300 4500 6175 4500
Wire Wire Line
	7675 3600 7675 3550
Wire Wire Line
	7675 3550 7250 3550
Wire Wire Line
	7250 3550 7250 4400
Wire Wire Line
	7250 4400 6175 4400
Wire Wire Line
	7875 2525 7875 3600
Wire Wire Line
	7875 3550 8025 3550
Wire Wire Line
	8025 3550 8025 5200
Wire Wire Line
	8025 5200 7875 5200
Wire Wire Line
	7875 5200 7875 5250
Wire Wire Line
	7875 4150 7875 4100
Wire Wire Line
	7875 4100 8025 4100
Connection ~ 8025 4100
Wire Wire Line
	7875 4700 7875 4650
Wire Wire Line
	7875 4650 8025 4650
Connection ~ 8025 4650
NoConn ~ 7775 3600
NoConn ~ 7775 4150
NoConn ~ 7775 4700
NoConn ~ 7775 5250
Wire Wire Line
	3675 2550 3925 2550
Wire Wire Line
	3925 2550 3925 4100
Wire Wire Line
	3925 4100 4825 4100
Wire Wire Line
	3675 2450 3975 2450
Wire Wire Line
	3975 2450 3975 4000
Wire Wire Line
	3975 4000 4825 4000
Wire Wire Line
	3675 2650 3875 2650
Wire Wire Line
	3875 2650 3875 3900
Wire Wire Line
	3875 3900 4825 3900
Wire Wire Line
	6525 2525 7925 2525
Wire Wire Line
	7575 2225 7925 2225
Wire Wire Line
	6475 2225 6775 2225
$Comp
L C C1
U 1 1 56DF4201
P 6725 2375
F 0 "C1" H 6750 2475 50  0000 L CNN
F 1 "C" H 6750 2275 50  0000 L CNN
F 2 "drone-custom:C_Rect_L7_W2_P5_custom" H 6763 2225 50  0001 C CNN
F 3 "" H 6725 2375 50  0000 C CNN
	1    6725 2375
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 56DF429E
P 7625 2375
F 0 "C2" H 7650 2475 50  0000 L CNN
F 1 "C" H 7650 2275 50  0000 L CNN
F 2 "drone-custom:C_Rect_L7_W2_P5_custom" H 7663 2225 50  0001 C CNN
F 3 "" H 7625 2375 50  0000 C CNN
	1    7625 2375
	1    0    0    -1  
$EndComp
Wire Wire Line
	6525 2525 6525 3700
Wire Wire Line
	6525 3700 6175 3700
Wire Wire Line
	6475 2225 6475 3600
Wire Wire Line
	6475 3600 6175 3600
Connection ~ 7875 2525
Connection ~ 7875 3550
$Comp
L Battery_Commector U4
U 1 1 56DF4514
P 8375 2375
F 0 "U4" H 8375 1975 60  0000 C CNN
F 1 "Battery_Connector" H 8375 2725 60  0000 C CNN
F 2 "drone-custom:Pin_Header_Angled_1x03_custom" H 8375 2375 60  0001 C CNN
F 3 "" H 8375 2375 60  0000 C CNN
	1    8375 2375
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR01
U 1 1 56DF46E5
P 6175 3900
F 0 "#PWR01" H 6175 3750 50  0001 C CNN
F 1 "+5V" H 6175 4040 50  0000 C CNN
F 2 "" H 6175 3900 50  0000 C CNN
F 3 "" H 6175 3900 50  0000 C CNN
	1    6175 3900
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR02
U 1 1 56DF47F5
P 2525 3375
F 0 "#PWR02" H 2525 3225 50  0001 C CNN
F 1 "+5V" H 2525 3515 50  0000 C CNN
F 2 "" H 2525 3375 50  0000 C CNN
F 3 "" H 2525 3375 50  0000 C CNN
	1    2525 3375
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 56DF4825
P 2425 3375
F 0 "#PWR03" H 2425 3125 50  0001 C CNN
F 1 "GND" H 2425 3225 50  0000 C CNN
F 2 "" H 2425 3375 50  0000 C CNN
F 3 "" H 2425 3375 50  0000 C CNN
	1    2425 3375
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR04
U 1 1 56DF4935
P 6725 2525
F 0 "#PWR04" H 6725 2275 50  0001 C CNN
F 1 "GND" H 6725 2375 50  0000 C CNN
F 2 "" H 6725 2525 50  0000 C CNN
F 3 "" H 6725 2525 50  0000 C CNN
	1    6725 2525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 56DF4B26
P 4325 3900
F 0 "#PWR05" H 4325 3650 50  0001 C CNN
F 1 "GND" H 4325 3750 50  0000 C CNN
F 2 "" H 4325 3900 50  0000 C CNN
F 3 "" H 4325 3900 50  0000 C CNN
	1    4325 3900
	-1   0    0    1   
$EndComp
Wire Wire Line
	4825 4200 4225 4200
Wire Wire Line
	4225 4200 4225 3675
Wire Wire Line
	4325 3900 4325 3675
Connection ~ 4325 3900
$Comp
L Buzzer BUZ1
U 1 1 56DF25BE
P 4275 3275
F 0 "BUZ1" H 4275 2975 60  0000 C CNN
F 1 "Buzzer" H 4275 3225 60  0000 C CNN
F 2 "drone-custom:MagneticBuzzer_StarMicronics_HMB-06_HMB-12_custom" H 4275 3275 60  0001 C CNN
F 3 "" H 4275 3275 60  0000 C CNN
	1    4275 3275
	1    0    0    1   
$EndComp
NoConn ~ 4825 3400
NoConn ~ 4825 3500
NoConn ~ 4825 3600
NoConn ~ 4825 3700
NoConn ~ 4825 3800
NoConn ~ 4825 4300
NoConn ~ 4825 4400
NoConn ~ 4825 4500
NoConn ~ 4825 5000
NoConn ~ 6175 5000
NoConn ~ 6175 4800
NoConn ~ 6175 4300
NoConn ~ 6175 4200
NoConn ~ 6175 3800
NoConn ~ 6175 3500
NoConn ~ 6175 3400
Connection ~ 7625 2525
Connection ~ 7175 2525
Connection ~ 6725 2525
Connection ~ 6725 2225
Connection ~ 7625 2225
NoConn ~ 3675 1950
NoConn ~ 3675 2050
NoConn ~ 3675 2150
NoConn ~ 3675 2250
NoConn ~ 3675 2350
NoConn ~ 3675 2850
NoConn ~ 3300 5575
Wire Wire Line
	3675 2750 3825 2750
Wire Wire Line
	3825 2750 3825 3200
Wire Wire Line
	3825 3200 3025 3200
Text Label 3025 3200 0    60   ~ 0
ARDUINO_3.3V
Wire Wire Line
	6175 4900 6850 4900
Text Label 6850 4900 2    60   ~ 0
ARDUINO_3.3V
$EndSCHEMATC
