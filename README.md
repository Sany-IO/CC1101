CC1101 for STM32L0xx 32-bit MCU and RaspberryPI
======

This is a CC1101 Library for the STM32L0xx Ulta-Low-Power 32-bit MCU, and the originally RaspberryPI Library from Spaceteddy.
The STM32 Library is actually under development, but it runs including with Sleep-Mode.

actually i develop a function, to download automatically the actual firmware for the device it's self, from a master station.
later, can you update your firmware over RF, with packet control, and sleeping to save battery.

Hardware connection
===================

check cc1101_stm32l0xx.h and/or cc1101_raspi.h for Pin description

CC1101 Vdd = 3.3V
CC1101 max. digital voltage level = 3.3V (not 5V tolerant)

```
CC1101<->STM32L0xx

Vdd    -    3.3V
SI     -    MOSI PA7 (PIN13)
SO     -    MISO PA6 (PIN12)
CS     -    SS   PA4 (PIN10)
SCLK   -    SCK  PA5 (PIN11)
GDO2   -    GPIO_EXTI15 PA15 (PIN23)
GDO0   -    GPIO_EXTI10 PA10 (PIN20) actually not used...
GND    -    GND


CC1101<->Raspi

Vdd    -    3.3V (P1-01)
SI     -    MOSI (P1-19)
SO     -    MISO (P1-21)
CS     -    SS   (P1-24)
SCLK   -    SCK  (P1-23)
GDO2   -    GPIO (P1-22)
GDO0   -    not used in this demo
GND    -    P1-25
```

General description of RF packet
================================

```
-> pkt_len [1byte] | rx_addr [1byte] | tx_addr [1byte] | payload data [1..60bytes]
```

pkt_len = count of bytes which shall transfered over air (rx_addr + tx_addr + payload data)<br />
rx_addr = address of device, which shall receive the message (0x00 = broadcast to all devices)<br />
tx_addr = transmitter or my address. the receiver should know who has sent a message.<br />
payload = 1 to 60 bytes payload data.<br />

TX Bytes example:<br />
-> 0x06 0x03 0x01 0x00 0x01 0x02 0x03<br />

Basic configuration
===================

use **uint8_t rf_begin(volatile uint8_t &My_addr)** always as first configuration step. For Arduino devices, this function returns the device address, which was already stored in the Arduino EEPROM.

Device address
--------------
you should set a unique device address for the transmitter and a unique device address for the receiver. 
This can be done with **void rf_set_myaddr(uint8_t addr)**.

i.E. -> TX = 0x01 ; RX = 0x03


Modulation modes
----------------
the following modulation modes can be set by **void rf_set_mode(uint8_t mode)**. Transmitter and receiver must have the same Mode setting.

```
1 = GFSK_1_2_kb
2 = GFSK_38_4_kb
3 = GFSK_100_kb
4 = MSK_250_kb
5 = MSK_500_kb
6 = OOK_4_8_kb
```

ISM frequency band
------------------
you can set a frequency operation band by **void rf_set_ISM(uint8_t ism_freq)** to make it compatible with your hardware.

```
1 = 315
2 = 433
3 = 868
4 = 915
```

STM32L0xx specific
================

CC1101 RF settings are stored in Firmware/Flash, or can changed remotely over RF, with a sender.

STM32L0xx specific
================
Bootloader Mode for Flashing Firmware over RF (in Development, BETA)



Raspberry Pi
============

How to compile Raspi Demo files
-------------------------------

be sure first, that you have already wiringPi installed on your Raspberry Pi hardware. 

copy RX_Demo.cpp, TX_Demo.cpp, cc1100_raspi.cpp, cc1100_raspi.h in the same directory and compile: <br />

RX_Demo.cpp<br />
```
sudo g++ -lwiringPi RX_Demo.cpp cc1100_raspi.cpp -o RX_Demo
sudo chmod 755 RX_Demo
```

TX_Demo.cpp<br />
```
sudo g++ -lwiringPi TX_Demo.cpp cc1100_raspi.cpp -o TX_Demo
sudo chmod 755 TX_Demo
```

Command Line parameters
-----------------------

TX_Demo:<br />
```
CC1100 SW [-h] [-V] [-a My_Addr] [-r RxDemo_Addr] [-i Msg_Interval] [-t tx_retries] [-c channel] [-f frequency]
          [-m modulation]

  -h              			print this help and exit
  -V              			print version and exit
  -v              			set verbose flag
  -a my address [1-255] 		set my address
  -r rx address [1-255] 	  	set RxDemo receiver address
  -i interval ms[1-6000] 	  	sets message interval timing
  -t tx_retries [0-255] 	  	sets message send retries
  -c channel    [1-255] 		set transmit channel
  -f frequency  [315,434,868,915]  	set ISM band
  -m modulation [1,38,100,250,500,4]	set modulation
  ```
  
  Example,<br />
  ```
  sudo ./TX_Demo -v -a1 -r3 -i1000 -t5 -c1 -f434 -m100
  ```
  
  RX_Demo:<br />
  ```
  CC1100 SW [-h] [-V] [-v] [-a My_Addr] [-c channel] [-f frequency] [-m modulation]
  -h              			print this help and exit
  -V              			print version and exit
  -v              			set verbose flag
  -a my address [1-255] 		set my address
  -c channel    [1-255] 		set transmit channel
  -f frequency  [315,434,868,915]  	set ISM band
  -m modulation [1,38,100,250,500,4]	set modulation
  ```
  
  Example,<br />
  ```
  sudo ./RX_Demo -v -a3 -c1 -f434 -m100
  ```
