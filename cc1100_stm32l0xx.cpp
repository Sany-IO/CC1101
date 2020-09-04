/*
 * rf_driver.c
 *
 *  Created on: 20.08.2020
 *      Author: Daniel Steiner, Sany.IO (www.sany.io)
 *      The following code was forked by SpaceTeddy (https://github.com/SpaceTeddy/CC1101):
 *
 */
//#define RF_DRIVER_H_
#include "main.h"
#include "cc1100_stm32l0xx.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

/* Private variables -------------------------------------------------------------*/
SPI_HandleTypeDef* hal_spi;
UART_HandleTypeDef* hal_uart;

uint16_t CS_Pin;
GPIO_TypeDef* CS_GPIO_Port;
volatile uint8_t RF_packetsAvailable = 0;
volatile uint8_t wor_enable_flag = 0;
volatile uint8_t mcu_sleep_enable = 0;

uint16_t GDO0_Pin; //Board pin connected to CC1101's GDO0 pin

uint8_t GDO0_FLAG = 0; //Flag of the tx threshold interrupt. Used to control the reading and writting to the buffer and to check the
						//arrival of a packet.

uint32_t TimeOut = 0x6FF; //TODO It's hardcoded

//----------------------[PATABLES]---------------------------------------------
/*PATABLES Registers presets for various frequencies. This values are the (suposed) optimal values for -30, -20, -15,
-10, 0, 5, 7, 10 dBm for each carrier frequency.
*/
uint8_t patable_power_315[]  = {0x17,0x1D,0x26,0x69,0x51,0x86,0xCC,0xC3};
uint8_t patable_power_433[]  = {0x6C,0x1C,0x06,0x3A,0x51,0x85,0xC8,0xC0};
uint8_t patable_power_868[]  = {0x03,0x17,0x1D,0x26,0x50,0x86,0xCD,0xC0};
uint8_t patable_power_915[]  = {0x0B,0x1B,0x6D,0x67,0x50,0x85,0xC9,0xC1};


//----------------------[REGISTER BASIC CONFIGURATION]------------------------
//Preset for Gaussian Frequency Shift Keying mod at 1.2KBits/s
uint8_t cc1100_GFSK_1_2_kb[] = {
                    0x07,  // IOCFG2        GDO2 Output Pin Configuration
                    0x2E,  // IOCFG1        GDO1 Output Pin Configuration
                    0x80,  // IOCFG0        GDO0_Pin Output Pin Configuration
                    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
                    0x57,  // SYNC1         Sync Word, High Byte
                    0x43,  // SYNC0         Sync Word, Low Byte
                    0x3E,  // PKTLEN        Packet Length
                    0xD8,  // PKTCTRL1      Packet Automation Control //TODO changed from DC to disable lqi and rssi appending
                    0x45,  // PKTCTRL0      Packet Automation Control
                    0xFF,  // ADDR          Device Address
                    0x00,  // CHANNR        Channel Number
                    0x08,  // FSCTRL1       Frequency Synthesizer Control
                    0x00,  // FSCTRL0       Frequency Synthesizer Control
                    0x21,  // FREQ2         Frequency Control Word, High Byte
                    0x65,  // FREQ1         Frequency Control Word, Middle Byte
                    0x6A,  // FREQ0         Frequency Control Word, Low Byte
                    0xF5,  // MDMCFG4       Modem Configuration
                    0x83,  // MDMCFG3       Modem Configuration
                    0x13,  // MDMCFG2       Modem Configuration
                    0xC0,  // MDMCFG1       Modem Configuration
                    0xF8,  // MDMCFG0       Modem Configuration
                    0x15,  // DEVIATN       Modem Deviation Setting
                    0x07,  // MCSM2         Main Radio Control State Machine Configuration
                    0x00,  // MCSM1         Main Radio Control State Machine Configuration //TODO was 0x0C
                    0x18,  // MCSM0         Main Radio Control State Machine Configuration
                    0x16,  // FOCCFG        Frequency Offset Compensation Configuration
                    0x6C,  // BSCFG         Bit Synchronization Configuration
                    0x03,  // AGCCTRL2      AGC Control
                    0x40,  // AGCCTRL1      AGC Control
                    0x91,  // AGCCTRL0      AGC Control
                    0x02,  // WOREVT1       High Byte Event0 Timeout
                    0x26,  // WOREVT0       Low Byte Event0 Timeout
                    0x09,  // WORCTRL       Wake On Radio Control
                    0x56,  // FREND1        Front End RX Configuration
                    0x17,  // FREND0        Front End TX Configuration
                    0xA9,  // FSCAL3        Frequency Synthesizer Calibration
                    0x0A,  // FSCAL2        Frequency Synthesizer Calibration
                    0x00,  // FSCAL1        Frequency Synthesizer Calibration
                    0x11,  // FSCAL0        Frequency Synthesizer Calibration
                    0x41,  // RCCTRL1       RC Oscillator Configuration
                    0x00,  // RCCTRL0       RC Oscillator Configuration
                    0x59,  // FSTEST        Frequency Synthesizer Calibration Control,
                    0x7F,  // PTEST         Production Test
                    0x3F,  // AGCTEST       AGC Test
                    0x81,  // TEST2         Various Test Settings
                    0x3F,  // TEST1         Various Test Settings
                    0x0B   // TEST0         Various Test Settings
                };

uint8_t cc1100_GFSK_38_4_kb[] = {
                    0x07,  // IOCFG2        GDO2 Output Pin Configuration
                    0x2E,  // IOCFG1        GDO1 Output Pin Configuration
                    0x80,  // IOCFG0        GDO0_Pin Output Pin Configuration
                    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
                    0x57,  // SYNC1         Sync Word, High Byte
                    0x43,  // SYNC0         Sync Word, Low Byte
                    0x3E,  // PKTLEN        Packet Length
					0xDC,  // PKTCTRL1      Packet Automation Control
                    0x45,  // PKTCTRL0      Packet Automation Control
                    0xFF,  // ADDR          Device Address
                    0x00,  // CHANNR        Channel Number
                    0x06,  // FSCTRL1       Frequency Synthesizer Control
                    0x00,  // FSCTRL0       Frequency Synthesizer Control
                    0x21,  // FREQ2         Frequency Control Word, High Byte
                    0x65,  // FREQ1         Frequency Control Word, Middle Byte
                    0x6A,  // FREQ0         Frequency Control Word, Low Byte
                    0xCA,  // MDMCFG4       Modem Configuration
                    0x83,  // MDMCFG3       Modem Configuration
                    0x13,  // MDMCFG2       Modem Configuration
                    0xA0,  // MDMCFG1       Modem Configuration
                    0xF8,  // MDMCFG0       Modem Configuration
                    0x34,  // DEVIATN       Modem Deviation Setting
                    0x07,  // MCSM2         Main Radio Control State Machine Configuration
                    0x0C,  // MCSM1         Main Radio Control State Machine Configuration
                    0x18,  // MCSM0         Main Radio Control State Machine Configuration
                    0x16,  // FOCCFG        Frequency Offset Compensation Configuration
                    0x6C,  // BSCFG         Bit Synchronization Configuration
                    0x43,  // AGCCTRL2      AGC Control
                    0x40,  // AGCCTRL1      AGC Control
                    0x91,  // AGCCTRL0      AGC Control
                    0x02,  // WOREVT1       High Byte Event0 Timeout
                    0x26,  // WOREVT0       Low Byte Event0 Timeout
                    0x09,  // WORCTRL       Wake On Radio Control
                    0x56,  // FREND1        Front End RX Configuration
                    0x17,  // FREND0        Front End TX Configuration
                    0xA9,  // FSCAL3        Frequency Synthesizer Calibration
                    0x0A,  // FSCAL2        Frequency Synthesizer Calibration
                    0x00,  // FSCAL1        Frequency Synthesizer Calibration
                    0x11,  // FSCAL0        Frequency Synthesizer Calibration
                    0x41,  // RCCTRL1       RC Oscillator Configuration
                    0x00,  // RCCTRL0       RC Oscillator Configuration
                    0x59,  // FSTEST        Frequency Synthesizer Calibration Control,
                    0x7F,  // PTEST         Production Test
                    0x3F,  // AGCTEST       AGC Test
                    0x81,  // TEST2         Various Test Settings
                    0x3F,  // TEST1         Various Test Settings
                    0x0B   // TEST0         Various Test Settings
                };

uint8_t cc1100_GFSK_100_kb[]  = {
					0x07,  // IOCFG2        GDO2 Output Pin Configuration
					0x2E,  // IOCFG1        GDO1 Output Pin Configuration
					0x80,  // IOCFG0        GDO0 Output Pin Configuration
					0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
					0x57,  // SYNC1         Sync Word, High Byte
					0x43,  // SYNC0         Sync Word, Low Byte
					0x3E,  // PKTLEN        Packet Length
					0x0E,  // PKTCTRL1      Packet Automation Control
					0x45,  // PKTCTRL0      Packet Automation Control
					0xFF,  // ADDR          Device Address
					0x00,  // CHANNR        Channel Number
					0x08,  // FSCTRL1       Frequency Synthesizer Control
					0x00,  // FSCTRL0       Frequency Synthesizer Control
					0x21,  // FREQ2         Frequency Control Word, High Byte
					0x65,  // FREQ1         Frequency Control Word, Middle Byte
					0x6A,  // FREQ0         Frequency Control Word, Low Byte
					0x5B,  // MDMCFG4       Modem Configuration
					0xF8,  // MDMCFG3       Modem Configuration
					0x13,  // MDMCFG2       Modem Configuration
					0xA0,  // MDMCFG1       Modem Configuration
					0xF8,  // MDMCFG0       Modem Configuration
					0x47,  // DEVIATN       Modem Deviation Setting
					0x07,  // MCSM2         Main Radio Control State Machine Configuration
					0x0C,  // MCSM1         Main Radio Control State Machine Configuration
					0x18,  // MCSM0         Main Radio Control State Machine Configuration
					0x1D,  // FOCCFG        Frequency Offset Compensation Configuration
					0x1C,  // BSCFG         Bit Synchronization Configuration
					0xC7,  // AGCCTRL2      AGC Control
					0x00,  // AGCCTRL1      AGC Control
					0xB2,  // AGCCTRL0      AGC Control
					0x02,  // WOREVT1       High Byte Event0 Timeout
					0x26,  // WOREVT0       Low Byte Event0 Timeout
					0x09,  // WORCTRL       Wake On Radio Control
					0xB6,  // FREND1        Front End RX Configuration
					0x17,  // FREND0        Front End TX Configuration
					0xEA,  // FSCAL3        Frequency Synthesizer Calibration
					0x0A,  // FSCAL2        Frequency Synthesizer Calibration
					0x00,  // FSCAL1        Frequency Synthesizer Calibration
					0x11,  // FSCAL0        Frequency Synthesizer Calibration
					0x41,  // RCCTRL1       RC Oscillator Configuration
					0x00,  // RCCTRL0       RC Oscillator Configuration
					0x59,  // FSTEST        Frequency Synthesizer Calibration Control,
					0x7F,  // PTEST         Production Test
					0x3F,  // AGCTEST       AGC Test
					0x81,  // TEST2         Various Test Settings
					0x3F,  // TEST1         Various Test Settings
					0x0B   // TEST0         Various Test Settings
                };

uint8_t cc1100_MSK_250_kb[] = {
                    0x07,  // IOCFG2        GDO2 Output Pin Configuration
                    0x2E,  // IOCFG1        GDO1 Output Pin Configuration
                    0x80,  // IOCFG0        GDO0_Pin Output Pin Configuration
                    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
                    0x57,  // SYNC1         Sync Word, High Byte
                    0x43,  // SYNC0         Sync Word, Low Byte
                    0x3E,  // PKTLEN        Packet Length
					0xDC,  // PKTCTRL1      Packet Automation Control
                    0x45,  // PKTCTRL0      Packet Automation Control
                    0xFF,  // ADDR          Device Address
                    0x00,  // CHANNR        Channel Number
                    0x0B,  // FSCTRL1       Frequency Synthesizer Control
                    0x00,  // FSCTRL0       Frequency Synthesizer Control
                    0x21,  // FREQ2         Frequency Control Word, High Byte
                    0x65,  // FREQ1         Frequency Control Word, Middle Byte
                    0x6A,  // FREQ0         Frequency Control Word, Low Byte
                    0x2D,  // MDMCFG4       Modem Configuration
                    0x3B,  // MDMCFG3       Modem Configuration
                    0x73,  // MDMCFG2       Modem Configuration
                    0xA0,  // MDMCFG1       Modem Configuration
                    0xF8,  // MDMCFG0       Modem Configuration
                    0x00,  // DEVIATN       Modem Deviation Setting
                    0x07,  // MCSM2         Main Radio Control State Machine Configuration
                    0x0C,  // MCSM1         Main Radio Control State Machine Configuration
                    0x18,  // MCSM0         Main Radio Control State Machine Configuration
                    0x1D,  // FOCCFG        Frequency Offset Compensation Configuration
                    0x1C,  // BSCFG         Bit Synchronization Configuration
                    0xC7,  // AGCCTRL2      AGC Control
                    0x00,  // AGCCTRL1      AGC Control
                    0xB2,  // AGCCTRL0      AGC Control
                    0x02,  // WOREVT1       High Byte Event0 Timeout
                    0x26,  // WOREVT0       Low Byte Event0 Timeout
                    0x09,  // WORCTRL       Wake On Radio Control
                    0xB6,  // FREND1        Front End RX Configuration
                    0x17,  // FREND0        Front End TX Configuration
                    0xEA,  // FSCAL3        Frequency Synthesizer Calibration
                    0x0A,  // FSCAL2        Frequency Synthesizer Calibration
                    0x00,  // FSCAL1        Frequency Synthesizer Calibration
                    0x11,  // FSCAL0        Frequency Synthesizer Calibration
                    0x41,  // RCCTRL1       RC Oscillator Configuration
                    0x00,  // RCCTRL0       RC Oscillator Configuration
                    0x59,  // FSTEST        Frequency Synthesizer Calibration Control,
                    0x7F,  // PTEST         Production Test
                    0x3F,  // AGCTEST       AGC Test
                    0x81,  // TEST2         Various Test Settings
                    0x3F,  // TEST1         Various Test Settings
                    0x0B   // TEST0         Various Test Settings
                };

uint8_t cc1100_MSK_500_kb[] = {
                    0x07,  // IOCFG2        GDO2 Output Pin Configuration
                    0x2E,  // IOCFG1        GDO1 Output Pin Configuration
                    0x80,  // IOCFG0        GDO0_Pin Output Pin Configuration
                    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
                    0x57,  // SYNC1         Sync Word, High Byte
                    0x43,  // SYNC0         Sync Word, Low Byte
                    0x3E,  // PKTLEN        Packet Length
					0xDC,  // PKTCTRL1      Packet Automation Control
                    0x45,  // PKTCTRL0      Packet Automation Control
                    0xFF,  // ADDR          Device Address
                    0x00,  // CHANNR        Channel Number
                    0x0C,  // FSCTRL1       Frequency Synthesizer Control
                    0x00,  // FSCTRL0       Frequency Synthesizer Control
                    0x21,  // FREQ2         Frequency Control Word, High Byte
                    0x65,  // FREQ1         Frequency Control Word, Middle Byte
                    0x6A,  // FREQ0         Frequency Control Word, Low Byte
                    0x0E,  // MDMCFG4       Modem Configuration
                    0x3B,  // MDMCFG3       Modem Configuration
                    0x73,  // MDMCFG2       Modem Configuration
                    0xA0,  // MDMCFG1       Modem Configuration
                    0xF8,  // MDMCFG0       Modem Configuration
                    0x00,  // DEVIATN       Modem Deviation Setting
                    0x07,  // MCSM2         Main Radio Control State Machine Configuration
                    0x0C,  // MCSM1         Main Radio Control State Machine Configuration
                    0x18,  // MCSM0         Main Radio Control State Machine Configuration
                    0x1D,  // FOCCFG        Frequency Offset Compensation Configuration
                    0x1C,  // BSCFG         Bit Synchronization Configuration
                    0xC7,  // AGCCTRL2      AGC Control
                    0x40,  // AGCCTRL1      AGC Control
                    0xB2,  // AGCCTRL0      AGC Control
                    0x02,  // WOREVT1       High Byte Event0 Timeout
                    0x26,  // WOREVT0       Low Byte Event0 Timeout
                    0x09,  // WORCTRL       Wake On Radio Control
                    0xB6,  // FREND1        Front End RX Configuration
                    0x17,  // FREND0        Front End TX Configuration
                    0xEA,  // FSCAL3        Frequency Synthesizer Calibration
                    0x0A,  // FSCAL2        Frequency Synthesizer Calibration
                    0x00,  // FSCAL1        Frequency Synthesizer Calibration
                    0x19,  // FSCAL0        Frequency Synthesizer Calibration
                    0x41,  // RCCTRL1       RC Oscillator Configuration
                    0x00,  // RCCTRL0       RC Oscillator Configuration
                    0x59,  // FSTEST        Frequency Synthesizer Calibration Control,
                    0x7F,  // PTEST         Production Test
                    0x3F,  // AGCTEST       AGC Test
                    0x81,  // TEST2         Various Test Settings
                    0x3F,  // TEST1         Various Test Settings
                    0x0B   // TEST0         Various Test Settings
                };

uint8_t cc1100_OOK_4_8_kb[] = { //In fact it's 2.4Kb/s because of the Manhattan codification, see Datasheet.
                    0x02,  // IOCFG2        GDO2 Output Pin Configuration //0x06 --> 0x02 (deasserts when below threshold)
                    0x2E,  // IOCFG1        GDO1 Output Pin Configuration
                    0x06,  // IOCFG0        GDO0_Pin Output Pin Configuration
                    0x48,  // FIFOTHR       RX FIFO and TX FIFO Thresholds //0x47 --> 0x48
                    0x57,  // SYNC1         Sync Word, High Byte
                    0x43,  // SYNC0         Sync Word, Low Byte
                    0xFF,  // PKTLEN        Packet Length
					0xDC,  // PKTCTRL1      Packet Automation Control
                    0x05,  // PKTCTRL0      Packet Automation Control
                    0x00,  // ADDR          Device Address
                    0x00,  // CHANNR        Channel Number
                    0x06,  // FSCTRL1       Frequency Synthesizer Control
                    0x00,  // FSCTRL0       Frequency Synthesizer Control
                    0x21,  // FREQ2         Frequency Control Word, High Byte
                    0x65,  // FREQ1         Frequency Control Word, Middle Byte
                    0x6A,  // FREQ0         Frequency Control Word, Low Byte
                    0x87,  // MDMCFG4       Modem Configuration
                    0x83,  // MDMCFG3       Modem Configuration
                    0x3B,  // MDMCFG2       Modem Configuration
                    0x22,  // MDMCFG1       Modem Configuration
                    0xF8,  // MDMCFG0       Modem Configuration
                    0x15,  // DEVIATN       Modem Deviation Setting
                    0x07,  // MCSM2         Main Radio Control State Machine Configuration
                    0x30,  // MCSM1         Main Radio Control State Machine Configuration
                    0x18,  // MCSM0         Main Radio Control State Machine Configuration
                    0x14,  // FOCCFG        Frequency Offset Compensation Configuration
                    0x6C,  // BSCFG         Bit Synchronization Configuration
                    0x07,  // AGCCTRL2      AGC Control
                    0x00,  // AGCCTRL1      AGC Control
                    0x92,  // AGCCTRL0      AGC Control
                    0x87,  // WOREVT1       High Byte Event0 Timeout
                    0x6B,  // WOREVT0       Low Byte Event0 Timeout
                    0xFB,  // WORCTRL       Wake On Radio Control
                    0x56,  // FREND1        Front End RX Configuration
                    0x17,  // FREND0        Front End TX Configuration
                    0xE9,  // FSCAL3        Frequency Synthesizer Calibration
                    0x2A,  // FSCAL2        Frequency Synthesizer Calibration
                    0x00,  // FSCAL1        Frequency Synthesizer Calibration
                    0x1F,  // FSCAL0        Frequency Synthesizer Calibration
                    0x41,  // RCCTRL1       RC Oscillator Configuration
                    0x00,  // RCCTRL0       RC Oscillator Configuration
                    0x59,  // FSTEST        Frequency Synthesizer Calibration Control
                    0x7F,  // PTEST         Production Test
                    0x3F,  // AGCTEST       AGC Test
                    0x81,  // TEST2         Various Test Settings
                    0x35,  // TEST1         Various Test Settings
                    0x09,  // TEST0         Various Test Settings
};
//----------------------[END REGISTER BASIC CONFIGURATION]--------------------

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN Header_processRF_Packets */
/**
* @brief Function implementing the processRF thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_processRF_Packets */
void processRF_Packets(void const * argument)
{
  /* USER CODE BEGIN processRF_Packets */
  /* Infinite loop */
  for(;;)
  {
	  if(RF_packetsAvailable == 1)
	  {
		  uint8_t tx_fifo[60], rx_fifo[60];
		  uint8_t my_addr, tx_addr, rx_addr, pktlen, lqi, rssi;
		  uint8_t sender;
		  int8_t rssi_dbm;

		  RF_packetsAvailable = 0;
		  // Interrupts erstmal disablen....
		  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

		  if(rf_get_payload(rx_fifo, pktlen, my_addr, sender, rssi_dbm, lqi))
		  {

		  }

		  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	  }
    osDelay(50);
  }
  /* USER CODE END processRF_Packets */
}

/* RF DRIVER ----------------------------------------------------------------------------------------------------------------------*/

/*--------------------------[CC1101 Init and Settings]------------------------------*/
uint8_t rf_begin(SPI_HandleTypeDef* hspi, MODULATION_TypeDef mode, ISMBAND_TypeDef ism_band, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint16_t gdo0){
	/**
	 * @brief Calls all the functions needed to make the RF chip operative. This should be the first function used when
	 * using the RF chip.
	 * @param hspi: Pointer to the spi handler
	 * @param mode: Modulation used
	 * @param ism_band Frequency used
	 * @param cs_port: Chip Select (SPI) Pin Port (ie: GPIOD)
	 * @param cs_pin: Chip Select (SPI) Pin number (ie: GPIO_Pin_14)
	 * @param gdo0: Pin number of the pin connected to C1101 CGDO0, used for interruptions. Interruption is configured as FALLING EDGE.
	 *
	 */

	//Pinout linking
	hal_spi = hspi;
	CS_GPIO_Port = cs_port;
	CS_Pin = cs_pin;
	GDO0_Pin = gdo0;

	//Turn on the chip
	rf_reset();

	//Check that the SPI works
	if(!rf_check()){
		return FALSE;
	}


	rf_write_strobe(SFTX); //Flush TX FIFO
	HAL_Delay(1); //TODO I don't think this is really needed
	rf_write_strobe(SFRX); //Flush RX FIFO
	HAL_Delay(1);

	rf_set_modulation_mode(mode);

	rf_set_ISMband(ism_band);
	rf_set_channel(0);
	rf_set_output_power_level(0);
	return TRUE;


}



void rf_reset(){
	/**
	 * @brief Turns on the RF chip with a specific sequence on the CS pin and a SRES command.
	 * The former is only needed on a cold start.
	 */

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	delay_us(10);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	delay_us(40);

	rf_write_strobe(SRES);
	HAL_Delay(1);

}

uint8_t rf_check(){
	/**
	 * @brief Checks the version of the RF chip to check if SPI is OK. It checks 10 times to make sure wires are really OK.
	 */

	uint8_t version;
	version = rf_read_register(VERSION);

	if(version == 0x00 || version == 0xFF)
		return 0;

	return 1;
}

void rf_set_modulation_mode(MODULATION_TypeDef mode){
	/*
	 * @brief Loads the wanted modulation preset to the CC1101.
	 */

    uint8_t* cfg_reg;

    switch (mode)
    {
        case GFSK_1_2_kb:
        			cfg_reg = cc1100_GFSK_1_2_kb;
                    break;
        case GFSK_38_4_kb:
                    cfg_reg = cc1100_GFSK_38_4_kb;
                    break;
        case GFSK_100_kb:
        			cfg_reg = cc1100_GFSK_100_kb;
                    break;
        case MSK_250_kb:
        			cfg_reg = cc1100_MSK_250_kb;
                    break;
        case MSK_500_kb:
        			cfg_reg = cc1100_MSK_500_kb;
                    break;
        case OOK_4_8_kb:
        			cfg_reg = cc1100_OOK_4_8_kb;
                    break;
        default:
        			cfg_reg = cc1100_GFSK_38_4_kb;
                    break;
    }

    rf_write_data(WRITE_BURST(0), cfg_reg, CFG_REGISTER);


}

//(Semi)DEPRECATED
void rf_set_ISMband(ISMBAND_TypeDef band){
	/*
	 * Deprecated by rf_set_frequency(float), although the second still doesn't configure the PATABLES registers, so it is still needed.
	 */
    uint8_t freq2, freq1, freq0;
    uint8_t* patable;

    switch (band)
    {
        case MHz315:
                    freq2=0x0C;
                    freq1=0x1D;
                    freq0=0x89;
                    patable = patable_power_315;
                    break;
        case MHz434:                                                          //433.92MHz
                    freq2=0x10;
                    freq1=0xB0;
                    freq0=0x71;
                    patable = patable_power_433;
                    break;
        case MHz868:                                                          //868.3MHz
                    freq2=0x21;
                    freq1=0x65;
                    freq0=0x6A;
                    patable = patable_power_868;
                    break;
        case MHz915:
                    freq2=0x23;
                    freq1=0x31;
                    freq0=0x3B;
                    patable = patable_power_915;
                    break;
        default:                                                          //868.3MHz
					freq2=0x21;
					freq1=0x65;
					freq0=0x6A;
					patable = patable_power_868;
					break;
    }
    rf_write_register(FREQ2,freq2);
    rf_write_register(FREQ1,freq1);
    rf_write_register(FREQ0,freq0);
    rf_write_data(PATABLE_BURST, patable, 8);
}

void rf_set_addr(uint8_t myAddr)
{
	rf_write_register(ADDR, myAddr);
}

void rf_set_channel(uint8_t channel){
	/*
	 * @brief Set channel number.
	 */
	rf_write_register(CHANNR, channel);
}

void rf_set_output_power_level(int8_t dBm)
/*
 * @brief Selects the entry of the PATABLES preset selected previously.
 */
{
    uint8_t pa = 0xC0;

    if      (dBm <= -30) pa = 0x00;
    else if (dBm <= -20) pa = 0x01;
    else if (dBm <= -15) pa = 0x02;
    else if (dBm <= -10) pa = 0x03;
    else if (dBm <= 0)   pa = 0x04;
    else if (dBm <= 5)   pa = 0x05;
    else if (dBm <= 7)   pa = 0x06;
    else if (dBm <= 10)  pa = 0x07;

    rf_write_register(FREND0,pa);
}

float rf_set_carrier_offset(float offset){
	/*
	 * @Brief Configures frequency offset register to achieve the tergeted offset.
	 * @param offset Desired offset. Should be between -200KHz and +200KHz, depends on crystal.
	 * @returns The actual offset
	 */
	//rf_write_register(FSCTRL0, offset);
	int8_t freqoff = offset*(1<<14)/CRYSTAL_FREQUENCY;
	rf_write_register(FSCTRL0, freqoff);
	return freqoff*(CRYSTAL_FREQUENCY/(1<<14));
}

float rf_set_carrier_frequency(float target_freq){
	/* Note that this functions depends on the value of CRYSTAL_FREQUENCY_M.
	 * @param target_freq Frequency targeted, in MHz. Positive number. Note that the actual frequency may vary.
	 * @return Actual configured frequency.
	 */
	target_freq = target_freq*1000000;
	float freqf = target_freq*65536.0/(float)CRYSTAL_FREQUENCY_M;
	uint32_t freq = (uint32_t)freqf;
	freq = freq&0x00FFFFFF;
	rf_write_register(FREQ0, freq);
	rf_write_register(FREQ1, (freq>>8));
	rf_write_register(FREQ2, (freq>>16));
	float t = ((float)freq*(float)CRYSTAL_FREQUENCY_M)/65536.0;

	return t;
}

float rf_set_channel_spacing(float cspacing){
	/*
	 * @brief Configures channel spacing registers to achieve the closer spacing possible to the target spacing
	 * Note that this functions depends on the value of CRYSTAL_FREQUENCY_M.
	 * @param cspacing Target spacing, in KHz. Positive number.
	 * @returns The actual configured spacing, in KHz
	 */
	uint8_t chanspc_e = 0;
	uint8_t chanspc_m = 0;
	float tmp;

	tmp = cspacing*((1<<18)/((float)CRYSTAL_FREQUENCY*(1<<chanspc_e)))-256.0;
	while(tmp>256 && chanspc_e<4){
		chanspc_e++;
		tmp = cspacing*((1<<18)/((float)CRYSTAL_FREQUENCY*(1<<chanspc_e)))-256.0;
	}
	chanspc_m = (uint8_t)tmp;
	rf_write_register(MDMCFG0, chanspc_m);

	uint8_t mdmcfg1 = rf_read_register(MDMCFG1);
	mdmcfg1 &= 0xFC;
	mdmcfg1 |= (chanspc_e & 0x2);
	rf_write_register(MDMCFG1, mdmcfg1);

	cspacing = ((float)CRYSTAL_FREQUENCY/(1<<18))*((float)chanspc_m+256.0)*(1<<chanspc_e);
	return cspacing;
}

void rf_set_preamble(uint8_t nbytes){
	/*
	 * @brief Sets the preamble size. The preamble is a stream of 1s and 0s that is sent before the packet.
	 */
	rf_sidle();
	//TODO Rright now it is harcoded to be 8 bytes.

}

void rf_set_preamble_threshold(uint8_t nbytes){
	/*
	 * @brief Sets the minimum preamble bytes to detect.
	 */
	rf_sidle();
	//TODO Right now it is hardocded to 4 bytes.

}


/*----------------------------[CC1101 States]----------------------------------------------*/
void rf_sidle(){
	/**
	 * @brief Set RF chip to idle state
	 */
    uint8_t marcstate;

    rf_write_strobe(SIDLE);              //sets to idle first. must be in

    marcstate = 0xFF;                     //set unknown/dummy state value

    while(marcstate != IDLE)
    {
        marcstate = (rf_read_register(MARCSTATE) & 0x1F);
    }

    HAL_Delay(100);

}

uint8_t rf_receive(){
	/*
	 * @brief Set RF chip to receive state (RX)
	 */
	//configure interruption, 1 when incoming packet
	rf_write_register(IOCFG0, 0x06);
	rf_write_strobe(SFRX);
	rf_write_strobe(SRX);

	uint8_t marcstate = 0xFF;
	while(marcstate != RX){
		marcstate = (rf_read_register(MARCSTATE)); //read out state of cc1100 to be sure in RX
	}

	HAL_Delay(100);
	return 1;
}

uint8_t rf_transmit()
{
		rf_sidle();
		rf_write_strobe(STX);

		uint8_t marcstate = 0xFF;
		while(marcstate != TX){
			marcstate = (rf_read_register(MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in TX
		}

		HAL_Delay(100);
		return 1;
}

uint8_t rf_tx_payload_burst(uint8_t my_addr, uint8_t rx_addr,uint8_t *txbuffer, uint8_t length)
{
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
	txbuffer[0] = length -1;
	txbuffer[1] = rx_addr;
	txbuffer[2] = my_addr;

	rf_write_data(TXFIFO_BURST, txbuffer, length);

	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
	return 1;
}

uint8_t rf_rx_payload_burst(uint8_t rxbuffer[], uint8_t &pktlen)
{
	uint8_t bytes_in_RXFIFO = 0;
	uint8_t res = 0;

	bytes_in_RXFIFO = rf_read_register(RXBYTES);

	if((bytes_in_RXFIFO & 0x7F) && !(bytes_in_RXFIFO & 0x80))
	{
		rf_read_data(RXFIFO_BURST, rxbuffer, bytes_in_RXFIFO);
		pktlen = rxbuffer[0];
		res = 1;
	}
	else
	{
		rf_sidle();
		rf_write_strobe(SFRX);
		HAL_Delay(100);
		rf_receive();
		res = 0;
	}

	return res;
}

uint8_t rf_sent_packet(uint8_t my_addr, uint8_t rx_addr, uint8_t *txbuffer,uint8_t pktlen,  uint8_t tx_retries)
{
	uint8_t pktlen_ack, rssi, lqi;
	uint8_t rxbuffer[FIFOBUFFER];
	uint8_t tx_retries_count = 0;
	uint8_t from_sender;
	uint8_t ackWaitCounter = 0;

	if(pktlen > (FIFOBUFFER -1))
	{
		return 0;
	}

	do {

		rf_tx_payload_burst(my_addr, rx_addr, txbuffer, pktlen);
		rf_transmit();
		rf_receive();

		if(rx_addr == BROADCAST_ADDRESS) {
			return 1;
		}

		while(ackWaitCounter < ACK_TIMEOUT)
		{
			if(RF_packetsAvailable == 1)
			{
				from_sender = rx_addr;
				rf_rx_fifo_erase(rxbuffer);
				rf_rx_payload_burst(rxbuffer, pktlen_ack);
				rf_check_acknowledge(rxbuffer, pktlen_ack, from_sender, my_addr);
				return 1;
			}
			else
			{
				ackWaitCounter++;
				HAL_Delay(1);
			}
		}

		ackWaitCounter = 0;
		tx_retries_count ++;

	} while(tx_retries_count <= tx_retries);

	return 0;
}

uint8_t rf_get_payload(uint8_t rxbuffer[], uint8_t &pktlen, uint8_t &my_addr,uint8_t &sender, int8_t &rssi_dbm, uint8_t &lqi)
{
	uint8_t crc;

	rf_rx_fifo_erase(rxbuffer);

	if(rf_rx_payload_burst(rxbuffer, pktlen) == 0)
	{
		rf_rx_fifo_erase(rxbuffer);
		return 0;
	}
	else
	{
		my_addr = rxbuffer[1];
		sender = rxbuffer[2];

		if(rf_check_acknowledge(rxbuffer, pktlen, sender, my_addr) == 1)
		{
			rf_rx_fifo_erase(rxbuffer);
			return 0;
		}
		else
		{
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
			rssi_dbm = rf_rssi_convert(rxbuffer[pktlen+1]);
			lqi = rf_lqi_convert(rxbuffer[pktlen+2]);
			crc = rf_check_crc(lqi);

			my_addr = rxbuffer[1];
			sender = rxbuffer[2];

			rf_sent_acknowledge(my_addr, sender);
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
		}

		return 1;
	}

	return 0;
}

uint8_t rf_check_crc(uint8_t lqi)
{
    return (lqi & 0x80);
}

uint8_t rf_lqi_convert(uint8_t lqi)
{
    return (lqi & 0x7F);
}

uint8_t rf_rssi_convert(uint8_t Rssi_hex)
{
    int8_t rssi_dbm;
    int16_t Rssi_dec;

    Rssi_dec = Rssi_hex;        //convert unsigned to signed

    if(Rssi_dec >= 128){
        rssi_dbm=((Rssi_dec-256)/2)-RSSI_OFFSET_868MHZ;
    }
    else{
        if(Rssi_dec<128){
            rssi_dbm=((Rssi_dec)/2)-RSSI_OFFSET_868MHZ;
        }
    }
    return rssi_dbm;
}


void rf_sent_acknowledge(uint8_t my_addr, uint8_t tx_addr)
{
	uint8_t pktlen = 0x06;
	uint8_t tx_buffer[0x06];

	tx_buffer[3] = 'A'; tx_buffer[4] = 'c'; tx_buffer[5] = 'k';

	rf_tx_payload_burst(my_addr, tx_addr, tx_buffer, pktlen);
	rf_transmit();
	rf_receive();
}

uint8_t rf_check_acknowledge(uint8_t *rxbuffer, uint8_t pktlen, uint8_t sender, uint8_t my_addr)
{
	int8_t rssi_dbm;
	uint8_t crc, lqi;

	if((pktlen == 0x05 &&
			(rxbuffer[1] == my_addr || rxbuffer[1] == BROADCAST_ADDRESS)) && \
			rxbuffer[2] == sender && \
			rxbuffer[3] == 'A' && rxbuffer[4] == 'c' && rxbuffer[5] == 'k')
	{
		if(rxbuffer[1] == BROADCAST_ADDRESS)
		{
			return 0;
		}

		rssi_dbm = rf_rssi_convert(rxbuffer[pktlen+1]);
		lqi = rf_lqi_convert(rxbuffer[pktlen+2]);
		crc = rf_check_crc(lqi);

		return 1;
	}
	return 0;
}

void rf_tx_fifo_erase(uint8_t *txbuffer)
{
    memset(txbuffer, 0, sizeof(FIFOBUFFER));  //erased the TX_fifo array content to "0"
}

void rf_rx_fifo_erase(uint8_t *rxbuffer)
{
    memset(rxbuffer, 0, sizeof(FIFOBUFFER)); //erased the RX_fifo array content to "0"
}

void rf_power_down(){
    rf_sidle();
    rf_write_strobe(SPWD);               // CC1100 Power Down
}

void rf_wakeup(void){
	/*
	 * @brief Wakes up the c1101 from power down.
	 */
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    //TODO rf_receive();                            // go to RX Mode
}

void rf_wor_enable(){
	/*
	 * @brief enables WOR Mode  EVENT0 ~1890ms; rx_timeout ~235ms
	 */
	/*
		EVENT1 = WORCTRL[6:4] -> Datasheet page 88
		EVENT0 = (750/Xtal)*(WOREVT1<<8+WOREVT0)*2^(5*WOR_RES) = (750/26Meg)*65407*2^(5*0) = 1.89s
							(WOR_RES=0;RX_TIME=0)               -> Datasheet page 80
	i.E RX_TimeOut = EVENT0*       (3.6038)      *26/26Meg = 235.8ms
							(WOR_RES=0;RX_TIME=1)               -> Datasheet page 80
	i.E.RX_TimeOut = EVENT0*       (1.8029)      *26/26Meg = 117.9ms
	*/


	rf_write_register(IOCFG2, 0x06);

    rf_sidle();

    rf_write_register(MCSM0, 0x18);    //FS Autocalibration
    rf_write_register(MCSM2, 0x01);    //MCSM2.RX_TIME = 1b

    // configure EVENT0 time
    rf_write_register(WOREVT1, 0xFF);  //High byte Event0 timeout
    rf_write_register(WOREVT0, 0x7F);  //Low byte Event0 timeout

    // configure EVENT1 time
    rf_write_register(WORCTRL, 0x78);  //WOR_RES=0b; tEVENT1=0111b=48d -> 48*(750/26MHz)= 1.385ms

    rf_write_strobe(SFRX);             //flush RX buffer
    rf_write_strobe(SWORRST);          //resets the WOR timer to the programmed Event 1
    rf_write_strobe(SWOR);             //put the radio in WOR mode when CSn is released

    HAL_Delay(100); //TODO Really necessary?

    wor_enable_flag = 1;
}

void rf_wor_disable(){

	if(wor_enable_flag == 1)
	{
		rf_wor_disable();
		wor_enable_flag = 0;
		rf_write_data(PATABLE_BURST, patable_power_868, 8);

		rf_sidle();                            //exit WOR Mode
		rf_write_register(MCSM2, 0x07); //stay in RX. No RX timeout
	}
}

void rf_wor_reset(){
    rf_sidle();                            //go to IDLE
    rf_write_register(MCSM2, 0x01);    //MCSM2.RX_TIME = 1b
    rf_write_strobe(SFRX);             //flush RX buffer
    rf_write_strobe(SWORRST);          //resets the WOR timer to the programmed Event 1
    rf_write_strobe(SWOR);             //put the radio in WOR mode when CSn is released

    HAL_Delay(100); //Really necessary?
}

/* NEW FUNCTIONS------------------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == RF_GDO2_Pin)
	{
		RF_packetsAvailable = 1;
	}
	else if(GPIO_Pin == BTN_LINK_Pin)
	{

	}

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}


/* SPI Comm ----------------------------------------------------------------*/

void rf_write_strobe(uint8_t strobe){
	/**
	 * @brief Writes command to the CC1101 to change its state-machine state.
	 */
	strobe = WRITE(strobe);
	__spi_write(&strobe, NULL, NULL);
}

uint8_t rf_read_register(uint8_t reg){
	/**
	 * @brief Reads the content of a single 1-byte register.
	 * @Returns The register value.
	 */
	uint8_t data;
	reg= READ(reg);
	__spi_read(&reg, &data, 1);
	return data;
}

void rf_write_register(uint8_t reg, uint8_t data){
	/**
	 * @brief Overwrites a register.
	 */
	reg = WRITE(reg);
	__spi_write(&reg, &data, 1);
}

void rf_read_data(uint8_t addr, uint8_t* data, uint8_t size){
	/**
	 * @brief Reads multiple data.
	 * @param addr Base address.
	 * @param data The buffer where the read data will be stored.
	 * @param size Number of bytes to be read.
	 */
	if(size>1){
		addr = READ_BURST(addr);
	}else{
		addr = READ(addr);
	}
	__spi_read(&addr, data, size);
}

void rf_write_data(uint8_t addr, uint8_t* data, uint8_t size){
	/**
	 * @brief Writes multiple data.
	 * @param addr Base address.
	 * @param data The buffer where the data to be written is located.
	 * @param size Number of bytes to be written.
	 */
	if(size>1){
		addr = WRITE_BURST(addr);
	}else{
		addr = WRITE(addr);
	}
	__spi_write(&addr, data, size);
}

/* SPI Handling -------------------------------------------------------------*/

HAL_StatusTypeDef __spi_write(uint8_t *addr, uint8_t *pData, uint16_t size){
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //set Chip Select to Low
	status = HAL_SPI_Transmit(hal_spi, addr, 1, 0xFFFF);
	if(status==HAL_OK && pData!=NULL)
		status = HAL_SPI_Transmit(hal_spi, pData, size, 0xFFFF);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //set Chip Select to High
	return status;

}

HAL_StatusTypeDef __spi_read(uint8_t *addr, uint8_t *pData, uint16_t size){

	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //set Chip Select to Low
	status = HAL_SPI_Transmit(hal_spi, addr, 1, 0xFFFF);
	status = HAL_SPI_Receive(hal_spi, pData, size, 0xFFFF);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //set Chip Select to High

	return status;

}

/* INT Handling -------------------------------------------------------------*/



void init_serial(UART_HandleTypeDef* huart)
{

	hal_uart = huart;
}
