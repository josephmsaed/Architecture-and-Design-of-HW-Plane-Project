/******************************************************************************
*
* Copyright (C) 2010 - 2015 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

#include "xparameters.h"
#include "xil_cache.h"

#include "platform_config.h"

#include "platform.h"
//#include "spidefs.h"
//#include "bitmap.h"
//#include "PmodOLEDrgb.h"
#include "sleep.h"


#include "xil_io.h"

/*
 * Uncomment one of the following two lines, depending on the target,
 * if ps7/psu init source files are added in the source directory for
 * compiling example outside of SDK.
 */
/*#include "ps7_init.h"*/
/*#include "psu_init.h"*/

#ifdef STDOUT_IS_16550
 #include "xuartns550_l.h"

 #define UART_BAUD 9600
#endif

void
enable_caches()
{
#ifdef __PPC__
    Xil_ICacheEnableRegion(CACHEABLE_REGION_MASK);
    Xil_DCacheEnableRegion(CACHEABLE_REGION_MASK);
#elif __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_ICACHE
    Xil_ICacheEnable();
#endif
#ifdef XPAR_MICROBLAZE_USE_DCACHE
    Xil_DCacheEnable();
#endif
#endif
}

void
disable_caches()
{
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_DCACHE
    Xil_DCacheDisable();
#endif
#ifdef XPAR_MICROBLAZE_USE_ICACHE
    Xil_ICacheDisable();
#endif
#endif
}

void
init_uart()
{
#ifdef STDOUT_IS_16550
    XUartNs550_SetBaud(STDOUT_BASEADDR, XPAR_XUARTNS550_CLOCK_HZ, UART_BAUD);
    XUartNs550_SetLineControlReg(STDOUT_BASEADDR, XUN_LCR_8_DATA_BITS);
#endif
    /* Bootrom/BSP configures PS7/PSU UART to 115200 bps */
}

void
init_platform()
{
    /*
     * If you want to run this example outside of SDK,
     * uncomment one of the following two lines and also #include "ps7_init.h"
     * or #include "ps7_init.h" at the top, depending on the target.
     * Make sure that the ps7/psu_init.c and ps7/psu_init.h files are included
     * along with this example source files for compilation.
     */
    /* ps7_init();*/
    /* psu_init();*/
    enable_caches();
    init_uart();
}

void
cleanup_platform()
{
    disable_caches();
}

/********************************************************
 * Specific Driver for IMU application
 * Author : Dr. Arnaud Dion
 * 2017
 ********************************************************/


/******************************************************************************
* @brief Generates a wait delay.
*
* @param ms_count - nb of ms to wait (ABSOLUTLY UNPRECISE !).
*
* @return None.
******************************************************************************/
void delay_ms(unsigned long ms_count)
{
	unsigned long count;
	for (count = 0; count < ((ms_count * 66700) + 1); count++)
	{
	  asm("nop");
	}
}

/******************************************************************************
* @brief Generates a wait delay.
*
* @param ms_count - nb of ms to wait (ABSOLUTLY UNPRECISE !).
*
* @return None.
******************************************************************************/
void delay_us(unsigned long us_count)
{
	unsigned long count;
	for (count = 0; count < ((us_count * 67) + 1); count++)
	{
	  asm("nop");
	}
}

/******************************************************************************
* @brief Writes to an ADXL362 Register.
*
* @param addr - register address.
* @param data - data to be written
*
* @return None.
******************************************************************************/
void ADXL362_WriteReg(char addr, char data)
{
	// Pointer to HW registers
	volatile unsigned long* Config_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_CONFIG_REG); // register addr = 0x00
	volatile unsigned long* Intr_Status_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_STS_REG); // register addr = 0x04
	volatile unsigned long* Intrpt_en_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_EN_REG); // register addr = 0x08
	volatile unsigned long* Intrpt_dis_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_DIS_REG); // register addr = 0x0C
	volatile unsigned long* Tx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_TXDATA_REG); // register addr = 0x1C
	volatile unsigned long* Rx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_RXDATA_REG); // register addr = 0x20
	volatile unsigned long* En_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_EN_REG); // register addr = 0x14
	// Local copy (stored in memory) of HW register
	unsigned long _Config_reg0  	= 0;
	unsigned long Tmp_Config_reg0  	= 0;
	unsigned long _Intr_Status_reg0 	= 0;
	// Received data from SPI
	char rx = 0x00;

	// Procedure : See user guide ug585 v1.11 page 544
	// See 17.3.3 Master Mode Data Transfer
	// Example: Master Mode  Manual SS and Auto Start
	// Refer to ADXL362 datasheet revD
	// page 20-21

	// Point 1 :
	// It has already been done during SPI_Init()
	// Can be done here, harmless but mainly useless, so skip it

	// Point 2 :
	// Write configuration data to manually assert CS
	_Config_reg0 = *Config_reg0;
	Tmp_Config_reg0 = _Config_reg0 & 0xFFFFFBFF;  //             1011  1111  1111
	*Config_reg0=Tmp_Config_reg0;


	// Point 3 :
	// Enable the controller
	*En_reg0 = 0x01;

	// Point 4 :
	// Write data to TX FIFO
	// Refer to ADXL362 datasheet revD
	// page 20-21
	*Tx_data_reg0 = 0x0A;	//write register
	*Tx_data_reg0 = addr;
	*Tx_data_reg0 = data;

	// Point 5 :
	// Enable TX FIFO Not Full Interrupts
	// Useful to do the polling on TX FIFO Empty event
	// We use only the TxFIFO empty event, not RxFIFO full, RxFIFO overflow, or fault conditions.
	// So we can write 0x04 instead of 0x27
	// But it's possible to write 0x27, it doesn't change our application
	*Intrpt_en_reg0=0x00000004;

	// Point 5 bis :
	// This line erase eventually remainding flag occured on a previous Tx Fifo empty event
	// See user guide ug585 v1.11
	// Annexe B.30, page 1755 and 1756
	// This line can be avoided, but it's a good practice to flush a register before polling
	*Intr_Status_reg0=0x00000004;


	// Point 6 :
	// Wait for TX FIFO to empty
	// Do the polling on bit 2 of register 4
	do
	{
		_Intr_Status_reg0 = *Intr_Status_reg0;
	}
	while((_Intr_Status_reg0 & 0x00000004) == 0x00);

	// Clear Interrupt
	// See above Point 5 bis :
	*Intr_Status_reg0 = 0x00000004;

	// Point 7 :
	// Not concerned because there is no more data to send

	// Point 8 :
	// Disable the interrupts
	// We use only the TxFIFO empty event, not RxFIFO full, RxFIFO overflow, or fault conditions.
	// So we can write 0x04 instead of 0x27
	// But it's possible to write 0x27, it doesn't change our application
	*Intrpt_dis_reg0 = 0x00000004;

	// Point 9 :
	// Deassert /CS
	// Here there is an error in the documentation (ug585)
	// Easy to understand : The /CS should be deasserted before the controller is disabled !!
	*Config_reg0=_Config_reg0;


	// Point 10 :
    // No need to disable SPI here

	// And Finally flush the data from RX FIFO !!
	// The data are dumped because of the simultaneous read and write on SPI bus

	rx=*Rx_data_reg0;
	rx=*Rx_data_reg0;
	rx=*Rx_data_reg0;

}

/******************************************************************************
* @brief Read a data from an ADXL362 Register.
*
* @param addr - register address.
*
* @return data received.
******************************************************************************/
char ADXL362_ReadReg(char addr)
{
	// Pointer to HW registers
	// refer to user guide ug585 v1.11
	// Appendix B.30, page 1753
	volatile unsigned long* Config_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_CONFIG_REG); // register addr = 0x00
	volatile unsigned long* Intr_Status_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_STS_REG); // register addr = 0x04
	volatile unsigned long* Intrpt_en_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_EN_REG); // register addr = 0x08
	volatile unsigned long* Intrpt_dis_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_DIS_REG); // register addr = 0x0C
	volatile unsigned long* Tx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_TXDATA_REG); // register addr = 0x1C
	volatile unsigned long* Rx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_RXDATA_REG); // register addr = 0x20
	volatile unsigned long* En_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_EN_REG); // register addr = 0x14
	// Local copy (stored in memory) of HW registers
	unsigned long _Config_reg0  	= 0;
	unsigned long Tmp_Config_reg0  	= 0;
	unsigned long _Intr_Status_reg0 	= 0;
	// Received data from SPI
	char rx = 0x00;

	// Procedure : See user guide ug585 v1.11 page 544
	// See 17.3.3 Master Mode Data Transfer
	// Example: Master Mode � Manual SS and Auto Start
	// Refer to ADXL362 datasheet revD
	// page 20-21

	// Point 1 :
	// It has already been done during SPI_Init()
	// Can be done here, harmless but mainly useless, so skip it

	// Point 2 :
	// Write configuration data to manually assert CS
	_Config_reg0 =*Config_reg0;
	Tmp_Config_reg0 = _Config_reg0& 0xFFFFFBFF;
	*Config_reg0=Tmp_Config_reg0;

	// Point 3 :
	// Enable the controller
	*En_reg0 = 0x01;

	// Point 4 :
	// Write data to TX FIFO
	// Refer to ADXL362 datasheet revD
	// page 20-21
	*Tx_data_reg0 = 0x0B;
	*Tx_data_reg0 = addr;
	*Tx_data_reg0 = 0x00;

	// Point 5 :
	// Enable TX FIFO Not Full Interrupts
	// Useful to do the polling on TX FIFO Empty event
	// We use only the TxFIFO empty event, not RxFIFO full, RxFIFO overflow, or fault conditions.
	// So we can write 0x04 instead of 0x27
	// But it's possible to write 0x27, it doesn't change our application
	*Intrpt_en_reg0=0x00000004;

	// Point 5 bis :
	// This line erase eventually remainding flag occured on a previous Tx Fifo empty event
	// See user guide ug585 v1.11
	// Annexe B.30, page 1755 and 1756
	// This line can be avoided, but it's a good practice to flush a register before polling
	*Intr_Status_reg0=0x00000004;

	// Point 6 :
	// Wait for TX FIFO to empty
	// Do the polling on bit 2 of register 4
	do
	{
		_Intr_Status_reg0 = *Intr_Status_reg0;
	}
	while((_Intr_Status_reg0 & 0x00000004) == 0x00);

	// Clear Interrupt
	// See above Point 5 bis :
	*Intr_Status_reg0=0x00000004;

	// Point 7 :
	// Not concerned because there is no more data to send

	// Point 8 :
	// Disable the interrupts
	// We use only the TxFIFO empty event, not RxFIFO full, RxFIFO overflow, or fault conditions.
	// So we can write 0x04 instead of 0x27
	// But it's possible to write 0x27, it doesn't change our application
	*Intrpt_dis_reg0 = 0x00000004;

	// Point 9 :
	// Deassert /CS
	// Here there is an error in the documentation (ug585)
	// Easy to understand : The /CS should be deasserted before the controller is disabled !!
	*Config_reg0=_Config_reg0;

	// Point 10 :
	// No need to disable SPI here

	// And Finally read the data from RX FIFO !!
	// The first 2 data are dumped because of the simultaneous read and write on SPI bus
	rx=*Rx_data_reg0;
	rx=*Rx_data_reg0;
	// Useful Data
	rx=*Rx_data_reg0;
	// Send useful data
	return(rx);
}

void JSTK_readval(float *rx)
{
	volatile unsigned long* Config_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_CONFIG_REG);
	volatile unsigned long* Intr_Status_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_STS_REG);
	volatile unsigned long* Tx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_TXDATA_REG);
	volatile unsigned long* Rx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_RXDATA_REG);
	volatile unsigned long* En_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_EN_REG); // register addr = 0x14


	uint8_t rxdata[8];
	uint16_t twobytes_x;
	uint16_t twobytes_y;

	unsigned long _Config_reg0  	= 0;
	unsigned long Tmp_Config_reg0  	= 0;
	unsigned long _Intr_Status_reg0 	= 0;

    // Initialization of the SPI for the joystick
	SPI_JSTK_Init();

	// Wait for 10ms
	delay_ms(10);

	// assert CS
	_Config_reg0 =*Config_reg0;
	Tmp_Config_reg0 = _Config_reg0& 0xFFFFF7FF;
	*Config_reg0=Tmp_Config_reg0;

	// Point 3 :
	// Enable the controller
	*En_reg0 = 0x01;

	// Wait for 15us
	delay_us(15);

	for(int i=0; i<5; i++){
		// flush a register before polling
		*Intr_Status_reg0=SPI_TX_FIFO_not_full;

		// wait SPI_TX_FIFO_not_full
		do
		{
			_Intr_Status_reg0 = *Intr_Status_reg0;
		}while((_Intr_Status_reg0 & SPI_TX_FIFO_not_full) == 0x00);

		// send data
		*Tx_data_reg0 = 0x00;

		// start command
		Tmp_Config_reg0 = 0x10000;
		*Config_reg0=*Config_reg0 | Tmp_Config_reg0;

		// flush a register before polling
		*Intr_Status_reg0=SPI_RX_FIFO_not_empty;

		// wait SPI_RX_FIFO_not_empty
		do
		{
			_Intr_Status_reg0 = *Intr_Status_reg0;
		}
		while((_Intr_Status_reg0 & SPI_RX_FIFO_not_empty) == 0x00);

		// read data
		rxdata[i] = *Rx_data_reg0;

        // Wait for 10us
		delay_us(10);

	}
	// Convert data to floating point values

	twobytes_x = (rxdata[1]<<8) | (rxdata[0]);
	twobytes_y = (rxdata[3]<<8) | (rxdata[2]);
	*rx = (float)(twobytes_x);
	*(rx+1) = (float)(twobytes_y);
	*(rx+2) = (float)rxdata[4];

	// deassert CS
	*Config_reg0=_Config_reg0;

}
/******************************************************************************
* @brief Initialization of the accelerometer for the measurements.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADXL362_Init(void)
{
	// Software Reset
	// Refer to ADXL362datasheet revD
	// page 27
	ADXL362_WriteReg(ADXL362_SOFT_RESET, ADXL362_RESET_CMD);
	delay_ms(10);
	ADXL362_WriteReg(ADXL362_SOFT_RESET, 0x00);

	// Enable Measurement
	// Refer to ADXL362datasheet revD
	// page 35
	ADXL362_WriteReg(ADXL362_POWER_CTL, (2 << ADXL362_MEASURE));
	delay_ms(500);
}

/******************************************************************************
* @brief Set the range of the acceleration to measure.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADXL362_SetRange(void)
{

	// Refer to datasheet revD
	// page 34

	// Local copy (stored in memory) of HW register
	char _FILTER_CTL 	= 0;

	_FILTER_CTL = (ADXL362_ReadReg(ADXL362_FILTER_CTL) & 0x3F);

	//Set to +/-2g range
	_FILTER_CTL = 0x00 | _FILTER_CTL;

	ADXL362_WriteReg(ADXL362_FILTER_CTL, _FILTER_CTL);


}


/******************************************************************************
* @brief Initialization of the SPI controler.
*
* @param None.
*
* @return None.
******************************************************************************/
void SPI_ADXL_Init(void)
{



	// Pointer to HW registers
	volatile unsigned long* Config_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_CONFIG_REG);
	volatile unsigned long* Intr_Status_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_STS_REG);
	volatile unsigned long* Rx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_RXDATA_REG);
	volatile unsigned long* TX_thres_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_TX_THRES_REG);
	// Local copy (stored in memory) of HW register
//	unsigned long _Config_reg0 = 0;
	// Received data from SPI
	unsigned long DumpData = 0;
	// for loop index
	unsigned long i = 0;

//	_Config_reg0 = *Config_reg0;
//	// nouvelle config
//	// attention : horloge base baudrate = 128MHz
//	_Config_reg0 |= SPI_Modefail_gen_en
//					| SPI_Manual_CS
//					| SPI_Man_start_en
//					| SPI_CS_Bits*0xF
//					| SPI_BAUD_RATE_DIV*6
//					| SPI_MODE_SEL;
	// Configuration Register Settings
	// Refer to user guide ug585 v1.11
	// Appendix B.30, page 1754
//	_Config_reg0 |= (1 << SPI_Modefail_gen_en)      |
//                (0 << SPI_Man_start_com)        |
//                (0 << SPI_Man_start_en)         |
//                (1 << SPI_Manual_CS)            |
//                (0x0F << SPI_CS_Bits)           |
//                (0 << SPI_PERI_SEL)             |
//                (0 << SPI_REF_CLK)          |
//                (4 << SPI_BAUD_RATE_DIV)    |
//                (0 << SPI_CLK_PH)        |
//                (0 << SPI_CLK_POL)       |
//                (1 << SPI_MODE_SEL);

	*Config_reg0= (1 << 17)      |
	                (0 << 16)        |
	                (0 << 15)         |
	                (1 << 14)            |
	                (0x0F << 10)           |
	                (0 << 9)             |
	                (0 << 8)          |
	                (6 << 3)    |
	                (0 << 2)        |
	                (0 << 1)       |
	                (1 << 0);


//	*Config_reg0 = _Config_reg0;

	*TX_thres_reg0 = 1;
	// Flush RX FIFO
	while((*Intr_Status_reg0 & SPI_RX_FIFO_not_empty) != 0)	DumpData= *Rx_data_reg0;
//	for(i = 0; i < 128; i++)
//	{
//		DumpData = *Rx_data_reg0;
//	}

//
//    // Write TX FIFO Not Full Level
//    *TX_thres_reg0 = 0x01;

}

void SPI_JSTK_Init(void)
{

	// Pointer to HW registers
	volatile unsigned long* Config_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_CONFIG_REG);
	volatile unsigned long* Intr_Status_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_INTR_STS_REG);
	volatile unsigned long* Rx_data_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_RXDATA_REG);
	volatile unsigned long* TX_thres_reg0 = (unsigned long*)(SPI_BASEADDR + HW_SPI_TX_THRES_REG);
	// Received data from SPI
	unsigned long DumpData = 0;
	// for loop index
	unsigned long i = 0;


//	// nouvelle config
//	// attention : horloge base baudrate = 128MHz
	*Config_reg0 = SPI_Modefail_gen_en
					| SPI_Manual_CS
					| SPI_Man_start_en
					| SPI_CS_Bits*0xF
					| SPI_BAUD_RATE_DIV*6
					| SPI_MODE_SEL;
	*TX_thres_reg0 = 1;
	// Flush RX FIFO
	while((*Intr_Status_reg0 & SPI_RX_FIFO_not_empty) != 0)	DumpData= *Rx_data_reg0;
}

/******************************************************************************
* @brief Poll the ADXL362 for data.
*
* @param None.
*
* @return char : 1 it data, 0 otherwise.
******************************************************************************/
char ADXL362_IsDataReady(void)
{
	// Point 6 :
	// Wait for TX FIFO to empty
	// Do the polling on bit 2 of register 4
	if((ADXL362_ReadReg(ADXL362_STATUS) & (1 << ADXL362_DATA_READY)) == 0x01)
	{
		return(1);
	}
	else
	{
		return(0);
	}
}

/******************************************************************************
* @brief Display an acceleration value in floating point.
*
* @param rxG - Raw acceleration value.
*
* @return none.
******************************************************************************/
float ADXL362_Convert(char rx)
{
	int 	sign 		= 0;
	float 	frxG 		= 0.0;
	int rxG = 0;

	// Sign is MSB. If 1 -> Negative Number, else Positive Number
	sign = (rx & 0x80) >> 7;

	// Shift by 4 bits
	// Because we read only the MSB data of a 12-bit data
	// Refer to ADXL362datasheet revD
	// page 25
	rxG = (rx << 4);
	if(sign == 1)
		rxG |= 0xFFFFF000;// If negative number, padding with FFFFF MSB's

	//Sensitivity for +/-2g range
	// Refer to ADXL362datasheet revD
	// table 1, page 5
	frxG = ((float)rxG / 1000.0);

	return frxG;

}

/******************************************************************************
* @brief Print ID of ADXL362.
*
* @param None
*
* @return none.
******************************************************************************/
void ADXL362_PrintID(void)
{
	xil_printf("\n\r");
	xil_printf("Device ID:  0x%x\n",ADXL362_ReadReg(ADXL362_DEVID_AD) & 0xFF);
	xil_printf("Device ID:  0x%x\n",ADXL362_ReadReg(ADXL362_DEVID_MST) & 0xFF);
	xil_printf("Part ID:    0x%x\n",ADXL362_ReadReg(ADXL362_PARTID) & 0xFF);
	xil_printf("Silicon ID: 0x%x\n",ADXL362_ReadReg(ADXL362_REVID) & 0xFF);


	xil_printf("\n\r");
}




