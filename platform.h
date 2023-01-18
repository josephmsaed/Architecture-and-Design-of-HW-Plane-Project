/******************************************************************************
*
* Copyright (C) 2008 - 2014 Xilinx, Inc.  All rights reserved.
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

#ifndef __PLATFORM_H_
#define __PLATFORM_H_

//#include "platform_config.h"
#include "xil_types.h"

#define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_BASEADDR	XPAR_PS7_UART_1_BASEADDR
#define UART_INTERRUPT	XPAR_XUARTPS_1_INTR

#define BIT(n) (1U<<n)
#define BITSET(reg,msk) (reg|=(msk))
#define BITCLR(reg,msk) (reg&=~(msk))

/********************************************************
 * Specific Driver for IMU application
 * Author : Arnaud Dion
 * 2017
 ********************************************************/


// ADXL362 Registers
#define ADXL362_DEVID_AD			0x00
#define ADXL362_DEVID_MST			0x01
#define ADXL362_PARTID				0x02
#define ADXL362_REVID				0x03
#define ADXL362_XDATA				0x08
#define ADXL362_YDATA				0x09
#define ADXL362_ZDATA				0x0A
#define ADXL362_STATUS				0x0B
#define ADXL362_FIFO_ENTRIES_L		0x0C
#define ADXL362_FIFO_ENTRIES_H		0x0D
#define ADXL362_XDATA_L				0x0E
#define ADXL362_XDATA_H				0x0F
#define ADXL362_YDATA_L				0x10
#define ADXL362_YDATA_H				0x11
#define ADXL362_ZDATA_L				0x12
#define ADXL362_ZDATA_H				0x13
#define ADXL362_TEMP_L				0x14
#define ADXL362_TEMP_H				0x15
#define ADXL362_SOFT_RESET			0x1F
#define ADXL362_THRESH_ACT_L		0x20
#define ADXL362_THRESH_ACT_H		0x21
#define ADXL362_TIME_ACT			0x22
#define ADXL362_THRESH_INACT_L		0x23
#define ADXL362_THRESH_INACT_H		0x24
#define ADXL362_TIME_INACT_H		0x25
#define ADXL362_TIM_INACT_L			0x26
#define ADXL362_ACT_INACT_CTL		0x27
#define ADXL362_FIFO_CONTROL		0x28
#define ADXL362_FIFO_SAMPLES		0x29
#define ADXL362_INTMAP1				0x2A
#define ADXL362_INTMAP2				0x2B
#define ADXL362_FILTER_CTL			0x2C
#define ADXL362_POWER_CTL			0x2D
#define ADXL362_SELF_TEST			0x2E

// Register specific bits
// Device ID Register 0x00 - Read value should be 0xAD

// Device ID Register 0x01 - Read value should be 0x1D

// Part ID Register 0x02 - Read value should be 0xF2

// Silicon Revision ID Register 0x03 - Read value should be 0x01

// X-AXIS Data Register 0x08 - Used to read 8 MSB in Power conscious applications

// Y-AXIS Data Register 0x09 - Used to read 8 MSB in Power conscious applications

// Z-AXIS Data Register 0x0A - Used to read 8 MSB in Power conscious applications

// Status Register 0x0B
#define ADXL362_ERR_USER_REGS		7
#define ADXL362_AWAKE				6
#define ADXL362_INACT				5
#define ADXL362_ACT					4
#define ADXL362_FIFO_OVERRUN		3
#define ADXL362_FIFO_WATERMARK		2
#define ADXL362_FIFO_READY			1
#define ADXL362_DATA_READY			0

// FIFO Entries L Register 0x0C - Contains number of FIFO Entries in FIFO Buffer (0 to 512)
// FIFO Entries H Register 0x0D [15:10] are unused

// X-AXIS Data Register L and H 0x0E and 0x0F

// Y-AXIS Data Register L and H 0x10 and 0x11

// Z-AXIS Data Register L and H 0x12 and 0x13

// Temperature Register L and H 0x14 and 0x15

// Software Reset Register
#define ADXL362_RESET_CMD			0x52

// Activity Threshold Register L and H 0x20 and 0x21

// Activity Timer Register 0x22

// Inactivity Threshold Register L and H 0x23 and 0x24

// Inactivity Timer Register L and H 0x25 and 0x26

// Activity / Inactivity Control Register 0x27
#define ADXL362_LINKLOOP			4
#define ADXL362_INACT_REF			3
#define ADXL362_INACT_EN			2
#define ADXL362_ACT_REF				1
#define ADXL362_ACT_EN				0

// FIFO Control Register 0x28
#define ADXL362_AH					3
#define ADXL362_FIFO_TEMP			2
#define ADXL362_FIFO_MODE			0

// FIFO Samples Register 0x29

// INT1/INT2 Function Map Registers 0x2A and 0x2B
#define ADXL362_INT_LOW				7
#define ADXL362_AWAKE				6
#define ADXL362_INACT				5
#define ADXL362_ACT					4
#define ADXL362_FIFO_OVERRUN		3
#define ADXL362_FIFO_WATERMARK		2
#define ADXL362_FIFO_READY			1
#define ADXL362_DATA_READY			0

// Filter Control Register 0x2C
#define ADXL362_RANGE				6
#define ADXL362_HALF_BW				4
#define ADXL362_EXT_SAMPLE			3
#define ADXL362_ODR					0

// Power Control Register 0x2D
#define ADXL362_EXT_CLK				6
#define ADXL362_LOW_NOISE			4
#define ADXL362_WAKEUP				3
#define ADXL362_AUTOSLEEP			2
#define ADXL362_MEASURE				0

// Self Test Register 0x2E
#define ADXL362_ST					1


#define SPI_BASEADDR 	XPAR_PS7_SPI_1_BASEADDR

/* ZYNQ Hardware SPI Registers */
#define HW_SPI_CONFIG_REG           0x00
#define HW_SPI_INTR_STS_REG         0x04
#define HW_SPI_INTR_EN_REG          0x08
#define HW_SPI_INTR_DIS_REG         0x0C
#define HW_SPI_INTR_MASK_REG        0x10
#define HW_SPI_EN_REG               0x14
#define HW_SPI_DELAY_REG            0x18
#define HW_SPI_TXDATA_REG           0x1C
#define HW_SPI_RXDATA_REG           0x20
#define HW_SPI_SLAVE_IDLE_COUNT_REG 0x24
#define HW_SPI_TX_THRES_REG         0x28
#define HW_SPI_RX_THRES_REG         0x2C
#define HW_SPI_MOD_ID_REG           0xFC

///* ZYNQ Hardware SPI Registers */
///* Config Register 0x00 */
//#define SPI_Modefail_gen_en         17
//#define SPI_Man_start_com           16
//#define SPI_Man_start_en            15
//#define SPI_Manual_CS               14
//#define SPI_CS_Bits                 10
//#define SPI_PERI_SEL                9
//#define SPI_REF_CLK                 8
//#define SPI_BAUD_RATE_DIV           3
//#define SPI_CLK_PH                  2
//#define SPI_CLK_POL                 1
//#define SPI_MODE_SEL                0
///* Interrupt Status Register 0x04 */
//#define SPI_TX_FIFO_underflow       6
//#define SPI_RX_FIFO_full            5
//#define SPI_RX_FIFO_not_empty       4
//#define SPI_TX_FIFO_full            3
//#define SPI_TX_FIFO_not_full        2
//#define SPI_MODE_FAIL               1
//#define SPI_RX_OVERFLOW             0
///* SPI Enable Register */
//#define SPI_EN                      0

#define SPI_Modefail_gen_en         BIT(17)
#define SPI_Man_start_com           BIT(16)
#define SPI_Man_start_en            BIT(15)
#define SPI_Manual_CS               BIT(14)
#define SPI_CS_Bits                 BIT(10)
#define SPI_PERI_SEL                BIT(9)
#define SPI_REF_CLK                 BIT(8)
#define SPI_BAUD_RATE_DIV           BIT(3)
#define SPI_CLK_PH                  BIT(2)
#define SPI_CLK_POL                 BIT(1)
#define SPI_MODE_SEL                BIT(0)
/* Interrupt Status Register 0x04 */
#define SPI_TX_FIFO_underflow       BIT(6)
#define SPI_RX_FIFO_full            BIT(5)
#define SPI_RX_FIFO_not_empty       BIT(4)
#define SPI_TX_FIFO_full            BIT(3)
#define SPI_TX_FIFO_not_full        BIT(2)
#define SPI_MODE_FAIL               BIT(1)
#define SPI_RX_OVERFLOW             BIT(0)
/* SPI Enable Register */
#define SPI_EN                      BIT(0)

#define ADXL_CS (SPI_CS_Bits*BIT(0))
#define JSTK_CS (SPI_CS_Bits*BIT(1))


void init_platform(void);
void cleanup_platform(void);
void init_uart(void);


void delay_ms(unsigned long ms_count);
void delay_us(unsigned long us_count);
void ADXL362_WriteReg(char addr, char data);
char ADXL362_ReadReg(char addr);
void ADXL362_Init(void);
void JSTK2_read(float* X, float* Y, int* B);
void SPI_ADXL_Init(void);

char ADXL362_IsDataReady(void);
float ADXL362_Convert(char);
void ADXL362_PrintID(void);
void enable_caches(void);
void disable_caches(void);

void SPI_JSTK_Init(void);
void JSTK_readval(float *rx);





#endif
