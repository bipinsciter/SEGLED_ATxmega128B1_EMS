#ifndef _SCPB_SENSOR_H
#define _SCPB_SENSOR_H

#include <util/delay.h>

#define COMMAND_MODE		0x72
#define START_MSRMT			0x01
#define START_AD_P_AZC		0xD8
#define START_AD_T1_AZC		0xD9

#define SDA_DP_HIGH			PORTB_OUTSET = BIT3
#define SDA_DP_LOW			PORTB_OUTCLR = BIT3
#define SDA_DP_SENSE		(PORTB_IN & BIT3)
#define SDA_DP_DIR_IN		PORTB_DIRCLR = BIT3
#define SDA_DP_DIR_OUT		PORTB_DIRSET = BIT3

#define SCL_DP_HIGH			PORTB_OUTSET = BIT2
#define SCL_DP_LOW			PORTB_OUTCLR = BIT2
#define SCL_DP_DIR_IN		PORTB_DIRCLR = BIT2
#define SCL_DP_DIR_OUT		PORTB_DIRSET = BIT2

unsigned short ReadDPressure(unsigned char);

//------------------------ I2C ROUTINS -----------------------------------------------
//void delay1(unsigned short);
void I2C_DP_Init(void);
void I2C_DP_Start(void);
void I2C_DP_Stop(void);
void Write_Byte_DP_I2C(unsigned char Data);
unsigned char Read_Byte_DP_I2C(unsigned char ACK_Bit);


extern unsigned char clkmode;

#endif