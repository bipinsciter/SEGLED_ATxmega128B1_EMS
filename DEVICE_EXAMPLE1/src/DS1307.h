#ifndef _DS1307_H
#define _DS1307_H

#include <util/delay.h>

#define SDA_HIGH		PORTC_OUTSET = BIT0
#define SDA_LOW			PORTC_OUTCLR = BIT0
#define SDA_SENSE		(PORTC_IN & BIT0)
#define SDA_DIR_IN		PORTC_DIRCLR = BIT0
#define SDA_DIR_OUT		PORTC_DIRSET = BIT0

#define SCL_HIGH		PORTC_OUTSET = BIT1
#define SCL_LOW			PORTC_OUTCLR = BIT1
#define SCL_DIR_IN		PORTC_DIRCLR = BIT1
#define SCL_DIR_OUT		PORTC_DIRSET = BIT1

#define SDA_DP2_HIGH		PORTM_OUTSET = BIT1
#define SDA_DP2_LOW			PORTM_OUTCLR = BIT1
#define SDA_DP2_SENSE		(PORTM_IN & BIT1)
#define SDA_DP2_DIR_IN		PORTM_DIRCLR = BIT1
#define SDA_DP2_DIR_OUT		PORTM_DIRSET = BIT1

#define SCL_DP2_HIGH		PORTM_OUTSET = BIT0
#define SCL_DP2_LOW			PORTM_OUTCLR = BIT0
#define SCL_DP2_DIR_IN		PORTM_DIRCLR = BIT0
#define SCL_DP2_DIR_OUT		PORTM_DIRSET = BIT0

#define EEPROM_ID		0xA0
#define RTC_ID			0xD0
#define TM1680_ID		0xE4//0xE6//0xE5//0xE7//0xE4


#define ACK		1
#define NO_ACK	0

void Init_DS1307(void);
void Write_DS1307(unsigned char addr,unsigned char *buff, unsigned char NoOfByte);
void Read_DS1307(unsigned char addr,unsigned char *buff, unsigned char NoOfByte);
unsigned char Read_byte_DS1307(unsigned char addr);
void Write_byte_DS1307(unsigned char addr,unsigned char msgbyte);
unsigned char BCD2HEX(unsigned char bcd);
unsigned char HEX2BCD(unsigned char hex);

//------------------ I2C ROUTINS for IDT1338, HTU25, DP1 -----------------------------------------------
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void Write_Byte_I2C(unsigned char Data);
unsigned char Read_Byte_I2C(unsigned char ACK_Bit);

//------------------ I2C ROUTINS for DP2 -----------------------------------------------
void I2C_DP2_Init(void);
void I2C_DP2_Start(void);
void I2C_DP2_Stop(void);
void Write_Byte_I2C_DP2(unsigned char Data);
unsigned char Read_Byte_I2C_DP2(unsigned char ACK_Bit);


unsigned char RTC_data[7]={0};


#endif