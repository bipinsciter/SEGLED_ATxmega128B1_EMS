#ifndef SPI_H_
#define SPI_H_

#include <util/delay.h>

#define TRUE		0x01
#define FALSE		0x00

#define SCLK_DIR_IN		PORTC_DIRCLR = BIT7
#define SCLK_DIR_OUT	PORTC_DIRSET = BIT7
#define SDI_DIR_IN		PORTC_DIRCLR = BIT5
#define SDI_DIR_OUT		PORTC_DIRSET = BIT5
#define CS_DIR_IN		PORTC_DIRCLR = BIT4
#define CS_DIR_OUT		PORTC_DIRSET = BIT4
#define SDO_DIR_IN		PORTC_DIRCLR = BIT6
#define SDO_DIR_OUT		PORTC_DIRSET = BIT6

#define SCLK_HIGH		PORTC_OUTSET = BIT7
#define SCLK_LOW		PORTC_OUTCLR = BIT7
#define SDI_HIGH		PORTC_OUTSET = BIT5
#define SDI_LOW			PORTC_OUTCLR = BIT5
#define CS_HIGH			PORTC_OUTSET = BIT4
#define CS_LOW			PORTC_OUTCLR = BIT4
#define SDO_HIGH		PORTC_OUTSET = BIT6
#define SDO_LOW			PORTC_OUTCLR = BIT6

#define SENSE_SDO		(PORTC_IN & BIT6)

void Init_SPI(void);
void SPI_WriteByte(unsigned char datum);
unsigned char SPI_ReadByte(void);

#endif /*SPI_H_*/