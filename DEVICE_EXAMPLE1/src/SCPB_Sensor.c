#include "SCPB_Sensor.h"

unsigned short ReadDPressure(unsigned char Index)
{
	unsigned short pval=0;
	
	//Start Command Mode ----------------------------------
	I2C_DP_Start();						// Start condition
	Write_Byte_DP_I2C(0xF0); 				// Write device address
	Write_Byte_DP_I2C(COMMAND_MODE);		// Write address of register
	I2C_DP_Stop();							// Send a STOP condition on the TWI bus.
	
	//Start Measurement cycle -----------------------------
	I2C_DP_Start();						// Start condition
	Write_Byte_DP_I2C(0xF0); 				// Write device address
	switch(Index)
	{
		case 0:		Write_Byte_DP_I2C(START_MSRMT);		break;
		case 1:		Write_Byte_DP_I2C(START_AD_P_AZC);		break;
		case 2:		Write_Byte_DP_I2C(START_AD_T1_AZC);		break;
	}
	//Write_Byte_DP_I2C(START_AD_P_AZC);		// Write address of register
	I2C_DP_Stop();							// Send a STOP condition on the TWI bus.
	
	//Read Pressure Count ---------------------------------
	I2C_DP_Start();						// Start condition
	Write_Byte_DP_I2C(0xF1);				// Write device address
	//NoOfByte=2;
	
	SDA_DP_DIR_IN;
	
	pval = Read_Byte_DP_I2C(ACK);               // Load ACK. Set data register bit 7 (output for SDA) low.
	pval<<=8;
	pval |= (unsigned short)(Read_Byte_DP_I2C(NO_ACK));              // Load NACK to confirm End Of Transmission.
	
	SDA_DP_DIR_OUT;
	
	I2C_DP_Stop();              // Send a STOP condition on the TWI bus.
	
	return pval;
}

//----------------------------------------------------------------------------------------
// I2C FUNCTIONS
//----------------------------------------------------------------------------------------
void I2C_DP_Init(void)
{
	SCL_DP_DIR_OUT;           					// Enable SCL as output.
	SDA_DP_DIR_OUT;           					// Enable SDA as output.
	
	SDA_DP_HIGH;           					// Enable pullup on SDA, to set high as released state.
	SCL_DP_HIGH;						        // Enable pullup on SCL, to set high as released state.
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
---------------------------------------------------------------*/
unsigned char Read_Byte_DP_I2C(unsigned char ACK_Bit)
{
	unsigned char Data=0,i=0;

    SDA_DP_DIR_IN;	
	
	for (i=0;i<8;i++)
	{
		SCL_DP_HIGH;		
		Data<<= 1;
		
		if(SDA_DP_SENSE) Data  |= 0x01;
		
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
				
		SCL_DP_LOW;
		
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
	}
    
	SDA_DP_DIR_OUT;
	
 	if (ACK_Bit == 1)
		SDA_DP_LOW;  // Send ACK		
	else		
		SDA_DP_HIGH; // Send NO ACK				

	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_DP_HIGH;		
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_DP_LOW;
	
	return Data;
}

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition. 
---------------------------------------------------------------*/
void I2C_DP_Start(void)
{
	SDA_DP_HIGH;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_DP_HIGH;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SDA_DP_LOW;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_DP_LOW;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
}

/*---------------------------------------------------------------
 Function for writing byte
---------------------------------------------------------------*/
void Write_Byte_DP_I2C(unsigned char datum)
{
	unsigned char i=0;
	
	SCL_DP_LOW;                // Pull SCL LOW.
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	
	for (i=0;i<8;i++)
	{
		if(datum & 0x80) SDA_DP_HIGH;
		else 			 SDA_DP_LOW;
		
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		
		SCL_DP_HIGH;
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		SCL_DP_LOW;
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		
		datum<<=1;
	}

	SDA_DP_DIR_IN;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	
  	SCL_DP_HIGH; 
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_DP_LOW;
	
	SDA_DP_DIR_OUT;
}


/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
void I2C_DP_Stop(void)
{
	SDA_DP_LOW;	    	
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_DP_HIGH;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SDA_DP_HIGH;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_DP_LOW;
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
}


