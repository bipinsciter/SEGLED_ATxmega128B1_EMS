#include "DS1307.h"
#include "SCPB_Sensor.h"

unsigned short ReadDPressure(unsigned char Index)
{
	unsigned short pval=0;
	
	//Start Command Mode ----------------------------------
	I2C_Start();						// Start condition
	Write_Byte_I2C(0xF0); 				// Write device address
	Write_Byte_I2C(COMMAND_MODE);		// Write address of register
	I2C_Stop();							// Send a STOP condition on the TWI bus.
	
	//Start Measurement cycle -----------------------------
	I2C_Start();						// Start condition
	Write_Byte_I2C(0xF0); 				// Write device address
	switch(Index)
	{
		case 0:		Write_Byte_I2C(START_MSRMT);		break;
		case 1:		Write_Byte_I2C(START_AD_P_AZC);		break;
		case 2:		Write_Byte_I2C(START_AD_T1_AZC);		break;
	}
	//Write_Byte_DP_I2C(START_AD_P_AZC);		// Write address of register
	I2C_Stop();							// Send a STOP condition on the TWI bus.
	
	//Read Pressure Count ---------------------------------
	I2C_Start();						// Start condition
	Write_Byte_I2C(0xF1);				// Write device address
	
	SDA_DIR_IN;
	
	pval = Read_Byte_I2C(ACK);               // Load ACK. Set data register bit 7 (output for SDA) low.
	pval<<=8;
	pval |= (unsigned short)(Read_Byte_I2C(NO_ACK));              // Load NACK to confirm End Of Transmission.
	
	SDA_DIR_OUT;
	
	I2C_Stop();              // Send a STOP condition on the TWI bus.
	
	return pval;
}

/*unsigned short Read_SM9541(void)
{
	#define SM9541_ADDR	0x28
	
	unsigned short pval=0,tval=0;
	unsigned char status=0;
	
	//Start Command Mode ----------------------------------
	I2C_Start();						// Start condition
	Write_Byte_I2C(SM9541_ADDR+1); 				// Write device address to start measurement	
	I2C_Stop();							// Send a STOP condition on the TWI bus.
	
	//Start Measurement cycle -----------------------------
	I2C_Start();						// Start condition
	Write_Byte_I2C(SM9541_ADDR+1); 							// Write device address
	
	SDA_DIR_IN;
	
	pval = Read_Byte_I2C(ACK);						// PressureByte MSB
	pval<<=8;
	pval |= (unsigned short)(Read_Byte_I2C(ACK));   // PressureByte LSB
	
	tval = Read_Byte_I2C(ACK);						// TemperatureByte MSB
	tval<<=8;
	tval |= (unsigned short)(Read_Byte_I2C(ACK));   // TemperatureByte LSB
	
	SDA_DIR_OUT;
	
	I2C_Stop();										// Send a STOP condition on the TWI bus.
	
	status = (unsigned char)(pval >> 14);
	pval &= 0x3FFF;
	pval >>= 5; 
}
*/

/*unsigned short Read_SM5852(void)
{
	#define SM5852_ADDR			0x5F
	#define CORR_PRESSURE_LSB	0x80
	#define CORR_PRESSURE_MSB	0x81
	#define TEMP_LSB			0x82
	#define TEMP_MSB			0x83
	#define UNCORR_PRESSURE_LSB	0x84
	#define UNCORR_PRESSURE_MSB	0x85
	#define BGV_LSB				0x86
	#define BGV_MSB				0x87
	
	unsigned short cpval=0,tval=0,ucpval=0,bgval=0;
		
	//Read SM5852 ----------------------------------
	I2C_Start();						// Start condition
	Write_Byte_I2C(SM5852_ADDR);		// Write device address
	Write_Byte_I2C(CORR_PRESSURE_LSB);	// Write Register address
	I2C_Start();						// Start condition
	
	SDA_DIR_IN;
	
	cpval = Read_Byte_I2C(ACK);			// Corrected PressureByte LSB
	cpval |= Read_Byte_I2C(ACK)<<8;		// Corrected PressureByte MSB
	tval = Read_Byte_I2C(ACK);			// Temperature Byte LSB
	tval |= Read_Byte_I2C(ACK)<<8;		// Temperature Byte MSB
	ucpval = Read_Byte_I2C(ACK);		// UnCorrected PressureByte LSB
	ucpval |= Read_Byte_I2C(ACK)<<8;	// UnCorrected PressureByte MSB
	bgval = Read_Byte_I2C(ACK);			// Band Gap Voltage Byte LSB
	bgval |= Read_Byte_I2C(ACK)<<8;		// Band Gap Voltage Byte MSB
	
	SDA_DIR_OUT;
	
	I2C_Stop();							// Send a STOP condition on the TWI bus.
}
*/

//-------------------------------
// Initialize DS1307
//-------------------------------
void Init_DS1307(void)
{
	RTC_data[0] = Read_byte_DS1307(0x00);        
	RTC_data[0] &= ~BIT7;
	Write_byte_DS1307(0x00,RTC_data[0]);
	
	
	/*
	RTC_data[0] = Read_byte_DS1307(0x00);        
	RTC_data[0] &= ~BIT7;
	RTC_data[1] = 0x09;	// minute = 59
	RTC_data[2] = 0x01;	// hour = 05 ,24-hour mode(bit 6=0)
	RTC_data[3] = 0x06;	// day = 1 or sunday
	RTC_data[4] = 0x20;	// date = 30
	RTC_data[5] = 0x07;	// month = december
	RTC_data[6] = 0x13;	// year = 13 or 2013
	Write_DS1307(0x00,&RTC_data[0],7);	// set RTC
	*/
	
	/*RTC_data[1] = 0x54;	// minute = 59
	RTC_data[2] = 0x12 | BIT6;	// hour = 05 ,24-hour mode(bit 6=0)
	RTC_data[3] = 0x06;	// day = 1 or sunday
	RTC_data[4] = 0x20;	// date = 30
	RTC_data[5] = 0x07;	// month = december
	RTC_data[6] = 0x13;	// year = 13 or 2013
	Write_DS1307(0x01,&RTC_data[1],6);	// set rTC
	*/

	//Set AM/PM 12 Hour Mode
	//RTC_data[2] = Read_byte_DS1307(0x02);        
	//RTC_data[2] |= BIT6;
	//Write_byte_DS1307(0x02,RTC_data[2]);
}

//-------------------------------
// Read 1 byte from I2C
//-------------------------------
unsigned char Read_byte_DS1307(unsigned char addr)
{
   	unsigned char Data;
	
	I2C_Start();				// Start condition
	
	Write_Byte_I2C(RTC_ID);			// Write device address
	
	Write_Byte_I2C(addr);			// Write address of register
	
	I2C_Start();				// Start condition
	
	Write_Byte_I2C(RTC_ID+1);			// Write device address
	
	SDA_DIR_IN;   
	
	Data = Read_Byte_I2C(NO_ACK);
	
	SDA_DIR_OUT; 
	
   	I2C_Stop();              // Send a STOP condition on the TWI bus.		
	
	return Data;
}

//-------------------------------
// Write 1 byte to RTC
//-------------------------------

void Write_byte_DS1307(unsigned char addr,unsigned char msgbyte)
{
	I2C_Start();				// Start condition
	
	Write_Byte_I2C(RTC_ID);			// Write device address
	
	Write_Byte_I2C(addr);			// Write address of register
	
	Write_Byte_I2C(msgbyte);		// DATA
	
	I2C_Stop();              // Send a STOP condition on the TWI bus.
}


//-------------------------------
// Read RTC (all real time)
//-------------------------------
void Read_DS1307(unsigned char addr,unsigned char *buff, unsigned char NoOfByte)
{
	I2C_Start();				// Start condition
	
	Write_Byte_I2C(RTC_ID);			// Write device address
	
	Write_Byte_I2C(addr);			// Write address of register
	
	I2C_Start();				// Start condition
	
	Write_Byte_I2C(RTC_ID+1);			// Write device address
	
	SDA_DIR_IN;   
	
	while(NoOfByte)
	{
		NoOfByte--;
		
		/* Prepare to generate ACK (or NACK in case of End Of Transmission) */
		if(NoOfByte)
		{
			*buff = Read_Byte_I2C(ACK);
		}
		else
		{
			*buff = Read_Byte_I2C(NO_ACK);
		}
		buff++;
   	}
	SDA_DIR_OUT;  
	
   	I2C_Stop();              // Send a STOP condition on the TWI bus.	
}

//-------------------------------
// Write RTC
//-------------------------------
void Write_DS1307(unsigned char addr,unsigned char *buff, unsigned char NoOfByte)
{
	I2C_Start();				// Start condition
	
	Write_Byte_I2C(RTC_ID);			// Write device address
	
	Write_Byte_I2C(addr);			// Write address of register
	
	while(NoOfByte)
	{	
		Write_Byte_I2C(*buff);				// DATA
	 	buff++;							// Point to Next Location	 
		NoOfByte--;
	}
	
	I2C_Stop();              // Send a STOP condition on the TWI bus.
}

//-------------------------------
// Convert BCD 1 byte to HEX 1 byte
//-------------------------------
unsigned char BCD2HEX(unsigned char bcd)
{
	unsigned char temp=0;
	//temp=((bcd>>4)*10)|(bcd & 0x0f);
	temp=((bcd>>4)*10);
	temp+=(bcd & 0x0F);
	
	return temp;
}

//-------------------------------
// Convert HEX 1 byte to BCD 1 byte
//-------------------------------
unsigned char HEX2BCD(unsigned char hex)
{
	unsigned char temp=0;
	//temp = ((hex / 10)<<4) | (hex % 10);
	temp = ((hex / 10)<<4);
	temp |= (hex % 10);
	return temp;
}

//----------------------------------------------------------------------------------------
// I2C FUNCTIONS
//----------------------------------------------------------------------------------------
void I2C_Init(void)
{
	SCL_DIR_OUT;           					// Enable SCL as output.
	SDA_DIR_OUT;           					// Enable SDA as output.
	
	SDA_HIGH;           					// Enable pullup on SDA, to set high as released state.
	SCL_HIGH;						        // Enable pullup on SCL, to set high as released state.
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
---------------------------------------------------------------*/
unsigned char Read_Byte_I2C(unsigned char ACK_Bit)
{
	unsigned char Data=0,i=0;

    SDA_DIR_IN;	
	
	for (i=0;i<8;i++)
	{
		SCL_HIGH;		
		Data<<= 1;
		
		if(SDA_SENSE) Data  |= 0x01;
		
		_delay_us(5);
		SCL_LOW;
		_delay_us(5);
	}
    
	SDA_DIR_OUT;
	
 	if (ACK_Bit == 1)
		SDA_LOW;  // Send ACK		
	else		
		SDA_HIGH; // Send NO ACK				

	_delay_us(5);
	SCL_HIGH;		
	_delay_us(5);
	SCL_LOW;
	
	return Data;
}

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition. 
---------------------------------------------------------------*/
void I2C_Start(void)
{
	SDA_HIGH;
	_delay_us(5);
	SCL_HIGH;
	_delay_us(5);
	SDA_LOW;
	_delay_us(5);
	SCL_LOW;
	_delay_us(5);
}

/*---------------------------------------------------------------
 Function for writing byte
---------------------------------------------------------------*/
void Write_Byte_I2C(unsigned char datum)
{
	unsigned char i=0;
	
	SCL_LOW;                // Pull SCL LOW.
	
	for (i=0;i<8;i++)
	{
		if(datum & 0x80) SDA_HIGH;
		else 			 SDA_LOW;
		
		_delay_us(5);
		
		SCL_HIGH;
		_delay_us(5);
		SCL_LOW;
		_delay_us(5);
		
		datum<<=1;
	}

	SDA_DIR_IN;
	
  	SCL_HIGH; 
	_delay_us(5);
	SCL_LOW;
	
	SDA_DIR_OUT;
}


/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
void I2C_Stop(void)
{
	SDA_LOW;	    	
	_delay_us(5);
	SCL_HIGH;
	_delay_us(5);
	SDA_HIGH;
	_delay_us(5);
	//SCL_LOW;
	//_delay_us(5);
}