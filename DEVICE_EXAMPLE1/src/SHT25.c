#include "SHT25.h"
#include "DS1307.h"
#include "SCPB_Sensor.h"

//==============================================================================
unsigned char SHT2x_CheckCrc(unsigned char data[], unsigned char nbrOfBytes, unsigned char checksum)
//==============================================================================
{
	unsigned char crc = 0;	
	unsigned char byteCtr;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{ 
		crc ^= (data[byteCtr]);
		
		for (unsigned char bit = 8; bit > 0; --bit)
		{ 
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	
	if (crc != checksum) return CHECKSUM_ERROR;
	else return 0;
}

//===========================================================================
unsigned char SHT2x_ReadUserRegister(unsigned char *pRegisterValue)
//===========================================================================
{
	unsigned char checksum;   //variable for checksum byte
	unsigned char error=0;    //variable for error code

	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_WRITE);
	error |= SHT25_I2CWriteByte (USER_REG_R);
	
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_READ);
	*pRegisterValue = SHT25_I2CReadByte(ACK);
	checksum=SHT25_I2CReadByte(NO_ACK);
	
	error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
	I2C_Stop();//SHT25_I2CStopCondition();
	
	return error;
}

//===========================================================================
unsigned char SHT2x_WriteUserRegister(unsigned char *pRegisterValue)
//===========================================================================
{
	unsigned char error=0;   //variable for error code

	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_WRITE);
	error |= SHT25_I2CWriteByte (USER_REG_W);
	error |= SHT25_I2CWriteByte (*pRegisterValue);
	I2C_Stop();//SHT25_I2CStopCondition();
	
	return error;
}

//===========================================================================
unsigned char SHT2x_MeasureHM(unsigned char eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
	unsigned char  checksum;   //checksum
	unsigned char  data[2];    //data array for checksum verification
	unsigned char  error=0;    //error variable
	unsigned short i;          //counting variable

	//-- write I2C sensor address and command --
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_WRITE); // I2C Adr
	switch(eSHT2xMeasureType)
	{ 
		case HUMIDITY: error |= SHT25_I2CWriteByte (TRIG_RH_MEASUREMENT_HM); break;
		case TEMP    : error |= SHT25_I2CWriteByte (TRIG_T_MEASUREMENT_HM);  break;
		//default: assert(0);
	}
	
	//-- wait until hold master is released --
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_READ);
	SCL_SHT25_DIR_IN;                     // set SCL I/O port as input
	for(i=0; i<1000; i++)         // wait until master hold is released or
	{ 
		//_delay_us(1000);    // a timeout (~1s) is reached
		if(!clkmode) _delay_us(200);		else	_delay_us(1000);
		
		if (SCL_SHT25_SENSE==1) break;
	}
	
	//-- check for timeout --
	if(SCL_SHT25_SENSE==0) error |= TIME_OUT_ERROR;
	
	SCL_SHT25_DIR_OUT;                     // set SCL I/O port as output

	//-- read two data bytes and one checksum byte --
	pMeasurand->s16.u8H = data[0] = SHT25_I2CReadByte(ACK);
	pMeasurand->s16.u8L = data[1] = SHT25_I2CReadByte(ACK);
	checksum=SHT25_I2CReadByte(NO_ACK);

	//-- verify checksum --
	error |= SHT2x_CheckCrc (data,2,checksum);
	I2C_Stop();//SHT25_I2CStopCondition();
	return error;
}

//===========================================================================
unsigned char SHT2x_MeasurePoll(unsigned char eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
	unsigned char  checksum;   //checksum
	unsigned char  data[2];    //data array for checksum verification
	unsigned char  error=0;    //error variable
	unsigned short i=0;        //counting variable

	//-- write I2C sensor address and command --
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_WRITE); // I2C Adr
	switch(eSHT2xMeasureType)
	{ 
		case HUMIDITY: error |= SHT25_I2CWriteByte (TRIG_RH_MEASUREMENT_POLL); break;
		case TEMP    : error |= SHT25_I2CWriteByte (TRIG_T_MEASUREMENT_POLL);  break;
		//default: assert(0);
	}
	//-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
	do
	{ 
		I2C_Start();//SHT25_I2CStartCondition();
		//_delay_ms(5);  //delay 10ms
		if(!clkmode) _delay_ms(2);		else	_delay_ms(10);
		
		if(i++ >= 40) break;
	} while(SHT25_I2CWriteByte (SHT25_I2C_READ) == ACK_ERROR);
	
	if (i>=40) error |= TIME_OUT_ERROR;

	//-- read two data bytes and one checksum byte --
	pMeasurand->s16.u8H = data[0] = SHT25_I2CReadByte(ACK);
	pMeasurand->s16.u8L = data[1] = SHT25_I2CReadByte(ACK);
	checksum=SHT25_I2CReadByte(NO_ACK);

	//-- verify checksum --
	error |= SHT2x_CheckCrc (data,2,checksum);
	I2C_Stop();//SHT25_I2CStopCondition();

	return error;
}

//===========================================================================
unsigned char SHT2x_SoftReset(void)
//===========================================================================
{
	unsigned char  error=0;           //error variable

	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_WRITE); // I2C Adr
	error |= SHT25_I2CWriteByte (SOFT_RESET);                            // Command
	I2C_Stop();//SHT25_I2CStopCondition();

	if(!clkmode) _delay_ms(3);		else	_delay_ms(15);	// wait till sensor has restarted
	
	return error;
}

//==============================================================================
float SHT2x_CalcRH(unsigned short u16sRH)
//==============================================================================
{
	float humidityRH;              // variable for result

	u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
	//-- calculate relative humidity [%RH] --

	humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
	return humidityRH;
}

//==============================================================================
float SHT2x_CalcTemperatureC(unsigned short u16sT)
//==============================================================================
{
	float temperatureC;            // variable for result

	u16sT &= ~0x0003;           // clear bits [1..0] (status bits)

	//-- calculate temperature [°C] --
	temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
	return temperatureC;
}

//==============================================================================
unsigned char SHT2x_GetSerialNumber(unsigned char u8SerialNumber[])
//==============================================================================
{
	unsigned char  error=0;                          //error variable

	//Read from memory location 1
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_WRITE);    //I2C address
	error |= SHT25_I2CWriteByte (0xFA);         //Command for readout on-chip memory
	error |= SHT25_I2CWriteByte (0x0F);         //on-chip memory address
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_READ);    //I2C address
	u8SerialNumber[5] = SHT25_I2CReadByte(ACK); //Read SNB_3
	SHT25_I2CReadByte(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
	u8SerialNumber[4] = SHT25_I2CReadByte(ACK); //Read SNB_2
	SHT25_I2CReadByte(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
	u8SerialNumber[3] = SHT25_I2CReadByte(ACK); //Read SNB_1
	SHT25_I2CReadByte(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
	u8SerialNumber[2] = SHT25_I2CReadByte(ACK); //Read SNB_0
	SHT25_I2CReadByte(NO_ACK);                  //Read CRC SNB_0 (CRC is not analyzed)
	I2C_Stop();//SHT25_I2CStopCondition();

	//Read from memory location 2
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_WRITE);    //I2C address
	error |= SHT25_I2CWriteByte (0xFC);         //Command for readout on-chip memory
	error |= SHT25_I2CWriteByte (0xC9);         //on-chip memory address
	I2C_Start();//SHT25_I2CStartCondition();
	error |= SHT25_I2CWriteByte (SHT25_I2C_READ);    //I2C address
	u8SerialNumber[1] = SHT25_I2CReadByte(ACK); //Read SNC_1
	u8SerialNumber[0] = SHT25_I2CReadByte(ACK); //Read SNC_0
	SHT25_I2CReadByte(ACK);                     //Read CRC SNC0/1 (CRC is not analyzed)
	u8SerialNumber[7] = SHT25_I2CReadByte(ACK); //Read SNA_1
	u8SerialNumber[6] = SHT25_I2CReadByte(ACK); //Read SNA_0
	SHT25_I2CReadByte(NO_ACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
	I2C_Stop();//SHT25_I2CStopCondition();

	return error;
}

//----------------------------------------------------------------------------------------
// I2C FUNCTIONS
//----------------------------------------------------------------------------------------

void SHT25_I2C_Init(void)
{
	SCL_SHT25_DIR_OUT;           					// Enable SCL as output.
	SDA_SHT25_DIR_OUT;           					// Enable SDA as output.
	
	SDA_SHT25_HIGH;           					// Enable pullup on SDA, to set high as released state.
	SCL_SHT25_HIGH;						        // Enable pullup on SCL, to set high as released state.
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
---------------------------------------------------------------*/
unsigned char SHT25_I2CReadByte(unsigned char ACK_Bit)
{
	unsigned char Data=0,i=0;

    SDA_SHT25_DIR_IN;	
	SDA_SHT25_HIGH;
	
	for (i=0;i<8;i++)
	{
		SCL_SHT25_HIGH;		
		Data<<= 1;
		
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		
		if(SDA_SHT25_SENSE) Data  |= 0x01;
		
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		SCL_SHT25_LOW;
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
	}
    
	SDA_SHT25_DIR_OUT;
	if(!clkmode) _delay_us(1);		else	_delay_us(2);
	
 	if (ACK_Bit == ACK)
		SDA_SHT25_LOW;  // Send ACK		
	else		
		SDA_SHT25_HIGH; // Send NO ACK				

	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_SHT25_HIGH;		
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	SCL_SHT25_LOW;
	
	return Data;
}

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition. 
---------------------------------------------------------------*/
//void SHT25_I2CStartCondition(void)
//{
	//SDA_SHT25_HIGH;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
	//SCL_SHT25_HIGH;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
	//SDA_SHT25_LOW;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
	//SCL_SHT25_LOW;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
//}
//
/*---------------------------------------------------------------
 Function for writing byte
---------------------------------------------------------------*/
unsigned char SHT25_I2CWriteByte(unsigned char datum)
{
	unsigned char i=0,error=0;
	
	//SCL_SHT25_LOW;                // Pull SCL LOW.
	
	for (i=0;i<8;i++)
	{
		if(datum & 0x80) SDA_SHT25_HIGH;
		else 			 SDA_SHT25_LOW;
		
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		
		SCL_SHT25_HIGH;
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		SCL_SHT25_LOW;
		if(!clkmode) _delay_us(1);		else	_delay_us(5);
		
		datum<<=1;
	}

	SDA_SHT25_DIR_IN;
	
  	SCL_SHT25_HIGH; 
	if(!clkmode) _delay_us(1);		else	_delay_us(5);
	if(SDA_SHT25_SENSE) error=ACK_ERROR; //check ack from i2c slave
	SCL_SHT25_LOW;
	
	SDA_SHT25_DIR_OUT;

	if(!clkmode) _delay_us(1);		else	_delay_us(20);	//wait time to see byte package on scope
	
	return error;                       //return error code
}


/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
//void SHT25_I2CStopCondition(void)
//{
	//SDA_SHT25_LOW;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
	//SCL_SHT25_HIGH;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
	//SDA_SHT25_HIGH;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
	//SCL_SHT25_LOW;
	//if(!clkmode) _delay_us(1);		else	_delay_us(5);
//}



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
	//NoOfByte=2;
	
	SDA_DIR_IN;
	
	pval = Read_Byte_I2C(ACK);               // Load ACK. Set data register bit 7 (output for SDA) low.
	pval<<=8;
	pval |= (unsigned short)(Read_Byte_I2C(NO_ACK));              // Load NACK to confirm End Of Transmission.
	
	SDA_DIR_OUT;
	
	I2C_Stop();              // Send a STOP condition on the TWI bus.
	
	return pval;
}

//-------------------------------
// Initialize DS1307
//-------------------------------
void Init_DS1307(void)
{
	RTC_data[0] = Read_byte_DS1307(0x00);        
	RTC_data[0] &= ~BIT7;
	Write_byte_DS1307(0x00,RTC_data[0]);
	Write_byte_DS1307(0x07,0x00);
	
	/*RTC_data[0] = Read_byte_DS1307(0x00);        
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
	SCL_LOW;
	_delay_us(5);
}



//----------------------------------------------------------------------------------------
// DP2 I2C FUNCTIONS
//----------------------------------------------------------------------------------------
void I2C_DP2_Init(void)
{
	SCL_DP2_DIR_OUT;           					// Enable SCL as output.
	SDA_DP2_DIR_OUT;           					// Enable SDA as output.
	
	SDA_DP2_HIGH;           					// Enable pullup on SDA, to set high as released state.
	SCL_DP2_HIGH;						        // Enable pullup on SCL, to set high as released state.
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
---------------------------------------------------------------*/
unsigned char Read_Byte_I2C_DP2(unsigned char ACK_Bit)
{
	unsigned char Data=0,i=0;

    SDA_DP2_DIR_IN;	
	
	for (i=0;i<8;i++)
	{
		SCL_DP2_HIGH;		
		Data<<= 1;
		
		if(SDA_DP2_SENSE) Data  |= 0x01;
		
		_delay_us(5);
		SCL_DP2_LOW;
		_delay_us(5);
	}
    
	SDA_DP2_DIR_OUT;
	
 	if (ACK_Bit == 1)
		SDA_DP2_LOW;  // Send ACK		
	else		
		SDA_DP2_HIGH; // Send NO ACK				

	_delay_us(5);
	SCL_DP2_HIGH;		
	_delay_us(5);
	SCL_DP2_LOW;
	
	return Data;
}

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition. 
---------------------------------------------------------------*/
void I2C_DP2_Start(void)
{
	SDA_DP2_HIGH;
	_delay_us(5);
	SCL_DP2_HIGH;
	_delay_us(5);
	SDA_DP2_LOW;
	_delay_us(5);
	SCL_DP2_LOW;
	_delay_us(5);
}

/*---------------------------------------------------------------
 Function for writing byte
---------------------------------------------------------------*/
void Write_Byte_I2C_DP2(unsigned char datum)
{
	unsigned char i=0;
	
	SCL_DP2_LOW;                // Pull SCL LOW.
	
	for (i=0;i<8;i++)
	{
		if(datum & 0x80) SDA_DP2_HIGH;
		else 			 SDA_DP2_LOW;
		
		_delay_us(5);
		
		SCL_DP2_HIGH;
		_delay_us(5);
		SCL_DP2_LOW;
		_delay_us(5);
		
		datum<<=1;
	}

	SDA_DP2_DIR_IN;
	
  	SCL_DP2_HIGH; 
	_delay_us(5);
	SCL_DP2_LOW;
	
	SDA_DP2_DIR_OUT;
}


/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
void I2C_DP2_Stop(void)
{
	SDA_DP2_LOW;	    	
	_delay_us(5);
	SCL_DP2_HIGH;
	_delay_us(5);
	SDA_DP2_HIGH;
	_delay_us(5);
	SCL_DP2_LOW;
	_delay_us(5);
}

