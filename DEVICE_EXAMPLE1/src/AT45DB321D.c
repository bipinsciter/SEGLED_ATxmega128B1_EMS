/***********************************************************************************************\
 * AUTHOR: 		Bipin Patel			 															*
 * Date: 		30/08/2013																		*
 * File Name:	SPI.c																			*  		
 ***********************************************************************************************/
#include "AT45DB321D.h"


void AT45D_Init(void)
{
	FRESET_DIR_OUT;
	FRESET_HIGH;
	_delay_ms(1);
	FRESET_LOW;
	_delay_ms(5);
	FRESET_HIGH;
}

void AT45D_ChipErase(void)
{
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());
	
	
	// Select DataFlash
	CS_LOW;

	// Send command
	SPI_WriteByte(0xC7);
	SPI_WriteByte(0x94);
	SPI_WriteByte(0x80);
	SPI_WriteByte(0x9A);
	
	// Deselect DataFlash
	CS_HIGH;
	
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());
}

void AT45D_SectorErase(unsigned char Sector)
{
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());
	
	// Select DataFlash
	CS_LOW;

	// Send command
	SPI_WriteByte(0x7C);
	SPI_WriteByte(Sector);
	SPI_WriteByte(0x00);
	SPI_WriteByte(0x00);
	
	// Deselect DataFlash
	CS_HIGH;
	
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());
}

void AT45D_BlockErase(unsigned short Block)
{
	// Select DataFlash
	CS_LOW;

	Block <<= 4;
	
	// Send command
	SPI_WriteByte(0x50);
	SPI_WriteByte(Block>>8);
	SPI_WriteByte(Block & 0x00FF);
	SPI_WriteByte(0x00);
	
	// Deselect DataFlash
	CS_HIGH;
	
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());
}

void AT45D_PageErase(unsigned short Page)
{
	// Select DataFlash
	CS_LOW;

	Page <<= 1;
	
	// Send command
	SPI_WriteByte(0x81);
	SPI_WriteByte(Page>>8);
	SPI_WriteByte(Page & 0x00FF);
	SPI_WriteByte(0x00);
	
	// Deselect DataFlash
	CS_HIGH;
	
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());
}

void WriteLog(unsigned long AddrOffset,unsigned long LogInd,unsigned char *buffer,unsigned short bytes)
{
	unsigned short Page=0,StartAddress=0,RemainSpace=0;
	unsigned long Address=0;
	unsigned char nextcheck=0;
	
	Address = LogInd * bytes;
	Address += AddrOffset;
	
	//AT45D_ResumeFromPowerDown();
	
	while(!nextcheck)
	{
		Page = (unsigned short)(Address >> 9);
		StartAddress = (unsigned short)(Address & 0x000001FF);

		//Copy Selected Page to Buffer1 ----------------------------
		
		dA.dummyAddress = Address & 0xFFFFFE00;
		
		while(!AT45D_Ready());

		CS_LOW;		// Select DataFlash
		
		// Send command
		SPI_WriteByte(AT45D_CMD_MAIN_MEM_PAGE_TO_BUF1);

		// Send start byte in buffer
		SPI_WriteByte(dA.dAddr[2]);
		SPI_WriteByte(dA.dAddr[1]);
		SPI_WriteByte(dA.dAddr[0]);
		
		CS_HIGH;		// Deselect DataFlash
		
		//Write Data to Buffer1 -------------------------------------

		//dA.dummyAddress = 0;
		dA.dummyAddress = (unsigned long)StartAddress;
		
		// Set flag to busy
		at45d_ready_flag = FALSE;
		while(!AT45D_Ready());
		
		CS_LOW;		// Select DataFlash

		// Send command
		SPI_WriteByte(AT45D_CMD_BUFFER1_WRITE);

		// Send start byte in buffer
		SPI_WriteByte(dA.dAddr[2]);
		SPI_WriteByte(dA.dAddr[1]);
		SPI_WriteByte(dA.dAddr[0]);
		
		// See if "number of bytes to read" should be clipped
		RemainSpace = 512 - StartAddress;
		if(bytes > RemainSpace)
		{
			//RemainBytes = bytes-RemainSpace;
			while(RemainSpace)
			{
				SPI_WriteByte(*buffer);
				buffer++;
				RemainSpace--;
				bytes--;
				Address++;
			}
			nextcheck=0;
		}
		else
		{
			while(bytes)
			{
				SPI_WriteByte(*buffer);
				buffer++;
				bytes--;
				Address++;
			}
			
			nextcheck=1;
		}
		
		CS_HIGH;		// Deselect DataFlash
		
		//Copy Buffer1 to Selected Page -----------------------------
		dA.dummyAddress = (unsigned long)Page<<9;
		//dA.dummyAddress <<= 9;
		
		//dA.dummyAddress &= 0xFFFFFE00;
		
		// Set flag to busy
		at45d_ready_flag = FALSE;
		while(!AT45D_Ready());
		
		CS_LOW;		// Select DataFlash

		// Send command
		SPI_WriteByte(AT45D_CMD_BUF1_TO_MAIN_PAGE_PRG_W_ERASE);

		// Send address
		SPI_WriteByte(dA.dAddr[2]);
		SPI_WriteByte(dA.dAddr[1]);
		SPI_WriteByte(dA.dAddr[0]);

		CS_HIGH;		// Deselect DataFlash
		//----------------------------------------------------------
		
		// Set flag to busy
		at45d_ready_flag = FALSE;
	}
	
	//AT45D_PowerDown();
}

void ReadLog(unsigned long LogInd,unsigned char *buffer,unsigned short bytes)
{
	unsigned long Address=0;
	
	Address = LogInd * LOG_SIZE;
	
	//AT45D_ResumeFromPowerDown();
	
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());

	// Select DataFlash
	CS_LOW;
	
	// Send command
	SPI_WriteByte(AT45D_CMD_CONTINUOUS_ARRAY_READ);

	// Calculate page, offset and number of bytes remaining in page
	//page               = address >> 8;
	//start_byte_in_page = address & 0xff;
	
	// Send address
	SPI_WriteByte((unsigned char)(Address>>16));
	SPI_WriteByte((unsigned char)(Address>>8));
	SPI_WriteByte((unsigned char)(Address));
		
	// Send dont-care bits
	SPI_WriteByte(0x00);
	SPI_WriteByte(0x00);
	SPI_WriteByte(0x00);
	SPI_WriteByte(0x00);

	// Read data
	while(bytes)
	{
		*buffer = SPI_ReadByte();
		buffer++;
		bytes--;
	}

	// Deselect DataFlash
	CS_HIGH;
	
	//AT45D_PowerDown();
}

void ReadMinMaxLog(unsigned long AddrOffset,unsigned long LogInd,unsigned char *buffer,unsigned short bytes)
{
	unsigned long Address=0;
	
	Address = LogInd * bytes;
	Address += AddrOffset;
	//AT45D_ResumeFromPowerDown();
	
	// Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
	while(!AT45D_Ready());

	// Select DataFlash
	CS_LOW;
	
	// Send command
	SPI_WriteByte(AT45D_CMD_CONTINUOUS_ARRAY_READ);

	// Calculate page, offset and number of bytes remaining in page
	//page               = address >> 8;
	//start_byte_in_page = address & 0xff;
	
	// Send address
	SPI_WriteByte((unsigned char)(Address>>16));
	SPI_WriteByte((unsigned char)(Address>>8));
	SPI_WriteByte((unsigned char)(Address));
	
	// Send dont-care bits
	SPI_WriteByte(0x00);
	SPI_WriteByte(0x00);
	SPI_WriteByte(0x00);
	SPI_WriteByte(0x00);

	// Read data
	while(bytes)
	{
		*buffer = SPI_ReadByte();
		buffer++;
		bytes--;
	}

	// Deselect DataFlash
	CS_HIGH;
	
	//AT45D_PowerDown();
}

void AT45D_PowerDown(void)
{
	CS_LOW;			// Select DataFlash
    SPI_WriteByte(AT45D_CMD_MAIN_POWER_DOWN);		// Send command
    CS_HIGH;		// Deselect DataFlash
}

void AT45D_ResumeFromPowerDown(void)
{
	CS_LOW;			// Select DataFlash
	SPI_WriteByte(AT45D_CMD_MAIN_RESUME_FROM_POWER_DOWN);		// Send command
	CS_HIGH;		// Deselect DataFlash
}

unsigned char AT45D_Ready(void)
{
	unsigned char data;

    // If flag has already been set, then take short cut
    if(at45d_ready_flag)
    {
        return TRUE;
    }

    // Get DataFlash status
    data = AT45D_GetStatus();

    // See if DataFlash is ready
    if(data & AT45D_STATUS_READY)
    {
        // Set flag
        at45d_ready_flag = TRUE;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

unsigned char AT45D_GetStatus(void)
{
	unsigned char data;

    // Select DataFlash
    CS_LOW;

    // Send command
    SPI_WriteByte(AT45D_CMD_STATUS_REGISTER_READ);

    // Read status
    data = SPI_ReadByte();

    // Deselect DataFlash
    CS_HIGH;

    return data;
}

unsigned char AT45D_page_size_is_pwr_of_two(void)
{
	unsigned char data = AT45D_GetStatus();

    if(data & AT45D_STATUS_PAGE_SIZE)
    {
        // Page size is a power of two
        return TRUE;
    }
    else
    {
        // Page size is not a power of two
        return FALSE;
    }
}

unsigned char AT45D_set_page_size_to_pwr_of_two(void)
{
	if(AT45D_page_size_is_pwr_of_two())
    {
        // Page size is already a power of two
        return FALSE;
    }

    // Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
    while(!AT45D_Ready());
	
    // Select DataFlash
    CS_LOW;

    // Send command
    SPI_WriteByte(0x3D);
    SPI_WriteByte(0x2A);
    SPI_WriteByte(0x80);
    SPI_WriteByte(0xA6);    

    // Deselect DataFlash
    CS_HIGH;

    // Wait until DataFlash is not busy
	at45d_ready_flag = FALSE;
    while(!AT45D_Ready());

    return TRUE;
}


