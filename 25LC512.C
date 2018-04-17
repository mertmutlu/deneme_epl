
/********************************************************************
 *                                                                    
 *                     Software License Agreement                      
 *                                                                     
 * The software supplied herewith by Microchip Technology Incorporated 
 * (the "Company") for its PICmicro® Microcontroller is intended and   
 * supplied to you, the Company’s customer, for use solely and         
 * exclusively on Microchip PICmicro Microcontroller products.         
 *                                                                     
 * The software is owned by the Company and/or its supplier, and is     
 * protected under applicable copyright laws. All rights are reserved.  
 * Any use in violation of the foregoing restrictions may subject the  
 * user to criminal sanctions under applicable laws, as well as to     
 * civil liability for the breach of the terms and conditions of this  
 * license.                                                             
 *                                                                      
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,   
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED   
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A         
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,   
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR          
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.                    
 *                                                                     
 *******************************************************************
 *
 *  Filename:           AN1096_SPI.c
 *  Date:               May 11, 2007
 *  File Version:       1.0
 *  Compiled using:     MPLAB IDE 7.50
 *                      MPLAB C18 2.40
 *
 *  Author:             Martin Kvasnicka
 *  Company:            Microchip Technology Inc.
 *
 *******************************************************************
 *
 *  Files required:     p18f1220.h
 *                      AN1018.h
 *
 ** I N C L U D E S ************************************************/
 //#include "main.h"

#include "25LC512.h"
#include "extern.h"

/** P R I V A T E   P R O T O T Y P E S ****************************/
void byteout(unsigned char);            // Byte output function
unsigned char bytein(void);             // Byte input function
extern  void enable_ee(void);
extern   void disable_ee(void);
/********************************************************************
 * Function:        void LowDensByteWrite(unsigned char data,
 *                                        unsigned int address)
 *
 * Description:     This function performs a byte write operation for
 *                  low-density (<= 4 Kb) devices. It embeds the MSb
 *                  of the memory address into the instruction in
 *                  order to support the 25XX040X. After initiating
 *                  the write cycle, the WIP_Poll function is called
 *                  to determine the end of the cycle.
 *
 * Parameters:      data - Byte of data to be written
 *                  address - Memory location at which to start
 *******************************************************************/
//void LowDensByteWrite(unsigned char data, unsigned int address)
//{
//    static unsigned char command;   // Temp. variable to store cmd.

//    // First embed MSb of address (bit 8) into instruction
//    command = address >> 5;         // Align MSb of address
//    command &= 0x08;                // Mask off extra bits
//    command |= WRITE;               // Merge in WRITE instruction

//    WriteEnable();                  // Set WEL bit for write
//    CS = 0;                         // Bring CS low (active)
//    byteout(command);               // Output command & MSb of address
//    byteout(address&0xFF);          // Output LSB of address
//    byteout(data);                  // Write byte of data
//    CS = 1;                         // Bring CS high (inactive)
//    WIP_Poll();                     // Perform WIP polling
//} // end of LowDensByteWrite(...)

/********************************************************************
 * Function:        void LowDensByteRead(unsigned char data,
 *                                       unsigned int address)
 *
 * Description:     This function performs a byte read operation
 *                  for low-density (<= 4 Kb) devices. It embeds the
 *                  MSb of the memory address into the instruction in
 *                  order to support the 25XX040X.
 *
 * Parameters:      data - Pointer to store byte of data read
 *                  address - Memory location at which to start
 *******************************************************************/
//void LowDensByteRead(unsigned char *data, unsigned int address)
//{
//    static unsigned char command;   // Temp. variable to store cmd.

//    // First embed MSb of address (bit 8) into instruction
//    command = address >> 5;         // Align MSb of address
//    command &= 0x08;                // Mask off extra bits
//    command |= READ;                // Merge in READ instruction

//    CS = 0;                         // Bring CS low (active)
//    byteout(command);               // Output command & MSb of address
//    byteout(address&0xFF);          // Output LSB of address
//    *data = bytein();               // Read byte of data
//    CS = 1;                         // Bring CS high (inactive)
//} // end of LowDensByteRead(...)

/********************************************************************
 * Function:        void LowDensPageWrite(unsigned char *data,
 *                                        unsigned int address,
 *                                        unsigned char size)
 *
 * Description:     This function performs a page write operation for
 *                  low-density (<= 4 Kb) devices. It embeds the MSb
 *                  of the memory address into the instruction in
 *                  order to support the 25XX040X. After initiating
 *                  the write cycle, the WIP_Poll function is called
 *                  to determine the end of the cycle. Note that this
 *                  function does not check for page boundary
 *                  violations.
 *
 * Parameters:      data - Byte array of data to be written
 *                  address - Memory location at which to start
 *                  size - Number of bytes to write
 *******************************************************************/
//void LowDensPageWrite(unsigned char *data, unsigned int address,
//                      unsigned char size)
//{
//    static unsigned char i;         // Loop counter
//    static unsigned char command;   // Temp. variable to store cmd.

//    // First embed MSb of address (bit 8) into instruction
//    command = address >> 5;         // Align MSb of address
//    command &= 0x08;                // Mask off extra bits
//    command |= WRITE;               // Merge in WRITE instruction

//    WriteEnable();                  // Set WEL bit for write
//    CS = 0;                         // Bring CS low (active)
//    byteout(command);               // Output command & MSb of address
//    byteout(address&0xFF);          // Output LSB of address
//    for (i = 0; i < size; i++)      // Loop through number of bytes
//    {
//        byteout(data[i]);           // Write next byte from array
//    }
//    CS = 1;                         // Bring CS high (inactive)
//    WIP_Poll();                     // Perform WIP polling
//} // end of LowDensPageWrite(...)

/********************************************************************
 * Function:        void LowDensSeqRead(unsigned char *data,
 *                                      unsigned int address,
 *                                      unsigned char size)
 *
 * Description:     This function performs a sequential read operation
 *                  for low-density (<= 4 Kb) devices. It embeds the
 *                  MSb of the memory address into the instruction in
 *                  order to support the 25XX040X.
 *
 * Parameters:      data - Array to store data read
 *                  address - Memory location at which to start
 *                  size - Number of bytes to read
 *******************************************************************/
//void LowDensSeqRead(unsigned char *data, unsigned int address,
//                    unsigned char size)
//{
//    static unsigned char i;         // Loop counter
//    static unsigned char command;   // Temp. variable to store cmd.

//    // First embed MSb of address (bit 8) into instruction
//    command = address >> 5;         // Align MSb of address
//    command &= 0x08;                // Mask off extra bits
//    command |= READ;                // Merge in READ instruction

//    CS = 0;                         // Bring CS low (active)
//    byteout(command);               // Output command & MSb of address
//    byteout(address&0xFF);          // Output LSB of address
//    for (i = 0; i < size; i++)      // Loop through number of bytes
//    {
//        data[i] = bytein();         // Read next byte into array
//    }
//    CS = 1;                         // Bring CS high (inactive)
//} // end of LowDensSeqRead(...)

/********************************************************************
 * Function:        void HighDensByteWrite(unsigned char data,
 *                                         unsigned int address)
 *
 * Description:     This function performs a byte write operation for
 *                  high-density (> 4 Kb) devices which feature a
 *                  2-byte address structure. After initiating
 *                  the write cycle, the WIP_Poll function is
 *                  called to determine the end of the cycle. Note
 *                  that this function does not check for page
 *                  boundary violations.
 *
 * Parameters:      data - Byte of data to be written
 *                  address - Memory location at which to start
 *******************************************************************/
 
 
 
 
 
// void send_byte_adc(unsigned char data) 

//{ 
//unsigned char i; 
//for(i = 0; i < 8; i++) {          			 // Send 8 bits 
//	if(data & 0x80) 
//		GPIO_SetBits(ADC_DIN_PORT, ADC_DIN);   		 // SET DATA
//	else 
//		GPIO_ResetBits(ADC_DIN_PORT, ADC_DIN);   	 	// RESSET DATA
// 
// 	delay_clock();
//  	GPIO_SetBits(ADC_SCLK_PORT, ADC_SCLK );       	 // Make a positive clock pulse 
//	data <<= 1; 
//	delay_clock();
//	GPIO_ResetBits(ADC_SCLK_PORT, ADC_SCLK );
//   	delay_clock();        			  	// Get next bit into MSB position   
//	}
//}

////----

//unsigned char readspi_data_adc(void )

//{
//unsigned char i,spidata=0;

//for (i = 0; i < 8; i++) {

//	spidata = spidata << 1;
//		//delay_clock();
//  //spidata += (unsigned char)GPIO_ReadInputDataBit(ADC_DOUT_PORT, ADC_DOUT);
//	GPIO_SetBits(ADC_SCLK_PORT, ADC_SCLK );       	 // Make a positive clock pulse 
//	delay_clock();
//	GPIO_ResetBits(ADC_SCLK_PORT, ADC_SCLK );
//	delay_clock();
//	spidata += (unsigned char)GPIO_ReadInputDataBit(ADC_DOUT_PORT, ADC_DOUT);
//	}
//return((unsigned char)spidata);   
//}
void delay_ee(void) 
{ 
unsigned int i; 

for (i = 0; i < 4000; i++) {
	__nop();

	}
}


void delay_clock(void) 
{ 
//unsigned int i; 

//for (i = 0; i < 4; i++) {
//	__nop();

//	}
}
//--
  void spi_send( char data)
	{
	unsigned char i; 
for(i = 0; i < 8; i++) {          			 // Send 8 bits 
	if(data & 0x80) 
		GPIO_SetBits( ee_si_GPIO_PORT ,  ee_si);   		 // SET DATA
	else 
		GPIO_ResetBits( ee_si_GPIO_PORT ,  ee_si);    	 	// RESSET DATA
 
 	delay_clock();
  	GPIO_SetBits( ee_sck_GPIO_PORT ,  ee_sck);       	 // Make a positive clock pulse 
	data <<= 1; 
	delay_clock();
	GPIO_ResetBits( ee_sck_GPIO_PORT ,  ee_sck); 
   	delay_clock();        			  	// Get next bit into MSB position   
	
	
	}
}
	//--
	
	  char spi_rec(void )
	{

		
		
	unsigned char i,spidata=0;

for (i = 0; i < 8; i++) {

	spidata = spidata << 1;
		//delay_clock();
  //spidata += (unsigned char)GPIO_ReadInputDataBit(ADC_DOUT_PORT, ADC_DOUT);
	//	spidata += (unsigned char)GPIO_ReadInputDataBit( ee_so_GPIO_PORT ,  ee_so); 
	GPIO_SetBits( ee_sck_GPIO_PORT ,  ee_sck);        	 // Make a positive clock pulse 
	delay_clock();
		spidata += (unsigned char)GPIO_ReadInputDataBit( ee_so_GPIO_PORT ,  ee_so); 
	GPIO_ResetBits( ee_sck_GPIO_PORT ,  ee_sck); 
	delay_clock();

	}
return((unsigned char)spidata); 
		}
 
 //--
 
 void write_u16_eeprom(unsigned short data,unsigned short address)
	
{
 
		HighDensByteWrite(	((unsigned char)(data>>8)&0xFF), address);
delay_ee();
		HighDensByteWrite(	(unsigned char)(data&0xFF), address+1);
delay_ee();		
}

unsigned short read_u16_eeprom( unsigned short address)
{
unsigned char temp1,temp2; 
	unsigned short data; 
HighDensByteRead(&temp1,   address);
HighDensByteRead(&temp2,   address+1);
data= (((unsigned short) temp1)<<8)| (unsigned short)temp2; 
	return data; 
	
}


//---

unsigned int read_u32_eeprom( unsigned short address)
{
unsigned char temp1,temp2,temp3,temp4; 
	unsigned int data; 
HighDensByteRead(&temp1,   address);
HighDensByteRead(&temp2,   address+1);
HighDensByteRead(&temp3,   address+2);
HighDensByteRead(&temp4,   address+3);
	
data= ((unsigned int) temp1<<24)| ((unsigned int) temp2<<16)|((unsigned int) temp3<<8)|((unsigned int) temp4);
	return data; 
	
}

//---------------------------------
 void write_u32_eeprom(unsigned int data,unsigned short address)
	
{
 
		HighDensByteWrite(	((unsigned char)(data>>24)&0xFF), address);
delay_ee();
	
		HighDensByteWrite(	((unsigned char)(data>>16)&0xFF), address+1);
delay_ee();
	
		HighDensByteWrite(	((unsigned char)(data>>8)&0xFF), address+2);
delay_ee();
		HighDensByteWrite(	(unsigned char)(data&0xFF), address+3);
delay_ee();		
}

///----------------
  void enable_ee(void)
	{
	
	
	GPIO_ResetBits(ee_cs_GPIO_PORT,ee_cs);
	
	}

	//----------

  void disable_ee(void)
	{
	
	
	GPIO_SetBits(ee_cs_GPIO_PORT,ee_cs);
	
	}
 
void HighDensByteWrite(unsigned char data, unsigned short address)
{
    WriteEnable();                  // Set WEL bit for write
    enable_ee( );                         // Bring CS low (active)
    byteout(WRITE);                 // Output WRITE command
    byteout((address>>8)&0xFF);     // Output MSB of address
    byteout(address&0xFF);          // Output LSB of address
    byteout(data);                  // Write byte of data
    disable_ee();                         // Bring CS high (inactive)
    WIP_Poll();                     // Perform WIP polling
} // end of HighDensByteWrite(...)

/********************************************************************
 * Function:        void HighDensByteRead(unsigned char *data,
 *                                        unsigned int address)
 *
 * Description:     This function performs a byte read operation
 *                  for high-density (> 4 Kb) devices which feature a
 *                  2-byte address structure.
 *
 * Parameters:      data - Pointer to store byte of data read
 *                  address - Memory location at which to start
 *******************************************************************/
void HighDensByteRead(unsigned char *data, unsigned short  address)
{
    enable_ee();                        // Bring CS low (active)
    byteout(READ);                  // Output READ command
    byteout((address>>8)&0xFF);     // Output MSB of address
    byteout(address&0xFF);          // Output LSB of address
    *data = bytein();               // Read byte of data
    disable_ee();                      // Bring CS high (inactive)
} // end of HighDensByteRead(...)

/********************************************************************
 * Function:        void HighDensPageWrite(unsigned char *data,
 *                                         unsigned int address,
 *                                         unsigned char size)
 *
 * Description:     This function performs a page write operation for
 *                  high-density (> 4 Kb) devices which feature a
 *                  2-byte address structure. After initiating
 *                  the write cycle, the WIP_Poll function is
 *                  called to determine the end of the cycle. Note
 *                  that this function does not check for page
 *                  boundary violations.
 *
 * Parameters:      data - Byte array of data to be written
 *                  address - Memory location at which to start
 *                  size - Number of bytes to write
 *******************************************************************/
void HighDensPageWrite(unsigned char *data, unsigned int address,
                       unsigned char size)
{
    static unsigned char i;         // Loop counter

    WriteEnable();                  // Set WEL bit for write
    enable_ee();                      // Bring CS low (active)
    byteout(WRITE);                 // Output WRITE command
    byteout((address>>8)&0xFF);     // Output MSB of address
    byteout(address&0xFF);          // Output LSB of address
    for (i = 0; i < size; i++)      // Loop through number of bytes
    {
        byteout(data[i]);           // Write next byte from array
    }
    disable_ee();                         // Bring CS high (inactive)
    WIP_Poll();                     // Perform WIP polling
} // end of HighDensPageWrite(...)

/********************************************************************
 * Function:        void HighDensSeqRead(unsigned char *data,
 *                                       unsigned int address,
 *                                       unsigned char size)
 *
 * Description:     This function performs a sequential read operation
 *                  for high-density (> 4 Kb) devices which feature a
 *                  2-byte address structure.
 *
 * Parameters:      data - Array to store data read
 *                  address - Memory location at which to start
 *                  size - Number of bytes to read
 *******************************************************************/
void HighDensSeqRead(unsigned char *data, unsigned int address,
                     unsigned char size)
{
    static unsigned char i;         // Loop counter

   enable_ee();                         // Bring CS low (active)
    byteout(READ);                  // Output READ command
    byteout((address>>8)&0xFF);     // Output MSB of address
    byteout(address&0xFF);          // Output LSB of address
    for (i = 0; i < size; i++)      // Loop through number of bytes
    {
        data[i] = bytein();         // Read next byte into array
    }
     disable_ee();                        // Bring CS high (inactive)
} // end of HighDensSeqRead(...)

/********************************************************************
 * Function:        void WriteStatusReg(unsigned char status)
 *
 * Description:     This function writes the value specified by
 *                  status to the Status Register of a device.
 *                  Note that after performing a WREN instruction, a
 *                  RDSR instruction is executed to check that the
 *                  WEL bit is set. However, currently no error-
 *                  handling is done if the WEL bit is not set.
 *
 * Parameters:      status - Value to be written to Status Register
 *******************************************************************/
void WriteStatusReg(unsigned char status)
{
    WriteEnable();                  // Set WEL bit for write
     enable_ee();                        // Bring CS low (active)
    byteout(WRSR);                  // Output WRSR command
    byteout(status);                // Output value to be written
     disable_ee();                        // Bring CS high (inactive)
    WIP_Poll();                     // Perform WIP polling
} // end of WriteStatusReg(...)

/********************************************************************
 * Function:        unsigned char ReadStatusReg(void)
 *
 * Description:     This function reads and returns the value of the
 *                  Status Register of a device.
 *******************************************************************/
unsigned char ReadStatusReg(void)
{
    unsigned char retval;           // Return value variable

     enable_ee();                        // Bring CS low (active)
    byteout(RDSR);                  // Output RDSR command
    retval = bytein();              // Read Status Register byte
     disable_ee();                       // Bring CS high (inactive)

    return retval;                  // Return value
} // end of ReadStatusReg(void)

/********************************************************************
 * Function:        void WriteEnable(void)
 *
 * Description:     This function performs a Write Enable instruction
 *                  to set the WEL bit in the Status Register.
 *                  Note that after performing the WREN instruction, a
 *                  RDSR instruction is executed to check that the
 *                  WEL bit is set. However, currently no error-
 *                  handling is done if the WEL bit is not set.
 *******************************************************************/
void WriteEnable(void)
{
    static unsigned char status;    // Temp. variable to store status reg.

     enable_ee();                        // Bring CS low (active)
    byteout(WREN);                  // Output WREN command
     disable_ee();                         // Bring CS high (inactive)
    status = ReadStatusReg();       // Check that WEL bit is set
} // end of WriteEnable(void)

/********************************************************************
 * Function:        void WriteDisable(void)
 *
 * Description:     This function performs a Write Disable instruction
 *                  to clear the WEL bit in the Status Register.
 *******************************************************************/
void WriteDisable(void)
{
     enable_ee();                       // Bring CS low (active)
    byteout(WRDI);                  // Output WRDS command
     disable_ee();                         // Bring CS high (inactive)
} // end of WriteDisable(void)

/********************************************************************
 * Function:        void byteout(unsigned char byte)
 *
 * Description:     This function outputs a single byte onto the
 *                  SPI bus, MSb first.
 *******************************************************************/
void byteout(unsigned char byte)
{
  spi_send(byte);
} // end of byteout(...)

/********************************************************************
 * Function:        unsigned char bytein(void)
 *
 * Description:     This function inputs a single byte from the
 *                  SPI bus, MSb first.
 *******************************************************************/
unsigned char bytein(void)
{
   
    static unsigned char retval;    // Return value

    retval=spi_rec();

    return retval;
} // end of bytein(void)

/********************************************************************
 * Function:        void WIP_Poll(void)
 *
 * Description:     This function performs WIP polling to determine
 *                  the end of the current write cycle. It does this
 *                  by continuously executing a Read Status Register
 *                  operation until the WIP bit (bit 0 of the Status
 *                  Register) is read low.
 *******************************************************************/
void WIP_Poll(void)
{
    static unsigned char status;    // Variable to store Status Reg.

    do {
        status = ReadStatusReg();   // Perform RDSR operation
    } while(status & 0x01);         // Loop while WIP (bit 0) is 1
} // end of WIP_Poll(void)
//----






//void test (void)  {
//  val.f = 34.567;           // set floating point value
//  store_byte (val.uc[0]);   // write access to the bytes of the float value
//  store_byte (val.uc[1]);
//  store_byte (val.uc[2]);
//  store_byte (val.uc[3]);

//  val.uc[0] = 0x3F;         // set float value (1.1)
//  val.uc[1] = 0x8C;
//  val.uc[2] = 0xCC;
//  val.uc[3] = 0xCD;
//  f = val.f;                // read the value as FLOAT type.
//}
