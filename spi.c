
#include "spi.h"
#include "stm32f4xx_spi.h"
void init_SPI3(void)

               

{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

	  SPI_StructInit(&SPI_InitStruct);

    // enable clock for used IO pins

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    /* configure pins used by SPI1

     * PA5 = SCK

     * PA6 = MISO

     * PA7 = MOSI

     */

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // connect SPI1 pins to SPI alternate function
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
    // enable clock for used IO pins
 

    /* Configure the chip select pin

       in this case we will use PE7 */

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode =   GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF ;
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOA, &GPIO_InitStruct);

 

    //GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 high

 

    // enable peripheral clock

 

 

    /* configure SPI1 in Mode 0

     * CPOL = 0 --> clock is low when idle

     * CPHA = 0 --> data is sampled at the first edge

     */

    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      // data sampled at first edge
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS management to internal and pull internal NSS high
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // SPI frequency is APB2 frequency / 4
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first

                               //SPI_InitStructure.SPI_CRCPolynomial = 0;

    SPI_Init(SPI3, &SPI_InitStruct);
    SPI_Cmd(SPI3, ENABLE); // enable

//--

}

//void init_SPI1(void)
//   {
//    GPIO_InitTypeDef GPIO_InitStruct;
//    SPI_InitTypeDef SPI_InitStruct;
//    SPI_StructInit(&SPI_InitStruct);

//    // enable clock for used IO pins
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//    /* configure pins used by SPI1

//     * PA5 = SCK

//     * PA6 = MISO

//     * PA7 = MOSI

//     */

//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;

//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;

//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

//    GPIO_Init(GPIOA, &GPIO_InitStruct);

// 

//    // connect SPI1 pins to SPI alternate function

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

// 

//    // enable clock for used IO pins

// 

// 

//    /* Configure the chip select pin

//       in this case we will use PE7 */

//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_5;

//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;

//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

//    GPIO_Init(GPIOA, &GPIO_InitStruct);

//                              

//                                 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;

//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF ;

//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

//    GPIO_Init(GPIOA, &GPIO_InitStruct);

// 

// 

//    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines

//    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high

//    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide

//    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle

//    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      // data sampled at first edge

//    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS management to internal and pull internal NSS high

//    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // SPI frequency is APB2 frequency / 4

//    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first

//                               //SPI_InitStructure.SPI_CRCPolynomial = 0;

//    SPI_Init(SPI1, &SPI_InitStruct);

//    SPI_Cmd(SPI1, ENABLE); // enable

////--

// 

// 

// 

//}

///--

 

char SPI3_send(char data)
	{

					SPI3->DR = data; // write data to be transmitted to the SPI data register
			 while( !(SPI3->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
			 while( !(SPI3->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
				while( SPI3->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
			 return SPI3->DR; // return    received data from SPI data register

}

 

//-

 

 

unsigned char  SPISendByte(unsigned char  byte)

{
    /** flush away any rogue data in rx buffer **/
    if (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == SET)
    {
			SPI_I2S_ReceiveData(SPI3);
    }

  /** Loop while DR register in not empty **/
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);

   /** Send byte through the SPI1 peripheral **/

  SPI_I2S_SendData(SPI3, byte);

   /* Wait to receive a byte */

  while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);

 
  /* Return the byte read from the SPI bus */

  return (uint8_t)SPI_I2S_ReceiveData(SPI3);

}
