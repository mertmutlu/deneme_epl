#include "main.h"
#include "extern.h"
#include "ads1232.h"


void ads_set_speed(unsigned char speed)//10--80
{
	
	
	switch(speed)
	{
		case speed_10_sps:   GPIO_ResetBits(ads_speed_GPIO_PORT ,ads_speed);break;
	  case speed_80_sps:   GPIO_SetBits(ads_speed_GPIO_PORT ,ads_speed);break;
	
	
	}



}
//------------

void ads_set_gain(unsigned char gain )
{

switch(gain)
{
	case gain_1 : {
								GPIO_ResetBits(ads_g0_GPIO_PORT ,ads_g0);
	            	GPIO_ResetBits(ads_g1_GPIO_PORT ,ads_g1);
	              };break;
//-----------------------------
	case gain_2 : {
								GPIO_SetBits(ads_g0_GPIO_PORT ,ads_g0);
	            	GPIO_ResetBits(ads_g1_GPIO_PORT ,ads_g1);
	              };break;	

//---
	case gain_64 : {
								GPIO_ResetBits(ads_g0_GPIO_PORT ,ads_g0);
	            	GPIO_SetBits(ads_g1_GPIO_PORT ,ads_g1);
	              };break;								

//---
	case gain_128 : {
								GPIO_SetBits(ads_g0_GPIO_PORT ,ads_g0);
	            	GPIO_SetBits(ads_g1_GPIO_PORT ,ads_g1);
	              };break;	
}


}
///---

void ads_channel(unsigned char channel)
{
switch(channel)
{

	case channel_0:GPIO_ResetBits(ads_a0_GPIO_PORT ,ads_a0);break;
  case channel_1:GPIO_SetBits(ads_a0_GPIO_PORT ,ads_a0);break;

}


}

//---------------------

void ads_pdwn(unsigned char power)
{
switch(power)
{

	case power_disable: GPIO_ResetBits(ads_pwdn_GPIO_PORT ,ads_pwdn); break;
	case power_enable: GPIO_SetBits(ads_pwdn_GPIO_PORT ,ads_pwdn); break;
}


}

//---

void delay_clock_ads(void) 
{ 
unsigned int i; 

for (i = 0; i < 5; i++) {
	__nop();

	}
}
//---------------------------------

signed int ads_read(void)
{
signed int adc,adc1; 
unsigned char i;
adc=0;
	
	for (i = 0; i < 24; i++) 
	{
		
	
	GPIO_SetBits( ads_sclk_GPIO_PORT ,  ads_sclk);        	 // Make a positive clock pulse 
	delay_clock_ads();
		adc = adc << 1;
	GPIO_ResetBits( ads_sclk_GPIO_PORT ,  ads_sclk); 
	delay_clock_ads();
	adc += /*(signed int)*/GPIO_ReadInputDataBit( ads_dout_GPIO_PORT ,  ads_dout); 


	}
	
	GPIO_SetBits( ads_sclk_GPIO_PORT ,  ads_sclk);        	 // Make a positive clock pulse 
	delay_clock_ads();
	GPIO_ResetBits( ads_sclk_GPIO_PORT ,  ads_sclk); 
	delay_clock_ads();
	
	//----
	
		if ((adc& 0x00800000) == 0x00800000)   // check if negative 
			{
				
				adc1=((~adc)&0x00FFFFFF)+1;

				adc=-1*adc1;
			}
				
	
return adc; 


}	
//---



signed int ads_read_18(void)
{
signed int adc,adc1; 
unsigned char i;
adc=0;
	
	for (i = 0; i < 24; i++) 
	{
		
	
	GPIO_SetBits( ads_sclk_GPIO_PORT ,  ads_sclk);        	 // Make a positive clock pulse 
	delay_clock_ads();
		adc = adc << 1;
	GPIO_ResetBits( ads_sclk_GPIO_PORT ,  ads_sclk); 
	delay_clock_ads();
	adc += /*(signed int)*/GPIO_ReadInputDataBit( ads_dout_GPIO_PORT ,  ads_dout); 


	}
	
	GPIO_SetBits( ads_sclk_GPIO_PORT ,  ads_sclk);        	 // Make a positive clock pulse 
	delay_clock_ads();
	GPIO_ResetBits( ads_sclk_GPIO_PORT ,  ads_sclk); 
	delay_clock_ads();
	
	adc=(adc>>5)& 0x0007FFFF;
	//----
	
		if ((adc& 0x0040000) == 0x40000)   // check if negative 
			{
				
				adc1=((~adc)&0x007ffff)+1;

				adc=-1*adc1;
			}
				
	
return adc; 


}

//---

void ads_cal(void) // offset calibration 26 sck 
{
unsigned char i; 
	
while(  GPIO_ReadInputDataBit(ads_dout_GPIO_PORT , ads_dout)==1);
	for(i=0;i<26;i++)
	{
		GPIO_SetBits( ads_sclk_GPIO_PORT ,  ads_sclk);        	 // Make a positive clock pulse 
	delay_clock_ads();
	GPIO_ResetBits( ads_sclk_GPIO_PORT ,  ads_sclk); 
	delay_clock_ads();
	
	}

	for(i=0;i<100;i++)
	{
		delay_clock_ads();
	}
	
}
