//#include "main.h"
//#include "extern.h"
//#include "25LC512.h"
//#include "ads1232.h"
//#include "lcd.h"

//#define calibration_def_value  5000 
//#define calibration_def_analog_value   300000

//#define start_pace_calib  200 
//#define    stop_calib      1 

//unsigned char button_state=0;
//void delay_lcd(void);

//void all_icon_off(void)
//{
//unsigned short i; 
//	
//	for(i=0;i<12;i++)
//	{
//put_icon( 2,0x00f0 +(i*2));
//	delay_lcd( );
//	}
//	

//}
//void delay_lcd(void);


//unsigned char  start_manual_calibration(unsigned char channel )
//{
// unsigned char  status; 
//	signed int adc; 
//signed short pace; 
//	unsigned int i ;
//		signed int temp,index ; 
//	EXTI6_disable();
//pace=start_pace_calib; 
//	input_number_unsigned16( (unsigned short)pace,0x004c );
//	

//	ads_set_speed( speed_10_sps);
//	ads_pdwn(power_enable);
//	ads_set_gain(gain_1 );
//	if(channel==1)	ads_channel(channel_0);
//	if(channel==2)	ads_channel(channel_1);
//	 ads_cal();
//  for(i=0;i<0xffff;i++);
//	//adc=ads_read_18();

//	
//	//EXTI6_disable();

//	
//	

//	
//relay_1( 1);
//relay_driver_enable(1);
//i=0; 



//DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);  // 4 *1000 
//Codeur_total=0;
//flags.adc_ready=0;
//EXTI6_Config();
//i=0;
//index=0;
//button_state=0;

//		put_icon( 0,0x00f0 +(index*2));
//while(1)
//{

//	//------
//	// stop mu ? 
//		if(flags.sifre==1)
//		{flags.sifre=0;
//				vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
//    password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
//			if(vp_genel==0xffff)
//			{
//			if( password==0x0002)//start key 
//			{
//				goto_pic(9);
//			relay_1( 0);
//relay_driver_enable(0);
//pace=0;
//DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);  // 4 *1000 
//				status=stop_calib;
//				return status; 
//				
//			}
//			
//			
//		}
//	}
//	//---		
//			
//			switch(button_state) // encoder button 
//			{
//				case 0 : 
//				{
//						if (GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
//		{
//						calibration_analog[index]=adc;
//			input_number_unsigned32( 	calibration_analog[index],(4*index+ 0x0018));
//		
//			index++;
//			if(index>11)index=11;
//			
//				put_icon( 2,0x00f0 +((index-1)*2));
//	     delay_lcd();	
//			put_icon( 0,0x00f0 +((index)*2));
//		button_state=1;
//		}
//				 };break; 
//				
//				//--------- 

//			case 1 : 
//				{
//				
//	     if	(GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==1)button_state=0;
//				
//				 };break; 		
//			
//			
//			
//			}
//				

//		
//	
//	//----
//	if(flags.adc_ready==1)
//	{
//	flags.adc_ready=0;
//	adc=adc_data;
//	input_number_unsigned32( (unsigned int)adc,0x0048 );
//	
//	}
//	
//	
//if(Codeur_total>4)
//{
//		temp=Codeur_total*2;
//pace= (signed short)temp+pace;
//Codeur_total=0;
//if(pace>1000)pace=1000;
//DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);
//input_number_unsigned16( (unsigned short)pace,0x004c );
//}

//if(Codeur_total<-4)
//{
//	temp=Codeur_total*2;
//pace= (signed short)temp+pace;
//Codeur_total=0;
//if(pace<0)pace=0;
//DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);

//input_number_unsigned16( (unsigned short)pace ,0x004c);
//}




//}
//	



//}

////-----
//void calculate_slope(unsigned char channel )
//{

// 	
//unsigned int i ; 

//calibration_real[i];
//calibration_analog[i];

//float slope ; 
//float intercept ;
//float sum_x;
//float sum_y;
//float sum_xy;
//float sum_xx;
//float meanx;
//float meany;


//sum_x=0;
//sum_y=0;
//sum_xy=0;

// for(i=0;i<12;i++)
//{
//sum_x+= (float)calibration_analog[i];


//}	
//	
// for(i=0;i<12;i++)
//{
//sum_y+= (float)calibration_real[i];


//}

//meanx=sum_x/12;
//meany=sum_y/12;

//sum_xx=0;

//for(i=0;i<12;i++)
//{
//sum_xx+=(float)calibration_analog[i]*(float)calibration_analog[i];

//}
//sum_xx=(float)calibration_real[1]*calibration_real[1]+(float)calibration_real[2]*calibration_real[2]+
//	(float)calibration_real[3]*calibration_real[3]+(float)calibration_real[4]*calibration_real[4]+
//		(float)calibration_real[5]*calibration_real[5]+(float)calibration_real[6]*calibration_real[6]+
//			(float)calibration_real[7]*calibration_real[7]+(float)calibration_real[8]*calibration_real[8]+
//				(float)calibration_real[9]*calibration_real[9]+(float)calibration_real[10]*calibration_real[10]+
//					(float)calibration_real[11]*calibration_real[11];

//sum_xy= (float)calibration_analog[1]*calibration_real[1]+(float)calibration_analog[2]*calibration_real[2]+(float)calibration_analog[3]*calibration_real[3]+(float)calibration_analog[4]*calibration_real[4]+(float)calibration_analog[5]*calibration_real[5]+(float)calibration_analog[6]*calibration_real[6]+(float)calibration_analog[7]*calibration_real[7]+(float)calibration_analog[8]*calibration_real[8]+(float)calibration_analog[9]*calibration_real[9]+(float)calibration_analog[10]*calibration_real[10]+(float)calibration_analog[11]*calibration_real[11]+(float)calibration_analog[12]*calibration_real[12];
//sum_xy=0;
//for(i=0;i<12;i++)
//{
//sum_xy+=(float)calibration_real[i]*(float)calibration_analog[i];

//}

//slope= ((12*sum_xy)-(sum_x*sum_y))/((12*sum_xx)-(sum_x*sum_x));

//intercept= meany-(slope*meanx);
////------------------------------------------------------------------


//switch(channel)
//{
//	case 1:{
//     		channel_calib_1.slope=slope;
//		     channel_calib_1.intercept=intercept; 
//						};break;
////--------------------
//	case 2:{
//     		channel_calib_2.slope=slope;
//		     channel_calib_2.intercept=intercept; 
//						};break;
//		//-------------
//case 3:{
//     		channel_calib_3.slope=slope;
//		     channel_calib_3.intercept=intercept; 
//						};break;						
//}

//}

////----
//void calib_save_analog(unsigned char channel)
//{
//	unsigned short address,i; 

//	
//	switch(channel)
//{
//	case 1: address=calibration_channel_1_analog_adress ;break;
//	case 2: address=calibration_channel_2_analog_adress ;break;
//	case 3: address=calibration_channel_3_analog_adress ;break;
//}


//for(i=0;i<12;i++)
//{
//	write_u32_eeprom(calibration_analog[i],(address+ i*4));
//delay_lcd();
//}
//}

////---

// void delay_lcd(void)
//{

//unsigned short i;
//	for(i=0;i<100;i++)
//	{
//	__nop();
//	}

//}

//void lcd_put_calib(void)
//{


//unsigned short i; 
//	
//	
//	for(i=0;i<12;i++)
//	{
//	
//	input_number_unsigned16( 	calibration_real[i],2*i);
//	delay_lcd();	
//	}

//	
//	//------------------------------------------------------
//	
//		for(i=0;i<12;i++)
//	{
//	
//	input_number_unsigned32( 	calibration_analog[i],(4*i+ 0x0018));
//	delay_lcd();	
//	}

//}

////-[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]][[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]



////-[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]][[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]




//	
