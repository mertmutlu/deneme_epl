#include "main.h"
#include "extern.h"
#include "25LC512.h"
#include "ads1232.h"
#include "lcd.h"
#include "task_test.h"
#include "calibration_press.h"

#define calibration_def_value  5000 
#define calibration_def_analog_value   300000

#define start_pace_calib  200 
#define    stop_calib      1 

unsigned char which_channel=0;
unsigned char button_state=0;

unsigned char sensor_ch1;
unsigned char sensor_ch2;
unsigned char sensor_ch3;
//********************************************************
void setting_task_state_3 (void){
	
		    /* Get the RTC current Time */
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    /* Get the RTC current Date */
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		
		input_number_unsigned16(RTC_DateStructure.RTC_Date,2);
		input_number_unsigned16(RTC_DateStructure.RTC_Month,4);
		input_number_unsigned16(RTC_DateStructure.RTC_Year,6);
		input_number_unsigned16(RTC_TimeStructure.RTC_Hours,8);
		input_number_unsigned16(RTC_TimeStructure.RTC_Minutes,10);
		input_number_unsigned16(RTC_TimeStructure.RTC_Seconds,12);
		
	
	
			delay_setting=0;
//	while(delay_setting<25);
setting_task_state=2;
	
}

void setting_task_state_2(void){ //TARIH SAAT AYARI

		if(paket_geldi==1)
{
   paket_geldi=0; 
	 unsigned short i;
	 unsigned short address,eeprom_address;
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low;
	

	
	
	//		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
//		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 

	switch(vp_genel)
		{
		case 0: 
		{	if(password==1)
	{
	
	goto_pic(18); 
	setting_task_state=0; 
		
	
	}
		
		};break;
			
			//---
			case 2 : {
				RTC_DateStructure.RTC_Date=password;
				input_number_unsigned16(RTC_DateStructure.RTC_Date,2);
				RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
			};break;
		
			case 4 :{
				RTC_DateStructure.RTC_Month=password;
				input_number_unsigned16(RTC_DateStructure.RTC_Month,4);
			  RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
			};break;
			case 6 :{
				RTC_DateStructure.RTC_Year=password;
				input_number_unsigned16(RTC_DateStructure.RTC_Year,6);
			  RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
			};break;
			case 8 :{
				RTC_TimeStructure.RTC_Hours=password;
				input_number_unsigned16(RTC_TimeStructure.RTC_Hours,8);
			RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);  
			};break;
			case 10 :{
				RTC_TimeStructure.RTC_Minutes=password;
				input_number_unsigned16(RTC_TimeStructure.RTC_Minutes,10);
			RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);  
			};break;
			case 12 :{
				RTC_TimeStructure.RTC_Seconds=password;
				input_number_unsigned16(RTC_TimeStructure.RTC_Seconds,12);
			RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);  
			};break;
	
		}
		
	
	
	
	
}
	
	
else if (delay_setting>25)
{

	setting_task_state=3;

}
	
}
//*******************************************************
void kanal_setting_task_state_0(void){

//unsigned short i;
	//unsigned short address,eeprom_address;
	if(paket_geldi==1)
{
   paket_geldi=0; 
	 unsigned short i;
	 unsigned short address,eeprom_address;
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
	 // keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);  
	//keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	
	if(	vp_genel==0)
	{
	switch(password)
	{
		case 1: {  // VALVE DELAY SAYFASINA GIT
			//--
			goto_pic(22); 
			eeprom_address=settings_ch_1_adress; //kanala girmeden eepromu oku ve yaz 
			
	
			
			for (i=0; i<7;i++)
	{
	settings[i]=read_u16_eeprom(eeprom_address + 2*i );
	//if(settings[i]>calibration_def_value)settings[i]=calibration_def_value; 
			input_number_unsigned16(settings[i],2*i);
			delay_lcd();
	}
	     HighDensByteRead(&sensor_ch1,sensor_1_adress);
	
	if(sensor_ch1==1){
		input_number_unsigned16( 'OK',0x000E );
			  input_number_unsigned16( 0,0x0010 );
	
	}
	else 	{
		
		input_number_unsigned16( 'OK',0x0010 );
			  input_number_unsigned16( 0,0x000E );
	}
	
		


			kanal_setting_task_state=1;

			};break;
		//-----
			case 2: {		
				goto_pic(22); 
			eeprom_address=settings_ch_2_adress; //kanala girmeden eepromu oku ve yaz 

			for (i=0; i<7;i++)
	{
	settings[i]=read_u16_eeprom(eeprom_address + 2*i );
	//if(settings[i]>calibration_def_value)settings[i]=calibration_def_value; 
			input_number_unsigned16(settings[i],2*i);
			delay_lcd();
	}
	     HighDensByteRead(&sensor_ch2,sensor_2_adress);
	
	if(sensor_ch2==1){
		input_number_unsigned16( 'OK',0x000E );
			  input_number_unsigned16( 0,0x0010 );
	
	}
	else 	{
		
		input_number_unsigned16( 'OK',0x0010 );
			  input_number_unsigned16( 0,0x000E );
	}
	
		


			kanal_setting_task_state=2;

		};break;
			
			case 3: {
					goto_pic(22); 
			eeprom_address=settings_ch_3_adress; //kanala girmeden eepromu oku ve yaz 
			
	
			
			for (i=0; i<7;i++)
	{
	settings[i]=read_u16_eeprom(eeprom_address + 2*i );
	//if(settings[i]>calibration_def_value)settings[i]=calibration_def_value; 
			input_number_unsigned16(settings[i],2*i);
			delay_lcd();
	}
	     HighDensByteRead(&sensor_ch3,sensor_3_adress);
	
	if(sensor_ch3==1){
		input_number_unsigned16( 'OK',0x000E );
			  input_number_unsigned16( 0,0x0010 );
	
	}
	else 	{
		
		input_number_unsigned16( 'OK',0x0010 );
			  input_number_unsigned16( 0,0x000E );
	}
	
		


			kanal_setting_task_state=3;

		};break;
		
			case 4: {   //BACK TO SETTINGS SCREEN
	   setting_task_state=0; 
	  	goto_pic(18);
			};break;
}
}		
}
}

//********************************************************
void kanal_setting_task_state_1(void){
unsigned short  address;
	if(paket_geldi==1)
{
		paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
    //keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);
	  //keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	

if(	vp_genel<=0x0010)// real values   EEPROMA KAYDET
{
  address=settings_ch_1_adress;
	write_u16_eeprom(password,(address+vp_genel));
	//write_u16_eeprom(password,(	sensor_selected_1+vp_genel));
	//write_u32_eeprom(unsigned int data,unsigned short address)
  delay_lcd(); 
	
	

}

if(	vp_genel==0x000E||vp_genel==0x0010||vp_genel==0x0012)
	{
		
		switch(password){ //4-20 mA
			case 1:{	
			
				input_number_unsigned16( 'OK',0x000E );
			  input_number_unsigned16( 0,0x0010 );
				//write_u16_eeprom('OK',(sensor_1_adress));
				sensor_ch1=1;
				HighDensByteWrite(sensor_ch1, sensor_1_adress);

			};break;
			case 2:{         //Load Cell
			
				input_number_unsigned16( 'OK',0x0010 );
			  input_number_unsigned16( 0,0x000E );
			//	write_u16_eeprom('OK',(sensor_2_adress));
					sensor_ch1=2;
				HighDensByteWrite(sensor_ch1, sensor_1_adress);
			};break;
			
			case 3:{
		goto_pic(9);
		kanal_setting_task_state=0;
	
			};break;
		}
		
	}
	


	}



}
//********************************************************
void kanal_setting_task_state_2(void){
unsigned short  address;
	if(paket_geldi==1)
{
		paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
    //keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);
	  //keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	

if(	vp_genel<=0x0010)// real values   EEPROMA KAYDET
{
  address=settings_ch_2_adress;
	write_u16_eeprom(password,(address+vp_genel));
	//write_u16_eeprom(password,(	sensor_selected_1+vp_genel));
	//write_u32_eeprom(unsigned int data,unsigned short address)
  delay_lcd(); 
	
	

}

if(	vp_genel==0x000E||vp_genel==0x0010||vp_genel==0x0012)
	{
		
		switch(password){ //4-20 mA
			case 1:{	
			
				input_number_unsigned16( 'OK',0x000E );
			  input_number_unsigned16( 0,0x0010 );
				//write_u16_eeprom('OK',(sensor_1_adress));
				sensor_ch2=1;
				HighDensByteWrite(sensor_ch2, sensor_2_adress);

			};break;
			case 2:{         //Load Cell
			
				input_number_unsigned16( 'OK',0x0010 );
			  input_number_unsigned16( 0,0x000E );
			//	write_u16_eeprom('OK',(sensor_2_adress));
					sensor_ch2=2;
				HighDensByteWrite(sensor_ch2, sensor_2_adress);
			};break;
			
			case 3:{
		goto_pic(9);
		kanal_setting_task_state=0;
	
			};break;
		}
		
	}
	


	}




}
//********************************************************
void kanal_setting_task_state_3(void){

	unsigned short  address;
	if(paket_geldi==1)
{
		paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
    //keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);
	  //keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	

if(	vp_genel<=0x0010)// real values   EEPROMA KAYDET
{
  address=settings_ch_3_adress;
	write_u16_eeprom(password,(address+vp_genel));
	//write_u16_eeprom(password,(	sensor_selected_1+vp_genel));
	//write_u32_eeprom(unsigned int data,unsigned short address)
  delay_lcd(); 
	
	

}

if(	vp_genel==0x000E||vp_genel==0x0010||vp_genel==0x0012)
	{
		
		switch(password){ //4-20 mA
			case 1:{	
			
				input_number_unsigned16( 'OK',0x000E );
			  input_number_unsigned16( 0,0x0010 );
				//write_u16_eeprom('OK',(sensor_1_adress));
				sensor_ch3=1;
				HighDensByteWrite(sensor_ch3, sensor_3_adress);

			};break;
			case 2:{         //Load Cell
			
				input_number_unsigned16( 'OK',0x0010 );
			  input_number_unsigned16( 0,0x000E );
			//	write_u16_eeprom('OK',(sensor_2_adress));
					sensor_ch3=2;
				HighDensByteWrite(sensor_ch3, sensor_3_adress);
			};break;
			
			case 3:{
		goto_pic(9);
		kanal_setting_task_state=0;
	
			};break;
		}
		
	}
	


	}





}


//********************************************************
void setting_task_state_0(void){ ////Denetle hangisine basildi 
if(paket_geldi==1)
{
    paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
	if(	vp_genel==0)
	{
	
	switch(password){
	
		case 1: {   ///KANAL AYARI SECILDI
			goto_pic(9);
			
			
			
			
			setting_task_state=1;
				};break;
		case 2: {   ///DATE SETTINGS SECILDI
			goto_pic(26);
			setting_task_state=3;	
		};break;	
		case 3: {  ///GERI GEL ANA MENUYE DON
		goto_pic(1);
		press_task_state=0;
		};break;
	}
}
}
}
void setting_task_state_1(void){ //HANGI KANALA GIRECEGINI DENETLE
switch(kanal_setting_task_state)
{
	case 0 : kanal_setting_task_state_0(); break; //hangisi secildi GIT DENETLE
	case 1 : kanal_setting_task_state_1(); break; //  CH1
	case 2 : kanal_setting_task_state_2(); break; //	CH2
	case 3 : kanal_setting_task_state_3(); break; //	CH3
}	

}

	
void setting_task(void){
switch(setting_task_state)
{
	case 0 : setting_task_state_0(); break; //hangisi secildi GIT DENETLE
	case 1 : setting_task_state_1(); break; // Channel Setting
	case 2 : setting_task_state_2(); break; //	Date Setting
	case 3 : setting_task_state_3(); break; //	Date ayarlari yazdirma
}	

}


//**
//****
//******
//********
//**********
//************
//**************
//****************
//**************
//************
//**********
//********
//******
//****
//**
static void save_eeprom(unsigned char channel )
{
	unsigned short address ; 
if( vp_genel<=0x0046)// data changes 
{
	//--save eeprom : 

if(	vp_genel<=0x0016)// real values 
{
switch(calibration_channel)
{
	case 1: address=calibration_channel_1_real_adress ;break;
	case 2: address=calibration_channel_2_real_adress ;break;
	case 3: address=calibration_channel_3_real_adress ;break;
}

	write_u16_eeprom(password,(address+vp_genel));
delay_lcd(); 

}
else//analog
{

switch(calibration_channel)
{
	case 1: address=calibration_channel_1_analog_adress ;break;
	case 2: address=calibration_channel_2_analog_adress ;break;
	case 3: address=calibration_channel_3_analog_adress ;break;
}

	write_u32_eeprom(keypad_number,(address+(vp_genel-0x0018)));
delay_lcd();

}

}
}
//**
//****
//******
//********
//**********
//************
//**************
//****************
//**************
//************
//**********
//********
//******
//****
//**
void read_calibration_eeprom(unsigned char calibration_channel)
{

	unsigned int i,eeprom_address; 
	switch(calibration_channel)
	{
		case 1: eeprom_address=calibration_channel_1_real_adress;break;
		case 2: eeprom_address=calibration_channel_2_real_adress;break;
		case 3: eeprom_address=calibration_channel_3_real_adress;break;
	}
	//--------------------------------------------------------
	for (i=0; i<12;i++)
	{
	calibration_real[i]=read_u16_eeprom(eeprom_address + 2*i );
	if(calibration_real[i]>calibration_def_value)calibration_real[i]=calibration_def_value;
	}
	input_number_unsigned16( 	calibration_real[i],2*i);
	delay_lcd();	
	
		switch(calibration_channel)
	{
		case 1: eeprom_address=calibration_channel_1_analog_adress;break;
		case 2: eeprom_address=calibration_channel_2_analog_adress;break;
		case 3: eeprom_address=calibration_channel_3_analog_adress;break;
	}
	//--------------------------------------------------------
	for (i=0; i<12;i++)
	{
	calibration_analog[i]=read_u32_eeprom(eeprom_address + 4*i );
	if(calibration_analog[i]>calibration_def_value)calibration_analog[i]=calibration_def_analog_value;
	}
	input_number_unsigned32( 	calibration_analog[i],(4*i+ 0x001A));
	delay_lcd();
}
//**
//****
//******
//********
//**********
//************
//**************
//****************
//**************
//************
//**********
//********
//******
//****
//**
void calibration_task_state_0(void){ // DENETLE HANGI KANALA GIRILIYOR 
unsigned short i;
	unsigned short address,eeprom_address;
	if(paket_geldi==1)
{
paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
	 // keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);  
	//keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	
	if(	vp_genel==0)
	{
	switch(password)
	{
		case 1: {
			//--
			goto_pic(11);
			eeprom_address=calibration_channel_1_real_adress; //kanala girmeden eepromu oku ve yaz 
	for (i=0; i<12;i++)
	{
	calibration_real[i]=read_u16_eeprom(eeprom_address + 2*i );
	if(calibration_real[i]>calibration_def_value)calibration_real[i]=calibration_def_value; 
			input_number_unsigned16( 	calibration_real[i],2*i);
	delay_lcd();
	}

	//--

	eeprom_address=calibration_channel_1_analog_adress;
	for (i=0; i<12;i++)
	{	
		//calibration_analog[i]=read_u32_eeprom(0x0034);
	calibration_analog[i]=read_u32_eeprom(eeprom_address + 4*i );
	if(calibration_analog[i]>calibration_def_analog_value)calibration_analog[i]=calibration_def_analog_value;
			input_number_unsigned32( 	calibration_analog[i],(4*i+ 0x001A));
	delay_lcd();
	}
			calibration_task_state=1;
//				if(vp_genel=58)
//				{}
			};break;
		//-----
			case 2: {			
				
				goto_pic(11);
			eeprom_address=calibration_channel_2_real_adress;
	for (i=0; i<12;i++)
	{
	calibration_real[i]=read_u16_eeprom(eeprom_address + 2*i );
	if(calibration_real[i]>calibration_def_value)calibration_real[i]=calibration_def_value;
			input_number_unsigned16( 	calibration_real[i],2*i);
	delay_lcd();
	}

	//--
	
	eeprom_address=calibration_channel_2_analog_adress;
	for (i=0; i<12;i++)
	{
	calibration_analog[i]=read_u32_eeprom(eeprom_address + 4*i );
	if(calibration_analog[i]>calibration_def_analog_value)calibration_analog[i]=calibration_def_analog_value;
			input_number_unsigned32( 	calibration_analog[i],(4*i+ 0x001A));
	delay_lcd();
	}

	calibration_task_state=2;
		};break;
			
			case 3: {
				
				
				goto_pic(11);
			eeprom_address=calibration_channel_3_real_adress;
	for (i=0; i<12;i++)
	{
	calibration_real[i]=read_u16_eeprom(eeprom_address + 2*i );
	if(calibration_real[i]>calibration_def_value)calibration_real[i]=calibration_def_value;
			input_number_unsigned16( 	calibration_real[i],2*i);
	delay_lcd();
	}

	//--
	
	eeprom_address=calibration_channel_3_analog_adress;
	for (i=0; i<12;i++)
	{
	calibration_analog[i]=read_u32_eeprom(eeprom_address + 4*i );
	if(calibration_analog[i]>calibration_def_analog_value)calibration_analog[i]=calibration_def_analog_value;
			input_number_unsigned32( 	calibration_analog[i],(4*i+ 0x001A));
	delay_lcd();
	}
	
	calibration_task_state=3;
	goto_pic(11);
		};break;
		
			case 4: {   //BACK TO MAIN SCREEN
	   press_task_state=0; //en basa donduk
	  	goto_pic(1);
			};break;
	}
}
	
			
	}

}
//*****************************************************
void calibration_task_state_1(void){ //CHANNEL 1 
	which_channel=1;
	unsigned short  address;
	if(paket_geldi==1)
{
paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
    keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);
	  //keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	
	//---***///****////***/*/*/*/*//**/**/*/*/*/*
//	if( vp_genel<=0x0046)// data changes 
//{
	//--save eeprom : 

if(	vp_genel<=0x0016)// real values   EEPROMA KAYDET
{
  address=calibration_channel_1_real_adress;
	write_u16_eeprom(password,(address+vp_genel));
  delay_lcd(); 

}
else if ( vp_genel<=0x0046) //analog
{
	address=  calibration_channel_1_analog_adress;
	address=address+ vp_genel- 0x001A;
	write_u32_eeprom(keypad_number,address);
  //delay_lcd();

}


	
		if(	vp_genel==0x0058)
	{
	switch(password)
	{
		 case 1: {  //Calibration OK
					
		};break;
		
			case 2: {  //Calibration START
					calibration_task_state=4;
		};break;
			
			case 3: {  //BACK SCREEN
			goto_pic(9);
		  calibration_task_state=0;		
		};break;
	
	}
}
	}

}

	
	

//************************************************
void calibration_task_state_2(void){ //CHANNEL 2
  which_channel=2;
	unsigned int address2;
	if(paket_geldi==1)
{
paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
   keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);
	//keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	
	//---***///****////***/*/*/*/*//**/**/*/*/*/*
	if( vp_genel<=0x0046)// data changes 
{
	//--save eeprom : 

if(	vp_genel<=0x0016)// real values 
{
  address2=calibration_channel_2_real_adress;
	write_u16_eeprom(password,(address2+vp_genel));
  delay_lcd(); 

}
else//analog
{
	address2=calibration_channel_2_analog_adress;

	write_u32_eeprom(keypad_number,(address2+(vp_genel-0x001A)));
  delay_lcd();

}

}
	
		if(	vp_genel==0x0058)
	{
	switch(password)
	{
		 case 1: {  //Calibration OK
					
		};break;
		
			case 2: {  //Calibration START
					calibration_task_state=4;
		};break;
			
			case 3: {  //BACK SCREEN
		
				goto_pic(9);
		calibration_task_state=0;
		};break;
	
	}
}
	

}

}
//************************************************
void calibration_task_state_3(void){  //CHANNEL 3
   which_channel=3;
	unsigned int address3;
	if(paket_geldi==1)
{
paket_geldi=0; 
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
    keypad_number=((signed int )keypad_3<<24)|((signed int )keypad_2<<16)|((signed int )keypad_1<<8)|((signed int )keypad_0);
	//keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

	
	//---***///****////***/*/*/*/*//**/**/*/*/*/*
	if( vp_genel<=0x0046)// data changes 
{
	//--save eeprom : 

if(	vp_genel<=0x0016)// real values 
{
  address3=calibration_channel_3_real_adress;
	write_u16_eeprom(password,(address3+vp_genel));
  delay_lcd(); 

}
else  //analog
{
	address3=calibration_channel_3_analog_adress;

	write_u32_eeprom(keypad_number,(address3+(vp_genel-0x001A)));
  delay_lcd();

}

}
	
		if(	vp_genel==0x0058)
	{
	switch(password)
	{
		 case 1: {  //Calibration OK
					
		};break;
		
			case 2: {  //Calibration START
				calibration_task_state=4;
				
		};break;
			
			case 3: {  //BACK SCREEN
		
				goto_pic(9);
		calibration_task_state=0;
		};break;
	}
}
}
}
//************************************************
	void calibration_task_state_4(void){    // VE KALIBRASYON BASLAR
  unsigned char channel;


 unsigned char  status; 
	signed int adc; 
signed short pace; 
	unsigned int i ;
		signed int temp,index ; 
	EXTI6_disable();
  pace=start_pace_calib; 
	input_number_unsigned16( (unsigned short)pace,0x0054 );
	ads_set_speed( speed_10_sps);
	ads_pdwn(power_enable);
	ads_set_gain(gain_1 );
	
		
  if( which_channel==1)	ads_channel(channel_0);
	if(which_channel==2)	ads_channel(channel_1);
	 ads_cal();
  for(i=0;i<0xffff;i++);
	//adc=ads_read_18();
	//EXTI6_disable();
	relay_1( 1);
	relay_driver_enable(1);
	i=0; 
	DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);  // 4 *1000 
	Codeur_total=0;
	flags.adc_ready=0;
	EXTI6_Config();
	i=0;
	index=0;
	button_state=0;

		put_icon( 0,0x00f0 +(index*2));

		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
    password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
			if(vp_genel==0xffff)
			{
			if( password==0x0004)//start key 
			{
			relay_1( 0);
relay_driver_enable(0);
pace=0;
DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);  // 4 *1000 
				status=stop_calib;
				//return status; 
				
			}
			
	//------------------------------------------		
				if( password==0x0005)//start key 
			{
				
				
		index--;
			if(index<0)index=0;
			
				put_icon( 2,0x00f0 +((index+1)*2));
	     delay_lcd();	
			put_icon( 0,0x00f0 +((index)*2));
			}
			
			//---
						if( password==0x0006)//start key 
			{
				
				
			index++;
			if(index>11)index=11;
			
				put_icon( 2,0x00f0 +((index-1)*2));
	     delay_lcd();	
			put_icon( 0,0x00f0 +((index)*2));
			}
			
			
			//---
			
							if( password==0x0003)//start key 
			{
    calibration_analog[index]=adc;
			input_number_unsigned32( 	calibration_analog[index],(4*index+ 0x0018));
		
			index++;
			if(index>11)index=11;
			
				put_icon( 2,0x00f0 +((index-1)*2));
	     delay_lcd();	
			put_icon( 0,0x00f0 +((index)*2));

			}
			
			
		}
	
	//---		
			
			switch(button_state) // encoder button 
			{
				case 0 : 
				{
						if (GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
		{
						calibration_analog[index]=adc;
			input_number_unsigned32( 	calibration_analog[index],(4*index+ 0x0018));
		
			index++;
			if(index>11)index=11;
			
				put_icon( 2,0x00f0 +((index-1)*2));
	     delay_lcd();	
			put_icon( 0,0x00f0 +((index)*2));
		button_state=1;
		}
				 };break; 
				
				//--------- 

			case 1 : 
				{
				
	     if	(GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==1)button_state=0;
				
				 };break; 		
			
			
			
			}
				

		
	
	//----
	if(flags.adc_ready==1)
	{
	flags.adc_ready=0;
	adc=adc_data;
	input_number_unsigned32( (unsigned int)adc,0x0048 );
	
	}
	
	
if(Codeur_total>4)
{
		temp=Codeur_total*2;
pace= (signed short)temp+pace;
Codeur_total=0;
if(pace>1000)pace=1000;
DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);
input_number_unsigned16( (unsigned short)pace,0x004c );
}

if(Codeur_total<-4)
{
	temp=Codeur_total*2;
pace= (signed short)temp+pace;
Codeur_total=0;
if(pace<0)pace=0;
DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, 4*(unsigned short)pace);

input_number_unsigned16( (unsigned short)pace ,0x004c);
}

}


//************************************************
void calibration_task(void) { // Kalibrasyon
	

		
switch(	calibration_task_state)
{
	case 0 : calibration_task_state_0(); break; //hangi kanal secildi GIT DENETLE
	case 1 : calibration_task_state_1(); break; // CH1
	case 2 : calibration_task_state_2(); break; //	CH2
  case 3 : calibration_task_state_3(); break; // CH3 
	case 4 : calibration_task_state_4();break;//KALIBRASYON CALISIRKEN*****


}	
}






void genel_task_test_state_0(void) // ilk bosta kalan task okuyoruz eepromu ve yaziyoruz acilista
{

if(paket_geldi==1)
{
paket_geldi=0; 
		
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
	if(	vp_genel==0)
	{
	switch(password)
	{
		case 1: {// cube 
			
			genel_task_test_state=1;
			cube_1.a_diff=read_u16_eeprom(cube_sample_a_adress );
			cube_1.b_diff=read_u16_eeprom(cube_sample_b_adress );
			cube_1.c_diff=read_u16_eeprom(cube_sample_c_adress );
			
			input_number_unsigned16( cube_1.a_diff,0x0002);
			input_number_unsigned16( cube_1.b_diff,0x0004);
			input_number_unsigned16( cube_1.c_diff,0x0006);
			
			goto_pic(13); 
		
		       };break; 
		//--
		case 2: {// cylinder
		genel_task_test_state=2;
		
			cylinder_1.diameter_diff=read_u16_eeprom(cylinder_sample_d_adress);
			cylinder_1.height_diff=read_u16_eeprom(cylinder_sample_h_adress);
		
			input_number_unsigned16( cylinder_1.diameter_diff,0x0002);
			input_number_unsigned16( cylinder_1.height_diff,0x0004);		
			goto_pic(15); 
		       };break; 
		
					 
//				case 3: {// beam 3 
//		
//		genel_task_test_state=3;
//		       };break; 
//						case 4: {// beam 3 
//		genel_task_test_state=4;
//		
//		       };break; 
		case 5: {// exit 
		
			goto_pic(1); 
		
		press_task_state=0;
		       };break; 
		
			}
	
	}
	



}


}

//------

void genel_task_test_state_1(void)// cube 
{

if(paket_geldi==1)
{
paket_geldi=0; 

		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
		
	
	if(vp_genel==0)
	{
	switch(password)
	{
		case 1: {
		genel_task_test_state=5;
			goto_pic(28);
cube_1.a=40;
cube_1.b=40;
cube_1.c=40;		
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		     };break; 
	//--------------------------
		case 2: {
				genel_task_test_state=5;
			goto_pic(28);
cube_1.a=50;
cube_1.b=50;
cube_1.c=50;		
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		
		     };break; 				 
	//---
		case 3: {
				genel_task_test_state=5;
			goto_pic(28);
cube_1.a=70;
cube_1.b=70;
cube_1.c=70;		
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		
		     };break; 				 
	//---
				 
case 4: {
				genel_task_test_state=5;
			goto_pic(28);
cube_1.a=100;
cube_1.b=100;
cube_1.c=100;		
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		
		     };break; 
//---
		case 5: {
				genel_task_test_state=5;
			goto_pic(28);
cube_1.a=150;
cube_1.b=150;
cube_1.c=150;		
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		
		     };break; 				 
	//---
		case 6: {
				genel_task_test_state=5;
			goto_pic(28);
cube_1.a=200;
cube_1.b=200;
cube_1.c=200;		
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		
		     };break; 	
	//---
		case 7: {  //different
				genel_task_test_state=5;
			goto_pic(28);
cube_1.a=cube_1.a_diff; 
cube_1.b=cube_1.b_diff;
cube_1.c=cube_1.c_diff;		
cube_1.area=  (unsigned int)(cube_1.a_diff)* (unsigned int)(cube_1.b_diff); 
		
		     };break; 
		case 8: {// Exit 
			goto_pic(7); 
		genel_task_test_state=0;
		     };break; 			
		
	
	}
	
	
	}
	else if (vp_genel==2) //A yazildi {}
	
	{
	
	cube_1.a_diff=password;
	write_u16_eeprom(cube_1.a_diff,cube_sample_a_adress );
	}
	//*****************
		else if (vp_genel==4) //B yazildi {}
	
	{
	
	cube_1.b_diff=password;
	write_u16_eeprom(cube_1.b_diff,cube_sample_b_adress );
	}
			
	////***************
	else if (vp_genel==6) //C yazildi {}
	
	{
	
	cube_1.c_diff=password;
	write_u16_eeprom(cube_1.c_diff,cube_sample_c_adress );
	}
	


}




}


//--

void genel_task_test_state_2(void)// cylinder
{

if(paket_geldi==1)
{
  paket_geldi=0; 
	
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
   
if(vp_genel==0)
	{
	switch(password)
	{
		case 1: {
		genel_task_test_state=6;
			goto_pic(28);
cylinder_1.diameter=50;
cylinder_1.height=100;
cylinder_1.cylinder_area=((unsigned int)(cylinder_1.diameter)* (unsigned int)(cylinder_1.diameter)*3.14159265359)/4;
		     };break; 
	//--------------------------
		case 2: {
				genel_task_test_state=6;
			goto_pic(28);
cylinder_1.diameter=100;
cylinder_1.height=100;
cylinder_1.cylinder_area=((unsigned int)(cylinder_1.diameter)* (unsigned int)(cylinder_1.diameter)*3.14159265359)/4;		
		     };break; 				 
	//---
		case 3: {
				genel_task_test_state=6;
			goto_pic(28);
cylinder_1.diameter=150;
cylinder_1.height=300;
cylinder_1.cylinder_area=((unsigned int)(cylinder_1.diameter)* (unsigned int)(cylinder_1.diameter)*3.14159265359)/4;		
		     };break; 				 
	//---
				 
case 4: { //different
				genel_task_test_state=6;
		    goto_pic(28);
cylinder_1.diameter=cylinder_1.diameter_diff;
cylinder_1.height=cylinder_1.height_diff;
cylinder_1.cylinder_area=((unsigned int)(cylinder_1.diameter_diff)* (unsigned int)(cylinder_1.diameter_diff)*3.14159265359)/4;
		     };break; 
//---
case 5: { //back
			
				goto_pic(7); 
				genel_task_test_state=0;
		     };break; 				 
	//---
		 	}	
}
}
else if (vp_genel==2) // Diameter yazildi 
	{	
	cylinder_1.diameter_diff=password;
	write_u16_eeprom(cylinder_1.diameter_diff,cylinder_sample_d_adress  );
	}
	//*****************
else if (vp_genel==4) // Height yazildi 
	{
	
	cylinder_1.height_diff=password;
	write_u16_eeprom(cylinder_1.height_diff,cylinder_sample_h_adress  );
	}


}
//----*******************************************************************************************************
void genel_task_test_state_5(void) {  //TEST EKRANI CUBE

	if(paket_geldi==1)
{
	 //send_time_info();
	 paket_geldi=0; 

	
	
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
	 //keypad_number=((unsigned char )keypad_3<<24)|((unsigned char )keypad_2<<16)|((unsigned char )keypad_1<<8)|((unsigned char )keypad_0);

if(vp_genel==0)
	{
			
	switch(password)
	{
			case 1: {  ////// BACK FROM TEST SCREEN
				
				genel_task_test_state=1;
				goto_pic(13);
			};break;
			
			case 2: {  // START'A BASILINCA		 
				goto_pic(30);	
				GPIO_SetBits(buzzer_GPIO_PORT , buzzer);
				delay_main=0;
				while(delay_main<50);
			 	GPIO_ResetBits(buzzer_GPIO_PORT , buzzer); 
			
			};break;
				case 3: {  // STOP'A BASILINCA
					goto_pic(28);
				GPIO_SetBits(buzzer_GPIO_PORT , buzzer);
				delay_main=0;
				while(delay_main<50);
			 	GPIO_ResetBits(buzzer_GPIO_PORT , buzzer); 
					
			};break;
				case 4: {  // AUTO BASILINCA
	
			};break;
				case 5: {  // MAN BASILINCA
				
		
			};break;
			
	
	}
		
	}
	
	
	
	}
}

//----
void genel_task_test_state_6(void) {  //TEST EKRANI CYLINDER

if(paket_geldi==1)
{
paket_geldi=0; 
		
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 

if(vp_genel==0)
	{
			
	switch(password)
	{
			case 1: {  ////// BACK FROM TEST SCREEN
				
				genel_task_test_state=2;
				goto_pic(15);	
			};break;
			
				case 2: {  // START'A BASILINCA		 
					goto_pic(30);	
				GPIO_SetBits(buzzer_GPIO_PORT , buzzer);
				delay_main=0;
				while(delay_main<50);
			 	GPIO_ResetBits(buzzer_GPIO_PORT , buzzer); 
			};break;
				case 3: {  // STOP'A BASILINCA
					goto_pic(28);
				GPIO_SetBits(buzzer_GPIO_PORT , buzzer);
				delay_main=0;
				while(delay_main<50);
			 	GPIO_ResetBits(buzzer_GPIO_PORT , buzzer); 
			};break;
				case 4: {  // AUTO BASILINCA
	
			};break;
				case 5: {  // MAN BASILINCA
			
			};break;
		
	}	
	}
 }
}


//----
void genel_task_test(void) // TEST BLOGU ALTINA GIRENLER
{
	
	
switch(	genel_task_test_state)
{
	case 0 : genel_task_test_state_0(); break; 
	case 1 : genel_task_test_state_1(); break; // cube 
	case 2 : genel_task_test_state_2(); break;//cyclinder
//case 3 : genel_task_test_state_3(); break; // beam 3  
//case 4 : genel_task_test_state_4(); break;//  beam 4
	case 5 : genel_task_test_state_5(); break;//  TEST SAYFASI CUBE
	case 6 : genel_task_test_state_6(); break;//  TEST SAYFASI CYLINDER
}	


}

///---



void key_check_ana(void) 
{
	
	
//	password_low
//password_high
//vp_low
//vp_high
	if( paket_geldi==1)
	{
		paket_geldi=0;
		
		vp_genel=((unsigned short )vp_high<<8)|(unsigned short)vp_low; 
		password=((unsigned short )password_high<<8)|(unsigned short)password_low; 
		
	if(	vp_genel==0)
	{
		
		if(password==1)
		{
	goto_pic(7); 
			genel_task_test_state=0;
		press_task_state=test_task_pointer;
		}
	
	}
	else if (	vp_genel==2)
	{
	
	
	}
	
		else if (	vp_genel==4)
	{
		
		if(	password== 1234)
		{
		goto_pic(9); 
		press_task_state=calibration_task_pointer;	
			
	  }
	}
	
		else if (vp_genel==6)
	{
	
		{
		
		if(	password== 1234)
		{
		goto_pic(18); 
		press_task_state=settings_task_pointer;	
			
	  }
	}
		
	
	}
	}




}


//--------------------------
void ana_task(void)
{
key_check_ana(); 


}

//--

//--MAIN FUNCTION 
void indukator(void) 
{

	switch(press_task_state)
	{
		case 0: ana_task(); break; 
		
	 case test_task_pointer : genel_task_test();break;
//	 case admin_task_pointer :  admin_task();break;
	 case calibration_task_pointer : calibration_task(); break;
	 case settings_task_pointer: setting_task();break; 
	}

}

