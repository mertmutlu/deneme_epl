#include "main.h"
#include "extern.h"
#include "25LC512.h"
#include "ads1232.h"
#include "lcd.h"
#include "fire.h"



void button_3_check(void)
{
static unsigned char state=0; 
	
	switch (state)
	{
	
		case 0: {
							if(  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
							{

state=1; 
button_timer=0; 								


									}
		
		       };break; 
		//-----------------------------------------------------------
		case 1:
		{
							if(  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
							{

								
								if(button_timer>=button_period)
								{
								
										state=3; 
								flags.cube_c_task=1;
								}


									}
							
									else
									{
									
									state=0; 
									
									}
			
			
			
		}break; 
//-----------
		
		case 3: 
		{
			if(  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==1)
							{
	            state=0; 
							}
		
		};break; 

						 	
	}			

}

//----

void button_2_check(void)
{
static unsigned char state=0; 
	
	switch (state)
	{
	
		case 0: {
							if(  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==0)
							{

state=1; 
button_timer=0; 								


									}
		
		       };break; 
		//-----------------------------------------------------------
		case 1:
		{
							if(  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==0)
							{

								
								if(button_timer>=button_period)
								{
								
										state=3; 
								flags.cube_b_task=1;
								}


									}
							
									else
									{
									
									state=0; 
									
									}
			
			
			
		}break; 
//-----------
		
		case 3: 
		{
			if(  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==1)
							{
	            state=0; 
							}
		
		};break; 

						 	
	}			

}
//---

void button_1_check(void)
{
static unsigned char state=0; 
	
	switch (state)
	{
	
		case 0: {
							if(  GPIO_ReadInputDataBit(button_1_GPIO_PORT , button_1)==0)
							{

state=1; 
button_timer=0; 								


									}
		
		       };break; 
		//-----------------------------------------------------------
		case 1:
		{
							if(  GPIO_ReadInputDataBit(button_1_GPIO_PORT , button_1)==0)
							{

								
								if(button_timer>=button_period)
								{
								
										state=3; 
								flags.cube_a_task=1;
								}


									}
							
									else
									{
									
									state=0; 
									
									}
			
			
			
		}break; 
//-----------
		
		case 3: 
		{
			if(  GPIO_ReadInputDataBit(button_1_GPIO_PORT , button_1)==1)
							{
	            state=0; 
							}
		
		};break; 

						 	
	}			

}

//----

void cube_task_state1(void)
{


	
switch(password)
{
	case 0: 
	{
	input_number_unsigned16( cube_1.a,0x000C);	
	input_number_unsigned16( cube_1.b,0x000E);	
	input_number_unsigned16( cube_1.c,0x0010);	
	};break;
	
	//---------------------------------------------
	
	case 0x0009: {
cube_1.a=40;
cube_1.b=40;
cube_1.c=40;		
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		
		state_test.cube_task=2;
		
						};break; 
	
//-----------------------------------						
	case 0x000A: {

cube_1.a=50;
cube_1.b=50;
cube_1.c=50;	
		cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		state_test.cube_task=2;
						};break; 
	
	//-----------------------------------						
	case 0x000B: {

cube_1.a=70;
cube_1.b=70;
cube_1.c=70;	
		cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		state_test.cube_task=2;
						};break; 		
//-----------------------------------						
	case 0x000C: {

cube_1.a=100;
cube_1.b=100;
cube_1.c=100;
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 	
state_test.cube_task=2;		
						};break; 
	//-----------------------------------						
	case 0x000D: {
cube_1.a=150;
cube_1.b=150;
cube_1.c=150;	
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		state_test.cube_task=2;
						};break;

//-----------------------------------						
	case 0x000E: {
cube_1.a=200;
cube_1.b=200;
cube_1.c=200;	
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b); 
		state_test.cube_task=2;
						};break; 			
//-----------------------------------						

									
}




if(vp_low==0x0C||vp_low==0x0E||vp_low==0x10){
cube_1.area=  (unsigned int)(cube_1.a)* (unsigned int)(cube_1.b);
	
				switch(vp_low){
	
			case 0x0C: {
			
						flags.cube_a_task =1;

						};break; 
			case 0x0E: {

						flags.cube_b_task =1;
						
						};break; 		
			case 0x10: {
		
						flags.cube_c_task =1;

						};break; 							
}
}
// touch if 
	
//	
//if((flags.cube_a_task==0)||(flags.cube_b_task==0)||(flags.cube_c_task==0))
//{
//button_1_check();
//button_2_check(); 
//button_3_check(); 
//	
//}
//-----------------------------------------	
	
	//		if(password==0x000B){  //Mert
		
	if(flags.cube_a_task==1)
{
  flags.cube_a_task=0;
 //cube_1.a=encoder_input_16(cube_1.a, 0x5000 ,0 , 0xf000,0x0000,500   );
  input_number_unsigned16( cube_1.a,0x000C);
	write_u16_eeprom(cube_1.a,cube_sample_a_adress );

}	

if(flags.cube_b_task==1)
{
	flags.cube_b_task=0;
//cube_1.b=encoder_input_16(cube_1.b, 0x6000 ,2 , 0xf000,0x0000,250   );
	input_number_unsigned16( cube_1.b,0x000E);	
	write_u16_eeprom(cube_1.b,cube_sample_b_adress );

}	
	
	
if(flags.cube_c_task==1)
{
	flags.cube_c_task=0;
//cube_1.c=encoder_input_16(cube_1.c, 0x7000 ,4 , 0xf000,0x0000,250   );
	input_number_unsigned16( cube_1.c,0x0010);
	write_u16_eeprom(cube_1.c,cube_sample_c_adress );

}		
	

//}
}

//---------------------------
void cube_task(void)
{
	goto_pic(13);
	switch(state_test.cube_task)
	{
		case 0:  ///------ initiate cube task 
		{
			
			cube_1.a=read_u16_eeprom(cube_sample_a_adress  );
      cube_1.b=read_u16_eeprom(cube_sample_b_adress  );
      cube_1.c=read_u16_eeprom(cube_sample_c_adress  );
			
			
			input_number_unsigned16( cube_1.a,0x000C);	
			input_number_unsigned16( cube_1.b,0x000E);	
			input_number_unsigned16( cube_1.c,0x0010);	
			flags.cube_touch=0;
			state_test.cube_task=1;
			
		};break; 			
	
	//---

		case 1: // choose sample 
		{
			if(flags.cube_touch==1)
			{
			flags.cube_touch=0; 
		  cube_task_state1();
			}
		
		}		break;

//---
		case 2: 
		{
					flags.oto= automatic; 
		    	flags.durum=stop; 
			
			
		test_seconds=0; 
    pace=200;
			HighDensByteRead(&speed_kn,  speed_kn_adress );
  	
		goto_pic(5);
		input_number_unsigned16( 0,0);	
		input_number_unsigned16( 0,2);	
		input_number_unsigned16( speed_kn,4);
  	input_number_unsigned16( test_seconds,6);
  	input_number_unsigned16( pace,8);				
		put_icon( 0,0x00f0 );/// start icon 
		put_icon( 2,0x00f2 );/// auto icon 
		state_test.cube_task=3;
		}break; 			
			
//-----------------------------------
case 3: 
		{
		
		if(	flags.cube_touch==1)
		{
			
			flags.cube_touch=0;
		switch(key_return)
		{
     case  1: // speed change 
      {
speed_kn=encoder_input_16(speed_kn, 0x8000 ,4 , 0xf000,0x0000,30   );
//write_u16_eeprom(speed_kn,speed_kn_adress );
HighDensByteWrite(	speed_kn, speed_kn_adress);
delay_ee();			
       };break;

//--------------------------------------------
case  2: // geri  
      {
					
		  goto_pic(13);
			state_test.cube_task=0;
				
			};break;
			
//---
case  3: // start
      {
					
		  put_icon(1,0x00f0);  // stop
			
		   flags.durum=start; 
				state_test.cube_task=4;
			};break;
						
	case  4: // manula/ oto
      {
					
				if(	flags.oto== automatic)
				{
					flags.oto=manual;
				  put_icon(4,0x00f2);  //manual
				}
				else
				{
					flags.oto=automatic;
				  put_icon(2,0x00f2);  //oto
				
				}
				
		
		
				
			};break;		

		}	}// touch if 
//------------------------------------
		
			if(  GPIO_ReadInputDataBit(button_1_GPIO_PORT , button_1)==0)
			{
			
	speed_kn=encoder_input_16(speed_kn, 0x8000 ,4 , 0xf000,0x0000,30   );
//write_u16_eeprom(speed_kn,speed_kn_adress );
HighDensByteWrite(	speed_kn, speed_kn_adress);
delay_ee();	
			
			
			}
			//-------
				if(  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==0)
			{
			
		if(	flags.oto== automatic)
				{
					flags.oto=manual;
				  put_icon(4,0x00f2);  //manual
				}
				else
				{
					flags.oto=automatic;
				  put_icon(2,0x00f2);  //oto
				
				}
				while((  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==0));
				
				
			}
			
			//---
							if(  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
			{
			
			 put_icon(1,0x00f0);  // stop
			
		   flags.durum=start; 
				state_test.cube_task=4;
			
			
			}
		
		
		}		;break; 	/// state3 : 
		
		//--------
		
		
	case 4:  // start test 
	{
 test_begin( test_type_cube , flags.oto);  // cube 
		
		
		
	};break; 
		
	}
}
//-------------------------------------------------------  CYLINDER
//----------------------------------------------------- CYLINDER
//--------------------------------------------------- CYLINDER
//------------------------------------------------- CYLINDER
//----------------------------------------------- CYLINDER
//--------------------------------------------- CYLINDER
//------------------------------------------- CYLINDER

void cylinder_task_state1(void){

	
if(flags.cylinder_touch==1)
{

flags.cylinder_touch=0; 
	
switch(key_return)
{
	case 0 : 
	{
	input_number_unsigned16( cylinder_1.diameter,0x0028);	
	input_number_unsigned16( cylinder_1.height,0x002A);	
	
	};break;
	
	//---------------------------------------------   
	
	case cylinder50: {
cylinder_1.diameter=50;
cylinder_1.height=100;		
cylinder_1.cylinder_area=   ( (unsigned int)(cylinder_1.diameter)* (unsigned int)(cylinder_1.diameter)*3.14159265359)/4;
		
		state_test.cylinder_task=2;
		
						};break; 
	
//-----------------------------------						
	case cylinder100: {

cylinder_1.diameter=100;
cylinder_1.height=100;		
cylinder_1.cylinder_area=   ( (unsigned int)(cylinder_1.diameter)* (unsigned int)(cylinder_1.diameter)*3.14159265359)/4;
		state_test.cylinder_task=2;
						};break; 
	
	//-----------------------------------						
	case cylinder150: {

cylinder_1.diameter=150;
cylinder_1.height=300;	
	
cylinder_1.cylinder_area= ( (unsigned int)(cylinder_1.diameter)* (unsigned int)(cylinder_1.diameter)*3.14159265359)/4; 
		state_test.cylinder_task=2;
						};break; 	
	
//------------------------------------------	

	case cube_diff: {
		
cylinder_1.cylinder_area= ( (unsigned int)(cylinder_1.diameter)* (unsigned int)(cylinder_1.diameter)*3.14159265359)/4; 
		
		state_test.cylinder_task=2;

						};break; 	
	//------------------
		case 5: {
						flags.cylinder_d_task =1;

						};break; 

	case 6: {

						flags.cylinder_h_task =1;
						};break; 		

}
}

if((flags.cylinder_d_task==0)||(flags.cylinder_h_task==0))
{
button_1_check();
button_2_check(); 
button_3_check(); 
	
}
//--------------------------------------KONTROL ET


if(flags.cylinder_d_task==1)
{

cylinder_1.diameter=encoder_input_16(cylinder_1.diameter, 0x5000, 0x0028 , 0xf000,0x0000,500   );
	write_u16_eeprom(cube_1.a,cube_sample_a_adress );
flags.cylinder_d_task=0;
}	

if(flags.cylinder_h_task==1)
{

cylinder_1.height=encoder_input_16(cylinder_1.height, 0x6000 ,0x002A , 0xf000,0x0000,250   );
		write_u16_eeprom(cylinder_1.height,cube_sample_b_adress );
flags.cylinder_h_task=0;
}	
		

}

//--------------------------------------
void cylinder_task(void){
	
    goto_pic(15);
	switch(state_test.cylinder_task)
	{
		case 0:  ///------ initiate cube task 
		{
			
			cylinder_1.diameter=read_u16_eeprom(cylinder_sample_d_adress);
      cylinder_1.height=read_u16_eeprom(cylinder_sample_h_adress  );

			input_number_unsigned16( cylinder_1.diameter,0x0028);	
			input_number_unsigned16( cylinder_1.height,0x002A);		
			flags.cylinder_touch=0;
			state_test.cylinder_task=1;
			
		};break; 			
	
	//---

		case 1: // choose sample 
		{
		  cylinder_task_state1();
		
		
		}		break;

//---
		case 2: 
		{
					flags.oto= automatic; 
		    	flags.durum=stop; 
			
			
		test_seconds=0; 
    pace=200;
			HighDensByteRead(&speed_kn,  speed_kn_adress );
  	
		goto_pic(5);
		input_number_unsigned16( 0,0);	
		input_number_unsigned16( 0,2);	
		input_number_unsigned16( speed_kn,4);
  	input_number_unsigned16( test_seconds,6);
  	input_number_unsigned16( pace,8);				
		put_icon( 0,0x00f0 );/// start icon 
		put_icon( 2,0x00f2 );/// auto icon 
		state_test.cylinder_task=3;
		}break; 			
			
//-----------------------------------
case 3: 
		{
		
		if(	flags.cylinder_touch==1)
		{
			
			flags.cylinder_touch=0;
		switch(key_return)
		{
     case  1: // speed change 
      {
speed_kn=encoder_input_16(speed_kn, 0x8000 ,4 , 0xf000,0x0000,30   );
//write_u16_eeprom(speed_kn,speed_kn_adress );
HighDensByteWrite(	speed_kn, speed_kn_adress);
delay_ee();			
       };break;

//--------------------------------------------
case  2: // geri  
      {
					
		  goto_pic(3);
			state_test.cylinder_task=0;
				
			};break;
			
//---
case  3: // start
      {
					
		  put_icon(1,0x00f0);  // stop
			
		   flags.durum=start; 
				state_test.cylinder_task=4;
			};break;
						
	case  4: // manula/ oto
      {
					
				if(	flags.oto== automatic)
				{
					flags.oto=manual;
				  put_icon(4,0x00f2);  //manual
				}
				else
				{
					flags.oto=automatic;
				  put_icon(2,0x00f2);  //oto
				
				}
				
		
		
				
			};break;		

		}	}// touch if 
//------------------------------------
		
			if(  GPIO_ReadInputDataBit(button_1_GPIO_PORT , button_1)==0)
			{
			
	speed_kn=encoder_input_16(speed_kn, 0x8000 ,4 , 0xf000,0x0000,30   );
//write_u16_eeprom(speed_kn,speed_kn_adress );
HighDensByteWrite(	speed_kn, speed_kn_adress);
delay_ee();	
			
			
			}
			//-------
				if(  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==0)
			{
			
		if(	flags.oto== automatic)
				{
					flags.oto=manual;
				  put_icon(4,0x00f2);  //manual
				}
				else
				{
					flags.oto=automatic;
				  put_icon(2,0x00f2);  //oto
				
				}
				while((  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==0));
				
				
			}
			
			//---
							if(  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
			{
			
			 put_icon(1,0x00f0);  // stop
			
		   flags.durum=start; 
				state_test.cylinder_task=4;
			
			
			}
		
		
		}		;break; 	/// state3 : 
		
		//--------
		
		
	case 4:  // start test 
	{
 test_begin( test_type_cube , flags.oto);  // cube 
		
		
		
	};break; 
		
	}
}
	



//--------------------------------------------------------
 //  if(flags.test_touch=1){ 
void test_task(void) {
	
	
switch(test_task_state)
{
		case cube_task_state:cube_task();break;
		case cylinder_task_state:cylinder_task();break;
//	case beam_3_task_state: beam_3_task();break;
//	case beam_4_task_state: beam_4_task();break;

}	
	
}
//} 