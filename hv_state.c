#include "stm32f3348_discovery.h"
#include <math.h>
#include "main.h"
#include "extern.h"

//--



void send_check_ok(void)
{
unsigned char temp,temp1;
	temp=(unsigned char) joules;
	temp1=(unsigned char)impedance;

	SendChar1 (0xff) ;
	SendChar1 (command_check_ok) ;
		SendChar1 (command_check_ok) ;
		SendChar1 (command_check_ok) ;
}
void send_shock_info_error(void)
{


	SendChar1 (0xff) ;
	SendChar1 (command_error) ;
	SendChar1 (status_shock) ;
	SendChar1 (status_shock) ;
}

//---


void send_shock_info(void)
{
unsigned char temp,temp1;
	temp=(unsigned char) joules;
	temp1=(unsigned char)impedance;

	SendChar1 (0xff) ;
	SendChar1 (command_info	) ;
	SendChar1 (temp) ;
	SendChar1 (temp1) ;
}
//--
void send_charge_ready(void)
{



	SendChar1 (0xff) ;
	SendChar1 (command_charge_ready) ;
	SendChar1 (command_charge_ready) ;
	SendChar1 (command_charge_ready) ;
}

//--

void send_charge_problem(void)
{



	SendChar1 (0xff) ;
	SendChar1 (command_charge_problem) ;
	SendChar1 (command_charge_problem) ;
	SendChar1 (command_charge_problem) ;
}

//--

void state_0(void)
{
float temp,temp1; 
	unsigned int i; 
//command_flag=1;
//	set_joule=50;
	//command=charge_command;
if(command_flag==1)
{
command_flag=0;
switch(command)
{

	case  charge_command	: {
	
											if(set_joule<max_joule)
											{
												disharge_relay_control(1);
												body_joule=set_joule;
												temp=  1+ (float)error_joule/100;
												
											temp1= (float)set_joule*temp;
												set_joule= (unsigned char )(temp1);
												
											
											
											state=1;
											}
	
											}; break;
//-------------------------
case  check_command	: {
	
	send_check_ok();
	
};break;


//---

case  shock_command_res	: {
	
	disharge_relay_control(0);
	
	
	send_shock_info();
	
};break;

}	


}


}
//---------

void state_1(void)
{	unsigned int i; 
unsigned  short dac_temp;
	float temp,temp1; 
	
	

}

void state_2(void)
{
	unsigned int i; 
 
 
 


}


void state_3(void)
{
//	command_flag=1;
//	command=shock_command;
if(command_flag==1)
{
command_flag=0;
switch(command)
{

	case  shock_command	: {
												disable_charge();
												DAC_SetChannel1Data(DAC2, DAC_Align_12b_R,0); 
												TimingDelay_tick(14400);
												 status_shock=shock_task(body_joule);
		
												if(status_shock>0)state=4;// error 
												else   state=5;   // no error     
	
											}; break;
//---------------------------------------
											
	case  shock_command_res	: {
	disable_charge();
												DAC_SetChannel1Data(DAC2, DAC_Align_12b_R,0);
		
			shock_relay_control(0);
	disharge_relay_control(0);
	
	
	send_shock_info();
	state=0;
};break;

}	


}

}

void state_4(void)
{
shock_relay_control(0);
send_shock_info_error();
disharge_relay_control(0);
state=0;

}


void state_5(void)// no error 
{
		shock_relay_control(0);
send_shock_info();
disharge_relay_control(0);
state=0;

}


void state_6(void)
{


}


//-----


void hv_task(void)
{

switch(state)
{
case 0:  state_0();break;
case 1:  state_1();break;
case 2:  state_2();break;
case 3:  state_3();break;
case 4:  state_4();break;
case 5:  state_5();break;
case 6:  state_6();break;	
}






}