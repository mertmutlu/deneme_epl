#include "main.h"
#include "extern.h"
#include "25LC512.h"
#include "ads1232.h"
#include "lcd.h"
#include "fire.h"




unsigned char test_begin( unsigned char test_type, bool oto_man)
{
unsigned char error; 
error=0;

if(test_type==test_type_cube )
{

press.intercept	=0.1; 
press.slope=0.002;
press.break_load=0;
press.load_kn=0;
press.max_load=0;
press.period=0;
press.speed_kn=speed_kn;
press.stress=0;
press.tare=5;
press.break_100=90;
press.load_start=100; 
press.start_hz=50;	
	
	

}
else if(test_type==test_type_cylinder )
{



}
else if(test_type==test_type_beam_3 )
{



}
else if(test_type==test_type_beam_4 )
{



}
else
{

return error; 
}
	

//---

while(1)
{













}
	
	





}