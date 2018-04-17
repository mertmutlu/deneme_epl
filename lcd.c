#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "lcd.h"
#include "extern.h"



  char hi_byte;
  char lo_byte;
	extern int SendChar3 (int ch) ;

//**********

void send_time_info(void){  // seriden saat tarih bilgisini gönderir

 	SendChar3(0xAA);
 	SendChar3(0xAA);
 	SendChar3(0x03);
 	SendChar3(0x81);
 	SendChar3(0x20);
 	SendChar3(0x07);

}

//**********
void send_header(void)
{
SendChar3 (header_1);
SendChar3 (header_2);
}
//----
void send_data_sum( unsigned char sum ) 
{
SendChar3 (sum);

}
//---
void send_command( unsigned char command ) 
{
SendChar3 (command);

}
//--

void send_register(unsigned char address)
{
SendChar3 (address);

}
//---
void send_short(unsigned short  data)
{
	

  
  hi_byte = data/0x100;
  lo_byte = data%0x100;
SendChar3 (hi_byte);
SendChar3 (lo_byte);	

}
//--

void send_int(unsigned int  data)
{
	
unsigned int temp;
 char tx; 
	
	
temp=(data>>24) &0xff; 
tx=(char)temp;
SendChar3 (tx);	

	temp=(data>>16) &0xff; 
tx=(char)temp;
SendChar3 (tx);	
	
	temp=(data>>8) &0xff; 
tx=(char)temp;
SendChar3 (tx);	
	
	temp=(data) &0xff; 
tx=(char)temp;
SendChar3 (tx);	

}



void goto_pic(unsigned short pic_id )
{
	//5a a5 04 80 03 00 01 
	
// 	while(1)
// 	{	
// 	SendChar3(0x55);
// 	}
// 	SendChar3(0x55);
// 	SendChar3(0xAA);
// 	SendChar3(0x04);
// 	SendChar3(0x80);
// 	SendChar3(0x03);
// 	SendChar3(0x00);
// 	SendChar3(0x01);
// 	return ;
send_header();
send_data_sum(0x04);
send_command(write_command);
send_register(pic_id_register);
	
send_short(pic_id);	
}
//--



void set_rtc(unsigned char year, unsigned char month,unsigned char date,unsigned char day , unsigned char hour,unsigned char min,
	unsigned char seconds)
	{

	//A5 5A 0A 80 1F 5A 12 10 25 0412 00 01.
	
	send_header();
send_data_sum(0x0A);
	SendChar3 (0x80);
		SendChar3 (0x1f);
	SendChar3 (0x5a);
		SendChar3 (year);
		SendChar3 (month);
		SendChar3 (date);
		SendChar3 (day);
		SendChar3 (hour);
		SendChar3 (min);
		SendChar3 (second);
	}

void input_number_unsigned16( unsigned short number,unsigned short number_vp )
{


// 5a a5 05 82 0020 0065
// 0020=vp 
// 0065=variavle in he x 
// 82 : write vp 
send_header();
send_data_sum(0x05);
send_command(write_command_sram);
send_short(number_vp);
send_short(number);
}

//-------------
void input_number_unsigned8( unsigned char number,unsigned short number_vp )
{

send_header();
send_data_sum(0x04);
send_command(write_command_sram);
send_short(number_vp);
SendChar3(number);
}
//---

//---


void put_key_popup( unsigned char key ) 
{
	//5A A5 03 80 4F + KEY" 
send_header();
send_data_sum(0x03);
send_data_sum(0x80);
send_data_sum(0x4f);
send_data_sum(key);
}


void put_icon( unsigned short icon_number,unsigned short icon_vp )
{


// 5a a5 05 82 0020 0065
// 0020=vp 
// 0065=variavle in he x 
// 82 : write vp 
send_header();
send_data_sum(0x05);
send_command(write_command_sram);
send_short(icon_vp );
send_short(icon_number);
}

//---


void input_number_unsigned32( unsigned int number,unsigned short number_vp )
{


// 5a a5 05 82 0020 0065
// 0020=vp 
// 0065=variavle in he x 
// 82 : write vp 
send_header();
send_data_sum(0x07);
send_command(write_command_sram);
send_short(number_vp);
send_int(number);
}
//---


//void lcd_date(void)
//{
//unsigned short i; 
//	sprintf(file_name_ecg,"%02d/%02d/%02d",RTC_DateStructure.RTC_Date,RTC_DateStructure.RTC_Month,RTC_DateStructure.RTC_Year );
//	
//	
//send_header();
//send_data_sum(11);//30+1+2=33
//send_command(write_command_sram);
//send_short(date_vp);
//	
//for (i=0;i< 8;i++)
//	{
//		sami_lcd= file_name_ecg[i];
//		SendChar3 (sami_lcd );

//}

////while(1);


//}

//--

void input_text( unsigned short index,unsigned short char_num,unsigned short vp)
{
unsigned short i; 
	
//	lcd_busy=1;
//// 	send_header();
//// send_data_sum(0x0d);
//// send_command(write_command_sram);
//// send_short(0x0008);
//// SendChar3 ('s');
//// SendChar3 ('a');
//// SendChar3 (0x67);
//// SendChar3 (0x67);
//// SendChar3 (0x67);
//// SendChar3 (0x67);
//// SendChar3 (0x67);
//// SendChar3 (0x67);
//// SendChar3 (0x70);
//// SendChar3 ('i');
//// 	return;
//	
//send_header();
//send_data_sum(char_num+3);//30+1+2=33
//send_command(write_command_sram);
//send_short(vp);
//	
//for (i=0;i< char_num;i++)
//	{
//		sami_lcd= setting_buffer[index+i];
//		SendChar3 (sami_lcd );

//}

//lcd_busy=0;	

//}

//void test_text(void)
//{

//send_header();
//send_data_sum(0x0d);
//send_command(write_command_sram);
//send_short(0x0008);
//SendChar3 ('s');
//SendChar3 (0x67);
//SendChar3 (0x67);
//SendChar3 (0x67);
//SendChar3 (0x67);
//SendChar3 (0x67);
//SendChar3 (0x67);
//SendChar3 (0x67);
//SendChar3 (0x70);
//SendChar3 (0x70);
//	
}
//---

//----
void change_color( unsigned short sp, unsigned short color)
{
	unsigned short temp; 
	temp=sp+3; 
send_header();
SendChar3(0x05);
SendChar3(0x82);
send_short(temp); 
send_short(color); 	



}
void reset_graph(void)
{

send_data_sum(11);
send_command(write_command_sram);
send_short(0x0100);		// vp 
send_short(0x0002);		//line command 
//send_short(479);		// number of lines 	
send_short(239);		// number of lines 
	send_short(0x07E0);// green colour 
send_short(0xFF00);	

}
//--
void graph_init(void)
{
	unsigned short i,j;
send_header();
send_data_sum(211);
send_command(write_command_sram);
send_short(0x0100);		// vp 
send_short(0x0002);		//line command 
//send_short(479);		// number of lines 	
	send_short(239);		// number of lines 
send_short(0x07E0);// green colour 
	j=3;
for (i=0;i<50;i++)
{
send_short(2*i);	
send_short(130);
j=j+2;

}

send_short(0xFF00);	

j=0x0100+j;
i=0xffff;
while(i--);
//for (i=50;i<478;i++)
for (i=50;i<239;i++)

{
//----
graph(2*i, 130, j );
j=j+2;



}



// while(1);
// //--------------
// send_header();
// send_data_sum(205);
// send_command(write_command_sram);

// send_short(0x0100+j);		// vp 
// for (i=50;i<100;i++)
// {


// send_short(i);	
// send_short(130);

// 	
// }
// send_short(0xFF00);	



// //---
// send_header();
// send_data_sum(205);
// send_command(write_command_sram);

// send_short(0x0100+j);		// vp 
// for (i=100;i<150;i++)
// {


// send_short(i);	
// send_short(170);

// 	
// }
// send_short(0xFF00);	


}
//--------------------------------------------
void graph(unsigned short x, unsigned short y, unsigned short vp )
{
send_header();
send_data_sum(9);
send_command(write_command_sram);
send_short(vp);
send_short(x);
send_short(y);	
// send_short(x+1);
// send_short(y);		
send_short(0xFF00);	


}
//--
void graph_mod(unsigned short x, unsigned short y, unsigned short vp )
{
send_header();
send_data_sum(7);
send_command(write_command_sram);
send_short(vp);
send_short(x);
send_short(y);	

}

//---

void test_graph(void)

{
	unsigned short x, y ,vp,j,direction; 
graph_init();
	direction=0;
	x=0;
	y=100;

	//graph_index=3;
 vp=0x0103;
	while(1)
	{
		
		
graph_mod(x, y, vp );
		j=0xffff;
		while(j--);
		while(j--);
		while(j--);
		while(j--);
		while(j--);
		while(j--);
		while(j--);
		while(j--);
	vp=vp+2;	 
		x++;
		if(x==479)
		{	
		x=0;
		vp=0x0103;
			if(direction==0)
			{y++;
				if(y==190)
			{
			direction=1;
			}

			}
			else
			{
					y--;
					if(y==70)
			{
			direction=0;
			
				
			}
		
		
		}
	
		
	}
}
}
//--
void test_line(void)
{
send_header();
send_data_sum(0x1B);
send_command(write_command_sram);
send_short(0x0100);
send_short(0x0002);
send_short(0x0009);	
	
send_short(0x07E0);
	
send_short(0);	
send_short(135);
	
send_short(30);	
send_short(100);	
	
send_short(135);
send_short(80);
	
send_short(135);
send_short(135);	
	
send_short(0xFF00);	


//--


send_header();
send_data_sum(19);
send_command(write_command_sram);
send_short(0x1000);
send_short(0x0002);
send_short(0x0001);	
send_short(0xF800);
	
send_short(136);	
send_short(75);
	
send_short(136);	
send_short(190);	
send_short(0xFF00);
//---	
	
	send_header();
send_data_sum(9);
send_command(write_command_sram);
send_short(0x010B);
send_short(200);
send_short(150);			
send_short(0xFF00);	
//------------------------------------
send_header();
send_data_sum(19);
send_command(write_command_sram);
send_short(0x1000);
send_short(0x0002);
send_short(0x0001);	
send_short(0xF800);
	
send_short(201);	
send_short(75);
	
send_short(201);	
send_short(190);	
send_short(0xFF00);

	//---

send_header();
send_data_sum(9);
send_command(write_command_sram);
send_short(0x010D);
send_short(300);
send_short(100);			
send_short(0xFF00);	

send_header();
send_data_sum(19);
send_command(write_command_sram);
send_short(0x1000);
send_short(0x0002);
send_short(0x0001);	
send_short(0xF800);
	
send_short(300);	
send_short(75);
	
send_short(300);	
send_short(190);	
send_short(0xFF00);
//-----------------------------
send_header();
send_data_sum(9);
send_command(write_command_sram);
send_short(0x010F);
send_short(350);
send_short(135);			
send_short(0xFF00);	


send_header();
send_data_sum(19);
send_command(write_command_sram);
send_short(0x1000);
send_short(0x0002);
send_short(0x0001);	
send_short(0xF800);
	
send_short(351);	
send_short(75);
	
send_short(351);	
send_short(190);	
send_short(0xFF00);

//---

send_header();
send_data_sum(9);
send_command(write_command_sram);
send_short(0x0111);
send_short(419);
send_short(120);			
send_short(0xFF00);	

//--
send_header();
send_data_sum(19);
send_command(write_command_sram);
send_short(0x1000);
send_short(0x0002);
send_short(0x0001);	
send_short(0xF800);
	
send_short(420);	
send_short(75);
	
send_short(420);	
send_short(190);	
send_short(0xFF00);

//-------
send_header();
send_data_sum(7);
send_command(write_command_sram);
send_short(0x0103);
send_short(10);
send_short(150);

send_header();
send_data_sum(7);
send_command(write_command_sram);
send_short(0x0105);
send_short(45);
send_short(135);			
//send_short(0xFF00);	

//while(1);
//------------------------
send_header();
send_data_sum(19);
send_command(write_command_sram);
send_short(0x0100+19+27);
send_short(0x0002);
send_short(0x0001);	
send_short(0xF800);
//send_short(0x07E0);	
send_short(137);
send_short(75);	
send_short(137);	
send_short(195);
send_short(0xFF00);

//---
send_header();
send_data_sum(19);
send_command(write_command_sram);
send_short(0x0100+27+19+19);
send_short(0x0002);
send_short(0x0001);	
send_short(0xF800);
//send_short(0x07E0);	
send_short(138);
send_short(75);	
send_short(138);	
send_short(195);
send_short(0xFF00);
}
//--

void set_operating_timer(void)
{
send_header();
send_data_sum(0x0a);
send_command(write_command);
SendChar3 (0x1F);
SendChar3 (0x5A);
SendChar3 (0x13);
SendChar3 (0x11);	
SendChar3 (0x08);
SendChar3 (0x00);
SendChar3 (0x00);	
SendChar3 (0x00);	
SendChar3 (0x00);	
}
//---

void aed_graph(unsigned short y)
{
static unsigned short x=0,z=0; 
unsigned short data,vp;
unsigned short temp;
	
	//data=y;
	if(y>=120)
	{
temp=y-120;
		
		
// 		temp_1= 0.96*(old_y+temp- old_x);   // old_y+ 0.98*(data-old_y);
// 	old_x=temp;
// 	temp=temp_1; 
// 	old_y=temp_1;
		//if(temp<2)temp=0;
		
		temp=temp*3;
		
		if(temp>120)temp=120;
		
	//----


//---		
		
		
		//if(temp>120)temp=120;
		data=temp+120;

	}
	else
	{

		temp=120-y;
		//if(temp<2)temp=0;
		
		
// 		
// 		temp_1= 0.96*(old_y+temp- old_x);   // old_y+ 0.98*(data-old_y);
// 	old_x=temp;
// 	temp=temp_1; 
// 	old_y=temp_1;
// 		
		
		
		temp=temp*3;

		
			if(temp>60)temp=60;
		
		
		
	data=120-temp;;	
	}
	
	//y[i] := y[i-1] + a * (x[i] - y[i-1])low
	//y[i] := a * (y[i-1] + x[i] - x[i-1])// high
// 	temp= old_y+ 0.98*(data-old_y);
// 	data=temp; 
// 	old_y=temp;
	

	data=data/2;
	
	//---
	
// 	if(data >old)
// 	{
// 	if((data-old)<3)data=old;
// 		else old=data;

// 	}
// 	else
// 	{
// if((old-data)<3)data=old;
// 		else old=data;

// 	}
	
	//--
	
	data=230-data; 
//	old_y=data;
//data=(data+old_x)/2;
//	old_x=old_y;
	
	//----
	
// 	if(data>=110)
// 	{
// 	temp=data-110;
// 		if(temp<10)data=110;

// 	}
// 	
// 	else
// 	{
// temp=110-data;
// 		if(temp<10)data=110;


// }
	///----
	vp= (2*x)+ 0x0103;
	graph_mod(z, data, vp );
	x++;
	
	z=z+2;
	//	x++;
	if(x==239)
	{	
	x=0;
		z=0;
	}
}
