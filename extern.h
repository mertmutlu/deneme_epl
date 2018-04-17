
#include "main.h"
#include "relay.h"

#include <stdbool.h >

//---- eeprom adresss ; 

//#define cube_sample_a_adress          0x0000
//#define cube_sample_b_adress          0x0002
//#define cube_sample_c_adress          0x0004


#define cube_sample_a_adress              0x0000
#define cube_sample_b_adress              cube_sample_a_adress+sizeof(cube_1.a_diff)
#define cube_sample_c_adress              cube_sample_b_adress+sizeof(cube_1.b_diff)
#define speed_kn_adress                   cube_sample_c_adress+sizeof(cube_1.c_diff)
	
#define cylinder_sample_d_adress          speed_kn_adress+sizeof(speed_kn)
#define cylinder_sample_h_adress	        cylinder_sample_d_adress+2
	

#define calibration_channel_1_real_adress                 cylinder_sample_h_adress+2
#define calibration_channel_1_analog_adress 	            calibration_channel_1_real_adress +24  // 48 byte 

#define calibration_channel_2_real_adress                 calibration_channel_1_analog_adress + 48
#define calibration_channel_2_analog_adress 	            calibration_channel_2_real_adress   +24  // 48 byte 

#define calibration_channel_3_real_adress 	              calibration_channel_2_analog_adress 	 + 48
#define calibration_channel_3_analog_adress 	            calibration_channel_3_real_adress	 +24  // 48 byte 

#define settings_ch_1_adress	                            calibration_channel_3_analog_adress	 + 48
#define settings_ch_2_adress                              settings_ch_1_adress	 + 14 
#define settings_ch_3_adress                              settings_ch_2_adress	 + 14
#define sensor_1_adress																  	settings_ch_3_adress   + 14
#define sensor_2_adress																	  sensor_1_adress        + 2
#define sensor_3_adress																		sensor_2_adress				 + 2

 
//#define calibration_channel_1_real_adress                 speed_kn_adress +sizeof(speed_kn_adress)
//#define calibration_channel_1_analog_adress 	            calibration_channel_1_real_adress +24  // 48 byte 

//#define calibration_channel_2_real_adress                 calibration_channel_1_analog_adress + 48
//#define calibration_channel_2_analog_adress 	            calibration_channel_2_real_adress   +24  // 48 byte 

//#define calibration_channel_3_real_adress 	              calibration_channel_2_analog_adress 	 + 48
//#define calibration_channel_3_analog_adress 	            calibration_channel_3_real_adress	 +24  // 48 byte 

//#define settings_ch_1_adress	                            calibration_channel_3_analog_adress	 + 48
//#define settings_ch_2_adress                              settings_ch_1_adress	 + 14 
//#define settings_ch_3_adress                              settings_ch_2_adress	 + 14
//#define sensor_1_adress																  	settings_ch_3_adress   + 14
//#define sensor_2_adress																	  sensor_1_adress        + 2
//#define sensor_3_adress																		sensor_2_adress				 + 2

//----

//----
#define LED				                       				GPIO_Pin_15
#define LED_GPIO_PORT                    				GPIOC
#define LED_GPIO_CLK                   	 				RCC_AHBPeriph_GPIOC




#define RLY1				                       			GPIO_Pin_11
#define RLY1_GPIO_PORT                    			GPIOA
#define RLY1_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOA

#define RLY2				                       			GPIO_Pin_12
#define RLY2_GPIO_PORT                    			GPIOA
#define RLY2_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOA


#define driver_enable			                       			  GPIO_Pin_9
#define driver_enable_GPIO_PORT                    			GPIOB
#define driver_enable_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOB


#define dip_adc				                       			GPIO_Pin_3
#define dip_adc_GPIO_PORT                    			GPIOA
#define dip_adc_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOA



#define buzzer 		                       			          GPIO_Pin_13
#define buzzer_GPIO_PORT                    			      GPIOC
#define buzzer_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOC


#define button_1 		                       			          GPIO_Pin_15
#define button_1_GPIO_PORT                    			      GPIOB
#define button_1_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOB

#define button_2 		                       			          GPIO_Pin_8
#define button_2_GPIO_PORT                    			      GPIOA
#define button_2_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOA

#define button_3 		                       			          GPIO_Pin_2 // encoder
#define button_3_GPIO_PORT                    			      GPIOA
#define button_3_GPIO_CLK                   	 			RCC_AHBPeriph_GPIOA

//---
//-- eeprom pins 
#define ee_cs			                       			         GPIO_Pin_8 
#define ee_cs_GPIO_PORT                    			       GPIOB
#define ee_cs_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB

#define ee_sck			                       			         GPIO_Pin_5 
#define ee_sck_GPIO_PORT                    			       GPIOB
#define ee_sck_GPIO_CLK                   	 			       RCC_AHBPeriph_GPIOB

#define ee_si			                       			          GPIO_Pin_6 
#define ee_si_GPIO_PORT                    			        GPIOB
#define ee_si_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB


#define ee_so			                       			          GPIO_Pin_7 //input
#define ee_so_GPIO_PORT                    			        GPIOB
#define ee_so_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB

#define max_switch			                       			   GPIO_Pin_14//input
#define max_switch_GPIO_PORT                    			        GPIOB
#define max_switch_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB

#define min_switch			                       			         GPIO_Pin_13//input
#define min_switch_GPIO_PORT                    			        GPIOB
#define min_switch_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB
//---

//-ads1232  pins 

#define ads_dout			                       			            GPIO_Pin_6//input
#define ads_dout_GPIO_PORT                    			        GPIOA
#define ads_dout_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOA

#define ads_sclk			                       			            GPIO_Pin_5//input
#define ads_sclk_GPIO_PORT                    			        GPIOA
#define ads_sclk_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOA

#define ads_a0			                       			            GPIO_Pin_7//input
#define ads_a0_GPIO_PORT                    			        GPIOA
#define ads_a0_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOA


#define ads_pwdn		                       			            GPIO_Pin_12
#define ads_pwdn_GPIO_PORT                    			        GPIOB
#define ads_pwdn_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB



#define ads_speed		                       			            GPIO_Pin_0
#define ads_speed_GPIO_PORT                    			        GPIOB
#define ads_speed_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB

#define ads_g0	                       			             GPIO_Pin_1
#define ads_g0_GPIO_PORT                    			        GPIOB
#define ads_g0_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB

#define ads_g1	                       			             GPIO_Pin_2
#define ads_g1_GPIO_PORT                    			        GPIOB
#define ads_g1_GPIO_CLK                   	 			        RCC_AHBPeriph_GPIOB
#define speed_10_sps                                            0
#define speed_80_sps                                            1

#define    gain_1			  0
#define    gain_2			  1
#define    gain_64			2
#define    gain_128			3

#define    channel_0   0
#define    channel_1   1

#define  power_disable  0
#define  power_enable  1
//---

#define charge_mcu				                			GPIO_Pin_9
#define charge_mcu_GPIO_PORT                    GPIOB
#define charge_mcu_GPIO_CLK                   	RCC_AHBPeriph_GPIOB

//----------------
#define status_mcu				                			GPIO_Pin_8
#define status_mcu_GPIO_PORT                    GPIOB
#define status_mcu_GPIO_CLK                   	RCC_AHBPeriph_GPIOB

//----
#define dac_set				                					GPIO_Pin_4
#define dac_set_GPIO_PORT                       GPIOA
#define dac_set_GPIO_CLK                   	    RCC_AHBPeriph_GPIOA

#define disharge_relay				                					GPIO_Pin_13
#define disharge_relay_GPIO_PORT                         GPIOC
#define disharge_relay_GPIO_CLK                   	    RCC_AHBPeriph_GPIOC

#define shock_relay						                				GPIO_Pin_14
#define shock_relay_GPIO_PORT                         GPIOC
#define shock_relay_GPIO_CLK                   	      RCC_AHBPeriph_GPIOC

///-----


#define phase_1_h				                					  GPIO_Pin_15
#define phase_1_h_GPIO_PORT                         GPIOB
#define phase_1_h_GPIO_CLK                   	     RCC_AHBPeriph_GPIOB


#define phase_1_l				                					  GPIO_Pin_14
#define phase_1_l_GPIO_PORT                         GPIOB
#define phase_1_l_GPIO_CLK                   	     RCC_AHBPeriph_GPIOB


#define phase_2_h				                					  GPIO_Pin_13
#define phase_2_h_GPIO_PORT                         GPIOB
#define phase_2_h_GPIO_CLK                   	     RCC_AHBPeriph_GPIOB


#define phase_2_l				                					  GPIO_Pin_12
#define phase_2_l_GPIO_PORT                         GPIOB
#define phase_2_l_GPIO_CLK                   	      RCC_AHBPeriph_GPIOB


//--
#define high_volt_mcu				                					  GPIO_Pin_5
#define high_volt_mcu_GPIO_PORT                         GPIOA
#define high_volt_mcu_GPIO_CLK                   	      RCC_AHBPeriph_GPIOA

#define cap_volt_mcu				                					  GPIO_Pin_1
#define cap_volt_mcu_GPIO_PORT                          GPIOA
#define cap_volt_mcu_GPIO_CLK                   	      RCC_AHBPeriph_GPIOA


#define current_mcu	  			                					  GPIO_Pin_0
#define current_mcu_GPIO_PORT                         GPIOB
#define current_mcu_GPIO_CLK                   	      RCC_AHBPeriph_GPIOB


#define    capacitor									170.0f     // micro 
#define    max_volt										1600.0f     // micro 
#define    max_dac										2900					
#define    error_joule								10  // % 

#define    max_joule                    250
#define  			charge_command						0xA0 
#define 		 shock_command						  0xA1
#define 		 check_command						  0xA2    // orda misin ? 
#define 		 shock_command_res						  0xA3

#define     command_charge_ready      				0xB0
#define     command_charge_problem      			0xB1
#define     command_info						     			0xB2
#define     command_check_ok						     	0xB3
#define     command_error						        	0xB4


#define 		 max_charge_time				30000//;	30000

 //--- encoder 
 //PINS CODEUR A ET B
#define Codeur_A           GPIO_Pin_0
#define Codeur_A_SOURCE    GPIO_PinSource0
#define Codeur_B           GPIO_Pin_1
#define Codeur_B_SOURCE    GPIO_PinSource1
#define Codeur_GPIO        GPIOA
#define Codeur_RCC         RCC_AHBPeriph_GPIOA   // RCC_AHB1Periph_GPIOA
#define Codeur_AF          GPIO_AF_1
 
//TIMER UTILISE
#define Codeur_TIMER       TIM2
#define Codeur_TIMER_CLK   RCC_APB1Periph_TIM2
#define Codeur_COUNT()     Codeur_TIMER->CNT
 
 //---
  #define  cube40      1  
  #define  cube50      2
  #define  cube70      3
  #define  cube100     4
	#define  cube150     5
	#define  cube200     6
	#define  cube_diff   7  // non standart 

	#define a_cube 26
	#define b_cube 28
	#define c_cube 30


	//----- Mert
	#define  cylinder50      1
	#define  cylinder100     2
	#define  cylinder150     3
	#define  cylinder_diff   4  // non standart 
	//-----
	
	#define test_task_pointer         1 
	#define admin_task_pointer        2 
	#define calibration_task_pointer  3
  #define settings_task_pointer     4   
	#define button_period  150
	#define cube_task_state    1
  #define cylinder_task_state    2  //Mert
	#define beam_3_task_state 3
	#define beam_4_task_state 4
	#define automatic  0
	#define manual   1
	
	#define stop  0
	#define start 1

#define test_type_cube       1
#define test_type_cylinder   2
#define test_type_beam_3     3
#define test_type_beam_4     4

extern unsigned char year,month,day,week_day,hour,minute,second;

extern unsigned char paket_geldi; 
extern unsigned char genel_task_test_state;
extern unsigned char calibration_channel_state;
extern void 	EXTI6_disable(void);
extern void 	EXTI6_Config(void);
extern unsigned short calibration_real[12];
extern unsigned short settings[9];
extern unsigned char sensor;
extern signed int   calibration_analog[12];
extern unsigned char calibration_channel;
extern unsigned char calibration_task_state;
extern unsigned char password_low; 
extern unsigned char password_high;
extern unsigned char keypad_3,keypad_2,keypad_1,keypad_0; 
//extern signed int keypad_3,keypad_2,keypad_1,keypad_0; //***
extern unsigned short password,vp_genel; 
extern signed int keypad_number;
extern volatile signed int adc_data;
extern unsigned char calib_status;
	extern float load_kn, stress; 
	extern void delay_ee(void) ;
	extern signed short  encoder_input_16( signed short number, unsigned short sp ,unsigned short vp , unsigned short color,unsigned short color_start, signed short max    );;
 typedef struct 
{
	bool test_touch;
	bool calibration_touch;
	bool settings_touch;
	bool admin_touch;
	
	bool cylinder_touch;	
	bool cylinder_d_task;
	bool cylinder_h_task;	
	bool cube_touch; 
	bool cube_a_task; 
	bool cube_b_task; 
	bool cube_c_task; 
	bool durum;   // start  / stop 
	bool oto;  // manua_ // aoto
	bool sifre; 
	bool key_analog;
	bool adc_ready;
}system_flag;


///--
 typedef struct 
{
float  load_kn ; 
float   stress; 
float max_load; 
float break_load; 
float slope; 
float intercept; 
float  load_start;  // kn 
float tare ; 
unsigned short period; 
unsigned char start_hz;
unsigned char speed_kn;	
unsigned char  break_100;  
	
	
}test_test;

//---

 typedef struct 
{
float  slope; 
float   intercept; 

	
	
}channels_calib ;


//--

 typedef struct 
{
unsigned char cube_task;
unsigned char cylinder_task; //M	
}states;


 typedef struct 
{
	unsigned short a;
	unsigned short b; 
	unsigned  short c; 
	unsigned int area; 
	unsigned short a_diff;
	unsigned short b_diff;
	unsigned short c_diff;
}cube_sample;




//--
 typedef struct 
{
unsigned short diameter;
unsigned short height; 
unsigned short diameter_diff;
unsigned short height_diff; 
unsigned short cylinder_area;
}cylinder_sample;

 typedef struct 
{
unsigned short a;
unsigned short b;
unsigned short c; 
	
}beam_sample;

//------
extern unsigned char which_channel;
extern volatile int16_t Codeur_count;
extern volatile int32_t Codeur_total;
extern channels_calib  channel_calib_1; 
extern channels_calib  channel_calib_2; 
extern channels_calib  channel_calib_3; 
extern test_test press; 
extern unsigned short test_seconds;  
extern  unsigned short    pace;
extern unsigned char   speed_kn;
extern states state_test;
extern unsigned char test_task_state;
extern unsigned char setting_task_state;
extern unsigned char kanal_setting_task_state;
extern unsigned char date_task_state;

extern unsigned char sensor_ch1;
extern unsigned char sensor_ch2;
extern unsigned char sensor_ch3;
extern cube_sample cube_1; 
extern cylinder_sample cylinder_1; //Mert
extern signed short 	cube_non_a;
extern unsigned  char a_task_state;
extern system_flag  flags;
extern unsigned  char press_task_state;
extern unsigned char usart_3_state, rxdata_3,vp_low,key_return,vp_high,key_return_high,key_return_low;
extern unsigned char calib_channel;
//--
extern volatile unsigned int   hv_task_delay; 
extern volatile unsigned int usart_1_timeout; 
extern unsigned char state_usart1;
extern unsigned 	char rxdata_1;
extern unsigned int phase_1_time;
extern uint16_t  ADC1ConvertedValue , ADC1ConvertedVoltage ;
 extern  uint16_t  ADC1ConvertedValue_before , ADC1ConvertedVoltage_before ;
extern   uint16_t  ADC2ConvertedValue , ADC2ConvertedVoltage ,ADC2ConvertedVoltage_before ;

extern float set_voltage; 
extern unsigned short dac_joule;
extern unsigned char  set_joule,body_joule,status_shock;
extern unsigned char command ;
extern unsigned int command_flag;
extern volatile unsigned int delay_main,button_timer;
extern unsigned int state;
extern float joules;
extern  float impedance , current ,volt; 
//extern float current_reading[ ];
extern unsigned int current_index;
//----
extern volatile unsigned int delay_setting;
extern void CODEUR_Read (void);
extern int SendChar1 (int ch);
extern void close_left_bridge(void);
extern void close_right_bridge(void);
extern void open_right_bridge(void);
extern void open_left_bridge(void);
extern  unsigned char  shock_task(unsigned char  energy);
extern void TimingDelay_tick(unsigned int tick);
extern void hv_task(void);
extern void shock_relay_control(unsigned char status);
extern void disharge_relay_control(unsigned char status);
extern void enable_charge(void);
extern void disable_charge(void);
extern void indukator(void);
extern void genel_task_test(void);
extern void genel_task_calibration(void);
extern void channel_select(void);
extern  void delay_lcd(void);
extern void send_time_info(void);
extern void input_number_unsigned8( unsigned char number,unsigned short number_vp );
extern RTC_DateTypeDef RTC_DateStructure;
extern RTC_TimeTypeDef RTC_TimeStructure;
