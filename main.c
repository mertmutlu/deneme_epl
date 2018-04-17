/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-June-2014
  * @brief   Buck LED Demonstration of STM32F3348-DISCO board
  ******************************************************************************
  * @attention
  *
	
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "extern.h"
#include "25LC512.h"
#include "ads1232.h"
#include "lcd.h"
#include "task_press.h"
#include "fire.h"

//ekledim 
__IO uint8_t showtime[50] = {0};
__IO uint8_t showdate[50] = {0};
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;
static void RTC_Config(void);
unsigned char paket_geldi; 
unsigned char genel_task_test_state;
unsigned char  calibration_channel_state;
unsigned char calib_channel;
extern void calculate_slope(unsigned char channel );

channels_calib  channel_calib_1; 
channels_calib  channel_calib_2; 
channels_calib  channel_calib_3; 
volatile unsigned int delay_setting;
unsigned short calibration_real[12];
unsigned short settings[9];
unsigned char sensor;
signed int   calibration_analog[12];
signed short  encoder_input_16( signed short number, unsigned short sp ,unsigned short vp , unsigned short color,unsigned short color_start, signed short max    );
float load_kn, stress; 

unsigned char year,month,day,week_day,hour,minute,second;
unsigned char calibration_channel;
//--
unsigned char calibration_task_state;
unsigned char setting_task_state;
unsigned char date_task_state;
unsigned char kanal_setting_task_state;
unsigned char password_high; 
unsigned char password_low;
unsigned char	keypad_3,keypad_2,keypad_1,keypad_0; 
//signed int	keypad_3,keypad_2,keypad_1,keypad_0; //***

unsigned short password,vp_genel; 
signed int keypad_number;
unsigned short test_seconds;  
 unsigned short    pace;
 unsigned char   speed_kn;
//---
test_test press; 
cube_sample cube_1; 
cylinder_sample cylinder_1; //Mert

system_flag  flags;
signed short 	cube_non_a;
unsigned  char a_task_state;
unsigned  char press_task_state=0;
unsigned char usart_3_state=0,rxdata_3,vp_low,key_return=0,vp_high,key_return_high,key_return_low;
//--------------
volatile unsigned int   hv_task_delay,button_timer; 
volatile unsigned int usart_1_timeout; 
unsigned char state_usart1;
unsigned char 	rxdata_1;
unsigned int phase_1_time;
float set_voltage; 
unsigned short dac_joule;
unsigned char  set_joule,body_joule,status_shock;
unsigned char command ;
float joules;
volatile unsigned int delay_main;
unsigned char test_task_state;

states state_test;
/* Private typedef -----------------------------------------------------------*/ 
RCC_ClocksTypeDef RCC_Clocks;
GPIO_InitTypeDef GPIO_InitStructure;
HRTIM_CompareCfgTypeDef HRTIM_CompareStructure;
HRTIM_OutputCfgTypeDef HRTIM_TIM_OutputStructure;
HRTIM_TimerCfgTypeDef HRTIM_TimerWaveStructure;
  
static ADC_CommonInitTypeDef ADC_CommonInitStructure;
static ADC_InitTypeDef ADC_InitStructure;
static ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
static NVIC_InitTypeDef    NVIC_InitStructure;

/* Private define ------------------------------------------------------------*/
#define DAC_DHR12RD_Address      0x40007420
#define Min_Intensity            625
#define Max_Intensity            0
#define Min_Voltage              4500
#define Max_Voltage              5500
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//GLOBAL
volatile int16_t Codeur_count;
volatile int32_t Codeur_total;
 
//LOCAL

unsigned char calib_status;
static volatile int16_t Codeur_old;
static volatile int16_t Codeur_actual;
unsigned int command_flag;
//float current_reading[1000];
unsigned int current_index;
static __IO uint32_t TimingDelay = 0, calibration_value = 0,calibration_value_2 = 0;
__IO uint8_t DownUp = FALSE; /* UP DOWN LED intensity */
__IO uint8_t NoWait = TRUE;
__IO uint8_t StateMachine = STATE_OFF;
__IO uint16_t TriangCMP1=0x100;
__IO uint8_t TriangleGeneration = 0;
uint32_t Tempo = 40000;

uint32_t CurrentSenseTab[5] = {280, 240, 200, 360, 320}; /* Current table centered around ~250mA in the Power LED */
static uint32_t SenseTab[5] = {0,0,0,0,0}; /* mA */
uint16_t DitherTab[8][8] = {{0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,1},
                            {0,0,0,1,0,0,0,1},
                            {0,0,1,0,0,1,0,1},
                            {0,1,0,1,0,1,0,1},
                            {0,1,0,1,1,0,1,1},
                            {0,1,1,1,0,1,1,1},
                            {0,1,1,1,1,1,1,1}};

uint16_t REFINT_CAL; /* embedded reference voltage : raw data acquired at 30캜 / VDDA=3.3V and stored by ST production test */
uint16_t  ADC1ReferenceConvertedValue = 0;
__IO uint32_t  ADC1_Channel1_ConvertedValue_IN = 0;
__IO uint32_t  ADC1_Channel1_ConvertedValue_OUT = 0;
uint32_t VIN;
uint32_t VOUT;
__IO uint16_t Keypressed = FALSE; 
__IO uint8_t index1 =0;
__IO float variable = 625;
__IO float Ratio = 2.0;
__IO uint16_t TESTMODE = FALSE;
__IO uint16_t Current_Mode = BUCK;
__IO uint16_t Converter_Mode_Change =1;
uint32_t VOUT_Target = 5000; /* Default Vout target set in mV */
unsigned int multiplier ;
														
float impedance , current ,volt; 
/* Private function prototypes -----------------------------------------------*/
/* Privoltvate functions ---------------------------------------------------------*/
static void BuckInit(void);
static void DMA_Config(void);
static void DAC_Config(void);
static void COMP4_Config(void);
static void HRTIM_Config(void);
static void ADC_Config(void);
static void TriangleGenerationInit(void);
void HRTIM_SetBurstCompare(float BurstCompare);
static void TestProgram(void);
void SetHRTIM_BuckMode(void);
void SetHRTIM_BoostMode(void);
static void HRTIM_Unselect_OutputTIMx(void);

/**
* @brief   Main program
* @param  None
* @retval None
*/
unsigned int state; 
//******************************************************************************
 uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0;
  uint16_t  ADC1ConvertedValue_before = 0, ADC1ConvertedVoltage_before = 0;
  uint16_t  ADC2ConvertedValue = 0, ADC2ConvertedVoltage = 0,ADC2ConvertedVoltage_before = 0;
// From http://forums.arm.com/index.php?showtopic=13949
 
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR        = (volatile unsigned int *)0xE000EDFC; //address of the register

volatile signed int adc_data;
 
 float adc1_volt; 
 float adc2_volt;
//******************************************************************************
signed int ads_data; 

void DAC_Config(void)
{
  DAC_InitTypeDef  DAC_InitStructure;
  

  /* Configure DAC1 OUT1 ******************************************************/ 
  /* GPIO Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* Configure PA4 (DAC1_OUT1) as analog */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* DAC1 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC1, ENABLE);
  
  /* DAC1 deinitialize */
  DAC_DeInit(DAC1);
  
  /* Fill DAC InitStructure */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_Buffer_Switch = DAC_BufferSwitch_Disable  ; // DAC_BufferSwitch_Enable;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  /* DAC Channel1 Init */
  DAC_Init(DAC1, DAC_Channel_1, &DAC_InitStructure);
//	DAC_Init(DAC2, DAC_Channel_1, &DAC_InitStructure);
  /* Enable DAC Channel1 */
  DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);
  
  /* Set DAC Channel1 DHR register: DAC1_OUT */
 // DAC_SetChannel1Data(DAC1, DAC_Align_12b_R, (uint16_t)CurrentSenseTab[0]);  
}


 void EXTI6_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  
  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger =   EXTI_Trigger_Falling ;// EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//---
 void EXTI6_disable(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  
  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger =   EXTI_Trigger_Falling ;// EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}


//---
void ads1232_test(void)
{
	unsigned int i,counter; 
	counter=0;
	ads_set_speed( speed_10_sps);
	ads_pdwn(power_enable);
	ads_set_gain(gain_1 );
	ads_channel(channel_0);
	
	 ads_cal();
  for(i=0;i<0xffff;i++);
	
	
EXTI6_Config(); 
	while(1);
	
	while(1)
	{
	while(  GPIO_ReadInputDataBit(ads_dout_GPIO_PORT , ads_dout)==1);
		//ads_data=ads_read();
	ads_data=ads_read_18();
	counter++; 
	
if(counter==2)
{
input_number_unsigned32( ads_data,0 );
counter=0; 
GPIOA->ODR ^= RLY1;	

}	
//	for(i=0;i<0xff;i++);
	
	
	}
	
	

}	

int SendChar1 (int ch)  {
	


  while (!(USART1->ISR & USART_FLAG_TXE));
  USART1->TDR  = (ch & 0x1FF);

  return (ch);
}

//--

int SendChar3 (int ch)  {
	


  while (!(USART3->ISR & USART_FLAG_TXE));
  USART3->TDR  = (ch & 0x1FF);

  return (ch);
}

//---



void usart3_init(void)
{



NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	
	/* GPIOA clock enable */
	
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	  
	/* GPIOA Configuration:  USART2 TX on PA2 + PA3= rx2*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
		


		


	/* Connect USART2 pins to AF2 */ 
	// TX = PA2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);
	
	
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx    ;
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE); // enable USART2

	
	//---
	  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
		// this is used to configure the NVIC (nested vector interrupt controller)
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;                             // we want to configure the USART1 interrupts
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;            // this sets the priority group of the USART1 interrupts
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                        // this sets the subpriority inside the group
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                 // the USART1 interrupts are globally enabled
    NVIC_Init(&NVIC_InitStructure);                                                                 // the properties are passed to the NVIC_Init function which takes care of the low level stuff    
    
  
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  

}

//--

void usart1_init(void)
{



NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* enable peripheral clock for USART2 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	
	/* GPIOA clock enable */
	
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);

	  
	/* GPIOA Configuration:  USART2 TX on PA2 + PA3= rx2*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
		


		


	/* Connect USART2 pins to AF2 */ 
	// TX = PA2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
	
	
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx    ;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE); // enable USART2
	
	
	//---
	  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
		// this is used to configure the NVIC (nested vector interrupt controller)
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                             // we want to configure the USART1 interrupts
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;            // this sets the priority group of the USART1 interrupts
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                        // this sets the subpriority inside the group
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                 // the USART1 interrupts are globally enabled
    NVIC_Init(&NVIC_InitStructure);                                                                 // the properties are passed to the NVIC_Init function which takes care of the low level stuff    
    
  
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  

}


//---


//--
 unsigned char  shock_task(unsigned char  energy)
{
	 unsigned char  status;
	
return status; 
}

 
 //--
void EnableTiming(void)
{
    static int enabled = 0;
 
    if (!enabled)
    {
        *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
        *DWT_CYCCNT = 0; // reset the counter
        *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
 
        enabled = 1;
    }
}

//*********************
 void delay_lcd(void)
{

unsigned short i;
	for(i=0;i<100;i++)
	{
	__nop();
	}

}
 
//******************************************************************************
 
void TimingDelay_tick(unsigned int tick)//  i tick = 1/72000 000 ; so 1 sec = tick = 72000 000 
{
//    unsigned int start, current;
// 
//    start = *DWT_CYCCNT;
// 
//    do
//    {
//        current = *DWT_CYCCNT;
//    } while((current - start) < tick);
}


//--
void init_ports(void)
	
{
GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef  DAC_InitStructure;

	
 RCC_AHBPeriphClockCmd(RLY1_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = RLY1	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RLY1_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(RLY1_GPIO_PORT , RLY1);
	
	 RCC_AHBPeriphClockCmd(RLY2_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = RLY2	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RLY1_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(RLY2_GPIO_PORT , RLY2);
	 //------------------------------------------------
	  RCC_AHBPeriphClockCmd(driver_enable_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = driver_enable	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(driver_enable_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(driver_enable_GPIO_PORT , driver_enable);
	
	
	//---
	
	  RCC_AHBPeriphClockCmd(dip_adc_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = dip_adc	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(dip_adc_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(dip_adc_GPIO_PORT , dip_adc);
	 
	 
	 
	
		  RCC_AHBPeriphClockCmd(buzzer_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = buzzer	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(buzzer_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(buzzer_GPIO_PORT , buzzer);
	


//---

		  RCC_AHBPeriphClockCmd(button_1_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = button_1	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(button_1_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(button_1_GPIO_PORT , button_1);
	 
	 
	   RCC_AHBPeriphClockCmd(button_2_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = button_2	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(button_2_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(button_2_GPIO_PORT , button_2);
	 
	 
	    RCC_AHBPeriphClockCmd(button_3_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = button_3	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(button_3_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(button_3_GPIO_PORT , button_3);
	 
	/// eeprom  
	 RCC_AHBPeriphClockCmd(ee_cs_GPIO_CLK , ENABLE);
  GPIO_InitStructure.GPIO_Pin = ee_so	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ee_so_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(ee_so_GPIO_PORT , ee_so);
	 
	 GPIO_InitStructure.GPIO_Pin = ee_cs|ee_sck|ee_si	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ee_so_GPIO_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(ee_cs_GPIO_PORT , ee_cs|ee_sck|ee_si);
	
	
	
	RCC_AHBPeriphClockCmd(max_switch_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = max_switch|min_switch	;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(max_switch_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(max_switch_GPIO_PORT , max_switch|min_switch);
	 
	 
	RCC_AHBPeriphClockCmd(ads_dout_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = ads_dout;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ads_dout_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(ads_dout_GPIO_PORT , ads_dout);
	 
	 
		RCC_AHBPeriphClockCmd(ads_dout_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = ads_sclk|ads_a0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ads_dout_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(ads_dout_GPIO_PORT , ads_sclk|ads_a0);
	 
	 
	 		RCC_AHBPeriphClockCmd(ads_pwdn_GPIO_CLK , ENABLE);
 /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = ads_pwdn|ads_speed	|ads_g0|ads_g1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ads_pwdn_GPIO_PORT, &GPIO_InitStructure);
   GPIO_ResetBits(ads_pwdn_GPIO_PORT , ads_pwdn|ads_speed	|ads_g0|ads_g1);
	  
	 
//	
// RCC_AHBPeriphClockCmd(LED_GPIO_CLK , ENABLE);
// /* Configure the GPIO_LED pin */
//  GPIO_InitStructure.GPIO_Pin = LED	;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
//   GPIO_ResetBits(LED_GPIO_PORT , LED);
//	
//	
////--- shock relay 
//	
// RCC_AHBPeriphClockCmd(shock_relay_GPIO_CLK , ENABLE);
// /* Configure the GPIO_LED pin */
//  GPIO_InitStructure.GPIO_Pin = shock_relay	;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(shock_relay_GPIO_PORT, &GPIO_InitStructure);
//   GPIO_ResetBits(shock_relay_GPIO_PORT , shock_relay	);	
//	
////--


////--- disharge_relay 
//	
// RCC_AHBPeriphClockCmd(disharge_relay_GPIO_CLK , ENABLE);
// /* Configure the GPIO_LED pin */
//  GPIO_InitStructure.GPIO_Pin = disharge_relay;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(disharge_relay_GPIO_PORT, &GPIO_InitStructure);
//   GPIO_ResetBits(disharge_relay_GPIO_PORT , disharge_relay	);	
//	 
//	 
//	 
//	 //-----charge mcu 
//	RCC_AHBPeriphClockCmd(charge_mcu_GPIO_CLK , ENABLE);
// /* Configure the GPIO_LED pin */
//  GPIO_InitStructure.GPIO_Pin = charge_mcu;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(charge_mcu_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_ResetBits(charge_mcu_GPIO_PORT , charge_mcu);	
//	
//	
//	
//		 //-----status_mcu
//	RCC_AHBPeriphClockCmd(status_mcu_GPIO_CLK , ENABLE);
// /* Configure the GPIO_LED pin */
//  GPIO_InitStructure.GPIO_Pin = status_mcu;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(status_mcu_GPIO_PORT, &GPIO_InitStructure);
//  //GPIO_ResetBits(status_mcu_GPIO_PORT , status_mcu);
//	
//	
//	
//	
//	//--- phases
//	
//		 //-----charge mcu 
//	RCC_AHBPeriphClockCmd(phase_1_h_GPIO_CLK , ENABLE);
//  GPIO_InitStructure.GPIO_Pin = phase_1_h;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(phase_1_h_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_ResetBits(phase_1_h_GPIO_PORT , phase_1_h);	
//	
//	
//	//--
//	
//	RCC_AHBPeriphClockCmd(phase_1_l_GPIO_CLK , ENABLE);
//  GPIO_InitStructure.GPIO_Pin = phase_1_l;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(phase_1_l_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_ResetBits(phase_1_l_GPIO_PORT , phase_1_l);


//	RCC_AHBPeriphClockCmd(phase_2_h_GPIO_CLK , ENABLE);
//  GPIO_InitStructure.GPIO_Pin = phase_2_h;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(phase_2_h_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_ResetBits(phase_2_h_GPIO_PORT , phase_2_h);	
//	
//	
//		RCC_AHBPeriphClockCmd(phase_2_l_GPIO_CLK , ENABLE);
//  GPIO_InitStructure.GPIO_Pin = phase_2_l;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(phase_2_l_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_ResetBits(phase_2_l_GPIO_PORT , phase_2_l);
//	
//	
//	//----
//	

 

	
	
  
  /* Set DAC Channel1 DHR register: DAC1_OUT */
 
	 
}



//---



//---
void enable_charge(void)
{
  GPIO_SetBits(charge_mcu_GPIO_PORT , charge_mcu);	

}

//--

void disable_charge(void)
{
 GPIO_ResetBits(charge_mcu_GPIO_PORT , charge_mcu);

}	

void disharge_relay_control(unsigned char status)
{

if(status==1) GPIO_SetBits(disharge_relay_GPIO_PORT , disharge_relay	);	
	
else  GPIO_ResetBits(disharge_relay_GPIO_PORT , disharge_relay	);		

}

//--


void shock_relay_control(unsigned char status)
{

if(status==1) GPIO_SetBits(shock_relay_GPIO_PORT , shock_relay	);	
	
else  GPIO_ResetBits(shock_relay_GPIO_PORT , shock_relay	);		

}

//--

void open_left_bridge(void)
{

GPIO_SetBits(phase_1_h_GPIO_PORT , phase_1_h);	
GPIO_SetBits(phase_1_l_GPIO_PORT , phase_1_l);
}

void close_left_bridge(void)
{

GPIO_ResetBits(phase_1_h_GPIO_PORT , phase_1_h);	
GPIO_ResetBits(phase_1_l_GPIO_PORT , phase_1_l);
}

//---

void open_right_bridge(void)
{

GPIO_SetBits(phase_2_h_GPIO_PORT , phase_2_h);	
GPIO_SetBits(phase_2_l_GPIO_PORT , phase_2_l);
}

//---
void close_right_bridge(void)
{

GPIO_ResetBits(phase_2_h_GPIO_PORT , phase_2_h);	
GPIO_ResetBits(phase_2_l_GPIO_PORT , phase_2_l);
}

//---

void TM_Delay_Init(void) {
    RCC_ClocksTypeDef RCC_Clocks;
    
    /* Get system clocks */
    RCC_GetClocksFreq(&RCC_Clocks);
    
    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    multiplier = RCC_Clocks.HCLK_Frequency / 4000000;
}

 
void TM_DelayMicros(uint32_t micros) {
    /* Multiply micros with multipler */
    /* Substract 10 */
    micros = micros * multiplier - 10;
    /* 4 cycles for one loop */
    while (micros--);
}
 
void TM_DelayMillis(uint32_t millis) {
    /* Multiply millis with multipler */
    /* Substract 10 */
    millis = 1000 * millis * multiplier - 10;
    /* 4 cycles for one loop */
    while (millis--);
}
 //--
void ADC_Config(void)
{
 unsigned int i; 
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  
  /* ADC Channel configuration */
  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  
  /* Enable ADC1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  
  
  
  ADC_StructInit(&ADC_InitStructure);
  
  /* Calibration procedure */  
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  
  /* Insert delay equal to 10 탎 */
//  Delay(10);
  for(i=0;i<0x00ffff;i++);
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
  calibration_value = ADC_GetCalibrationValue(ADC1);
  
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;//ADC_ContinuousConvMode_Enable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC1 regular channel2 configuration */ 
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_7Cycles5);
	 ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_181Cycles5  );
 
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));   
}
//--
void ADC_Config_2(void)
{
 unsigned int i; 
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  
  /* ADC Channel configuration */
  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  
  /* Enable ADC1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  /* Configure ADC1 Channel2 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  ADC_StructInit(&ADC_InitStructure);
  
  /* Calibration procedure */  
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  
  /* Insert delay equal to 10 탎 */
//  Delay(10);
  for(i=0;i<0x00ffff;i++);
  ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC2);
  
  while(ADC_GetCalibrationStatus(ADC2) != RESET );
  calibration_value_2 = ADC_GetCalibrationValue(ADC2);
  
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_ContinuousConvMode =  ADC_ContinuousConvMode_Disable	;//ADC_ContinuousConvMode_Enable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  
  /* ADC1 regular channel2 configuration */ 
  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_181Cycles5  );
  
  /* Enable ADC1 */
  ADC_Cmd(ADC2, ENABLE);
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));   
}
//---------
void GPIO_Configuration_encoder(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
 
    // 2 Inputs for A and B Encoder Channels
    GPIO_InitStructure.GPIO_Pin = Codeur_A|Codeur_B;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(Codeur_GPIO, &GPIO_InitStructure);
 
    //Timer AF Pins Configuration
    GPIO_PinAFConfig(Codeur_GPIO, Codeur_A_SOURCE, Codeur_AF);
    GPIO_PinAFConfig(Codeur_GPIO, Codeur_B_SOURCE, Codeur_AF);
 
  
 
}

void CODEUR_Read (void)
{
    Codeur_old = Codeur_actual;
    Codeur_actual = TIM_GetCounter (Codeur_TIMER) ;
    Codeur_count = Codeur_actual - Codeur_old;
    Codeur_total += Codeur_count;
}

void CODEUR_RESET (void)
{
    __disable_irq();
    Codeur_old = 0;
    Codeur_total = 0;
    TIM_SetCounter (Codeur_TIMER, 0);
    CODEUR_Read();
    __enable_irq();
}

void TIMER_Configuration(void)
{
	
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
// 
//    TIM_TimeBaseStructure.TIM_Prescaler = 0;
//  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF; // Maximal
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;//TIM_CKD_DIV1;
//  TIM_TimeBaseStructure.TIM_CounterMode =TIM_EncoderMode_TI12;// TIM_CounterMode_Down  ;//TIM_CounterMode_Up;
// 
//  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	
// 
  // TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
  //TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
 
 // TIM_Cmd(TIM2, ENABLE);

    // set them up as encoder inputs
    // set both inputs to rising polarity to let it use both edges
    //TIM_EncoderInterfaceConfig (Codeur_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig (Codeur_TIMER, TIM_EncoderMode_TI12,  TIM_ICPolarity_Rising ,  TIM_ICPolarity_Falling );
    TIM_SetAutoreload (Codeur_TIMER, 0xffff);
    TIM_PrescalerConfig(Codeur_TIMER, 3, TIM_PSCReloadMode_Immediate );
    // turn on the timer/counters
    TIM_Cmd (Codeur_TIMER, ENABLE);
 
    // Reset
    CODEUR_RESET ();
}


void test_eeprom(void)
{
	 
HighDensByteWrite(0x55, 0x0000);
	delay_main=0;
while(delay_main<100);
	
	
HighDensByteRead(&command, 0x0000);	
	
HighDensByteRead(&command, 0x0001);		
while(1);
}

//-------

void read_mem(void)
{

	//-- cube 
cube_1.a=read_u16_eeprom(cube_sample_a_adress  );
cube_1.b=read_u16_eeprom(cube_sample_b_adress  );
cube_1.c=read_u16_eeprom(cube_sample_c_adress  );




}

//-

signed short  encoder_input_16( signed short number, unsigned short sp ,unsigned short vp , unsigned short color,unsigned short color_start, signed short max    )
{
	
	signed short temp; 

	

	temp=number; 
input_number_unsigned16( temp,vp);	
	Codeur_total =0;
	change_color(sp,color )	; 
	
	
	while  (GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0);
	
	
	
		while(1)
		{
		if(Codeur_total != 0)
		{
			temp=temp+ (signed short) Codeur_total; 
				if(temp>max)temp=max;
				if(temp<0)temp=0;
    	input_number_unsigned16( temp,vp);	
      Codeur_total=0;
		
		}

//----------------------------
if(  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
{	
change_color(sp,color_start )	;
	break; 
}	
}

while((  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0));
return temp; 
}

//-----
int main(void)
{
	
  uint8_t toggle = 0;
  double midvalue = 625;
  uint32_t i;
  float y;
		GPIO_InitTypeDef  GPIO_InitStructure;
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  files (startup_stm32f334x8.s) before to branch to application main. 
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f30x.c file
  */  
  
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100); // 10MS ; 




//    /* Display time Format : hh:mm:ss */
//    sprintf((char*)showtime,"%.2d:%.2d:%.2d",RTC_TimeStructure.RTC_Hours, RTC_TimeStructure.RTC_Minutes, RTC_TimeStructure.RTC_Seconds);
//    /* Display date Format : mm-dd-yy */
//    sprintf((char*)showdate,"%.2d-%.2d-%.2d",RTC_DateStructure.RTC_Month, RTC_DateStructure.RTC_Date, 2000 + RTC_DateStructure.RTC_Year); 
 
  
	
	
	 // SysTick_Config(SystemCoreClock / 1000000);
  /* Allows temporization */
  NoWait = FALSE;
  TM_Delay_Init();
	 delay_main=0;
	EnableTiming();
	init_ports();
	 
		 // Enable RCC codeur
    RCC_AHBPeriphClockCmd(Codeur_RCC, ENABLE);
   
    // Enable Timer 4 clock
    RCC_APB1PeriphClockCmd(Codeur_TIMER_CLK, ENABLE);
		
		  GPIO_Configuration_encoder();
    TIMER_Configuration();// encoder timer 
		
		//test_eeprom();
//	HighDensByteRead(&command, 0x0000);	
//	relay_1( 1);
//	while(1);
DAC_Config();
ADC_Config();
RTC_Config();
usart1_init();
usart3_init();// lcd 
delay_main=0;



 delay_main=0;
    GPIO_SetBits(buzzer_GPIO_PORT , buzzer);
	 while( delay_main<50);
	 GPIO_ResetBits(buzzer_GPIO_PORT , buzzer);

 
 
read_mem(); // 

//ads1232_test();

 goto_pic(0);
 delay_main=0;
while( delay_main<200);
goto_pic(1);
//press_task_state=0;
cube_non_a=0;
Codeur_total=0;
 flags.sifre=0;
 

 
 // write_u32_eeprom(12345,0x0022);
	


//for(i=0;i<12;i++)
//{

// calibration_real[i]=i*100;
//  calibration_analog[i] =i*3000;
//}
//	calculate_slope(1);

//while(1);
	//ads1232_test();
	EXTI6_disable();
	press_task_state=0;
	paket_geldi=0;
	genel_task_test_state=0;
	calibration_task_state=0;
	setting_task_state=0;
	
	DAC_SetChannel1Data(DAC1,DAC_Align_12b_R,4095);
	
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
//***************************************************************************

while(1)// press 
{

	indukator();
	
}
//***************************************************************************
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

 while(1)
 {
 
 ADC_StartConversion(ADC1);
while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    ADC1ConvertedValue_before = ADC_GetConversionValue(ADC1);
    
    /* Compute the voltage */
    ADC1ConvertedVoltage_before = (ADC1ConvertedValue_before *3300)/0xFFF;
	 delay_main=0;
	 	while( delay_main<99);
 }
 
 
//while(1)
//{
//	if( delay_main>99)
//	{
//	 delay_main=0;
//	
//SendChar1 ('S');
//	SendChar1 ('A');
//	SendChar1 ('M');
//	SendChar1 ('I');
//	}
//	


//}
//	
	while(1)
	{
	 CODEUR_Read();
		
			if(  GPIO_ReadInputDataBit(max_switch_GPIO_PORT , max_switch)==0)relay_1( 1);
	else relay_1( 0);
		
		
	if(  GPIO_ReadInputDataBit(min_switch_GPIO_PORT , min_switch)==0)relay_2( 1);
	else relay_2( 0);
	
	if(  GPIO_ReadInputDataBit(button_1_GPIO_PORT , button_1)==0)relay_1( 1);
	else relay_1( 0);
		
		
	if(  GPIO_ReadInputDataBit(button_2_GPIO_PORT , button_2)==0)relay_2( 1);
	else relay_2( 0);
		

if(  GPIO_ReadInputDataBit(button_3_GPIO_PORT , button_3)==0)
{	
	relay_1( 1);
relay_2( 1);
}
	else 
	{	
	relay_2( 0);
	relay_1( 0);
	}			
	
	}

while(1)
{
 
delay_main=0;
while(delay_main<100);
	 relay_1( 1);
	 relay_2( 1);
//buzzer_control(1);
	//relay_driver_enable(1);
delay_main=0;
while(delay_main<100);
relay_1( 0);
relay_2( 0);
	//	buzzer_control(0);
	//relay_driver_enable(0);
}
		
	//	  ADC_Config();
//ADC_Config_2();

//----
command_flag=0;
state_usart1=0;
//usart1_init();
delay_main=0;
/*
while(1)
{
//	if( delay_main>999)
//	{
//	 delay_main=0;
//	
//SendChar1 ('S');
//	SendChar1 ('A');
//	SendChar1 ('M');
//	SendChar1 ('I');
//	}
	
	if(command_flag==1)
	{
	command_flag=0;
	SendChar1 (command);
	SendChar1 (set_joule);
	
	}

}
*/
state=0; 
command_flag=0;







while(1)
{}



  /* Start ADC1 Software Conversion */ 
  //ADC_StartConversion(ADC1);
   // ADC_StartConversion(ADC2);
  /* Infinite loop */
//  while (1)
//  {
//    /* Test EOC flag */
//		
//		  ADC_StartConversion(ADC1);
//    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
//    {
//    }
//		
//		
//    
//    /* Get ADC1 converted data */
//    ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
//    
//    /* Compute the voltage */
//    ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;   
//  ADC_StartConversion(ADC2);

// while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET)
//    {
//    }		
//		
//		    /* Get ADC2 converted data */
//    ADC2ConvertedValue = ADC_GetConversionValue(ADC2);
//    
//    /* Compute the voltage */
//    ADC2ConvertedVoltage = (ADC2ConvertedValue *3300)/0xFFF;   
//  }


			
	//ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_15Cycles);
//			while(1)
//	{

//	// 	TimingDelay_tick(72000000);
//		for(i=0;i<50000;i++)   // 5000 
//		{
//TimingDelay_tick(14400);// 200 usec
//		}
//		GPIOC->ODR ^= LED;
//		
//	}
//	
//	while(1)
//	{  delay_main=0;
//	while(delay_main<999);	
//		//	disharge_relay_control(1);
//		shock_relay_control(1);
//	//GPIO_SetBits(disharge_relay_GPIO_PORT , disharge_relay	);	
//		
//		  delay_main=0;
//	while(delay_main<999);	
//		//	disharge_relay_control(0);
//		shock_relay_control(0);
//		//GPIO_ResetBits(disharge_relay_GPIO_PORT , disharge_relay	);	
//	}


//	
//	 open_right_bridge();
//	 while(1);
	 for(i=0;i<0xfffff;i++);
	  for(i=0;i<0xfffff;i++);
	for(i=0;i<0xfffff;i++);
	  for(i=0;i<0xfffff;i++);
		for(i=0;i<0xfffff;i++);
	  for(i=0;i<0xfffff;i++);	
while(delay_main<999);		
	//shock_relay_control(1);
	// open_left_bridge();
	 
//	 while(1)
//	 {
//	 while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

//    ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
//    
//    /* Compute the voltage */
//    ADC1ConvertedVoltage = (ADC1ConvertedValue*3300)/0xFFF; 
//	 }
  /* Enable the GPIO_LED Clock */
 //shock_relay_control(1);
 
// disharge_relay_control(1);

 //DAC_SetChannel1Data(DAC2, DAC_Align_12b_R, 2850); 1550
  //DAC_SetChannel1Data(DAC2, DAC_Align_12b_R, 2940); 1600
	//2900=1600v
	  //DAC_SetChannel1Data(DAC2, DAC_Align_12b_R, 2900); 
		  
 enable_charge();
 
 //while(1);
 //disable_charge();
 
 for(i=0;i<0x0ffff;i++);
 
 current_index=0;
 while(   GPIO_ReadInputDataBit(status_mcu_GPIO_PORT , status_mcu)); 
 	shock_relay_control(1);
 for(i=0;i<0x0ffff;i++);
 delay_main=0;
while(delay_main<2000);	
		
 disable_charge();
 for(i=0;i<0x0ffff;i++);
//  close_right_bridge();
//open_left_bridge();
 ADC_StartConversion(ADC1);
while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    ADC1ConvertedValue_before = ADC_GetConversionValue(ADC1);
    
    /* Compute the voltage */
    ADC1ConvertedVoltage_before = (ADC1ConvertedValue_before *3300)/0xFFF; 
		
 ADC_StartConversion(ADC2);
while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
  ADC2ConvertedValue = ADC_GetConversionValue(ADC2);
    
    /* Compute the voltage */
    ADC2ConvertedVoltage_before = (ADC2ConvertedValue *3300)/0xFFF;  
			

	 ADC_StartConversion(ADC2);
while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
  ADC2ConvertedValue = ADC_GetConversionValue(ADC2);
    
    /* Compute the voltage */
    ADC2ConvertedVoltage_before = (ADC2ConvertedValue *3300)/0xFFF;  

 open_right_bridge();
TimingDelay_tick(14400);
//TimingDelay_tick(14400);
//(14400);
//TimingDelay_tick(14400);
//TimingDelay_tick(14400);


 ADC_StartConversion(ADC1);
while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
    
		ADC1ConvertedVoltage = (ADC1ConvertedValue*3300)/0xFFF; 
		


 ADC_StartConversion(ADC2);
while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
  ADC2ConvertedValue = ADC_GetConversionValue(ADC2);
    
    /* Compute the voltage */
    ADC2ConvertedVoltage = (ADC2ConvertedValue *3300)/0xFFF; 

volt=(float)ADC2ConvertedVoltage*640/1000;
	current =  ((float)ADC1ConvertedVoltage_before-(float)ADC1ConvertedVoltage)/40;  // 0.04/1000 current sensor 
	
impedance= volt /current;



// TimingDelay_tick(14400);
//  TimingDelay_tick(14400);
// TimingDelay_tick(14400);
//   
  // ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
    
   
	
//		
//		    /* Get ADC2 converted data */
//    ADC2ConvertedValue = ADC_GetConversionValue(ADC2);
//     /* Compute the voltage */
//    ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;   
//    /* Compute the voltage */
//    ADC2ConvertedVoltage = (ADC2ConvertedValue *3300)/0xFFF;  
// delay_main=0;
//while(delay_main<8);	


for(i=0;i<40;i++)
{
	TimingDelay_tick(14400);
 ADC_StartConversion(ADC1);
while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
    
		ADC1ConvertedVoltage = (ADC1ConvertedValue*3300)/0xFFF; 
current =  ((float)ADC1ConvertedVoltage_before-(float)ADC1ConvertedVoltage)/40;  // 0.04/1000 current sensor 
	

	
}

 close_left_bridge();
close_right_bridge();
	TimingDelay_tick(14400);
TimingDelay_tick(14400);
	 open_left_bridge();
	 
//	  delay_main=0;
//while(delay_main<10);	

for(i=0;i<50;i++)
{
	TimingDelay_tick(14400);
 ADC_StartConversion(ADC1);
while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
    
		ADC1ConvertedVoltage = (ADC1ConvertedValue*3300)/0xFFF; 
current =  ((float)ADC1ConvertedVoltage_before-(float)ADC1ConvertedVoltage)/40;  // 0.04/1000 current sensor 
	
	
current_index++;
	
}
close_left_bridge();
close_right_bridge();
 	disharge_relay_control(0);
// while(1)
// {
// 
//  disable_charge();
// for(i=0;i<0xfffff;i++);
//  enable_charge();
// for(i=0;i<0xfffff;i++);
//	 
// }

joules=0;
for(i=0;i<90;i++)
{
	

	
joules=joules+current;
}

joules=joules*impedance/4510; // 4518hz aprroz 200us 
	while(1);
	
	
	
	while(1)
	{
	
	
	
	
	
	
	
	}
	
	//--------------------
	
  /* Initialize LEDs available on STM32F3348-DISCO */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  
  /* Initialize User_Button on STM32F3348-DISCO */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);   
  
  /* Check if TestProgram is called by User button at startup*/
  if((STM_EVAL_PBGetState(BUTTON_USER) == SET))
  {
    while((STM_EVAL_PBGetState(BUTTON_USER) == SET))
    {
      /* Toggle LED5/LED6 to show TestProgram selection until key pressed */
      STM_EVAL_LEDOn(LED6);
      Delay(7000);
      STM_EVAL_LEDOff(LED6);
      STM_EVAL_LEDOn(LED5);
      Delay(7000);
      STM_EVAL_LEDOff(LED5);       
    } 
    STM_EVAL_LEDOff(LED6);   
    STM_EVAL_LEDOff(LED5);
    /* Jump to TestProgram function */
    TESTMODE = TRUE;
    TestProgram();
  }
  
  /* Initializes HRTIM, DAC, DMA and COMP to drive Buck LED*/
  BuckInit();
  
  /* Initializes HRTIM to generate a triangle signal on PB14 */
  TriangleGenerationInit();
  
  /* Infinite loop */
  while (1)
  { 
    switch (StateMachine)
    {
      
    case STATE_MAN:
      /* Disable HRTIM Master Burst period interrupt */
      HRTIM_ITCommonConfig(HRTIM1, HRTIM_IT_BMPER, DISABLE);
      /* Reset all signal LEDs */
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED4);
      STM_EVAL_LEDOff(LED5);
      STM_EVAL_LEDOff(LED6); 
      
      TriangleGeneration=0;
      /* Disable the TD1 output */
      HRTIM_WaveformOutputStop(HRTIM1, HRTIM_OUTPUT_TD1);
      
      /* By keeping pressed User push-button, intensity can be adjusted manually */
      /* Alternatively the LED intensity is increased or decreased */
      /* Always starts with low intensity value for first time */ 
      /* Decrease LED intensity */
      while((STM_EVAL_PBGetState(BUTTON_USER) == SET) && (DownUp==TRUE))
      {
        /* Display Decrease LED */
        STM_EVAL_LEDOn(LED6);
        STM_EVAL_LEDOff(LED3);
        /* Temporization active */
        NoWait = FALSE;
        HRTIM_SetBurstCompare(midvalue);
        midvalue++;
        Delay(1000);
        /* Check if min. intensity reached */
        if (midvalue >= Min_Intensity)
        {
          midvalue = Min_Intensity;
          /* Then blink display LED min. intensity */
          STM_EVAL_LEDOff(LED6);
          Delay(10000);
        }
      }
      /* Increase LED intensity */
      while((STM_EVAL_PBGetState(BUTTON_USER) == SET) && (DownUp==FALSE))
      { 
        /* Display Increase LED */
        STM_EVAL_LEDOff(LED6);
        STM_EVAL_LEDOn(LED3);        
        /* Temporization active */
        NoWait = FALSE;
        HRTIM_SetBurstCompare(midvalue);
        midvalue--;
        Delay(1000);
        /* Check if max. intensity reached */
        if (midvalue <= Max_Intensity)
        {
          midvalue = Max_Intensity;
          /* Then blink display LED max. intensity */
          STM_EVAL_LEDOff(LED3);
          Delay(10000);
        }
      }
      
      break; 
      
    case STATE_OFF:
      /* This mode disables LED dimmer or flash */
      /* Disable HRTIM Master Burst period interrupt */
      HRTIM_ITCommonConfig(HRTIM1, HRTIM_IT_BMPER, DISABLE);
      DownUp = TRUE;
      midvalue = 625;
      /* BUCK LED is OFF, only signal LEDs are blinking */
      HRTIM_SetBurstCompare(Min_Intensity);
      /* Define initial blinking time between two LEDs 400ms */
      NoWait = FALSE;
      /* Red LED ON */
      STM_EVAL_LEDOn(LED3);
      Delay(Tempo);
      STM_EVAL_LEDOff(LED3);
      /* Green LED ON */
      STM_EVAL_LEDOn(LED5);
      Delay(Tempo);
      STM_EVAL_LEDOff(LED5);
      /* Blue LED ON */
      STM_EVAL_LEDOn(LED6);
      Delay(Tempo);    
      STM_EVAL_LEDOff(LED6);
      /* Orange LED ON */
      STM_EVAL_LEDOn(LED4);
      Delay(Tempo);
      STM_EVAL_LEDOff(LED4);
      
      TriangleGeneration=1;
      /* Enable the TD1 output */
      HRTIM_WaveformOutputStart(HRTIM1, HRTIM_OUTPUT_TD1);
      
      /* Decrease time between blinking sequence */
      Tempo = Tempo /2;
      if(Tempo==1)
      {
        Tempo = 40000;
      }
      
      break; 
      
    case STATE_DIMMER:
      
      /* The LED intensity is automatically controlled */
      /* Levels of current thresholds are progressively set */
      /* Intensity variation and flicker removal are enhanced thanks to dither sequences 
         that allows greater precision by steps */
      DownUp = TRUE;
      midvalue = 625;
      /* Reset all signal LEDs */
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED4);
      STM_EVAL_LEDOff(LED5);
      STM_EVAL_LEDOff(LED6); 
      
      TriangleGeneration=0;
      /* Disable the TD1 output */
      HRTIM_WaveformOutputStop(HRTIM1, HRTIM_OUTPUT_TD1);
      
      NoWait =FALSE;
      /* Set signal LEDs to rising mode */
      STM_EVAL_LEDOn(LED3);
      STM_EVAL_LEDOff(LED6);
      /* Start with current thresholds set to minimum */
      SenseTab[0]= 80;
      SenseTab[1]= 40;
      SenseTab[2]= 0;
      SenseTab[3]= 160;
      SenseTab[4]= 120;
      CurrentSenseTab[0]= 0;
      CurrentSenseTab[1]= 0;
      CurrentSenseTab[2]= 0;
      CurrentSenseTab[3]= 0;
      CurrentSenseTab[4]= 0;
      /* Set variable at minimum intensity */
      variable = Min_Intensity;
      /* Enable HRTIM Master Burst period interrupt */
      /* Each time the interrupt is serviced, the LED is driven */
      HRTIM_ITCommonConfig(HRTIM1, HRTIM_IT_BMPER, ENABLE);
      /* Intensity changes from Min to Max */
      for(variable = Min_Intensity; variable >= Max_Intensity && NoWait == FALSE;variable-=0.1)
      {
        /* Increment progressively all thresholds up to the setting value (centered value is ~250mA)*/
        if(variable > 605 + 0.1)
        {
          for (i=0;i<5;i++)
          {
            SenseTab[i] ++;
            CurrentSenseTab[i] = SenseTab[i] * 4096 / 3300;
          }
        }
        /* Delay between 2 steps is decreased exponentially */
        y = ((variable/100) * 1.1);
        y = exp((double) y);
        Delay((uint32_t) y);
      }
      
      /* Set signal LEDs to falling mode */
      /* Intensity falling phase */
      STM_EVAL_LEDOn(LED6);
      STM_EVAL_LEDOff(LED3);
      /* Intensity changes from Max to Min */
      for(variable = Max_Intensity; variable <= Min_Intensity && NoWait == FALSE;variable+=0.1)
      {
        /* Decrement progressively all thresholds down to 0*/
        if(variable > 605)
        {
          for (i=0;i<5;i++)
          {
            SenseTab[i] --;
            CurrentSenseTab[i] = SenseTab[i] * 4096 / 3300;
          }
        }
        /* Delay between 2 steps is increased exponentially */
        y = ((variable/100) * 1.1);
        y = exp((double) y);
        Delay((uint32_t) y);
      }      
      
      break;
      
    case STATE_FLASH:
      /* Flash mode consists to drive the LED @10Hz frequency (20% time ON) at maximum intensity */
      /* Disable HRTIM Master Burst period interrupt */      
      HRTIM_ITCommonConfig(HRTIM1, HRTIM_IT_BMPER, DISABLE);
      /* (Re)Initialization of Current thresholds */
      DownUp = TRUE;
      midvalue = 625;
      /* Reset all signal LEDs except green*/       
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED6);
      /* Alternatively signals LED4 and LED5 are toggled */ 
      if (toggle)
      {
        STM_EVAL_LEDOn(LED4);
        STM_EVAL_LEDOff(LED5);
      }
      else
      {
        STM_EVAL_LEDOff(LED4);
        STM_EVAL_LEDOn(LED5);
      }
      
      TriangleGeneration=0;
      /* Disable the TD1 output */
      HRTIM_WaveformOutputStop(HRTIM1, HRTIM_OUTPUT_TD1);
      
      NoWait =FALSE; 
      /* freq= 10Hz tON = 20ms; tOFF = 80ms */
      /* BUCK LED ON */
      HRTIM_SetBurstCompare(Max_Intensity);
      /* tON = 20ms */
      Delay(2000);
      /* Alternatively signals LED4 and LED5 are toggled */ 
      if (toggle)
      {
        STM_EVAL_LEDOff(LED4);
        STM_EVAL_LEDOn(LED5);
      }
      else
      {
        STM_EVAL_LEDOn(LED4);
        STM_EVAL_LEDOff(LED5);
      }     
      
      /* BUCK LED OFF */
      HRTIM_SetBurstCompare(Min_Intensity);
      /* tOFF = 80 ms */
      Delay(8000);
      toggle =!toggle;
    }
  }
}


/**
* @brief  Configures Buck
* @param  None
* @retval None
*/
static void BuckInit(void)
{  
  /* Convert current sense value thresholds for DAC */
  uint8_t Index;
  for (Index=0;Index<5;Index++)
  {
    CurrentSenseTab[Index] = CurrentSenseTab[Index] * 4096 / 3300; 
  }
  
  /* DMA1 Configuration */
  DMA_Config();
  
  /* DAC1 Configuration */
  DAC_Config();
  
  /* COMP4 Configuration */
  COMP4_Config();
  
  /* HRTIM TIMC Configuration */
  HRTIM_Config(); 
}

/**
* @brief  DMA configuration
* @param  None
* @retval None
*/
static void DMA_Config(void)
{
  DMA_InitTypeDef  DMA_InitStructure;  
  /* Enable DMA1 clock -------------------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12RD_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&CurrentSenseTab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 5;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  
  /* Enable DMA1 Channel5 */
  DMA_Cmd(DMA1_Channel5, ENABLE);
}



/**
* @brief  COMP4 configuration
* @param  None
* @retval None
*/
static void COMP4_Config(void)
{
  COMP_InitTypeDef COMP_InitStructure;
  
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* GPIOB Peripheral clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
  
  /* Configure PB0 in analog mode: PB0 is connected to COMP4 non inverting input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
#ifdef DEBUG  
  /* COMP4 output config: PB1 for debug */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;  
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Alternate function configuration : Comparator4 Out2 / PB1 */ 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_8);  
#endif
  
  /*  COMP4 deinitialize */
  COMP_DeInit(COMP_Selection_COMP4);  
  
  /* COMP4 config */
  COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;
  COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_DAC1OUT1;
  COMP_InitStructure.COMP_Output = COMP_Output_None;
  COMP_InitStructure.COMP_OutputPol = COMP_OutputPol_NonInverted;
  COMP_InitStructure.COMP_Hysteresis = COMP_CSR_COMPxHYST;
  COMP_InitStructure.COMP_Mode = COMP_Mode_HighSpeed; 
  COMP_InitStructure.COMP_BlankingSrce = COMP_BlankingSrce_None; 
  COMP_Init(COMP_Selection_COMP4, &COMP_InitStructure);
  
  /* Enable COMP4 */
  COMP_Cmd(COMP_Selection_COMP4, ENABLE);   
}

/**
* @brief  HRTIM TIMC configuration
* @param  None
* @retval None
*/

//****************** RTC
static void RTC_Config(void)
{
  RTC_InitTypeDef RTC_InitStructure;
  RTC_AlarmTypeDef RTC_AlarmStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* Enable PWR APB1 Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  /* Allow access to Backup */
  PWR_BackupAccessCmd(ENABLE);
  
//  /* Reset RTC Domain */
//  RCC_BackupResetCmd(ENABLE);
//  RCC_BackupResetCmd(DISABLE);
  
  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);
  
  /* The RTC Clock may varies due to LSI frequency dispersion */   
  /* Enable the LSI OSC */ 
 // RCC_LSICmd(ENABLE);
  RCC_LSEConfig(RCC_LSE_ON); ///external oldugu icin
  /* Wait till LSI is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }
  
  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);
  
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();
  
  /* RTC prescaler configuration */
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStructure.RTC_AsynchPrediv = 127;
  RTC_InitStructure.RTC_SynchPrediv = 255;
  RTC_Init(&RTC_InitStructure);
  
  /* Set the alarm 01h:00min:04s */
//  RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
//  RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = 0x01;
//  RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 0x00;
//  RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0x04;
//  RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
//  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
  /* Alarm mask hour, min and second:default Alarm generation each 1s */
//  RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_All; 
//  RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
//  
  /* Enable RTC Alarm A Interrupt */
//  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
//  
//  /* Enable the alarm */
//  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
  
  /* Set the date: Wednesday February 5th 2014 */
//  RTC_DateStructure.RTC_Year = 16;
//  RTC_DateStructure.RTC_Month = RTC_Month_August;
//  RTC_DateStructure.RTC_Date = 22;
//  RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Tuesday;
//  RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
//  
//  /* Set the time to 01h 00mn 00s AM */
//  RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
//  RTC_TimeStructure.RTC_Hours   = 0x01;
//  RTC_TimeStructure.RTC_Minutes = 0x00;
//  RTC_TimeStructure.RTC_Seconds = 0x00; 
//  RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);    
  
  RTC_ClearFlag(RTC_FLAG_ALRAF);
  

}


//*****************

static void HRTIM_Config(void)
{
  HRTIM_EventCfgTypeDef HRTIM_ExternalEventStructure;
  HRTIM_BaseInitTypeDef HRTIM_BaseInitStructure;
  HRTIM_CompareCfgTypeDef HRTIM_CompareStructure;
  HRTIM_BurstModeCfgTypeDef HRTIM_BurstStructure; 
  
  /* Configure HRTIM TIM C ****************************************************/  
  /* HRTIM output channel configuration : HRTIM_CHC1 (Buck drive) / PB12 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  /* Alternate function configuration : HRTIM_CHC1 / PB12 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_13);
  
  /* Use the PLLx2 clock for HRTIM */
  RCC_HRTIM1CLKConfig(RCC_HRTIM1CLK_PLLCLK);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_HRTIM1, ENABLE);
  
  /* DMA Configuration */
  HRTIM_DMACmd(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_DMA_REP, ENABLE);
  HRTIM_DMACmd(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_DMA_CMP1, ENABLE);
  HRTIM_DMACmd(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_DMA_CMP2, ENABLE);
  HRTIM_DMACmd(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_DMA_CMP3, ENABLE);
  HRTIM_DMACmd(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_DMA_CMP4, ENABLE);
  
  /* HRTIM initialization startup */   
  HRTIM_DLLCalibrationStart(HRTIM1, HRTIM_CALIBRATIONRATE_14);
  while((HRTIM_GetCommonFlagStatus(HRTIM1,HRTIM_FLAG_DLLRDY)) == RESET)
  {
  }
  
  /* Configure the output features */  
  HRTIM_TIM_OutputStructure.Polarity = HRTIM_OUTPUTPOLARITY_HIGH; 
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_TIMPER;  
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTRESET_EEV_2; 
  HRTIM_TIM_OutputStructure.IdleMode = HRTIM_OUTPUTIDLEMODE_IDLE;  
  HRTIM_TIM_OutputStructure.IdleState = HRTIM_OUTPUTIDLESTATE_INACTIVE;          
  HRTIM_TIM_OutputStructure.FaultState = HRTIM_OUTPUTFAULTSTATE_NONE;          
  HRTIM_TIM_OutputStructure.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;        
  HRTIM_TIM_OutputStructure.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &HRTIM_TIM_OutputStructure);
  
  /* Configure HRTIM1_TIMC Deadtime */
  HRTIM_TimerWaveStructure.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  HRTIM_TimerWaveStructure.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  HRTIM_TimerWaveStructure.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  HRTIM_TimerWaveStructure.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  HRTIM_TimerWaveStructure.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  HRTIM_TimerWaveStructure.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE; 
  HRTIM_TimerWaveStructure.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  HRTIM_TimerWaveStructure.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_C; 
  HRTIM_WaveformTimerConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, &HRTIM_TimerWaveStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = 0; 
  HRTIM_CompareStructure.CompareValue = 3686; /* 20% time */
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &HRTIM_CompareStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = 0; 
  HRTIM_CompareStructure.CompareValue = 7373; /* 40% time */
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_2, &HRTIM_CompareStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = 0; 
  HRTIM_CompareStructure.CompareValue = 11059; /* 60% time */
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_3, &HRTIM_CompareStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = 0; 
  HRTIM_CompareStructure.CompareValue = 14746; /* 80% time */
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_4, &HRTIM_CompareStructure);  
  
  HRTIM_BaseInitStructure.Period = 18432; /* 1 period = 4 탎 = 100% time */
  HRTIM_BaseInitStructure.RepetitionCounter = 0x00;
  HRTIM_BaseInitStructure.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32; 
  HRTIM_BaseInitStructure.Mode = HRTIM_MODE_CONTINOUS;          
  HRTIM_SimpleBase_Init(HRTIM1, HRTIM_TIMERINDEX_TIMER_C, &HRTIM_BaseInitStructure);
  
  /* Configure External Event Source 2 */
  HRTIM_ExternalEventStructure.Source = HRTIM_EVENTSRC_2; 
  HRTIM_ExternalEventStructure.Polarity = HRTIM_EVENTPOLARITY_HIGH; 
  HRTIM_ExternalEventStructure.Sensitivity = HRTIM_EVENTSENSITIVITY_LEVEL;
  HRTIM_ExternalEventStructure.FastMode = HRTIM_EVENTFASTMODE_ENABLE;
  HRTIM_EventConfig(HRTIM1, HRTIM_EVENT_2, &HRTIM_ExternalEventStructure);
  
  /* Burst Mode configuration */
  HRTIM_BurstStructure.Mode = HRTIM_BURSTMODE_CONTINOUS;
  HRTIM_BurstStructure.ClockSource = HRTIM_BURSTMODECLOCKSOURCE_TIMER_C; 
  HRTIM_BurstStructure.Prescaler = HRTIM_BURSTMODEPRESCALER_DIV1;
  HRTIM_BurstStructure.PreloadEnable = HRIM_BURSTMODEPRELOAD_ENABLED;
  HRTIM_BurstStructure.Trigger =  HRTIM_BURSTMODETRIGGER_SOFTWARE; 
  HRTIM_BurstStructure.IdleDuration = Min_Intensity;
  HRTIM_BurstStructure.Period = Min_Intensity;  
  HRTIM_BurstModeConfig(HRTIM1, &HRTIM_BurstStructure);
  
  /* Enable the TC1 output */
  HRTIM_WaveformOutputStart(HRTIM1, HRTIM_OUTPUT_TC1); 
  
  HRTIM_BurstModeCtl(HRTIM1, HRTIM_BURSTMODECTL_ENABLED);
  
  /* Start slave*/
  HRTIM_WaveformCounterStart(HRTIM1, HRTIM_TIMERID_TIMER_C);
  
  /* Select Burst Trigger */
  HRTIM1->HRTIM_COMMON.BMTRGR = HRTIM_BURSTMODETRIGGER_SOFTWARE;
  
  /* Configure and enable HRTIM interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = HRTIM1_Master_IRQn;   	 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable Burst mode period completed interrupt */
  HRTIM_ITCommonConfig(HRTIM1, HRTIM_IT_BMPER, ENABLE);
}

/**
* @brief  Set HRTIM to generate a triangle on channel D1
* @param  None
* @retval None
*/
static void TriangleGenerationInit(void)
{
  HRTIM_OutputCfgTypeDef  HRTIM_TIM_OutputStructure;
  HRTIM_BaseInitTypeDef   HRTIM_BaseInitStructure;
  HRTIM_TimerCfgTypeDef   HRTIM_TimerWaveStructure;
  HRTIM_CompareCfgTypeDef HRTIM_CompareStructure;
  
  /* GPIOB Peripheral clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
  /* HRTIM output channel configuration : HRTIM_CHD1 / PB14 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  /* Alternate function configuration : HRTIM_CHD1 / PB14 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_13);
  
  /* Configure the output features */  
  HRTIM_TIM_OutputStructure.Polarity = HRTIM_OUTPUTPOLARITY_HIGH; 
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_TIMPER;  
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1; 
  HRTIM_TIM_OutputStructure.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;  
  HRTIM_TIM_OutputStructure.IdleState = HRTIM_OUTPUTIDLESTATE_INACTIVE;          
  HRTIM_TIM_OutputStructure.FaultState = HRTIM_OUTPUTFAULTSTATE_NONE;          
  HRTIM_TIM_OutputStructure.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;        
  HRTIM_TIM_OutputStructure.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TC1, &HRTIM_TIM_OutputStructure);
  
  /* Configure HRTIM1_TIMD Deadtime */
  HRTIM_TimerWaveStructure.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  HRTIM_TimerWaveStructure.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  HRTIM_TimerWaveStructure.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  HRTIM_TimerWaveStructure.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  HRTIM_TimerWaveStructure.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  HRTIM_TimerWaveStructure.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  HRTIM_TimerWaveStructure.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  HRTIM_TimerWaveStructure.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_D; 
  HRTIM_WaveformTimerConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_D, &HRTIM_TimerWaveStructure);
  
  HRTIM_BaseInitStructure.Period = 0x900; /* 2MHz => 2304 = 0x900 (timer input clock freq is 144MHz x 32)*/
  HRTIM_BaseInitStructure.RepetitionCounter = 0x00;
  HRTIM_BaseInitStructure.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;  
  HRTIM_BaseInitStructure.Mode = HRTIM_MODE_CONTINOUS;          
  HRTIM_SimpleBase_Init(HRTIM1, HRTIM_TIMERINDEX_TIMER_D, &HRTIM_BaseInitStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = 0; /* meaningless in regular mode */
  HRTIM_CompareStructure.CompareValue = TriangCMP1;
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, &HRTIM_CompareStructure);
  
  /* Enable the TD1 output */
  HRTIM_WaveformOutputStart(HRTIM1, HRTIM_OUTPUT_TD1); 
  
  /* Start slave timer D*/
  HRTIM_WaveformCounterStart(HRTIM1, HRTIM_TIMERID_TIMER_D);
  
}

/**
* @brief  Set Burst Compare value
* @param  BurstCompare: Burst Compare value, it can be a value between 0x0 and 0xFFFF 
* @retval None
*/
void HRTIM_SetBurstCompare(float BurstCompare)
{
  /* Set Burst Compare value */
  HRTIM1_COMMON->BMCMPR = (uint16_t)BurstCompare; 
}

/**
* @brief  Checks the Discovery Kit features.
* @param  None
* @retval None
*/
static void TestProgram(void)
{  
  HRTIM_BaseInitTypeDef    HRTIM_BaseInitStructure; 
  HRTIM_DeadTimeCfgTypeDef HRTIM_TIM_DeadTimeStructure;
  HRTIM_TimerInitTypeDef   HRTIM_TimerInitStructure;
  HRTIM_OutputCfgTypeDef   HRTIM_TIM_OutputStructure;
  HRTIM_CompareCfgTypeDef  HRTIM_CompareStructure;
  
  uint32_t SetPeriod = 18432; /*Period equivalent to 250kHz frequency */
  uint32_t DutyCycleTimA = 2500; /*Buck Boost duty cycle Timer A initialization */
  uint32_t BlankTimeTimA = 500; /*Blank Time between alternate BuckBoost Timer A initialization */
  uint32_t MinTimeTimA = 500; /*Min time corresponding to DutyCycle Max = 97% */
  uint32_t ADCTriggerValue = 1000; /*ADC trigger Event time where noise is not present */
   
  /* Use the PLLx2 clock for HRTIM */
  RCC_HRTIM1CLKConfig(RCC_HRTIM1CLK_PLLCLK);
  
  /* Enable the HRTIM, SYSCFG and GPIOs (port A, B, C) Clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_HRTIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* Configure H bridge BuckBoost converter N and P MOS output config */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
#ifdef Debug  
  /* For Debug only */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
#endif
  
  /* Initialize User_Button on STM32F3348-DISCO */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
    
  /* Configure HRTIM drivers PA8/PA9/PA10/PA11 HRTIM_CHA1/A2/B1/B2 */
  /* Alternate function configuration : HRTIM_CHA1/A2/B1/B2 / PA8-9-10-11 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_13); /* A */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_13); /* B */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_13); /* C */ 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_13); /* D */
  
  /* HRTIM intialisation startup */  
  HRTIM_DLLCalibrationStart(HRTIM1, HRTIM_CALIBRATIONRATE_14);
  while((HRTIM_GetCommonFlagStatus(HRTIM1,HRTIM_FLAG_DLLRDY)) == RESET)
  {
  }
  
  /* Configure the output features */  
  HRTIM_TIM_OutputStructure.Polarity = HRTIM_OUTPUTPOLARITY_HIGH; 
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_TIMPER;  
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1; 
  HRTIM_TIM_OutputStructure.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;  
  HRTIM_TIM_OutputStructure.IdleState = HRTIM_OUTPUTIDLESTATE_INACTIVE;          
  HRTIM_TIM_OutputStructure.FaultState = HRTIM_OUTPUTFAULTSTATE_NONE;          
  HRTIM_TIM_OutputStructure.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;        
  HRTIM_TIM_OutputStructure.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &HRTIM_TIM_OutputStructure);
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &HRTIM_TIM_OutputStructure);

  /* HRTIM1_TIMA and HRTIM1_TIMB Deadtime enable */
  HRTIM_TimerWaveStructure.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  HRTIM_TimerWaveStructure.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  HRTIM_TimerWaveStructure.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  HRTIM_TimerWaveStructure.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  HRTIM_TimerWaveStructure.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  HRTIM_TimerWaveStructure.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE; 
  HRTIM_TimerWaveStructure.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  HRTIM_TimerWaveStructure.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE; 
  HRTIM_WaveformTimerConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, &HRTIM_TimerWaveStructure);
  HRTIM_WaveformTimerConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, &HRTIM_TimerWaveStructure);
  
  /* Configure HRTIM1_TIMA Deadtime */
  HRTIM_TIM_DeadTimeStructure.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  HRTIM_TIM_DeadTimeStructure.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.FallingValue = QB_ON_DEADTIME;
  HRTIM_TIM_DeadTimeStructure.Prescaler = 0x0;
  HRTIM_TIM_DeadTimeStructure.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  HRTIM_TIM_DeadTimeStructure.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.RisingValue = QB_OFF_DEADTIME;
  HRTIM_DeadTimeConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, &HRTIM_TIM_DeadTimeStructure);
  
  /* Configure HRTIM1_TIMB Deadtime */
  HRTIM_TIM_DeadTimeStructure.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  HRTIM_TIM_DeadTimeStructure.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.FallingValue = QD_ON_DEADTIME;
  HRTIM_TIM_DeadTimeStructure.Prescaler = 0x0;
  HRTIM_TIM_DeadTimeStructure.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  HRTIM_TIM_DeadTimeStructure.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  HRTIM_TIM_DeadTimeStructure.RisingValue = QD_OFF_DEADTIME;
  HRTIM_DeadTimeConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, &HRTIM_TIM_DeadTimeStructure);
  
  /* Initialize HRTIM1_TIMA and HRTIM1_TIMB */
  HRTIM_TimerInitStructure.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  HRTIM_TimerInitStructure.DACSynchro = HRTIM_DACSYNC_NONE;
  HRTIM_TimerInitStructure.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  HRTIM_TimerInitStructure.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  HRTIM_TimerInitStructure.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  HRTIM_TimerInitStructure.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  HRTIM_TimerInitStructure.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  HRTIM_TimerInitStructure.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
   
  HRTIM_Waveform_Init(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, &HRTIM_BaseInitStructure, &HRTIM_TimerInitStructure); 
  HRTIM_Waveform_Init(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, &HRTIM_BaseInitStructure, &HRTIM_TimerInitStructure);
  
  /*HRTIM 250KHz configuration */
  HRTIM_BaseInitStructure.Period = SetPeriod; /* 1 period = 4 탎 = 100% time */
  HRTIM_BaseInitStructure.RepetitionCounter = 10;
  HRTIM_BaseInitStructure.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;  
  HRTIM_BaseInitStructure.Mode = HRTIM_MODE_CONTINOUS;          
  HRTIM_SimpleBase_Init(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, &HRTIM_BaseInitStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = DutyCycleTimA; 
  HRTIM_CompareStructure.CompareValue = DutyCycleTimA;
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &HRTIM_CompareStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = DutyCycleTimA + BlankTimeTimA; 
  HRTIM_CompareStructure.CompareValue = DutyCycleTimA + BlankTimeTimA;
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &HRTIM_CompareStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = (uint32_t)Ratio*DutyCycleTimA + BlankTimeTimA; 
  HRTIM_CompareStructure.CompareValue = (uint32_t)Ratio*DutyCycleTimA + BlankTimeTimA;
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, &HRTIM_CompareStructure);
  
  HRTIM_SimpleBase_Init(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, &HRTIM_BaseInitStructure);
  
  HRTIM_CompareStructure.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  HRTIM_CompareStructure.AutoDelayedTimeout = ADCTriggerValue; 
  HRTIM_CompareStructure.CompareValue = ADCTriggerValue;
  HRTIM_WaveformCompareConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &HRTIM_CompareStructure);
  
  /* End HRTIM 250KHz configuration */
  
  /* Configure and enable HRTIM interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = HRTIM1_TIMA_IRQn;   	 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Configure and enable HRTIM interrupt */
  /* NVIC_InitStructure.NVIC_IRQChannel = HRTIM1_TIMB_IRQn;   	 
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);*/
  
  HRTIM_ITConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_IT_REP, ENABLE);
  
  /* Enable the TA1 output */
  HRTIM_WaveformOutputStart(HRTIM1, HRTIM_OUTPUT_TA1);
  /* Enable the TA2 output */
  HRTIM_WaveformOutputStart(HRTIM1, HRTIM_OUTPUT_TA2);
  /* Enable the TB1 output */
  HRTIM_WaveformOutputStart(HRTIM1, HRTIM_OUTPUT_TB1);  
  /* Enable the TB2 output */
  HRTIM_WaveformOutputStart(HRTIM1, HRTIM_OUTPUT_TB2);
  
  /* Start H bridge converter in Buck Mode */
  SetHRTIM_BuckMode();
  
  /* ADC Configuration */
  ADC_Config();
  
  Delay(10000); 
  while(1)
  {
    uint32_t timeout = 0 ;
    
    /* Check if VIN connected to 5V and wait until voltage is present and included in the expected range */
    /* Range is 4.5V to 5.5V */
    while ((VIN < Min_Voltage) | (VIN >  Max_Voltage))
    {
      /* Enable next trigger on ADC during data process*/
      ADC_StartInjectedConversion(ADC1);
      /* Compute the voltage (Voltages have to be multiplied by ~5 as bridge divider is ~20%) */
      ADC1_Channel1_ConvertedValue_IN = ADC1_Channel1_ConvertedValue_IN * REFINT_CAL / ADC1ReferenceConvertedValue;
      VIN = (ADC1_Channel1_ConvertedValue_IN *3300)/0xFFF;
      /* VIN bridge conversion is 4.97 (6.8K / 6.8K + 27K) */
      VIN = (497 * VIN )/100;
      timeout++;
      if (timeout >= 100)
      {
        while(1)
        {   
          /* Orange LED is blinking to alert on missing input voltage */
          STM_EVAL_LEDOn(LED4);
          Delay(10000);
          STM_EVAL_LEDOff(LED4);
          Delay(10000);
        }
      }
    }
    /* Input voltage connected with expected value: Green LED is ON */
    STM_EVAL_LEDOn(LED5);
    /* Set Boost Mode and Vout target value to 10V  */
    VOUT_Target = 10000;
    Current_Mode = BOOST;
    Converter_Mode_Change =1; 
    Delay(10000);
    /* Enable next trigger on ADC during data process*/
    ADC_StartInjectedConversion(ADC1);
    while((VOUT < (VOUT_Target - 5))| (VOUT > (VOUT_Target + 5)))
    {
      /* VIN and VOUT ADC sampling */
      ADC1_Channel1_ConvertedValue_IN = ADC1_Channel1_ConvertedValue_IN * REFINT_CAL / ADC1ReferenceConvertedValue;
      ADC1_Channel1_ConvertedValue_OUT = ADC1_Channel1_ConvertedValue_OUT * REFINT_CAL / ADC1ReferenceConvertedValue;
      
      VIN = (ADC1_Channel1_ConvertedValue_IN *3300)/0xFFF;
      VOUT = (ADC1_Channel1_ConvertedValue_OUT *3300)/0xFFF;
      VIN = (497 * VIN )/100;
      /* VOUT bridge conversion is 5.03 (3.3K / 3.3K + 13.3K) */
      VOUT = (503 * VOUT)/100;
      /* Control Timer A Duty Cycle to reach the VOUT target */ 
      if(VOUT <= VOUT_Target)
      {
        DutyCycleTimA +=10;
        if (DutyCycleTimA >= SetPeriod - MinTimeTimA)
        {
          DutyCycleTimA = SetPeriod - MinTimeTimA;
        }
      }
      else
      {
        DutyCycleTimA -=10;
        if (DutyCycleTimA<= MinTimeTimA)
        {
          DutyCycleTimA = MinTimeTimA;
        }  
      }
      timeout++;
      HRTIM_SlaveSetCompare(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1,  DutyCycleTimA);
      Delay(50);
      /* Enable next trigger on ADC during data process*/
      ADC_StartInjectedConversion(ADC1);
      /* Verify if timeout not reached OR VIN not in expected range */
      if ((timeout > 10000) | (VIN < Min_Voltage) | (VIN >  Max_Voltage))
      {
        STM_EVAL_LEDOff(LED5);        
        while (1)
        {
          /* Red LED is set to alert on Buck boost function failure */
          STM_EVAL_LEDOn(LED3);
          Delay(10000);
          STM_EVAL_LEDOff(LED3);
          Delay(10000);
        }
      }     
    }
    while(!Keypressed)
    {
      /* Boost mode successful */
      /* Blue LED is toggling */
      STM_EVAL_LEDOn(LED6);
      Delay(10000);
      STM_EVAL_LEDOff(LED6);
      Delay(10000);
    }
    Keypressed = FALSE;
    
    /* Set Buck Mode and Vout target value to 2V  */
    VOUT_Target = 2000;
    Current_Mode = BUCK;
    Converter_Mode_Change =1;
    Delay(1000); 
    timeout = 0;
    
    STM_EVAL_LEDOn(LED6);
    STM_EVAL_LEDOff(LED5);
    
    while((VOUT < (VOUT_Target - 5))| (VOUT > (VOUT_Target + 5)))
    {
      /* VIN and VOUT ADC sampling */
      ADC1_Channel1_ConvertedValue_IN = ADC1_Channel1_ConvertedValue_IN * REFINT_CAL / ADC1ReferenceConvertedValue;
      ADC1_Channel1_ConvertedValue_OUT = ADC1_Channel1_ConvertedValue_OUT * REFINT_CAL / ADC1ReferenceConvertedValue;
      
      VIN = (ADC1_Channel1_ConvertedValue_IN *3300)/0xFFF;
      VOUT = (ADC1_Channel1_ConvertedValue_OUT *3300)/0xFFF;
      VIN = (497 * VIN )/100;
      VOUT = (503 * VOUT)/100;  
      /* Control Timer A Duty Cycle to reach the VOUT target */       
      if(VOUT <= VOUT_Target)
      {
        DutyCycleTimA +=10;
        if (DutyCycleTimA >= SetPeriod - MinTimeTimA)
        {
          DutyCycleTimA = SetPeriod - MinTimeTimA;
        }
      }
      else
      {
        DutyCycleTimA -=10;
        if (DutyCycleTimA<= MinTimeTimA)
        {
          DutyCycleTimA = MinTimeTimA;
        }  
      }
      timeout++;
      HRTIM_SlaveSetCompare(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1,  DutyCycleTimA);
      Delay(50);
      /* Enable next trigger on ADC during data process*/
      ADC_StartInjectedConversion(ADC1);
      /* Verify if timeout not reached OR VIN not in expected range */      
      if ((timeout > 10000) | (VIN < Min_Voltage) | (VIN >  Max_Voltage))
      {
        STM_EVAL_LEDOff(LED6);     
        while (1)
        {
          /* Red LED is set to alert on Buck boost function failure */
          STM_EVAL_LEDOn(LED3);
          Delay(10000);
          STM_EVAL_LEDOn(LED3);
          Delay(10000);
        }
      } 
    }
    
    while(!Keypressed)
    {
      /* Buck mode successful */
      /* Green LED is toggling */
      STM_EVAL_LEDOn(LED5);
      Delay(10000);
      STM_EVAL_LEDOff(LED5);
      Delay(10000);
    } 
    while (1)
      /* Test complete */
    {
      /* All LEDs are toggling : TEST SUCCESSFUL */
      STM_EVAL_LEDOn(LED3);  
      STM_EVAL_LEDOn(LED4);
      STM_EVAL_LEDOn(LED5);          
      STM_EVAL_LEDOn(LED6);
      Delay(15000);
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED4);
      STM_EVAL_LEDOff(LED5);
      STM_EVAL_LEDOff(LED6);
      Delay(15000);
    }
  }  
}


/**
* @brief  ADC configuration
* @param  None
* @retval None
*/
static void ADC_Config_3(void)
{
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg;
  
  /* ADC configuration */
  /* ADC is first set in single channel to record VREF */
  /* then turned into injected mode to monitor VIN and VOUT voltages */
  
  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  
  /* Enable ADC1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  
  /* ADC Channel configuration */
  /* Configure ADC Channel2 as analog input / PA1/VIN */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure ADC Channel4 as analog input / PA3/VOUT */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  ADC_StructInit(&ADC_InitStructure);
  
  /* Calibration procedure */ 
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  
  /* Insert delay equal to 100 탎 */
  Delay(10);
  
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
  calibration_value = ADC_GetCalibrationValue(ADC1);
  
  /* Start single channel voltage reference measurement */
  REFINT_CAL = (uint16_t) *(__IO uint32_t *)((uint32_t)0x1FFFF7BA);
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4; /* AHB clock / 4 = 16MHz : 12-bit resolution */
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);  
  
  /* Enable the reference voltage channel */
  ADC_VrefintCmd(ADC1, ENABLE);
  
  /* 61.5 clock cycles @16MHz = 3.84us */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_61Cycles5);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  
  Delay(100);
  
  /* Start ADC1 Software Conversion */ 
  ADC_StartConversion(ADC1);
  
  /* Test EOC flag */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  
  /* Get ADC1 converted data */
  ADC1ReferenceConvertedValue = ADC_GetConversionValue(ADC1);
  
  /* Check no conversion is ongoing */
  while (ADC_GetStartConversionStatus(ADC1)==SET);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, DISABLE);
  
  /* check and wait ADC is disabled */
  while (ADC_GetDisableCmdStatus(ADC1)==SET);
  
  /* Clear ADC ready flag */
  ADC_ClearFlag(ADC1, ADC_FLAG_RDY); 
  
  Delay(100);
  
  /* Switch to ADC injected mode after VREFINT calibration */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_InjSimul;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable; //ADC_ContinuousConvMode_Enable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_9;          /* HRTIM_ADCTRG2 */
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_InjectedStructInit( &ADC_InjectedInitStruct);
  
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_9;
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge;
  ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_InjectedChannel_2; /* corresponds to PA1 VIN */
  ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_InjectedChannel_4; /* corresponds to PA3 VOUT */
  ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 2;
  
  ADC_InjectedInit(ADC1, &ADC_InjectedInitStruct);
  
  /* ADC1 channel1 configuration */ 
  /* Example VIN Tconv = Tsamp (7.5) + 12.5 ADC clock cycles = 20 ADC clock samples = 278ns @72MHz */
  ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_Channel_2, ADC_SampleTime_7Cycles5); /* VIN */
  ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_Channel_4, ADC_SampleTime_19Cycles5); /* VOUT */
  
  /* Configure and enable ADC1 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;   	 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable JEOS interrupt */
  ADC_ITConfig(ADC1, ADC_IT_JEOS, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  
  /*HRTIM trigger configuration for ADC1 */
  pADCTriggerCfg.UpdateSource =  HRTIM_ADCTRIGGERUPDATE_TIMER_B;
  pADCTriggerCfg.Trigger =  HRTIM_ADCTRIGGEREVENT24_TIMERB_CMP2;
  HRTIM_ADCTriggerConfig(HRTIM1,  HRTIM_ADCTRIGGER_2, &pADCTriggerCfg);

  /* Start ADC1 Software Conversion */ 
  ADC_StartInjectedConversion(ADC1);
}


/**
* @brief  Configure HRTIM for Buck Mode configuration
* @param  None
* @retval None
*/
void SetHRTIM_BuckMode(void)
{
  HRTIM_Unselect_OutputTIMx();
  /* Set TIMA A1(PA8) and A2(PA9) opposite PWM ouputs */
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_TIMPER;  
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTSET_TIMCMP1; 
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &HRTIM_TIM_OutputStructure);
  
  /* Reset Output B1(PA10) to open T12 NMOS and set B2 (PA11) to close T5 PMOS bridge, operated as I/O */
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_NONE;
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTSET_TIMPER; 
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &HRTIM_TIM_OutputStructure);
  
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_TIMPER;
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTSET_NONE;
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &HRTIM_TIM_OutputStructure);
  
  /* Start both TIMA and TIMB */
  HRTIM_WaveformCounterStart(HRTIM1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B); 
  Converter_Mode_Change =0;
}

/**
* @brief  Configure HRTIM for Boost Mode configuration
* @param  None
* @retval None
*/
void SetHRTIM_BoostMode(void)
{
  HRTIM_Unselect_OutputTIMx();
  /* Set TIMA B1(PA10) and B2(PA11) opposite PWM ouputs */
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_TIMPER;  
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTSET_TIMEV_1; 
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &HRTIM_TIM_OutputStructure);
  
  /* Set Output A1(PA8) to open T4 PMOS and reset A2(PA9) to close T5 NMOS bridge, operated as I/O */
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_NONE;
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTSET_TIMPER; 
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &HRTIM_TIM_OutputStructure);
  
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_TIMPER;
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTSET_NONE;
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &HRTIM_TIM_OutputStructure);
  
  /* Start both TIMA and TIMB */
  HRTIM_WaveformCounterStart(HRTIM1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B); 
  Converter_Mode_Change =0;
}
//----



/**
* @brief  Reset ALL HRTIM Setx and Rstx registers Output Disable 
* @param  None
* @retval None
*/
static void HRTIM_Unselect_OutputTIMx(void)
{  
  HRTIM_TIM_OutputStructure.SetSource = HRTIM_OUTPUTSET_NONE;
  HRTIM_TIM_OutputStructure.ResetSource = HRTIM_OUTPUTRESET_NONE; 
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &HRTIM_TIM_OutputStructure);
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &HRTIM_TIM_OutputStructure);
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &HRTIM_TIM_OutputStructure);
  HRTIM_WaveformOutputConfig(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &HRTIM_TIM_OutputStructure);
  
}

/**
* @brief  Inserts a delay time.
* @param  nTime: specifies the delay time length, in 1 ms.
* @retval None
*/
void Delay(__IO uint32_t nTime)
{
  /* value in tens of 탎: example nTime = 10 delay = 10 * 10탎 = 100탎*/
  TimingDelay = nTime;
  if (NoWait == TRUE)
  {
    TimingDelay=0;
  }
  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
