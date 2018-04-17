/**
  ******************************************************************************
  * @file    stm32f3xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-June-2014
  * @brief   This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "stm32f3xx_it.h"
#include "extern.h"
#include "lcd.h"
#include "ads1232.h"

unsigned char buffer3[17];
unsigned char buffer_index3=0;

/** @addtogroup STM32F3348-Discovery_Demo
* @{
*/
#define adc_rdy         EXTI_Line6
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Counter32=0;
uint8_t Counter10us=0;
uint8_t FallingSlope=0;
extern __IO uint8_t StateMachine;
extern __IO uint8_t NoWait;
extern __IO uint8_t DownUp;
extern __IO uint16_t TriangCMP1;
extern __IO uint8_t TriangleGeneration;
extern __IO uint16_t Converter_Mode_Change;
extern __IO uint16_t Current_Mode;
extern __IO uint16_t Keypressed;
extern __IO float Ratio;
extern __IO uint16_t TESTMODE;
extern __IO uint32_t  ADC1_Channel1_ConvertedValue_IN;
extern __IO uint32_t  ADC1_Channel1_ConvertedValue_OUT;
extern __IO uint8_t index1;
float index2;
extern uint16_t DitherTab[8][8];
extern __IO float variable;
extern uint32_t CurrentSenseTab[5];

/* Private function prototypes -----------------------------------------------*/
extern void SetHRTIM_BuckMode(void);
extern void SetHRTIM_BoostMode(void);
extern void HRTIM_SetBurstCompare(float BurstCompare);
	
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                          */
/******************************************************************************/

/**
* @brief  This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{  
  static unsigned int ticks;
//	TimingDelay_Decrement();
	delay_main++;
	delay_setting++;
	button_timer++;
	 CODEUR_Read();
	//------
	

}

/******************************************************************************/
/*                 STM32F3xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f334x8.s).                                             */
/******************************************************************************/

/**
* @brief  This function handles ADC1 interrupts requests.
* @param  None
* @retval None
*/
void ADC1_2_IRQHandler(void)
{
  
  if(ADC_GetFlagStatus(ADC1, ADC_IT_JEOS) != RESET)
  {   
    HRTIM_ClearFlag(HRTIM1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_FLAG_CMP2);
    /* Get ADC1 converted data */
    ADC1_Channel1_ConvertedValue_IN = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    ADC1_Channel1_ConvertedValue_OUT = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
    /* Clear ADC1 EOC pending interrupt bit */
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOS);
  }
  /* Disable next trigger on ADC during data process*/
  ADC_StopInjectedConversion(ADC1);
}

/**
* @brief  This function handles EXTI0 Line interrupts requests.
* @param  None
* @retval None
*/

// 0xff command  data // 3 byte 
void USART1_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		rxdata_1=( USART1->RDR & 0x1FF);
		if(rxdata_1=='S')
		{
		buzzer_control(1);
		
		}
		
		
			if(rxdata_1=='O')
		{
		buzzer_control(0);
		
		}
	
		
	
	}
}



//burasiiiiiiiiiiiiiiiii
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************

void usart_isr_genel(void)
{	
	switch(usart_3_state)
			{
			
			
				case 0: {
								if(rxdata_3==header_1	)
								{
								usart_3_state=1;
								
								}};break; 
//-------------------------------------------------------				
			
			case 1: {
								if(rxdata_3==header_2)
								{
								usart_3_state=2;
								
								}
								else 	usart_3_state=0;
			
			
			        };break; 		
	//----------------		
			case 2: {
				       if(rxdata_3==0x06)
								{
								usart_3_state=3;
								
								}
								else if(rxdata_3==0x08)
								{
								
									usart_3_state=9;
								}
									else if(rxdata_3==0x0A)
								{
								
									usart_3_state=17;
								}
								else 	usart_3_state=0;
								
			
			        };break; 		
//---------------------
			case 3: {
								if(rxdata_3==0x83)
								{
								usart_3_state=4;
								
								}
								else 	usart_3_state=0;
			
			
			        };break; 							
//---------------------
			case 4: {//--- vp high hep 0
							
							
						vp_high=rxdata_3;
						usart_3_state=5;
			
			        };break; 				
			
		//------

		case 5: {// ************vp low***************
								
			vp_low=rxdata_3;
			
			
			usart_3_state=6;
				
											};break; 		
//-------------------------------------------
		case 6: // always 01  
			{
			usart_3_state=7;
		
	  	};break;
	//--------------------------------------------		
		case 7:  // pass word high 
			
			{
			password_high=rxdata_3;
			usart_3_state=8;
		
	  	};break;
			
	//----------
		case 8: // pass word low ___ key value 
		{
			password_low=rxdata_3;
			
			
			paket_geldi=1; 
			usart_3_state=0;	
								
			};break;
		
		case 9: 
		{
					if(rxdata_3==0x83)
								{
								usart_3_state=10;
								
								}
								else 	usart_3_state=0;
			

		
			};break;
		//------------
			
		case 10 : 
		{
				
						vp_high=rxdata_3;
								usart_3_state=11;
		};break; 

//---
		case 11 : 
		{
				
						vp_low=rxdata_3;
								usart_3_state=12;
		};break; 
		
	//--
		case 12 : // always 02 
		{
				
					
								usart_3_state=13;
		};break;		
	///--------
case 13: // 00 00 00 02 geliyor sirayla aliyoruz
		{       
				
					keypad_3=rxdata_3;
	
								usart_3_state=14;
		};break;	
		
//---
		case 14:
		{
				
					   keypad_2=rxdata_3;
	          usart_3_state=15;
		};break;	
//-----
				case 15: 
		{
				
					   keypad_1=rxdata_3;
	          usart_3_state=16;
		};break;	
	//-----------------------------------	
				case 16: 
		{
				
					   keypad_0=rxdata_3;
			       flags.key_analog=1;
						 paket_geldi=1; 
			       usart_3_state=0;	
		};break;
		//-----------------------------------	
				case 17: 
		{
				if(rxdata_3==0x81) {
				 usart_3_state=18;					
				}
				else  usart_3_state=0;
		};break;	
		//-----------------------------------	
				case 18: 
		{
				if(rxdata_3==0x20) {
				 usart_3_state=19;					
				}
				else  usart_3_state=0;
		};break;	
		//-----------------------------------	
				case 19: 
		{
				if(rxdata_3==0x07) {
				 usart_3_state=20;					
				}
				else  usart_3_state=0;
		};break;	
		//-----------------------------------	
				case 20: 
		{
				year=rxdata_3;
			  usart_3_state=21;
		};break;	
		//-----------------------------------	
				case 21: 
		{
				month=rxdata_3;
			  usart_3_state=22;
		};break;	
		//-----------------------------------	
				case 22: 
		{
				day=rxdata_3;
			  usart_3_state=23;
		};break;	
		//-----------------------------------	
				case 23: 
		{
				week_day=rxdata_3;
			  usart_3_state=24;
		};break;	
		//-----------------------------------	
				case 24: 
		{
				hour=rxdata_3;
			  usart_3_state=25;
		};break;	
		//-----------------------------------	
				case 25: 
		{
				minute=rxdata_3;
			  usart_3_state=25;
		};break;	
		//-----------------------------------	
				case 26: 
		{
				 paket_geldi=1;
				second=rxdata_3;
			  usart_3_state=0;
		};break;	
			

}
		}	

//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//---****************************************************************
//----------------------------
//----------

void USART3_IRQHandler(void)
	{


	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART3, USART_IT_RXNE) )
		
	{
		rxdata_3=( USART3->RDR & 0x1FF);
		buffer3[buffer_index3]=rxdata_3;
		buffer_index3++;
		
		if(buffer_index3==17){
		buffer_index3=0;
		}
		usart_isr_genel();
		}
//		
//	switch(press_task_state)
//	{
//		case 0: usart_isr_genel();break; 
//		case test_task_pointer : usart_isr_test_task();break;
//	//	case 2: admin_task();break;
//	 case calibration_task_pointer : usart_isr_calibration_task(); break;
//	//	case 4: setting_task();break; 
//	}

	
	}


void EXTI0_IRQHandler(void)
{
  uint32_t i=0;
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED5);
  Keypressed = TRUE;
  if(!TESTMODE)
  {
    NoWait = TRUE;
    /* Wait until USER button released */
    while((STM_EVAL_PBGetState(BUTTON_USER) == SET))
    {
      i++;
      if(i==1000000)
      {
        /* Clear the EXTI line 0 pending bit */
        StateMachine = STATE_MAN;
        EXTI_ClearITPendingBit(EXTI_Line0);
        DownUp =!DownUp;
        return;
      }
    }
    /* Change state machine mode */
    StateMachine ++;
    CurrentSenseTab[0]= 280;
    CurrentSenseTab[1]= 240;
    CurrentSenseTab[2]= 200;
    CurrentSenseTab[3]= 360;
    CurrentSenseTab[4]= 320;
    /* Convert current sense value thresholds for DAC */
    for (i=0;i<5;i++)
    {
      CurrentSenseTab[i] = CurrentSenseTab[i] * 4096 / 3300; 
    }
    if (StateMachine >= MAX_STATE)
    {
      StateMachine =1;
    }
  }
  /* Clear the EXTI line 0 pending bit */
  EXTI_ClearITPendingBit(EXTI_Line0); 	
}


/**
* @brief  This function handles HRTIM TIMA interrupts requests.
* @param  None
* @retval None
*/
void HRTIM1_TIMA_IRQHandler(void)
{
  /* Converter mode is changed during interrupt to synchronize HRTIM output changes */
  if(Converter_Mode_Change)
  {
    switch(Current_Mode)
    {
    case BUCK:
      SetHRTIM_BuckMode();
      break;
      
    case BOOST:
      SetHRTIM_BoostMode();
      break;
      
    }
  }
  /* Reset HRTIMA repetition flag */
  HRTIM_ClearFlag(HRTIM1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_FLAG_REP);
}

/**
* @brief  This function handles HRTIM Master Timer interrupts requests.
* @param  None
* @retval None
*/
void HRTIM1_Master_IRQHandler(void)
{
  uint32_t var = 0;
  /* Each Master Burst mode period interrupt, the burst compare is updated */
  HRTIM_ClearCommonITPendingBit(HRTIM1, HRTIM_ICR_BMPERC);
  var = (uint32_t) variable;
  index2 = variable - var;
  index2 *= 8;
  
  if (DitherTab[(uint32_t)index2][index1]==0)
  { 
    /* If Dither table value is zero then standard drive ...*/
    HRTIM_SetBurstCompare(var);
  }
  else
  {
    /* or extra period is added */ 
    HRTIM_SetBurstCompare(var + 1);
  }
  /* Each interrupt the table data are rolling from 1 to 7 */
  index1 ++;
  if (index1>7)
  {
    index1=0;
  }
}
//--
void EXTI9_5_IRQHandler(void)
{
	static unsigned int ticks =0; 
  if ((EXTI_GetITStatus(adc_rdy ) == SET)) 
  {
    ticks ++ ; 
		if(ticks>=9)
		{
		ticks=0; 
		GPIOA->ODR ^= RLY1;	
		}
adc_data=ads_read_18(); 
    flags.adc_ready=1;
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(adc_rdy );
  }    
}

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/

/**
* @}
*/ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
