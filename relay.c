#include "main.h"
#include "extern.h"


void relay_1( unsigned char status)
{
if(status==1)
{
GPIO_SetBits(RLY1_GPIO_PORT , RLY1);
}
else
{
GPIO_ResetBits(RLY1_GPIO_PORT , RLY1);

}

}
//--------------------------------------------------
void relay_2( unsigned char status)
{
if(status==1)
{
GPIO_SetBits(RLY2_GPIO_PORT , RLY2);
}
else
{
GPIO_ResetBits(RLY2_GPIO_PORT , RLY2);

}

}

//---

void relay_driver_enable( unsigned char status)
{
if(status==1)
{
GPIO_SetBits(driver_enable_GPIO_PORT , driver_enable);
}
else
{
GPIO_ResetBits(driver_enable_GPIO_PORT , driver_enable);

}

}

//----
void buzzer_control( unsigned char status)
{
if(status==1)
{
GPIO_SetBits(buzzer_GPIO_PORT , buzzer);
}
else
{
GPIO_ResetBits(buzzer_GPIO_PORT , buzzer);

}

}