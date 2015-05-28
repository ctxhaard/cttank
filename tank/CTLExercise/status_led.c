#include "status_led.h"
#include <ctl_api.h>
#include <stm32f30x.h>
#include <__cross_studio_io.h>

// LD6 (green): PE15

void status_led_task_code(void *p)
{  
 #ifdef DEBUG
    debug_puts("status led task started!");
#endif
    while (1)
    {
        GPIO_SetBits(GPIOE,GPIO_Pin_15);
        ctl_timeout_wait(ctl_get_current_time()+150); // ctl_sleep(100);
        GPIO_ResetBits(GPIOE,GPIO_Pin_15);
        ctl_timeout_wait(ctl_get_current_time()+350); // ctl_sleep(400);
    }  
}

void status_led_config()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE,ENABLE);

    GPIO_InitTypeDef gpio_init;
    GPIO_StructInit(&gpio_init);

    gpio_init.GPIO_Pin = GPIO_Pin_15;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;

    GPIO_Init(GPIOE,&gpio_init);
}