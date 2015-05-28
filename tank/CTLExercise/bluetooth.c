#include "stm32f30x.h"
#include <ctl_api.h>
#include "bluetooth.h"
#include "lsm303.h"
#include <__cross_studio_io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// IN-QUEUE
CTL_MESSAGE_QUEUE_t qMsgIn;
#define MSG_IN_QUEUE_SIZE (0x10)
void *qMsgInContent[MSG_IN_QUEUE_SIZE];

// OUT-QUEUE
CTL_MESSAGE_QUEUE_t qMsgOut;
#define MSG_OUT_QUEUE_SIZE (0x10)
void *qMsgOutContent[MSG_OUT_QUEUE_SIZE];
void *sCurOutMsg = 0;
char *pCurOutChar = 0;

// CONNECTION EVENT
CTL_EVENT_SET_t eConn;
#define EVENT_CONNECTED  (1 << 0)

// OUT MESSAGES MEMORY MGMT
#define MSG_OUT_COUNT MSG_OUT_QUEUE_SIZE
#define MSG_OUT_WORDS_SIZE  (0x08)
CTL_MEMORY_AREA_t mAreaOut;
unsigned mAreaOutMemory[MSG_OUT_COUNT * MSG_OUT_WORDS_SIZE]; 

void bt_task_code(void *p)
{
#ifdef DEBUG
    debug_puts("Bluetooth task started!");
#endif
    bt_init();

    while(1)
    {
    	ctl_events_wait_uc(CTL_EVENT_WAIT_ANY_EVENTS,&eConn,EVENT_CONNECTED);
    	
    	void *heading;
    	ctl_message_queue_receive_uc(&qHeading,&heading);
    	
    	void *pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    	if(pMsgOut)
    	{
    		sprintf(pMsgOut,"\r\nH:%u\r\n",(uint16_t)(unsigned)heading);
    		ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_DELAY,1000);
    		bt_flush();
    	}
    	else
    	{
#ifdef DEBUG
    		debug_printf("%s:%s: memory area allocation problem",__FILE__,__LINE__);
#endif    		
            while(1);
    	}
    	// TODO: consumare messaggi in ingresso
        //ctl_timeout_wait(ctl_get_current_time()+2000); // TODO rimuovere
    }
}


void bt_config(void)
{
	ctl_memory_area_init(&mAreaOut,mAreaOutMemory,MSG_OUT_WORDS_SIZE,MSG_OUT_COUNT);
	
    ctl_message_queue_init(&qMsgIn,qMsgInContent,MSG_IN_QUEUE_SIZE);
    ctl_message_queue_init(&qMsgOut,qMsgOutContent,MSG_OUT_QUEUE_SIZE);
    
    ctl_events_init(&eConn,0);

	// PD0 PIO1: connection status (HIGH=connected LOW=disconnected)
	// PD5 USART2 TX
	// PD6 USART2 RX

	// GPIO
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_7);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_7);
	GPIO_InitTypeDef gpioInit;
	GPIO_StructInit(&gpioInit);
	gpioInit.GPIO_Mode = GPIO_Mode_AF;
	gpioInit.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInit.GPIO_OType = GPIO_OType_PP;
	gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&gpioInit);

	GPIO_StructInit(&gpioInit);
	gpioInit.GPIO_Pin = GPIO_Pin_0;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInit.GPIO_Mode = GPIO_Mode_IN;
	gpioInit.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_Init(GPIOD,&gpioInit);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource0);

	// NVIC
	NVIC_InitTypeDef initNVIC;
	initNVIC.NVIC_IRQChannel = USART2_IRQn;
	initNVIC.NVIC_IRQChannelPreemptionPriority = 3;
	initNVIC.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&initNVIC);

	initNVIC.NVIC_IRQChannel = EXTI0_IRQn;
	initNVIC.NVIC_IRQChannelPreemptionPriority = 3;
	initNVIC.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&initNVIC);

	// UART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);    
	USART_InitTypeDef UART_settings;
	USART_StructInit(&UART_settings);
	UART_settings.USART_BaudRate = 38400;
	USART_Init(USART2,&UART_settings);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART2,ENABLE);  

	// Interrupt on line 0 (for PD0)
	EXTI_InitTypeDef initEXTI;
	EXTI_StructInit(&initEXTI);
	initEXTI.EXTI_Line = EXTI_Line0;
	initEXTI.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	initEXTI.EXTI_LineCmd = ENABLE;
	EXTI_Init(&initEXTI);
}

void bt_init()
{
    // to allow the Bt shield to power up
    ctl_timeout_wait(ctl_get_current_time()+500);
    
    void *pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+SETCHO\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);

    pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+RTADDR\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);

    pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+STWMOD=0\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);

    pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+STNA=MAX MERDA\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);

    pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+STAUTO=0\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);

    pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+STOAUT=1\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);

    pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+STPIN =0000\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);
    bt_flush();
    ctl_timeout_wait(ctl_get_current_time()+2000);
    pMsgOut = ctl_memory_area_allocate(&mAreaOut);
    strcpy(pMsgOut,"\r\n+INQ=1\r\n");
    ctl_message_queue_post(&qMsgOut,pMsgOut,CTL_TIMEOUT_NONE,0);
    bt_flush();
}

void bt_flush(void)
{
    USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
}

void USART2_IRQHandler()
{
	if(RESET != USART_GetITStatus(USART2,USART_IT_ORE))
	{
            USART_ClearITPendingBit(USART2,USART_IT_ORE);
//		event.bit.UART_overflow = EVENT_SET;
	}
	if(RESET != USART_GetITStatus(USART2,USART_IT_RXNE))
	{
            uint8_t val = (USART_ReceiveData(USART2) & 0xff);
        // TODO: implementare
//		buffer_put(&in_buf,val);
//		event.bit.UART_rx = EVENT_SET;
	}
	if(RESET != USART_GetITStatus(USART2,USART_IT_TXE))
	{
		if(sCurOutMsg)
		{
			if(*pCurOutChar)
			{
				USART_SendData(USART2,*pCurOutChar);
				++pCurOutChar;
			}
			else
			{
				ctl_memory_area_free(&mAreaOut,sCurOutMsg);
				sCurOutMsg = 0;
				pCurOutChar = 0;
			}
		}
		else
		{
			if(ctl_message_queue_receive_nb(&qMsgOut,&sCurOutMsg))
			{
				pCurOutChar = sCurOutMsg;
			}
			else
			{	
				USART_ITConfig(USART2,USART_IT_TXE,DISABLE); 
			}
		}
	}
	USART2->ICR = -1;
}

void EXTI0_IRQHandler()
{
	if(RESET != EXTI_GetITStatus(EXTI_Line0))
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(Bit_SET == GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0))
		{
            ctl_events_set_clear(&eConn,EVENT_CONNECTED,0);
#ifdef DEBUG
            debug_puts("BT connected");
#endif		
        }
		else
		{
            ctl_events_set_clear(&eConn,0,EVENT_CONNECTED);
#ifdef DEBUG
            debug_puts("BT disconnected");
#endif
		}
	}
}
