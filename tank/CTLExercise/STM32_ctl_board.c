// Copyright (c) 2011 Rowley Associates Limited.
//
// This file may be distributed under the terms of the License Agreement
// provided with this software.
//
// THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

#include <ctl_api.h>
#include <stm32f30x.h>

void SystemInit (void) __attribute__ ((section (".init")));
static void SetSysClock(void) __attribute__ ((section (".init")));

// Use SystemInit from ST
#include "system_stm32f30x.c"

//STM32F3-DISCOVERY LD4 led connected to PE.08 pin, Key push-button connected to pin PA.0 (EXTI Line 0).

//void
//ctl_board_init(void)
//{
//  // Turn on GPIOA, GPIOE
//  RCC->AHBENR = RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN;
//  // Turn on SYSCFG
//  RCC->APB2ENR = RCC_APB2ENR_SYSCFGEN;
//  // Set PE_08 to output
//  GPIOE->MODER |= (1<<16);
//  // Associate PA.0 to EXTI0 - interrupt on a falling edge
//  SYSCFG->EXTICR[0] = 0; 
//  EXTI->FTSR |= (1<<0); // falling edge trigger
//  EXTI->IMR |= (1<<0);  // enable interrupt
//}

//void 
//ctl_board_set_leds(unsigned on)
//{
//  if (on)
//    GPIOE->BSRR = (1<<8);
//  else
//    GPIOE->BRR = (1<<8);
//}
//
//static CTL_ISR_FN_t userButtonISR;

//void
//EXTI0_IRQHandler(void)
//{
//  ctl_enter_isr();
//  if (EXTI->PR & (1<<0))
//    {
//      userButtonISR();
//      EXTI->PR = (1<<0);
//    }
//  ctl_exit_isr();
//}

//void 
//ctl_board_on_button_pressed(CTL_ISR_FN_t isr)
//{
//  userButtonISR = isr;
//  ctl_set_priority(EXTI0_IRQn, ctl_adjust_isr_priority(ctl_highest_isr_priority(), -1));
//  ctl_unmask_isr(EXTI0_IRQn);
//}
