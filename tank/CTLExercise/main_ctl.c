#include <string.h>
#include <stdio.h>
#include <ctl_api.h>
#include <stm32f30x.h>
#include "status_led.h"
#include "lsm303.h"
#include "bluetooth.h"
#include <__cross_studio_io.h>

CTL_TASK_t main_task, status_led_task, lsm303_task, bt_task;

#define CALLSTACKSIZE (12)
#define STATUS_LED_TASK_STACKSIZE (0x40)          
unsigned status_led_task_stack[1+STATUS_LED_TASK_STACKSIZE+1];

#define LSM303_TASK_STACKSIZE (0x40)
unsigned lsm303_task_stack[1+LSM303_TASK_STACKSIZE+1];

#define BT_TASK_STACKSIZE (0x40)
unsigned bt_task_stack[1+BT_TASK_STACKSIZE+1];

void
ctl_handle_error(CTL_ERROR_CODE_t e)
{
  while (1);
}

int main(void)
{
#ifdef DEBUG
   debug_puts("----------\nCTL exercise\n----------");    
#endif
   status_led_config();
   lsm303_config();
   bt_config();

  ctl_timeslice_period = 100; // 100 ms
  // main task
  ctl_task_init(&main_task, 255, "main"); // create subsequent tasks whilst running at the highest priority.
  ctl_start_timer(ctl_increment_tick_from_isr); // start the timer 

  // status led task
  memset(status_led_task_stack, 0xcd, sizeof(status_led_task_stack));  // write known values into the stack
  status_led_task_stack[0]=status_led_task_stack[1+STATUS_LED_TASK_STACKSIZE]=0xfacefeed; // put marker values at the words before/after the stack
  ctl_task_run(&status_led_task, 20, status_led_task_code, 0, "status led task", STATUS_LED_TASK_STACKSIZE, status_led_task_stack+1, CALLSTACKSIZE);
  // magnetometer task
  memset(lsm303_task_stack, 0xcd, sizeof(lsm303_task_stack));  // write known values into the stack
  lsm303_task_stack[0]=lsm303_task_stack[1+LSM303_TASK_STACKSIZE]=0xfacefeed; // put marker values at the words before/after the stack
  ctl_task_run(&lsm303_task,20,lsm303_task_code,0,"LSM303 task",LSM303_TASK_STACKSIZE,lsm303_task_stack+1,CALLSTACKSIZE);
  // bluetooth task
  memset(bt_task_stack, 0xcd, sizeof(bt_task_stack));  // write known values into the stack
  bt_task_stack[0]=bt_task_stack[1+BT_TASK_STACKSIZE]=0xfacefeed; // put marker values at the words before/after the stack
  ctl_task_run(&bt_task,20,bt_task_code,0,"Bluetooth task",BT_TASK_STACKSIZE,bt_task_stack+1,CALLSTACKSIZE);
  // main task -> idle task
  ctl_task_set_priority(&main_task, 0); // drop to lowest priority to start created tasks running.
  while (1)
    {    
      __WFI();    
    }
  return 0;
}

void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    /* Infinite loop */
    while (1)
    {

    }
}
