#include <stm32f30x.h>
#include <math.h>
#include <__cross_studio_io.h>
#include <ctl_api.h>
#include "lsm303.h"

#define FULL_INTERVAL (100)
#define SHORT_INTERVAL (20)

#define MAGNETOMETER_ADDR (0x3C)

#define M_PI 3.14159265358979323846

#define CRB_REG_M (0x01)
#define CRB_REG_M_GAIN_MIN (1<<7 | 1<<6 | 1<<5)
#define CRB_REG_M_GAIN_MAX (1<<5)

#define MR_REG_M  (0x02)
#define M_REG_M_CONTINUOS_MODE (0x00)
#define M_REG_M_SINGLE_MODE (1<<0)

#define OUT_X_H_M (0x03)
#define OUT_X_L_M (0x04)
#define OUT_Z_H_M (0x05)
#define OUT_Z_L_M (0x06)
#define OUT_Y_H_M (0x07)
#define OUT_Y_L_M (0x08)
#define SR_REG_M  (0x09)

#define SR_REG_M_DRDY (1 << 0)

#define EVENT_TXIS  (1 << 0)
#define EVENT_RXNE  (1 << 1)
#define EVENT_STOPF (1 << 2)
#define EVENT_TC    (1 << 3)
#define EVENT_TCR   (1 << 4)

#define LED_PIN  GPIO_Pin_10   // PE10 -> LD5 (arancio)
#define LED_PORT GPIOE

void lsm303_init(void);
void lsm303_WrReg(I2C_TypeDef *i2c,uint8_t addr,uint8_t Reg, uint8_t Val);
uint8_t lsm303_RdReg(I2C_TypeDef *i2c,uint8_t addr,int8_t Reg, int8_t *Data, uint8_t DCnt);
uint16_t lsm303_read_heading();

#define QUEUE_SIZE (1)
CTL_MESSAGE_QUEUE_t qHeading;
void *qHeadingContent[QUEUE_SIZE];

CTL_EVENT_SET_t e1;

static inline void __attribute__((always_inline))
led_on()
{
    GPIO_SetBits(LED_PORT,LED_PIN);
}

static inline void __attribute__((always_inline))
led_off()
{
    GPIO_ResetBits(LED_PORT,LED_PIN);
}

void lsm303_task_code(void *p)
{
#ifdef DEBUG
    debug_puts("lsm303 task started!");
#endif
    lsm303_init();
    CTL_TIME_t last_read = 0;
    CTL_TIME_t cur_read;
    while(1)
    {
        led_on();
        uint16_t heading = lsm303_read_heading(); 
        if(heading != HEADING_UNAVAIL)
        {
            if(0 == ctl_message_queue_num_free(&qHeading))
            {
                void *trash;
                ctl_message_queue_receive(&qHeading,&trash,CTL_TIMEOUT_NONE,0);
            }
            ctl_message_queue_post(&qHeading,(void *)(unsigned)heading,CTL_TIMEOUT_NONE,0);

            cur_read = ctl_get_current_time();
#ifdef DEBUG
            debug_printf("Heading:%.1f (%i ms)\r",(heading/10.0),(cur_read - last_read));
#endif            
            last_read = cur_read;
            led_off();
            ctl_timeout_wait(ctl_get_current_time()+FULL_INTERVAL); //ctl_sleep(FULL_INTERVAL);
        }
        else
        {
            ctl_timeout_wait(ctl_get_current_time()+SHORT_INTERVAL); // ctl_sleep(SHORT_INTERVAL);
        }
    }
}

void lsm303_config(void)
{
    ctl_message_queue_init(&qHeading,qHeadingContent,QUEUE_SIZE);
    // PB6 - I2C1 - SCL
    // PB7 - I2C1 - SDA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

    // GPIO
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_4);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_4);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOB,&gpio);

    // LED
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = LED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(LED_PORT,&gpio);

    // I2C
    I2C_InitTypeDef i2c;
    I2C_StructInit(&i2c);

    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    i2c.I2C_DigitalFilter = 0;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_OwnAddress1 = 0xAB;
    i2c.I2C_Timing = 0xC062121F; // preso da http://stackoverflow.com/questions/19576182/stm32f3-i2c-read-data
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1,&i2c);
    I2C_Cmd(I2C1,ENABLE);

    // events
    ctl_events_init(&e1,0);
    
    // NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = I2C1_EV_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 15;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);    
}

void lsm303_init(void)
{
    lsm303_WrReg(I2C1,MAGNETOMETER_ADDR,MR_REG_M,M_REG_M_CONTINUOS_MODE); 
    lsm303_WrReg(I2C1,MAGNETOMETER_ADDR,CRB_REG_M,CRB_REG_M_GAIN_MIN);
}


void lsm303_WrReg(I2C_TypeDef *i2c,uint8_t addr,uint8_t Reg, uint8_t Val){

	//Wait until I2C isn't busy
	while(I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY) == SET);
	
	I2C_TransferHandling(i2c, addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    I2C_ITConfig(I2C1,I2C_IT_TXI,ENABLE);
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_TXIS,CTL_TIMEOUT_NONE, 0);

	//Send the address of the register we wish to write to
	I2C_SendData(i2c, Reg);

	//Ensure that the transfer complete reload flag is
	//set, essentially a standard TC flag
    I2C_ITConfig(I2C1,I2C_IT_TCI,ENABLE);
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_TCR,CTL_TIMEOUT_NONE,0);

	//Now that the HMC5883L knows which register
	//we want to write to, send the address again
	//and ensure the I2C peripheral doesn't add
	//any start or stop conditions
	I2C_TransferHandling(i2c, addr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    I2C_ITConfig(I2C1,I2C_IT_TXI,ENABLE);
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_TXIS,CTL_TIMEOUT_NONE,0);
	//Again, wait until the transmit interrupted flag is set
	//while(I2C_GetFlagStatus(i2c, I2C_FLAG_TXIS) == RESET);

	//Send the value you wish you write to the register
	I2C_SendData(i2c, Val);

	//Wait for the stop flag to be set indicating
	//a stop condition has been sent
    I2C_ITConfig(I2C1,I2C_IT_STOPI,ENABLE);
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_STOPF,CTL_TIMEOUT_NONE,0);
    I2C_ITConfig(I2C1,I2C_IT_STOPI,DISABLE);
	//Clear the stop flag for the next potential transfer
	I2C_ClearFlag(i2c, I2C_FLAG_STOPF);
}

uint8_t lsm303_RdReg(I2C_TypeDef *i2c,uint8_t addr,int8_t Reg, int8_t *Data, uint8_t DCnt){
	int8_t Cnt, SingleData = 0;

	//As per, ensure the I2C peripheral isn't busy!
	while(I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY) == SET);

	//Again, start another tranfer using the "transfer handling"
	//function, the end bit being set in software this time
	//round, generate a start condition and indicate you will
	//be writing data to the HMC device.
	I2C_TransferHandling(i2c, addr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    
    I2C_ITConfig(I2C1,I2C_IT_TXI,ENABLE);
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_TXIS,CTL_TIMEOUT_NONE, 0);

	//Wait until the transmit interrupt status is set
	//while(I2C_GetFlagStatus(i2c, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register you wish to read
	I2C_SendData(i2c, (uint8_t)Reg);

	//Wait until transfer is complete!
	//while(I2C_GetFlagStatus(i2c, I2C_FLAG_TC) == RESET);

    I2C_ITConfig(I2C1,I2C_IT_TCI,ENABLE);
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_TC,CTL_TIMEOUT_NONE,0);

	//As per, start another transfer, we want to read DCnt
	//amount of bytes. Generate a start condition and
	//indicate that we want to read.
	I2C_TransferHandling(i2c, addr, DCnt, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	//Read in DCnt pieces of data
	for(Cnt = 0; Cnt<DCnt; Cnt++){
        //Wait until the RX register is full of luscious data!
        //while(I2C_GetFlagStatus(i2c, I2C_FLAG_RXNE) == RESET); 
        
        I2C_ITConfig(I2C1,I2C_IT_RXI,ENABLE);
        ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_RXNE,CTL_TIMEOUT_NONE,0);
        //If we're only reading one byte, place that data direct into the 
        //SingleData variable. If we're reading more than 1 piece of data 
        //store in the array "Data" (a pointer from main) 		
        if(DCnt > 1) Data[Cnt] = I2C_ReceiveData(i2c);
        else SingleData = I2C_ReceiveData(i2c);
     }

     //Wait for the stop condition to be sent
     //while(I2C_GetFlagStatus(i2c, I2C_FLAG_STOPF) == RESET);
    I2C_ITConfig(I2C1,I2C_IT_STOPI,ENABLE);
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&e1,EVENT_STOPF,CTL_TIMEOUT_NONE,0);
    I2C_ITConfig(I2C1,I2C_IT_STOPI,DISABLE);

     //Clear the stop flag for next transfers
     I2C_ClearFlag(i2c, I2C_FLAG_STOPF);

     //Return a single piece of data if DCnt was
     //less than 1, otherwise 0 will be returned.
	return SingleData;
}

uint16_t lsm303_read_heading()
{
    uint16_t result = HEADING_UNAVAIL;
    if(lsm303_RdReg(I2C1,MAGNETOMETER_ADDR,SR_REG_M,0,1) & SR_REG_M_DRDY)
    {
        int8_t mag_data[6] = {0,0,0,0,0,0};
        int16_t x_val = 0,z_val = 0, y_val = 0;
        
        lsm303_RdReg(I2C1,MAGNETOMETER_ADDR,OUT_X_H_M,mag_data,6);

        // i valori ottenuti sono in 1/230 gauss (per z 1/205 gauss)
        uint8_t data[2];

        data[0] = mag_data[1];
        data[1] = mag_data[0];
//        x_val = (mag_data[0]<<8) | mag_data[1];
        x_val = *((int16_t*)data);

        data[0] = mag_data[5];
        data[1] = mag_data[4];

//        y_val = (mag_data[4]<<8) | mag_data[5];
        y_val = *((int16_t*)data);
        float val = (atan2(y_val,x_val) * (180 * 10)/*in decimi di grado*/) / M_PI;
        
        result = val < 0 ? (3600 + val) : val;
    }
    return result;
}

void I2C1_EV_IRQHandler()
{
    ctl_enter_isr();
    if(RESET != I2C_GetITStatus(I2C1,I2C_IT_TXIS))
    {
        ctl_events_set_clear(&e1,EVENT_TXIS,0);
        I2C_ITConfig(I2C1,I2C_IT_TXI,DISABLE);

    }
    if(RESET != I2C_GetITStatus(I2C1,I2C_IT_RXNE))
    {
        ctl_events_set_clear(&e1,EVENT_RXNE,0);
        I2C_ITConfig(I2C1,I2C_IT_RXI,DISABLE);
    }
    if(RESET != I2C_GetITStatus(I2C1,I2C_IT_STOPF))
    {
        I2C_ClearITPendingBit(I2C1,I2C_IT_STOPF);
        ctl_events_set_clear(&e1,EVENT_STOPF,0);
    }
    if(RESET != I2C_GetITStatus(I2C1,I2C_IT_TC))
    {
        ctl_events_set_clear(&e1,EVENT_TC,0);
        I2C_ITConfig(I2C1,I2C_IT_TCI,DISABLE);
    }
    if(RESET != I2C_GetITStatus(I2C1,I2C_IT_TCR))
    {
        ctl_events_set_clear(&e1,EVENT_TCR,0);
        I2C_ITConfig(I2C1,I2C_IT_TCI,DISABLE);
    }
    ctl_exit_isr();
}



