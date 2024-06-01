#include "FreeRTOS.h"
#include "task.h"
#include "tm4c123gh6pm.h"
#include "semphr.h"
#include "queue.h"
#include "Macros.h"

static void vDriverTask   ( void *pvParameters );
static void vPassengerTask( void *pvParameters );
static void vLockTask     ( void *pvParameters );
static void vUnlockTask   ( void *pvParameters );

void DIO_A_Init(void);
void DIO_C_Init(void);
void DIO_D_Init(void);
void PortF_Init(void);
void delayMs 	(uint32_t n);

void GPIOA_Handler();
void GPIOD_Handler();

xSemaphoreHandle xBinarySemaphore1;
xSemaphoreHandle xBinarySemaphore2;
xSemaphoreHandle xBinarySemaphore3;
xSemaphoreHandle xBinarySemaphore4;

xQueueHandle xQueue;
portTickType Start = 0 ;
portTickType End = 0 ;

int main( void )
{
		DIO_A_Init();
		DIO_C_Init();
		DIO_D_Init();
		PortF_Init();
		vSemaphoreCreateBinary(xBinarySemaphore1);
		vSemaphoreCreateBinary(xBinarySemaphore2);
		vSemaphoreCreateBinary(xBinarySemaphore3);
		vSemaphoreCreateBinary(xBinarySemaphore4);

		xQueue = xQueueCreate( 5 , sizeof(uint32_t)); 

		xTaskCreate( vDriverTask		, "Driver_Panel" 	 , 92 , NULL , 1 , NULL);
		xTaskCreate( vPassengerTask , "passnger_Panel" , 92 , NULL , 1 , NULL);
		xTaskCreate( vLockTask 		  , "Locking" 			 , 92 , NULL , 1 , NULL);
		xTaskCreate( vUnlockTask 	  , "Unlocking" 		 , 92 , NULL , 1 , NULL);

		vTaskStartScheduler();
		for(;;);
}

static void vDriverTask ( void *pvParameters )
{
    xSemaphoreTake( xBinarySemaphore1 , 0 );
	
    for( ;; )
    {
				xSemaphoreTake( xBinarySemaphore1 , portMAX_DELAY );
				if((GPIO_PORTD_DATA_R&(1<<1)) == 0x00)
				{
						Start = xTaskGetTickCount();
						while((GPIO_PORTD_DATA_R&(1<<1)) == 0x00)
						{
							if((GPIO_PORTC_DATA_R&(1<<5)) == (1<<5))
							{
								Motor_Up();
								setBit(GPIO_PORTF_DATA_R,Green_Led);						
							}
							else
							{
								Motor_Off();
								Leds_Off();
							}
						}
						End = xTaskGetTickCount();
						if((End - Start) <= 200 && (End - Start) > 0)
						{
							while((GPIO_PORTC_DATA_R&(1<<6)) == (1<<6) && (GPIO_PORTC_DATA_R&(1<<5)) == (1<<5))
							{
								Motor_Up();
								setBit(GPIO_PORTF_DATA_R,Green_Led);						
							}
							if((GPIO_PORTC_DATA_R&(1<<5)) == 0x0)
							{
								Motor_Off();
								Leds_Off();	
							}
							else{
								Motor_Off();
								Leds_Off();	
								Motor_Down();
								setBit(GPIO_PORTF_DATA_R,Blue_Led);						
								vTaskDelay(250/portTICK_RATE_MS);
								Motor_Off();
								Leds_Off();	
							}								
						}
						else
						{
								Motor_Off();
								Leds_Off();								
						}
				}						
				else
				{
						Start = xTaskGetTickCount();
						while((GPIO_PORTD_DATA_R&(1<<0)) == 0x00)
						{
							if((GPIO_PORTC_DATA_R&(1<<4)) == (1<<4))
							{
								Motor_Down();
								setBit(GPIO_PORTF_DATA_R, Blue_Led);						
							}
							else
							{
								Motor_Off();
								Leds_Off();					
							}
						}
						End = xTaskGetTickCount();
						if((End - Start) <= 200 && (End - Start) > 0)
						{
							while((GPIO_PORTC_DATA_R&(1<<4)) == (1<<4))
							{
								Motor_Down();
								setBit(GPIO_PORTF_DATA_R,Blue_Led);	
							}
								Motor_Off();
								Leds_Off();	
						}
						else
						{
								Motor_Off();
								Leds_Off();	
						}
				}						
				delayMs(1);
		}
}

static void vPassengerTask ( void *pvParameters )
{
		portBASE_TYPE xStatus;
		uint32_t receive_item;
    xSemaphoreTake( xBinarySemaphore2 , 0);
	
    for( ;; )
    {
			xSemaphoreTake( xBinarySemaphore2 , portMAX_DELAY );
			xStatus = xQueueReceive( xQueue, &receive_item, 0);
			
			if(receive_item == 1);
				
			else
			{
				
				if((GPIO_PORTD_DATA_R&(1<<3)) == 0x00)
				{
						Start = xTaskGetTickCount();
						while((GPIO_PORTD_DATA_R&(1<<3)) == 0x00)
						{
							if((GPIO_PORTC_DATA_R&(1<<5)) == (1<<5))
							{
								Motor_Up();
								setBit(GPIO_PORTF_DATA_R,Green_Led);
							}
							else
							{
								Motor_Off();
								Leds_Off();						
							}
						}
						End = xTaskGetTickCount();
						if((End - Start) <= 200 && (End - Start) > 0)
						{
							while((GPIO_PORTC_DATA_R&(1<<6)) == (1<<6) &&(GPIO_PORTC_DATA_R&(1<<5)) == (1<<5))
							{
								Motor_Up();
								setBit(GPIO_PORTF_DATA_R,Green_Led);
							}
							if((GPIO_PORTC_DATA_R&(1<<5)) == 0x0)
							{
								Motor_Off();
								Leds_Off();	
							}
							else
							{
								Motor_Off();
								Leds_Off();	
								Motor_Down();
								setBit(GPIO_PORTF_DATA_R,Blue_Led);						
								vTaskDelay(250/portTICK_RATE_MS);
								Motor_Off();
								Leds_Off();			
							}

						}
						else
						{
								Motor_Off();
								Leds_Off();		
						}
				}						
				else
				{
						Start = xTaskGetTickCount();
						while((GPIO_PORTD_DATA_R&(1<<2)) == 0x00)
						{
							if((GPIO_PORTC_DATA_R&(1<<4)) == (1<<4))
							{
								Motor_Down();
								setBit(GPIO_PORTF_DATA_R, Blue_Led);
							}
							else
							{
								Motor_Off();
								Leds_Off();							
							}
						}
						End = xTaskGetTickCount();
						if((End - Start) <= 200 && (End - Start) > 0)
						{
							while((GPIO_PORTC_DATA_R&(1<<4)) == (1<<4))
							{
								Motor_Down();
								setBit(GPIO_PORTF_DATA_R, Blue_Led);
							}
								Motor_Off();
								Leds_Off();		
						}
						else
						{
								Motor_Off();
								Leds_Off();		
						}
				}						
				delayMs(1);

			}
		}
}

static void vLockTask( void *pvParameters )
{
		uint32_t Value_ToSend1 = 1;
		portBASE_TYPE xStatus;
    xSemaphoreTake( xBinarySemaphore3, 0 );

    for( ;; )
    {
			xSemaphoreTake( xBinarySemaphore3 , portMAX_DELAY );
			xStatus = xQueueSendToBack(xQueue , & Value_ToSend1 , 0 );
		}
}

static void vUnlockTask( void *pvParameters )
{
		uint32_t Value_ToSend2 = 2;
		portBASE_TYPE xStatus;
    xSemaphoreTake( xBinarySemaphore4 , 0 );

    for( ;; )
    {
			xSemaphoreTake( xBinarySemaphore4 , portMAX_DELAY );
			xStatus = xQueueSendToBack(xQueue , & Value_ToSend2 , 0 );
		}
}

void DIO_A_Init() 
{
		SYSCTL_RCGCGPIO_R |= 0x0000001;
		while((SYSCTL_PRGPIO_R&0x0000001) == 0);
		GPIO_PORTA_DIR_R |=	((1<<2)|(1<<3));
		GPIO_PORTA_DIR_R &=~((1<<5)|(1<<6));
    GPIO_PORTA_DEN_R |=  (1<<5)|(1<<6)|(1<<2)|(1<<3);
    GPIO_PORTA_PUR_R |=  (1<<5)|(1<<6);
    GPIO_PORTA_IS_R  &=~((1<<5)|(1<<6)); 		//interrupt sense edge Trigger
    GPIO_PORTA_IBE_R &= ((1<<5)|(1<<6));		//DISABLE both edges interrupt
    GPIO_PORTA_IEV_R &=~((1<<5)|(1<<6));		//AT falling edge only
    GPIO_PORTA_ICR_R |=  (1<<5)|(1<<6);			// clearing Flag
    GPIO_PORTA_IM_R  |=  (1<<5)|(1<<6);			// not masked
		NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFF00) | 0x00000040;
	  NVIC_EN0_R = (NVIC_EN0_R  & 0xFFFFFFF0) | (1<<0);
}

void DIO_C_Init(void)
{
		SYSCTL_RCGCGPIO_R |= 0x00000004;
		while((SYSCTL_PRGPIO_R&0x00000004) == 0); 
		GPIO_PORTC_CR_R  |= ((1<<4)|(1<<5)|(1<<6));
		GPIO_PORTC_DIR_R &=~ ((1<<4)|(1<<5)|(1<<6));
		GPIO_PORTC_PUR_R |= ((1<<4)|(1<<5)|(1<<6));
		GPIO_PORTC_DEN_R |= ((1<<4)|(1<<5)|(1<<6));
}


void DIO_D_Init(void)
{	 
		SYSCTL_RCGCGPIO_R |= 0x0000008;
		while((SYSCTL_PRGPIO_R&0x0000008) == 0); 
		GPIO_PORTD_DIR_R &=~ ((1<<0) | (1<<1) | (1<<2) | (1<<3));
		GPIO_PORTD_DEN_R |=  ((1<<0) | (1<<1) | (1<<2) | (1<<3));
		GPIO_PORTD_PUR_R |=  ((1<<0) | (1<<1) | (1<<2) | (1<<3));
		GPIO_PORTD_IS_R  &=~ ((1<<0) | (1<<1) | (1<<2) | (1<<3));			// Interrupt sense edge Trigger
		GPIO_PORTD_IBE_R &=~ ((1<<0) | (1<<1) | (1<<2) | (1<<3));			// DISABLE both edges interrupt
		GPIO_PORTD_IEV_R &=~ ((1<<0) | (1<<1) | (1<<2) | (1<<3));			// AT falling edge only
		GPIO_PORTD_ICR_R |=  ((1<<0) | (1<<1) | (1<<2) | (1<<3));			// Clearing Flag
		GPIO_PORTD_IM_R  |=  ((1<<0) | (1<<1) | (1<<2) | (1<<3));			// Not masked
		NVIC_PRI0_R = (NVIC_PRI0_R & 0x00FFFFFF) | 0x60000000;
		NVIC_EN0_R = (NVIC_EN0_R  & 0xFFFFFFF0) | (1<<3); 
}

void PortF_Init(void)
{	 
		SYSCTL_RCGCGPIO_R |= 0x00000020; //Intialize PORTF
		while((SYSCTL_PRGPIO_R&0x00000020) == 0); 
		GPIO_PORTF_LOCK_R = 0x4C4F434B;
		GPIO_PORTF_CR_R = 0x1F;
		GPIO_PORTF_DIR_R = 0x0E;
		GPIO_PORTF_PUR_R = 0x11;
		GPIO_PORTF_DEN_R = 0x1F;
}

void delayMs (uint32_t n)
{
    uint32_t i, j; 
    for(i=0; i < n; i++) 
    {
        for(j=0; j < 3180; j++)
        {
        }
    }
}

void GPIOA_Handler()
{
		portBASE_TYPE xHigherPriorityTaskWoken1 = pdFALSE;
		portBASE_TYPE xHigherPriorityTaskWoken2 = pdFALSE;

	  if((GPIO_PORTA_MIS_R&(1<<5))==(1<<5))
		{
		//Clear Flag
		GPIO_PORTA_ICR_R |= (1<<5);
    // 'Give' the semaphore to unblock the task.
    xSemaphoreGiveFromISR( xBinarySemaphore3, &xHigherPriorityTaskWoken1 );
		//Force Switch
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
		}
		else
		{
		//Clear Flag
		GPIO_PORTA_ICR_R |= (1<<6);
		// 'Give' the semaphore to unblock the task.
    xSemaphoreGiveFromISR( xBinarySemaphore4, &xHigherPriorityTaskWoken2 );
		//Force Switch
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken2 );
		}
}

void GPIOD_Handler()
{
		portBASE_TYPE xHigherPriorityTaskWoken3 = pdFALSE;
		portBASE_TYPE xHigherPriorityTaskWoken4 = pdFALSE;

	  if((GPIO_PORTD_MIS_R&(1<<0))==(1<<0) || (GPIO_PORTD_MIS_R&(1<<1))==(1<<1))
		{
		//Clear Flag
		GPIO_PORTD_ICR_R |= (1<<0)|(1<<1);
    // 'Give' the semaphore to unblock the task.
    xSemaphoreGiveFromISR( xBinarySemaphore1, &xHigherPriorityTaskWoken3 );

		//Force Switch
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken3 );
		}
		else
		{
		//Clear Flag
		GPIO_PORTD_ICR_R |= (1<<2)|(1<<3);
		// 'Give' the semaphore to unblock the task.
    xSemaphoreGiveFromISR( xBinarySemaphore2, &xHigherPriorityTaskWoken4 );
		//Force Switch
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken4 );
		}
}