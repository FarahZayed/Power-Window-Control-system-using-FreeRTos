/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "tm4c123gh6pm.h"
#include "DIO.h"
#include "semphr.h"
#include "queue.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_sysctl.h"


#define STOP 0
#define UPReleasePass 1
#define DOWNReleasePass 2
#define UPReleaseDriver 3
#define DOWNReleaseDriver 4
#define UPShort 5
#define DownShort 6


int duty_cycle = 3750;
int Direction=0;
int xReceivedStructure;

int count=0;



void WindowUpReleaseDriver(void);
void WindowDownRealseDriver(void);
void WindowUpReleasePass(void);
void WindowDownRealsePass(void);
void Stopmotor(void);
void WindowUpshort(void *pvParameters);
void WindowDownshort(void *pvParameters);
void Delay_ms(int time_ms);
void PWM_init(void);
void DC_Motor(void *pvParameters);
void systemHandler(void *pvParameters);
void GPIOD_Handler(void);
void Jamming(void *pvParameters);
void WindowUPshort(void);
void WindowDOWNshort(void);
/*-----------------------------------------------------------*/
xQueueHandle xQueue;
xSemaphoreHandle xBinarySemaphore1;
xSemaphoreHandle xMutex;
/*-----------------------------------------------------------*/

void PWM_init(void)
{

    DIO_Init(PORTF);
    DIO_Init(PORTA); 
    SYSCTL_RCGCPWM_R |=0x2 ;       
	  while((READ_BIT(SYSCTL_RCGCPWM_R,1)) == 0){};
	  SYSCTL_RCC_R |= (1<<20);   
    SYSCTL_RCC_R |= 0x000E0000; 
   
   
   	GPIO_PORTF_AFSEL_R |= (1<<2);        
    GPIO_PORTF_PCTL_R &= ~0x00000F00; 
    GPIO_PORTF_PCTL_R |= 0x00000500; 
    GPIO_PORTF_DEN_R |= (1<<2);         
    
	
    PWM1_3_CTL_R&= ~(1<<0);    
	  PWM1_3_CTL_R &= ~(1<<1);   
    PWM1_3_GENA_R= 0x0000008C; 
    PWM1_3_LOAD_R = 5000;     
    PWM1_3_CMPA_R= 3750;     
    PWM1_3_CTL_R = 1;         
    PWM1_ENABLE_R= 0x40;    
		
		GPIO_PORTA_DIR_R |= (1<<3)|(1<<2);            
    GPIO_PORTA_DEN_R |= (1<<3)|(1<<2); 
		
		
		//portE
		DIO_Init(PORTE);
		DIO_DIR(PORTE,PIN2,0);
		DIO_DIR(PORTE,PIN4,0);
		DIO_DIR(PORTA,PIN7,0);  //limit switch
		DIO_DIR(PORTF,PIN1,1);
		GPIO_PORTE_AMSEL_R=0x00;
		GPIO_PORTE_PCTL_R = 0x00000000; 
		GPIO_PORTE_AFSEL_R = 0x00; 
	  DIO_DIR(PORTA,PIN7,0);  //limit switch
		DIO_DIR(PORTA,PIN4,0);
		
		DIO_DIR(PORTA,PIN5,0); //child lock
			
		//portB 
		DIO_Init(PORTB);
		DIO_DIR(PORTB,PIN6,0);
		DIO_DIR(PORTB,PIN4,0);
		DIO_DIR(PORTF,PIN1,1);
		DIO_DIR(PORTB,PIN7,0);
		GPIO_PORTB_AMSEL_R=0x00;
		GPIO_PORTB_PCTL_R = 0x00000000; 
		GPIO_PORTB_AFSEL_R = 0x00; 
		
		DIO_Init(PORTD);
		DIO_DIR(PORTD,PIN2,0);
		GPIO_PORTD_AMSEL_R=0x00;
		GPIO_PORTD_PCTL_R = 0x00000000; 
		GPIO_PORTD_AFSEL_R = 0x00; 
		GPIO_PORTD_ICR_R |=(1<<2) ;  
		GPIO_PORTD_IM_R |=(1<<2); 
		GPIO_PORTD_IS_R |= (1<<2); 
		GPIO_PORTD_IEV_R &= ~(1<<2); 
		NVIC_EN0_R = (1<<3);

		
}
/*-----------------------------------------------------------*/

void WindowUpReleaseDriver(void){
	xSemaphoreTake( xMutex, portMAX_DELAY );
	{
	while(1){
		 GPIO_PORTA_DATA_R |=(1<<3);  
		 GPIO_PORTA_DATA_R &=~(1<<2);
		GPIO_PORTA_DATA_R=0x08;
		 duty_cycle = duty_cycle - 10;
     if (duty_cycle <= 0) {duty_cycle = 5000;}
     PWM1_3_CMPA_R = duty_cycle;
     Delay_ms(100);
		 if((READ_BIT(GPIO_PORTB_DATA_R,PIN6))==1){
			 Stopmotor();
			 break;
		 }
	 }
		}
	xSemaphoreGive( xMutex );

}
void WindowDownReleaseDriver(void){
	xSemaphoreTake( xMutex, portMAX_DELAY );
	{
	while(1){
	GPIO_PORTA_DATA_R |=(1<<2);  
	GPIO_PORTA_DATA_R &=~(1<<3);
	GPIO_PORTA_DATA_R=0x04;
	duty_cycle = duty_cycle - 100;
  if (duty_cycle <= 0)  {duty_cycle = 5000;}
  PWM1_3_CMPA_R = duty_cycle;
  Delay_ms(100);
	if((READ_BIT(GPIO_PORTB_DATA_R,PIN4))==1) 	{
			 Stopmotor();
				GPIO_PORTB_DATA_R=0x10;
		GPIO_PORTE_DATA_R=0x10;
			 break;
		 }

	}	}
	xSemaphoreGive( xMutex );

}

void WindowUpReleasePass(void){
	xSemaphoreTake( xMutex, portMAX_DELAY );
	{
	while(1){
		 GPIO_PORTA_DATA_R |=(1<<3);  
		 GPIO_PORTA_DATA_R &=~(1<<2);
		GPIO_PORTA_DATA_R=0x08;
		 duty_cycle = duty_cycle - 10;
     if (duty_cycle <= 0) {duty_cycle = 5000;}
     PWM1_3_CMPA_R = duty_cycle;
     Delay_ms(100);
		 if((READ_BIT(GPIO_PORTE_DATA_R,PIN2))==1){
			 Stopmotor();

			 Delay_ms(250);
			 break;
		 }
	 }
		}
	xSemaphoreGive( xMutex );

}
void WindowDownReleasePass(void){
	xSemaphoreTake( xMutex, portMAX_DELAY );
	{
	while(1){
	GPIO_PORTA_DATA_R |=(1<<2);  
	GPIO_PORTA_DATA_R &=~(1<<3);
	GPIO_PORTA_DATA_R=0x04;
	duty_cycle = duty_cycle - 100;
  if (duty_cycle <= 0)  {duty_cycle = 5000;}
  PWM1_3_CMPA_R = duty_cycle;
  Delay_ms(100);
	if((READ_BIT(GPIO_PORTE_DATA_R,PIN4))==1) 	{
			 Stopmotor();
				GPIO_PORTB_DATA_R=0x10;
		GPIO_PORTE_DATA_R=0x10;
			 break;
		 }
}
		}
	xSemaphoreGive( xMutex );
	
}

void Stopmotor(void){
	xSemaphoreTake( xMutex, portMAX_DELAY );
	{
	GPIO_PORTA_DATA_R &=~ (1<<2);  
	GPIO_PORTA_DATA_R &=~(1<<3);
	GPIO_PORTA_DATA_R=0x0;
	Direction=0;
  xReceivedStructure=0;
			}
	xSemaphoreGive( xMutex );

}

void Delay_ms(int time_ms)
{
    int i;
    for(i = 0 ; i < time_ms; i++){}  /* excute NOP for 1ms */
}

void WindowUPshort(void){
	xSemaphoreTake( xMutex, portMAX_DELAY );
	{
	while(1){
	GPIO_PORTA_DATA_R |=(1<<3);  
		 GPIO_PORTA_DATA_R &=~(1<<2);
		GPIO_PORTA_DATA_R=0x08;
		 duty_cycle = duty_cycle - 10;
     if (duty_cycle <= 0) {duty_cycle = 5000;}
     PWM1_3_CMPA_R = duty_cycle;
     Delay_ms(100);
	if((READ_BIT(GPIO_PORTA_DATA_R,PIN7))==1) 	{
			 Stopmotor();
			 break;
	}
}
		}
	xSemaphoreGive( xMutex );

}

void WindowDOWNshort(void){
	xSemaphoreTake( xMutex, portMAX_DELAY );
	{
	while(1){
	GPIO_PORTA_DATA_R |=(1<<3);  
		 GPIO_PORTA_DATA_R &=~(1<<2);
		GPIO_PORTA_DATA_R=0x08;
		 duty_cycle = duty_cycle - 10;
     if (duty_cycle <= 0) {duty_cycle = 5000;}
     PWM1_3_CMPA_R = duty_cycle;
     Delay_ms(100);
	if((READ_BIT(GPIO_PORTA_DATA_R,PIN4))==1) 	{
			 Stopmotor();
			 break;
	}
}
		}
	xSemaphoreGive( xMutex );

}

/*-----------------------------------------------------------*/
static TickType_t pressTime = 0;
const TickType_t longTime = pdMS_TO_TICKS(500);
void Jamming(void *pvParameters){
	  xSemaphoreTake(xBinarySemaphore1, 0 );
	while(1){
				xSemaphoreTake(xBinarySemaphore1, portMAX_DELAY );
		    Stopmotor();
		    pressTime = xTaskGetTickCount();
		    TickType_t nowTime = xTaskGetTickCount(); // Record the current tick count when button is released
			xSemaphoreTake( xMutex, portMAX_DELAY );
	{
        while(nowTime-pressTime<longTime){
					GPIO_PORTA_DATA_R |=(1<<2);  
					GPIO_PORTA_DATA_R &=~(1<<3);
				 GPIO_PORTA_DATA_R=0x08;
					duty_cycle = duty_cycle - 10;
					if (duty_cycle <= 0) {duty_cycle = 5000;}
					PWM1_3_CMPA_R = duty_cycle;
					Delay_ms(100);
					nowTime = xTaskGetTickCount(); 
				 } 
				 Stopmotor();
	}	
	xSemaphoreGive( xMutex );
}
}

void DC_Motor(void *pvParameters)
{
	
 while(1){
	
	portBASE_TYPE xStatus;
	xStatus = xQueueReceive( xQueue, &xReceivedStructure, 0 );
	if(xReceivedStructure ==UPReleaseDriver)
	{
		WindowUpReleaseDriver();

	}
	if(xReceivedStructure==DOWNReleaseDriver){
	  WindowDownReleaseDriver();
	}
		if(xReceivedStructure ==UPReleasePass)
	{
		WindowUpReleasePass();

	}
	if(xReceivedStructure==DOWNReleasePass){
	  WindowDownReleasePass();
	}
	if(xReceivedStructure==UPShort){
		 WindowUPshort();
	
	}
	if(xReceivedStructure ==DownShort){
		WindowDOWNshort();
	}
}
}

void systemHandler(void *pvParameters){
  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
  while(1){
		
	if((READ_BIT(GPIO_PORTB_DATA_R,PIN6))==0){
		

		  vTaskDelay(500/portTICK_RATE_MS);
		if((READ_BIT(GPIO_PORTB_DATA_R,PIN6))==0){

		   Direction=UPReleaseDriver;
       xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
			count =1;
		 }
		 else {
			 if(count==0){
			 Direction=UPShort;
			xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
			 }
			 else{
				 count=0;
			 }
		 }

		 }


	

	
	if((READ_BIT(GPIO_PORTB_DATA_R,PIN4))==0){

			vTaskDelay(500/portTICK_RATE_MS);

			 if((READ_BIT(GPIO_PORTB_DATA_R,PIN4))==0){
				 Direction=DOWNReleaseDriver;
				 xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
				count=1;
			 }
			 else{
      if(count==0){
			 Direction=DownShort;
     	xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
			 }
			 else{
				 count=0;
			 }
		 }

	}
	if((READ_BIT(GPIO_PORTE_DATA_R,PIN2))==0 && (READ_BIT(GPIO_PORTA_DATA_R,PIN5))==1){

		 vTaskDelay(500/portTICK_RATE_MS);
		 if((READ_BIT(GPIO_PORTE_DATA_R,PIN2))==0){
	
		   Direction=UPReleasePass;
       xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
			count=1;
		 }
		 else{
		 if(count==0){
			 Direction=UPShort;
			  xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
			 }
			 else{
				 count=0;
			 }
		 }

	}
	
	if((READ_BIT(GPIO_PORTE_DATA_R,PIN4))==0 && (READ_BIT(GPIO_PORTA_DATA_R,PIN5))==1){
		
			 vTaskDelay(500/portTICK_RATE_MS);

			 if((READ_BIT(GPIO_PORTE_DATA_R,PIN4))==0){
			
				 Direction=DOWNReleasePass;
				 xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
					count=1;
			 }
			 else{
			 if(count==0){
			 Direction=DownShort;
			  xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
			 }
			 else{
				 count=0;
			 }
		 }
	}
	
	}

		
}



/*-----------------------------------------------------------*/

int main( void )
{

		 __asm("CPSIE i");
    vSemaphoreCreateBinary( xBinarySemaphore1 );
	  xQueue = xQueueCreate( 7, sizeof( Direction ) );
	  xMutex = xSemaphoreCreateMutex();
   if(xQueue != NULL  && xBinarySemaphore1 !=NULL &&  xMutex!=NULL)
    {	
	   PWM_init();
     xTaskCreate( DC_Motor, " DC Motor ", 90, NULL,1, NULL );
		 xTaskCreate( systemHandler, "system handling ", 90,  ( void * ) &(Direction ), 2, NULL );
			xTaskCreate( Jamming, "jam ", 90,  NULL, 4, NULL );
	   vTaskStartScheduler();
		}
    for(;;);
		 

}



/*-----------------------------------------------------------*/
//interrupt jamming handler 
void GPIOD_Handler(void){
	if(Direction==UPReleaseDriver || Direction==UPReleasePass || Direction==UPShort){
   portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( xBinarySemaphore1, &xHigherPriorityTaskWoken );
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
 }
} 
		

/*-----------------------------------------------------------*/


void HardFault_Handler(void){}


/*-----------------------------------------------------------*/

