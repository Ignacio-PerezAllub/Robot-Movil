#include "main.h"


//#######################################################
#define EX1 1
#define EX2 2
#define EX3 3

#define SYNCHRO_EX EX1

//#######################################################

// Motor command
#define SPEED_COMM 900
#define IR_SEUIL 900

// PI Control
#define Kp 0.001
#define Te 5
#define tau 40
#define Ti (0.1*tau)
#define Ki (Te/Ti)

//#######################################################


// Déclaration des objets synchronisants !! Ne pas oublier de les créer
xSemaphoreHandle xSemaphore = NULL;
xQueueHandle qmotor = NULL;
xQueueHandle qlcd = NULL;

int count = 0;

extern uint8_t rec_buf2[NB_CAR_TO_RECEIVE+1];	 // defined in drv_uart.c
extern uint8_t rec_buf6[NB_CAR_TO_RECEIVE+1];

struct AMessage
{
	char command;
	int data;
};

//========================================================
#if SYNCHRO_EX == EX1

int tab_speedL[100], tab_speedR[100];
float tab_commandL[100], tab_commandR[100];

static void task_Sensors(void *pvParameters)
{
	uint16_t lUS, rUS;
	int obstacle_l, obstacle_r;
	int table[2];
	struct AMessage pxMessage;

	vTaskDelay(5);

	for (;;)
	{
	// Ultrasonic sensor
		if(count == 0)
		{
			captDistUS_Measure(0xE0);
			captDistUS_Measure(0xE2);
			HAL_Delay(70);
		}
		else if (count == 14)
		{
			rUS = captDistUS_Get(0xE0); //right ultrasonic sensor
			lUS = captDistUS_Get(0xE2); //left ultrasonic sensor
			count = 0;
		}
	// IR sensor
		captDistIR_Get(table);
		//Value in mV [500 = 80cm ; 2500 = 10]
		obstacle_l = table[0] > IR_SEUIL; //left IR sensor
		obstacle_r = table[1] > IR_SEUIL; //right IR sensor

	//===== Obstacle Management =====
		//=== IR Sensor - forward motion===

		if(obstacle_l && obstacle_r) //obstacles both sides - back
		{
			pxMessage.command = 'b';
			xQueueSend( qmotor, ( void * ) &pxMessage,  portMAX_DELAY );
			xQueueSend( qlcd, ( void * ) &pxMessage,  portMAX_DELAY );
		}

		else if(obstacle_l && !obstacle_r) //obstacle left side - turn right
		{
			pxMessage.command = 'r';
			xQueueSend( qmotor, ( void * ) &pxMessage,  portMAX_DELAY );
			xQueueSend( qlcd, ( void * ) &pxMessage,  portMAX_DELAY );
		}

		else if(obstacle_r && !obstacle_l) //obstacle right side - turn left
		{
			pxMessage.command = 'l';
			xQueueSend( qmotor, ( void * ) &pxMessage,  portMAX_DELAY );
			xQueueSend( qlcd, ( void * ) &pxMessage,  portMAX_DELAY );
		}
		
		else //no obstacles - retrieve the pre-defined instructions
		{
			pxMessage.command = 'c';
			xQueueSend( qmotor, ( void * ) &pxMessage,  portMAX_DELAY );
			xQueueSend( qlcd, ( void * ) &pxMessage,  portMAX_DELAY );
		}
			
		xSemaphoreTake( xSemaphore, portMAX_DELAY );
		count++;
		vTaskDelay(5);
	}

}

static void task_Motors(void *pvParameters)
{
	int speedLeft, speedRight;
	int errorL , errorR;
	int commL;// command motors
	int commR;
	static int i = 0;
	float upL, upR;
	static float uiL= 0.0;
	static float uiR= 0.0;

	struct AMessage pxRxedMessage;

	for (;;)
	{	
		xQueueReceive( qmotor,  &( pxRxedMessage ) , portMAX_DELAY );

		// According to the received command
		if(pxRxedMessage.command == 'b') //obstacles both sides - back
		{
			commL = -SPEED_COMM / 3;
			commR = -SPEED_COMM / 3;
		}

		else if(pxRxedMessage.command == 'r') //obstacle left side - turn right
		{
			commL = SPEED_COMM;
			commR = -SPEED_COMM / 3;
		}

		else if(pxRxedMessage.command == 'l') //obstacle right side - turn left
		{
			commL = -SPEED_COMM / 3;
			commR = SPEED_COMM;
		}

		else if(pxRxedMessage.command == 'c') //no obstacles - retrieve the pre-defined instructions
		{
			commL = SPEED_COMM;
			commR = SPEED_COMM;
		}

		speedLeft = quadEncoder_GetSpeedL();
		speedRight = quadEncoder_GetSpeedR();
		errorL = commL - speedLeft;
		errorR = commR - speedRight;
		upL = Kp*(float)errorL;
		upR = Kp*(float)errorR;
		uiL += Kp*Ki*(float)errorL;
		uiR += Kp*Ki*(float)errorR;
		//0 < duty < 200
		motorLeft_SetDuty((int) (upL + uiL) + 100);
		motorRight_SetDuty((int) (upR + uiR) + 100);

		if (i < 100)
		{
		tab_speedR[i]= speedRight;
		tab_speedL[i]= speedLeft;
		tab_commandR[i]=upR+uiR;
		tab_commandL[i]=upL+uiL;
		i++;
		}
		//vTaskDelay(5); // 1000 ms
		xSemaphoreGive( xSemaphore );
	}

}
//LCD
static void task_LCD(void *pvParameters)
{
	struct AMessage pxRxedMessage;
	for (;;)
	{
		xQueueReceive( qlcd,  &( pxRxedMessage ) , portMAX_DELAY );
		groveLCD_setColor(4);
		groveLCD_setCursor(0, 0);

		if(pxRxedMessage.command == 'b') //obstacles both sides - back
		{
			groveLCD_term_printf("DIR: Back");
		}

		else if(pxRxedMessage.command == 'r') //obstacle left side - turn right
		{
			groveLCD_term_printf("DIR: Right");
		}

		else if(pxRxedMessage.command == 'l') //obstacle right side - turn left
		{
			groveLCD_term_printf("DIR: Left");
		}

		else if(pxRxedMessage.command == 'c') //no obstacles - retrieve the pre-defined instructions
		{
			groveLCD_term_printf("DIR: Command");
		}
		xSemaphoreGive( xSemaphore);
	}
}

//========================================================
#elif SYNCHRO_EX == EX2

static void task_C( void *pvParameters )
{
	for (;;)
	{
		term_printf("TASK C \n\r");
		xSemaphoreTake( xSemaphore, portMAX_DELAY );
	}
}

static void task_D( void *pvParameters )
{
	for (;;)
	{
		term_printf("TASK D \n\r");
		xSemaphoreGive( xSemaphore );
	}
}

//========================================================
#elif SYNCHRO_EX == EX3

static void task_E( void *pvParameters )
{
	struct SpeedComand  commandRight;
	commandRight.command= commandR;
	commandRight.data=10;

	struct SpeedComand commandLeft;
	commandRight.command= commandL;
	commandRight.data=10;
	vTaskDelay(1000);
	for (;;)
	{
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
		//term_printf("TASK E \n\r");
		xQueueSend( motorR, ( void * ) &pxMessage,  portMAX_DELAY );
		xSemaphoreTake( xSemaphore, portMAX_DELAY );
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
		vTaskDelay(SAMPLING_PERIOD_ms);

	}
}

static void task_F(void *pvParameters)
{
	struct AMessage pxRxedMessage;

	for (;;)
	{
		xQueueReceive( qh,  &( pxRxedMessage ) , portMAX_DELAY );
		term_printf("TASK F \n\r");
		xSemaphoreGive( xSemaphore );
	}
}
#endif
//=========================================================
//	>>>>>>>>>>>>	MAIN	<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//=========================================================

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	uart2_Init();			// CABLE
	uart6_Init();			// ZIGBEE
	i2c1_Init();
	spi1Init();
	pixyCam2_Init();			// Caméra Pixy
	captDistIR_Init();		// Capteurs Infrarouge
	quadEncoder_Init();		// Encodeurs Incrémentaux
	motorCommand_Init();	// Commande des Hacheurs
	servoCommand_Init();	// Commande des Servomoteurs

	// Test Ecran LCD
	//int a, b;
	groveLCD_begin(16,2,0); // !! cette fonction prend du temps
	HAL_Delay(100);
	groveLCD_setColor(5);
	//a=5; b=2;
	//groveLCD_term_printf("%d+%d=%d",a,b,a+b);
	groveLCD_term_printf("Robot Mobile");

	HAL_Delay(1000);

	//pixyCam2_Test();  // !! boucle infinie

	motorLeft_SetDuty(100);
	motorRight_SetDuty(100);

	char text[]="Hello World\r\n";
	term_printf_zigbee(text);



#if SYNCHRO_EX == EX1
	xTaskCreate( task_Sensors, ( const portCHAR * ) "task Sensors", 512 /* stack size */, NULL, tskIDLE_PRIORITY+1, NULL );
	xTaskCreate( task_Motors, ( const portCHAR * ) "task Motors", 512 /* stack size */, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( task_LCD, ( const portCHAR * ) "task LCD", 512 /* stack size */, NULL, tskIDLE_PRIORITY+2, NULL );

#elif SYNCHRO_EX == EX2
	xTaskCreate( task_C, ( signed portCHAR * ) "task C", 512 /* stack size */, NULL, tskIDLE_PRIORITY+2, NULL );
	xTaskCreate( task_D, ( signed portCHAR * ) "task D", 512 /* stack size */, NULL, tskIDLE_PRIORITY+1, NULL );
#elif SYNCHRO_EX == EX3
	xTaskCreate( task_E, ( signed portCHAR * ) "task E", 512 /* stack size */, NULL, tskIDLE_PRIORITY+2, NULL );
	xTaskCreate( task_F, ( signed portCHAR * ) "task F", 512 /* stack size */, NULL, tskIDLE_PRIORITY+1, NULL );
#endif

	vSemaphoreCreateBinary(xSemaphore);
	xSemaphoreTake( xSemaphore, portMAX_DELAY );

	qmotor = xQueueCreate( 1, sizeof(struct AMessage ) );
	qlcd = xQueueCreate( 1, sizeof(struct AMessage ) );

	vTaskStartScheduler();

	return 0;

}


//=================================================================
//	UART RECEIVE CALLBACK5
//=================================================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle -> Instance == USART2)
	{

		 HAL_UART_Receive_IT(&Uart2Handle, (uint8_t *)rec_buf2, NB_CAR_TO_RECEIVE);

	}

	if(UartHandle -> Instance == USART6)
	{
		term_printf_zigbee(rec_buf6);
		 HAL_UART_Receive_IT(&Uart6Handle, (uint8_t *)rec_buf6, NB_CAR_TO_RECEIVE);
	}

}



//=================================================================
// Called if stack overflow during execution
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	//term_printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}
//=================================================================
//This function is called by FreeRTOS idle task
extern void vApplicationIdleHook(void)
{
}
//=================================================================
// brief This function is called by FreeRTOS each tick
extern void vApplicationTickHook(void)
{
//	HAL_IncTick();
}