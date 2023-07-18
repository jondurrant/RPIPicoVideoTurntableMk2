/***
 * Control Turntable and Led Lights
 *
 * Uses FreeRTOS Task
 * Jon Durrant
 * 15-Aug-2022
 */


#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "BlinkAgent.h"
#include "Stepper28BYJ.h"

#include <PicoLed.hpp>
#include "OledDisplay.h"

#include "RotEncListener.h"
#include "RotEncAgent.h"

#include "UI.h"

#include "uRosBridge.h"

extern"C"{
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"
}


//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

//LED PAD to use
#define LED_PAD			2
#define LEDS_PAD		27

//Rotary Encoder
#define ROTENC_SW_PAD	4
#define ROTENC_A_PAD	3
#define ROTENC_B_PAD	5
#define ROTENC_COUNT	20


//Switches
#define SW1_PAD			6
#define SW2_PAD			7
#define SW3_PAD			8
#define SW4_PAD			9


//OLED I2C
#define OLED_SDA_PAD	14
#define OLED_SDL_PAD	15

//Stepper
#define STEPPER_A_PAD		18
#define STEPPER_B_PAD		19
#define STEPPER_C_PAD		20
#define STEPPER_D_PAD		21
#define STEPPER_HALL_PAD	28

/***
 * Debug function to look at Task Stats
 */
void runTimeStats(   ){
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime;


   // Get number of takss
   uxArraySize = uxTaskGetNumberOfTasks();
   printf("Number of tasks %d\n", uxArraySize);

   //Allocate a TaskStatus_t structure for each task.
   pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL ){
      // Generate raw status information about each task.
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

	 // Print stats
	 for( x = 0; x < uxArraySize; x++ )
	 {
		 printf("Task: %d \t cPri:%d \t bPri:%d \t hw:%d \t%s\n",
				pxTaskStatusArray[ x ].xTaskNumber ,
				pxTaskStatusArray[ x ].uxCurrentPriority ,
				pxTaskStatusArray[ x ].uxBasePriority ,
				pxTaskStatusArray[ x ].usStackHighWaterMark ,
				pxTaskStatusArray[ x ].pcTaskName
				);
	 }


      // Free array
      vPortFree( pxTaskStatusArray );
   } else {
	   printf("Failed to allocate space for stats\n");
   }

   //Get heap allocation information
   HeapStats_t heapStats;
   vPortGetHeapStats(&heapStats);
   printf("HEAP avl: %d, blocks %d, alloc: %d, free: %d\n",
		   heapStats.xAvailableHeapSpaceInBytes,
		   heapStats.xNumberOfFreeBlocks,
		   heapStats.xNumberOfSuccessfulAllocations,
		   heapStats.xNumberOfSuccessfulFrees
		   );
}


/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void *params){
	//BlinkAgent blink(LED_PAD);

	printf("Boot task started\n");


	//blink.start("Blink", TASK_PRIORITY);


	Stepper28BYJ stepper(
			STEPPER_A_PAD,
			STEPPER_B_PAD,
			STEPPER_C_PAD,
			STEPPER_D_PAD,
			STEPPER_HALL_PAD);
	stepper.start("Stepper", TASK_PRIORITY+1);


	auto ledStrip = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 0, LEDS_PAD, 10, PicoLed::FORMAT_GRB);
	ledStrip.setBrightness(255);
	ledStrip.fill( PicoLed::RGB(255, 0, 0) );
	ledStrip.show();


	RotEncAgent rotEnc(
			ROTENC_SW_PAD,
			ROTENC_A_PAD,
			ROTENC_B_PAD,
			ROTENC_COUNT);
	rotEnc.start("RotEnc", TASK_PRIORITY);


	//UI
	UI ui;
	ui.initRotEnc(&rotEnc);
	ui.initSws(
			SW1_PAD,
			SW2_PAD,
			SW3_PAD,
			SW4_PAD);
	ui.initStepper(&stepper);
	ui.initOled(
			OLED_SDA_PAD,
			OLED_SDL_PAD);
	ui.initLeds(&ledStrip);
	ui.start("UI", TASK_PRIORITY);


	//Start up a uROS Bridge
	uRosBridge *bridge = uRosBridge::getInstance();

	bridge->setuRosEntities(&stepper);
	bridge->setLed(LED_PAD);
	bridge->start("Bridge",  TASK_PRIORITY+2);

	for (;;){
		vTaskDelay(3000);
	}


}




/***
 * Launch the tasks and scheduler
 */
void vLaunch( void) {

	//Start blink task
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

/***
 * Main
 * @return
 */
int main( void )
{
	//Setup serial over UART and give a few seconds to settle before we start
    stdio_init_all();
    stdio_filter_driver(&stdio_uart);
    sleep_ms(2000);
    printf("GO\n");

    //Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();


    return 0;
}
