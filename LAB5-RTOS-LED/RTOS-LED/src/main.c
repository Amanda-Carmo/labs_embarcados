/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_LED1_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_LED2_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED2_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_LED3_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED3_STACK_PRIORITY (tskIDLE_PRIORITY)

#define LED1_PIO       PIOA
#define LED1_PIO_ID    ID_PIOA
#define LED1_IDX       0
#define LED1_IDX_MASK  (1u << LED1_IDX)

#define LED2_PIO       PIOC
#define LED2_PIO_ID    ID_PIOC
#define LED2_IDX       30
#define LED2_IDX_MASK  (1u << LED2_IDX)

#define LED3_PIO       PIOB
#define LED3_PIO_ID    ID_PIOB
#define LED3_IDX       2
#define LED3_IDX_MASK  (1u << LED3_IDX)

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         ID_PIOD
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

#define BUT3_PIO            PIOA
#define BUT3_PIO_ID         ID_PIOA
#define BUT3_PIO_IDX        19
#define BUT3_PIO_IDX_MASK   (1u << BUT3_PIO_IDX)

/** Semaforo a ser usado pela task led 
    tem que ser var global! */
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore_1;
SemaphoreHandle_t xSemaphore_2;
SemaphoreHandle_t xSemaphore_3;

volatile char but3_flag;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void pin_toggle(Pio *pio, uint32_t mask);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/**                                                               
* callback do botao                                               
* libera semaforo: xSemaphore                                    
*/
void but1_callback(void){
    printf("but_callback \n");
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
	xSemaphoreGiveFromISR(xSemaphore_1, &xHigherPriorityTaskWoken);
	but3_flag = 0;
	
}

void but3_callback(void){
    printf("but_callback \n");
	// BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// xSemaphoreGiveFromISR(xSemaphore_1, &xHigherPriorityTaskWoken);
	but3_flag = 1;
	
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);
	const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;

	for (;;) {
		printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(xDelay);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */


static void task_led(void *pvParameters) {

  for (;;) {
    // aguarda sem??foro com timeout de  500 ms
    if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500 / portTICK_PERIOD_MS) == pdTRUE ){
      LED_Toggle(LED0);
    } else {
      // time out
    }
  }
}

void pisca_led(int n, int t){
  for (int i=0;i<n;i++){
    pio_set(LED1_PIO, LED1_IDX_MASK);
    vTaskDelay(t);
	pio_clear(LED1_PIO, LED1_IDX_MASK);
    vTaskDelay(t);
  }
}

void pisca_led2(int n, int t){
  for (int i=0;i<n;i++){
    pio_set(LED2_PIO, LED2_IDX_MASK);
    vTaskDelay(t);
	pio_clear(LED2_PIO, LED2_IDX_MASK);
    vTaskDelay(t);
  }
}

void pisca_led3(int n, int t){
  for (int i=0;i<n;i++){
    pio_set(LED3_PIO, LED3_IDX_MASK);
    vTaskDelay(t);
	pio_clear(LED3_PIO, LED3_IDX_MASK);
    vTaskDelay(t);
  }
}

static void task_led1(void *pvParameters) {
  for (;;) {
    // aguarda sem??foro com timeout de  500 ms
	
    if( xSemaphoreTake(xSemaphore_1, ( TickType_t ) 500 / portTICK_PERIOD_MS) == pdTRUE ){
      pisca_led(3, 200);
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR(xSemaphore_2, &xHigherPriorityTaskWoken); 
    } else {
      // time out
    }
  }  
}

static void task_led2(void *pvParameters) {
  for (;;) {
    // aguarda sem??foro com timeout de  500 ms
    if( xSemaphoreTake(xSemaphore_2, ( TickType_t ) 500 / portTICK_PERIOD_MS) == pdTRUE ){
      pisca_led2(3, 200);
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR(xSemaphore_3, &xHigherPriorityTaskWoken); 
    } else {
      // time out
    }
  }
}

static void task_led3(void *pvParameters) {
  for (;;) {
    // aguarda sem??foro com timeout de  500 ms
    if(but3_flag && (xSemaphoreTake(xSemaphore_3, ( TickType_t ) 500 / portTICK_PERIOD_MS) == pdTRUE) ){
		pisca_led3(3, 200);
		but3_flag = 0;
    } else {
      // time out
    }
  }
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	
	// Inicializa PIO do led do Xplained
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_IDX_MASK, 0, 0, 0);	

	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_IDX_MASK, 0, 0, 0);	

	  /*
   * devemos iniciar a interrupcao no pino somente apos termos alocado
   * os recursos (no caso semaforo), nessa funcao inicializamos 
   * o botao e seu callback
  */
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but1_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4

	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but3_callback);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

	  /*
   * We are using the semaphore for synchronisation so we create a binary
   * semaphore rather than a mutex.  We must make sure that the interrupt
   * does not attempt to use the semaphore before it is created!
   */
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphore_1 = xSemaphoreCreateBinary();
	xSemaphore_2 = xSemaphoreCreateBinary();
	xSemaphore_3 = xSemaphoreCreateBinary();

	if (xSemaphore == NULL)
		printf("Falha em criar o semaforo \n");


	if (xSemaphore_1 == NULL)
		printf("Falha em criar o semaforo 1 \n");

	if (xSemaphore_2 == NULL)
		printf("Falha em criar o semaforo 2 \n");

	if (xSemaphore_3 == NULL)
		printf("Falha em criar o semaforo 3 \n");

	init();

	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}


	if (xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create led 1 task\r\n");
	}

	if (xTaskCreate(task_led2, "Led2", TASK_LED2_STACK_SIZE, NULL, TASK_LED2_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create led 2 task\r\n");
	}

	if (xTaskCreate(task_led3, "Led3", TASK_LED3_STACK_SIZE, NULL, TASK_LED3_STACK_PRIORITY, NULL) != pdPASS){
		printf("Failed to create led 3 task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
