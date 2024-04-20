/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_gpio.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include <lin1d3_driver.h>
#include "FreeRTOSConfig.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define xJUST_MASTER
#define DEBUG_LIN

#define BOARD_RED_GPIO     			BOARD_LED_RED_GPIO
#define BOARD_RED_GPIO_PIN 			BOARD_LED_RED_GPIO_PIN
#define BOARD_GREEN_GPIO     		BOARD_LED_GREEN_GPIO
#define BOARD_GREEN_GPIO_PIN 		BOARD_LED_GREEN_GPIO_PIN
#define BOARD_BLUE_GPIO    			BOARD_LED_BLUE_GPIO
#define BOARD_BLUE_GPIO_PIN 		BOARD_LED_BLUE_GPIO_PIN

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define LOCAL_SLAVE_UART UART3
#define LOCAL_SLAVE_UART_CLKSRC UART3_CLK_SRC
#define LOCAL_SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define LOCAL_SLAVE_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART4
#define SLAVE_UART_CLKSRC UART4_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

#define app_message_id_1_d 1
#define app_message_id_2_d 2
#define app_message_id_3_d 3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);

static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);

static void	message_1_callback_local_slave(void* message);
static void	message_2_callback_local_slave(void* message);
static void	message_3_callback_local_slave(void* message);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
	/* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput,
		0,
	};

    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);

    /* Init LEDs GPIO. */
    GPIO_PinInit(BOARD_RED_GPIO, BOARD_RED_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_GREEN_GPIO, BOARD_GREEN_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_BLUE_GPIO, BOARD_BLUE_GPIO_PIN, &led_config);

    /* Clean LEDs */
    GPIO_PinWrite(BOARD_RED_GPIO, BOARD_RED_GPIO_PIN, 1U);
    GPIO_PinWrite(BOARD_GREEN_GPIO, BOARD_GREEN_GPIO_PIN, 1U);
    GPIO_PinWrite(BOARD_BLUE_GPIO, BOARD_BLUE_GPIO_PIN, 1U);

    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    PRINTF(" *** LIN driver demo ***\r\n");
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{
	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* slave_handle;
	lin1d3_handle_t* local_slave_handle;
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#if !defined(JUST_MASTER)
	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 0;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = 0;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);

	/* Set local Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = LOCAL_SLAVE_UART;
	node_config.srcclk = LOCAL_SLAVE_UART_CLK_FREQ;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = message_1_callback_local_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = 1;
	node_config.messageTable[1].handler = message_2_callback_local_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = 1;
	node_config.messageTable[2].handler = message_3_callback_local_slave;
	node_config.skip_uart_init = 1;
	node_config.uart_rtos_handle = master_handle->uart_rtos_handle;
	/* Init local Slave Node*/
	local_slave_handle = lin1d3_InitNode(node_config);

#endif

	if((NULL == master_handle)
#if !defined(JUST_MASTER)
		|| (NULL == slave_handle)
		|| (NULL == local_slave_handle)
#endif
	   ){
		PRINTF(" Init failed!! \r\n");
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	while (kStatus_Success == error)
    {
#ifdef DEBUG_LIN
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	vTaskDelay(100);
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(100);
    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
#endif
    }

    vTaskSuspend(NULL);
}

static void	message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 1 request\r\n");
	message_data[0] = 1;
}

static void	message_2_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 2 request\r\n");
}

static void	message_3_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 request\r\n");
}

static void	message_1_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local slave got message 1 request\r\n");
	PRINTF("Data: %d, %d, %d\r\n", message_data[0], message_data[1], message_data[2]);
	if(*message_data == 1){
		GPIO_PinWrite(BOARD_RED_GPIO, BOARD_RED_GPIO_PIN, 0U);
		GPIO_PinWrite(BOARD_GREEN_GPIO, BOARD_GREEN_GPIO_PIN, 1U);
		GPIO_PinWrite(BOARD_BLUE_GPIO, BOARD_BLUE_GPIO_PIN, 1U);
	}
	else if(*message_data == 2){
		GPIO_PinWrite(BOARD_RED_GPIO, BOARD_RED_GPIO_PIN, 1U);
		GPIO_PinWrite(BOARD_GREEN_GPIO, BOARD_GREEN_GPIO_PIN, 0U);
		GPIO_PinWrite(BOARD_BLUE_GPIO, BOARD_BLUE_GPIO_PIN, 1U);
	}
	else if(*message_data == 3){
		GPIO_PinWrite(BOARD_RED_GPIO, BOARD_RED_GPIO_PIN, 1U);
		GPIO_PinWrite(BOARD_GREEN_GPIO, BOARD_GREEN_GPIO_PIN, 1U);
		GPIO_PinWrite(BOARD_BLUE_GPIO, BOARD_BLUE_GPIO_PIN, 0U);
	}
	else{
		GPIO_PinWrite(BOARD_RED_GPIO, BOARD_RED_GPIO_PIN, 1U);
		GPIO_PinWrite(BOARD_GREEN_GPIO, BOARD_GREEN_GPIO_PIN, 1U);
		GPIO_PinWrite(BOARD_BLUE_GPIO, BOARD_BLUE_GPIO_PIN, 1U);
	}
}

static void	message_2_callback_local_slave(void* message){
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local slave got message 2 request\r\n");
}

static void	message_3_callback_local_slave(void* message){
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local slave got message 3 request\r\n");
}
