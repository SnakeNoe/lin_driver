/*
 * lin1d3_driver.c
 *
 *  Created on: Sep 14, 2018
 *      Author: Nico
 */
#include "lin1d3_driver.h"
#include <string.h>
#include <fsl_debug_console.h>

#define ID0_MASK 	1 << 7
#define ID0			7
#define ID1_MASK 	1 << 6
#define ID1			6
#define ID2_MASK 	1 << 5
#define ID2			5
#define ID3_MASK 	1 << 4
#define ID3			4
#define ID4_MASK 	1 << 3
#define ID4			3
#define ID5_MASK 	1 << 2
#define ID5			2
#define P1_MASK		1 << 1
#define P0_MASK		1 << 0

#define master_stack_size_d	(256)
#define master_task_priority (configMAX_PRIORITIES - 1)
#define master_queue_size_d	(8)

#define slave_stack_size_d	(256)
#define slave_task_priority (configMAX_PRIORITIES - 1)

#define size_of_uart_buffer	(10)

#define size_of_lin_header_d (2)

/*Static function prototypes */
static void master_task(void *pvParameters);
static void slave_task(void *pvParameters);

static uint8_t calculateChecksum(uint8_t datas[size_of_uart_buffer], uint8_t size);

/******************************************************************************
 * Public functions
 *
 *****************************************************************************/

/*
 * Init a LIN node
 * */
lin1d3_handle_t* lin1d3_InitNode(lin1d3_nodeConfig_t config)
{
	lin1d3_handle_t* handle = NULL;
	static uint8_t node_idx = 0;
	char master_task_name[] = "linMaster0";
	char slave_task_name[] = "linSlave0";
	/* Check init parameters */
	if(config.type >= lin1d3_max_nodeType) {
		return NULL;
	}

	/* Create the handle structure and */
	handle = (lin1d3_handle_t*)pvPortMalloc(sizeof(lin1d3_handle_t));
	if(handle ==  NULL) {
		/* Failed to allocate memory for the node handle */
		return NULL;
	}
	/* Init the handle structure with 0s */
	memset(handle, 0, sizeof(lin1d3_handle_t));
	/* Copy the config */
	memcpy(&(handle->config), &config, sizeof(lin1d3_nodeConfig_t));

	/* Init/Configure the UART */
	handle->uart_config.base = handle->config.uartBase;
	handle->uart_config.srcclk = handle->config.srcclk;
	handle->uart_config.baudrate = handle->config.bitrate;
	handle->uart_config.parity = kUART_ParityDisabled;
	handle->uart_config.stopbits = kUART_OneStopBit;
	handle->uart_config.buffer = pvPortMalloc(size_of_uart_buffer);
	handle->uart_config.buffer_size = size_of_uart_buffer;
	if(handle->uart_config.buffer == NULL){
		return NULL;
	}

	if(config.skip_uart_init == 0) {
		/* Create the handle UART handle structures */
		handle->uart_rtos_handle = (uart_rtos_handle_t*)pvPortMalloc(sizeof(uart_rtos_handle_t));
		if(handle->uart_rtos_handle ==  NULL) {
			/* Failed to allocate memory for the node handle */
			return NULL;
		}

		handle->uart_handle = (uart_handle_t*)pvPortMalloc(sizeof(uart_handle_t));
		if(handle->uart_handle ==  NULL) {
			/* Failed to allocate memory for the node handle */
			return NULL;
		}

		if (0 > UART_RTOS_Init(handle->uart_rtos_handle, handle->uart_handle, &(handle->uart_config)))
		{
			return NULL;
		}
	}
	else {
		handle->uart_rtos_handle = config.uart_rtos_handle;
	}
	/* Create the Node Task */
	if(lin1d3_master_nodeType == config.type) {
		/* Create a queue for User message requests */
		handle->node_queue = xQueueCreate( master_queue_size_d, sizeof(uint8_t));
		if(handle->node_queue == NULL){
			vPortFree(handle);
			return NULL;
		}
		/* Create a task for the node */
		master_task_name[strlen(master_task_name)-1] += node_idx++;
		if (xTaskCreate(master_task, master_task_name, master_stack_size_d, handle, master_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}
	else if(lin1d3_slave_nodeType == config.type) {
		/* Create a task for the node */
		slave_task_name[strlen(slave_task_name)-1] += node_idx++;
		if (xTaskCreate(slave_task, slave_task_name, slave_stack_size_d, handle, slave_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}

	return handle;
}

/*
 * Send a message frame from a LIN Master node
 * */
uint32_t lin1d3_masterSendMessage(lin1d3_handle_t* handle, uint8_t ID)
{
	if(handle !=  NULL) {
		/* Put the requested ID on the master queue */
		xQueueSend( handle->node_queue, &ID, ( TickType_t ) 0 );
	}
	return 0;
}

/******************************************************************************
 * Static functions
 *
 *****************************************************************************/
static void master_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  synch_break_byte = 0;
	uint8_t  lin1p3_header[] = {0x55, 0x00};
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

    while(1) {
    	/* Wait for messages on the Queue */
        if(xQueueReceive(handle->node_queue, &ID, portMAX_DELAY)){
        	/* Build and send the LIN Header */
        	/* Put the ID into the header */
        	lin1p3_header[1] = ID<<2;
        	// TODO: Parity
        	// P0 parity
			lin1p3_header[1] = (((ID0_MASK & lin1p3_header[1]) >> ID0) ^ ((ID1_MASK & lin1p3_header[1]) >> ID1) ^
        			((ID2_MASK & lin1p3_header[1]) >> ID2) ^ ((ID4_MASK & lin1p3_header[1]) >> ID4)) | lin1p3_header[1];
        	// P1 parity
        	lin1p3_header[1] = ((((ID1_MASK & lin1p3_header[1]) >> ID1) ^ ((ID3_MASK & lin1p3_header[1]) >> ID3) ^
        	        			((ID4_MASK & lin1p3_header[1]) >> ID4) ^ ((ID5_MASK & lin1p3_header[1]) >> ID5)) << 1) | lin1p3_header[1];

        	/* Init the message recevie buffer */
        	memset(lin1p3_message, 0, size_of_uart_buffer);
        	/* Calc the message size */
        	switch(ID&0x03) {
        		case 0x00: message_size = 2;
        		break;
        		case 0x01: message_size = 2;
        		break;
        		case 0x02: message_size = 4;
        		break;
        		case 0x03: message_size = 8;
        		break;
        	}
        	message_size+=1;
        	/* Send a Break It is just sending one byte 0, *** CHANGE THIS WITH A REAL SYNCH BREAK ****/
        	UART_RTOS_Send(handle->uart_rtos_handle, (uint8_t *)&synch_break_byte, 1);
        	vTaskDelay(1);
        	/* Send the header */
        	UART_RTOS_Send(handle->uart_rtos_handle, (uint8_t *)lin1p3_header, size_of_lin_header_d);
        	vTaskDelay(1);
        }
    }
}

static void slave_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1p3_header[size_of_lin_header_d];
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;
	size_t n;
	uint8_t  msg_idx;
	uint8_t synch_break_byte = 0;
	uint8_t p0;
	uint8_t p1;
	uint8_t checksum = 0;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

    while(1) {
    	/* Init the message header buffer */
    	memset(lin1p3_header, 0, size_of_lin_header_d);
    	/* TODO: Synch break
    	 *
    	DisableIRQ(RXBREAK_UART_RX_TX_IRQn); //Disable RX interrupt so the break won't mess with the UART_RTOS driver
    	handle.base->S2 |= 0x01<<7; //Clear the LIN Break Detect Interrupt Flag
    	handle.base->S2 |= 0x01<<1; //Enable LIN Break Detection
    	while((handle.base->S2 &  0x01<<7) == 0x00) vTaskDelay(1); //Wait for the flag to be set
    	handle.base->S2 &= ~(0x01<<1); //Disable LIN Break Detection
    	handle.base->S2 |= 0x01<<7; //Clear the LIN Break Detect Interrupt Flag
    	EnableIRQ(RXBREAK_UART_RX_TX_IRQn); //Enable RX interrupt so the UART_RTOS driver works again
    	 */
    	synch_break_byte = 0xFF;
    	do {
    		UART_RTOS_Receive(handle->uart_rtos_handle, &synch_break_byte, 1, &n);
    	}while(synch_break_byte != 0);

    	/* Wait for header on the UART */
    	UART_RTOS_Receive(handle->uart_rtos_handle, lin1p3_header, size_of_lin_header_d, &n);
    	/* TODO: Parity check
    	p0 = (((ID0_MASK & lin1p3_header[1]) >> ID0) ^ ((ID1_MASK & lin1p3_header[1]) >> ID1) ^
    			((ID2_MASK & lin1p3_header[1]) >> ID2) ^ ((ID4_MASK & lin1p3_header[1]) >> ID4));
    	p1 = (((ID1_MASK & lin1p3_header[1]) >> ID1) ^ ((ID3_MASK & lin1p3_header[1]) >> ID3) ^
    	    			((ID4_MASK & lin1p3_header[1]) >> ID4) ^ ((ID5_MASK & lin1p3_header[1]) >> ID5));
    	// Validate parity bits
    	if((p1 == ((P1_MASK & lin1p3_header[1]) >> 1)) && (p0 == (P0_MASK & lin1p3_header[1]))){
		*/
			/* Get the message ID */
			ID = (lin1p3_header[1] & 0xFC)>>2;
			/* If the header is correct, check if the message is in the table */
			msg_idx = 0;
			/*Look for the ID in the message table */
			while(msg_idx < lin1d3_max_supported_messages_per_node_cfg_d) {
				if(handle->config.messageTable[msg_idx].ID == ID) {
					break;
				}
				msg_idx++;
			}
			/* If the message ID was not found then ignore it */
			if(msg_idx == lin1d3_max_supported_messages_per_node_cfg_d) continue;

			/* Calc the message size */
			switch(ID&0x03) {
				case 0x00: message_size = 2;
				break;
				case 0x01: message_size = 2;
				break;
				case 0x02: message_size = 4;
				break;
				case 0x03: message_size = 8;
				break;
			}

			message_size+=1;
			/* Init the message transmit buffer */
			memset(lin1p3_message, 0, size_of_uart_buffer);

			if(handle->config.messageTable[msg_idx].rx == 0) {
				/*If the message is in the table call the message callback */
				/* User shall fill the message */
				handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
				/* TODO: Checksum
				//Checksum = (Byte1 + Byte2 + ... + Byten)%256
				//Where n = number of bytes (in LIN, 2, 4 or 8)
				if(message_size == 2) lin1p3_message[2] = calculateChecksum(lin1p3_message, message_size);
				else if(message_size == 4) lin1p3_message[4] = calculateChecksum(lin1p3_message, message_size);
				else if(message_size == 8) lin1p3_message[8] = calculateChecksum(lin1p3_message, message_size);
				*/
				/* Send the message data */
				UART_RTOS_Send(handle->uart_rtos_handle, (uint8_t *)lin1p3_message, message_size/* + 1 */);
			}
			else {
				/* Wait for Response on the UART */
				UART_RTOS_Receive(handle->uart_rtos_handle, lin1p3_message, message_size, &n);
				/* TODO: Validate the checksum
				checksum = calculateChecksum(lin1p3_message, message_size);
				if(message_size == 2) if(lin1p3_message[2] == checksum)
					handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
				else if(message_size == 4) if(lin1p3_message[4] == checksum)
					handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
				else if(message_size == 8) if(lin1p3_message[8] == checksum)
					handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
				else ;
				*/
				/*If the message is in the table call the message callback */
				//handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
			}
    	//}
    }
}

static uint8_t calculateChecksum(uint8_t datas[size_of_uart_buffer], uint8_t size){
	uint16_t checksum = 0;
	uint8_t exceed = 0;

	if(size == 2){
		checksum = datas[0] + datas[1];
		if(checksum > 255){
			checksum = checksum - 256;
		}
	}
	else if(size == 4){
		checksum = datas[0] + datas[1] + datas[2] + datas[3];
		if(checksum > 255){
			while(exceed){
				checksum = checksum - 256;
				if(checksum > 255){
					exceed = 1;
				}
				else{
					exceed = 0;
				}
			}
		}
	}
	else if(size == 8){
		checksum = datas[0] + datas[1] + datas[2] + datas[3] + datas[4] + datas[5] + datas[6] + datas[7];
		if(checksum > 255){
			while(exceed){
				checksum = checksum - 256;
				if(checksum > 255){
					exceed = 1;
				}
				else{
					exceed = 0;
				}
			}
		}
	}

	return (uint8_t)checksum;
}
