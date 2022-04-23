/*------------------------------------------------------------------
 *  uart.c -- configures uart
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */
#include "uart.h"
#include <stdbool.h>
#include <stdio.h>
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "utils/quad_ble.h"
#include "in4073.h"
#include "utils/queue.h"
#include "control.h"

#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14

Queue rx_queue;
Queue tx_queue;

static bool txd_available = true;

// transfers bytes from drone to PC.
void uart_put(uint8_t byte)
{
	if (wireless_mode) {
		enqueue(&ble_tx_queue, byte);
	}

	NVIC_DisableIRQ(UART0_IRQn);

	if (txd_available) {
		txd_available = false;
		NRF_UART0->TXD = byte;
	} else {
		enqueue(&tx_queue, byte);
	}

	NVIC_EnableIRQ(UART0_IRQn);
}

// Reroute printf, so that the printf transfers data from drone to PC.
int _write(int file, const char * p_char, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		uart_put(*p_char++);
	}

	return len;
}

// Set UART interrupt function
void UART0_IRQHandler(void)
{
	if (NRF_UART0->EVENTS_RXDRDY != 0) {
		NRF_UART0->EVENTS_RXDRDY  = 0;
		enqueue(&rx_queue, NRF_UART0->RXD);
	}

	if (NRF_UART0->EVENTS_TXDRDY != 0) {
		NRF_UART0->EVENTS_TXDRDY = 0;
		if (tx_queue.count) NRF_UART0->TXD = dequeue(&tx_queue);
		else txd_available = true;
	}

	if (NRF_UART0->EVENTS_ERROR != 0) {
		NRF_UART0->EVENTS_ERROR = 0;
		printf("uart error: %lu\n", NRF_UART0->ERRORSRC);
	}
}

void uart_init(void)
{
	init_queue(&rx_queue); // Initialize receive queue
	init_queue(&tx_queue); // Initialize transmit queue

	nrf_gpio_cfg_output(TX_PIN_NUMBER);
	nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL); 
	NRF_UART0->PSELTXD = TX_PIN_NUMBER;
	NRF_UART0->PSELRXD = RX_PIN_NUMBER;
	NRF_UART0->BAUDRATE        = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);

	NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	NRF_UART0->EVENTS_RXDRDY    = 0;
	NRF_UART0->EVENTS_TXDRDY    = 0;
	NRF_UART0->TASKS_STARTTX    = 1;
	NRF_UART0->TASKS_STARTRX    = 1;

	NRF_UART0->INTENCLR = 0xffffffffUL;
	NRF_UART0->INTENSET = 	(UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
			(UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
			(UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos);

	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, 3);
	NVIC_EnableIRQ(UART0_IRQn);
}
