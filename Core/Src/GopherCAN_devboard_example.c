// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file

#include "GopherCAN_devboard_example.h"
#include "main.h"
#include <stdio.h>

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;

#define PRINTF_HB_MS_BETWEEN 1000

// some global variables for examples
U8 last_button_state = 0;


// the CAN callback function used in this example
static void change_led_state(U8 sender, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);

// init
//  What needs to happen on startup in order to run GopherCAN
void init(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	if (init_can(hcan_ptr, GCAN0))
	{
		init_error();
	}

	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
	// will be run whenever this can command is sent to the module
	attach_callback_cmd(SET_LED_STATE, &change_led_state);
}


// can_buffer_handling_loop
//  This loop will handle CAN RX software task and CAN TX hardware task. Should be
//  called every 1ms or as often as received messages should be handled
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx(example_hcan);
}

uint8_t pullup_2_state = 0;
uint8_t pullup_3_state = 0;

// main_loop
//  another loop. This includes logic for sending a CAN command. Designed to be
//  called every 10ms
void main_loop()
{

	static U32 last_print_hb = 0;
	//U8 button_state;
	HAL_GPIO_WritePin(PU2_GPIO_Port, PU2_Pin, pullup_2_state);
	HAL_GPIO_WritePin(PU3_GPIO_Port, PU3_Pin, pullup_3_state);

	// send the current tick over UART every second
	if (HAL_GetTick() - last_print_hb >= PRINTF_HB_MS_BETWEEN)
	{
		HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
		HAL_GPIO_TogglePin(GSENSE_GPIO_Port, GSENSE_Pin);
		last_print_hb = HAL_GetTick();
	}

}


// can_callback_function example
static void change_led_state(U8 sender, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, !!remote_param);
	return;
}


// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(GRN_LED_GPIO_Port, GRN_LED_Pin);
		HAL_Delay(250);
	}
}

// end of GopherCAN_example.c
