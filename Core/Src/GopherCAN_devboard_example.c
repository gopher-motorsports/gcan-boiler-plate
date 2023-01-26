// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file

#include "GopherCAN_devboard_example.h"
#include "main.h"
#include <stdio.h>
#include <stdbool.h>

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;
extern TIM_HandleTypeDef htim3;


// Use this to define what module this board will be
#define THIS_MODULE_ID PDM_ID
#define PRINTF_HB_MS_BETWEEN 1000
#define HEARTBEAT_MS_BETWEEN 500
#define DMA_READ_MS_BETWEEN 1
#define DMA_STOPPED_TIMEOUT 10
#define DMA_LOW_SAMPLES 2
#define DMA_HIGH_SAMPLES IC_BUF_SIZE
#define HIGH_RPM 1500

// some global variables for examples
U8 last_button_state = 0;
static U16 averageSpeed = 0;
static bool stopped = true;

// Debug Variables Start
static U32 lastDMACheckMs = 0;
static U16 DMACurrentPosition = 0;
static U16 mostRecentDelta = 0;
static U16 amountOfSamples = 0;
static S32 valueInQuestion = 0;
static U16 firstValue = 0;
static U16 lastValue = 0;
// Debug Variables End


// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);
static U16 checkTransSpeedDMA(U16* ic1buf, U32* last_DMA_read, U16 averageSpeed);

// init
//  What needs to happen on startup in order to run GopherCAN
void init(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	// NOTE: CAN will also need to be added in CubeMX and code must be generated
	// Check the STM_CAN repo for the file "F0xx CAN Config Settings.pptx" for the correct settings
	if (init_can(example_hcan, THIS_MODULE_ID, BXTYPE_MASTER))
	{
		init_error();
	}

	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
	// will be run whenever this can command is sent to the module
	if (add_custom_can_func(SET_LED_STATE, &change_led_state, TRUE, NULL))
	{
		init_error();
	}
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
	service_can_tx_hardware(example_hcan);
}


// main_loop
//  another loop. This includes logic for sending a CAN command. Designed to be
//  called every 10ms
void main_loop(U16* ic1buf)
{
	static uint32_t last_heartbeat = 0;
	static U32 last_print_hb = 0;
	static U32 lastDMAReadValueTimeMs = 0;
	U8 button_state;

	if (HAL_GetTick() - last_heartbeat > HEARTBEAT_MS_BETWEEN)
	{
		last_heartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
	}

	// send the current tick over UART every second
	if (HAL_GetTick() - last_print_hb >= PRINTF_HB_MS_BETWEEN)
	{
		printf("Current tick: %lu\n", HAL_GetTick());
		last_print_hb = HAL_GetTick();
	}

	if (HAL_GetTick() - lastDMACheckMs >= DMA_READ_MS_BETWEEN)
	{
		lastDMACheckMs = HAL_GetTick();
		averageSpeed = checkTransSpeedDMA(ic1buf, &lastDMAReadValueTimeMs, averageSpeed);
	}

	// If the button is pressed send a can command to another to change the LED state
	// To on or off depending on the button
	button_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

	// Logic to only send one message per change in button state
	if (button_state != last_button_state)
	{
		last_button_state = button_state;

		if (send_can_command(PRIO_HIGH, ALL_MODULES_ID, SET_LED_STATE,
				!button_state, !button_state, !button_state, !button_state))
		{
			// error sending command
		}
	}
}

static U16 checkTransSpeedDMA(U16* ic1buf, U32* lastDMAReadValueTimeMs, U16 averageSpeed) {

	static U16 DMA_lastReadValue = 0;

	//U16 DMACurrentPosition = 0;
	U16 ic1bufCopy[IC_BUF_SIZE] = {0};

	// Copy all the values so they can't change while doing calculations
	//HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_1);
	DMACurrentPosition = IC_BUF_SIZE - (U16)((htim3.hdma[1])->Instance->NDTR);
	for (U16 c = 0; c < IC_BUF_SIZE - 1; c++)
	{
		ic1bufCopy[c] = ic1buf[c];

	}
	//HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, (U32*)ic1buf, IC_BUF_SIZE);

	valueInQuestion = (S32)ic1bufCopy[(U16)((S32)DMACurrentPosition - 1 + IC_BUF_SIZE) % IC_BUF_SIZE];

	if (stopped) {
		if (valueInQuestion != 0) {
			stopped = false;
		} else {
			return 0;
		}
	}
//
//	// Check if the last read value is the same as the current
	if (DMA_lastReadValue == valueInQuestion) {
		// Check if we haven't changed values in a while which might mean we stopped
		if (HAL_GetTick() - *lastDMAReadValueTimeMs >= DMA_STOPPED_TIMEOUT){
			stopped = true;

			// Clear buffer so any non-zero values will be quickly identified that the car is moving again
			HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_1);
			for (U16 c = 0; c < IC_BUF_SIZE - 1; c++)
			{
				ic1buf[c] = 0;

			}
			HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, (U32*)ic1buf, IC_BUF_SIZE);

			return 0;
		}
		return averageSpeed;
	}
	DMA_lastReadValue = valueInQuestion;

	// Calculate the amount of samples to take to get the speed based on the delta between the last 2 values
	// TODO: Protect this from rollover, and make sure there aren't other issues
	/*U16*/ mostRecentDelta = ic1bufCopy[(DMACurrentPosition - 2 + IC_BUF_SIZE) % IC_BUF_SIZE] - valueInQuestion;
	/*U16*/ amountOfSamples = (DMA_HIGH_SAMPLES / HIGH_RPM) * mostRecentDelta + DMA_LOW_SAMPLES; // Equation for getting how many samples we should average

	// Calculate the deltas between each of the time values and store them in deltaList
	U16 deltaList[IC_BUF_SIZE] = {0};
	for (U16 c = 0; c < amountOfSamples - 1; c++)
	{
		S16 i = (DMACurrentPosition + c) % IC_BUF_SIZE;
		U16 value1 = ic1bufCopy[i];
		U16 value2 = ic1bufCopy[(i - 1 + IC_BUF_SIZE) % IC_BUF_SIZE];
		if (value1 < value2) {
			deltaList[c] = ((1 << 16) | value1) - value2;
		} else {
			deltaList[c] = value1 - value2;
		}
	}

	// Get average of deltas
	U16 deltaTotal = 0;
	for (U16 c = 0; c < amountOfSamples - 1; c++) {
		deltaTotal += deltaList[c];
	}

	// Debug Start
	firstValue = ic1bufCopy[0];
	lastValue = ic1bufCopy[IC_BUF_SIZE - 1];

	//	printf("Last DMA Read Time: %lu", *last_DMA_read);
	//	printf("DMA Current Position: %u", DMACurrentPosition);
	//	printf("First array value: %u" + ic1bufCopy[0]);
	//	printf("Last array value: %u" + ic1bufCopy[IC_BUF_SIZE - 1]);
	//	printf("Value in question: %u" + valueInQuestion);
	//	printf("Most recent delta: %u" + mostRecentDelta);
	//	printf("Amount of samples: " + amountOfSamples);

	// Debug End



	*lastDMAReadValueTimeMs = HAL_GetTick();

	// TODO: Translate value to desired units

	return deltaTotal / amountOfSamples - 1;
}




// can_callback_function example

// change_led_state
//  a custom function that will change the state of the LED specified
//  by parameter to remote_param. In this case parameter is a U16*, but
//  any data type can be pointed to, as long as it is configured and casted
//  correctly
static void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	//HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, !!remote_param);
	return;
}


// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
		HAL_Delay(250);
	}
}

// end of GopherCAN_example.c
