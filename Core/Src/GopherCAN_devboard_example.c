// GopherCAN_devboard_example.c
//  TODO DOCS - when the project becomes more fleshed out add a quick comment
//  explaining the purpose of this file

#include "GopherCAN_devboard_example.h"
#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;


// Use this to define what module this board will be
#define THIS_MODULE_ID PDM_ID


// some global variables for examples
U8 last_button_state = 0;


// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);


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
		// an error has occurred, stay here
		while (1)
		{
			// failed to init CAN
			HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
			HAL_Delay(100);
		}
	}

	// enable all of the variables in GopherCAN for testing
	set_all_params_state(TRUE);
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
void main_loop()
{
	while(1)
	{
#define MAX_OLD_DATA_TIME 200
#define THROTTLE_DIF_THRESH_percent 10
#define THROTTLE_DIF_TIME_ms 1000
		// check to make sure the ETC is not stuck. ECU and BSPD handles checking TPS1 and TPS2
		request_parameter(PRIO_HIGH, TCM_ID, THROTTLE_POS_REQUIRED_ECU_ID);
		request_parameter(PRIO_HIGH, TCM_ID, THROTTLE_POS_1_ECU_ID);

		uint32_t curr_tick = HAL_GetTick();
		static uint32_t throttle_off_time = 0;
		static bool throttle_off_active = false;
		static bool bspd_off = false;
		static uint32_t last_blink = 0;

		// blinky
		if (curr_tick - last_blink >= 500)
		{
			HAL_GPIO_TogglePin(GRN_LED_GPIO_Port, GRN_LED_Pin);
			last_blink = curr_tick;
		}

		// make sure the data is up to date
		if (curr_tick - throttle_pos_required_ecu.last_rx < MAX_OLD_DATA_TIME &&
				curr_tick - throttle_pos_1_ecu.last_rx < MAX_OLD_DATA_TIME)
		{
			// off if we are getting data
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, RESET);

			// check to make sure the expected throttle and actual throttle are not
			// different by more than 10% for than a second
			if (throttle_pos_1_ecu.data - throttle_pos_required_ecu.data > THROTTLE_DIF_THRESH_percent)
			{
				HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, SET);

				// the error case has been triggered. Start the timer
				if (!throttle_off_active)
				{
					throttle_off_active = true;
					throttle_off_time = curr_tick;
				}
				else
				{
					// see if it has been long enough since the BSPD was detected to be off
					if (curr_tick - throttle_off_time > THROTTLE_DIF_TIME_ms)
					{
						HAL_GPIO_WritePin(BSPD_Pow_GPIO_Port, BSPD_Pow_Pin, RESET);
						bspd_off = true;
					}
				}
			}
			else
			{
				// reset the error states
				throttle_off_time = 0;
				throttle_off_active = false;

				if (!bspd_off) HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
			}
		}
		else
		{
			// not getting data. Reset the error timers
			throttle_off_time = 0;
			throttle_off_active = false;

			// note that we are not getting data
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, SET);
		}

		osDelay(25);
	}
}


// can_callback_function example

// change_led_state
//  a custom function that will change the state of the LED specified
//  by parameter to remote_param. In this case parameter is a U16*, but
//  any data type can be pointed to, as long as it is configured and casted
//  correctly
static void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, !!remote_param);
	return;
}

// end of GopherCAN_example.c
