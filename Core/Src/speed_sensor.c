// speed_sensor.c
//  TODO DOCS

#include "speed_sensor.h"

// static function prototypes
// TODO

extern TIM_HandleTypeDef htim2;

TIM_HandleTypeDef* htim;
U16 ic1buf[IC_BUF_SIZE] = {0};
U16 ic2buf[IC_BUF_SIZE] = {0};
U16 ic3buf[IC_BUF_SIZE] = {0};
U16 ic4buf[IC_BUF_SIZE] = {0};


// init_speed_sensor
//  TODO DOCS
S8 init_speed_sensor()
{
	// starting the timer inputs for DMA
	if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (U32*)ic1buf, IC_BUF_SIZE) ||
		HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (U32*)ic2buf, IC_BUF_SIZE) ||
		HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, (U32*)ic3buf, IC_BUF_SIZE) ||
		HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, (U32*)ic4buf, IC_BUF_SIZE) ||
		HAL_TIM_Base_Start(&htim2))
	{
		return -1;
	}

	// TODO
	return -1;
}


// get_sensor_speed
//  TODO DOCS
float get_sensor_speed(void)
{
	/*
	// read and average the timer buffers to get the average period
	HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_4);

	U32 ic1tot = 0;
	U32 ic2tot = 0;
	U32 ic3tot = 0;
	U32 ic4tot = 0;
	for (U16 c = 0; c < IC_BUF_SIZE - 1; c++)
	{
		ic1tot += (ic1buf[c+1] - ic1buf[c]);
		ic2tot += (ic2buf[c+1] - ic2buf[c]);
		ic3tot += (ic3buf[c+1] - ic3buf[c]);
		ic4tot += (ic4buf[c+1] - ic4buf[c]);
	}

	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (U32*)ic1buf, IC_BUF_SIZE);
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (U32*)ic2buf, IC_BUF_SIZE);
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, (U32*)ic3buf, IC_BUF_SIZE);
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, (U32*)ic4buf, IC_BUF_SIZE);
	*/
}




// End of speed_sensor.c
