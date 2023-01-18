// speed_sensor.h
//  TODO DOCS

#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H


// includes
#include "base_types.h"
#include "main.h"


// defines
#define IC_BUF_SIZE 64


// enums
typedef enum
{
	U16_TIMER = 16,
	U32_TIMER = 32,
} TIMER_BITS_t;


// structs
typedef struct
{
	TIM_HandleTypeDef* htim;
	TIMER_BITS_t timer_type;
	U32 channel;
	U32 period_ns; // period of this timer in nanoseconds
	float convert_const; // conversion from the timer value delta (from timer value in nanoseconds to whatever output is needed)
} SPEED_SENSOR_t; // TODO consider how to handle buffers with 16 and 32 bit timers


// function prototypes
// TODO

#endif // #ifndef SPEED_SENSOR_H

// End of speed_sensor.h
