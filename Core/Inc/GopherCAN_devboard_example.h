// GopherCAN_devboard_example.h
//  Header file for GopherCAN_devboard_example.c

#ifndef GOPHERCAN_DEVBOARD_EXAMPLE_H
#define GOPHERCAN_DEVBOARD_EXAMPLE_H

#include "GopherCAN.h"
#include <stdbool.h>

void init(CAN_HandleTypeDef* hcan_ptr);
void can_buffer_handling_loop();
void setHeartBeatMode(bool errorState, uint16_t blinksPerErrorCycle, GPIO_TypeDef* ledPort, uint16_t ledPin);
void main_loop();

#endif
