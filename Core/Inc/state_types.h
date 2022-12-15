/*
 * state_types.h
 *
 *  Created on: Dec 15, 2022
 *      Author: Yaroslav
 */

#ifndef INC_STATE_TYPES_H_
#define INC_STATE_TYPES_H_

typedef enum {
	btn_wait_press,
	btn_pressed,
	btn_realese_delay
} buttonAntibounceStates_t;

typedef enum {
	uart_wait_buff,
	unproc_buff_detected
} uartBufferStates_t;

#endif /* INC_STATE_TYPES_H_ */
