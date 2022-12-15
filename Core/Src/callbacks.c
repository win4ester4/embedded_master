#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "state_types.h"
#include "callbacks.h"

extern buttonAntibounceStates_t btn_anti_states;
extern uartBufferStates_t uart_buff_states;
extern uint32_t btn_time;

inline void HAL_SYSTICK_Callback(void) {
	if (btn_anti_states == btn_pressed && (HAL_GetTick() - btn_time) > PRESS_TIMEOUT) {
		btn_anti_states = btn_realese_delay;
	}
}

inline void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	btn_anti_states = btn_pressed;
}

inline void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    //Stop this DMA transmission
    HAL_UART_DMAStop(huart);
    uart_buff_states = unproc_buff_detected;
    //Next DMA transmission will be allowed after buffer processing
}

