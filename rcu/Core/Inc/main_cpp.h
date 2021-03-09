/*
 * main.hpp
 *
 *  Created on: 30.11.2019
 *      Author: live
 */

#ifndef INC_MAIN_CPP_H_
#define INC_MAIN_CPP_H_

#ifdef __cplusplus
extern "C" {
#endif

// for SWO debugging, see http://stefanfrings.de/stm32/cube_ide.html
#define PRINTF( fmt,...) printf( "%10lu ms: " fmt, HAL_GetTick(), ##__VA_ARGS__ )

void setup(UART_HandleTypeDef *huart);

void loop(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAIN_CPP_H_ */
