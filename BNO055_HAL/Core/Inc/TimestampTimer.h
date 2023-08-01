/*
 * TimestampTimer.h
 *
 *  Created on: Jul 31, 2023
 *      Author: ndani
 */
// THIS IS USING A SOFTWARE BASED TIMER.. AS TO KEEP IT SIMPLE FOR NOW IN THIS EARLY STAGE
// NEXT IMPLEMENT A TIMER BASED APPROACH

#ifndef INC_TIMESTAMPTIMER_H_
#define INC_TIMESTAMPTIMER_H_

#include "stm32f4xx_hal.h"

namespace TimestampTimer {
/**
 * @brief Get the current timestamp in milliseconds.
 *
 * @return The current timestamp in milliseconds.
 */
uint64_t getTimestamp() {
	return HAL_GetTick();
}
} // namespace TimestampTimer

#endif /* INC_TIMESTAMPTIMER_H_ */
