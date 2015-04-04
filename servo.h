/*
 * Copyright (C) 2015  Shahbaz Youssefi <ShabbyX@gmail.com>
 *
 * This file is part of Spidy.
 *
 * Spidy is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Spidy is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Spidy.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SERVO_H
#define SERVO_H

#include "common.h"

struct servo_control_data
{
	/* high level variables */
	int16_t goal;				/* goal distance from home position */
	int16_t position;			/* current position from home position */
	uint8_t velocity;			/* absolute velocity towards goal */
	/* low level control */
	volatile uint16_t *pwm_register;	/* the OCRxy register used to set PWM active time */
};

void servo_init(struct servo_control_data *servo);
void servo_control(struct servo_control_data *servo);

#endif
