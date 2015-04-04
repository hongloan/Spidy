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

#include "servo.h"

/*
 * Smooth control a servo motor
 *
 * The servo has a current position, current speed and a goal.  The PWM required for the motor is simulated by a timer
 * on a GPIO because later on, many motors are required to be controlled and there aren't enough PWM's for them.
 *
 * The servo is controlled in 20ms periods, where the goal, G, dictates the position of the servo, i.e., the high time
 * of the PWM, according to the following formula:
 *
 *    SERVO_HOME + G
 *
 * In other words, G is the offset from servo's home (1.5ms).  G is limited to +-SERVO_RANGE.  All values are in us.
 *
 * Current position, P, dictates the current position of the servo and follows the same formula as G.  In each period
 * of the control algorithm, P moves towards G with the current speed V.  During the movement, V is increased until
 * it reaches its maximum of SERVO_MAX_SPEED, with SERVO_ACCELERATION acceleration.  Once P gets close to G,
 * it will start decelerating with SERVO_ACCELERATION.
 *
 * Given maximum speed V and deceleration -A, the time it takes for the speed to reach zero is given by:
 *
 *    t = V / A
 *
 * and the distance D traveled is given by:
 *
 *    D = -1/2 A * t^2 + V * t = V^2 / 2A
 *
 * With a large distance to travel (from position P), the speed vs distance graph would thus look like this:
 *
 *    v ^
 *      |
 *      |    __________
 *    V |  _-|        |-_
 *      | /  |        |  \
 *      |/   |        |   \
 *      ||   |        |   |
 *      ------------------------>
 *      O    D       P-D  P     d
 *
 * Taking into consideration the cases where P < D and P < 2D, one simple solution is to accelerate when the
 * point (p, v) showing current position and velocity is below the curve L from O to (D, V) and decelerate
 * otherwise.  There are two options to prevent oscillation around O.  One is to ensure V doesn't go over the
 * curve L, and the other is to set velocity to zero when getting close enough to O.  The first method suffers
 * from arithmetic errors calculating points of L, while the second reduces precision.  A hybrid solution is
 * possible.  First, curve L can be overestimated so that clamping velocities higher than L to L wouldn't bring
 * them below L.  This ensures that around O, V is very small.  Therefore, a tiny threshold can be used to set
 * V to 0 when getting close to O.
 */

/* constants regarding motor characteristics and control */
#define SERVO_HOME 1500
#define SERVO_RANGE 500
#define SERVO_MAX_SPEED 30
#define SERVO_ACCELERATION 2
#define SERVO_NEAR_GOAL_THRESHOLD 4

void servo_init(struct servo_control_data *servo)
{
	*servo->pwm_register = SERVO_HOME;	
}

static inline uint8_t calc_sqrt(uint16_t x)
{
	uint8_t res = 0;
	for (int8_t i = 7; i >= 0; --i)
	{
		uint8_t tmp = res | (1 << i);
		if (tmp * tmp <= x)
			res = tmp;
	}

	return res;
}

void servo_control(struct servo_control_data *servo)
{
	/*
	 * the algorithm is simple.  Assuming d for current distance to goal, v for current velocity, V for maximum velocity and D for deceleration distance,
	 * we first check if the point (d, v) falls above or under the curve L passing from O (the origin) to point (D, V).  If it falls under, we
	 * accelerate.  If it is over, we decelerate.  A tight under-estimation of the curve is used to check this.  Once the new v is calculated,
	 * it is ensured that it doesn't move the point above curve L.  A tight over-estimation of the curve is used for bringing the value of v lower.
	 *
	 * If the speed is v, the distance traveled before decelerating to 0 is, as calculated on the top of this file, v^2/2a.  Thus, at distance d,
	 * the maximum speed possible so that we decelerate to 0 just as we reach the goal is sqrt(2ad).
	 */

	int16_t distance = servo->goal - servo->position;

	/* if at goal, do nothing */
	if (distance == 0)
		return;

	uint16_t abs_distance = distance < 0?-distance:distance;
	uint16_t allowed_v_underestimate = calc_sqrt(2 * SERVO_ACCELERATION * abs_distance);
	uint16_t allowed_v_overestimate = allowed_v_underestimate + 1;

	/* if going faster than currently allowed, slow down */
	if (servo->velocity > allowed_v_overestimate)
		servo->velocity = allowed_v_overestimate;

	/* TODO: make velocity signed, and check if it is in the opposite direction of goal.  This means the goal was changed before I reached it.  In such
	 * a case, I should always decelerate first and only then change direction */
	uint8_t new_velocity = 0;
	uint16_t delta_distance = servo->velocity;
	uint16_t possible_acceleration = SERVO_ACCELERATION;
	uint16_t possible_deceleration = SERVO_ACCELERATION;

	if (SERVO_MAX_SPEED - servo->velocity < SERVO_ACCELERATION)
		possible_acceleration = SERVO_MAX_SPEED - servo->velocity;
	if (servo->velocity < SERVO_ACCELERATION)
		possible_deceleration = servo->velocity;

	/*
	 * the distance that would be traveled by the next time this function is called is given by:
	 *
	 *     d = 1/2at^2 + vt
	 *
	 * where t is always 1.
	 */
	if (servo->velocity < allowed_v_underestimate)
	{
		/* accelerate */
		new_velocity = servo->velocity + possible_acceleration;
		delta_distance += possible_acceleration / 2;
	}
	else
	{
		/* decelerate */
		new_velocity = servo->velocity - possible_deceleration;
		delta_distance = delta_distance > possible_deceleration / 2?delta_distance - possible_deceleration / 2:0;
	}

	if (delta_distance > abs_distance)
		delta_distance = abs_distance;

	/* set the new location */
	if (servo->goal > servo->position)
		servo->position += delta_distance;
	else
		servo->position -= delta_distance;

	/* if reached close enough to goal, just get to goal and be done with it */
	if (abs_distance - delta_distance < SERVO_NEAR_GOAL_THRESHOLD)
	{
		servo->position = servo->goal;
		new_velocity = 0;
	}

	/* set the new velocity */
	if (new_velocity > SERVO_MAX_SPEED)
		new_velocity = SERVO_MAX_SPEED;
	servo->velocity = new_velocity;

	/* set the corresponding PWM to the new position */
	*servo->pwm_register = SERVO_HOME + servo->position;
}

