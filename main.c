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

#include "common.h"
#include "servo.h"

/*
 * The real-time scheduler uses 10ms ticks to schedule tasks.  With 8-bit counters, this allows tasks with a period of
 * up to ~2.5s.  The real-time control algorithm is expected to ensure no overrun occurs.
 */

#define SCHED_DEFAULT_LED_PERIOD 100
#define SCHED_DEFAULT_SERVO_CONTROL_PERIOD 2

struct scheduler_data
{
	/* counters */
	volatile uint8_t led;
	volatile uint8_t servo_control;
	volatile uint8_t one_second;
	volatile uint8_t time;
	/* periods */
	uint8_t led_period;
	uint8_t servo_control_period;
};

struct spidy_calibration
{
	int16_t home;				/* where the motor actually centers the leg */
	int16_t range;				/* how much the motor is actually allowed to move in each direction */
};

enum spidy_state
{
	SPIDY_STILL = 0,			/* standing still, doing nothing */
	SPIDY_FORWARD = 1,
	SPIDY_BACKWARD = 2,
	SPIDY_TURN_LEFT = 3,
	SPIDY_TURN_RIGHT = 4,
};

struct spidy_action_direction
{
	int direction[3];			/* 1, 0 or -1 for clockwise, none and counter-clockwise of motors middle, left and right */
};

struct spidy_behavior
{
	const struct spidy_action_direction *steps;
	const uint8_t steps_count;
	uint8_t current_step;
};

static struct scheduler_data scheduler;

static const struct spidy_calibration spidy_calib[3] = {
	[0] = { .home = -50, .range = 150, },		/* the center motor */
	[1] = { .home = 0, .range = 200, },		/* left limbs */
	[2] = { .home = 0, .range = 200, },		/* right limbs */
};

static const struct spidy_action_direction spidy_still[] = {
	{ .direction = {0, 0, 0}, },
};
static const struct spidy_action_direction spidy_forward[] = {
	{ .direction = { -1, 0, 0}, },
	{ .direction = { 0, -1, -1}, },
	{ .direction = { 1, 0, 0}, },
	{ .direction = { 0, 1, 1}, },
};

static const struct spidy_action_direction spidy_backward[] = {
	{ .direction = { -1, 0, 0}, },
	{ .direction = { 0, 1, 1}, },
	{ .direction = { 1, 0, 0}, },
	{ .direction = { 0, -1, -1}, },
};

static const struct spidy_action_direction spidy_turn_left[] = {
	{ .direction = { -1, 0, 0}, },
	{ .direction = { 0, 1, -1}, },
	{ .direction = { 1, 0, 0}, },
	{ .direction = { 0, -1, 1}, },
};

static const struct spidy_action_direction spidy_turn_right[] = {
	{ .direction = { -1, 0, 0}, },
	{ .direction = { 0, -1, 1}, },
	{ .direction = { 1, 0, 0}, },
	{ .direction = { 0, 1, -1}, },
};

static struct spidy_behavior spidy_behaviors[] = {
	[SPIDY_STILL] = { .steps = spidy_still, .steps_count = sizeof spidy_still / sizeof *spidy_still },
	[SPIDY_FORWARD] = { .steps = spidy_forward, .steps_count = sizeof spidy_forward / sizeof *spidy_forward },
	[SPIDY_BACKWARD] = { .steps = spidy_backward, .steps_count = sizeof spidy_backward / sizeof *spidy_backward },
	[SPIDY_TURN_LEFT] = { .steps = spidy_turn_left, .steps_count = sizeof spidy_turn_left / sizeof *spidy_turn_left },
	[SPIDY_TURN_RIGHT] = { .steps = spidy_turn_right, .steps_count = sizeof spidy_turn_right / sizeof *spidy_turn_right },
};

static struct servo_control_data servo[3] = {
	[0] = { .pwm_register = &OCR1A, },
	[1] = { .pwm_register = &OCR1B, },
	[2] = { .pwm_register = &OCR1C, },
};

/* scheduler interrupt, ticking every 10ms */
ISR(TIMER0_COMPA_vect)
{
	++scheduler.led;
	++scheduler.servo_control;

	++scheduler.one_second;
	if (scheduler.one_second >= 100)
	{
		scheduler.one_second -= 100;
		++scheduler.time;
	}
}

static void setup_peripherals(void)
{
	/* use the LED on C7 to show timing errors (until I get to an oscilloscope) */
	DDRC |= _BV(DDC7);
	PORTC &= ~_BV(PORTC7);

	/* disable USB interrupt flags, apparently enabled by the bootloader */
	USBCON &= ~_BV(VBUSTE);
	UDIEN = 0;

	/* setup Timer1 for 20ms ticks, with 1us precision */
	/* Note for future: Timer3 works exactly like Timer1 */
	TCCR1A = _BV(COM1A1)			/* Compare Output Mode fro channel A set to 2, for non-inverted phase and frequency correct PWM */
		| _BV(COM1B1)			/* ^^^ the same for channel B */
		| _BV(COM1C1);			/* ^^^ the same for channel C */
						/* WGM1[1:0] set to zero (see comment on WGM[3:2]) */
	TCCR1B = _BV(WGM13)			/* WGM1 set to 0b1000 to select phase and frequency correct PWM with top in ICR1 */
		| _BV(CS11);			/*
						 * CLK/8, i.e., @2MHz
						 * The timer counts up and then down.  That means when ICR1 is set, the frequency would actually be
						 * half.  With that in mind, the value for ICR1 directly translates to a period in us.  Similarly,
						 * the value for OCR1x translates to a PWM active time in us.
						 */
	ICR1 = 20000;				/* 20ms period for Timer1 */
	for (int i = 0; i < 3; ++i)
		servo_init(&servo[i]);		/* start the motor at home position */

	/* setup Timer0 with 1ms ticks for real-time control */
	TIMSK0 = _BV(OCIE0A);			/* interrupt on reaching timer limit */
	TCCR0A = _BV(WGM01);			/* CTC mode */
	TCCR0B = _BV(CS02) | _BV(CS00);		/* CLK/1024, i.e., @16KHz */
	OCR0A = 159;				/* interrupt every 160 counts (-1 because counter starts from 0), i.e., every 10ms */

	/* enable outputs for PWMs */
	DDRB |= _BV(DDB5)			/* output from OC1A */
		| _BV(DDB6)			/* output from OC1B */
		| _BV(DDB7);			/* output from OC1C */

	/* set micro to sleep into idle mode, for fast wake up */
	set_sleep_mode(SLEEP_MODE_IDLE);

	/* enable interrupts */
	sei();
}

static void stand_still(void)
{
	for (int i = 0; i < 3; ++i)
		servo[i].goal = spidy_calib[i].home;
}

static void apply_action(const struct spidy_action_direction *action)
{
	for (int i = 0; i < 3; ++i)
		if (action->direction[i])
			servo[i].goal = action->direction[i] * spidy_calib[i].range + spidy_calib[i].home;
}

static bool reached_goal(void)
{
	for (int i = 0; i < 3; ++i)
		if (servo[i].position != servo[i].goal)
			return false;
	return true;
}

int main (void)
{
	enum spidy_state state = SPIDY_FORWARD;

	scheduler = (struct scheduler_data){
		.led_period = SCHED_DEFAULT_LED_PERIOD,
		.servo_control_period = SCHED_DEFAULT_SERVO_CONTROL_PERIOD,
	};

	setup_peripherals();

	while (1)
	{
		/* wait for tick or event */
		sleep_mode();

		/* you know, LED */
		if (scheduler.led >= scheduler.led_period)
		{
			scheduler.led -= scheduler.led_period;

			PORTC ^= _BV(PORTC7);
		}

		/* low level servo control.  This moves each servo towards its goal smoothly */
		if (scheduler.servo_control >= scheduler.servo_control_period)
		{
			scheduler.servo_control -= scheduler.servo_control_period;

			for (int i = 0; i < 3; ++i)
				servo_control(&servo[i]);
		}

		/* once each step of behavior is complete, take the next step */
		if (state != SPIDY_STILL && reached_goal())
		{
			struct spidy_behavior *behavior = &spidy_behaviors[state];
			apply_action(&behavior->steps[behavior->current_step++]);
			if (behavior->current_step >= behavior->steps_count)
				behavior->current_step -= behavior->steps_count;
		}

		/* TODO: for testing only: every 15 seconds change behavior */
		if (scheduler.time >= 15)
		{
			scheduler.time -= 15;

			++state;
			if (state > SPIDY_TURN_RIGHT)
				state = SPIDY_STILL;

			stand_still();
		}
	}

	return 1;
}
