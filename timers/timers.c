/*
 *
 * Timers management
 *
 */

#include <msp430.h>
#include <stddef.h>
#include <timers.h>
#include <stddef.h>
#include <stdbool.h>

// ---------------------------------------------------------------
// Internals
// ---------------------------------------------------------------
// Periodic timer duration
static uint16_t _periodic_timer_duration = 0u;
// IDs of the events (periodic and one shot)
static uint16_t _periodic_event;
//static uint16_t _one_shot_event;

// One shot timer structure
struct timer_one_shot_t
{
	bool free;
	uint16_t event_id;
	volatile unsigned int *CTL;
	volatile unsigned int *CCR;
};
#define ONE_SHOT_TIMERS_COUNT	7u
static struct timer_one_shot_t _one_shot_timers[ONE_SHOT_TIMERS_COUNT];

// Minimal timer duration
#define MIN_TIMER_DURATION	5u


// -----------------------------------------------------------------
// Interface implementation
// -----------------------------------------------------------------
/*
 * Initialize the timers
 */
void timers_init(void)
{
	// Adding the events
	_periodic_event = event_add(NULL);
	//_one_shot_event = event_add(NULL);

	_one_shot_timers[0].event_id = event_add(NULL);
	_one_shot_timers[0].free = true;
	_one_shot_timers[0].CTL = &(TB0CCTL0);
	_one_shot_timers[0].CCR = &(TB0CCR0);
	_one_shot_timers[1].event_id = event_add(NULL);
	_one_shot_timers[1].free = true;
	_one_shot_timers[1].CTL = &(TB0CCTL1);
	_one_shot_timers[1].CCR = &(TB0CCR1);
	_one_shot_timers[2].event_id = event_add(NULL);
	_one_shot_timers[2].free = true;
	_one_shot_timers[2].CTL = &(TB0CCTL2);
	_one_shot_timers[2].CCR = &(TB0CCR2);
	_one_shot_timers[3].event_id = event_add(NULL);
	_one_shot_timers[3].free = true;
	_one_shot_timers[3].CTL = &(TB0CCTL3);
	_one_shot_timers[3].CCR = &(TB0CCR3);
	_one_shot_timers[4].event_id = event_add(NULL);
	_one_shot_timers[4].free = true;
	_one_shot_timers[4].CTL = &(TB0CCTL4);
	_one_shot_timers[4].CCR = &(TB0CCR4);
	_one_shot_timers[5].event_id = event_add(NULL);
	_one_shot_timers[5].free = true;
	_one_shot_timers[5].CTL = &(TB0CCTL5);
	_one_shot_timers[5].CCR = &(TB0CCR5);
	_one_shot_timers[6].event_id = event_add(NULL);
	_one_shot_timers[6].free = true;
	_one_shot_timers[6].CTL = &(TB0CCTL6);
	_one_shot_timers[6].CCR = &(TB0CCR6);

	// Both timer A and B are sources from ACLK (32 kHz)
	TA0CTL = TACLR;
	TA0CTL = TASSEL__ACLK | ID__8 | MC__STOP;
	TB0CTL = TBCLR;
	TB0CTL = TBSSEL__ACLK | ID__8 | MC__CONTINUOUS;	// Starting timer B
}

/*
 * Set the periodic event
 * Time unit for duration is 1/4096 s
 */
void timer_set_periodic_event(uint16_t duration, event_handler_t handler)
{
	if (duration < MIN_TIMER_DURATION)
	{
		duration = MIN_TIMER_DURATION;
	}

	_periodic_timer_duration = duration;

	event_set_handler(_periodic_event, handler);

	TA0CCTL1 = CCIE;
	TA0CCR1 = duration;
	TA0CTL |= MC__CONTINOUS;
}

/*
 * Change periodic event period
 */
void timer_periodic_set_period(uint16_t duration)
{
	_periodic_timer_duration = duration;
	TA0CCR1 = TA0R + duration;
}

/*
 * Set one shot timer.
 * Time unit is 1/32768 s
 */
int timer_set_one_shot_event(uint16_t duration, event_handler_t handler)
{
	/*
	// If timer B is running we stop it first
	TB0CTL &= ~(BIT4 + BIT5);
	TB0R = 0u;

	if (duration < MIN_TIMER_DURATION)
	{
		duration = MIN_TIMER_DURATION;
	}

	event_set_handler(_one_shot_event, handler);

	TB0CCTL1 = CCIE;
	TB0CCR1 = duration;
	TB0CTL |= MC__CONTINOUS;
	*/

	if (duration < MIN_TIMER_DURATION)
	{
		duration = MIN_TIMER_DURATION;
	}

	int i;
	// Looking for a free spot
	for (i = 0; i < ONE_SHOT_TIMERS_COUNT; i++)
	{
		if (_one_shot_timers[i].free)
		{
			break;
		}
	}
	_one_shot_timers[i].free = false;
	event_set_handler(_one_shot_timers[i].event_id, handler);
	*(_one_shot_timers[i].CCR) = TB0R + duration;
	*(_one_shot_timers[i].CTL) = CCIE;

	return i;
}


/*
 * Cancel the one shot timer
 */
void timer_cancel_one_shot(int timer_id)
{
	/*
	// Stoping the timer
	TB0CTL &= ~(BIT4 + BIT5);
	TB0R = 0u;
	TB0CCTL1 &= ~CCIE;
	*/

	if (!_one_shot_timers[timer_id].free)
	{
		*(_one_shot_timers[timer_id].CTL) &= ~CCIE;
		_one_shot_timers[timer_id].free = true;
	}
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void timerA1_isr(void)
{
	// Setting next interrupt
	TA0CCR1 += _periodic_timer_duration;

	// Clearing interrupt flag
	TA0CCTL1 &= ~CCIFG;

	EVENT_SIGNAL_ISR(_periodic_event);
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void timerB0_isr(void)
{
	/*
	// Stoping the timer
	TB0CTL &= ~(BIT4 + BIT5);

	// Clearing the interrupt flag
	TB0CCTL1 &= ~CCIFG;

	EVENT_SIGNAL_ISR(_one_shot_event);
	*/

	*(_one_shot_timers[0].CTL) &= ~CCIE;
	_one_shot_timers[0].free = true;
	EVENT_SIGNAL_ISR(_one_shot_timers[0].event_id);
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void timerB1_isr(void)
{
	/*
	// Stoping the timer
	TB0CTL &= ~(BIT4 + BIT5);

	// Clearing the interrupt flag
	TB0CCTL1 &= ~CCIFG;

	EVENT_SIGNAL_ISR(_one_shot_event);
	*/

	int i;
	for (i = 1; i < ONE_SHOT_TIMERS_COUNT; i++)
	{
		if (!_one_shot_timers[i].free)
		{
			if (*(_one_shot_timers[i].CTL) & CCIE)
			{
				*(_one_shot_timers[i].CTL) &= ~CCIE;
				_one_shot_timers[i].free = true;
				EVENT_SIGNAL_ISR(_one_shot_timers[i].event_id);
			}
		}
	}
}
