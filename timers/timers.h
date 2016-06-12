/*
 * Timers management.
 * Interface
 *
 * Timer A is used for periodic event and runs at 4096 Hz
 * Timer B is used for one shot event and runs at 4096 Hz
 *
 * NOTE: only one periodic event and one one shot event at the time can run.
 */

#ifndef TIMERS_TIMERS_H_
#define TIMERS_TIMERS_H_

#include <stdint.h>
#include <event.h>

/*
 * Initialize the timers
 */
void timers_init(void);

/*
 * Set the periodic event
 * Time unit for duration is 1/4096 s
 */
void timer_set_periodic_event(uint16_t duration, event_handler_t handler);

/*
 * Change periodic event period
 */
void timer_periodic_set_period(uint16_t duration);

/*
 * Set one shot timer.
 * Time unit is 1/4096 s
 *
 * Returns a timer id that must be used to cancel it
 */
int timer_set_one_shot_event(uint16_t duration, event_handler_t handler);

/*
 * Cancel the one shot timer
 */
void timer_cancel_one_shot(int timer_id);


#endif /* TIMERS_TIMERS_H_ */
