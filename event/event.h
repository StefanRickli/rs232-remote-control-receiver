/*
 * Event handler is the core of this program.
 *
 * A user can add an event and associate a handler to it.
 * When the event happen, the handler is called.
 * Each event is identified by the id returned by the function event_add(handler).
 * When the event happens, it should be signaled via the function event_signal(id).
 *
 * This make evenmential programming more easy.
 */

#ifndef EVENT_EVENT_H_
#define EVENT_EVENT_H_

#include <stdint.h>

/*
 * Initialize the event handler, which is the core of
 */
void event_init(void);

/*
 * Handler use to handler the events
 */
typedef void (*event_handler_t)(void);

/*
 * Add an event
 */
uint16_t event_add(event_handler_t handler);

/*
 * Change the handler of an event
 */
void event_set_handler(uint16_t id, event_handler_t handler);

/*
 * Signal an event. This function should be used when signaling an event OUTSIDE AN INTERRUPT HANDLER.
 * Use the MACRO 'EVENT_SIGNAL_ISR' to signal an event from an interrupt handler
 */
int event_signal(uint16_t id);

#define EVENT_SIGNAL_ISR(id)\
	do\
	{\
		event_signal(id);\
		LPM1_EXIT;\
	} while (0)

/*
 * Main loop of events.
 * Start the main loop.
 * This function should be called in the main after all the initializations.
 * It never returns.
 */
void event_loop(void);

/*
 * Maximum number of event that can be declared
 */
#define EVENT_MAX_COUNT	32u


#endif /* EVENT_EVENT_H_ */
