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

#include <event.h>
#include <stdbool.h>
#include <misc.h>
#include <msp430.h>

// -------------------------------------------------------------------
// Internals
// -------------------------------------------------------------------
// An event
struct event_t
{
	event_handler_t handler;	// Handler for the event
	uint16_t id;	// id of the event.
	bool pending;
};

// The events
static struct event_t _events[EVENT_MAX_COUNT];

// Next availe event ID
static uint16_t _next_event_id = 0u;


// List of events that arrised.
// Circular buffer used as a FIFO
static uint16_t _pending_events[EVENT_MAX_COUNT];
static uint16_t _pending_events_first = 0u;
static uint16_t _pending_events_last = 0u;

// -------------------------------------------------------------------
// Implementation of the interface
// -------------------------------------------------------------------

/*
 * Initialize the event handler, which is the core of
 */
void event_init(void)
{
	// Nothing here
}

/*
 * Add an event
 */
uint16_t event_add(event_handler_t handler)
{
	uint16_t id;

	id = _next_event_id++;
	_events[id].pending = false;
	_events[id].handler = handler;
	_events[id].id = id;

	return id;
}

/*
 * Change the handler of an event
 */
void event_set_handler(uint16_t id, event_handler_t handler)
{
	_events[id].handler = handler;
}

/*
 * Signal an event
 */
int event_signal(uint16_t id)
{
	// Nothing to do if the event is already pending
	if (_events[id].pending)
	{
		return -1;
	}

	_events[id].pending = true;
	_pending_events[_pending_events_last] = id;
	FIFO_INCR(_pending_events_last, EVENT_MAX_COUNT);
	return 0;
}

/*
 * Main loop of events.
 * Start the main loop.
 * This function should be called in the main after all the initializations.
 * It never returns.
 */
void event_loop(void)
{
	for ( ;; )
	{
		// Process the pending events
		while (!FIFO_EMPTY(_pending_events_first, _pending_events_last))
		{
			uint16_t id = _pending_events[_pending_events_first];
			_events[id].handler();
			_events[id].pending = false;
			FIFO_INCR(_pending_events_first, EVENT_MAX_COUNT);
		}
		// zzZz...
		//_bis_SR_register(LPM3_bits + GIE);
	}
}
