/*
 * states_remote_dev.h
 *
 *  Created on: 23.05.2016
 *      Author: stefa
 */

#ifndef STATE_HANDLING_STATES_REMOTE_DEV_H_
#define STATE_HANDLING_STATES_REMOTE_DEV_H_

#include "config.h"

void state_not_responding(void);

void state_boot_up(void);

void state_trans_stop(void);

void state_trans_play(void);

void state_trans_ready(void);

void state_trans_other(void);

void update_state(enum Player_Mecha_Status);

enum Player_Mecha_Status get_player_status(void);

void update_track_no(unsigned int);

unsigned int get_track_no(void);

void update_track_name(char[]);

inline char* get_track_name(void);

#endif /* STATE_HANDLING_STATES_REMOTE_DEV_H_ */
