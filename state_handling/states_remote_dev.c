/*
 * states_remote_dev.c
 *
 *  Created on: 23.05.2016
 *      Author: stefa
 */

#include <ascii_comm_functions_player_dev.h>
#include "states_remote_dev.h"

#include <string.h>

#include "config.h"
#include "gpio_low_level_player_dev.h"
#include "misc.h"

enum Player_Mecha_Status current_player_state = NOT_RESPONDING;
unsigned int current_track_no = 0;
char current_track_name[MAX_TRACK_NAME_LENGTH];

void state_not_responding(void) {
	current_player_state = NOT_RESPONDING;
	led1_off();
	led2_off();
	led3_off();
	debug_uart_sendstr("state_not_responding");
}

void state_boot_up(void) {
	current_player_state = BOOT_UP;
	led1_off();
	led2_off();
	led3_off();
	debug_uart_sendstr("state_boot_up");
}

void state_trans_stop(void){
	current_player_state = STOP;
	led1_on();
	led2_off();
	led3_off();
	debug_uart_sendstr("state_trans_stop");
}

void state_trans_play(void){
	current_player_state = PLAY;
	led1_off();
	led2_on();
	led3_off();
	debug_uart_sendstr("state_trans_start");
}

void state_trans_ready(void){
	current_player_state = READY_ON;
	led1_off();
	led2_off();
	led3_on();
	debug_uart_sendstr("state_trans_ready");
}

void state_trans_other(void){
	current_player_state = OTHER;
	led1_off();
	led2_off();
	led3_off();
	debug_uart_sendstr("state_trans_other");
}

void update_state(enum Player_Mecha_Status new_player_state) {
	if (current_player_state == new_player_state) {
		// nothing to do
		return;
	}

	switch (new_player_state) {
	case NOT_RESPONDING:
		// TODO RIC: recheck status after XX ms
		state_not_responding();
		break;
	case BOOT_UP:
		// TODO RIC: recheck status after XX ms
		state_boot_up();
		break;
	case STOP:
		state_trans_stop();
		break;
	case PLAY:
		state_trans_play();
		break;
	case READY_ON:
		state_trans_ready();
		break;
	case NO_MEDIA:
	case EJECT:
	case MONITOR:
	case RECORD:
	case RECORD_READY:
	case INFORMATION_WRITING:
		state_trans_other();
		break;
	case EMPTY_STATUS:
		break;
	default:
		panic("undefined state received");
		break;
	}
}

enum Player_Mecha_Status get_player_status(void) {
	return current_player_state;
}

inline void update_track_no(unsigned int new_track_no) {
	current_track_no = new_track_no;
}

inline unsigned int get_track_no(void) {
	return current_track_no;
}

void update_track_name(char new_track_name[]) {
	strcpy(current_track_name, new_track_name);
}

inline char* get_track_name(void) {
	return current_track_name;
}
