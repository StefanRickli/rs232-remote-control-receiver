/*
 * config.h
 *
 *  Created on: 3 mars 2016
 *      Author: faitaoudia
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define PLAYER_DEV // to make it easier to copy paste Code RK
#define LORA_ENABLE

#define DEBUG_UART_RX_BUFFER_SIZE	64u //required by UART code: DON'T TOUCH
#define DEBUG_UART_TX_BUFFER_SIZE	64u
#define COMM_UART_RX_BUFFER_SIZE	64u
#define COMM_UART_TX_BUFFER_SIZE	64u
#define TASCAM_UART_RX_BUFFER_SIZE	128u
#define TASCAM_UART_TX_BUFFER_SIZE	128u

#define MAX_TRACK_NAME_LENGTH		17u	// 16 chars for string + null-termination

enum Player_Mecha_Status
{
	EMPTY_STATUS,
	NOT_RESPONDING,
	BOOT_UP,
	NO_MEDIA,
	EJECT,
	STOP,
	PLAY,
	READY_ON,
	MONITOR,
	RECORD,
	RECORD_READY,
	INFORMATION_WRITING,
	OTHER
};

#define BROADCAST_ADDRESS	0x00u

// Assumed maximum payload packet size
#define MAX_PACKET_SIZE	16	// bytes

// Wake-up beacons preamble
#define WUB_PREAMBLE	0xEAu

// Sensing period
// Unit: 1/4096 s
#define MAX_SENSING_PERIOD	0xFFFF	// ~15s
#define MIN_SENSING_PERIOD	3*4096u	// 3s

// Address of the cluster head
#define PLAYER_ADDRESS	0x55u

// Address of the sink
#define REMOTE_ADDRESS	0x54u

// Address of the end-devices sensor nodes
#define SENSOR_NODE_1			0x53u

// My address
#ifdef PLAYER_DEV
#define NODE_ADDRESS	PLAYER_ADDRESS
#define TARGET_ADDRESS REMOTE_ADDRESS

#elif defined(REMOTE_DEV)
#define NODE_ADDRESS	REMOTE_ADDRESS
#define TARGET_ADDRESS PLAYER_ADDRESS
#endif

#endif /* CONFIG_H_ */

