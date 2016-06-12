/*
 * config.h
 *
 *  Created on: 3 mars 2016
 *      Author: faitaoudia
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//#LORA_ENABLE

#define DEBUG_UART_RX_BUFFER_SIZE	64u
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

/*
 * Packet type
 */
enum Packet_type_t
{
	SENSING_DATA,			// Deprecated, Sensing data
	PLAYER_COMMAND,			// Command is in Tascam formatting
	CUSTOM_COMMAND,			// Command is in custom formatting (one int following)
	PLAYER_COMMAND_SUCC,	// Command has been executed successfully
	PLAYER_COMMAND_RESPONSE,// Response with one int for the command and data (char[])
	CUSTOM_COMMAND_RESPONSE	// Response with one int for the command and data (char[])
};

struct Packet_t
{
	enum Packet_type_t type;	// Type of the packet
	int command_no;				// Information: specific to each type
	enum Player_Mecha_Status player_status;
	char text[MAX_TRACK_NAME_LENGTH];
};

// Broadcast address
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
#define CLUSTER_HEAD_ADDRESS	0x55u

// Address of the sink
#define SINK_ADDRESS			0x54u

// Address of the end-devices sensor nodes
#define SENSOR_NODE_1			0x53u

// My address
#define NODE_ADDRESS	SENSOR_NODE_1


#endif /* CONFIG_H_ */
