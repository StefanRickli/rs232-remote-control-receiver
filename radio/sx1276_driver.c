/*
 * Driver for the SEMTECH SX1276 radio chip.
 *
 * Implementation
 */

#include <sx1276_driver.h>
#include <sx1276_lora_csts.h>
#include <sx1276_fsk_csts.h>
#include <spi.h>
#include <msp430.h>
#include <config.h>
#include <timers.h>
#include <misc.h>
#include <gpio_low_level_player_dev.h>
// ---------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------
#define RegFifo 0x00 //LoRaTM base-band FIFO data input/output. FIFO is cleared an not accessible when device is in SLEEP mode
#define RegOpMode 0x01
#define RegBitrateMsb 0x02
#define RegBitrateLsb 0x03
#define RegFdevMsb 0x04
#define RegFdevLsb 0x05
#define RegFrMsb 0x06 //MSB of RF carrier frequency Register values must be modified only when device is in SLEEP or STAND-BY mode
#define RegFrMid 0x07 //MSB of RF carrier frequency Register values must be modified only when device is in SLEEP or STAND-BY mode
#define RegFrLsb 0x08 //LSB of RF carrier frequency Register values must be modified only when device is in SLEEP or STAND-BY mode
#define RegPaConfig 0x09
#define RegPaRamp 0x0A
#define RegOcp 0x0B
#define RegLna 0x0C
#define RegFifoAddrPtr 0x0D //SPI interface address pointer in FIFO data buffer.
#define RegFifoTxBaseAdrr 0x0E //write base address in FIFO data buffer for TX modulator
#define RegFifoRxBaseAdrr 0x0F //read base address in FIFO data buffer for RX demodulator
#define RegFifoRxCurrentAdrr 0x10 //Start address (in data buffer) of last packet received
#define RegIRQFlagMask 0x11
#define RegIRQFlags 0x12
#define RegRxNbBytes 0x13 //Number of payload bytes of latest packet received
#define RegRxHeaderCntValueMsb 0x14 //Number of valid headers received since last transition into Rx modeMSB(15:8) Header and packet counters are reseted in Sleep mode
#define RegRxHeaderCntValueLsb 0x15  //Number of valid headers received since last transition into Rx modeMSB(7:0) Header and packet counters are reseted in Sleep mode
#define RegRxPacketCntValueMsb 0x16  //Number of valid packets received since last transition into Rx mode MSB(15:8) Header and packet counters are reseted in Sleep mode
#define RegRxPacketCntValueLsb 0x17  //Number of valid packets received since last transition into Rx mode MSB(7:0) Header and packet counters are reseted in Sleep mode
#define RegModemStat 0x18
#define RegPktSnrValue 0x19
#define RegPktRssiValue 0x1A
#define RegRssiValue 0x1B
#define RegHopChannel 0x1C
#define RegModemConfig1 0x1D
#define RegModemConfig2 0x1E
#define RegSymbTimeoutLsb 0x1F
#define RegPreambleMsb 0x20
#define RegPreambleLsb 0x21
#define RegPayloadLenght 0x22
#define RegMaxPayloadLenght 0x23
#define RegHopPeriod 0x24
#define RegFifoRxByteAddr 0x25
#define RegModemConfig3 0x26
#define RegDioMapping1 0x40 //Mapping of pins DIO0 to DIO3
#define RegDioMapping2 0x41 //Mapping of pins DIO4 and DIO5, ClkOut frequency
#define RegVersion //Semtech ID relating the silicon reision
#define RegTcxo 0x4B
#define RegPaDac 0x4D
#define RegFormerTemp 0x5B
#define RegAgcRef 0x61
#define RegAgcThershold1 0x61
#define RegAgcThershold2 0x62
#define RegAgcThershold3 0x63
#define RegPll 0x70

// ---------------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------------
/*
 * Some useful macros
 *
 * DIO0 is mapped to P1.3
 *
 * When receiving:
 * 	- DIO0 : RX Done
 *
 * When transmitting:
 *  - DIO0 : TX Done
 */
#define DIO0	((P1IN & BIT3) != 0)
#define DIO1	((P1IN & BIT2) != 0)
#define DIO2	((P4IN & BIT3) != 0)
#define DIO3	((P1IN & BIT5) != 0)
#define DIO4	((P3IN & BIT6) != 0)
#define DIO5	((P3IN & BIT5) != 0)

/*
 * ----------------------------------- Constants
 */
#define XTAL_FREQ 32000000
#define RF_MID_BAND_THRESH 525000000
// Used band (Hz)
#define CARRIER_FREQUENCY	868000000
// Set the carrier frequency (in Hz)
#define FREQ_STEP	61.03515625

/*
 * ----------------------------------- Internal variables
 */
// Current modem
static Modem_t _modem;

// Current channel
static uint32_t _channel;

// Timeout for packet reception set by the user.
static uint16_t _timeout;

// Fixed packet length mode. If false, then the used mode is variable packet length mode
static bool _fixed_pkt_lgth;

// Current bandwidth used by LoRa
static uint8_t _lora_bandwidth;

// IDs of the events associated to the reception of a local packet and of a lora packet, a timeout
// and a CRC error
static uint16_t _lora_pkt_rx_ev_id;
static uint16_t _local_pkt_rx_ev_id;
static uint16_t _timeout_ev_id;
static uint16_t _crc_err_ev_id;

// Timer id of the one shot timer used for timeout
static int _one_shot_timer_id;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
static const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};

/*
 * ------------------------------------------ Internal functions
 */
static void set_carrier_frequency(uint32_t freq)
{
	_channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    spi_snd_data( RegFrMsb, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    spi_snd_data( RegFrMid, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    spi_snd_data( RegFrLsb, ( uint8_t )( freq & 0xFF ) );
}

/*
 * Perform calibration.
 * Must be called just after reset
 */
static void perform_calibration(void)
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = spi_rcv_data( RegPaConfig );
    initialFreq = ( double )( ( ( uint32_t )spi_rcv_data( RegFrMsb ) << 16 ) |
                              ( ( uint32_t )spi_rcv_data( RegFrMid ) << 8 ) |
                              ( ( uint32_t )spi_rcv_data( RegFrLsb ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    spi_snd_data( RegPaConfig, 0x00 );

    // Launch Rx chain calibration for LF band
    spi_snd_data( REG_IMAGECAL, ( spi_rcv_data( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( spi_rcv_data( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING );

    // Sets a Frequency in HF band
    set_carrier_frequency(868000000);

    // Launch Rx chain calibration for HF band
    spi_snd_data( REG_IMAGECAL, ( spi_rcv_data( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( spi_rcv_data( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING );

    // Restore context
    spi_snd_data( RegPaConfig, regPaConfigInitVal );
    set_carrier_frequency( initialFreq );
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel > RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else if ((_modem == MODEM_OOK) || (_modem == MODEM_FSK))
    {
        return RF_PACONFIG_PASELECT_RFO;
    }

    return 0;	// Should not be here...
}

/*
 * Go to TX/RX/STAND BY state
 */
static void go_tx_mode(void)
{
	if (_modem == MODEM_LORA)
	{
		uint8_t r = spi_rcv_data(REG_LR_OPMODE);
		r = (r & RFLR_OPMODE_MASK) | RFLR_OPMODE_TRANSMITTER;
		spi_snd_data(REG_LR_OPMODE, r);
	}
	else if ((_modem == MODEM_OOK) || (_modem == MODEM_FSK))
	{
		uint8_t r = spi_rcv_data(REG_OPMODE);
		r = (r & RF_OPMODE_MASK) | RF_OPMODE_TRANSMITTER;
		spi_snd_data(REG_OPMODE, r);
	}
}
static void go_rx_mode(void)
{
	if (_modem == MODEM_LORA)
	{
		uint8_t r = spi_rcv_data(REG_LR_OPMODE);
		r = (r & RFLR_OPMODE_MASK) | RFLR_OPMODE_RECEIVER;
		spi_snd_data(REG_LR_OPMODE, r);
	}
	else if ((_modem == MODEM_OOK) || (_modem == MODEM_FSK))
	{
		uint8_t r = spi_rcv_data(REG_OPMODE);
		r = (r & RF_OPMODE_MASK) | RF_OPMODE_RECEIVER;
		spi_snd_data(REG_OPMODE, r);
	}
}
static void go_sbtdby_mode(void)
{
	if (_modem == MODEM_LORA)
	{
		uint8_t r = spi_rcv_data(REG_LR_OPMODE);
		r = (r & RFLR_OPMODE_MASK) | RFLR_OPMODE_STANDBY;
		spi_snd_data(REG_LR_OPMODE, r);
	}
	else if ((_modem == MODEM_OOK) || (_modem == MODEM_FSK))
	{
		uint8_t r = spi_rcv_data(REG_OPMODE);
		r = (r & RF_OPMODE_MASK) | RF_OPMODE_STANDBY;
		spi_snd_data(REG_OPMODE, r);
	}
}

/*
 * Enable/Disable DIO interrupt
 */
#define ENABLE_DIO0_IT\
	do {\
		spi_snd_data(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);\
		P1IFG &= ~BIT3;\
		P1IE |= BIT3;\
	} while(0)
#define DISABLE_DIO0_IT	(P1IE &= ~BIT3)

// ---------------------------------------------------------------------------------
// Interface functions implementations
// ----------------------------------------------------------------------------------

/*
 * External variables
 */
// RX FIFO
struct sx1276_rx_packet_t sx1276_rx_fifo[SX1276_RX_FIFO_SIZE];
volatile uint16_t sx1276_rx_fifo_first = 0u;
volatile uint16_t sx1276_rx_fifo_last = 0u;

/*
 * Initialize the radio
 */
void sx1276_init(event_handler_t local_pkt_handler, event_handler_t lora_pkt_handler,
		event_handler_t timeout_handler, event_handler_t crc_err_handler)
{
	// Setting the NRESET pin up
	P1DIR |= BIT4;
	P1OUT |= BIT4;
	// Waiting 10ms (datasheet)
	__delay_cycles(10000u);

	// Setting DIOX
	// DIO0 | P2.5 --> 1.3
	// DIO1 | P2.6 --> 1.2
	// DIO2 | P4.2 --> 4.3
	// DIO3 | P2.4 --> 1.5
	// DIO4 | P4.3 --> 3.6
	// DIO5 | P3.6 --> 3.5
	P1DIR &= ~(BIT2 + BIT3 + BIT5);
	P1OUT &= ~(BIT2 + BIT3 + BIT5);	// Pull down
	P1IE &= ~(BIT2 + BIT3 + BIT5);	// Interrupt disabled
	P1IFG &= ~BIT3;	// Clear DIO0 interrupt
	P1IES &= ~BIT3;	// Interrupt on a low to high transiion
	P3DIR &= ~(BIT5 + BIT6);
	P3OUT &= ~(BIT5 + BIT6);
	P3IE &= ~(BIT5 + BIT6);
	P4DIR &= ~(BIT3);
	P4OUT &= ~(BIT3);	// Pull down
	P4IE &= ~(BIT3);	// Interrupt disabled

	// Carrier frequency set to 868 MHz
	set_carrier_frequency(CARRIER_FREQUENCY);

	perform_calibration();

	// Add the events associated to a local packet reception and lora packet reception
	_local_pkt_rx_ev_id = event_add(local_pkt_handler);
	_lora_pkt_rx_ev_id = event_add(lora_pkt_handler);
	_timeout_ev_id = event_add(timeout_handler);
	_crc_err_ev_id = event_add(crc_err_handler);

	_modem = MODEM_NONE;
}

/*
 * Hard reset.
 *
 * After the reset, calibration and carrier frequency are set
 */
void sx1276_reset()
{
	P1OUT &= ~BIT4;
	__delay_cycles(1000u);
	P1OUT |= BIT4;
	__delay_cycles(6000u);

	// Carrier frequency set to 868 MHz
	set_carrier_frequency(CARRIER_FREQUENCY);

	perform_calibration();

	_modem = MODEM_NONE;
}

/*
 * Get the currently used modulation scheme
 */
Modem_t sx1276_get_modem(void)
{
	return _modem;
}

/*
 * Handle interrupts from DIO0 when receiving a packet
 */
#pragma vector=PORT1_VECTOR
__interrupt void isr_gpio_p1(void)
{
	switch (__even_in_range(P1IV, P1IV_P1IFG7)) {
	case P1IV_NONE:
		break;
	case P1IV_P1IFG0:
		break;
	case P1IV_P1IFG1:
		__delay_cycles(200);
		while(!(P1IN & BIT1));
		__delay_cycles(200);
		P1IFG &= ~BIT1;
		EVENT_SIGNAL_ISR(_sw2_event);
		break;
	case P1IV_P1IFG2:
		break;
	case P1IV_P1IFG3: //interrupt is from LoRa (only RX the way it is configured now
	{
				// Stop RX timeout timer
				timer_cancel_one_shot(_one_shot_timer_id);

				// Put the received data into the RX fifo
				if (_modem == MODEM_LORA)
				{
					uint8_t irq_flags = spi_rcv_data(REG_LR_IRQFLAGS), j, pkt_sz, dst_address;

					// Put the transceiver in stand-by mode
					go_sbtdby_mode();
					__delay_cycles(1000u);

					// Clear the flag
					spi_snd_data(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

					if ((irq_flags & RFLR_IRQFLAGS_PAYLOADCRCERROR) != 0u)
					{
						spi_snd_data(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);
						EVENT_SIGNAL_ISR(_crc_err_ev_id);
						goto rx_pkt_it_end;
					}

					// Reading the base address of the last received packet
					pkt_sz = spi_rcv_data(REG_LR_FIFORXCURRENTADDR);
					spi_snd_data(REG_LR_FIFOADDRPTR, pkt_sz);
					pkt_sz = spi_rcv_data(REG_LR_RXNBBYTES);
					dst_address = spi_rcv_data(REG_LR_FIFO);
					sx1276_rx_fifo[sx1276_rx_fifo_last].address = spi_rcv_data(REG_LR_FIFO);
					pkt_sz -= 2; // Remove 2 for the source address and destination address
					for (j = 0u; j < pkt_sz; j++)
					{
						sx1276_rx_fifo[sx1276_rx_fifo_last].data[j] = spi_rcv_data(REG_LR_FIFO);
					}
					sx1276_rx_fifo[sx1276_rx_fifo_last].size = pkt_sz;
					// Incrementing the FIFO and signaling the event only if the packet was for us or broadcast
					if ((dst_address == NODE_ADDRESS) || (dst_address == BROADCAST_ADDRESS))
					{
						FIFO_INCR(sx1276_rx_fifo_last, SX1276_RX_FIFO_SIZE);
						EVENT_SIGNAL_ISR(_lora_pkt_rx_ev_id);
					}
				}
				else if ((_modem == MODEM_OOK) || (_modem == MODEM_FSK))
				{
					uint8_t j, irq_flags;

					irq_flags = spi_rcv_data(REG_IRQFLAGS2);

					if ((irq_flags & RF_IRQFLAGS2_CRCOK) == 0)
					{
						// Clear the FIFO
						// NOTE: Is there a better way to do this ?
						while ((spi_rcv_data(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOEMPTY) == 0)
						{
							spi_rcv_data(REG_FIFO);
						}
						EVENT_SIGNAL_ISR(_crc_err_ev_id);
						goto rx_pkt_it_end;
					}

					// Reading the address and size, then data
					sx1276_rx_fifo[sx1276_rx_fifo_last].size = spi_rcv_data(REG_FIFO);
					spi_rcv_data(REG_FIFO);	// Because of hardware address filtering, we don't need to check if the packet was intended to us.
					sx1276_rx_fifo[sx1276_rx_fifo_last].address = spi_rcv_data(REG_FIFO);
					j = 0;
					while ((spi_rcv_data(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOEMPTY) == 0)
					{
						sx1276_rx_fifo[sx1276_rx_fifo_last].data[j++] = spi_rcv_data(REG_FIFO);
					}
					FIFO_INCR(sx1276_rx_fifo_last, SX1276_RX_FIFO_SIZE);

					EVENT_SIGNAL_ISR(_local_pkt_rx_ev_id);
					spi_snd_data(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
				}
				rx_pkt_it_end:
				// Reset the interrupt flag
				P1IFG &= ~BIT3;
			}
		break;
	case P1IV_P1IFG4:
		break;
	case P1IV_P1IFG5:
		//EVENT_SIGNAL_ISR(_wurx_ev);
		P1IFG &= ~BIT5;
		break;
	case P1IV_P1IFG6:
		break;
	case P1IV_P1IFG7:
		break;
	}
}

void rx_timeout_handler(void)
{
	go_sbtdby_mode();
	event_signal(_timeout_ev_id);
}

/*
 * Receive a packet.
 * The size of the payload is assumed to be the one set using 'payload_length'
 */
void sx1276_rx_single_pkt(void)
{
	if (_modem == MODEM_LORA)
	{
		spi_snd_data( REG_LR_INVERTIQ, ( ( spi_rcv_data( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
        spi_snd_data( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );

        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        if( _lora_bandwidth < 9 )
        {
            spi_snd_data( REG_LR_DETECTOPTIMIZE, spi_rcv_data( REG_LR_DETECTOPTIMIZE ) & 0x7F );
            spi_snd_data( REG_LR_TEST30, 0x00 );
            switch( _lora_bandwidth )
            {
            case 0: // 7.8 kHz
                spi_snd_data( REG_LR_TEST2F, 0x48 );
                set_carrier_frequency(_channel + 7.81e3 );
                break;
            case 1: // 10.4 kHz
                spi_snd_data( REG_LR_TEST2F, 0x44 );
                set_carrier_frequency(_channel + 10.42e3 );
                break;
            case 2: // 15.6 kHz
                spi_snd_data( REG_LR_TEST2F, 0x44 );
                set_carrier_frequency(_channel + 15.62e3 );
                break;
            case 3: // 20.8 kHz
                spi_snd_data( REG_LR_TEST2F, 0x44 );
                set_carrier_frequency(_channel + 20.83e3 );
                break;
            case 4: // 31.2 kHz
                spi_snd_data( REG_LR_TEST2F, 0x44 );
                set_carrier_frequency(_channel + 31.25e3 );
                break;
            case 5: // 41.4 kHz
                spi_snd_data( REG_LR_TEST2F, 0x44 );
                set_carrier_frequency(_channel + 41.67e3 );
                break;
            case 6: // 62.5 kHz
                spi_snd_data( REG_LR_TEST2F, 0x40 );
                break;
            case 7: // 125 kHz
                spi_snd_data( REG_LR_TEST2F, 0x40 );
                break;
            case 8: // 250 kHz
                spi_snd_data( REG_LR_TEST2F, 0x40 );
                break;
            }
        }
        else
        {
            spi_snd_data( REG_LR_DETECTOPTIMIZE, spi_rcv_data( REG_LR_DETECTOPTIMIZE ) | 0x80 );
        }

		// Setting DIOs so that:
		// - DIO0 corresponds to RX DONE
		spi_snd_data(REG_LR_DIOMAPPING1, 0x0u);

		// Setting the timeout timer
		if (_timeout != 0)
		{
			_one_shot_timer_id = timer_set_one_shot_event(_timeout, rx_timeout_handler);
		}

		ENABLE_DIO0_IT;

		go_rx_mode();

		volatile uint8_t r = spi_rcv_data(REG_LR_IRQFLAGS);
	}
	else if ((_modem == MODEM_OOK) || (_modem == MODEM_FSK))
	{
		// Setting DIOs so that:
		// - DIO0 corresponds to PayloadReady
		spi_snd_data(REG_DIOMAPPING1, 0x0u);

		// Setting the timeout timer
		if (_timeout != 0)
		{
			_one_shot_timer_id = timer_set_one_shot_event(_timeout, rx_timeout_handler);
		}

		ENABLE_DIO0_IT;

		go_rx_mode();

		// We wait for either a timeout or a received packet
		volatile uint8_t r = spi_rcv_data(REG_IRQFLAGS1);
	}
}

/*
 * Transmit a packet
 */
void sx1276_tx_pkt(char *data, uint8_t pkt_size, uint8_t address)
{
	uint8_t i;

	DISABLE_DIO0_IT;

	if (_modem == MODEM_LORA)
	{
		uint8_t fifo_addr;

		// Go to standby
		go_sbtdby_mode();
		__delay_cycles(1000u);

        spi_snd_data( REG_LR_INVERTIQ, ( ( spi_rcv_data( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
        spi_snd_data( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );

		// Setting DIOs so that:
		// - DIO0 corresponds to TX DONE
		spi_snd_data( RegDioMapping1, ( spi_rcv_data( RegDioMapping1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );

		// Set the payload size
		spi_snd_data(REG_LR_PAYLOADLENGTH, pkt_size+2);	// +2 for the dst address and src address

		fifo_addr = spi_rcv_data(REG_LR_FIFOTXBASEADDR);
		spi_snd_data(REG_LR_FIFOADDRPTR, fifo_addr);

		// Writing the address of the recipient in the TX FIFO
		spi_snd_data(REG_LR_FIFO, address);
		// Writting our address in the TX FIFO
		spi_snd_data(REG_LR_FIFO, NODE_ADDRESS);
		// Writing the data in the transceiver TX FIFO
		for (i = 0u; i < pkt_size; i++)
		{
			spi_snd_data(REG_LR_FIFO, data[i]);
		}

		// Going to TX mode
		go_tx_mode();

		// Waiting for the end of the transmission
		volatile uint8_t r = spi_rcv_data(REG_LR_IRQFLAGS);
		while ( !DIO0 ); //seems to hang here during debug, reason not clear RK
		spi_snd_data(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
	}
	else if ((_modem == MODEM_OOK) || (_modem == MODEM_FSK))
	{
		// Setting DIOs so that:
		// - DIO0 corresponds to TX DONE
		spi_snd_data( RegDioMapping1, 0x0u);

		// Go to standby mode for FIFO operation
		go_sbtdby_mode();
		__delay_cycles(1000u);

		if (_fixed_pkt_lgth)
		{
			// Set the payload size
			spi_snd_data(REG_PAYLOADLENGTH, pkt_size+1); // +1 for our address
		}
		else
		{
			spi_snd_data(REG_FIFO, pkt_size+1);
		}

		// Writing the address of the recipient in the TX FIFO
		if (_modem != MODEM_OOK)
		{
			spi_snd_data(REG_FIFO, address);	// Address of the recipirent
			spi_snd_data(REG_FIFO, NODE_ADDRESS);	// Our address
		}
		// Writing the data in the transceiver TX FIFO
		for (i = 0u; i < pkt_size; i++)
		{
			spi_snd_data(REG_FIFO, data[i]);
		}

		// Going to TX mode
		go_tx_mode();

		// Waiting for the end of the transmission
		volatile uint8_t r = spi_rcv_data(REG_LR_IRQFLAGS);
		while ( !DIO0 );

		spi_snd_data(REG_IRQFLAGS2, RF_IRQFLAGS2_PACKETSENT);
	}
}

void sx1276_set_rx_config( Modem_t modem, uint32_t bandwidth,
        uint32_t datarate, uint8_t coderate,
        uint32_t bandwidthAfc, uint16_t preambleLen,
        uint16_t timeout, bool fixLen,
        uint8_t payloadLen,
        bool crcOn)
{
	_modem = modem;
	_timeout = timeout;

	if ((modem == MODEM_FSK) || (modem == MODEM_OOK))
	{
		uint8_t d = RF_OPMODE_LONGRANGEMODE_OFF
				+ RF_OPMODE_SLEEP
				+ ((modem == MODEM_FSK) ? RF_OPMODE_MODULATIONTYPE_FSK : RF_OPMODE_MODULATIONTYPE_OOK)
				+ RFLR_OPMODE_FREQMODE_ACCESS_HF;
		spi_snd_data(REG_OPMODE, d);

		spi_snd_data(REG_LNA, 0x23);
	    spi_snd_data(REG_RXCONFIG, 0x1E);
	    spi_snd_data(REG_RSSICONFIG, 0xD2);
	    spi_snd_data(REG_PREAMBLEDETECT, 0xAA);
	    spi_snd_data(REG_OSC, 0x07u);
	    spi_snd_data(REG_SYNCCONFIG, 0x12);
	    spi_snd_data(REG_SYNCVALUE1, 0xC1);
	    spi_snd_data(REG_SYNCVALUE2, 0x94);
	    spi_snd_data(REG_SYNCVALUE3, 0xC1);
	    spi_snd_data(REG_PACKETCONFIG1, 0x1C);
	    spi_snd_data(REG_FIFOTHRESH, 0x8F);
	    spi_snd_data(REG_IMAGECAL, 0x02);
	    spi_snd_data(REG_NODEADRS, NODE_ADDRESS);
	    spi_snd_data(REG_BROADCASTADRS, BROADCAST_ADDRESS);

		datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
		spi_snd_data( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
		spi_snd_data( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

		spi_snd_data( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
		spi_snd_data( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

		spi_snd_data( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
		spi_snd_data( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

		if( fixLen == 1 )
		{
			spi_snd_data( REG_PAYLOADLENGTH, payloadLen );
		}

		spi_snd_data( REG_PACKETCONFIG1,
					 ( spi_rcv_data( REG_PACKETCONFIG1 ) &
					   RF_PACKETCONFIG1_CRC_MASK &
					   RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
					   ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
					   ( crcOn << 4 ) );
	}
	else if (modem == MODEM_LORA)
	{
		uint8_t d = RFLR_OPMODE_LONGRANGEMODE_ON
				+ RFLR_OPMODE_SLEEP
				+ RFLR_OPMODE_ACCESSSHAREDREG_DISABLE
				+ RFLR_OPMODE_FREQMODE_ACCESS_HF;
		spi_snd_data(REG_OPMODE, d);

		if( bandwidth > 2 )
		{
			// Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
			while( 1 );
		}
		bandwidth += 7;
		_lora_bandwidth = bandwidth;

		if( datarate > 12 )
		{
			datarate = 12;
		}
		else if( datarate < 6 )
		{
			datarate = 6;
		}

		uint8_t LowDatarateOptimize;
		if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
			( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
		{
			LowDatarateOptimize = 0x01;
		}
		else
		{
			LowDatarateOptimize = 0x00;
		}

		spi_snd_data( REG_LR_MODEMCONFIG1,
					 ( spi_rcv_data( REG_LR_MODEMCONFIG1 ) &
					   RFLR_MODEMCONFIG1_BW_MASK &
					   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
					   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
					   ( bandwidth << 4 ) | ( coderate << 1 ) |
					   fixLen );

		spi_snd_data( REG_LR_MODEMCONFIG2,
					 ( spi_rcv_data( REG_LR_MODEMCONFIG2 ) &
					   RFLR_MODEMCONFIG2_SF_MASK &
					   RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
					   RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
					   ( datarate << 4 ) | ( crcOn << 2 ) |
					   ( ( timeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

		spi_snd_data( REG_LR_MODEMCONFIG3,
					 ( spi_rcv_data( REG_LR_MODEMCONFIG3 ) &
					   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
					   ( LowDatarateOptimize << 3 ) );

		spi_snd_data( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( timeout & 0xFF ) );

		spi_snd_data( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
		spi_snd_data( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

		if( fixLen == 1 )
		{
			spi_snd_data( REG_LR_PAYLOADLENGTH, payloadLen );
		}

		if( ( bandwidth == 9 ) && ( RF_MID_BAND_THRESH ) )
		{
			// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
			spi_snd_data( REG_LR_TEST36, 0x02 );
			spi_snd_data( REG_LR_TEST3A, 0x64 );
		}
		else if( bandwidth == 9 )
		{
			// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
			spi_snd_data( REG_LR_TEST36, 0x02 );
			spi_snd_data( REG_LR_TEST3A, 0x7F );
		}
		else
		{
			// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
			spi_snd_data( REG_LR_TEST36, 0x03 );
		}

		if( datarate == 6 )
		{
			spi_snd_data( REG_LR_DETECTOPTIMIZE,
						 ( spi_rcv_data( REG_LR_DETECTOPTIMIZE ) &
						   RFLR_DETECTIONOPTIMIZE_MASK ) |
						   RFLR_DETECTIONOPTIMIZE_SF6 );
			spi_snd_data( REG_LR_DETECTIONTHRESHOLD,
						 RFLR_DETECTIONTHRESH_SF6 );
		}
		else
		{
			spi_snd_data( REG_LR_DETECTOPTIMIZE,
						 ( spi_rcv_data( REG_LR_DETECTOPTIMIZE ) &
						 RFLR_DETECTIONOPTIMIZE_MASK ) |
						 RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
			spi_snd_data( REG_LR_DETECTIONTHRESHOLD,
						 RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
		}
	}
}

void sx1276_set_tx_config( Modem_t modem, int8_t power, uint32_t fdev,
        uint32_t bandwidth, uint32_t datarate,
        uint8_t coderate, uint16_t preambleLen,
        bool fixLen, bool crcOn )
{
	_modem = modem;

	uint8_t paConfig = 0;
	uint8_t paDac = 0;

	paConfig = spi_rcv_data( REG_PACONFIG );
	paDac = spi_rcv_data( REG_PADAC );

	paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( _channel);
	paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

	_fixed_pkt_lgth = fixLen;

	if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
	{
		if( power > 17 )
		{
			paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
		}
		else
		{
			paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
		}
		if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
		{
			if( power < 5 )
			{
				power = 5;
			}
			if( power > 20 )
			{
				power = 20;
			}
			paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
		}
		else
		{
			if( power < 2 )
			{
				power = 2;
			}
			if( power > 17 )
			{
				power = 17;
			}
			paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
		}
	}
	else
	{
		if( power < -1 )
		{
			power = -1;
		}
		if( power > 14 )
		{
			power = 14;
		}
		paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
	}
	spi_snd_data( REG_PACONFIG, paConfig );
	spi_snd_data( REG_PADAC, paDac );

	if ((modem == MODEM_FSK) || (modem == MODEM_OOK))
	{
		uint8_t d = RF_OPMODE_LONGRANGEMODE_OFF
				+ RF_OPMODE_SLEEP
				+ ((modem == MODEM_FSK) ? RF_OPMODE_MODULATIONTYPE_FSK : RF_OPMODE_MODULATIONTYPE_OOK)
				+ RFLR_OPMODE_FREQMODE_ACCESS_HF;
		spi_snd_data(REG_OPMODE, d);

		if (modem == MODEM_FSK)
		{
			fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
			spi_snd_data( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
			spi_snd_data( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );
		}

		datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
		spi_snd_data( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
		spi_snd_data( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

		spi_snd_data( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
		spi_snd_data( REG_PREAMBLELSB, preambleLen & 0xFF );

		spi_snd_data( REG_PACKETCONFIG1,
					 ( spi_rcv_data( REG_PACKETCONFIG1 ) &
					   RF_PACKETCONFIG1_CRC_MASK &
					   RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
					   ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
					   ( crcOn << 4 ) );

	}
	else if (modem == MODEM_LORA)
	{
		uint8_t d = RFLR_OPMODE_LONGRANGEMODE_ON
				+ RFLR_OPMODE_SLEEP
				+ RFLR_OPMODE_ACCESSSHAREDREG_DISABLE
				+ RFLR_OPMODE_FREQMODE_ACCESS_HF;
		spi_snd_data(REG_OPMODE, d);

		if( bandwidth > 2 )
		{
			// Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
			while( 1 );
		}
		bandwidth += 7;

		if( datarate > 12 )
		{
			datarate = 12;
		}
		else if( datarate < 6 )
		{
			datarate = 6;
		}

		uint8_t LowDatarateOptimize;
		if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
			( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
		{
			LowDatarateOptimize = 0x01;
		}
		else
		{
			LowDatarateOptimize = 0x00;
		}

		spi_snd_data( REG_LR_MODEMCONFIG1,
					 ( spi_rcv_data( REG_LR_MODEMCONFIG1 ) &
					   RFLR_MODEMCONFIG1_BW_MASK &
					   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
					   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
					   ( bandwidth << 4 ) | ( coderate << 1 ) |
					   fixLen );

		spi_snd_data( REG_LR_MODEMCONFIG2,
					 ( spi_rcv_data( REG_LR_MODEMCONFIG2 ) &
					   RFLR_MODEMCONFIG2_SF_MASK &
					   RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
					   ( datarate << 4 ) | ( crcOn << 2 ) );

		spi_snd_data( REG_LR_MODEMCONFIG3,
					 ( spi_rcv_data( REG_LR_MODEMCONFIG3 ) &
					   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
					   ( LowDatarateOptimize << 3 ) );

		spi_snd_data( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
		spi_snd_data( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

		if( datarate == 6 )
		{
			spi_snd_data( REG_LR_DETECTOPTIMIZE,
						 ( spi_rcv_data( REG_LR_DETECTOPTIMIZE ) &
						   RFLR_DETECTIONOPTIMIZE_MASK ) |
						   RFLR_DETECTIONOPTIMIZE_SF6 );
			spi_snd_data( REG_LR_DETECTIONTHRESHOLD,
						 RFLR_DETECTIONTHRESH_SF6 );
		}
		else
		{
			spi_snd_data( REG_LR_DETECTOPTIMIZE,
						 ( spi_rcv_data( REG_LR_DETECTOPTIMIZE ) &
						 RFLR_DETECTIONOPTIMIZE_MASK ) |
						 RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
			spi_snd_data( REG_LR_DETECTIONTHRESHOLD,
						 RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
		}
	}
}

/*
 * Enable/Disable Sync word generation and detection
 */
void sx1276_enable_sync_word(void)
{
	uint8_t r = spi_rcv_data(REG_SYNCCONFIG);
	r |= RF_SYNCCONFIG_SYNC_ON;
	spi_snd_data(REG_SYNCCONFIG, r);
}

void sx1276_disable_sync_word(void)
{
	uint8_t r = spi_rcv_data(REG_SYNCCONFIG);
	r &= ~RF_SYNCCONFIG_SYNC_ON;
	spi_snd_data(REG_SYNCCONFIG, r);
}

/*
 * Put the transceiver in the sleep state
 */
void sx1276_sleep(void)
{
	DISABLE_DIO0_IT;

	uint8_t r = spi_rcv_data(REG_OPMODE);
	r = (r & RF_OPMODE_MASK) | RF_OPMODE_SLEEP;
	spi_snd_data(REG_OPMODE, r);

	__delay_cycles(1000u);
}
