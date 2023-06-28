/*! ----------------------------------------------------------------------------
 *  @file    ss_twr_initiator_sts.c
 *  @brief   Single-sided two-way ranging (SS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "ss_twr_responder_sts" example
 *           code (companion to this application) to complete the exchange.
 *
 *           This example utilises the 802.15.4z STS to accomplish secure timestamps between the initiator and responder. A 32-bit STS counter
 *           is part of the STS IV used to generate the scrambled timestamp sequence (STS) in the transmitted packet and to cross correlate in the
 *           receiver. This count normally advances by 1 for every 1024 chips (~2탎) of STS in BPRF mode, and by 1 for every 512 chips (~1탎) of STS
 *           in HPRF mode. If both devices (initiator and responder) have count values that are synced, then the communication between devices should
 *           result in secure timestamps which can be used to calculate distance. If not, then the devices need to re-sync their STS counter values.
 *           In this example, the initiator will send a plain-text value of it's 32-bit STS counter inside the "poll" frame. The receiver first
 *           checks the quality of the STS of the received frame. If the received frame has bad STS quality, it can then use the plain-text
 *           counter value received to adjust it's own STS counter value to match. This means that the next message in the sequence should be in sync again.
 *
 * @attention
 *
 * Copyright 2019 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <deca_types.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <stdlib.h>
#include <udp_echoclient.h>
#include "dhcp.h"
#include "app_ethernet.h"

#if defined(TEST_SS_TWR_INITIATOR_STS)

extern void ethernetif_input(struct netif *netif);
extern void test_run_info(unsigned char *data);
extern void udp_echoclient_send(unsigned char *data);
extern struct netif gnetif;
extern char dist_str[32];
/* Example application name */
#define APP_NAME "SS TWR INIT v1.0"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t tx_poll_msg1[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '1', 'V', 'E', 0xE0, 0, 0 }; // 63이므로 MAC
static uint8_t tx_poll_msg2[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '2', 'V', 'E', 0xE0, 0, 0 };
static uint8_t tx_poll_msg3[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '3', 'V', 'E', 0xE0, 0, 0 }; //11번째인덱스까지, 아래는 12부터 시작
static uint8_t tx_poll_msg4[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '4', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//인덱스 총 38
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 41이므로 Data.  x시작 14번, y시작 24번
static uint8_t udp_msg[32]={0,};
/* Frames used in the ranging process. See NOTE 3 below. */
//static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0 };
//static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 2;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS (300 + CPU_PROCESSING_TIME)
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 700

/* Hold copy of diagnostics data so that it can be examined at a debug breakpoint. */
static dwt_rxdiag_t rx_diag;

/* Hold copy of accumulator data so that it can be examined at a debug breakpoint. See NOTE 2. */
//#define ACCUM_DATA_LEN (1016 * (3 + 3) + 1) //  64PRF는 1016개의 CIR을 갖고, 3개의 real num과 3개의 false 넘버. + 더미 바이트 [0]으로인해 1추가
//static uint8_t accum_data[ACCUM_DATA_LEN];

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Hold the amount of errors that have occurred */
static uint32_t errors[23] = { 0 };

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;

/*
 * 128-bit STS key to be programmed into CP_KEY register.
 *
 * This key needs to be known and programmed the same at both units performing the SS-TWR.
 * In a real application for security this would be private and unique to the two communicating units
 * and chosen/assigned in a secure manner lasting just for the period of their association.
 *
 * Here we use a default KEY as specified in the IEEE 802.15.4z annex
 */
static dwt_sts_cp_key_t cp_key = { 0x14EB220F, 0xF86050A8, 0xD1D336AA, 0x14148674 };

/*
 * 128-bit initial value for the nonce to be programmed into the CP_IV register.
 *
 * The IV, like the key, needs to be known and programmed the same at both units performing the SS-TWR.
 * It can be considered as an extension of the KEY. The low 32 bits of the IV is the counter.
 * In a real application for any particular key the value of the IV including the count should not be reused,
 * i.e. if the counter value wraps the upper 96-bits of the IV should be changed, e.g. incremented.
 *
 * Here we use a default IV as specified in the IEEE 802.15.4z annex
 */
static dwt_sts_cp_iv_t cp_iv = { 0x1F9A3DE4, 0xD37EC3CA, 0xC44FA8FB, 0x362EEB34 };

/*
 * The 'poll' message initiating the ranging exchange includes a 32-bit counter which is part
 * of the IV used to generate the scrambled timestamp sequence (STS) in the transmitted packet.
 */
static void send_tx_poll_msg(void)
{
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
//    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
//    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
//    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
//    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

    /* Start transmission. */
//    dwt_starttx(DWT_START_TX_IMMEDIATE);
//
//    /* Poll DW IC until TX frame sent event set. See NOTE 8 below. */
//    waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
//
//    /* Clear TXFRS event. */
//    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	switch(frame_seq_nb)
	{
	case 0:
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_poll_msg1), tx_poll_msg1, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg1), 0, 1);          /* Zero offset in TX buffer, ranging. */
        break;
	case 1:
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_poll_msg2), tx_poll_msg2, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg2), 0, 1);          /* Zero offset in TX buffer, ranging. */
        break;
	case 2:
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_poll_msg3), tx_poll_msg3, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg3), 0, 1);          /* Zero offset in TX buffer, ranging. */
        break;
	default:
		frame_seq_nb=0;
	}
    /* Start transmission. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
    /* Poll DW IC until TX frame sent event set. See NOTE 8 below. */
  //  waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

    /* Clear TXFRS event. */
    //dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}
typedef struct Anchor
{
	double x;
	double y;
	double distance;
}Anchor;
void tril_do();
Anchor A1={1,1,0};
Anchor A2={2.6,1,0};
Anchor A3={2.6,1.75,0};


readDW3000CIRData(float *CIRValues, int len, int offset);
float CIRValues[1016]; // 계산된 CIR값 대입

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ss_twr_initiator_sts()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ss_twr_initiator_sts(void)
{
	udp_echoclient_connect();
    int goodSts = 0;           /* Used for checking STS quality in received signal */
    int16_t stsQual;           /* This will contain STS quality index */
    uint16_t stsStatus;        /* Used to check for good STS status (no errors). */
    uint8_t firstLoopFlag = 0; /* Used to track if the program has gone through the first loop or not. */

    /* Display application name on UART. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 36 MHz */
#ifdef CONFIG_SPI_FAST_RATE
    port_set_dw_ic_spi_fastrate();
#endif /* CONFIG_SPI_FAST_RATE */
#ifdef CONFIG_SPI_SLOW_RATE
    port_set_dw_ic_spi_slowrate();
#endif /* CONFIG_SPI_SLOW_RATE */

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 15 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config_options))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    if (config_options.chan == 5)
    {
        dwt_configuretxrf(&txconfig_options);
    }
    else
    {
        dwt_configuretxrf(&txconfig_options_ch9);
    }

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay, this value can be set here once for all. */
    set_resp_rx_timeout(RESP_RX_TIMEOUT_UUS, &config_options);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help diagnostics, and also TX/RX LEDs */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_configciadiag(1);
    udp_echoclient_connect();
    /* Loop for user defined number of ranges. */
    while (1)
    {
        /*
         * Set STS encryption key and IV (nonce).
         * See NOTE 16 below.
         */
        if (!firstLoopFlag)
        {
            /*
             * On first loop, configure the STS key & IV, then load them.
             */
            dwt_configurestskey(&cp_key);
            dwt_configurestsiv(&cp_iv);
            dwt_configurestsloadiv();
            firstLoopFlag=1;
        }
        else
        {
            /*
             * On subsequent loops, we only need to reload the lower 32 bits of STS IV.
             */
            dwt_configurestsiv(&cp_iv);
            dwt_configurestsloadiv();
        }
        /*
         * Send the poll message to the responder.
         */
        send_tx_poll_msg();

        /*
         * Need to check the STS has been received and is good.
         */
        goodSts = dwt_readstsquality(&stsQual);

        if ((status_reg & DWT_INT_RXFCG_BIT_MASK) && (goodSts >= 0) && (dwt_readstsstatus(&stsStatus, 0) == DWT_SUCCESS))
        {
            uint16_t frame_len;

            /* Clear the RX events. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_getframelength();
            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                    int32_t rtd_init, rtd_resp;
                    float clockOffsetRatio;

                    /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                    poll_tx_ts = dwt_readtxtimestamplo32();
                    resp_rx_ts = dwt_readrxtimestamplo32();

                    /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

                    /* Get timestamps embedded in response message. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                    /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                    rtd_init = resp_rx_ts - poll_tx_ts;
                    rtd_resp = resp_tx_ts - poll_rx_ts;

                    tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;

                    /* Display computed distance on LCD. */
                	switch(frame_seq_nb)
                	{
                	case 0:
                		snprintf(dist_str, sizeof(dist_str), "A1: %3.2f m", distance);
                		A1.distance=distance;
                        /* 다음 앵커 순서로 증가 */
                        frame_seq_nb=2;
                        break;
                	case 1:
                		snprintf(dist_str, sizeof(dist_str), "A2: %3.2f m", distance);
                		A2.distance=distance;
                        /* 다음 앵커 순서로 증가 */
                        //frame_seq_nb==2;
                        break;
                	case 2:
                		snprintf(dist_str, sizeof(dist_str), "%3.2fm", distance);
                		A3.distance=distance;

                		//tril_do();
                        /* 다음 앵커 순서로 증가 */
                        frame_seq_nb=2;
                        break;
                	default:
                		frame_seq_nb=0;
                	}
					if(firstLoopFlag==1){
						ethernetif_input(&gnetif);
						sys_check_timeouts();
						udp_echoclient_send(dist_str);
						test_run_info((unsigned char *)dist_str);
						/* Read diagnostics data. */
						dwt_readdiagnostics(&rx_diag);

						/* Read accumulator. See NOTES 2 and 6. */
						uint16_t fp_int = rx_diag.ipatovFpIndex >> 6;
						uint16_t fp_sts_int = rx_diag.stsFpIndex >> 6;
						uint16_t sts_acc_cnt = rx_diag.stsAccumCount;
		                readDW3000CIRData(CIRValues, 1016, 0);
					}
                }
                else
                {
                    errors[BAD_FRAME_ERR_IDX] += 1;
                }

        //
        //        * 2. Accumulator values are complex numbers: one 24-bit integer for real part and one 24-bit value for imaginary part, for each sample. In this
        //        *    example, we chose to read 3 values below the first path index and 3 values above. It must be noted that the first byte read when accessing the
        //        *    accumulator memory is always garbage and must be discarded, that is why the data length to read is increased by one byte here.
        //        * 6. Here we chose to read only a few values around the first path index but it is possible and can be useful to get all accumulator values, using
        //        *    the relevant offset and length parameters. Reading the whole accumulator will require 4064 bytes of memory. First path value gotten from
        //        *    dwt_readdiagnostics is a 10.6 bits fixed point value calculated by the DW IC. By dividing this value by 64, we end up with the integer part of
        //        *    it. This value can be used to access the accumulator samples around the calculated first path index as it is done here.

                //dwt_readaccdata(accum_data, ACCUM_DATA_LEN, (fp_int - 2));
                /// read len CIR values starting at offset
                /// store magnitudes in float array CIRValues
            }
            else
            {
                errors[RTO_ERR_IDX] += 1;
            }
        }
        else
        {
            check_for_status_errors(status_reg, errors);

            if (!(status_reg & DWT_INT_RXFCG_BIT_MASK))
            {
                errors[BAD_FRAME_ERR_IDX] += 1;
            }
            if (goodSts < 0)
            {
                errors[PREAMBLE_COUNT_ERR_IDX] += 1;
            }
            if (stsQual <= 0)
            {
                errors[CP_QUAL_ERR_IDX] += 1;
            }
        }

        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Execute a delay between ranging exchanges. */
        Sleep(RNG_DELAY_MS);
    }
}
float norm (Anchor p) // get the norm of a vector
{
    return pow(pow(p.x,2)+pow(p.y,2),.5);
}
void trilateration(Anchor point1, Anchor point2, Anchor point3)
{
	Anchor resultPose;
	//Norm은 벡터의 길이 혹은 크기를 측정하는 방법
    //unit vector in a direction from point1 to point 2
    double p2p1Distance = pow(pow(point2.x-point1.x,2) + pow(point2.y-   point1.y,2),0.5);
    Anchor ex = {(point2.x-point1.x)/p2p1Distance, (point2.y-point1.y)/p2p1Distance};
    Anchor aux = {point3.x-point1.x,point3.y-point1.y};

    //Norm이 측정한 벡터의 크기는 원점에서 벡터 좌표까지의 거리 혹은 Magnitude
    //signed magnitude of the x component
    double i = ex.x * aux.x + ex.y * aux.y;

    //the unit vector in the y direction.
    Anchor aux2 = { point3.x-point1.x-i*ex.x, point3.y-point1.y-i*ex.y};
    Anchor ey = { aux2.x / norm (aux2), aux2.y / norm (aux2) };

    //the signed magnitude of the y component
    double j = ey.x * aux.x + ey.y * aux.y;

    //coordinates
    double x = (pow(point1.distance,2) - pow(point2.distance,2) + pow(p2p1Distance,2))/ (2 * p2p1Distance);
    double y = (pow(point1.distance,2) - pow(point3.distance,2) + pow(i,2) + pow(j,2))/(2*j) - i*x/j;

    //result coordinates
    double finalX = point1.x+ x*ex.x + y*ey.x;
    double finalY = point1.y+ x*ex.y + y*ey.y;

	sprintf(&tx_poll_msg4[14], "%.3f",finalX);
	sprintf(&tx_poll_msg4[24], "%.3f",finalY);

	test_run_info(&tx_poll_msg4[12]);
	Sleep(100);
	test_run_info(&tx_poll_msg4[22]);

    dwt_writetxdata(sizeof(tx_poll_msg4), tx_poll_msg4, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg4), 0, 0);          /* Zero offset in TX buffer, ranging. */

	dwt_starttx(DWT_START_TX_IMMEDIATE);

}

readDW3000CIRData(float *CIRValues, int len, int offset) {
	// DW3000 is 3 byte I, 3 byte Q (18 bits per value) with a 1 byte blank at the start.
	// DW1000 is 2 byte I, 2 byte Q with a 1 byte blank at the start.
	//	Max of 992 or 1016 points (16/64 prf)

	if ((len + offset)>1016)
		len = 1016-offset;
	if (len <1)
		return -1;

	const int bytesPerValue = 6; // dw3000
	unsigned char CIR[16];

	//uint8_t cir_buffer[1016*bytesPerValue+1]; // worst case
	uint8_t *cir_buffer;
	cir_buffer = (uint8_t *)malloc(1016*bytesPerValue+1 * sizeof(int8_t));
	dwt_readaccdata(cir_buffer, len*bytesPerValue+1, offset);
	int byteAddress = 1;
	for (int i=0;i<len;i++) {
		int32_t iValue = cir_buffer[byteAddress++];
		iValue |= ((int32_t)cir_buffer[byteAddress++]<<8);
		iValue |= ((int32_t)(cir_buffer[byteAddress++] & 0x03)<<16);

		int32_t qValue = cir_buffer[byteAddress++];
		qValue |= ((int32_t)cir_buffer[byteAddress++]<<8);
		qValue |= ((int32_t)(cir_buffer[byteAddress++] & 0x03)<<16);

		if (iValue & 0x020000)  // MSB of 18 bit value is 1
			iValue |= 0xfffc0000;
		if (qValue & 0x020000)  // MSB of 18 bit value is 1
			qValue |= 0xfffc0000;
		*(CIRValues+i) = sqrt((float)(iValue*iValue+qValue*qValue));
		sprintf(udp_msg, "%.4f",*(CIRValues+i));
		test_run_info((unsigned char *)udp_msg);
		//Sleep(2);

		ethernetif_input(&gnetif);
		sys_check_timeouts();
		udp_echoclient_send(udp_msg);
		for(int j=0; j<32; j++){
			udp_msg[j]=0;
		}
	}
	free(cir_buffer);
	return 1;
}

void tril_do()
{
    /********************************************************************************************/
	if((A1.distance>0) && (A2.distance>0) && (A3.distance>0))
	{
		trilateration(A1,A2,A3);
	}
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.  NB:SEE ALSO NOTE 11.
 *
 *    Initiator: |Poll TX| ..... |Resp RX|
 *    Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - time of Poll TX/RX
 *                                   ^|R RMARKER|    - time of Resp TX/RX
 *
 *                       <--TDLY->                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->            - RESP_RX_TIMEOUT_UUS   (length of response frame)
 *                    <----RDLY------>               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder can turn around and reply)
 *
 *
 * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate). For a real production application, each
 *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 700 탎).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 11. The use of the clock offset value to correct the TOF calculation, significantly improves the result of the SS-TWR where the remote
 *     responder unit's clock is a number of PPM offset from the local initiator unit's clock.
 *     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delay is calibrated and set correctly.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. This example uses the 802.15.4z STS with a packet configuration of mode 1 which looks like so:
 *     ---------------------------------------------------
 *     | Ipatov Preamble | SFD | STS | PHR | PHY Payload |
 *     ---------------------------------------------------
 *     There is a possibility that the TX and RX units in this example will go out of sync as their STS IV values may be misaligned. The STS IV value
 *     changes upon each receiving and transmitting event by the chip. While the TX and RX devices in this example start at the same STS IV values, it
 *     is possible that they can go out sync if a signal is not received correctly, devices are out of range, etc. To combat this, the 'poll message'
 *     that the initiator sends to the responder contains a plain-text STS counter value. The responder receives this message and first checks if
 *     the received frame is out of sync with it's own counter. If so, it will use this received counter value to update it's own counter. When out
 *     of sync with each other, the STS will not align correctly - thus we get no secure timestamp values.
 * 14. The receiver is enabled with reference to the timestamp of the previously received signal.
 *     The receiver will start after a defined delay.
 *     This defined delay is currently the same as the delay between the responder's received
 *     timestamp of it's last received frame and the timestamp of the transmitted signal that is
 *     sent in response.
 *     This means that the initiator needs to reduce it's delay by the configured preamble length.
 *     This allows for the receiver to enable on the initiator at the same time as responder is
 *     transmitting it's message. It should look something like this:
 *
 *     Initiator: |Poll TX| ..... |Resp RX|
 *     Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - time of Poll TX/RX
 *                                   ^|R RMARKER|    - time of Resp TX/RX
 *                    <--------->                    - POLL_TX_TO_RESP_RX_DLY_UUS - Preamble Length
 *                    <-------------->               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder can turn around and reply)
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 * 16. This example will set the STS key and IV upon each iteration of the main while loop. While this has the benefit of keeping the STS count in
 *     sync with the responder device (which does the same), it should be noted that this is not a 'secure' implementation as the count is reset upon
 *     each iteration of the loop. An attacker could potentially recognise this pattern if the signal was being monitored. While it serves it's
 *     purpose in this simple example, it should not be utilised in any final solution.
 ****************************************************************************************************************************************************/
