/*! ----------------------------------------------------------------------------
 *  @file    ss_twr_initiator.c
 *  @brief   Single-sided two-way ranging (SS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
 *           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
 *           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
 *           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>

#if defined(TEST_SS_TWR_INITIATOR)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SS TWR INIT v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/*
IEEE 802.15.4-2015 standard를 지킨 MAC프레임 설정
	다음과 같은 필드로 구성되어 있습니다:
    - byte 0/1: 프레임 컨트롤 (0x8863 MAC command frame으로 16-bit addressing 과 ACK를 요청합니다.)
      - bits 0-2: Frame Type: 011 - MAC command frame
      - bit 3: Security Enabled: 0 - No security enabled
      - bit 4: Frame Pending: 0 - No additional data for recipient
      - bit 5: AR: 1 - ACK frame required from recipient device on receipt of data frame
      - bit 6: PAN ID Compression: 1 - PAN IDs are identical, Source PAN ID field shall be omitted from transmitted
        frame
      - bit 7: Reserved: 0
      - bit 8: Sequence Number Suppression: 0 - Sequence number field is present
      - bit 9: IE Present: 0 - No IEs contained in frame
      - bits 10-11: Destination Addressing Mode: 10 - Address field contains short address
      - bits 12-13: Frame Version: 00 - Using IEEE Std 802.15.4-2003 frames
      - bits 14-15: Source Addressing Mode: 10 - Include source address in frame
    - byte 2: sequence number, incremented for each new frame.
    - byte 3/4: PAN ID (0xDECA)
    - byte 5/6: destination address, see NOTE 2 below.
    - byte 7/8: source address, see NOTE 2 below.
    - byte 9: Command ID field
    - byte 10/11: frame check-sum, automatically set by DW IC.
 */
#define SRC_A1   0x4131 /* "A1" Source Addr(상대방의 Addr)*/
#define SRC_A2   0x4132 /* "A2" Source Addr(상대방의 Addr)*/
#define SRC_A3   0x4133 /* "A3" Source Addr(상대방의 Addr)*/
#define SRC_A4   0x4134 /* "A4" Source Addr(네트워크 연결된 앵커)*/

//- byte 0/1: frame control (0x8841 - data frame using 16-bit addressing, 0x8863 - MAC command frame).
static uint8_t tx_poll_msg1[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '1', 'V', 'E', 0xE0, 0, 0 }; // 63이므로 MAC
static uint8_t tx_poll_msg2[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '2', 'V', 'E', 0xE0, 0, 0 };
static uint8_t tx_poll_msg3[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '3', 'V', 'E', 0xE0, 0, 0 }; //11번째인덱스까지, 아래는 12부터 추가
static uint8_t tx_poll_msg4[] = { 0x63, 0x88, 1, 0xCA, 0xDE, 'A', '4', 'V', 'E', 0xE0, 0, 0, 'X', ':', 0, 0, 0, 0, 0, 0, 0, 0,'Y', ':', 0, 0, 0, 0, 0, 0, 0, 0};//20칸 추가
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 41이므로 Data.  x시작 14번, y시작 24번

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 400

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

typedef struct Anchor
{
	double x;
	double y;
	double distance;
}Anchor;


void trilaterate(Anchor A1, Anchor A2, Anchor A3);
void tril_do();
Anchor A1={1,1,0};
Anchor A2={2.6,1,0};
Anchor A3={2.6,1.75,0};
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ss_twr_initiator(void)
{

    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 36 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset and initialize DW chip. */
    reset_DWIC(); /* Target specific drive of RSTn line into DW3000 low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 13 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);


    //int start_time, end_time, result;
    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
    	/*******************앵커에게 문자열 프레임 전송******************************/
    	//start_time=time(NULL);
    	switch(frame_seq_nb)
    	{
    	case 0:
            //dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_MAC_LE2_EN); // 프레임 필터링 기능 사용 (802.15.4 프로토콜, LE2_PEND의 주소가 source addr과 일치할 때)
            //dwt_configure_le_address(SRC_A1, LE2);
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
        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);


        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            uint16_t frame_len;

            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

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
                        frame_seq_nb++;
                        break;
                	case 1:
                		snprintf(dist_str, sizeof(dist_str), "A2: %3.2f m", distance);
                		A2.distance=distance;
                        /* 다음 앵커 순서로 증가 */
                        frame_seq_nb++;
                        break;
                	case 2:
                		snprintf(dist_str, sizeof(dist_str), "A3: %3.2f m", distance);
                		A3.distance=distance;

                		tril_do();
                        /* 다음 앵커 순서로 증가 */
                        frame_seq_nb=0;
                        break;
                	default:
                		frame_seq_nb=0;
                	}
                    test_run_info((unsigned char *)dist_str);
                    //tril_do();
                }
                /*
                end_time=time(NULL);
                result=(double)(end_time-start_time);
                snprintf(dist_str, sizeof(dist_str), "time: %3.2f", result);
                test_run_info((unsigned char *)dist_str);
				*/
            }
        }
        else
        {
            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* Execute a delay between ranging exchanges. */
        Sleep(RNG_DELAY_MS);
    }
}

unsigned char arr1[16] = {'X',':',0,0,0,0,0,0,0,0};
unsigned char arr2[16] = {'Y',':',0,0,0,0,0,0,0,0};

void trilaterate(Anchor A1, Anchor A2, Anchor A3)
{
	double A = 2*(A2.x - A1.x);
	double B = 2*(A2.y - A2.y);
	double C = ((pow(A1.distance, 2.0)) - (pow(A2.distance, 2.0)) - (pow(A1.x, 2.0)) + (pow(A2.x, 2.0)) - (pow(A1.y, 2.0)) + (pow(A2.y, 2.0)));
	double D = 2*(A3.x - A2.x);
	double E = 2*(A3.y = A2.y);
	double F = (pow(A2.distance, 2.0) - pow(A3.distance, 2.0) - pow(A2.x, 2.0) + pow(A3.x, 2.0) - pow(A2.y, 2.0) + pow(A3.y, 2.0));

	if( ((B * D) - (E * A))==0  || ((A * E) - (D * B))==0)
		return;

	double x = ( (F * B) - (E * C)) / ((B * D) - (E * A));
	double y = ( (F * A) - (D * C)) / ((A * E) - (D * B));

	sprintf(&arr1[2], "%f\n",x);
	sprintf(&arr2[2], "%f\n",y);

	test_run_info(arr1);
	Sleep(100);
	test_run_info(arr2);

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

void tril_do()
{
    /********************************************************************************************/
	if((A1.distance>0) && (A2.distance>0) && (A3.distance>0))
	{
		//trilaterate(A1,A2,A3);
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
 *    6.8M data rate used (around 400 탎).
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
 * 13. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/
