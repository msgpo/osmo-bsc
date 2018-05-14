#pragma once
#include <osmocom/core/fsm.h>

enum gscon_fsm_event {
	/* local SCCP stack tells us incoming conn from MSC */
	GSCON_EV_A_CONN_IND,
	/* RSL side requests CONNECT to MSC */
	GSCON_EV_A_CONN_REQ,
	/* MSC confirms the SCCP connection */
	GSCON_EV_A_CONN_CFM,
	/* MSC requests assignment */
	GSCON_EV_A_ASSIGNMENT_CMD,
	/* MSC has sent BSSMAP CLEAR CMD */
	GSCON_EV_A_CLEAR_CMD,
	/* MSC SCCP disconnect indication */
	GSCON_EV_A_DISC_IND,

	/* RR ASSIGNMENT COMPLETE received */
	GSCON_EV_RR_ASS_COMPL,
	/* RR ASSIGNMENT FAIL received */
	GSCON_EV_RR_ASS_FAIL,
	/* RR MODE MODIFY ACK received */
	GSCON_EV_RR_MODE_MODIFY_ACK,

	/* RSL RLL Release Indication */
	GSCON_EV_RLL_REL_IND,
	/* RSL CONNection FAILure Indication */
	GSCON_EV_RSL_CONN_FAIL,

	/* RSL/lchan tells us clearing is complete */
	GSCON_EV_RSL_CLEAR_COMPL,

	/* Mobile-originated DTAP (from MS) */
	GSCON_EV_MO_DTAP,
	/* Mobile-terminated DTAP (from MSC) */
	GSCON_EV_MT_DTAP,

	/* Transmit custom SCCP message */
	GSCON_EV_TX_SCCP,

	/* MGW is indicating failure (BTS) */
	GSCON_EV_MGW_FAIL_BTS,
	/* MGW is indicating failure (MSC) */
	GSCON_EV_MGW_FAIL_MSC,
	/* CRCX response received (BTS) */
	GSCON_EV_MGW_CRCX_RESP_BTS,
	/* MDCX response received (BTS) */
	GSCON_EV_MGW_MDCX_RESP_BTS,
	/* CRCX response received (MSC) */
	GSCON_EV_MGW_CRCX_RESP_MSC,

	/* Internal handover request (intra-BSC handover) */
	GSCON_EV_HO_START,
	/* HO success, or any error like no-channel-available, Chan Activ Nack, 04.08 HANDOVER FAIL,
	 * internal/arbitrary error or connection release, leading to a premature end of a handover. The
	 * precise result is sent as enum handover_result* in the event dispatch data argument. */
	GSCON_EV_HO_END,

	/* RSL has acknowledged activation of the new lchan */
	GSCON_EV_HO_CHAN_ACTIV_ACK,
	/* GSM 08.58 HANDOVER DETECT has been received */
	GSCON_EV_HO_DETECT,
	/* GSM 04.08 HANDOVER COMPLETE has been received on new channel */
	GSCON_EV_HO_COMPL,

	/* Request handover from this to another BSS (inter-BSC handover) */
	GSCON_EV_INTER_BSC_HO_MO_START,
	/* MSC replies to Handover Required with a Handover Command */
	GSCON_EV_A_HO_COMMAND,

	/* Inter-BSC HO MT: Handover Request: MSC asks us to accept a handover from another BSS */
	GSCON_EV_A_HO_REQUEST,

	/* Inter-BSC HO MT: MS replies to Handover Command */
	GSCON_EV_RR_HO_ACCEPT,

};

struct gsm_subscriber_connection;
struct gsm_network;

/* Allocate a subscriber connection and its associated FSM */
struct gsm_subscriber_connection *bsc_subscr_con_allocate(struct gsm_network *net);
