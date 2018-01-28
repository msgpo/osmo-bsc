/* (C) 2017 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <osmocom/core/fsm.h>
#include <osmocom/core/logging.h>
#include <osmocom/gsm/gsm0808.h>
#include <osmocom/sigtran/sccp_sap.h>
//#include <osmocom/sigtran/sccp_helpers.h>

#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/bsc_api.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/handover.h>
#include <osmocom/bsc/chan_alloc.h>
#include <osmocom/bsc/bsc_subscriber.h>
#include <osmocom/bsc/osmo_bsc_sigtran.h>
#include <osmocom/bsc/bsc_subscr_conn_fsm.h>

#define S(x)	(1 << (x))

#define GSM0808_T10_VALUE	6

enum gscon_fsm_states {
	ST_INIT,
	/* waiting for CC from MSC */
	ST_WAIT_CC,
	/* active connection */
	ST_ACTIVE,
	/* during assignment; waiting for ASS_CMPL */
	ST_WAIT_ASS_CMPL,
	/* during assignment; waiting for MODE_MODIFY_ACK */
	ST_WAIT_MODE_MODIFY_ACK,

	/* BSSMAP CLEAR has bene received */
	ST_CLEARING,

/* MT (inbound) handover */
	/* Wait for Handover Access from MS/BTS */
	ST_WAIT_MT_HO_ACC,
	/* Wait for RR Handover Complete from MS/BTS */
	ST_WAIT_MT_HO_COMPL,

/* MO (outbound) handover */
	/* Wait for Handover Command / Handover Required Reject from MSC */
	ST_WAIT_MO_HO_CMD,
	/* Wait for Clear Command from MSC */
	ST_MO_HO_PROCEEDING,
};

static const struct value_string gscon_fsm_event_names[] = {
	{ GSCON_EV_A_CONN_IND,		"MT-CONNECT.ind" },
	{ GSCON_EV_A_CONN_REQ,		"MO-CONNECT.req" },
	{ GSCON_EV_A_CONN_CFM,		"MO-CONNET.cfm" },
	{ GSCON_EV_A_ASSIGNMENT_CMD,	"ASSIGNMENT_CMD" },
	{ GSCON_EV_A_CLEAR_CMD,		"CLEAR_CMD" },
	{ GSCON_EV_A_DISC_IND,		"DISCONNET.ind" },
	{ GSCON_EV_A_HO_REQ,		"HANDOVER_REQUEST" },

	{ GSCON_EV_RR_ASS_COMPL,	"RR_ASSIGN_COMPL" },
	{ GSCON_EV_RR_ASS_FAIL,		"RR_ASSIGN_FAIL" },
	{ GSCON_EV_RR_MODE_MODIFY_ACK,	"RR_MODE_MODIFY_ACK" },
	{ GSCON_EV_RR_HO_ACC,		"RR_HO_ACCESS" },
	{ GSCON_EV_RR_HO_COMPL,		"RR_HO_COMPLETE" },
	{ GSCON_EV_RLL_REL_IND,		"RLL_RELEASE.ind" },
	{ GSCON_EV_RSL_CONN_FAIL,	"RSL_CONN_FAIL.ind" },
	{ GSCON_EV_RSL_CLEAR_COMPL,	"RSL_CLEAR_COMPLETE" },

	{ GSCON_EV_MO_DTAP,		"MO-DTAP" },
	{ GSCON_EV_MT_DTAP,		"MT-DTAP" },
	{ 0, NULL }
};

static void gscon_fsm_init(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct osmo_scu_prim *scu_prim = NULL;
	struct msgb *msg = NULL;
	int rc;

	switch (event) {
	case GSCON_EV_A_CONN_REQ:
		/* RLL ESTABLISH IND with initial L3 Message */
		msg = data;
		/* FIXME: Extract Mobile ID and update FSM using osmo_fsm_inst_set_id() */
		rc = osmo_bsc_sigtran_open_conn(conn, msg);
		if (rc < 0) {
			osmo_fsm_inst_term(fi, OSMO_FSM_TERM_ERROR, NULL);
		} else {
			/* SCCP T(conn est) is 1-2 minutes, way too long. The MS will timeout
			 * using T3210 (20s), T3220 (5s) or T3230 (10s) */
			osmo_fsm_inst_state_chg(fi, ST_WAIT_CC, 20, 993210);
		}
		break;
	case GSCON_EV_A_CONN_IND:
		scu_prim = data;
		if (!conn->sccp.msc) {
			LOGPFSML(fi, LOGL_NOTICE, "N-CONNET.ind from unknown MSC %s",
				 osmo_sccp_addr_dump(&scu_prim->u.connect.calling_addr));
			osmo_sccp_tx_disconn(conn->sccp.msc->a.sccp_user, scu_prim->u.connect.conn_id,
					     &scu_prim->u.connect.called_addr, 0);
			osmo_fsm_inst_term(fi, OSMO_FSM_TERM_REGULAR, NULL);
		}
		/* FIXME: Extract optional IMSI and update FSM using osmo_fsm_inst_set_id() */
		LOGPFSML(fi, LOGL_NOTICE, "No support for MSC-originated SCCP Connections yet\n");
		osmo_sccp_tx_disconn(conn->sccp.msc->a.sccp_user, scu_prim->u.connect.conn_id,
				     &scu_prim->u.connect.called_addr, 0);
		osmo_fsm_inst_term(fi, OSMO_FSM_TERM_REGULAR, NULL);
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

/* We've sent the CONNECTION.req to the SCCP provider and are waiting for CC from MSC */
static void gscon_fsm_wait_cc(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;

	switch (event) {
	case GSCON_EV_A_CONN_CFM:
		/* MSC has confirmed the connection */
		osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
		/* if there's user payload, forward it just like EV_MT_DTAP */
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

/* We're on an active subscriber connection, passing DTAP back and forth */
static void gscon_fsm_active(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	uint32_t *param = NULL;
	struct msgb *msg = NULL;
	struct msgb *resp = NULL;
	int chan_mode, full_rate;
	int rc;

	switch (event) {
	case GSCON_EV_A_ASSIGNMENT_CMD:
		/* MSC requests us to perform assignment. We need to check if current channel is
		 * sufficient. If yes, do MODIFY. If not, do assignment */
		param = data;
		full_rate = *param >> 16;
		chan_mode = *param & 0xffff;
		LOGPFSM(fi, "ASSIGNMENT_CMD(full_rate=%d, chan_mode=%d)\n", full_rate, chan_mode);
#if 1
		rc = gsm0808_assign_req(conn, chan_mode, full_rate);
		osmo_fsm_inst_state_chg(fi, ST_WAIT_ASS_CMPL, GSM0808_T10_VALUE, 10);
#else
		if (chan_compat_with_mode(conn->lchan, chan_mode, full_rate)) {
			rc = gsm48_lchan_modify(conn->lchan, chan_mode);
			osmo_fsm_inst_state_chg(fi, ST_WAIT_MODE_MODIFY_ACK, GSM0808_T10_VALUE, 10);
		} else {
			if (handle_new_assignment(conn, chan_mode, full_rate)) {
				osmo_fsm_inst_state_chg(fi, ST_WAIT_ASS_CMPL, GSM0808_T10_VALUE, 10);
			} else {
				/* FIXME: tx assignment failure */
			}
		}
#endif
		break;
	case GSCON_EV_MO_DTAP:
		/* forward MO DTAP from RSL side to BSSAP side */
		msg = data;
		resp = gsm0808_create_dtap(msg, OBSC_LINKID_CB(msg));
		osmo_bsc_sigtran_send(conn, resp);
		break;
	case GSCON_EV_MT_DTAP:
		/* forward MT DTAP from BSSAP side to RSL side */
		msg = data;
		rc = gsm0808_submit_dtap(conn, msg, OBSC_LINKID_CB(msg), 1);
		break;
	case GSCON_EV_A_HO_REQ:
		/* FIXME: reject any handover requests with HO FAIL until implemented */
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

/* We're waiting for an ASSIGNMENT COMPLETE from MS */
static void gscon_fsm_wait_ass_cmpl(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct gsm_lchan *lchan = conn->lchan;
	struct msgb *resp = NULL;

	switch (event) {
	case GSCON_EV_RR_ASS_COMPL:
		resp = gsm0808_create_assignment_completed(0, lchan_to_chosen_channel(lchan),
			 			lchan->encr.alg_id, chan_mode_to_speech(lchan));
		osmo_bsc_sigtran_send(conn, resp);
		osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
		break;
	case GSCON_EV_RR_ASS_FAIL:
		/* FIXME: Nobody generates this yet? */
		resp = gsm0808_create_assignment_failure(GSM0808_CAUSE_RADIO_INTERFACE_MESSAGE_FAILURE,
							 data);
		osmo_bsc_sigtran_send(conn, resp);
		osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

/* We're waiting for a MODE MODIFY ACK from MS + BTS */
static void gscon_fsm_wait_mode_modify_ack(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct gsm_lchan *lchan = conn->lchan;
	struct msgb *resp;

	switch (event) {
	case GSCON_EV_RR_MODE_MODIFY_ACK:
		/* we assume that not only have we received the RR MODE_MODIFY_ACK, but
		 * actually that also the BTS side of the channel mode has been changed accordingly */
		osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
		resp = gsm0808_create_assignment_completed(0, lchan_to_chosen_channel(lchan),
				 			lchan->encr.alg_id, chan_mode_to_speech(lchan));
		osmo_bsc_sigtran_send(conn, resp);
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

static void gscon_fsm_clearing(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct msgb *resp;

	switch (event) {
	case GSCON_EV_RSL_CLEAR_COMPL:
		/* FIXME: clear any MGCP context */
		resp = gsm0808_create_clear_complete();
		osmo_bsc_sigtran_send(conn, resp);
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

static const struct osmo_fsm_state gscon_fsm_states[] = {
	[ST_INIT] = {
		.in_event_mask = S(GSCON_EV_A_CONN_REQ) |
				 S(GSCON_EV_A_CONN_IND),
		.out_state_mask = S(ST_WAIT_CC),
		.name = "INIT",
		.action = gscon_fsm_init,
	},
	[ST_WAIT_CC] = {
		.in_event_mask = S(GSCON_EV_A_CONN_CFM),
		.out_state_mask = S(ST_ACTIVE),
		.name = "WAIT_CC",
		.action = gscon_fsm_wait_cc,
	},
	[ST_ACTIVE] = {
		.in_event_mask = S(GSCON_EV_A_ASSIGNMENT_CMD) |
				 S(GSCON_EV_A_HO_REQ) |
				 S(GSCON_EV_MO_DTAP) |
				 S(GSCON_EV_MT_DTAP),
		.out_state_mask = S(ST_WAIT_ASS_CMPL) |
				  S(ST_WAIT_MODE_MODIFY_ACK) |
				  S(ST_CLEARING) |
				  S(ST_WAIT_MO_HO_CMD) |
				  S(ST_CLEARING),
		.name = "ACTIVE",
		.action = gscon_fsm_active,
	},
	[ST_WAIT_ASS_CMPL] = {
		.in_event_mask = S(GSCON_EV_RR_ASS_COMPL) |
				 S(GSCON_EV_RR_ASS_FAIL) |
				 S(GSCON_EV_MO_DTAP) |
				 S(GSCON_EV_MT_DTAP),
		.out_state_mask = S(ST_ACTIVE) |
				  S(ST_CLEARING),
		.name = "WAIT_ASS_CMPL",
		.action = gscon_fsm_wait_ass_cmpl,
	},
	[ST_WAIT_MODE_MODIFY_ACK] = {
		.in_event_mask = S(GSCON_EV_RR_MODE_MODIFY_ACK) |
				 S(GSCON_EV_MO_DTAP) |
				 S(GSCON_EV_MT_DTAP),
		.out_state_mask = S(ST_ACTIVE) |
				  S(ST_CLEARING),
		.name = "WAIT_MODE_MODIFY_ACK",
		.action = gscon_fsm_wait_mode_modify_ack,
	},
	[ST_CLEARING] = {
		.in_event_mask = S(GSCON_EV_RSL_CLEAR_COMPL),
		.name = "CLEARING",
		.action = gscon_fsm_clearing,
	},
	/* TODO: external handover */
	[ST_WAIT_MT_HO_ACC] = {
		.name = "WAIT_MT_HO_ACC",
	},
	[ST_WAIT_MT_HO_COMPL] = {
		.name = "WAIT_MT_HO_CMPL",
	},
	[ST_WAIT_MO_HO_CMD] = {
		.name = "WAIT_MO_HO_CMD",
	},
	[ST_MO_HO_PROCEEDING] = {
		.name = "WAIT_MO_HO_PROCEEDING",
	},
};


static void gscon_fsm_allstate(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct msgb *resp = NULL;

	switch (event) {
	case GSCON_EV_A_CLEAR_CMD:
		/* MSC tells us to cleanly shut down */
		osmo_fsm_inst_state_chg(fi, ST_CLEARING, 0, 0);
		gsm0808_clear(conn);
		/* FIXME: Release all terestrial resources in ST_CLEARING */
		/* According to 3GPP 48.008 3.1.9.1. "The BSS need not wait for the radio channel
		 * release to be completed or for the guard timer to expire before returning the
		 * CLEAR COMPLETE message" */
		osmo_fsm_inst_dispatch(conn->fi, GSCON_EV_RSL_CLEAR_COMPL, NULL);
		break;
	case GSCON_EV_A_DISC_IND:
		/* MSC or SIGTRAN network has hard-released SCCP connection */
		/* hard-release the lchan */
		/* hard-release any MGCP state */
		break;
	case GSCON_EV_RLL_REL_IND:
		/* BTS reports that one of the LAPDm data links was released */
		/* send proper clear request to MSC */
		break;
	case GSCON_EV_RSL_CONN_FAIL:
		LOGPFSM(fi, "Tx BSSMAP CLEAR REQUEST to MSC");
		resp = gsm0808_create_clear_rqst(GSM0808_CAUSE_RADIO_INTERFACE_FAILURE);
		osmo_bsc_sigtran_send(conn, resp);
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

void ho_dtap_cache_flush(struct gsm_subscriber_connection *conn, int send);

static void gscon_cleanup(struct osmo_fsm_inst *fi, enum osmo_fsm_term_cause cause)
{
	struct gsm_subscriber_connection *conn = fi->priv;

	/* FIXME: merge with gsm0808_clear() */

	if (conn->ho_lchan) {
		LOGPFSML(fi, LOGL_DEBUG, "Releasing ho_lchan\n");
		bsc_clear_handover(conn, 1);
		conn->ho_lchan = NULL;
	}

	if (conn->secondary_lchan) {
		LOGPFSML(fi, LOGL_DEBUG, "Releasing secondary_lchan\n");
		lchan_release(conn->secondary_lchan, 0, RSL_REL_LOCAL_END);
		conn->secondary_lchan = NULL;
	}
	if (conn->lchan) {
		LOGPFSML(fi, LOGL_DEBUG, "Releasing lchan\n");
		lchan_release(conn->lchan, 0, RSL_REL_LOCAL_END);
		conn->lchan = NULL;
	}

	if (conn->bsub) {
		LOGPFSML(fi, LOGL_DEBUG, "Putting bsc_subscr\n");
		bsc_subscr_put(conn->bsub);
		conn->bsub = NULL;
	}

	if (conn->user_plane.mgcp_ctx) {
		/* FIXME: MGCP side of things */
		//mgcp_clear_complete(conn->user_plane.mgcp_ctx, NULL);
	}

	if (conn->sccp.state != SUBSCR_SCCP_ST_NONE) {
		LOGPFSML(fi, LOGL_DEBUG, "Disconnecting SCCP\n");
		struct bsc_msc_data *msc = conn->sccp.msc;
		/* FIXME: include a proper cause value / error message? */
		osmo_sccp_tx_disconn(msc->a.sccp_user, conn->sccp.conn_id, &msc->a.bsc_addr, 0);
		conn->sccp.state = SUBSCR_SCCP_ST_NONE;
	}

	/* drop pending messages */
	ho_dtap_cache_flush(conn, 0);

	penalty_timers_free(&conn->hodec2.penalty_timers);

	llist_del(&conn->entry);
	talloc_free(conn);
	fi->priv = NULL;
}

static int gscon_timer_cb(struct osmo_fsm_inst *fi)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct bsc_msc_data *msc = conn->sccp.msc;
	struct msgb *resp = NULL;

	switch (fi->T) {
	case 993210:
		/* MSC has not responded/confirmed connection witH CC */
		/* N-DISCONNET.req is sent in gscon_cleanup() above */
		osmo_fsm_inst_term(fi, OSMO_FSM_TERM_REGULAR, NULL);
		break;
	case 10: /* Assignment Failed */
		resp = gsm0808_create_assignment_failure(GSM0808_CAUSE_RADIO_INTERFACE_FAILURE, NULL);
		osmo_bsc_sigtran_send(conn, resp);
		osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
		break;
	default:
		OSMO_ASSERT(0);
	}
	return 0;
}

static struct osmo_fsm gscon_fsm = {
	.name = "SUBSCR_CONN",
	.states = gscon_fsm_states,
	.num_states = ARRAY_SIZE(gscon_fsm_states),
	.allstate_event_mask =  S(GSCON_EV_A_DISC_IND) |
				S(GSCON_EV_A_CLEAR_CMD) |
				S(GSCON_EV_RSL_CONN_FAIL) |
				S(GSCON_EV_RLL_REL_IND),
	.allstate_action = gscon_fsm_allstate,
	.cleanup = gscon_cleanup,
	.timer_cb = gscon_timer_cb,
	.log_subsys = DMSC,
	.event_names = gscon_fsm_event_names,
};


/* Allocate a subscriber connection and its associated FSM */
struct gsm_subscriber_connection *bsc_subscr_con_allocate(struct gsm_network *net)
{
	struct gsm_subscriber_connection *conn;
	static bool g_initialized = false;

	if (!g_initialized) {
		osmo_fsm_register(&gscon_fsm);
		g_initialized = true;
	}

	conn = talloc_zero(net, struct gsm_subscriber_connection);
	if (!conn)
		return NULL;

	conn->network = net;
	INIT_LLIST_HEAD(&conn->ho_dtap_cache);
	/* BTW, penalty timers will be initialized on-demand. */
	conn->sccp.conn_id = -1;

	/* don't allocate from 'conn' context, as gscon_cleanup() will call talloc_free(conn) before
	 * libosmocore will call talloc_free(conn->fi), i.e. avoid use-after-free during cleanup */
	conn->fi = osmo_fsm_inst_alloc(&gscon_fsm, net, conn, LOGL_NOTICE, NULL);
	if (!conn->fi) {
		talloc_free(conn);
		return NULL;
	}

	llist_add_tail(&conn->entry, &net->subscr_conns);
	return conn;
}
