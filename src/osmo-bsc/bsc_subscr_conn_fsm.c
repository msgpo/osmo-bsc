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
#include <osmocom/gsm/gsm0808_utils.h>

#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/a_reset.h>
#include <osmocom/bsc/bsc_api.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/handover_fsm.h>
#include <osmocom/bsc/lchan_fsm.h>
#include <osmocom/bsc/bsc_subscriber.h>
#include <osmocom/bsc/osmo_bsc_sigtran.h>
#include <osmocom/bsc/osmo_bsc_lcls.h>
#include <osmocom/bsc/bsc_subscr_conn_fsm.h>
#include <osmocom/bsc/osmo_bsc.h>
#include <osmocom/bsc/penalty_timers.h>
#include <osmocom/bsc/gsm_04_08_utils.h>
#include <osmocom/bsc/abis_rsl.h>
#include <osmocom/bsc/bsc_rll.h>
#include <osmocom/bsc/mgw_endpoint_fsm.h>
#include <osmocom/bsc/assignment_fsm.h>
#include <osmocom/mgcp_client/mgcp_client_fsm.h>
#include <osmocom/core/byteswap.h>

#define S(x)	(1 << (x))

#define MGCP_MGW_TIMEOUT 4	/* in seconds */
#define MGCP_MGW_TIMEOUT_TIMER_NR 1

#define MGCP_MGW_HO_TIMEOUT 4	/* in seconds */
#define MGCP_MGW_HO_TIMEOUT_TIMER_NR 2

enum gscon_fsm_states {
	ST_INIT,
	/* waiting for CC from MSC */
	ST_WAIT_CC,
	/* active connection */
	ST_ACTIVE,
	ST_ASSIGNMENT,
	ST_HANDOVER,
	/* BSSMAP CLEAR has been received */
	ST_CLEARING,
};

static const struct value_string gscon_fsm_event_names[] = {
	{GSCON_EV_A_CONN_IND, "MT-CONNECT.ind"},
	{GSCON_EV_A_CONN_REQ, "MO-CONNECT.req"},
	{GSCON_EV_A_CONN_CFM, "MO-CONNECT.cfm"},
	{GSCON_EV_A_CLEAR_CMD, "CLEAR_CMD"},
	{GSCON_EV_A_DISC_IND, "DISCONNET.ind"},
	{GSCON_EV_ASSIGNMENT_START, "ASSIGNMENT_START"},
	{GSCON_EV_ASSIGNMENT_END, "ASSIGNMENT_END"},
	{GSCON_EV_HANDOVER_START, "HANDOVER_START"},
	{GSCON_EV_HANDOVER_END, "HANDOVER_END"},
	{GSCON_EV_RSL_CONN_FAIL, "RSL_CONN_FAIL"},
	{GSCON_EV_MO_DTAP, "MO_DTAP"},
	{GSCON_EV_MT_DTAP, "MT_DTAP"},
	{GSCON_EV_TX_SCCP, "TX_SCCP"},
	{GSCON_EV_MGW_MDCX_RESP_MSC, "MGW_MDCX_RESP_MSC"},
	{GSCON_EV_LCLS_FAIL, "LCLS_FAIL"},
	{GSCON_EV_FORGET_LCHAN, "FORGET_LCHAN"},
	{GSCON_EV_FORGET_MGW_ENDPOINT, "FORGET_MGW_ENDPOINT"},
	{}
};

struct state_timeout conn_fsm_timeouts[32] = {
	[ST_WAIT_CC] = { .T = 993210 },
	[ST_CLEARING] = { .T = 999 },
};

/* Transition to a state, using the T timer defined in conn_fsm_timeouts.
 * The actual timeout value is in turn obtained from network->T_defs.
 * Assumes local variable fi exists. */
#define conn_fsm_state_chg(state) \
	fsm_inst_state_chg_T(conn->fi, state, \
			     conn_fsm_timeouts, \
			     conn->network->T_defs, \
			     1)


int gscon_sigtran_send(struct gsm_subscriber_connection *conn, struct msgb *msg)
{
	int rc;

	if (!msg)
		return -ENOMEM;

	/* Make sure that we only attempt to send SCCP messages if we have
	 * a life SCCP connection. Otherwise drop the message. */
	if (conn->fi->state == ST_INIT || conn->fi->state == ST_WAIT_CC) {
		LOGPFSML(conn->fi, LOGL_ERROR, "No active SCCP connection, dropping message\n");
		msgb_free(msg);
		return -ENODEV;
	}

	rc = osmo_bsc_sigtran_send(conn, msg);
	if (rc < 0)
		LOGPFSML(conn->fi, LOGL_ERROR, "Unable to deliver SCCP message\n");
	return rc;
}
	
#define GSCON_DTAP_CACHE_MSGB_CB_LINK_ID 0
#define GSCON_DTAP_CACHE_MSGB_CB_ALLOW_SACCH 1

static void gscon_dtap_cache_add(struct gsm_subscriber_connection *conn, struct msgb *msg,
				 int link_id, bool allow_sacch)
{
	if (conn->gscon_dtap_cache_len >= 23) {
		LOGP(DHO, LOGL_ERROR, "%s: Cannot cache more DTAP messages,"
		     " already reached sane maximum of %u cached messages\n",
		     bsc_subscr_name(conn->bsub), conn->gscon_dtap_cache_len);
		msgb_free(msg);
		return;
	}
	conn->gscon_dtap_cache_len ++;
	LOGP(DHO, LOGL_DEBUG, "%s: Caching DTAP message during ho/ass (%u)\n",
	     bsc_subscr_name(conn->bsub), conn->gscon_dtap_cache_len);
	msg->cb[GSCON_DTAP_CACHE_MSGB_CB_LINK_ID] = (unsigned long)link_id;
	msg->cb[GSCON_DTAP_CACHE_MSGB_CB_ALLOW_SACCH] = allow_sacch ? 1 : 0;
	msgb_enqueue(&conn->gscon_dtap_cache, msg);
}

void gscon_dtap_cache_flush(struct gsm_subscriber_connection *conn, int send)
{
	struct msgb *msg;
	unsigned int flushed_count = 0;

	while ((msg = msgb_dequeue(&conn->gscon_dtap_cache))) {
		conn->gscon_dtap_cache_len --;
		flushed_count ++;
		if (send) {
			int link_id = (int)msg->cb[GSCON_DTAP_CACHE_MSGB_CB_LINK_ID];
			bool allow_sacch = !!msg->cb[GSCON_DTAP_CACHE_MSGB_CB_ALLOW_SACCH];
			LOGP(DHO, LOGL_DEBUG, "%s: Sending cached DTAP message after handover/assignment (%u/%u)\n",
			     bsc_subscr_name(conn->bsub), flushed_count, conn->gscon_dtap_cache_len);
			gsm0808_submit_dtap(conn, msg, link_id, allow_sacch);
		} else
			msgb_free(msg);
	}
}

static void rll_ind_cb(struct gsm_lchan *lchan, uint8_t link_id, void *_data, enum bsc_rllr_ind rllr_ind)
{
	struct msgb *msg = _data;

	/*
	 * There seems to be a small window that the RLL timer can
	 * fire after a lchan_release call and before the S_CHALLOC_FREED
	 * is called. Check if a conn is set before proceeding.
	 */
	if (!lchan->conn)
		return;

	switch (rllr_ind) {
	case BSC_RLLR_IND_EST_CONF:
		rsl_data_request(msg, OBSC_LINKID_CB(msg));
		break;
	case BSC_RLLR_IND_REL_IND:
	case BSC_RLLR_IND_ERR_IND:
	case BSC_RLLR_IND_TIMEOUT:
		bsc_sapi_n_reject(lchan->conn, OBSC_LINKID_CB(msg));
		msgb_free(msg);
		break;
	}
}

/*! \brief process incoming 08.08 DTAP from MSC (send via BTS to MS) */
int gsm0808_submit_dtap(struct gsm_subscriber_connection *conn, struct msgb *msg, uint8_t link_id,
			bool allow_sacch)
{
	uint8_t sapi;

	if (!conn->lchan) {
		LOGP(DMSC, LOGL_ERROR,
		     "%s Called submit dtap without an lchan.\n",
		     bsc_subscr_name(conn->bsub));
		msgb_free(msg);
		return -1;
	}

	sapi = link_id & 0x7;
	msg->lchan = conn->lchan;
	msg->dst = msg->lchan->ts->trx->rsl_link;

	/* If we are on a TCH and need to submit a SMS (on SAPI=3) we need to use the SACH */
	if (allow_sacch && sapi != 0) {
		if (conn->lchan->type == GSM_LCHAN_TCH_F || conn->lchan->type == GSM_LCHAN_TCH_H)
			link_id |= 0x40;
	}

	msg->l3h = msg->data;
	/* is requested SAPI already up? */
	if (conn->lchan->sapis[sapi] == LCHAN_SAPI_UNUSED) {
		/* Establish L2 for additional SAPI */
		OBSC_LINKID_CB(msg) = link_id;
		if (rll_establish(msg->lchan, sapi, rll_ind_cb, msg) != 0) {
			msgb_free(msg);
			bsc_sapi_n_reject(conn, link_id);
			return -1;
		}
		return 0;
	} else {
		/* Directly forward via RLL/RSL to BTS */
		return rsl_data_request(msg, link_id);
	}
}

static void gscon_bssmap_clear(struct gsm_subscriber_connection *conn,
			       enum gsm0808_cause cause)
{
	struct msgb *resp = gsm0808_create_clear_rqst(cause);
	gscon_sigtran_send(conn, resp);
}

/* forward MT DTAP from BSSAP side to RSL side */
static void _submit_dtap(struct gsm_subscriber_connection *conn, struct msgb *msg,
			 bool to_cache)
{
	int rc;
	uint8_t link_id;
	bool allow_sacch;

	OSMO_ASSERT(msg);
	OSMO_ASSERT(conn);
	OSMO_ASSERT(conn->fi);

	link_id = OBSC_LINKID_CB(msg);
	allow_sacch = true;

	if (to_cache) {
		gscon_dtap_cache_add(conn, msg, link_id, allow_sacch);
		return;
	}

	rc = gsm0808_submit_dtap(conn, msg, link_id, allow_sacch);
	if (rc != 0) {
		LOGPFSML(conn->fi, LOGL_ERROR,
			 "Failed to send DTAP to MS, Tx BSSMAP CLEAR REQUEST to MSC\n");
		gscon_bssmap_clear(conn, GSM0808_CAUSE_EQUIPMENT_FAILURE);
		conn_fsm_state_chg(ST_ACTIVE);
	}
}

static void submit_dtap(struct gsm_subscriber_connection *conn, struct msgb *msg)
{
	_submit_dtap(conn, msg, false);
}

static void cache_dtap(struct gsm_subscriber_connection *conn, struct msgb *msg)
{
	_submit_dtap(conn, msg, true);
}

/* forward MO DTAP from RSL side to BSSAP side */
static void forward_dtap(struct gsm_subscriber_connection *conn, struct msgb *msg, struct osmo_fsm_inst *fi)
{
	struct msgb *resp = NULL;

	OSMO_ASSERT(msg);
	OSMO_ASSERT(conn);

	resp = gsm0808_create_dtap(msg, OBSC_LINKID_CB(msg));
	gscon_sigtran_send(conn, resp);
}

void gscon_release_lchans(struct gsm_subscriber_connection *conn, bool do_sacch_deact)
{
	if (conn->ho.fi)
		handover_end(conn, HO_RESULT_CONN_RELEASE);

	assignment_reset(conn);

	lchan_release(conn->lchan, do_sacch_deact, false, 0);
}

static void handle_bssap_n_connect(struct osmo_fsm_inst *fi, struct osmo_scu_prim *scu_prim)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct msgb *msg = scu_prim->oph.msg;
	struct bssmap_header *bs;
	uint8_t bssmap_type;

	msg->l3h = msgb_l2(msg);
	if (!msgb_l3(msg)) {
		LOGPFSML(fi, LOGL_ERROR, "internal error: no l3 in msg\n");
		goto refuse;
	}

	if (msgb_l3len(msg) < sizeof(*bs)) {
		LOGPFSML(fi, LOGL_NOTICE, "message too short for BSSMAP header (%u < %zu)\n",
			 msgb_l3len(msg), sizeof(*bs));
		goto refuse;
	}

	bs = (struct bssmap_header*)msgb_l3(msg);
	if (msgb_l3len(msg) < (bs->length + sizeof(*bs))) {
		LOGPFSML(fi, LOGL_NOTICE,
			 "message too short for length indicated in BSSMAP header (%u < %u)\n",
			 msgb_l3len(msg), bs->length);
		goto refuse;
	}

	switch (bs->type) {
	case BSSAP_MSG_BSS_MANAGEMENT:
		break;
	default:
		LOGPFSML(fi, LOGL_NOTICE,
			 "message type not allowed for N-CONNECT: %s\n", gsm0808_bssap_name(bs->type));
		goto refuse;
	}

	msg->l4h = &msg->l3h[sizeof(*bs)];
	bssmap_type = msg->l4h[0];

	LOGPFSML(fi, LOGL_DEBUG, "Rx N-CONNECT: %s: %s\n", gsm0808_bssap_name(bs->type),
		 gsm0808_bssmap_name(bssmap_type));

	switch (bssmap_type) {
	case BSS_MAP_MSG_HANDOVER_RQST:
		/* Inter-BSC MT Handover Request, another BSS is handovering to us. */
		handover_start_inter_bsc_mt(conn, msg);
		return;
	default:
		break;
	}

	LOGPFSML(fi, LOGL_NOTICE, "No support for N-CONNECT: %s: %s\n",
		 gsm0808_bssap_name(bs->type), gsm0808_bssmap_name(bssmap_type));
refuse:
	osmo_sccp_tx_disconn(conn->sccp.msc->a.sccp_user, scu_prim->u.connect.conn_id,
			     &scu_prim->u.connect.called_addr, 0);
	osmo_fsm_inst_term(fi, OSMO_FSM_TERM_REGULAR, NULL);
}

static void gscon_fsm_init(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct osmo_scu_prim *scu_prim = NULL;
	struct msgb *msg = NULL;
	int rc;
	enum handover_result ho_result;

	switch (event) {
	case GSCON_EV_A_CONN_REQ:
		/* RLL ESTABLISH IND with initial L3 Message */
		msg = data;
		/* FIXME: Extract Mobile ID and update FSM using osmo_fsm_inst_set_id()
		 * i.e. we will probably extract the mobile identity earlier, where the
		 * imsi filter code is. Then we could just use it here.
		 * related: OS#2969 */

		rc = osmo_bsc_sigtran_open_conn(conn, msg);
		if (rc < 0) {
			osmo_fsm_inst_term(fi, OSMO_FSM_TERM_ERROR, NULL);
		} else {
			/* SCCP T(conn est) is 1-2 minutes, way too long. The MS will timeout
			 * using T3210 (20s), T3220 (5s) or T3230 (10s) */
			conn_fsm_state_chg(ST_WAIT_CC);
		}
		gscon_update_id(conn);
		break;
	case GSCON_EV_A_CONN_IND:
		scu_prim = data;
		if (!conn->sccp.msc) {
			LOGPFSML(fi, LOGL_NOTICE, "N-CONNECT.ind from unknown MSC %s\n",
				 osmo_sccp_addr_dump(&scu_prim->u.connect.calling_addr));
			osmo_sccp_tx_disconn(conn->sccp.msc->a.sccp_user, scu_prim->u.connect.conn_id,
					     &scu_prim->u.connect.called_addr, 0);
			osmo_fsm_inst_term(fi, OSMO_FSM_TERM_REGULAR, NULL);
		}
		/* FIXME: Extract optional IMSI and update FSM using osmo_fsm_inst_set_id()
		 * related: OS2969 (same as above) */

		handle_bssap_n_connect(fi, scu_prim);
		gscon_update_id(conn);
		break;
	case GSCON_EV_HANDOVER_END:
		ho_result = HO_RESULT_ERROR;
		if (data)
			ho_result = *(enum handover_result*)data;
		if (ho_result == HO_RESULT_OK) {
			/* In this case the ho struct should still be populated. */
			if (conn->ho.scope & HO_INTER_BSC_MT) {
				/* Done with establishing a conn where we accept another BSC's MS via
				 * inter-BSC handover */

				osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
				gscon_dtap_cache_flush(conn, 1);
				return;
			}
			LOG_HO(conn, LOGL_ERROR,
			       "Conn is in state %s, the only accepted handover kind is inter-BSC MT",
			       osmo_fsm_inst_state_name(conn->fi));
		}
		gscon_bssmap_clear(conn, GSM0808_CAUSE_EQUIPMENT_FAILURE);
		osmo_fsm_inst_state_chg(fi, ST_CLEARING, 60, 999);
		return;
	default:
		OSMO_ASSERT(false);
	}
}

/* We've sent the CONNECTION.req to the SCCP provider and are waiting for CC from MSC */
static void gscon_fsm_wait_cc(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	switch (event) {
	case GSCON_EV_A_CONN_CFM:
		/* MSC has confirmed the connection, we now change into the
		 * active state and wait there for further operations */
		conn_fsm_state_chg(ST_ACTIVE);
		/* if there's user payload, forward it just like EV_MT_DTAP */
		/* FIXME: Question: if there's user payload attached to the CC, forward it like EV_MT_DTAP? */
		break;
	default:
		OSMO_ASSERT(false);
	}
}

static void gscon_fsm_active_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	if (!conn->lchan)
		gscon_bssmap_clear(conn, GSM0808_CAUSE_EQUIPMENT_FAILURE);
}

/* We're on an active subscriber connection, passing DTAP back and forth */
static void gscon_fsm_active(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	struct gsm_bts *bts;

	switch (event) {

	case GSCON_EV_ASSIGNMENT_START:
		bts = conn->lchan? conn->lchan->ts->trx->bts : NULL;

		if (!bts) {
			LOGPFSML(fi, LOGL_ERROR, "Cannot do assignment, no active BTS\n");
			return;
		}

		/* Rely on assignment_fsm timeout */
		osmo_fsm_inst_state_chg(fi, ST_ASSIGNMENT, 0, 0);
		assignment_fsm_start(conn, bts, data);
		return;

	case GSCON_EV_HANDOVER_START:
		rate_ctr_inc(&conn->network->bsc_ctrs->ctr[BSC_CTR_HANDOVER_ATTEMPTED]);
		/* Rely on handover_fsm timeout */
		osmo_fsm_inst_state_chg(fi, ST_HANDOVER, 0, 0);
		break;

	case GSCON_EV_MO_DTAP:
		forward_dtap(conn, (struct msgb *)data, fi);
		break;
	case GSCON_EV_MT_DTAP:
		submit_dtap(conn, (struct msgb *)data);
		break;
	case GSCON_EV_TX_SCCP:
		gscon_sigtran_send(conn, (struct msgb *)data);
		break;
	default:
		OSMO_ASSERT(false);
	}
}

static void gscon_fsm_assignment(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;

	switch (event) {
	case GSCON_EV_ASSIGNMENT_END:
		osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
		gscon_dtap_cache_flush(conn, 1);
		return;

	case GSCON_EV_MO_DTAP:
		forward_dtap(conn, (struct msgb *)data, fi);
		break;
	case GSCON_EV_MT_DTAP:
		/* cache until assignment is done */
		cache_dtap(conn, (struct msgb *)data);
		break;
	case GSCON_EV_TX_SCCP:
		gscon_sigtran_send(conn, (struct msgb *)data);
		break;
	default:
		OSMO_ASSERT(false);
	}
}

static void gscon_fsm_handover(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;

	switch (event) {
	case GSCON_EV_HANDOVER_END:
		osmo_fsm_inst_state_chg(fi, ST_ACTIVE, 0, 0);
		gscon_dtap_cache_flush(conn, 1);
		return;

	case GSCON_EV_MO_DTAP:
		forward_dtap(conn, (struct msgb *)data, fi);
		break;
	case GSCON_EV_MT_DTAP:
		/* cache until handover is done */
		cache_dtap(conn, (struct msgb *)data);
		break;
	case GSCON_EV_TX_SCCP:
		gscon_sigtran_send(conn, (struct msgb *)data);
		break;
	default:
		OSMO_ASSERT(false);
	}
}

static bool same_mgw_info(const struct mgcp_conn_peer *a, const struct mgcp_conn_peer *b)
{
	if (!a || !b)
		return false;
	if (a == b)
		return true;
	if (strcmp(a->addr, b->addr))
		return false;
	if (a->port != b->port)
		return false;
	if (a->call_id != b->call_id)
		return false;
	return true;
}

/* Make sure a conn->user_plane.mgw_endpoint is allocated with the proper mgw endpoint name. */
struct mgw_endpoint *gscon_ensure_mgw_endpoint(struct gsm_subscriber_connection *conn)
{
	struct bsc_msc_data *msc;
	if (!conn || !conn->sccp.msc)
		return NULL;
	if (conn->user_plane.mgw_endpoint)
		return conn->user_plane.mgw_endpoint;

	msc = conn->sccp.msc;

	switch (msc->a.asp_proto) {
	case OSMO_SS7_ASP_PROT_IPA:
		/* derive endpoint name from CIC on A interface side */
		conn->user_plane.mgw_endpoint =
			mgw_endpoint_alloc(conn->fi, GSCON_EV_FORGET_MGW_ENDPOINT,
					   conn->network->mgw.client, conn->fi->id,
					   "%x@mgw",
					   mgcp_port_to_cic(conn->user_plane.rtp_port, msc->rtp_base));

		break;
	default:
		/* use dynamic RTPBRIDGE endpoint allocation in MGW */
		conn->user_plane.mgw_endpoint =
			mgw_endpoint_alloc(conn->fi, GSCON_EV_FORGET_MGW_ENDPOINT,
					   conn->network->mgw.client, conn->fi->id,
					   "rtpbridge/*@mgw");
		break;
	}

	return conn->user_plane.mgw_endpoint;
}

bool gscon_connect_mgw_to_msc(struct gsm_subscriber_connection *conn,
			      const char *addr, uint16_t port,
			      struct osmo_fsm_inst *notify,
			      uint32_t event_success, uint32_t event_failure,
			      void *notify_data,
			      struct mgwep_ci **created_ci)
{
	int rc;
	struct mgwep_ci *ci;
	struct mgcp_conn_peer mgw_info = {
		.port = port,
		.call_id = conn->sccp.conn_id,
	};
	enum mgcp_verb verb;

	if (created_ci)
		*created_ci = NULL;

	rc = osmo_strlcpy(mgw_info.addr, addr, sizeof(mgw_info.addr));
	if (rc <= 0 || rc >= sizeof(mgw_info.addr)) {
		LOGPFSML(conn->fi, LOGL_ERROR, "Failed to compose MGW endpoint address for MGW -> MSC\n");
		return false;
	}

	ci = conn->user_plane.mgw_endpoint_ci_msc;
	if (ci) {
		const struct mgcp_conn_peer *prev_crcx_info = mgwep_ci_get_rtp_info(ci);

		if (!conn->user_plane.mgw_endpoint) {
			LOGPFSML(conn->fi, LOGL_ERROR, "Internal error: conn has a CI but no endoint\n");
			return false;
		}

		if (!prev_crcx_info) {
			LOGPFSML(conn->fi, LOGL_ERROR, "There already is an MGW connection for the MSC side,"
				 " but it seems to be broken. Will not CRCX another one (%s)\n",
				 mgwep_ci_name(ci));
			return false;
		}

		if (same_mgw_info(&mgw_info, prev_crcx_info)) {
			LOGPFSML(conn->fi, LOGL_DEBUG,
				 "MSC side MGW endpoint ci is already configured to %s",
				 mgwep_ci_name(ci));
			return true;
		}

		verb = MGCP_VERB_MDCX;
	} else
		verb = MGCP_VERB_CRCX;

	gscon_ensure_mgw_endpoint(conn);

	if (!conn->user_plane.mgw_endpoint) {
		LOGPFSML(conn->fi, LOGL_ERROR, "Unable to allocate endpoint info\n");
		return false;
	}

	if (!ci) {
		ci = mgw_endpoint_ci_add(conn->user_plane.mgw_endpoint, "to-MSC");
		if (created_ci)
			*created_ci = ci;
		conn->user_plane.mgw_endpoint_ci_msc = ci;
	}
	if (!ci) {
		LOGPFSML(conn->fi, LOGL_ERROR, "Unable to allocate endpoint CI info\n");
		return false;
	}

	mgw_endpoint_ci_request(ci, verb, &mgw_info, notify, event_success, event_failure, notify_data);
	return true;
}

#define EV_TRANSPARENT_SCCP S(GSCON_EV_TX_SCCP) | S(GSCON_EV_MO_DTAP) | S(GSCON_EV_MT_DTAP)

static const struct osmo_fsm_state gscon_fsm_states[] = {
	[ST_INIT] = {
		.name = "INIT",
		.in_event_mask = S(GSCON_EV_A_CONN_REQ) | S(GSCON_EV_A_CONN_IND)
			| S(GSCON_EV_HANDOVER_END),
		.out_state_mask = S(ST_WAIT_CC) | S(ST_ACTIVE) | S(ST_CLEARING),
		.action = gscon_fsm_init,
	 },
	[ST_WAIT_CC] = {
		.name = "WAIT_CC",
		.in_event_mask = S(GSCON_EV_A_CONN_CFM),
		.out_state_mask = S(ST_ACTIVE),
		.action = gscon_fsm_wait_cc,
	},
	[ST_ACTIVE] = {
		.name = "ACTIVE",
		.in_event_mask = EV_TRANSPARENT_SCCP | S(GSCON_EV_ASSIGNMENT_START) |
				 S(GSCON_EV_HANDOVER_START),
		.out_state_mask = S(ST_CLEARING) | S(ST_ASSIGNMENT) |
				  S(ST_HANDOVER),
		.onenter = gscon_fsm_active_onenter,
		.action = gscon_fsm_active,
	},
	[ST_ASSIGNMENT] = {
		.name = "ASSIGNMENT",
		.in_event_mask = EV_TRANSPARENT_SCCP | S(GSCON_EV_ASSIGNMENT_END),
		.out_state_mask = S(ST_ACTIVE) | S(ST_CLEARING),
		.action = gscon_fsm_assignment,
	},
	[ST_HANDOVER] = {
		.name = "HANDOVER",
		.in_event_mask = EV_TRANSPARENT_SCCP | S(GSCON_EV_HANDOVER_END),
		.out_state_mask = S(ST_ACTIVE) | S(ST_CLEARING),
		.action = gscon_fsm_handover,
	},
	[ST_CLEARING] = {
		.name = "CLEARING",
		/* dead end state */
	 },
};

void gscon_change_primary_lchan(struct gsm_subscriber_connection *conn, struct gsm_lchan **new_lchan)
{
	/* On release, do not receive release events that look like the primary lchan is gone. */
	struct gsm_lchan *old_lchan = conn->lchan;
	if (old_lchan)
		old_lchan->conn = NULL;

	conn->lchan = *new_lchan;
	conn->lchan->conn = conn;
	*new_lchan = NULL;

	if (old_lchan)
		lchan_release(old_lchan, false, false, 0);
}

void gscon_lchan_releasing(struct gsm_subscriber_connection *conn, struct gsm_lchan *lchan)
{
	if (!lchan)
		return;
	if (conn->assignment.new_lchan == lchan) {
		if (conn->assignment.fi)
			osmo_fsm_inst_dispatch(conn->assignment.fi, ASSIGNMENT_EV_LCHAN_ERROR, lchan);
		conn->assignment.new_lchan = NULL;
	}
	if (conn->ho.mt.new_lchan == lchan) {
		if (conn->ho.fi)
			osmo_fsm_inst_dispatch(conn->ho.fi, HO_EV_LCHAN_ERROR, lchan);
		conn->ho.mt.new_lchan = NULL;
	}
	if (conn->lchan == lchan)
		conn->lchan = NULL;
	if (!conn->lchan) {
		osmo_fsm_inst_state_chg(conn->fi, ST_CLEARING, 60, 999);
		gscon_bssmap_clear(conn, GSM0808_CAUSE_EQUIPMENT_FAILURE);
	}
}

/* An lchan was deallocated. */
void gscon_forgetx_lchan(struct gsm_subscriber_connection *conn, struct gsm_lchan *lchan)
{
	if (!lchan)
		return;
	if (conn->assignment.new_lchan == lchan)
		conn->assignment.new_lchan = NULL;
	if (conn->ho.mt.new_lchan == lchan)
		conn->ho.mt.new_lchan = NULL;
	if (conn->lchan == lchan)
		conn->lchan = NULL;
	if (!conn->lchan)
		gscon_bssmap_clear(conn, GSM0808_CAUSE_EQUIPMENT_FAILURE);
}

static void gscon_forget_mgw_endpoint(struct gsm_subscriber_connection *conn)
{
	conn->user_plane.mgw_endpoint = NULL;
	conn->user_plane.mgw_endpoint_ci_msc = NULL;
	lchan_forgetx_mgw_endpoint(conn->lchan);
	lchan_forgetx_mgw_endpoint(conn->assignment.new_lchan);
	lchan_forgetx_mgw_endpoint(conn->ho.mt.new_lchan);
}

static void gscon_fsm_allstate(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct gsm_subscriber_connection *conn = fi->priv;

	/* Regular allstate event processing */
	switch (event) {
	case GSCON_EV_A_CLEAR_CMD:
		/* MSC tells us to cleanly shut down */
		osmo_fsm_inst_state_chg(fi, ST_CLEARING, 60, 999);
		gscon_release_lchans(conn, true);
		/* FIXME: Release all terestrial resources in ST_CLEARING */
		/* According to 3GPP 48.008 3.1.9.1. "The BSS need not wait for the radio channel
		 * release to be completed or for the guard timer to expire before returning the
		 * CLEAR COMPLETE message" */

		/* Close MGCP connections */
		mgw_endpoint_clear(conn->user_plane.mgw_endpoint);

		gscon_sigtran_send(conn, gsm0808_create_clear_complete());
		break;
	case GSCON_EV_A_DISC_IND:
		/* MSC or SIGTRAN network has hard-released SCCP connection,
		 * terminate the FSM now. */
		osmo_fsm_inst_term(fi, OSMO_FSM_TERM_REGULAR, data);
		break;
	case GSCON_EV_FORGET_MGW_ENDPOINT:
		gscon_forget_mgw_endpoint(conn);
		break;
	case GSCON_EV_RSL_CONN_FAIL:
		if (conn->lchan) {
			conn->lchan->release_in_error = true;
			conn->lchan->error_cause = data ? *(uint8_t*)data : RSL_ERR_IE_ERROR;
		}
		gscon_bssmap_clear(conn, GSM0808_CAUSE_RADIO_INTERFACE_FAILURE);
		break;
	case GSCON_EV_MGW_MDCX_RESP_MSC:
		LOGPFSML(fi, LOGL_DEBUG, "Rx MDCX of MSC side (LCLS?)\n");
		break;
	case GSCON_EV_LCLS_FAIL:
		break;
	default:
		OSMO_ASSERT(false);
		break;
	}
}

static void gscon_cleanup(struct osmo_fsm_inst *fi, enum osmo_fsm_term_cause cause)
{
	struct gsm_subscriber_connection *conn = fi->priv;

	if (conn->sccp.state != SUBSCR_SCCP_ST_NONE) {
		LOGPFSML(fi, LOGL_DEBUG, "Disconnecting SCCP\n");
		struct bsc_msc_data *msc = conn->sccp.msc;
		/* FIXME: include a proper cause value / error message? */
		osmo_sccp_tx_disconn(msc->a.sccp_user, conn->sccp.conn_id, &msc->a.bsc_addr, 0);
		conn->sccp.state = SUBSCR_SCCP_ST_NONE;
	}

	if (conn->bsub) {
		LOGPFSML(fi, LOGL_DEBUG, "Putting bsc_subscr\n");
		bsc_subscr_put(conn->bsub);
		conn->bsub = NULL;
	}

	llist_del(&conn->entry);
	talloc_free(conn);
}

static void gscon_pre_term(struct osmo_fsm_inst *fi, enum osmo_fsm_term_cause cause)
{
	struct gsm_subscriber_connection *conn = fi->priv;
	
	mgw_endpoint_clear(conn->user_plane.mgw_endpoint);

	if (conn->lcls.fi) {
		/* request termination of LCLS FSM */
		osmo_fsm_inst_term(conn->lcls.fi, cause, NULL);
		conn->lcls.fi = NULL;
	}

	gscon_release_lchans(conn, false);

	/* drop pending messages */
	gscon_dtap_cache_flush(conn, 0);

	penalty_timers_free(&conn->hodec2.penalty_timers);
}

static int gscon_timer_cb(struct osmo_fsm_inst *fi)
{
	struct gsm_subscriber_connection *conn = fi->priv;

	switch (fi->T) {
	case 993210:
		lchan_release(conn->lchan, false, true, RSL_ERR_INTERWORKING);

		/* MSC has not responded/confirmed connection with CC, this
		 * could indicate a bad SCCP connection. We now inform the the
		 * FSM that controls the BSSMAP reset about the event. Maybe
		 * a BSSMAP reset is necessary. */
		a_reset_conn_fail(conn->sccp.msc->a.reset_fsm);

		/* Since we could not reach the MSC, we give up and terminate
		 * the FSM instance now (N-DISCONNET.req is sent in
		 * gscon_cleanup() above) */
		osmo_fsm_inst_term(fi, OSMO_FSM_TERM_REGULAR, NULL);
		break;
	case 999:
		/* The MSC has sent a BSSMAP Clear Command, we acknowledged that, but the conn was never
		 * disconnected. */
		LOGPFSML(fi, LOGL_ERROR, "Long after a BSSMAP Clear Command, the conn is still not"
			 " released. For sanity, discarding this conn now.\n");
		a_reset_conn_fail(conn->sccp.msc->a.reset_fsm);
		osmo_fsm_inst_term(fi, OSMO_FSM_TERM_ERROR, NULL);
		break;
	default:
		LOGPFSML(fi, LOGL_ERROR, "Unknown timer %d expired\n", fi->T);
		OSMO_ASSERT(false);
	}
	return 0;
}

static struct osmo_fsm gscon_fsm = {
	.name = "SUBSCR_CONN",
	.states = gscon_fsm_states,
	.num_states = ARRAY_SIZE(gscon_fsm_states),
	.allstate_event_mask = S(GSCON_EV_A_DISC_IND) | S(GSCON_EV_A_CLEAR_CMD) | S(GSCON_EV_RSL_CONN_FAIL) |
	    S(GSCON_EV_LCLS_FAIL) |
	    S(GSCON_EV_FORGET_LCHAN) |
	    S(GSCON_EV_FORGET_MGW_ENDPOINT),
	.allstate_action = gscon_fsm_allstate,
	.cleanup = gscon_cleanup,
	.pre_term = gscon_pre_term,
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
		osmo_fsm_register(&lcls_fsm);
		g_initialized = true;
	}

	conn = talloc_zero(net, struct gsm_subscriber_connection);
	if (!conn)
		return NULL;

	conn->network = net;
	INIT_LLIST_HEAD(&conn->gscon_dtap_cache);
	/* BTW, penalty timers will be initialized on-demand. */
	conn->sccp.conn_id = -1;

	/* don't allocate from 'conn' context, as gscon_cleanup() will call talloc_free(conn) before
	 * libosmocore will call talloc_free(conn->fi), i.e. avoid use-after-free during cleanup */
	conn->fi = osmo_fsm_inst_alloc(&gscon_fsm, net, conn, LOGL_NOTICE, NULL);
	if (!conn->fi) {
		talloc_free(conn);
		return NULL;
	}

	/* initialize to some magic values that indicate "IE not [yet] received" */
	conn->lcls.config = 0xff;
	conn->lcls.control = 0xff;
	conn->lcls.fi = osmo_fsm_inst_alloc_child(&lcls_fsm, conn->fi, GSCON_EV_LCLS_FAIL);
	if (!conn->lcls.fi) {
		osmo_fsm_inst_term(conn->fi, OSMO_FSM_TERM_ERROR, NULL);
		return NULL;
	}
	conn->lcls.fi->priv = conn;

	llist_add_tail(&conn->entry, &net->subscr_conns);
	return conn;
}

/* Compose an FSM ID, if possible from the current subscriber information */
void gscon_update_id(struct gsm_subscriber_connection *conn)
{
	osmo_fsm_inst_update_id_f(conn->fi, "conn%x%s%s",
				  conn->sccp.conn_id,
				  conn->bsub? "_" : "",
				  conn->bsub? bsc_subscr_id(conn->bsub) : "");
}

#if 0
static void handle_ass_fail(struct gsm_subscriber_connection *conn,
			    struct msgb *msg)
{
	uint8_t *rr_failure;
	struct gsm48_hdr *gh;

	if (conn->ho) {
		struct lchan_signal_data sig;
		struct gsm48_hdr *gh = msgb_l3(msg);

		LOGPLCHAN(msg->lchan, DRR, LOGL_DEBUG, "ASSIGNMENT FAILED cause = %s\n",
			  rr_cause_name(gh->data[0]));

		sig.lchan = msg->lchan;
		sig.mr = NULL;
		osmo_signal_dispatch(SS_LCHAN, S_LCHAN_ASSIGNMENT_FAIL, &sig);
		/* FIXME: release allocated new channel */

		/* send pending messages, if any */
		gscon_dtap_cache_flush(conn, 1);

		return;
	}

	if (conn->lchan != msg->lchan) {
		LOGPLCHAN(msg->lchan, DMSC, LOGL_ERROR,
			  "Assignment failure should occur on primary lchan.\n");
		return;
	}

	/* stop the timer and release it */
	if (conn->secondary_lchan) {
		lchan_release(conn->secondary_lchan, 0, RSL_REL_LOCAL_END);
		conn->secondary_lchan = NULL;
	}

	/* send pending messages, if any */
	gscon_dtap_cache_flush(conn, 1);

	gh = msgb_l3(msg);
	if (msgb_l3len(msg) - sizeof(*gh) != 1) {
		LOGPLCHAN(conn->lchan, DMSC, LOGL_ERROR, "assignment failure unhandled: %zu\n",
			  msgb_l3len(msg) - sizeof(*gh));
		rr_failure = NULL;
	} else {
		rr_failure = &gh->data[0];
	}

	bsc_assign_fail(conn, GSM0808_CAUSE_RADIO_INTERFACE_MESSAGE_FAILURE, rr_failure);
}

static void handle_rr_ho_compl(struct msgb *msg)
{
	struct lchan_signal_data sig;
	struct gsm48_hdr *gh = msgb_l3(msg);

	LOGPLCHAN(msg->lchan, DRR, LOGL_DEBUG,
		  "HANDOVER COMPLETE cause = %s\n", rr_cause_name(gh->data[0]));

	sig.lchan = msg->lchan;
	sig.mr = NULL;
	osmo_signal_dispatch(SS_LCHAN, S_LCHAN_HANDOVER_COMPL, &sig);
	/* FIXME: release old channel */

	/* send pending messages, if any */
	gscon_dtap_cache_flush(msg->lchan->conn, 1);
}

static void handle_rr_ho_fail(struct msgb *msg)
{
	struct lchan_signal_data sig;
	struct gsm48_hdr *gh = msgb_l3(msg);

	/* Log on both RR and HO categories: it is an RR message, but is still quite important when
	 * filtering on HO. */
	LOGPLCHAN(msg->lchan, DRR, LOGL_DEBUG,
		  "HANDOVER FAILED cause = %s\n", rr_cause_name(gh->data[0]));
	LOGPLCHAN(msg->lchan, DHO, LOGL_DEBUG,
		  "HANDOVER FAILED cause = %s\n", rr_cause_name(gh->data[0]));

	sig.lchan = msg->lchan;
	sig.mr = NULL;
	osmo_signal_dispatch(SS_LCHAN, S_LCHAN_HANDOVER_FAIL, &sig);
	/* FIXME: release allocated new channel */

	/* send pending messages, if any */
	gscon_dtap_cache_flush(msg->lchan->conn, 1);
}


static void handle_ass_compl(struct gsm_subscriber_connection *conn,
			     struct msgb *msg)
{
	struct gsm48_hdr *gh = msgb_l3(msg);
	enum gsm48_rr_cause cause;

	/* Expecting gsm48_hdr + cause value */
	if (msgb_l3len(msg) != sizeof(*gh) + 1) {
		LOGPLCHAN(msg->lchan, DRR, LOGL_ERROR,
			  "RR Assignment Complete: length invalid: %u, expected %zu\n",
			  msgb_l3len(msg), sizeof(*gh) + 1);
		return;
	}

	cause = gh->data[0];

	LOGPLCHAN(msg->lchan, DRR, LOGL_DEBUG, "ASSIGNMENT COMPLETE cause = %s\n",
		  rr_cause_name(cause));

	if (conn->ho) {
		struct lchan_signal_data sig = {
			.lchan = msg->lchan,
		};
		osmo_signal_dispatch(SS_LCHAN, S_LCHAN_ASSIGNMENT_COMPL, &sig);
		/* FIXME: release old channel */

		/* send pending messages, if any */
		gscon_dtap_cache_flush(conn, 1);

		return;
	}

	if (conn->secondary_lchan != msg->lchan) {
		LOGPLCHAN(msg->lchan, DRR, LOGL_ERROR,
			  "RR Assignment Complete does not match conn's secondary lchan.\n");
		return;
	}

	lchan_release(conn->lchan, 0, RSL_REL_LOCAL_END);
	conn->lchan = conn->secondary_lchan;
	conn->secondary_lchan = NULL;

	/* send pending messages, if any */
	gscon_dtap_cache_flush(conn, 1);

	if (is_ipaccess_bts(conn_get_bts(conn)) && conn->lchan->tch_mode != GSM48_CMODE_SIGN)
		rsl_ipacc_crcx(conn->lchan);

	bsc_assign_compl(conn, cause);
}
#endif
