/* Handover FSM implementation for intra-BSC and inter-BSC MT (to this BSC) Handover.
 * (For inter-BSC MO, from this BSC, see handover_inter_mo_fsm.c)
 *
 * (C) 2018 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 *
 * Author: Neels Hofmeyr <neels@hofmeyr.de>
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

#include <osmocom/gsm/rsl.h>

#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/bsc_subscriber.h>

#include <osmocom/bsc/handover_fsm.h>
#include <osmocom/bsc/bsc_subscr_conn_fsm.h>
#include <osmocom/bsc/lchan_select.h>
#include <osmocom/bsc/lchan_fsm.h>
#include <osmocom/bsc/gsm_04_08_utils.h>
#include <osmocom/bsc/abis_rsl.h>
#include <osmocom/bsc/bsc_msc_data.h>

#define LOG_FMT_BTS "bts %u lac-ci %u-%u arfcn-bsic %d-%d"
#define LOG_ARGS_BTS(bts) \
		(bts) ? (bts)->nr : 0, \
		(bts) ? (bts)->location_area_code : 0, \
		(bts) ? (bts)->cell_identity : 0, \
		(bts) ? (bts)->c0->arfcn : 0, \
		(bts) ? (bts)->bsic : 0

#define LOG_FMT_LCHAN "%u-%u-%u-%s%s%s-%u"
#define LOG_ARGS_LCHAN(lchan) \
		lchan ? lchan->ts->trx->bts->nr : 0, \
		lchan ? lchan->ts->trx->nr : 0, \
		lchan ? lchan->ts->nr : 0, \
		lchan ? gsm_pchan_name(lchan->ts->pchan_on_init) : "-", \
		(!lchan || lchan->ts->pchan_on_init == lchan->ts->pchan_is)? "" : ":", \
		(!lchan || lchan->ts->pchan_on_init == lchan->ts->pchan_is)? "" \
			: gsm_pchan_name(lchan->ts->pchan_is), \
		lchan ? lchan->nr : 0 \

#define LOG_FMT_HO_SCOPE "(subscr %s) %s"
#define LOG_ARGS_HO_SCOPE(conn) \
	     bsc_subscr_name(conn->bsub), \
	     handover_scope_name(conn->ho.scope)

static uint8_t g_next_ho_ref = 1;

const char *handover_status(struct gsm_subscriber_connection *conn)
{
	static char buf[256];
	struct handover *ho = &conn->ho;

	if (!conn)
		return "";

	if (ho->scope & (HO_INTRA_CELL | HO_INTRA_BSC)) {
		if (ho->mt.new_lchan)
			snprintf(buf, sizeof(buf),
				 "("LOG_FMT_LCHAN") --HO-> (" LOG_FMT_LCHAN ") " LOG_FMT_HO_SCOPE,
				 LOG_ARGS_LCHAN(conn->lchan),
				 LOG_ARGS_LCHAN(ho->mt.new_lchan),
				 LOG_ARGS_HO_SCOPE(conn));
		else if (ho->mt.new_bts)
			snprintf(buf, sizeof(buf),
				 "("LOG_FMT_LCHAN") --HO-> ("LOG_FMT_BTS",%s) " LOG_FMT_HO_SCOPE,
				 LOG_ARGS_LCHAN(conn->lchan),
				 LOG_ARGS_BTS(ho->mt.new_bts),
				 gsm_lchant_name(ho->new_lchan_type),
				 LOG_ARGS_HO_SCOPE(conn));
		else
			snprintf(buf, sizeof(buf),
				 "("LOG_FMT_LCHAN") --HO->(?) " LOG_FMT_HO_SCOPE,
				 LOG_ARGS_LCHAN(conn->lchan),
				 LOG_ARGS_HO_SCOPE(conn));

	} else if (ho->scope & HO_INTER_BSC_MO)
		snprintf(buf, sizeof(buf),
			 "("LOG_FMT_LCHAN") --HO-> (%s) " LOG_FMT_HO_SCOPE,
			 LOG_ARGS_LCHAN(conn->lchan),
			 neighbor_ident_key_name(&ho->target_cell),
			 LOG_ARGS_HO_SCOPE(conn));

	else if (ho->scope & HO_INTER_BSC_MT) {
		if (ho->mt.new_lchan)
			snprintf(buf, sizeof(buf),
				 "(remote:%s) --HO-> (local:%s|"LOG_FMT_LCHAN") " LOG_FMT_HO_SCOPE,
				 gsm0808_cell_id_name(&ho->mt.inter_bsc.cell_id_serving),
				 gsm0808_cell_id_name2(&ho->mt.inter_bsc.cell_id_target),
				 LOG_ARGS_LCHAN(ho->mt.new_lchan),
				 LOG_ARGS_HO_SCOPE(conn));
		else if (ho->mt.new_bts)
			snprintf(buf, sizeof(buf),
				 "(remote:%s) --HO-> (local:%s|"LOG_FMT_BTS",%s) " LOG_FMT_HO_SCOPE,
				 gsm0808_cell_id_name(&ho->mt.inter_bsc.cell_id_serving),
				 gsm0808_cell_id_name2(&ho->mt.inter_bsc.cell_id_target),
				 LOG_ARGS_BTS(ho->mt.new_bts),
				 gsm_lchant_name(ho->new_lchan_type),
				 LOG_ARGS_HO_SCOPE(conn));
		else
			snprintf(buf, sizeof(buf),
				 "(remote:%s) --HO-> (local:%s,%s) " LOG_FMT_HO_SCOPE,
				 gsm0808_cell_id_name(&ho->mt.inter_bsc.cell_id_serving),
				 gsm0808_cell_id_name2(&ho->mt.inter_bsc.cell_id_target),
				 gsm_lchant_name(ho->new_lchan_type),
				 LOG_ARGS_HO_SCOPE(conn));
	} else
		snprintf(buf, sizeof(buf), LOG_FMT_HO_SCOPE, LOG_ARGS_HO_SCOPE(conn));
	return buf;
}

static struct osmo_fsm ho_fsm;

/* From local var fi->priv, define local var conn. */
#define GET_CONN() \
	struct gsm_subscriber_connection *conn = fi->priv; \
	OSMO_ASSERT((fi)->fsm == &ho_fsm && (fi)->priv) \

#define GET_HO() \
	GET_CONN(); \
	struct handover *ho = &conn->ho

/*
ho_st_wait_lchan_active#HO_ST_WAIT_LCHAN_ACTIVE
ho_st_wait_rr_ho_detect#HO_ST_WAIT_RR_HO_DETECT
ho_st_wait_rr_ho_complete#HO_ST_WAIT_RR_HO_COMPLETE
ho_st_wait_lchan_established#HO_ST_WAIT_LCHAN_ESTABLISHED
ho_st_wait_mgw_endpoint_to_msc#HO_ST_WAIT_MGW_ENDPOINT_TO_MSC


:s/ho_st_\(.*\)#\(HO_ST_\(.*\)\)/static void ho_fsm_\1_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)\r{\r\tGET_CONN();\r}\r\rstatic void ho_fsm_\1(struct osmo_fsm_inst *fi, uint32_t event, void *data)\r{\r\tGET_CONN();\r\tswitch (event) {\r\tdefault:\r\t\tOSMO_ASSERT(false);\r\t}\r}\r



:s/ho_st_\(.*\)#\(HO_ST_\(.*\)\)/\t[\2] = {\r\t\t.name = "\3",\r\t\t.onenter = ho_fsm_\1_onenter,\r\t\t.action = ho_fsm_\1,\r\t\t.in_event_mask = 0\r\t\t\t| S(HO_EV_CONN_RELEASING)\r\t\t\t,\r\t\t.out_state_mask = 0\r\t\t\t| S(HO_ST_FAILURE)\r\t\t\t,\r\t},


*/


struct state_timeout ho_fsm_timeouts[32] = {
	[HO_ST_WAIT_LCHAN_ACTIVE] = { .T = 23042 },
	[HO_ST_WAIT_RR_HO_DETECT] = { .T = 23042 },
	[HO_ST_WAIT_RR_HO_COMPLETE] = { .T = 23042 },
	[HO_ST_WAIT_LCHAN_ESTABLISHED] = { .T = 23042 },
	[HO_ST_WAIT_MGW_ENDPOINT_TO_MSC] = { .T = 23042 },
	[MOHO_ST_WAIT_HO_COMMAND] = { .T = 7 },
	[MOHO_ST_WAIT_CLEAR] = { .T = 8 },
};

/* Transition to a state, using the T timer defined in ho_fsm_timeouts.
 * The actual timeout value is in turn obtained from network->T_defs.
 * Assumes local variable fi exists. */
#define ho_fsm_state_chg(state) \
	fsm_inst_state_chg_T(fi, state, \
			     ho_fsm_timeouts, \
			     ((struct gsm_subscriber_connection*)(fi->priv))->network->T_defs, \
			     5)

/* Log failure and transition to HO_ST_FAILURE, which triggers the appropriate actions. */
#define ho_fail(result, fmt, args...) do { \
		LOG_HO(conn, LOGL_ERROR, "Handover failed in state %s, %s: " fmt, \
		       osmo_fsm_inst_state_name(conn->fi), handover_result_name(result), ## args); \
		handover_end(conn, result); \
	} while(0)

#define ho_success() do { \
		LOG_HO(conn, LOGL_DEBUG, "Handover succeeded"); \
		handover_end(conn, HO_RESULT_OK); \
	} while(0)

/* issue handover to a cell identified by ARFCN and BSIC */
void handover_request(struct handover_mo_req *req)
{
	struct gsm_subscriber_connection *conn;
	OSMO_ASSERT(req->old_lchan);

	conn = req->old_lchan->conn;
	OSMO_ASSERT(conn && conn->fi);

	/* To make sure we're allowed to start a handover, go through a gscon event dispatch. */
	osmo_fsm_inst_dispatch(conn->fi, GSCON_EV_HANDOVER_START, req);
}

/* Check that ho has old_lchan and/or new_lchan and conn pointers match.
 * If old_lchan and/or new_lchan are NULL, omit those checks.
 * On error, return false, log an error and call handover_end() with HO_RESULT_ERROR. */
bool handover_is_sane(struct gsm_subscriber_connection *conn, struct gsm_lchan *old_lchan, struct gsm_lchan *new_lchan)
{
	if (!conn->ho.fi) {
		LOG_HO(conn, LOGL_ERROR, "No handover ongoing");
		return false;
	}

	if (old_lchan
	    && (conn != old_lchan->conn || conn->lchan != old_lchan))
		goto insane;
	if (new_lchan
	    && (conn != new_lchan->conn || conn->ho.mt.new_lchan != new_lchan))
		goto insane;
	if (conn->lchan && conn->lchan == conn->ho.mt.new_lchan)
		goto insane;

	return true;
insane:
	LOG_HO(conn, LOGL_ERROR, "Handover state is corrupted");
	handover_end(conn, HO_RESULT_ERROR);
	return false;
}

static void ho_fsm_update_id(struct osmo_fsm_inst *fi, const char *label)
{
	GET_CONN();
	if (conn->fi->id)
		osmo_fsm_inst_update_id_f(fi, "%s_%s", label, conn->fi->id);
	else
		osmo_fsm_inst_update_id_f(fi, "%s_conn_%x", label, conn->sccp.conn_id);
}

static void handover_start_intra_bsc(struct gsm_subscriber_connection *conn, struct gsm_bts *new_bts);
static void handover_start_inter_bsc_mo(struct gsm_subscriber_connection *conn,
					const struct gsm0808_cell_id_list2 *target_cells);

void handover_fsm_alloc(struct gsm_subscriber_connection *conn) {
	static bool g_initialized = false;
	if (!g_initialized) {
		OSMO_ASSERT(osmo_fsm_register(&ho_fsm) == 0);
		g_initialized = true;
	}

	OSMO_ASSERT(conn->fi);
	OSMO_ASSERT(!conn->ho.fi);

	conn->ho.fi = osmo_fsm_inst_alloc_child(&ho_fsm, conn->fi, GSCON_EV_HANDOVER_END);
	OSMO_ASSERT(conn->ho.fi);
	conn->ho.fi->priv = conn;
}


/* Invoked by gscon if a handover was accepted to start now. */
void handover_start(struct handover_mo_req *req)
{

	OSMO_ASSERT(req && req->old_lchan && req->old_lchan->conn);
	struct gsm_subscriber_connection *conn = req->old_lchan->conn;
	struct handover *ho = &conn->ho;
	struct gsm_bts *bts;
	const struct gsm0808_cell_id_list2 *cil;

	handover_fsm_alloc(conn);

	ho->from_hodec_id = req->from_hodec_id;
	ho->new_lchan_type = req->new_lchan_type == GSM_LCHAN_NONE ?
		req->old_lchan->type : req->new_lchan_type;
	ho->target_cell = req->target_nik;

	bts = bts_by_neighbor_ident(conn->network, &req->target_nik);
	if (bts) {
		handover_start_intra_bsc(conn, bts);
		return;
	}

	cil = neighbor_ident_get(conn->network->neighbor_bss_cells, &req->target_nik);
	if (cil) {
		handover_start_inter_bsc_mo(conn, cil);
		return;
	}

	LOG_HO(conn, LOGL_ERROR, "Cannot handover %s: neighbor unknown\n",
	       neighbor_ident_key_name(&req->target_nik));
	handover_end(conn, HO_RESULT_FAIL_NO_CHANNEL);
}

/*! Hand over the specified logical channel to the specified new BTS and possibly change the lchan type.
 * This is the main entry point for the actual handover algorithm, after the decision whether to initiate
 * HO to a specific BTS. To not change the lchan type, pass old_lchan->type. */
static void handover_start_intra_bsc(struct gsm_subscriber_connection *conn,
				     struct gsm_bts *new_bts)
{
	struct handover *ho = &conn->ho;
	struct osmo_fsm_inst *fi = conn->ho.fi;

	OSMO_ASSERT(new_bts);

	*ho = (struct handover){
		.fi = ho->fi,
		.scope = (new_bts == conn->lchan->ts->trx->bts) ? HO_INTRA_CELL : HO_INTRA_BSC,
		.mt = {
			.new_bts = new_bts,
			.ho_ref = g_next_ho_ref++,
			.async = true,
			.new_lchan = lchan_select_by_type(ho->mt.new_bts, ho->new_lchan_type),
		},
	};

	if (!ho->mt.new_lchan) {
		ho_fail(HO_RESULT_FAIL_NO_CHANNEL,
			"No %s lchan available on BTS %u",
			gsm_lchant_name(ho->new_lchan_type), ho->mt.new_bts->nr);
		return;
	}
	LOG_HO(conn, LOGL_DEBUG, "Selected lchan %s", gsm_lchan_name(ho->mt.new_lchan));

	if (ho->scope & HO_INTRA_CELL)
		ho_fsm_update_id(fi, "intraCell");
	else
		ho_fsm_update_id(fi, "intraBSC");

	ho_fsm_state_chg(HO_ST_WAIT_LCHAN_ACTIVE);

	{
		struct lchan_activate_info info = {
			.activ_for = FOR_HANDOVER,
			.for_conn = conn,
			.chan_mode = conn->lchan->tch_mode,
			.requires_voice_stream = conn->lchan->mgw_endpoint_ci_bts ? true : false,
			.old_lchan = conn->lchan,
		};

		lchan_activate(ho->mt.new_lchan, &info);
	}
}

/* 3GPP TS 48.008 § 3.2.1.8 Handover Request */
static bool parse_ho_request(struct gsm_subscriber_connection *conn, const struct msgb *msg,
			     struct handover_mt_req *req)
{
	struct tlv_parsed tp_arr[2];
	struct tlv_parsed *tp = &tp_arr[0];
	struct tlv_parsed *tp2 = &tp_arr[1];
	struct tlv_p_entry *e;

	int payload_length = msg->tail - msg->l4h;
	if (tlv_parse2(tp_arr, 2, gsm0808_att_tlvdef(), msg->l4h + 1, payload_length - 1, 0, 0) <= 0) {
		LOG_HO(conn, LOGL_ERROR, "Failed to parse IEs\n");
		return false;
	}

	if (!(e = TLVP_GET(tp, GSM0808_IE_CHANNEL_TYPE))) {
		LOG_HO(conn, LOGL_ERROR, "Missing Channel Type IE\n");
		return false;
	}
	if (gsm0808_dec_channel_type(&req->ct, e->val, e->len) <= 0) {
		LOG_HO(conn, LOGL_ERROR, "Failed to parse Channel Type IE\n");
		return false;
	}

	if (!(e = TLVP_GET(tp, GSM0808_IE_ENCRYPTION_INFORMATION))) {
		LOG_HO(conn, LOGL_ERROR, "Missing Encryption Information IE\n");
		return false;
	}
	if (gsm0808_dec_encrypt_info(&req->ei, e->val, e->len) <= 0) {
		LOG_HO(conn, LOGL_ERROR, "Failed to parse Encryption Information IE\n");
		return false;
	}

	if ((e = TLVP_GET(tp, GSM0808_IE_CLASSMARK_INFORMATION_TYPE_1))) {
		if (e->len != sizeof(req->classmark.classmark1)) {
			LOG_HO(conn, LOGL_ERROR, "Classmark Information 1 has wrong size\n");
			return false;
		}
		req->classmark.classmark1 = *(struct gsm48_classmark1*)e->val;
		req->classmark.classmark1_set = true;
	} else if ((e = TLVP_GET(tp, GSM0808_IE_CLASSMARK_INFORMATION_T2))) {
		uint8_t len = OSMO_MIN(sizeof(req->classmark.classmark2),
				       e->len);
		if (!len) {
			LOG_HO(conn, LOGL_ERROR, "Classmark Information 2 has zero size\n");
			return false;
		}
		memcpy(&req->classmark.classmark2, e->val, len);
		req->classmark.classmark2_len = len;
	} else {
		LOG_HO(conn, LOGL_ERROR, "Missing IE: either Classmark Information 1 or 2 required\n");
		return false;
	}

	/* The Cell Identifier (Serving) and Cell Identifier (Target) are both 3.2.2.17 and are
	 * identified by the same tag. So get one from tp and the other from tp2. */
	if (!(e = TLVP_GET(tp, GSM0808_IE_CELL_IDENTIFIER))) {
		LOG_HO(conn, LOGL_ERROR, "Missing IE: Cell Identifier (Serving)\n");
		return false;
	}
	if (gsm0808_dec_cell_id(&req->cell_id_serving, e->val, e->len) < 0) {
		LOG_HO(conn, LOGL_ERROR, "Invalid IE: Cell Identifier (Serving)\n");
		return false;
	}

	if (!(e = TLVP_GET(tp2, GSM0808_IE_CELL_IDENTIFIER))) {
		LOG_HO(conn, LOGL_ERROR, "Missing IE: Cell Identifier (Target)\n");
		return false;
	}
	if (gsm0808_dec_cell_id(&req->cell_id_target, e->val, e->len) < 0) {
		LOG_HO(conn, LOGL_ERROR, "Invalid IE: Cell Identifier (Target)\n");
		return false;
	}

	/* A lot of IEs remain ignored... */

	return true;
}

static bool chan_mode_is_tch(enum gsm48_chan_mode mode)
{
	switch (mode) {
	case GSM48_CMODE_SPEECH_V1:
	case GSM48_CMODE_SPEECH_EFR:
	case GSM48_CMODE_SPEECH_AMR:
		return true;
	default:
		return false;
	}
}

void handover_start_inter_bsc_mt(struct gsm_subscriber_connection *conn,
				 struct msgb *ho_request_msg)
{
	struct handover *ho = &conn->ho;
	struct bsc_msc_data *msc = conn->sccp.msc;
	struct handover_mt_req *req;
	int match_idx;
	enum gsm48_chan_mode mode;
	bool full_rate;
	struct osmo_fsm_inst *fi;

	handover_fsm_alloc(conn);

	*ho = (struct handover){
		.fi = ho->fi,
		.from_hodec_id = HODEC_REMOTE,
		.scope = HO_INTER_BSC_MT,
		.mt = {
			.ho_ref = g_next_ho_ref++,
			.async = true,
		},
	};

	fi = ho->fi;

	rate_ctr_inc(&conn->network->bsc_ctrs->ctr[BSC_CTR_INTER_BSC_HO_MT_ATTEMPTED]);

	if (!parse_ho_request(conn, ho_request_msg, &ho->mt.inter_bsc)) {
		ho_fail(HO_RESULT_ERROR, "Invalid Handover Request message from MSC\n");
		return;
	}

	req = &ho->mt.inter_bsc;

	/* Figure out channel type */
	if (bsc_match_codec_pref(&mode,
				 &full_rate,
				 &req->ct,
				 &req->scl,
				 msc->audio_support, msc->audio_length)) {
		ho_fail(HO_RESULT_FAIL_NO_CHANNEL,
			"Could not find an allowed channel codec (%s, speech codec list len = %u)",
			gsm0808_channel_type_name(&req->ct),
			req->scl.len);
		return;
	}

	LOG_HO(conn, LOGL_DEBUG, "Found matching audio type: %s %s for channel_type ="
	       " { ch_indctr=0x%x, ch_rate_type=0x%x, perm_spch=[ %s] }\n",
	       gsm48_chan_mode_name(mode), full_rate? "full-rate" : "half-rate",
	       req->ct.ch_indctr, req->ct.ch_rate_type,
	       osmo_hexdump(req->ct.perm_spch, req->ct.perm_spch_len));

	/* Figure out which cell to handover to. */
	for (match_idx = 0; ; match_idx++) {
		struct gsm_bts *bts;

		bts = gsm_bts_by_cell_id(conn->network, &req->cell_id_target,
					 match_idx);

		/* Did we iterate all matches? */
		if (!bts)
			break;

		ho->mt.new_bts = bts;
		LOG_HO(conn, LOGL_DEBUG, "BTS %u matches cell id %s",
		       bts->nr, gsm0808_cell_id_name(&req->cell_id_target));

		ho->mt.new_lchan = lchan_select_by_chan_mode(bts, mode, full_rate);
		if (!ho->mt.new_lchan) {
			LOG_HO(conn, LOGL_DEBUG, "BTS %u has no matching free channels",
			       bts->nr);
			ho->mt.new_bts = NULL;
			continue;
		}

		/* Found a match. */
		break;
	}

	if (!ho->mt.new_lchan) {
		ho_fail(HO_RESULT_ERROR, "No free/matching lchan found for %s %s %s\n",
		       gsm0808_cell_id_name(&req->cell_id_target),
		       gsm48_chan_mode_name(mode), full_rate ? "full-rate" : "half-rate");
		return;
	}

	/* Just for completeness' sake, maybe some logging uses it? */
	ho->new_lchan_type = ho->mt.new_lchan->type;

	ho_fsm_state_chg(HO_ST_WAIT_LCHAN_ACTIVE);

	{
		struct lchan_activate_info info = {
			.activ_for = FOR_HANDOVER,
			.for_conn = conn,
			.chan_mode = mode,
			.requires_voice_stream = chan_mode_is_tch(mode),
		};

		lchan_activate(ho->mt.new_lchan, &info);
	}
}


#define FUNC_RESULT_COUNTER(name) \
static int result_counter_##name(enum handover_result result) \
{ \
	switch (result) { \
	case HO_RESULT_OK: \
		return BSC_CTR_##name##_COMPLETED; \
	case HO_RESULT_FAIL_NO_CHANNEL: \
		return BSC_CTR_##name##_NO_CHANNEL; \
	case HO_RESULT_FAIL_RR_HO_FAIL: \
		return BSC_CTR_##name##_FAILED; \
	case HO_RESULT_FAIL_TIMEOUT: \
		return BSC_CTR_##name##_TIMEOUT; \
	case HO_RESULT_CONN_RELEASE: \
		return BSC_CTR_##name##_STOPPED; \
	default: \
	case HO_RESULT_ERROR: \
		return BSC_CTR_##name##_ERROR; \
	} \
}

FUNC_RESULT_COUNTER(ASSIGNMENT)
FUNC_RESULT_COUNTER(HANDOVER)
FUNC_RESULT_COUNTER(INTER_BSC_HO_MT)

static int result_counter_INTER_BSC_HO_MO(enum handover_result result) {
	switch (result) {
	case HO_RESULT_OK:
		return BSC_CTR_INTER_BSC_HO_MT_COMPLETED;
	case HO_RESULT_FAIL_TIMEOUT:
		return BSC_CTR_INTER_BSC_HO_MT_TIMEOUT;
	case HO_RESULT_CONN_RELEASE:
		return BSC_CTR_INTER_BSC_HO_MT_STOPPED;
	default:
	case HO_RESULT_ERROR:
		return BSC_CTR_INTER_BSC_HO_MT_ERROR;
	}
}

static int result_counter(enum handover_scope scope, enum handover_result result)
{
	switch (scope) {
	case HO_INTRA_CELL:
		return result_counter_ASSIGNMENT(result);
	default:
		LOGP(DHO, LOGL_ERROR, "invalid enum handover_scope value: %s\n",
		     handover_scope_name(scope));
		/* use "normal" HO_INTRA_BSC counter... */
	case HO_INTRA_BSC:
		return result_counter_HANDOVER(result);
	case HO_INTER_BSC_MO:
		return result_counter_INTER_BSC_HO_MO(result);
	case HO_INTER_BSC_MT:
		return result_counter_INTER_BSC_HO_MT(result);
		return result_counter_HANDOVER(result);
	}
}

static void handover_reset(struct gsm_subscriber_connection *conn)
{
	if (conn->ho.mt.new_lchan)
		/* New lchan was activated but never passed to a conn */
		lchan_release(conn->ho.mt.new_lchan, true, true, RSL_ERR_EQUIPMENT_FAIL);
	conn->ho = (struct handover){
		.fi = conn->ho.fi,
	};
}

/* Notify the handover decision algorithm of failure and clear out any handover state. */
void handover_end(struct gsm_subscriber_connection *conn, enum handover_result result)
{
	struct handover_decision_callbacks *hdc;
	struct handover *ho = &conn->ho;

	/* Sanity -- an error result ensures beyond doubt that we don't handover_mt_use_new_lchan() below
	 * when the handover isn't actually allowed to change this conn. */
	if (result == HO_RESULT_OK && ho->mt.new_lchan) {
		if (!(ho->scope & (HO_INTRA_CELL | HO_INTRA_BSC | HO_INTER_BSC_MT))) {
			LOG_HO(conn, LOGL_ERROR, "Got new lchan, but this is not an MT HO\n");
			result = HO_RESULT_ERROR;
		}
		if (ho->mt.new_lchan->conn != conn) {
			LOG_HO(conn, LOGL_ERROR, "Got new lchan, but it is for another conn\n");
			result = HO_RESULT_ERROR;
		}
	}

	hdc = handover_decision_callbacks_get(ho->from_hodec_id);
	if (hdc && hdc->on_handover_end)
		hdc->on_handover_end(conn, result);

	rate_ctr_inc(&conn->network->bsc_ctrs->ctr[result_counter(ho->scope, result)]);

	if (ho->mt.new_lchan) {
		if (result == HO_RESULT_OK)
			gscon_change_primary_lchan(conn, &conn->ho.mt.new_lchan);
		else {
			/* Release new lchan, it didn't work out */
			lchan_release(ho->mt.new_lchan, false, true, RSL_ERR_EQUIPMENT_FAIL);
			ho->mt.new_lchan = NULL;
		}
	}

	LOG_HO(conn, LOGL_INFO, "Result: %s\n", handover_result_name(result));

	osmo_fsm_inst_dispatch(conn->fi, GSCON_EV_HANDOVER_END, &result);
	handover_reset(conn);
	osmo_fsm_inst_term(conn->ho.fi, OSMO_FSM_TERM_REGULAR, 0);
}


static void ho_fsm_wait_lchan_active(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	GET_CONN();
	switch (event) {

	case HO_EV_LCHAN_ACTIVE:
		ho_fsm_state_chg(HO_ST_WAIT_RR_HO_DETECT);
		return;
		
	case HO_EV_LCHAN_ERROR:
		ho_fail(HO_RESULT_ERROR, "error while activating lchan %s",
			gsm_lchan_name(conn->ho.mt.new_lchan));
		return;

	default:
		OSMO_ASSERT(false);
	}
}

static void ho_fsm_wait_rr_ho_detect_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	int rc;
	GET_CONN();
	struct handover *ho = &conn->ho;

	struct msgb *rr_ho_cmd = gsm48_make_ho_cmd(ho->mt.new_lchan,
						   ho->mt.new_lchan->ms_power,
						   ho->mt.ho_ref);
	if (!rr_ho_cmd) {
		ho_fail(HO_RESULT_ERROR, "Unable to compose RR Handover Command");
		return;
	}


	if (ho->scope & (HO_INTRA_CELL | HO_INTRA_BSC)) {
		/* conn->lchan is the old lchan being handovered from */
		rr_ho_cmd->lchan = conn->lchan;
		rc = gsm48_sendmsg(rr_ho_cmd);
		if (rc)
			ho_fail(HO_RESULT_ERROR, "Unable to Tx RR Handover Command (rc=%d %s)",
				rc, strerror(-rc));
		return;
	}

	if (ho->scope & HO_INTER_BSC_MT) {
		rc = bsc_send_handover_request_ack(conn, rr_ho_cmd);
		if (rc)
			ho_fail(HO_RESULT_ERROR, "Unable to Tx BSSMAP Handover Request Ack (rc=%d %s)",
				rc, strerror(-rc));
		return;
	}

	ho_fail(HO_RESULT_ERROR, "Invalid situation, no target for RR Handover Command");
}

static void ho_fsm_wait_rr_ho_detect(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	GET_CONN();
	switch (event) {

	case HO_EV_RR_HO_DETECT:
		{
			OSMO_ASSERT(data);
			struct handover_rr_detect_data *d = data;
			if (d->access_delay) {
				LOG_HO(conn, LOGL_DEBUG, "RR Handover Detect (Access Delay=%u)",
				       *(d->access_delay));
			} else
				LOG_HO(conn, LOGL_DEBUG, "RR Handover Detect (no Access Delay IE)");
		}

		ho_fsm_state_chg(HO_ST_WAIT_RR_HO_COMPLETE);
		/* The lchan FSM will already start to redirect the RTP stream */
		return;

	case HO_EV_RR_HO_COMPLETE:
		ho_fail(HO_RESULT_ERROR,
			"Received RR Handover Complete, but haven't even seen a Handover Detect yet");
		return;

	case HO_EV_RR_HO_FAIL:
		ho_fail(HO_RESULT_FAIL_RR_HO_FAIL, "Received RR Handover Fail message");
		return;

	default:
		OSMO_ASSERT(false);
	}
}

static void ho_fsm_wait_rr_ho_complete(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	GET_CONN();

	switch (event) {

	case HO_EV_RR_HO_COMPLETE:
		ho_fsm_state_chg(HO_ST_WAIT_LCHAN_ESTABLISHED);
		return;

	case HO_EV_RR_HO_FAIL:
		ho_fail(HO_RESULT_FAIL_RR_HO_FAIL, "Received RR Handover Fail message");
		return;

	default:
		OSMO_ASSERT(false);
	}
}

static void ho_fsm_post_lchan_established(struct osmo_fsm_inst *fi);

static void ho_fsm_wait_lchan_established_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	GET_HO();

	/* The RR Assignment Complete counts as RLL Establish event */
	osmo_fsm_inst_dispatch(ho->mt.new_lchan->fi, LCHAN_EV_RLL_ESTABLISH_IND, 0);

	if (lchan_state_is(ho->mt.new_lchan, LCHAN_ST_ACTIVE))
		ho_fsm_post_lchan_established(fi);
}

static void ho_fsm_wait_lchan_established(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	switch (event) {

	case HO_EV_LCHAN_ESTABLISHED:
		ho_fsm_post_lchan_established(fi);
		break;

	default:
		OSMO_ASSERT(false);
	}
}

static void ho_fsm_post_lchan_established(struct osmo_fsm_inst *fi)
{
	GET_HO();

	if (ho->mt.new_lchan->activate.requires_voice_stream
	    && (ho->scope & HO_INTER_BSC_MT))
		ho_fsm_state_chg(HO_ST_WAIT_MGW_ENDPOINT_TO_MSC);
	else
		ho_success();
}

static void ho_fsm_wait_mgw_endpoint_to_msc_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{

	OSMO_ASSERT(false);
#if 0
	GET_CONN();

	if (!gscon_connect_mgw_to_msc(conn,
				      ho->mt.msc_rtp_addr,
				      ho->mt.msc_rtp_port,
				      fi,
				      HO_EV_MSC_MGW_OK,
				      HO_EV_MSC_MGW_FAIL,
				      NULL,
				      &ho->mt.created_ci_for_msc)) {
		ho_fail(HO_RESULT_ERROR,
			"Unable to connect MGW endpoint to the MSC side");
	}
#endif
}

static void ho_fsm_wait_mgw_endpoint_to_msc(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	GET_CONN();
	switch (event) {

	case HO_EV_MSC_MGW_OK:
		ho_success();
		return;

	case HO_EV_MSC_MGW_FAIL:
		ho_fail(HO_RESULT_ERROR,
			"Unable to connect MGW endpoint to the MSC side");
		return;

	default:
		OSMO_ASSERT(false);
	}
}

static void handover_start_inter_bsc_mo(struct gsm_subscriber_connection *conn,
					const struct gsm0808_cell_id_list2 *target_cells)
{
	int rc;
	struct msgb *msg;
	struct handover *ho = &conn->ho;
	struct osmo_fsm_inst *fi = conn->ho.fi;
	struct gsm0808_handover_required params = {
		.cause = GSM0808_CAUSE_BETTER_CELL,
		.cil = *target_cells,
		.current_channel_type_1_present = true,
		.current_channel_type_1 = gsm0808_current_channel_type_1(conn->lchan->type),
	};

	ho->scope = HO_INTER_BSC_MO;

	ho_fsm_update_id(fi, "interMO");

	switch (conn->lchan->type) {
	case GSM_LCHAN_TCH_F:
	case GSM_LCHAN_TCH_H:
		params.speech_version_used_present = true;
		params.speech_version_used = gsm0808_permitted_speech(conn->lchan->type,
								      conn->lchan->tch_mode);
		if (!params.speech_version_used) {
			ho_fail(HO_RESULT_ERROR,
				"Cannot encode Speech Version (Used) for HANDOVER REQUIRED message\n");
			return;
		}
		break;
	default:
		break;
	}

	msg = gsm0808_create_handover_required(&params);
	if (!msg) {
		ho_fail(HO_RESULT_ERROR, "Cannot compose Handover Required message");
		return;
	}

	rc = gscon_sigtran_send(conn, msg);
	if (rc) {
		ho_fail(HO_RESULT_ERROR, "Cannot send Handover Required message");
		return;
	}

	ho_fsm_state_chg(MOHO_ST_WAIT_HO_COMMAND);
}

static void moho_fsm_wait_ho_command(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	int rc;
	struct moho_rx_bssmap_ho_command *rx;
	GET_CONN();
	switch (event) {

	case MOHO_EV_BSSMAP_HO_COMMAND:
		rx = data;
		if (!rx) {
			ho_fail(HO_RESULT_ERROR,
				"Rx BSSMAP Handover Command: no L3 info passed with event");
			return;
		}

		LOG_HO(conn, LOGL_DEBUG, "Rx BSSMAP Handover Command: forwarding Layer 3 Info: %s",
		       osmo_hexdump(rx->l3_info, rx->l3_info_len));

		rc = rsl_forward_layer3_info(conn->lchan, rx->l3_info, rx->l3_info_len);
		if (rc) {
			ho_fail(HO_RESULT_ERROR,
				"Rx BSSMAP Handover Command: Failed to forward Layer 3 Info (rc=%d %s)",
				rc, strerror(-rc));
			return;
		}

		ho_fsm_state_chg(MOHO_ST_WAIT_CLEAR);
		return;

	default:
		OSMO_ASSERT(false);
	}
}


#define S(x)	(1 << (x))

static const struct osmo_fsm_state ho_fsm_states[] = {
	[HO_ST_NOT_STARTED] = {
		.name = "NOT_STARTED",
		.out_state_mask = 0
			| S(HO_ST_WAIT_LCHAN_ACTIVE)
			| S(MOHO_ST_WAIT_HO_COMMAND)
			,
	},
	[HO_ST_WAIT_LCHAN_ACTIVE] = {
		.name = "WAIT_LCHAN_ACTIVE",
		.action = ho_fsm_wait_lchan_active,
		.in_event_mask = 0
			| S(HO_EV_LCHAN_ACTIVE)
			| S(HO_EV_LCHAN_ERROR)
			,
		.out_state_mask = 0
			| S(HO_ST_WAIT_LCHAN_ACTIVE)
			| S(HO_ST_WAIT_RR_HO_DETECT)
			,
	},
	[HO_ST_WAIT_RR_HO_DETECT] = {
		.name = "WAIT_RR_HO_DETECT",
		.onenter = ho_fsm_wait_rr_ho_detect_onenter,
		.action = ho_fsm_wait_rr_ho_detect,
		.in_event_mask = 0
			| S(HO_EV_RR_HO_DETECT)
			| S(HO_EV_RR_HO_COMPLETE) /* actually as error */
			| S(HO_EV_RR_HO_FAIL)
			,
		.out_state_mask = 0
			| S(HO_ST_WAIT_RR_HO_COMPLETE)
			,
	},
	[HO_ST_WAIT_RR_HO_COMPLETE] = {
		.name = "WAIT_RR_HO_COMPLETE",
		.action = ho_fsm_wait_rr_ho_complete,
		.in_event_mask = 0
			| S(HO_EV_RR_HO_COMPLETE)
			| S(HO_EV_RR_HO_FAIL)
			,
		.out_state_mask = 0
			| S(HO_ST_WAIT_LCHAN_ESTABLISHED)
			,
	},
	[HO_ST_WAIT_LCHAN_ESTABLISHED] = {
		.name = "WAIT_LCHAN_ESTABLISHED",
		.onenter = ho_fsm_wait_lchan_established_onenter,
		.action = ho_fsm_wait_lchan_established,
		.in_event_mask = 0
			| S(HO_EV_LCHAN_ESTABLISHED)
			,
		.out_state_mask = 0
			| S(HO_ST_WAIT_MGW_ENDPOINT_TO_MSC)
			,
	},
	[HO_ST_WAIT_MGW_ENDPOINT_TO_MSC] = {
		.name = "WAIT_MGW_ENDPOINT_TO_MSC",
		.onenter = ho_fsm_wait_mgw_endpoint_to_msc_onenter,
		.action = ho_fsm_wait_mgw_endpoint_to_msc,
		.in_event_mask = 0
			| S(HO_EV_MSC_MGW_OK)
			| S(HO_EV_MSC_MGW_FAIL)
			,
	},

	[MOHO_ST_WAIT_HO_COMMAND] = {
		.name = "inter-BSC WAIT_HO_COMMAND",
		.action = moho_fsm_wait_ho_command,
		.in_event_mask = 0
			| S(MOHO_EV_BSSMAP_HO_COMMAND)
			,
		.out_state_mask = 0
			| S(MOHO_ST_WAIT_CLEAR)
			,
	},
	[MOHO_ST_WAIT_CLEAR] = {
		.name = "inter-BSC WAIT_CLEAR",
	},
};

static const struct value_string ho_fsm_event_names[] = {
	OSMO_VALUE_STRING(HO_EV_LCHAN_ACTIVE),
	OSMO_VALUE_STRING(HO_EV_LCHAN_ESTABLISHED),
	OSMO_VALUE_STRING(HO_EV_LCHAN_ERROR),
	OSMO_VALUE_STRING(HO_EV_RR_HO_DETECT),
	OSMO_VALUE_STRING(HO_EV_RR_HO_COMPLETE),
	OSMO_VALUE_STRING(HO_EV_RR_HO_FAIL),
	OSMO_VALUE_STRING(HO_EV_MSC_MGW_OK),
	OSMO_VALUE_STRING(HO_EV_MSC_MGW_FAIL),
	OSMO_VALUE_STRING(HO_EV_CONN_RELEASING),
	OSMO_VALUE_STRING(MOHO_EV_BSSMAP_HO_COMMAND),
	{}
};

void ho_fsm_allstate_action(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	GET_CONN();
	switch (event) {

	case HO_EV_CONN_RELEASING:
		switch (fi->state) {
		case MOHO_ST_WAIT_CLEAR:
			ho_success();
			return;
		default:
			ho_fail(HO_RESULT_CONN_RELEASE,
				"Connection releasing in the middle of handover");
			return;
		}

	case HO_EV_LCHAN_ERROR:
		switch (fi->state) {
		case MOHO_ST_WAIT_HO_COMMAND:
		case MOHO_ST_WAIT_CLEAR:
			LOG_HO(conn, LOGL_ERROR, "Event not permitted: %s",
			       osmo_fsm_event_name(fi->fsm, event));
			return;

		default:
			ho_fail(HO_RESULT_ERROR, "Error while establishing lchan %s",
				gsm_lchan_name(data));
			return;
		}

	default:
		OSMO_ASSERT(false);
	}
}

int ho_fsm_timer_cb(struct osmo_fsm_inst *fi)
{
	GET_CONN();
	ho_fail(HO_RESULT_FAIL_TIMEOUT, "Timeout");
	return 0;
}

void ho_fsm_cleanup(struct osmo_fsm_inst *fi, enum osmo_fsm_term_cause cause)
{
	GET_CONN();
	conn->ho.fi = NULL;
}

static struct osmo_fsm ho_fsm = {
	.name = "handover",
	.states = ho_fsm_states,
	.num_states = ARRAY_SIZE(ho_fsm_states),
	.log_subsys = DRSL,
	.event_names = ho_fsm_event_names,
	.allstate_action = ho_fsm_allstate_action,
	.allstate_event_mask = 0
		| S(HO_EV_CONN_RELEASING)
		| S(HO_EV_LCHAN_ERROR)
		,
	.timer_cb = ho_fsm_timer_cb,
	.cleanup = ho_fsm_cleanup,
};
