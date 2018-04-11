/* Handover Logic for Inter-BTS (Intra-BSC) Handover.  This does not
 * actually implement the handover algorithm/decision, but executes a
 * handover decision */

/* (C) 2009 by Harald Welte <laforge@gnumonks.org>
 *
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <netinet/in.h>

#include <osmocom/core/msgb.h>
#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/bsc/abis_rsl.h>
#include <osmocom/bsc/chan_alloc.h>
#include <osmocom/bsc/signal.h>
#include <osmocom/core/talloc.h>
#include <osmocom/bsc/bsc_subscriber.h>
#include <osmocom/bsc/gsm_04_08_utils.h>
#include <osmocom/bsc/handover.h>
#include <osmocom/bsc/handover_cfg.h>
#include <osmocom/bsc/bsc_subscr_conn_fsm.h>
#include <osmocom/bsc/neighbor_ident.h>
#include <osmocom/bsc/abis_nm.h>
#include <osmocom/gsm/gsm0808.h>
#include <osmocom/gsm/gsm0808_utils.h>

const struct value_string handover_scope_names[] = {
	{ HO_INTRA_CELL, "Assignment" },
	{ HO_INTRA_BSC, "Handover" },
	{ HO_INTER_BSC_MO, "Inter-BSC-Handover (MO)" },
	{ HO_INTER_BSC_MT, "Inter-BSC-Handover (MT)" },
	{ 0, NULL }
};

const struct value_string handover_result_names[] = {
	{ HO_RESULT_OK, "Complete" },
	{ HO_RESULT_FAIL_NO_CHANNEL, "Failure (no channel could be allocated)" },
	{ HO_RESULT_FAIL_RR_HO_FAIL, "Failure (MS sent RR Handover Failure)" },
	{ HO_RESULT_FAIL_TIMEOUT, "Failure (timeout)" },
	{ HO_RESULT_CONN_RELEASE, "Connection released" },
	{ HO_RESULT_ERROR, "Failure" },
	{ 0, NULL }
};

static LLIST_HEAD(handover_decision_callbacks);

void handover_decision_callbacks_register(struct handover_decision_callbacks *hdc)
{
	llist_add_tail(&hdc->entry, &handover_decision_callbacks);
}

static struct handover_decision_callbacks *handover_decision_callbacks_get(int hodec_id)
{
	struct handover_decision_callbacks *hdc;
	llist_for_each_entry(hdc, &handover_decision_callbacks, entry) {
		if (hdc->hodec_id == hodec_id)
			return hdc;
	}
	return NULL;
}

/*! Hand over the specified logical channel to the specified new BTS and possibly change the lchan type.
 * This is the main entry point for the actual handover algorithm, after the decision whether to initiate
 * HO to a specific BTS. To not change the lchan type, pass old_lchan->type. */
static void handover_start_intra_bsc(struct handover *ho, struct gsm_bts *new_bts)
{
	static uint8_t ho_ref = 0;

	OSMO_ASSERT(new_bts);

	ho->scope = (new_bts == ho->mo.old_lchan->ts->trx->bts) ? HO_INTRA_CELL : HO_INTRA_BSC;

	ho->mt.new_bts = new_bts;
	ho->mt.ho_ref = ho_ref++;
	ho->mt.async = true;

	LOGPHO(ho, LOGL_INFO, "Looking for target lchan\n");

	osmo_fsm_inst_dispatch(ho->conn->fi, GSCON_EV_HO_START, NULL);
	/* The gscon_fsm will next ask us to handover_mt_allocate_lchan(). */
}

static void handover_start_inter_bsc_mo(struct handover *ho,
					const struct gsm0808_cell_id_list2 *target_cells,
					enum gsm_chan_t new_lchan_type)
{
	int rc;
	struct gsm_lchan *old_lchan = ho->mo.old_lchan;
	struct gsm0808_handover_required ho_required_params = {
		.cause = GSM0808_CAUSE_BETTER_CELL,
		.cil = *target_cells,
		.current_channel_type_1_present = true,
		.current_channel_type_1 = gsm0808_current_channel_type_1(old_lchan->type),
	};

	ho->scope = HO_INTER_BSC_MO;

	LOGPHO(ho, LOGL_INFO, "Starting\n");

	if (osmo_fsm_inst_dispatch(ho->conn->fi, GSCON_EV_INTER_BSC_HO_MO_START, NULL)) {
		handover_end(ho, HO_RESULT_ERROR);
		return;
	}

	switch (old_lchan->type) {
	case GSM_LCHAN_TCH_F:
	case GSM_LCHAN_TCH_H:
		ho_required_params.speech_version_used_present = true;
		ho_required_params.speech_version_used = chan_to_perm_speech(old_lchan->type, old_lchan->tch_mode);
		if (!ho_required_params.speech_version_used) {
			LOGPHO(ho, LOGL_ERROR,
			       "Cannot encode Speech Version (Used) for HANDOVER REQUIRED message\n");
			handover_end(ho, HO_RESULT_ERROR);
			return;
		}
		break;
	default:
		break;
	}

	rc = bsc_send_handover_required(ho->mo.old_lchan, &ho_required_params);
	if (rc) {
		LOGPHO(ho, LOGL_ERROR, "Failed to send Handover Required (rc=%d)\n", rc);
		handover_end(ho, HO_RESULT_ERROR);
	}
}

/* issue handover to a cell identified by ARFCN and BSIC */
void handover_start_mo(enum hodec_id from_hodec_id, struct gsm_lchan *lchan,
		       struct neighbor_ident_key *ni, enum gsm_chan_t new_lchan_type)
{
	struct gsm_subscriber_connection *conn;
	struct handover *ho;
	struct gsm_network *net = lchan->ts->trx->bts->network;
	struct gsm_bts *bts;
	const struct gsm0808_cell_id_list2 *cil;

	OSMO_ASSERT(lchan);

	conn = lchan->conn;
	OSMO_ASSERT(conn);

	ho = talloc_zero(conn, struct handover);
	OSMO_ASSERT(ho);

	ho->conn = conn;
	ho->from_hodec_id = from_hodec_id;
	ho->new_lchan_type = new_lchan_type;
	ho->mo.old_lchan = lchan;

	rate_ctr_inc(&net->bsc_ctrs->ctr[BSC_CTR_HANDOVER_ATTEMPTED]);

	/* don't attempt multiple handovers for the same conn at
	 * the same time */
	if (conn->ho) {
		LOGPHO(conn->ho, LOGL_ERROR,
		       "Already in handover, cannot start another handover to %s\n",
		       neighbor_ident_key_name(ni));
		goto error;
	}
	conn->ho = ho;

	bts = bts_by_neighbor_ident(net, ni);
	if (bts) {
		handover_start_intra_bsc(ho, bts);
		return;
	}

	cil = neighbor_ident_get(net->neighbor_bss_cells, ni);
	if (cil) {
		handover_start_inter_bsc_mo(ho, cil, new_lchan_type);
		return;
	}

	LOGP(DHO, LOGL_ERROR, "%s Cannot handover to %s: neighbor unknown\n",
	     gsm_lchan_name(lchan), neighbor_ident_key_name(ni));
error:
	handover_end(ho, HO_RESULT_FAIL_NO_CHANNEL);
}

/* 3GPP TS 48.008 § 3.2.1.8 Handover Request */
static bool parse_ho_request(struct handover *ho, struct msgb *msg)
{
	struct tlv_parsed tp_arr[2];
	struct tlv_parsed *tp = &tp_arr[0];
	struct tlv_parsed *tp2 = &tp_arr[1];
	struct tlv_p_entry *e;

	int payload_length = msg->tail - msg->l4h;
	if (tlv_parse2(tp_arr, 2, gsm0808_att_tlvdef(), msg->l4h + 1, payload_length - 1, 0, 0) <= 0) {
		LOGPHO(ho, LOGL_ERROR, "Failed to parse IEs\n");
		return false;
	}

	if (!(e = TLVP_GET(tp, GSM0808_IE_CHANNEL_TYPE))) {
		LOGPHO(ho, LOGL_ERROR, "Missing Channel Type IE\n");
		return false;
	}
	if (gsm0808_dec_channel_type(&ho->mt.inter_bsc.ct, e->val, e->len) <= 0) {
		LOGPHO(ho, LOGL_ERROR, "Failed to parse Channel Type IE\n");
		return false;
	}

	if (!(e = TLVP_GET(tp, GSM0808_IE_ENCRYPTION_INFORMATION))) {
		LOGPHO(ho, LOGL_ERROR, "Missing Encryption Information IE\n");
		return false;
	}
	if (gsm0808_dec_encrypt_info(&ho->mt.inter_bsc.ei, e->val, e->len) <= 0) {
		LOGPHO(ho, LOGL_ERROR, "Failed to parse Encryption Information IE\n");
		return false;
	}

	if ((e = TLVP_GET(tp, GSM0808_IE_CLASSMARK_INFORMATION_TYPE_1))) {
		if (e->len != sizeof(ho->mt.inter_bsc.classmark.classmark1)) {
			LOGPHO(ho, LOGL_ERROR, "Classmark Information 1 has wrong size\n");
			return false;
		}
		ho->mt.inter_bsc.classmark.classmark1 = *(struct gsm48_classmark1*)e->val;
		ho->mt.inter_bsc.classmark.classmark1_set = true;
	} else if ((e = TLVP_GET(tp, GSM0808_IE_CLASSMARK_INFORMATION_T2))) {
		uint8_t len = OSMO_MIN(sizeof(ho->mt.inter_bsc.classmark.classmark2),
				       e->len);
		if (!len) {
			LOGPHO(ho, LOGL_ERROR, "Classmark Information 2 has zero size\n");
			return false;
		}
		memcpy(&ho->mt.inter_bsc.classmark.classmark2, e->val, len);
		ho->mt.inter_bsc.classmark.classmark2_len = len;
	} else {
		LOGPHO(ho, LOGL_ERROR, "Missing IE: either Classmark Information 1 or 2 required\n");
		return false;
	}

	/* The Cell Identifier (Serving) and Cell Identifier (Target) are both 3.2.2.17 and are
	 * identified by the same tag. So get one from tp and the other from tp2. */
	if (!(e = TLVP_GET(tp, GSM0808_IE_CELL_IDENTIFIER))) {
		LOGPHO(ho, LOGL_ERROR, "Missing IE: Cell Identifier (Serving)\n");
		return false;
	}
	if (gsm0808_dec_cell_id(&ho->mt.inter_bsc.cell_id_serving, e->val, e->len) < 0) {
		LOGPHO(ho, LOGL_ERROR, "Invalid IE: Cell Identifier (Serving)\n");
		return false;
	}

	if (!(e = TLVP_GET(tp2, GSM0808_IE_CELL_IDENTIFIER))) {
		LOGPHO(ho, LOGL_ERROR, "Missing IE: Cell Identifier (Target)\n");
		return false;
	}
	if (gsm0808_dec_cell_id(&ho->mt.inter_bsc.cell_id_target, e->val, e->len) < 0) {
		LOGPHO(ho, LOGL_ERROR, "Invalid IE: Cell Identifier (Target)\n");
		return false;
	}

	/* A lot of IEs remain ignored... */

	return true;
}

static bool bts_matches_lai(struct gsm_bts *bts,
			    struct osmo_location_area_id *lai)
{
	return osmo_plmn_cmp(&lai->plmn, &bts->network->plmn) == 0
		&& lai->lac == bts->location_area_code;
}

static bool bts_matches_cell_id(struct gsm_bts *bts,
				struct gsm0808_cell_id *cell_id)
{
	union gsm0808_cell_id_u *id = &cell_id->id;
	if (!bts || !cell_id)
		return false;

	switch (cell_id->id_discr) {
	case CELL_IDENT_NO_CELL:
		return false;
	case CELL_IDENT_BSS:
		return true;
	case CELL_IDENT_LAC:
		return id->lac == bts->location_area_code;
	case CELL_IDENT_CI:
		return id->ci == bts->cell_identity;
	case CELL_IDENT_LAC_AND_CI:
		return id->lac_and_ci.lac == bts->location_area_code
			&& id->lac_and_ci.ci == bts->cell_identity;
	case CELL_IDENT_LAI_AND_LAC:
		return bts_matches_lai(bts, &id->lai_and_lac);
	case CELL_IDENT_WHOLE_GLOBAL:
		return bts_matches_lai(bts, &id->global.lai)
			&& id->global.cell_identity == bts->cell_identity;
	default:
		return false;
	}
}

static struct gsm_bts *bts_by_cell_id(struct gsm_network *net,
				      struct gsm0808_cell_id *cell_id,
				      int match_idx)
{
	struct gsm_bts *bts;
	int i = 0;
	llist_for_each_entry(bts, &net->bts_list, list) {
		if (!bts_matches_cell_id(bts, cell_id))
			continue;
		if (i < match_idx) {
			/* this is only the i'th match, we're looking for a later one... */
			i++;
			continue;
		}
		return bts;
	}
	return NULL;
}

void handover_parse_inter_bsc_mt(struct gsm_subscriber_connection *conn,
				 struct msgb *ho_request_msg)
{
	struct handover *ho = talloc_zero(conn, struct handover);
	OSMO_ASSERT(ho);

	*ho = (struct handover){
		.conn = conn,
		.scope = HO_INTER_BSC_MT,
	};
	conn->ho = ho;

	rate_ctr_inc(&conn->network->bsc_ctrs->ctr[BSC_CTR_INTER_BSC_HO_MT_ATTEMPTED]);

	if (!parse_ho_request(ho, ho_request_msg)) {
		LOGPHO(ho, LOGL_ERROR, "Invalid Handover Request message from MSC\n");
		goto failed;
	}

	/* Figure out which cell to handover to */
	ho->mt.new_bts = bts_by_cell_id(conn->network, &ho->mt.inter_bsc.cell_id_target, 0);
	if (!ho->mt.new_bts) {
		LOGPHO(ho, LOGL_ERROR, "Target cell not found: %s\n",
		       gsm0808_cell_id_name(&ho->mt.inter_bsc.cell_id_target));
		goto failed;
	}

	/* Figure out channel type */
	{
		int full_rate;
		enum gsm48_chan_mode mode;

		if (bsc_msc_match_codec_pref(&full_rate, &mode, &ho->mt.inter_bsc.ct, NULL, conn->msc)) {
			LOGPHO(ho, LOGL_ERROR, "Could not match channel codec (%s)\n",
			       gsm0808_channel_type_name(&ho->mt.inter_bsc.ct));
			goto failed;
		}


	}

	LOGPHO(ho, LOGL_INFO, "Starting\n");
	return;

failed:
	handover_end(ho, HO_RESULT_ERROR);
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
	case HO_INTRA_BSC:
		return result_counter_HANDOVER(result);
	case HO_INTER_BSC_MO:
		return result_counter_INTER_BSC_HO_MO(result);
	case HO_INTER_BSC_MT:
		return result_counter_INTER_BSC_HO_MT(result);
	default:
		OSMO_ASSERT(false);
	}
}

static void handover_mt_use_new_lchan(struct handover *ho)
{
	ho->conn->lchan = ho->mt.new_lchan;

	/* For Intra-BSC HO, release the old lchan. For Inter-BSC MT HO, the old lchan is at the remote
	 * BSS. (For Inter-BSC MO HO, this BSS has only the old lchan, which is released by a BSSMAP
	 * Clear, and this code isn't ever reached.) */
	if (ho->mo.old_lchan) {
		ho->mo.old_lchan->conn = NULL;
		lchan_release(ho->mo.old_lchan, 0, RSL_REL_LOCAL_END);
	}
}

/* Notify the handover decision algorithm of failure and clear out any handover state. */
void handover_end(struct handover *ho, enum handover_result result)
{
	struct handover_decision_callbacks *hdc;
	struct gsm_subscriber_connection *conn = ho->conn;

	/* Sanity -- an error result ensures beyond doubt that we don't handover_mt_use_new_lchan() below
	 * when the handover isn't actually allowed to change this conn. */
	if (result == HO_RESULT_OK && ho->mt.new_lchan) {
		if (!(ho->scope & (HO_INTRA_CELL | HO_INTRA_BSC | HO_INTER_BSC_MT))) {
			LOGPHO(ho, LOGL_ERROR, "Got new lchan, but this is not an MT HO\n");
			result = HO_RESULT_ERROR;
		}
		if (ho->mt.new_lchan->conn != ho->conn) {
			LOGPHO(ho, LOGL_ERROR, "Got new lchan, but it is for another conn\n");
			result = HO_RESULT_ERROR;
		}
	}
	if (ho->mo.old_lchan && ho->mo.old_lchan->conn != ho->conn) {
		LOGPHO(ho, LOGL_ERROR, "The originating lchan does not belong to this conn\n");
		result = HO_RESULT_ERROR;
	}

	hdc = handover_decision_callbacks_get(ho->from_hodec_id);
	if (hdc && hdc->on_handover_end)
		hdc->on_handover_end(ho, result);

	rate_ctr_inc(&conn->network->bsc_ctrs->ctr[result_counter(ho->scope, result)]);

	if (ho->mt.new_lchan) {
		if (result == HO_RESULT_OK)
			handover_mt_use_new_lchan(ho);
		else {
			/* Release new lchan, it didn't work out */
			ho->mt.new_lchan->conn = NULL;
			lchan_release(ho->mt.new_lchan, 0, RSL_REL_LOCAL_END);
		}
	}

	LOGPHO(ho, LOGL_INFO, "Result: %s\n", handover_result_name(result));

	/* Clear conn->ho, only if it points at this ho; if it mismatches, a handover was attempted on an
	 * already active handover (internal error). Dispatch events to the GSCON FSM only if this is the
	 * properly registered handover operation for the conn. */
	if (ho->conn->ho == ho) {
		if (conn->fi)
			osmo_fsm_inst_dispatch(conn->fi, GSCON_EV_HO_END, &result);
		conn->ho = NULL;
	}

	talloc_free(ho);
}

static void ho_meas_rep(struct gsm_meas_rep *mr)
{
	struct handover_decision_callbacks *hdc;
	enum hodec_id hodec_id = ho_get_algorithm(mr->lchan->ts->trx->bts->ho);

	hdc = handover_decision_callbacks_get(hodec_id);
	if (!hdc || !hdc->on_measurement_report)
		return;
	hdc->on_measurement_report(mr);
}

/* Check that ho has old_lchan and/or new_lchan and conn pointers match.
 * If old_lchan and/or new_lchan are NULL, omit those checks.
 * On error, return false, log an error and call handover_end() with HO_RESULT_ERROR. */
bool handover_is_sane(struct handover *ho, struct gsm_lchan *old_lchan, struct gsm_lchan *new_lchan)
{
	if (!ho) {
		struct gsm_lchan *lchan = old_lchan ? old_lchan : new_lchan;
		LOGP(DHO, LOGL_ERROR, "%s Handover state is missing\n", gsm_lchan_name(lchan));
		return false;
	}

	if ((!ho->conn)
	    || (old_lchan
	        && (ho->conn != old_lchan->conn
		    || ho->mo.old_lchan != old_lchan))
	    || (new_lchan
		&& (ho->conn != new_lchan->conn
		    || ho->mt.new_lchan != new_lchan))
	    || (ho->mo.old_lchan
		&& ho->mo.old_lchan == ho->mt.new_lchan)) {

		LOGPHO(ho, LOGL_ERROR, "Handover state is corrupted\n");
		handover_end(ho, HO_RESULT_ERROR);
		return false;
	}
	return true;
}

/* Count ongoing handovers within the given BTS.
 * ho_scopes is an OR'd combination of enum handover_scope values include in the count. */
int handover_count(struct gsm_bts *bts, int ho_scopes)
{
	struct gsm_bts_trx *trx;
	struct handover *ho;
	int count = 0;

	llist_for_each_entry(trx, &bts->trx_list, list) {
		int i;
		for (i = 0; i < ARRAY_SIZE(trx->ts); i++) {
			struct gsm_bts_trx_ts *ts = &trx->ts[i];
			int j;
			int subslots;

			/* skip administratively deactivated timeslots */
			if (!nm_is_running(&ts->mo.nm_state))
				continue;

			subslots = ts_subslots(ts);
			for (j = 0; j < subslots; j++) {
				struct gsm_lchan *lchan = &ts->lchan[j];
				if (!lchan->conn)
					continue;
				ho = lchan->conn->ho;
				if (!ho)
					continue;
				if (ho->scope & ho_scopes)
					count++;
			}
		}
	}

	return count;
}

struct gsm_bts *bts_by_neighbor_ident(const struct gsm_network *net,
				      const struct neighbor_ident_key *search_for)
{
	struct gsm_bts *found = NULL;
	struct gsm_bts *bts;
	struct gsm_bts *wildcard_match = NULL;

	llist_for_each_entry(bts, &net->bts_list, list) {
		struct neighbor_ident_key entry = {
			.arfcn = bts->c0->arfcn,
			.bsic_kind = BSIC_6BIT,
			.bsic = bts->bsic,
		};
		if (neighbor_ident_key_match(&entry, search_for, true)) {
			if (found) {
				LOGP(DHO, LOGL_ERROR, "CONFIG ERROR: Multiple BTS match %s: %d and %d\n",
				     neighbor_ident_key_name(search_for),
				     found->nr, bts->nr);
				return found;
			}
			found = bts;
		}
		if (neighbor_ident_key_match(&entry, search_for, false))
			wildcard_match = bts;
	}

	if (found)
		return found;

	return wildcard_match;
}

struct neighbor_ident_key *bts_ident_key(const struct gsm_bts *bts)
{
	static struct neighbor_ident_key key;
	key = (struct neighbor_ident_key){
		.arfcn = bts->c0->arfcn,
		.bsic_kind = BSIC_6BIT,
		.bsic = bts->bsic,
	};
	return &key;
}

void handover_mt_allocate_lchan(struct handover *ho)
{
	int rc;
	struct gsm_subscriber_connection *conn = ho->conn;
	struct gsm_lchan *old_lchan;
	struct gsm_lchan *new_lchan;
	struct gsm_bts *new_bts;

	if (!handover_is_sane(ho, NULL, NULL))
		return;

	new_bts = ho->mt.new_bts;
	OSMO_ASSERT(new_bts);

	new_lchan = lchan_alloc(new_bts, ho->new_lchan_type, 0);
	if (!new_lchan) {
		LOGPHO(ho, LOGL_NOTICE, "No free channel for %s\n", gsm_lchant_name(ho->new_lchan_type));
		handover_end(ho, HO_RESULT_FAIL_NO_CHANNEL);
		return;
	}

	new_lchan->conn = conn;
	ho->mt.new_lchan = new_lchan;

	LOGPHO(ho, LOGL_INFO, "Starting\n");

	/* copy some parameters from old lchan */
	old_lchan = ho->mo.old_lchan;
	if (old_lchan) {
		memcpy(&new_lchan->encr, &old_lchan->encr, sizeof(new_lchan->encr));
		if (ho->scope == HO_INTRA_CELL) {
			new_lchan->ms_power = old_lchan->ms_power;
			new_lchan->rqd_ta = old_lchan->rqd_ta;
		} else {
			new_lchan->ms_power =
				ms_pwr_ctl_lvl(new_bts->band, new_bts->ms_max_power);
			/* FIXME: do we have a better idea of the timing advance? */
			//new_lchan->rqd_ta = old_lchan->rqd_ta;
		}
		new_lchan->bs_power = old_lchan->bs_power;
		new_lchan->rsl_cmode = old_lchan->rsl_cmode;
		new_lchan->tch_mode = old_lchan->tch_mode;
		memcpy(&new_lchan->mr_ms_lv, &old_lchan->mr_ms_lv, sizeof(new_lchan->mr_ms_lv));
		memcpy(&new_lchan->mr_bts_lv, &old_lchan->mr_bts_lv, sizeof(new_lchan->mr_bts_lv));
	} else {
		/* TODO: derive from MSC's HANDOVER REQUEST? */
	}

	rc = rsl_chan_activate_lchan(new_lchan,
				     ho->mt.async ? RSL_ACT_INTER_ASYNC : RSL_ACT_INTER_SYNC,
				     ho->mt.ho_ref);
	if (rc < 0) {
		LOGPHO(ho, LOGL_INFO, "Failure: activate lchan rc = %d\n", rc);
		lchan_free(new_lchan);
		ho->mt.new_lchan = NULL;
		handover_end(ho, HO_RESULT_FAIL_NO_CHANNEL);
		return;
	}

	rsl_lchan_set_state(new_lchan, LCHAN_S_ACT_REQ);

	/* Continue in ho_logic_sig_cb_on_new_lchan() receiving S_LCHAN_ACTIVATE_ACK and sending a
	 * GSCON_EV_HO_CHAN_ACTIV_ACK to the gscon_fsm (or others on error). */
}

static void ho_logic_sig_cb_on_old_lchan(unsigned int signal, struct gsm_lchan *old_lchan)
{
	struct handover *ho = old_lchan->conn->ho;

	if (!handover_is_sane(ho, old_lchan, NULL))
		return;

	switch (signal) {
	case S_LCHAN_HANDOVER_FAIL:
		/* GSM 04.08 HANDOVER FAIL has been received */
		handover_end(ho, HO_RESULT_FAIL_RR_HO_FAIL);
		return;
	default:
		OSMO_ASSERT(false);
	}
}

static void ho_logic_sig_cb_on_new_lchan(unsigned int signal, struct gsm_lchan *new_lchan)
{
	struct handover *ho;
	struct osmo_fsm_inst *fi;

	/* Don't care about lchans that have no conn */
	if (!new_lchan->conn)
		return;

	/* Don't care about lchans that are not in handover */
	ho = new_lchan->conn->ho;
	if (!ho)
		return;

	/* This lchan has an ongoing handover. Check it. */
	if (!handover_is_sane(ho, NULL, new_lchan))
		return;

	fi = ho->conn->fi;

	switch (signal) {
	case S_LCHAN_ACTIVATE_ACK:
		/* RSL has acknowledged activation of the new lchan */
		osmo_fsm_inst_dispatch(fi, GSCON_EV_HO_CHAN_ACTIV_ACK, new_lchan);
		return;
	case S_LCHAN_ACTIVATE_NACK:
		/* RSL has NACK'ed activation of the new lchan */
		handover_end(ho, HO_RESULT_FAIL_NO_CHANNEL);
		return;
	case S_LCHAN_HANDOVER_DETECT:
		/* GSM 08.58 HANDOVER DETECT has been received */
		osmo_fsm_inst_dispatch(fi, GSCON_EV_HO_DETECT, NULL);
		return;
	case S_LCHAN_HANDOVER_COMPL:
		/* GSM 04.08 HANDOVER COMPLETE has been received on new channel */
		osmo_fsm_inst_dispatch(fi, GSCON_EV_HO_COMPL, NULL);
		return;
	default:
		OSMO_ASSERT(false);
	}
}

static int ho_logic_sig_cb(unsigned int subsys, unsigned int signal,
			   void *handler_data, void *signal_data)
{
	struct lchan_signal_data *lchan_data;
	struct gsm_lchan *lchan;

	lchan_data = signal_data;
	switch (subsys) {
	case SS_LCHAN:
		OSMO_ASSERT(lchan_data);
		lchan = lchan_data->lchan;
		OSMO_ASSERT(lchan);

		switch (signal) {
		case S_LCHAN_MEAS_REP:
			ho_meas_rep(lchan_data->mr);
			break;
		case S_LCHAN_ACTIVATE_ACK:
		case S_LCHAN_ACTIVATE_NACK:
		case S_LCHAN_HANDOVER_DETECT:
		case S_LCHAN_HANDOVER_COMPL:
			ho_logic_sig_cb_on_new_lchan(signal, lchan);
			break;
		case S_LCHAN_HANDOVER_FAIL:
			ho_logic_sig_cb_on_old_lchan(signal, lchan);
			break;
		}

	default:
		break;
	}
	return 0;
}

static __attribute__((constructor)) void on_dso_load_ho_logic(void)
{
	osmo_signal_register_handler(SS_LCHAN, ho_logic_sig_cb, NULL);
}
