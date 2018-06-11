/* GSM Mobile Radio Interface Layer 3 messages on the A-bis interface
 * 3GPP TS 04.08 version 7.21.0 Release 1998 / ETSI TS 100 940 V7.21.0
 * utility functions
 */

/* (C) 2008-2009 by Harald Welte <laforge@gnumonks.org>
 * (C) 2008, 2009 by Holger Hans Peter Freyther <zecke@selfish.org>
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
#include <errno.h>
#include <netinet/in.h>

#include <osmocom/core/msgb.h>
#include <osmocom/gsm/gsm48.h>

#include <osmocom/bsc/abis_rsl.h>
#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/paging.h>
#include <osmocom/bsc/signal.h>
#include <osmocom/bsc/bsc_api.h>
#include <osmocom/bsc/gsm_04_08_utils.h>

/* should ip.access BTS use direct RTP streams between each other (1),
 * or should OpenBSC always act as RTP relay/proxy in between (0) ? */
int ipacc_rtp_direct = 1;

/* Section 9.1.8 / Table 9.9 */
struct chreq {
	uint8_t val;
	uint8_t mask;
	enum chreq_type type;
};

/* If SYSTEM INFORMATION TYPE 4 NECI bit == 1 */
static const struct chreq chreq_type_neci1[] = {
	{ 0xa0, 0xe0, CHREQ_T_EMERG_CALL },
	{ 0xc0, 0xe0, CHREQ_T_CALL_REEST_TCH_F },
	{ 0x68, 0xfc, CHREQ_T_CALL_REEST_TCH_H },
	{ 0x6c, 0xfc, CHREQ_T_CALL_REEST_TCH_H_DBL },
	{ 0xe0, 0xe0, CHREQ_T_TCH_F },
	{ 0x40, 0xf0, CHREQ_T_VOICE_CALL_TCH_H },
	{ 0x50, 0xf0, CHREQ_T_DATA_CALL_TCH_H },
	{ 0x00, 0xf0, CHREQ_T_LOCATION_UPD },
	{ 0x10, 0xf0, CHREQ_T_SDCCH },
	{ 0x80, 0xe0, CHREQ_T_PAG_R_ANY_NECI1 },
	{ 0x20, 0xf0, CHREQ_T_PAG_R_TCH_F },
	{ 0x30, 0xf0, CHREQ_T_PAG_R_TCH_FH },
	{ 0x67, 0xff, CHREQ_T_LMU },
	{ 0x60, 0xf9, CHREQ_T_RESERVED_SDCCH },
	{ 0x61, 0xfb, CHREQ_T_RESERVED_SDCCH },
	{ 0x63, 0xff, CHREQ_T_RESERVED_SDCCH },
	{ 0x70, 0xf8, CHREQ_T_PDCH_TWO_PHASE },
	{ 0x78, 0xfc, CHREQ_T_PDCH_ONE_PHASE },
	{ 0x78, 0xfa, CHREQ_T_PDCH_ONE_PHASE },
	{ 0x78, 0xf9, CHREQ_T_PDCH_ONE_PHASE },
	{ 0x7f, 0xff, CHREQ_T_RESERVED_IGNORE },
};

/* If SYSTEM INFORMATION TYPE 4 NECI bit == 0 */
static const struct chreq chreq_type_neci0[] = {
	{ 0xa0, 0xe0, CHREQ_T_EMERG_CALL },
	{ 0xc0, 0xe0, CHREQ_T_CALL_REEST_TCH_H },
	{ 0xe0, 0xe0, CHREQ_T_TCH_F },
	{ 0x50, 0xf0, CHREQ_T_DATA_CALL_TCH_H },
	{ 0x00, 0xe0, CHREQ_T_LOCATION_UPD },
	{ 0x80, 0xe0, CHREQ_T_PAG_R_ANY_NECI0 },
	{ 0x20, 0xf0, CHREQ_T_PAG_R_TCH_F },
	{ 0x30, 0xf0, CHREQ_T_PAG_R_TCH_FH },
	{ 0x67, 0xff, CHREQ_T_LMU },
	{ 0x60, 0xf9, CHREQ_T_RESERVED_SDCCH },
	{ 0x61, 0xfb, CHREQ_T_RESERVED_SDCCH },
	{ 0x63, 0xff, CHREQ_T_RESERVED_SDCCH },
	{ 0x70, 0xf8, CHREQ_T_PDCH_TWO_PHASE },
	{ 0x78, 0xfc, CHREQ_T_PDCH_ONE_PHASE },
	{ 0x78, 0xfa, CHREQ_T_PDCH_ONE_PHASE },
	{ 0x78, 0xf9, CHREQ_T_PDCH_ONE_PHASE },
	{ 0x7f, 0xff, CHREQ_T_RESERVED_IGNORE },
};

static const enum gsm_chan_t ctype_by_chreq[] = {
	[CHREQ_T_EMERG_CALL]		= GSM_LCHAN_TCH_F,
	[CHREQ_T_CALL_REEST_TCH_F]	= GSM_LCHAN_TCH_F,
	[CHREQ_T_CALL_REEST_TCH_H]	= GSM_LCHAN_TCH_H,
	[CHREQ_T_CALL_REEST_TCH_H_DBL]	= GSM_LCHAN_TCH_H,
	[CHREQ_T_SDCCH]			= GSM_LCHAN_SDCCH,
	[CHREQ_T_TCH_F]			= GSM_LCHAN_TCH_F,
	[CHREQ_T_VOICE_CALL_TCH_H]	= GSM_LCHAN_TCH_H,
	[CHREQ_T_DATA_CALL_TCH_H]	= GSM_LCHAN_TCH_H,
	[CHREQ_T_LOCATION_UPD]		= GSM_LCHAN_SDCCH,
	[CHREQ_T_PAG_R_ANY_NECI1]	= GSM_LCHAN_SDCCH,
	[CHREQ_T_PAG_R_ANY_NECI0]	= GSM_LCHAN_SDCCH,
	[CHREQ_T_PAG_R_TCH_F]		= GSM_LCHAN_TCH_F,
	[CHREQ_T_PAG_R_TCH_FH]		= GSM_LCHAN_TCH_H,
	[CHREQ_T_LMU]			= GSM_LCHAN_SDCCH,
	[CHREQ_T_RESERVED_SDCCH]	= GSM_LCHAN_SDCCH,
	[CHREQ_T_PDCH_ONE_PHASE]	= GSM_LCHAN_PDTCH,
	[CHREQ_T_PDCH_TWO_PHASE]	= GSM_LCHAN_PDTCH,
	[CHREQ_T_RESERVED_IGNORE]	= GSM_LCHAN_UNKNOWN,
};

static const enum gsm_chreq_reason_t reason_by_chreq[] = {
	[CHREQ_T_EMERG_CALL]		= GSM_CHREQ_REASON_EMERG,
	[CHREQ_T_CALL_REEST_TCH_F]	= GSM_CHREQ_REASON_CALL,
	[CHREQ_T_CALL_REEST_TCH_H]	= GSM_CHREQ_REASON_CALL,
	[CHREQ_T_CALL_REEST_TCH_H_DBL]	= GSM_CHREQ_REASON_CALL,
	[CHREQ_T_SDCCH]			= GSM_CHREQ_REASON_OTHER,
	[CHREQ_T_TCH_F]			= GSM_CHREQ_REASON_OTHER,
	[CHREQ_T_VOICE_CALL_TCH_H]	= GSM_CHREQ_REASON_CALL,
	[CHREQ_T_DATA_CALL_TCH_H]	= GSM_CHREQ_REASON_OTHER,
	[CHREQ_T_LOCATION_UPD]		= GSM_CHREQ_REASON_LOCATION_UPD,
	[CHREQ_T_PAG_R_ANY_NECI1]	= GSM_CHREQ_REASON_PAG,
	[CHREQ_T_PAG_R_ANY_NECI0]	= GSM_CHREQ_REASON_PAG,
	[CHREQ_T_PAG_R_TCH_F]		= GSM_CHREQ_REASON_PAG,
	[CHREQ_T_PAG_R_TCH_FH]		= GSM_CHREQ_REASON_PAG,
	[CHREQ_T_LMU]			= GSM_CHREQ_REASON_OTHER,
	[CHREQ_T_PDCH_ONE_PHASE]	= GSM_CHREQ_REASON_PDCH,
	[CHREQ_T_PDCH_TWO_PHASE]	= GSM_CHREQ_REASON_PDCH,
	[CHREQ_T_RESERVED_SDCCH]	= GSM_CHREQ_REASON_OTHER,
	[CHREQ_T_RESERVED_IGNORE]	= GSM_CHREQ_REASON_OTHER,
};

/* verify that the two tables match */
osmo_static_assert(sizeof(ctype_by_chreq) ==
	      sizeof(((struct gsm_network *) NULL)->ctype_by_chreq), assert_size);

/*
 * Update channel types for request based on policy. E.g. in the
 * case of a TCH/H network/bsc use TCH/H for the emergency calls,
 * for early assignment assign a SDCCH and some other options.
 */
void gsm_net_update_ctype(struct gsm_network *network)
{
	/* copy over the data */
	memcpy(network->ctype_by_chreq, ctype_by_chreq, sizeof(ctype_by_chreq));

	/*
	 * Use TCH/H for emergency calls when this cell allows TCH/H. Maybe it
	 * is better to iterate over the BTS/TRX and check if no TCH/F is available
	 * and then set it to TCH/H.
	 */
	if (network->neci)
		network->ctype_by_chreq[CHREQ_T_EMERG_CALL] = GSM_LCHAN_TCH_H;

	if (network->pag_any_tch) {
		if (network->neci) {
			network->ctype_by_chreq[CHREQ_T_PAG_R_ANY_NECI0] = GSM_LCHAN_TCH_H;
			network->ctype_by_chreq[CHREQ_T_PAG_R_ANY_NECI1] = GSM_LCHAN_TCH_H;
		} else {
			network->ctype_by_chreq[CHREQ_T_PAG_R_ANY_NECI0] = GSM_LCHAN_TCH_F;
			network->ctype_by_chreq[CHREQ_T_PAG_R_ANY_NECI1] = GSM_LCHAN_TCH_F;
		}
	}
}

enum gsm_chan_t get_ctype_by_chreq(struct gsm_network *network, uint8_t ra)
{
	int i;
	int length;
	const struct chreq *chreq;

	if (network->neci) {
		chreq = chreq_type_neci1;
		length = ARRAY_SIZE(chreq_type_neci1);
	} else {
		chreq = chreq_type_neci0;
		length = ARRAY_SIZE(chreq_type_neci0);
	}


	for (i = 0; i < length; i++) {
		const struct chreq *chr = &chreq[i];
		if ((ra & chr->mask) == chr->val)
			return network->ctype_by_chreq[chr->type];
	}
	LOGP(DRR, LOGL_ERROR, "Unknown CHANNEL REQUEST RQD 0x%02x\n", ra);
	return GSM_LCHAN_SDCCH;
}

int get_reason_by_chreq(uint8_t ra, int neci)
{
	int i;
	int length;
	const struct chreq *chreq;

	if (neci) {
		chreq = chreq_type_neci1;
		length = ARRAY_SIZE(chreq_type_neci1);
	} else {
		chreq = chreq_type_neci0;
		length = ARRAY_SIZE(chreq_type_neci0);
	}

	for (i = 0; i < length; i++) {
		const struct chreq *chr = &chreq[i];
		if ((ra & chr->mask) == chr->val)
			return reason_by_chreq[chr->type];
	}
	LOGP(DRR, LOGL_ERROR, "Unknown CHANNEL REQUEST REASON 0x%02x\n", ra);
	return GSM_CHREQ_REASON_OTHER;
}

void gsm48_cell_desc(struct gsm48_cell_desc *cd, const struct gsm_bts *bts)
{
	cd->ncc = (bts->bsic >> 3 & 0x7);
	cd->bcc = (bts->bsic & 0x7);
	cd->arfcn_hi = bts->c0->arfcn >> 8;
	cd->arfcn_lo = bts->c0->arfcn & 0xff;
}

/*! \brief Encode a TS 04.08 multirate config LV according to 10.5.2.21aa
 *  \param[out] lv caller-allocated buffer of 7 bytes. First octet is IS length
 *  \param[in] mr multi-rate configuration to encode
 *  \param[in] modes array describing the AMR modes
 *  \returns 0 on success */
int gsm48_multirate_config(uint8_t *lv, const struct amr_multirate_conf *mr, const struct amr_mode *modes)
{
	int num = 0, i;

	for (i = 0; i < 8; i++) {
		if (((mr->gsm48_ie[1] >> i) & 1))
			num++;
	}
	if (num > 4) {
		LOGP(DRR, LOGL_ERROR, "BUG: Using multirate codec with too "
				"many modes in config.\n");
		num = 4;
	}
	if (num < 1) {
		LOGP(DRR, LOGL_ERROR, "BUG: Using multirate codec with no "
				"mode in config.\n");
		num = 1;
	}

	lv[0] = (num == 1) ? 2 : (num + 2);
	memcpy(lv + 1, mr->gsm48_ie, 2);
	if (num == 1)
		return 0;

	lv[3] = modes[0].threshold & 0x3f;
	lv[4] = modes[0].hysteresis << 4;
	if (num == 2)
		return 0;
	lv[4] |= (modes[1].threshold & 0x3f) >> 2;
	lv[5] = modes[1].threshold << 6;
	lv[5] |= (modes[1].hysteresis & 0x0f) << 2;
	if (num == 3)
		return 0;
	lv[5] |= (modes[2].threshold & 0x3f) >> 4;
	lv[6] = modes[2].threshold << 4;
	lv[6] |= modes[2].hysteresis & 0x0f;

	return 0;
}

#define GSM48_HOCMD_CCHDESC_LEN	16

/* Chapter 9.1.15: Handover Command */
struct msgb *gsm48_make_ho_cmd(struct gsm_lchan *new_lchan, uint8_t power_command, uint8_t ho_ref)
{
	struct msgb *msg = gsm48_msgb_alloc_name("GSM 04.08 HO CMD");
	struct gsm48_hdr *gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh));
	struct gsm48_ho_cmd *ho =
		(struct gsm48_ho_cmd *) msgb_put(msg, sizeof(*ho));

	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_HANDO_CMD;

	/* mandatory bits */
	gsm48_cell_desc(&ho->cell_desc, new_lchan->ts->trx->bts);
	gsm48_lchan2chan_desc(&ho->chan_desc, new_lchan);
	ho->ho_ref = ho_ref;
	ho->power_command = power_command;

	if (new_lchan->ts->hopping.enabled) {
		struct gsm_bts *bts = new_lchan->ts->trx->bts;
		struct gsm48_system_information_type_1 *si1;
		uint8_t *cur;

		si1 = GSM_BTS_SI(bts, SYSINFO_TYPE_1);
		/* Copy the Cell Chan Desc (ARFCNS in this cell) */
		msgb_put_u8(msg, GSM48_IE_CELL_CH_DESC);
		cur = msgb_put(msg, GSM48_HOCMD_CCHDESC_LEN);
		memcpy(cur, si1->cell_channel_description,
			GSM48_HOCMD_CCHDESC_LEN);
		/* Copy the Mobile Allocation */
		msgb_tlv_put(msg, GSM48_IE_MA_BEFORE,
			     new_lchan->ts->hopping.ma_len,
			     new_lchan->ts->hopping.ma_data);
	}
	/* FIXME: optional bits for type of synchronization? */

	msgb_tv_put(msg, GSM48_IE_CHANMODE_1, new_lchan->tch_mode);

	/* in case of multi rate we need to attach a config */
	if (new_lchan->tch_mode == GSM48_CMODE_SPEECH_AMR)
		msgb_tlv_put(msg, GSM48_IE_MUL_RATE_CFG, new_lchan->mr_ms_lv[0],
			new_lchan->mr_ms_lv + 1);

	return msg;
}

int gsm48_parse_meas_rep(struct gsm_meas_rep *rep, struct msgb *msg)
{
	struct gsm48_hdr *gh = msgb_l3(msg);
	uint8_t *data = gh->data;
	struct gsm_bts *bts = msg->lchan->ts->trx->bts;
	struct bitvec *nbv = &bts->si_common.neigh_list;
	struct gsm_meas_rep_cell *mrc;

	if (gh->msg_type != GSM48_MT_RR_MEAS_REP)
		return -EINVAL;

	if (data[0] & 0x80)
		rep->flags |= MEAS_REP_F_BA1;
	if (data[0] & 0x40)
		rep->flags |= MEAS_REP_F_UL_DTX;
	if ((data[1] & 0x40) == 0x00)
		rep->flags |= MEAS_REP_F_DL_VALID;

	rep->dl.full.rx_lev = data[0] & 0x3f;
	rep->dl.sub.rx_lev = data[1] & 0x3f;
	rep->dl.full.rx_qual = (data[2] >> 4) & 0x7;
	rep->dl.sub.rx_qual = (data[2] >> 1) & 0x7;

	rep->num_cell = ((data[3] >> 6) & 0x3) | ((data[2] & 0x01) << 2);
	if (rep->num_cell < 1 || rep->num_cell > 6) {
		/* There are no neighbor cell reports present. */
		rep->num_cell = 0;
		return 0;
	}

	/* an encoding nightmare in perfection */
	mrc = &rep->cell[0];
	mrc->rxlev = data[3] & 0x3f;
	mrc->neigh_idx = data[4] >> 3;
	mrc->arfcn = bitvec_get_nth_set_bit(nbv, mrc->neigh_idx + 1);
	mrc->bsic = ((data[4] & 0x07) << 3) | (data[5] >> 5);
	if (rep->num_cell < 2)
		return 0;

	mrc = &rep->cell[1];
	mrc->rxlev = ((data[5] & 0x1f) << 1) | (data[6] >> 7);
	mrc->neigh_idx = (data[6] >> 2) & 0x1f;
	mrc->arfcn = bitvec_get_nth_set_bit(nbv, mrc->neigh_idx + 1);
	mrc->bsic = ((data[6] & 0x03) << 4) | (data[7] >> 4);
	if (rep->num_cell < 3)
		return 0;

	mrc = &rep->cell[2];
	mrc->rxlev = ((data[7] & 0x0f) << 2) | (data[8] >> 6);
	mrc->neigh_idx = (data[8] >> 1) & 0x1f;
	mrc->arfcn = bitvec_get_nth_set_bit(nbv, mrc->neigh_idx + 1);
	mrc->bsic = ((data[8] & 0x01) << 5) | (data[9] >> 3);
	if (rep->num_cell < 4)
		return 0;

	mrc = &rep->cell[3];
	mrc->rxlev = ((data[9] & 0x07) << 3) | (data[10] >> 5);
	mrc->neigh_idx = data[10] & 0x1f;
	mrc->arfcn = bitvec_get_nth_set_bit(nbv, mrc->neigh_idx + 1);
	mrc->bsic = data[11] >> 2;
	if (rep->num_cell < 5)
		return 0;

	mrc = &rep->cell[4];
	mrc->rxlev = ((data[11] & 0x03) << 4) | (data[12] >> 4);
	mrc->neigh_idx = ((data[12] & 0xf) << 1) | (data[13] >> 7);
	mrc->arfcn = bitvec_get_nth_set_bit(nbv, mrc->neigh_idx + 1);
	mrc->bsic = (data[13] >> 1) & 0x3f;
	if (rep->num_cell < 6)
		return 0;

	mrc = &rep->cell[5];
	mrc->rxlev = ((data[13] & 0x01) << 5) | (data[14] >> 3);
	mrc->neigh_idx = ((data[14] & 0x07) << 2) | (data[15] >> 6);
	mrc->arfcn = bitvec_get_nth_set_bit(nbv, mrc->neigh_idx + 1);
	mrc->bsic = data[15] & 0x3f;

	return 0;
}

struct msgb *gsm48_create_mm_serv_rej(enum gsm48_reject_value value)
{
	struct msgb *msg;
	struct gsm48_hdr *gh;

	msg = gsm48_msgb_alloc_name("GSM 04.08 SERV REJ");
	if (!msg)
		return NULL;

	gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh) + 1);
	gh->proto_discr = GSM48_PDISC_MM;
	gh->msg_type = GSM48_MT_MM_CM_SERV_REJ;
	gh->data[0] = value;

	return msg;
}

struct msgb *gsm48_create_loc_upd_rej(uint8_t cause)
{
	struct gsm48_hdr *gh;
	struct msgb *msg;

	msg = gsm48_msgb_alloc_name("GSM 04.08 LOC UPD REJ");
	if (!msg)
		return NULL;

	gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh) + 1);
	gh->proto_discr = GSM48_PDISC_MM;
	gh->msg_type = GSM48_MT_MM_LOC_UPD_REJECT;
	gh->data[0] = cause;
	return msg;
}

int gsm48_extract_mi(uint8_t *classmark2_lv, int length, char *mi_string, uint8_t *mi_type)
{
	/* Check the size for the classmark */
	if (length < 1 + *classmark2_lv)
		return -1;

	uint8_t *mi_lv = classmark2_lv + *classmark2_lv + 1;
	if (length < 2 + *classmark2_lv + mi_lv[0])
		return -2;

	*mi_type = mi_lv[1] & GSM_MI_TYPE_MASK;
	return gsm48_mi_to_string(mi_string, GSM48_MI_SIZE, mi_lv+1, *mi_lv);
}

int gsm48_paging_extract_mi(struct gsm48_pag_resp *resp, int length,
			    char *mi_string, uint8_t *mi_type)
{
	static const uint32_t classmark_offset =
		offsetof(struct gsm48_pag_resp, classmark2);
	uint8_t *classmark2_lv = (uint8_t *) &resp->classmark2;
	return gsm48_extract_mi(classmark2_lv, length - classmark_offset,
				mi_string, mi_type);
}

/* As per TS 03.03 Section 2.2, the IMSI has 'not more than 15 digits' */
uint64_t str_to_imsi(const char *imsi_str)
{
	uint64_t ret;

	ret = strtoull(imsi_str, NULL, 10);

	return ret;
}
