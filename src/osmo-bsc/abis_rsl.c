/* GSM Radio Signalling Link messages on the A-bis interface
 * 3GPP TS 08.58 version 8.6.0 Release 1999 / ETSI TS 100 596 V8.6.0 */

/* (C) 2008-2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2012 by Holger Hans Peter Freyther
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
#include <arpa/inet.h>

#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/timeslot_fsm.h>
#include <osmocom/bsc/lchan_fsm.h>
#include <osmocom/bsc/lchan_select.h>
#include <osmocom/bsc/gsm_04_08_utils.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/bsc/abis_rsl.h>
#include <osmocom/bsc/lchan_fsm.h>
#include <osmocom/bsc/bsc_rll.h>
#include <osmocom/bsc/debug.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/protocol/gsm_08_58.h>
#include <osmocom/bsc/paging.h>
#include <osmocom/bsc/signal.h>
#include <osmocom/bsc/meas_rep.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/gsm/rsl.h>
#include <osmocom/core/talloc.h>
#include <osmocom/bsc/pcu_if.h>
#include <osmocom/bsc/bsc_api.h>
#include <osmocom/bsc/bsc_subscr_conn_fsm.h>
#include <osmocom/bsc/gsm_timers.h>
#include <osmocom/bsc/assignment_fsm.h>
#include <osmocom/bsc/handover_fsm.h>

#define RSL_ALLOC_SIZE		1024
#define RSL_ALLOC_HEADROOM	128

#define RTP_PT_GSM_FULL 3
#define RTP_PT_GSM_HALF 96
#define RTP_PT_GSM_EFR 97
#define RTP_PT_AMR 98

enum sacch_deact {
	SACCH_NONE,
	SACCH_DEACTIVATE,
};

static void send_lchan_signal(int sig_no, struct gsm_lchan *lchan,
			      struct gsm_meas_rep *resp)
{
	struct lchan_signal_data sig;
	sig.lchan = lchan;
	sig.mr = resp;
	osmo_signal_dispatch(SS_LCHAN, sig_no, &sig);
}

static void count_codecs(struct gsm_bts *bts, struct gsm_lchan *lchan)
{
	OSMO_ASSERT(bts);

	if (lchan->type == GSM_LCHAN_TCH_H) {
		switch (lchan->tch_mode) {
		case GSM48_CMODE_SPEECH_AMR:
			rate_ctr_inc(&bts->bts_ctrs->ctr[BTS_CTR_CODEC_AMR_H]);
			break;
		case GSM48_CMODE_SPEECH_V1:
			rate_ctr_inc(&bts->bts_ctrs->ctr[BTS_CTR_CODEC_V1_HR]);
			break;
		default:
			break;
		}
	} else if (lchan->type == GSM_LCHAN_TCH_F) {
		switch (lchan->tch_mode) {
		case GSM48_CMODE_SPEECH_AMR:
			rate_ctr_inc(&bts->bts_ctrs->ctr[BTS_CTR_CODEC_AMR_F]);
			break;
		case GSM48_CMODE_SPEECH_V1:
			rate_ctr_inc(&bts->bts_ctrs->ctr[BTS_CTR_CODEC_V1_FR]);
			break;
		case GSM48_CMODE_SPEECH_EFR:
			rate_ctr_inc(&bts->bts_ctrs->ctr[BTS_CTR_CODEC_EFR]);
			break;
		default:
			break;
		}
	}
}

static uint8_t mdisc_by_msgtype(uint8_t msg_type)
{
	/* mask off the transparent bit ? */
	msg_type &= 0xfe;

	if ((msg_type & 0xf0) == 0x00)
		return ABIS_RSL_MDISC_RLL;
	if ((msg_type & 0xf0) == 0x10) {
		if (msg_type >= 0x19 && msg_type <= 0x22)
			return ABIS_RSL_MDISC_TRX;
		else
			return ABIS_RSL_MDISC_COM_CHAN;
	}
	if ((msg_type & 0xe0) == 0x20)
		return ABIS_RSL_MDISC_DED_CHAN;

	return ABIS_RSL_MDISC_LOC;
}

static inline void init_dchan_hdr(struct abis_rsl_dchan_hdr *dh,
				  uint8_t msg_type)
{
	dh->c.msg_discr = mdisc_by_msgtype(msg_type);
	dh->c.msg_type = msg_type;
	dh->ie_chan = RSL_IE_CHAN_NR;
}

/* call rsl_lchan_lookup and set the log context */
static struct gsm_lchan *lchan_lookup(struct gsm_bts_trx *trx, uint8_t chan_nr,
				      const char *log_name)
{
	int rc;
	struct gsm_lchan *lchan = rsl_lchan_lookup(trx, chan_nr, &rc);

	if (!lchan) {
		LOGP(DRSL, LOGL_ERROR, "%sunknown chan_nr=0x%02x\n",
		     log_name, chan_nr);
		return NULL;
	}

	if (rc < 0)
		LOGP(DRSL, LOGL_ERROR, "%s %smismatching chan_nr=0x%02x\n",
		     gsm_ts_and_pchan_name(lchan->ts), log_name, chan_nr);

	return lchan;
}

static struct msgb *rsl_msgb_alloc(void)
{
	return msgb_alloc_headroom(RSL_ALLOC_SIZE, RSL_ALLOC_HEADROOM,
				   "RSL");
}

static void pad_macblock(uint8_t *out, const uint8_t *in, int len)
{
	memcpy(out, in, len);

	if (len < GSM_MACBLOCK_LEN)
		memset(out+len, 0x2b, GSM_MACBLOCK_LEN - len);
}

/* Chapter 9.3.7: Encryption Information */
static int build_encr_info(uint8_t *out, struct gsm_lchan *lchan)
{
	*out++ = lchan->encr.alg_id & 0xff;
	if (lchan->encr.key_len)
		memcpy(out, lchan->encr.key, lchan->encr.key_len);
	return lchan->encr.key_len + 1;
}

/* If the TLV contain an RSL Cause IE, return the RSL cause name and point *rsl_cause_pp at the cause
 * value. If there is no Cause IE, return NULL and write NULL to *rsl_cause_pp. If NULL is passed as
 * rsl_cause_pp, ignore it. Implementation choice: presence of a Cause IE cannot be indicated by a zero
 * cause, because that would mean RSL_ERR_RADIO_IF_FAIL; a pointer-to-pointer can return NULL or point to
 * a cause value. */
static const char *rsl_cause_name(struct tlv_parsed *tp, const uint8_t **rsl_cause_pp)
{
	static char buf[128];
	if (rsl_cause_pp)
		*rsl_cause_pp = NULL;

	if (TLVP_PRESENT(tp, RSL_IE_CAUSE)) {
		const uint8_t *cause = TLVP_VAL(tp, RSL_IE_CAUSE);
		if (rsl_cause_pp)
			*rsl_cause_pp = cause;
		snprintf(buf, sizeof(buf), " (cause=%s [ %s])",
			 rsl_err_name(*cause),
			 osmo_hexdump(cause, TLVP_LEN(tp, RSL_IE_CAUSE)));
		return buf;
	} else
		return "";
}

/* Send a BCCH_INFO message as per Chapter 8.5.1 */
int rsl_bcch_info(const struct gsm_bts_trx *trx, enum osmo_sysinfo_type si_type, const uint8_t *data, int len)
{
	struct abis_rsl_dchan_hdr *dh;
	const struct gsm_bts *bts = trx->bts;
	struct msgb *msg = rsl_msgb_alloc();
	uint8_t type = osmo_sitype2rsl(si_type);

	if (bts->c0 != trx)
		LOGP(DRR, LOGL_ERROR, "Attempting to set BCCH SI%s on wrong BTS%u/TRX%u\n",
		     get_value_string(osmo_sitype_strs, si_type), bts->nr, trx->nr);

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof*dh);
	init_dchan_hdr(dh, RSL_MT_BCCH_INFO);
	dh->chan_nr = RSL_CHAN_BCCH;

	if (trx->bts->type == GSM_BTS_TYPE_RBS2000
	    && type == RSL_SYSTEM_INFO_13) {
		/* Ericsson proprietary encoding of SI13 */
		msgb_tv_put(msg, RSL_IE_SYSINFO_TYPE, RSL_ERIC_SYSTEM_INFO_13);
		if (data)
			msgb_tlv_put(msg, RSL_IE_FULL_BCCH_INFO, len, data);
		msgb_tv_put(msg, RSL_IE_ERIC_BCCH_MAPPING, 0x00);
	} else {
		/* Normal encoding */
		msgb_tv_put(msg, RSL_IE_SYSINFO_TYPE, type);
		if (data)
			msgb_tlv_put(msg, RSL_IE_FULL_BCCH_INFO, len, data);
	}

	msg->dst = trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_sacch_filling(struct gsm_bts_trx *trx, uint8_t type,
		      const uint8_t *data, int len)
{
	struct abis_rsl_common_hdr *ch;
	struct msgb *msg = rsl_msgb_alloc();

	ch = (struct abis_rsl_common_hdr *) msgb_put(msg, sizeof(*ch));
	ch->msg_discr = ABIS_RSL_MDISC_TRX;
	ch->msg_type = RSL_MT_SACCH_FILL;

	msgb_tv_put(msg, RSL_IE_SYSINFO_TYPE, type);
	if (data)
		msgb_tl16v_put(msg, RSL_IE_L3_INFO, len, data);

	msg->dst = trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_sacch_info_modify(struct gsm_lchan *lchan, uint8_t type,
			  const uint8_t *data, int len)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg = rsl_msgb_alloc();
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_SACCH_INFO_MODIFY);
	dh->chan_nr = chan_nr;

	msgb_tv_put(msg, RSL_IE_SYSINFO_TYPE, type);
	if (data)
		msgb_tl16v_put(msg, RSL_IE_L3_INFO, len, data);

	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_chan_bs_power_ctrl(struct gsm_lchan *lchan, unsigned int fpc, int db)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);

	db = abs(db);
	if (db > 30)
		return -EINVAL;

	msg = rsl_msgb_alloc();

	lchan->bs_power = db/2;
	if (fpc)
		lchan->bs_power |= 0x10;

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_BS_POWER_CONTROL);
	dh->chan_nr = chan_nr;

	msgb_tv_put(msg, RSL_IE_BS_POWER, lchan->bs_power);

	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_chan_ms_power_ctrl(struct gsm_lchan *lchan, unsigned int fpc, int dbm)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);
	int ctl_lvl;

	ctl_lvl = ms_pwr_ctl_lvl(lchan->ts->trx->bts->band, dbm);
	if (ctl_lvl < 0)
		return ctl_lvl;

	msg = rsl_msgb_alloc();

	lchan->ms_power = ctl_lvl;

	if (fpc)
		lchan->ms_power |= 0x20;

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_MS_POWER_CONTROL);
	dh->chan_nr = chan_nr;

	msgb_tv_put(msg, RSL_IE_MS_POWER, lchan->ms_power);

	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

static int channel_mode_from_lchan(struct rsl_ie_chan_mode *cm,
				   struct gsm_lchan *lchan)
{
	memset(cm, 0, sizeof(*cm));

	/* FIXME: what to do with data calls ? */
	cm->dtx_dtu = 0;
	if (lchan->ts->trx->bts->dtxu != GSM48_DTX_SHALL_NOT_BE_USED)
		cm->dtx_dtu |= RSL_CMOD_DTXu;
	if (lchan->ts->trx->bts->dtxd)
		cm->dtx_dtu |= RSL_CMOD_DTXd;

	/* set TCH Speech/Data */
	cm->spd_ind = lchan->rsl_cmode;

	if (lchan->rsl_cmode == RSL_CMOD_SPD_SIGN &&
	    lchan->tch_mode != GSM48_CMODE_SIGN)
		LOGP(DRSL, LOGL_ERROR, "unsupported: rsl_mode == signalling, "
			"but tch_mode != signalling\n");

	switch (lchan->type) {
	case GSM_LCHAN_SDCCH:
		cm->chan_rt = RSL_CMOD_CRT_SDCCH;
		break;
	case GSM_LCHAN_TCH_F:
		cm->chan_rt = RSL_CMOD_CRT_TCH_Bm;
		break;
	case GSM_LCHAN_TCH_H:
		cm->chan_rt = RSL_CMOD_CRT_TCH_Lm;
		break;
	case GSM_LCHAN_NONE:
	case GSM_LCHAN_UNKNOWN:
	default:
		LOGP(DRSL, LOGL_ERROR,
		     "unsupported activation lchan->type %u %s\n",
		     lchan->type, gsm_lchant_name(lchan->type));
		return -EINVAL;
	}

	switch (lchan->tch_mode) {
	case GSM48_CMODE_SIGN:
		cm->chan_rate = 0;
		break;
	case GSM48_CMODE_SPEECH_V1:
		cm->chan_rate = RSL_CMOD_SP_GSM1;
		break;
	case GSM48_CMODE_SPEECH_EFR:
		cm->chan_rate = RSL_CMOD_SP_GSM2;
		break;
	case GSM48_CMODE_SPEECH_AMR:
		cm->chan_rate = RSL_CMOD_SP_GSM3;
		break;
	case GSM48_CMODE_DATA_14k5:
	case GSM48_CMODE_DATA_12k0:
	case GSM48_CMODE_DATA_6k0:
		switch (lchan->csd_mode) {
		case LCHAN_CSD_M_NT:
			/* non-transparent CSD with RLP */
			switch (lchan->tch_mode) {
			case GSM48_CMODE_DATA_14k5:
				cm->chan_rate = RSL_CMOD_SP_NT_14k5;
				break;
			case GSM48_CMODE_DATA_12k0:
				cm->chan_rate = RSL_CMOD_SP_NT_12k0;
				break;
			case GSM48_CMODE_DATA_6k0:
				cm->chan_rate = RSL_CMOD_SP_NT_6k0;
				break;
			default:
				LOGP(DRSL, LOGL_ERROR,
				     "unsupported lchan->tch_mode %u\n",
				     lchan->tch_mode);
				return -EINVAL;
			}
			break;
			/* transparent data services below */
		case LCHAN_CSD_M_T_1200_75:
			cm->chan_rate = RSL_CMOD_CSD_T_1200_75;
			break;
		case LCHAN_CSD_M_T_600:
			cm->chan_rate = RSL_CMOD_CSD_T_600;
			break;
		case LCHAN_CSD_M_T_1200:
			cm->chan_rate = RSL_CMOD_CSD_T_1200;
			break;
		case LCHAN_CSD_M_T_2400:
			cm->chan_rate = RSL_CMOD_CSD_T_2400;
			break;
		case LCHAN_CSD_M_T_9600:
			cm->chan_rate = RSL_CMOD_CSD_T_9600;
			break;
		case LCHAN_CSD_M_T_14400:
			cm->chan_rate = RSL_CMOD_CSD_T_14400;
			break;
		case LCHAN_CSD_M_T_29000:
			cm->chan_rate = RSL_CMOD_CSD_T_29000;
			break;
		case LCHAN_CSD_M_T_32000:
			cm->chan_rate = RSL_CMOD_CSD_T_32000;
			break;
		default:
			LOGP(DRSL, LOGL_ERROR,
			     "unsupported lchan->csd_mode %u\n",
			     lchan->csd_mode);
			return -EINVAL;
		}
		break;
	default:
		LOGP(DRSL, LOGL_ERROR,
		     "unsupported lchan->tch_mode %u\n",
		     lchan->tch_mode);
		return -EINVAL;
	}

	return 0;
}

static void mr_config_for_bts(struct gsm_lchan *lchan, struct msgb *msg)
{
	if (lchan->tch_mode == GSM48_CMODE_SPEECH_AMR)
		msgb_tlv_put(msg, RSL_IE_MR_CONFIG, lchan->mr_bts_lv[0],
			     lchan->mr_bts_lv + 1);
}

/* Chapter 8.4.1 */
int rsl_tx_chan_activ(struct gsm_lchan *lchan, uint8_t act_type, uint8_t ho_ref)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg;
	int rc;
	uint8_t *len;
	uint8_t ta;

	struct rsl_ie_chan_mode cm;
	struct gsm48_chan_desc cd;

	DEBUGP(DRSL, "%s Tx RSL Channel Activate with act_type=%s\n",
	       gsm_ts_and_pchan_name(lchan->ts),
	       rsl_act_type_name(act_type));

	/* PDCH activation is a job for rsl_tx_dyn_ts_pdch_act_deact(); */
	OSMO_ASSERT(act_type != RSL_ACT_OSMO_PDCH);

	rc = channel_mode_from_lchan(&cm, lchan);
	if (rc < 0) {
		LOGP(DRSL, LOGL_ERROR,
		     "%s Cannot find channel mode from lchan type\n",
		     gsm_ts_and_pchan_name(lchan->ts));
		return rc;
	}

	ta = lchan->rqd_ta;

	/* BS11 requires TA shifted by 2 bits */
	if (lchan->ts->trx->bts->type == GSM_BTS_TYPE_BS11)
		ta <<= 2;

	memset(&cd, 0, sizeof(cd));
	gsm48_lchan2chan_desc(&cd, lchan);

	msg = rsl_msgb_alloc();
	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_CHAN_ACTIV);

	dh->chan_nr = gsm_lchan2chan_nr(lchan);

	msgb_tv_put(msg, RSL_IE_ACT_TYPE, act_type);
	msgb_tlv_put(msg, RSL_IE_CHAN_MODE, sizeof(cm),
		     (uint8_t *) &cm);

	/*
	 * The Channel Identification is needed for Phase1 phones
	 * and it contains the GSM48 Channel Description and the
	 * Mobile Allocation. The GSM 08.58 asks for the Mobile
	 * Allocation to have a length of zero. We are using the
	 * msgb_l3len to calculate the length of both messages.
	 */
	msgb_v_put(msg, RSL_IE_CHAN_IDENT);
	len = msgb_put(msg, 1);
	msgb_tv_fixed_put(msg, GSM48_IE_CHANDESC_2, sizeof(cd), (const uint8_t *) &cd);

	if (lchan->ts->hopping.enabled)
		msgb_tlv_put(msg, GSM48_IE_MA_AFTER, lchan->ts->hopping.ma_len,
			     lchan->ts->hopping.ma_data);
	else
		msgb_tlv_put(msg, GSM48_IE_MA_AFTER, 0, NULL);

	/* update the calculated size */
	msg->l3h = len + 1;
	*len = msgb_l3len(msg);

	if (lchan->encr.alg_id > RSL_ENC_ALG_A5(0)) {
		uint8_t encr_info[MAX_A5_KEY_LEN+2];
		rc = build_encr_info(encr_info, lchan);
		if (rc > 0)
			msgb_tlv_put(msg, RSL_IE_ENCR_INFO, rc, encr_info);
	}

	switch (act_type) {
	case RSL_ACT_INTER_ASYNC:
	case RSL_ACT_INTER_SYNC:
		msgb_tv_put(msg, RSL_IE_HANDO_REF, ho_ref);
		break;
	default:
		break;
	}

	msgb_tv_put(msg, RSL_IE_BS_POWER, lchan->bs_power);
	msgb_tv_put(msg, RSL_IE_MS_POWER, lchan->ms_power);
	msgb_tv_put(msg, RSL_IE_TIMING_ADVANCE, ta);
	mr_config_for_bts(lchan, msg);

	msg->dst = lchan->ts->trx->rsl_link;

	rate_ctr_inc(&lchan->ts->trx->bts->bts_ctrs->ctr[BTS_CTR_CHAN_ACT_TOTAL]);

	return abis_rsl_sendmsg(msg);
}

/* Chapter 8.4.9: Modify channel mode on BTS side */
int rsl_chan_mode_modify_req(struct gsm_lchan *lchan)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg;
	int rc;

	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);
	struct rsl_ie_chan_mode cm;

	rc = channel_mode_from_lchan(&cm, lchan);
	if (rc < 0)
		return rc;

	msg = rsl_msgb_alloc();
	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_MODE_MODIFY_REQ);
	dh->chan_nr = chan_nr;

	msgb_tlv_put(msg, RSL_IE_CHAN_MODE, sizeof(cm),
		     (uint8_t *) &cm);

	if (lchan->encr.alg_id > RSL_ENC_ALG_A5(0)) {
		uint8_t encr_info[MAX_A5_KEY_LEN+2];
		rc = build_encr_info(encr_info, lchan);
		if (rc > 0)
			msgb_tlv_put(msg, RSL_IE_ENCR_INFO, rc, encr_info);
	}

	mr_config_for_bts(lchan, msg);

	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

/* Chapter 8.4.6: Send the encryption command with given L3 info */
int rsl_encryption_cmd(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh;
	struct gsm_lchan *lchan = msg->lchan;
	uint8_t chan_nr = gsm_lchan2chan_nr(lchan);
	uint8_t encr_info[MAX_A5_KEY_LEN+2];
	uint8_t l3_len = msg->len;
	int rc;

	/* First push the L3 IE tag and length */
	msgb_tv16_push(msg, RSL_IE_L3_INFO, l3_len);

	/* then the link identifier (SAPI0, main sign link) */
	msgb_tv_push(msg, RSL_IE_LINK_IDENT, 0);

	/* then encryption information */
	rc = build_encr_info(encr_info, lchan);
	if (rc <= 0)
		return rc;
	msgb_tlv_push(msg, RSL_IE_ENCR_INFO, rc, encr_info);

	/* and finally the DCHAN header */
	dh = (struct abis_rsl_dchan_hdr *) msgb_push(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_ENCR_CMD);
	dh->chan_nr = chan_nr;

	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

/* Chapter 8.4.5 / 4.6: Deactivate the SACCH after 04.08 RR CHAN RELEASE */
int rsl_deact_sacch(struct gsm_lchan *lchan)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg = rsl_msgb_alloc();

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_DEACTIVATE_SACCH);
	dh->chan_nr = gsm_lchan2chan_nr(lchan);

	msg->lchan = lchan;
	msg->dst = lchan->ts->trx->rsl_link;

	DEBUGP(DRSL, "%s DEACTivate SACCH CMD\n", gsm_lchan_name(lchan));

	return abis_rsl_sendmsg(msg);
}

/* Chapter 8.4.14 / 4.7: Tell BTS to release the radio channel */
int rsl_tx_rf_chan_release(struct gsm_lchan *lchan)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg;

	msg = rsl_msgb_alloc();
	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_RF_CHAN_REL);
	dh->chan_nr = gsm_lchan2chan_nr(lchan);

	msg->lchan = lchan;
	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_paging_cmd(struct gsm_bts *bts, uint8_t paging_group, uint8_t len,
		   uint8_t *ms_ident, uint8_t chan_needed, bool is_gprs)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *msg = rsl_msgb_alloc();

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_PAGING_CMD);
	dh->chan_nr = RSL_CHAN_PCH_AGCH;

	msgb_tv_put(msg, RSL_IE_PAGING_GROUP, paging_group);
	msgb_tlv_put(msg, RSL_IE_MS_IDENTITY, len-2, ms_ident+2);
	msgb_tv_put(msg, RSL_IE_CHAN_NEEDED, chan_needed);

	/* Ericsson wants to have this IE in case a paging message
	 * relates to packet paging */
	if (bts->type == GSM_BTS_TYPE_RBS2000 && is_gprs)
		msgb_tv_put(msg, RSL_IE_ERIC_PACKET_PAG_IND, 0);

	msg->dst = bts->c0->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_forward_layer3_info(struct gsm_lchan *lchan, const uint8_t *l3_info, uint8_t l3_info_len)
{
	struct msgb *msg;
	uint8_t *dst;

	if (!l3_info || !l3_info_len)
		return -EINVAL;

	msg = rsl_msgb_alloc();
	dst = msgb_put(msg, l3_info_len);
	memcpy(dst, l3_info, l3_info_len);

	msg->lchan = lchan;
	return rsl_data_request(msg, 0);
}

int imsi_str2bcd(uint8_t *bcd_out, const char *str_in)
{
	int i, len = strlen(str_in);

	for (i = 0; i < len; i++) {
		int num = str_in[i] - 0x30;
		if (num < 0 || num > 9)
			return -1;
		if (i % 2 == 0)
			bcd_out[i/2] = num;
		else
			bcd_out[i/2] |= (num << 4);
	}

	return 0;
}

/* Chapter 8.5.6 */
struct msgb *rsl_imm_assign_cmd_common(struct gsm_bts *bts, uint8_t len, uint8_t *val)
{
	struct msgb *msg = rsl_msgb_alloc();
	struct abis_rsl_dchan_hdr *dh;
	uint8_t buf[GSM_MACBLOCK_LEN];

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_IMMEDIATE_ASSIGN_CMD);
	dh->chan_nr = RSL_CHAN_PCH_AGCH;

	switch (bts->type) {
	case GSM_BTS_TYPE_BS11:
		msgb_tlv_put(msg, RSL_IE_IMM_ASS_INFO, len, val);
		break;
	default:
		/* If phase 2, construct a FULL_IMM_ASS_INFO */
		pad_macblock(buf, val, len);
		msgb_tlv_put(msg, RSL_IE_FULL_IMM_ASS_INFO, GSM_MACBLOCK_LEN,
			     buf);
		break;
	}

	msg->dst = bts->c0->rsl_link;
	return msg;
}

/* Chapter 8.5.6 */
int rsl_imm_assign_cmd(struct gsm_bts *bts, uint8_t len, uint8_t *val)
{
	struct msgb *msg = rsl_imm_assign_cmd_common(bts, len, val);
	if (!msg)
		return 1;
	return abis_rsl_sendmsg(msg);
}

/* Chapter 8.5.6 */
int rsl_ericsson_imm_assign_cmd(struct gsm_bts *bts, uint32_t tlli, uint8_t len, uint8_t *val)
{
	struct msgb *msg = rsl_imm_assign_cmd_common(bts, len, val);
	if (!msg)
		return 1;

	/* ericsson can handle a reference at the end of the message which is used in
	 * the confirm message. The confirm message is only sent if the trailer is present */
	msgb_put_u8(msg, RSL_IE_ERIC_MOBILE_ID);
	msgb_put_u32(msg, tlli);

	return abis_rsl_sendmsg(msg);
}

/* Send Siemens specific MS RF Power Capability Indication */
int rsl_siemens_mrpci(struct gsm_lchan *lchan, struct rsl_mrpci *mrpci)
{
	struct msgb *msg = rsl_msgb_alloc();
	struct abis_rsl_dchan_hdr *dh;

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_SIEMENS_MRPCI);
	dh->c.msg_discr = ABIS_RSL_MDISC_DED_CHAN;
	dh->chan_nr = gsm_lchan2chan_nr(lchan);
	msgb_tv_put(msg, RSL_IE_SIEMENS_MRPCI, *(uint8_t *)mrpci);

	DEBUGP(DRSL, "%s TX Siemens MRPCI 0x%02x\n",
		gsm_lchan_name(lchan), *(uint8_t *)mrpci);

	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}


/* Send "DATA REQUEST" message with given L3 Info payload */
/* Chapter 8.3.1 */
int rsl_data_request(struct msgb *msg, uint8_t link_id)
{
	if (msg->lchan == NULL) {
		LOGP(DRSL, LOGL_ERROR, "cannot send DATA REQUEST to unknown lchan\n");
		return -EINVAL;
	}

	rsl_rll_push_l3(msg, RSL_MT_DATA_REQ, gsm_lchan2chan_nr(msg->lchan),
			link_id, 1);

	msg->dst = msg->lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

/* Send "ESTABLISH REQUEST" message with given L3 Info payload */
/* Chapter 8.3.1 */
int rsl_establish_request(struct gsm_lchan *lchan, uint8_t link_id)
{
	struct msgb *msg;

	msg = rsl_rll_simple(RSL_MT_EST_REQ, gsm_lchan2chan_nr(lchan),
			     link_id, 0);
	msg->dst = lchan->ts->trx->rsl_link;

	DEBUGP(DRLL, "%s RSL RLL ESTABLISH REQ (link_id=0x%02x)\n",
		gsm_lchan_name(lchan), link_id);

	return abis_rsl_sendmsg(msg);
}

/* Chapter 8.3.7 Request the release of multiframe mode of RLL connection.
   This is what higher layers should call.  The BTS then responds with
   RELEASE CONFIRM, which we in turn use to trigger RSL CHANNEL RELEASE,
   which in turn is acknowledged by RSL CHANNEL RELEASE ACK, which calls
   lchan_free() */
int rsl_release_request(struct gsm_lchan *lchan, uint8_t link_id,
			enum rsl_rel_mode release_mode)
{

	struct msgb *msg;

	msg = rsl_rll_simple(RSL_MT_REL_REQ, gsm_lchan2chan_nr(lchan),
			     link_id, 0);
	/* 0 is normal release, 1 is local end */
	msgb_tv_put(msg, RSL_IE_RELEASE_MODE, release_mode);

	msg->dst = lchan->ts->trx->rsl_link;

	DEBUGP(DRLL, "%s RSL RLL RELEASE REQ (link_id=0x%02x, reason=%u)\n",
		gsm_lchan_name(lchan), link_id, release_mode);

	abis_rsl_sendmsg(msg);

	return 0;
}

static bool msg_for_osmocom_dyn_ts(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *rslh = msgb_l2(msg);
	if (msg->lchan->ts->pchan_on_init != GSM_PCHAN_TCH_F_TCH_H_PDCH)
		return false;
	/* dyn TS messages always come in on the first lchan of a timeslot */
	if (msg->lchan->nr != 0)
		return false;
	return (rslh->chan_nr & RSL_CHAN_OSMO_PDCH) == RSL_CHAN_OSMO_PDCH;
}

/* Chapter 8.4.3: Channel Activate NACK */
static int rsl_rx_chan_act_nack(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	struct tlv_parsed tp;
	struct gsm_lchan *lchan = msg->lchan;
	const uint8_t *cause_p;

	rate_ctr_inc(&msg->lchan->ts->trx->bts->bts_ctrs->ctr[BTS_CTR_CHAN_ACT_NACK]);

	if (dh->ie_chan != RSL_IE_CHAN_NR) {
		LOG_LCHAN(msg->lchan, LOGL_ERROR, "Invalid IE: expected CHAN_NR IE (0x%x), got 0x%x",
			  RSL_IE_CHAN_NR, dh->ie_chan);
		return -EINVAL;
	}

	rsl_tlv_parse(&tp, dh->data, msgb_l2len(msg)-sizeof(*dh));
	LOG_LCHAN(lchan, LOGL_ERROR, "CHANNEL ACTIVATE NACK%s", rsl_cause_name(&tp, &cause_p));

	if (msg_for_osmocom_dyn_ts(msg))
		osmo_fsm_inst_dispatch(lchan->ts->fi, TS_EV_PDCH_ACT_NACK, (void*)cause_p);
	else
		osmo_fsm_inst_dispatch(lchan->fi, LCHAN_EV_RSL_CHAN_ACTIV_NACK, (void*)cause_p);
	return 0;
}

/* Chapter 8.4.4: Connection Failure Indication */
static int rsl_rx_conn_fail(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	struct gsm_lchan *lchan = msg->lchan;
	struct tlv_parsed tp;
	const uint8_t *cause_p;

	rsl_tlv_parse(&tp, dh->data, msgb_l2len(msg)-sizeof(*dh));

	LOG_LCHAN(lchan, LOGL_ERROR, "CONNECTION FAIL%s", rsl_cause_name(&tp, &cause_p));

	rate_ctr_inc(&lchan->ts->trx->bts->bts_ctrs->ctr[BTS_CTR_CHAN_RF_FAIL]);

	/* If the lchan is associated with a conn, we shall notify the MSC of the RSL Conn Failure, and
	 * the connection will presumably be torn down and lead to an lchan release. During initial
	 * Channel Request from the MS, an lchan has no conn yet, so in that case release now. */
	if (!lchan->conn) {
		lchan_release(lchan, false, true, *cause_p);
	} else
		osmo_fsm_inst_dispatch(lchan->conn->fi, GSCON_EV_RSL_CONN_FAIL, (void*)cause_p);

	return 0;
}

static void print_meas_rep_uni(struct gsm_meas_rep_unidir *mru,
				const char *prefix)
{
	DEBUGPC(DMEAS, "RXL-FULL-%s=%3ddBm RXL-SUB-%s=%3ddBm ",
		prefix, rxlev2dbm(mru->full.rx_lev),
		prefix, rxlev2dbm(mru->sub.rx_lev));
	DEBUGPC(DMEAS, "RXQ-FULL-%s=%d RXQ-SUB-%s=%d ",
		prefix, mru->full.rx_qual, prefix, mru->sub.rx_qual);
}

static void print_meas_rep(struct gsm_lchan *lchan, struct gsm_meas_rep *mr)
{
	int i;
	const char *name = "";

	if (lchan && lchan->conn) {
		if (lchan->conn->bsub)
			name = bsc_subscr_name(lchan->conn->bsub);
		else
			name = lchan->name;
	}

	DEBUGP(DMEAS, "[%s] MEASUREMENT RESULT NR=%d ", name, mr->nr);

	if (mr->flags & MEAS_REP_F_DL_DTX)
		DEBUGPC(DMEAS, "DTXd ");

	print_meas_rep_uni(&mr->ul, "ul");
	DEBUGPC(DMEAS, "BS_POWER=%d ", mr->bs_power);

	if (mr->flags & MEAS_REP_F_MS_TO)
		DEBUGPC(DMEAS, "MS_TO=%d ", mr->ms_timing_offset);

	if (mr->flags & MEAS_REP_F_MS_L1) {
		DEBUGPC(DMEAS, "L1_MS_PWR=%3ddBm ", mr->ms_l1.pwr);
		DEBUGPC(DMEAS, "L1_FPC=%u ",
			mr->flags & MEAS_REP_F_FPC ? 1 : 0);
		DEBUGPC(DMEAS, "L1_TA=%u ", mr->ms_l1.ta);
	}

	if (mr->flags & MEAS_REP_F_UL_DTX)
		DEBUGPC(DMEAS, "DTXu ");
	if (mr->flags & MEAS_REP_F_BA1)
		DEBUGPC(DMEAS, "BA1 ");
	if (!(mr->flags & MEAS_REP_F_DL_VALID))
		DEBUGPC(DMEAS, "NOT VALID ");
	else
		print_meas_rep_uni(&mr->dl, "dl");

	DEBUGPC(DMEAS, "NUM_NEIGH=%u\n", mr->num_cell);
	if (mr->num_cell == 7)
		return;
	for (i = 0; i < mr->num_cell; i++) {
		struct gsm_meas_rep_cell *mrc = &mr->cell[i];
		DEBUGP(DMEAS, "IDX=%u ARFCN=%u BSIC=%u => %d dBm\n",
			mrc->neigh_idx, mrc->arfcn, mrc->bsic, rxlev2dbm(mrc->rxlev));
	}
}

static struct gsm_meas_rep *lchan_next_meas_rep(struct gsm_lchan *lchan)
{
	struct gsm_meas_rep *meas_rep;

	meas_rep = &lchan->meas_rep[lchan->meas_rep_idx];
	memset(meas_rep, 0, sizeof(*meas_rep));
	meas_rep->lchan = lchan;
	lchan->meas_rep_idx = (lchan->meas_rep_idx + 1)
					% ARRAY_SIZE(lchan->meas_rep);

	return meas_rep;
}

static int rsl_rx_meas_res(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	struct tlv_parsed tp;
	struct gsm_meas_rep *mr = lchan_next_meas_rep(msg->lchan);
	uint8_t len;
	const uint8_t *val;
	int rc;

	if (!lchan_may_receive_data(msg->lchan)) {
		LOG_LCHAN(msg->lchan, LOGL_DEBUG, "MEAS RES for inactive channel");
		return 0;
	}

	memset(mr, 0, sizeof(*mr));
	mr->lchan = msg->lchan;

	rsl_tlv_parse(&tp, dh->data, msgb_l2len(msg)-sizeof(*dh));

	if (!TLVP_PRESENT(&tp, RSL_IE_MEAS_RES_NR) ||
	    !TLVP_PRESENT(&tp, RSL_IE_UPLINK_MEAS) ||
	    !TLVP_PRESENT(&tp, RSL_IE_BS_POWER)) {
		LOGP(DRSL, LOGL_ERROR, "%s Measurement Report lacks mandatory IEs\n",
		     gsm_lchan_name(mr->lchan));
		return -EIO;
	}

	/* Mandatory Parts */
	mr->nr = *TLVP_VAL(&tp, RSL_IE_MEAS_RES_NR);

	len = TLVP_LEN(&tp, RSL_IE_UPLINK_MEAS);
	val = TLVP_VAL(&tp, RSL_IE_UPLINK_MEAS);
	if (len >= 3) {
		if (val[0] & 0x40)
			mr->flags |= MEAS_REP_F_DL_DTX;
		mr->ul.full.rx_lev = val[0] & 0x3f;
		mr->ul.sub.rx_lev = val[1] & 0x3f;
		mr->ul.full.rx_qual = val[2]>>3 & 0x7;
		mr->ul.sub.rx_qual = val[2] & 0x7;
	}

	mr->bs_power = *TLVP_VAL(&tp, RSL_IE_BS_POWER);

	/* Optional Parts */
	if (TLVP_PRESENT(&tp, RSL_IE_MS_TIMING_OFFSET)) {
		/* According to 3GPP TS 48.058 § MS Timing Offset = Timing Offset field - 63 */
		mr->ms_timing_offset = *TLVP_VAL(&tp, RSL_IE_MS_TIMING_OFFSET) - 63;
		mr->flags |= MEAS_REP_F_MS_TO;
	}

	if (TLVP_PRESENT(&tp, RSL_IE_L1_INFO)) {
		struct e1inp_sign_link *sign_link = msg->dst;

		val = TLVP_VAL(&tp, RSL_IE_L1_INFO);
		mr->flags |= MEAS_REP_F_MS_L1;
		mr->ms_l1.pwr = ms_pwr_dbm(sign_link->trx->bts->band, val[0] >> 3);
		if (val[0] & 0x04)
			mr->flags |= MEAS_REP_F_FPC;
		mr->ms_l1.ta = val[1];
		/* BS11 and Nokia reports TA shifted by 2 bits */
		if (msg->lchan->ts->trx->bts->type == GSM_BTS_TYPE_BS11
		 || msg->lchan->ts->trx->bts->type == GSM_BTS_TYPE_NOKIA_SITE)
			mr->ms_l1.ta >>= 2;
		/* store TA for next assignment/handover */
		mr->lchan->rqd_ta = mr->ms_l1.ta;
	}
	if (TLVP_PRESENT(&tp, RSL_IE_L3_INFO)) {
		msg->l3h = (uint8_t *) TLVP_VAL(&tp, RSL_IE_L3_INFO);
		rc = gsm48_parse_meas_rep(mr, msg);
		if (rc < 0)
			return rc;
	}

	mr->lchan->meas_rep_count++;
	mr->lchan->meas_rep_last_seen_nr = mr->nr;
	LOGP(DRSL, LOGL_DEBUG, "%s: meas_rep_count++=%d meas_rep_last_seen_nr=%u\n",
	     gsm_lchan_name(mr->lchan), mr->lchan->meas_rep_count, mr->lchan->meas_rep_last_seen_nr);

	print_meas_rep(msg->lchan, mr);

	send_lchan_signal(S_LCHAN_MEAS_REP, msg->lchan, mr);

	return 0;
}

/* Chapter 8.4.7 */
static int rsl_rx_hando_det(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	struct tlv_parsed tp;
	struct handover_rr_detect_data d = {
		.msg = msg,
	};

	rsl_tlv_parse(&tp, dh->data, msgb_l2len(msg)-sizeof(*dh));

	if (TLVP_PRESENT(&tp, RSL_IE_ACCESS_DELAY))
		d.access_delay = TLVP_VAL(&tp, RSL_IE_ACCESS_DELAY);

	if (!msg->lchan->conn || !msg->lchan->conn->ho.fi) {
		LOGP(DRSL, LOGL_ERROR, "%s HANDOVER DETECT but no handover is ongoing\n",
		     gsm_lchan_name(msg->lchan));
		return 0;
	}

	osmo_fsm_inst_dispatch(msg->lchan->conn->ho.fi, HO_EV_RR_HO_DETECT, &d);

	return 0;
}

static int rsl_rx_ipacc_pdch(struct msgb *msg, char *name, uint32_t ts_ev)
{
	struct gsm_bts_trx_ts *ts = msg->lchan->ts;

	if (ts->pchan_on_init != GSM_PCHAN_TCH_F_PDCH) {
		LOG_TS(ts, LOGL_ERROR, "Rx RSL ip.access PDCH %s acceptable only for %s",
		       name, gsm_pchan_name(GSM_PCHAN_TCH_F_PDCH));
		return -EINVAL;
	}

	osmo_fsm_inst_dispatch(ts->fi, ts_ev, NULL);
	return 0;
}

static int abis_rsl_rx_dchan(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *rslh = msgb_l2(msg);
	int rc = 0;
	struct e1inp_sign_link *sign_link = msg->dst;

	if (rslh->ie_chan != RSL_IE_CHAN_NR) {
		LOGP(DRSL, LOGL_ERROR,
		     "Rx RSL DCHAN: invalid RSL header, expecting Channel Number IE tag, got 0x%x\n",
		     rslh->ie_chan);
		return -EINVAL;
	}

	msg->lchan = lchan_lookup(sign_link->trx, rslh->chan_nr,
				  "Abis RSL rx DCHAN: ");
	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR,
		     "Rx RSL DCHAN: unable to match RSL message to an lchan: chan_nr=0x%x\n",
		     rslh->chan_nr);
		return -EINVAL;
	}

	LOG_LCHAN(msg->lchan, LOGL_DEBUG, "Rx %s", rsl_or_ipac_msg_name(rslh->c.msg_type));

	if (!msg->lchan->fi) {
		LOG_LCHAN(msg->lchan, LOGL_ERROR, "Rx RSL DCHAN: RSL message for unconfigured lchan");
		return -EINVAL;
	}

	switch (rslh->c.msg_type) {
	case RSL_MT_CHAN_ACTIV_ACK:
		if (msg_for_osmocom_dyn_ts(msg))
			osmo_fsm_inst_dispatch(msg->lchan->ts->fi, TS_EV_PDCH_ACT_ACK, NULL);
		else {
			osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_RSL_CHAN_ACTIV_ACK, NULL);
			count_codecs(sign_link->trx->bts, msg->lchan);
		}
		break;
	case RSL_MT_CHAN_ACTIV_NACK:
		rc = rsl_rx_chan_act_nack(msg);
		break;
	case RSL_MT_CONN_FAIL:
		rc = rsl_rx_conn_fail(msg);
		break;
	case RSL_MT_MEAS_RES:
		rc = rsl_rx_meas_res(msg);
		break;
	case RSL_MT_HANDO_DET:
		rc = rsl_rx_hando_det(msg);
		break;
	case RSL_MT_RF_CHAN_REL_ACK:
		if (msg_for_osmocom_dyn_ts(msg))
			osmo_fsm_inst_dispatch(msg->lchan->ts->fi, TS_EV_PDCH_DEACT_ACK, NULL);
		else
			osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_RSL_RF_CHAN_REL_ACK, NULL);
		break;
	case RSL_MT_MODE_MODIFY_ACK:
		LOG_LCHAN(msg->lchan, LOGL_DEBUG, "CHANNEL MODE MODIFY ACK");
		count_codecs(sign_link->trx->bts, msg->lchan);
		break;
	case RSL_MT_MODE_MODIFY_NACK:
		LOG_LCHAN(msg->lchan, LOGL_DEBUG, "CHANNEL MODE MODIFY NACK");
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_MODE_MODIFY_NACK]);
		break;
	case RSL_MT_IPAC_PDCH_ACT_ACK:
		rc = rsl_rx_ipacc_pdch(msg, "ACT ACK", TS_EV_PDCH_ACT_ACK);
		break;
	case RSL_MT_IPAC_PDCH_ACT_NACK:
		rc = rsl_rx_ipacc_pdch(msg, "ACT NACK", TS_EV_PDCH_ACT_NACK);
		break;
	case RSL_MT_IPAC_PDCH_DEACT_ACK:
		rc = rsl_rx_ipacc_pdch(msg, "DEACT ACK", TS_EV_PDCH_DEACT_ACK);
		break;
	case RSL_MT_IPAC_PDCH_DEACT_NACK:
		rc = rsl_rx_ipacc_pdch(msg, "DEACT NACK", TS_EV_PDCH_DEACT_NACK);
		break;
	case RSL_MT_PHY_CONTEXT_CONF:
	case RSL_MT_PREPROC_MEAS_RES:
	case RSL_MT_TALKER_DET:
	case RSL_MT_LISTENER_DET:
	case RSL_MT_REMOTE_CODEC_CONF_REP:
	case RSL_MT_MR_CODEC_MOD_ACK:
	case RSL_MT_MR_CODEC_MOD_NACK:
	case RSL_MT_MR_CODEC_MOD_PER:
		LOG_LCHAN(msg->lchan, LOGL_NOTICE, "Unimplemented Abis RSL DChan msg 0x%02x\n",
			  rslh->c.msg_type);
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_UNKNOWN]);
		break;
	default:
		LOG_LCHAN(msg->lchan, LOGL_NOTICE, "Unknown Abis RSL DChan msg 0x%02x\n",
			  rslh->c.msg_type);
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_UNKNOWN]);
		return -EINVAL;
	}

	return rc;
}

static int rsl_rx_error_rep(struct msgb *msg)
{
	struct abis_rsl_common_hdr *rslh = msgb_l2(msg);
	struct tlv_parsed tp;
	struct e1inp_sign_link *sign_link = msg->dst;

	rsl_tlv_parse(&tp, rslh->data, msgb_l2len(msg)-sizeof(*rslh));

	LOGP(DRSL, LOGL_ERROR, "%s ERROR REPORT%s\n",
	     gsm_trx_name(sign_link->trx), rsl_cause_name(&tp, NULL));

	return 0;
}

static int abis_rsl_rx_trx(struct msgb *msg)
{
	struct abis_rsl_common_hdr *rslh = msgb_l2(msg);
	struct e1inp_sign_link *sign_link = msg->dst;
	int rc = 0;

	switch (rslh->msg_type) {
	case RSL_MT_ERROR_REPORT:
		rc = rsl_rx_error_rep(msg);
		break;
	case RSL_MT_RF_RES_IND:
		/* interference on idle channels of TRX */
		//DEBUGP(DRSL, "%s RF Resource Indication\n", gsm_trx_name(sign_link->trx));
		break;
	case RSL_MT_OVERLOAD:
		/* indicate CCCH / ACCH / processor overload */
		LOGP(DRSL, LOGL_ERROR, "%s CCCH/ACCH/CPU Overload\n",
		     gsm_trx_name(sign_link->trx));
		break;
	case 0x42: /* Nokia specific: SI End ACK */
		LOGP(DRSL, LOGL_INFO, "Nokia SI End ACK\n");
		break;
	case 0x43: /* Nokia specific: SI End NACK */
		LOGP(DRSL, LOGL_INFO, "Nokia SI End NACK\n");
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "%s Unknown Abis RSL TRX message "
			"type 0x%02x\n", gsm_trx_name(sign_link->trx), rslh->msg_type);
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_UNKNOWN]);
		return -EINVAL;
	}
	return rc;
}

/* Format an IMM ASS REJ according to 04.08 Chapter 9.1.20 */
static int rsl_send_imm_ass_rej(struct gsm_bts *bts,
				struct gsm48_req_ref *rqd_ref,
				uint8_t wait_ind)
{
	uint8_t buf[GSM_MACBLOCK_LEN];
	struct gsm48_imm_ass_rej *iar = (struct gsm48_imm_ass_rej *)buf;

	/* create IMMEDIATE ASSIGN REJECT 04.08 message */
	memset(iar, 0, sizeof(*iar));
	iar->proto_discr = GSM48_PDISC_RR;
	iar->msg_type = GSM48_MT_RR_IMM_ASS_REJ;
	iar->page_mode = GSM48_PM_SAME;

	/*
	 * Set all request references and wait indications to the same value.
	 * 3GPP TS 44.018 v4.5.0 release 4 (section 9.1.20.2) requires that
	 * we duplicate reference and wait indication to fill the message.
	 * The BTS will aggregate up to 4 of our ASS REJ messages if possible.
	 */
	memcpy(&iar->req_ref1, rqd_ref, sizeof(iar->req_ref1));
	iar->wait_ind1 = wait_ind;
	memcpy(&iar->req_ref2, rqd_ref, sizeof(iar->req_ref2));
	iar->wait_ind2 = wait_ind;
	memcpy(&iar->req_ref3, rqd_ref, sizeof(iar->req_ref3));
	iar->wait_ind3 = wait_ind;
	memcpy(&iar->req_ref4, rqd_ref, sizeof(iar->req_ref4));
	iar->wait_ind4 = wait_ind;

	/* we need to subtract 1 byte from sizeof(*iar) since ia includes the l2_plen field */
	iar->l2_plen = GSM48_LEN2PLEN((sizeof(*iar)-1));

	return rsl_imm_assign_cmd(bts, sizeof(*iar), (uint8_t *) iar);
}

int rsl_tx_imm_ass_rej(struct gsm_bts *bts, struct gsm48_req_ref *rqd_ref)
{
	uint8_t wait_ind;
	wait_ind = bts->T3122;
	if (!wait_ind)
		wait_ind = T_def_get(bts->network->T_defs, 3122, T_S, -1);
	if (!wait_ind)
		wait_ind = GSM_T3122_DEFAULT;
	/* The BTS will gather multiple CHAN RQD and reject up to 4 MS at the same time. */
	return rsl_send_imm_ass_rej(bts, rqd_ref, wait_ind);
}

/* Handle packet channel rach requests */
static int rsl_rx_pchan_rqd(struct msgb *msg, struct gsm_bts *bts)
{
	struct gsm48_req_ref *rqd_ref;
	struct abis_rsl_dchan_hdr *rqd_hdr = msgb_l2(msg);
	rqd_ref = (struct gsm48_req_ref *) &rqd_hdr->data[1];
	uint8_t ra = rqd_ref->ra;
	uint8_t t1, t2, t3;
	uint32_t fn;
	uint8_t rqd_ta;
	uint8_t is_11bit;

	/* Process rach request and forward contained information to PCU */
	if (ra == 0x7F) {
		is_11bit = 1;

		/* FIXME: Also handle 11 bit rach requests */
		LOGP(DRSL, LOGL_ERROR, "BTS %d eleven bit access burst not supported yet!\n",bts->nr);
		return -EINVAL;
	} else {
		is_11bit = 0;
		t1 = rqd_ref->t1;
		t2 = rqd_ref->t2;
		t3 = rqd_ref->t3_low | (rqd_ref->t3_high << 3);
		fn = (51 * ((t3-t2) % 26) + t3 + 51 * 26 * t1);

		rqd_ta = rqd_hdr->data[sizeof(struct gsm48_req_ref)+2];
	}

	return pcu_tx_rach_ind(bts, rqd_ta, ra, fn, is_11bit,
			       GSM_L1_BURST_TYPE_ACCESS_0);
}

/* MS has requested a channel on the RACH */
static int rsl_rx_chan_rqd(struct msgb *msg)
{
	struct e1inp_sign_link *sign_link = msg->dst;
	struct gsm_bts *bts = sign_link->trx->bts;
	struct abis_rsl_dchan_hdr *rqd_hdr = msgb_l2(msg);
	struct gsm48_req_ref *rqd_ref;
	enum gsm_chan_t lctype;
	enum gsm_chreq_reason_t chreq_reason;
	struct gsm_lchan *lchan;
	uint8_t rqd_ta;

	/* parse request reference to be used in immediate assign */
	if (rqd_hdr->data[0] != RSL_IE_REQ_REFERENCE)
		return -EINVAL;

	rqd_ref = (struct gsm48_req_ref *) &rqd_hdr->data[1];

	/* parse access delay and use as TA */
	if (rqd_hdr->data[sizeof(struct gsm48_req_ref)+1] != RSL_IE_ACCESS_DELAY)
		return -EINVAL;
	rqd_ta = rqd_hdr->data[sizeof(struct gsm48_req_ref)+2];

	/* Determine channel request cause code */
	chreq_reason = get_reason_by_chreq(rqd_ref->ra, bts->network->neci);
	LOGP(DRSL, LOGL_NOTICE, "(bts=%d) CHAN RQD: reason: %s (ra=0x%02x, neci=0x%02x, chreq_reason=0x%02x)\n",
	     msg->lchan->ts->trx->bts->nr,
	     get_value_string(gsm_chreq_descs, chreq_reason),
	     rqd_ref->ra, bts->network->neci, chreq_reason);

	/* Handle PDCH related rach requests (in case of BSC-co-located-PCU */
	if (chreq_reason == GSM_CHREQ_REASON_PDCH)
		return rsl_rx_pchan_rqd(msg, bts);

	/* determine channel type (SDCCH/TCH_F/TCH_H) based on
	 * request reference RA */
	lctype = get_ctype_by_chreq(bts->network, rqd_ref->ra);

	rate_ctr_inc(&bts->bts_ctrs->ctr[BTS_CTR_CHREQ_TOTAL]);

	/* check availability / allocate channel
	 *
	 * - First try to allocate SDCCH.
	 * - If SDCCH is not available, try whatever MS requested, if not SDCCH.
	 * - If there is still no channel available, reject channel request.
	 *
	 * lchan_alloc() possibly tries to allocate larger lchans.
	 *
	 * Note: If the MS requests not TCH/H, we don't know if the phone
	 *       supports TCH/H, so we must assign TCH/F or SDCCH.
	 */
	lchan = lchan_select_by_type(bts, GSM_LCHAN_SDCCH);
	if (!lchan && lctype != GSM_LCHAN_SDCCH) {
		LOGP(DRSL, LOGL_NOTICE, "(bts=%d) CHAN RQD: no resources for %s "
			"0x%x, retrying with %s\n",
			msg->lchan->ts->trx->bts->nr,
			gsm_lchant_name(GSM_LCHAN_SDCCH), rqd_ref->ra,
			gsm_lchant_name(lctype));
		lchan = lchan_select_by_type(bts, lctype);
	}
	if (!lchan) {
		LOGP(DRSL, LOGL_NOTICE, "(bts=%d) CHAN RQD: no resources for %s 0x%x\n",
		     msg->lchan->ts->trx->bts->nr, gsm_lchant_name(lctype), rqd_ref->ra);
		rate_ctr_inc(&bts->bts_ctrs->ctr[BTS_CTR_CHREQ_NO_CHANNEL]);
		rsl_tx_imm_ass_rej(bts, rqd_ref);
		return 0;
	}

	/* save the RACH data as we need it after the CHAN ACT ACK */
	lchan->rqd_ref = talloc_zero(bts, struct gsm48_req_ref);
	OSMO_ASSERT(lchan->rqd_ref);

	*(lchan->rqd_ref) = *rqd_ref;
	lchan->rqd_ta = rqd_ta;

	LOG_LCHAN(lchan, LOGL_DEBUG, "MS: Channel Request: reason=%s ra=0x%02x ta=%d\n",
		  gsm_chreq_name(chreq_reason), rqd_ref->ra, rqd_ta);
	{
		struct lchan_activate_info info = {
			.activ_for = FOR_MS_CHANNEL_REQUEST,
			.chan_mode = GSM48_CMODE_SIGN,
		};

		lchan_activate(lchan, &info);
	}
	return 0;
}

int rsl_tx_imm_assignment(struct gsm_lchan *lchan)
{
	struct gsm_bts *bts = lchan->ts->trx->bts;
	uint8_t buf[GSM_MACBLOCK_LEN];
	struct gsm48_imm_ass *ia = (struct gsm48_imm_ass *) buf;

	/* create IMMEDIATE ASSIGN 04.08 messge */
	memset(ia, 0, sizeof(*ia));
	/* we set ia->l2_plen once we know the length of the MA below */
	ia->proto_discr = GSM48_PDISC_RR;
	ia->msg_type = GSM48_MT_RR_IMM_ASS;
	ia->page_mode = GSM48_PM_SAME;
	gsm48_lchan2chan_desc(&ia->chan_desc, lchan);

	/* use request reference extracted from CHAN_RQD */
	memcpy(&ia->req_ref, lchan->rqd_ref, sizeof(ia->req_ref));
	ia->timing_advance = lchan->rqd_ta;
	if (!lchan->ts->hopping.enabled) {
		ia->mob_alloc_len = 0;
	} else {
		ia->mob_alloc_len = lchan->ts->hopping.ma_len;
		memcpy(ia->mob_alloc, lchan->ts->hopping.ma_data, ia->mob_alloc_len);
	}
	/* we need to subtract 1 byte from sizeof(*ia) since ia includes the l2_plen field */
	ia->l2_plen = GSM48_LEN2PLEN((sizeof(*ia)-1) + ia->mob_alloc_len);

	/* send IMMEDIATE ASSIGN CMD on RSL to BTS (to send on CCCH to MS) */
	return rsl_imm_assign_cmd(bts, sizeof(*ia)+ia->mob_alloc_len, (uint8_t *) ia);
}

/* current load on the CCCH */
static int rsl_rx_ccch_load(struct msgb *msg)
{
	struct e1inp_sign_link *sign_link = msg->dst;
	struct abis_rsl_dchan_hdr *rslh = msgb_l2(msg);
	struct ccch_signal_data sd;

	sd.bts = sign_link->trx->bts;
	sd.rach_slot_count = -1;
	sd.rach_busy_count = -1;
	sd.rach_access_count = -1;

	switch (rslh->data[0]) {
	case RSL_IE_PAGING_LOAD:
		sd.pg_buf_space = rslh->data[1] << 8 | rslh->data[2];
		if (is_ipaccess_bts(sign_link->trx->bts) && sd.pg_buf_space == 0xffff) {
			/* paging load below configured threshold, use 50 as default */
			sd.pg_buf_space = 50;
		}
		paging_update_buffer_space(sign_link->trx->bts, sd.pg_buf_space);
		osmo_signal_dispatch(SS_CCCH, S_CCCH_PAGING_LOAD, &sd);
		break;
	case RSL_IE_RACH_LOAD:
		if (msg->data_len >= 7) {
			sd.rach_slot_count = rslh->data[2] << 8 | rslh->data[3];
			sd.rach_busy_count = rslh->data[4] << 8 | rslh->data[5];
			sd.rach_access_count = rslh->data[6] << 8 | rslh->data[7];
			osmo_signal_dispatch(SS_CCCH, S_CCCH_RACH_LOAD, &sd);
		}
		break;
	default:
		break;
	}

	return 0;
}

/* Ericsson specific: Immediate Assign Sent */
static int rsl_rx_ericsson_imm_assign_sent(struct msgb *msg)
{
	struct e1inp_sign_link *sign_link = msg->dst;
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	uint32_t tlli;

	LOGP(DRSL, LOGL_INFO, "IMM.ass sent\n");
	msgb_pull(msg, sizeof(*dh));

	/* FIXME: Move to TLV once we support defining TV types with V having len != 1 byte */
	if(msg->len < 5)
		LOGP(DRSL, LOGL_ERROR, "short IMM.ass sent message!\n");
	else if(msg->data[0] != RSL_IE_ERIC_MOBILE_ID)
		LOGP(DRSL, LOGL_ERROR, "unsupported IMM.ass message format! (please fix)\n");
	else {
		msgb_pull(msg, 1); /* drop previous data to use msg_pull_u32 */
		tlli = msgb_pull_u32(msg);
		pcu_tx_imm_ass_sent(sign_link->trx->bts, tlli);
	}
	return 0;
}

static int abis_rsl_rx_cchan(struct msgb *msg)
{
	struct e1inp_sign_link *sign_link = msg->dst;
	struct abis_rsl_dchan_hdr *rslh = msgb_l2(msg);
	int rc = 0;

	msg->lchan = lchan_lookup(sign_link->trx, rslh->chan_nr,
				  "Abis RSL rx CCHAN: ");

	switch (rslh->c.msg_type) {
	case RSL_MT_CHAN_RQD:
		/* MS has requested a channel on the RACH */
		rc = rsl_rx_chan_rqd(msg);
		break;
	case RSL_MT_CCCH_LOAD_IND:
		/* current load on the CCCH */
		rc = rsl_rx_ccch_load(msg);
		break;
	case RSL_MT_DELETE_IND:
		/* CCCH overloaded, IMM_ASSIGN was dropped */
	case RSL_MT_CBCH_LOAD_IND:
		/* current load on the CBCH */
		LOGP(DRSL, LOGL_NOTICE, "Unimplemented Abis RSL TRX message "
			"type %s\n", rsl_msg_name(rslh->c.msg_type));
		break;
	case RSL_MT_ERICSSON_IMM_ASS_SENT:
		rc = rsl_rx_ericsson_imm_assign_sent(msg);
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "Unknown Abis RSL TRX message type "
			"0x%02x\n", rslh->c.msg_type);
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_UNKNOWN]);
		return -EINVAL;
	}

	return rc;
}

static int rsl_rx_rll_err_ind(struct msgb *msg)
{
	struct tlv_parsed tp;
	struct abis_rsl_rll_hdr *rllh = msgb_l2(msg);
	uint8_t rlm_cause;

	rsl_tlv_parse(&tp, rllh->data, msgb_l2len(msg) - sizeof(*rllh));
	if (!TLVP_PRESENT(&tp, RSL_IE_RLM_CAUSE)) {
		LOG_LCHAN(msg->lchan, LOGL_ERROR, "ERROR INDICATION without mandantory cause.");
		return -1;
	}

	rlm_cause = *TLVP_VAL(&tp, RSL_IE_RLM_CAUSE);
	LOG_LCHAN(msg->lchan, LOGL_ERROR, "ERROR INDICATION cause=%s\n", rsl_rlm_cause_name(rlm_cause));

	rll_indication(msg->lchan, rllh->link_id, BSC_RLLR_IND_ERR_IND);

	rate_ctr_inc(&msg->lchan->ts->trx->bts->bts_ctrs->ctr[BTS_CTR_CHAN_RLL_ERR]);

	osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_RLL_ERR_IND, &rlm_cause);

	return 0;
}

/*	ESTABLISH INDICATION, LOCATION AREA UPDATE REQUEST
	0x02, 0x06,
	0x01, 0x20,
	0x02, 0x00,
	0x0b, 0x00, 0x0f, 0x05, 0x08, ... */

static int abis_rsl_rx_rll(struct msgb *msg)
{
	struct e1inp_sign_link *sign_link = msg->dst;
	struct abis_rsl_rll_hdr *rllh = msgb_l2(msg);
	int rc = 0;
	uint8_t sapi = rllh->link_id & 0x7;

	msg->lchan = lchan_lookup(sign_link->trx, rllh->chan_nr, "Abis RSL rx RLL: ");

	switch (rllh->c.msg_type) {
	case RSL_MT_DATA_IND:
		LOG_LCHAN(msg->lchan, LOGL_DEBUG, "SAPI=%u DATA INDICATION", sapi);
		if (msgb_l2len(msg) >
		    sizeof(struct abis_rsl_common_hdr) + sizeof(*rllh) &&
		    rllh->data[0] == RSL_IE_L3_INFO) {
			msg->l3h = &rllh->data[3];
			return gsm0408_rcvmsg(msg, rllh->link_id);
		}
		break;
	case RSL_MT_EST_IND:
		LOG_LCHAN(msg->lchan, LOGL_DEBUG, "SAPI=%u ESTABLISH INDICATION", sapi);
		/* lchan is established, stop T3101 */

		/* Note: By definition the first Establish Indication must
		 * happen first on SAPI 0, once the connection on SAPI 0 is
		 * made, parallel connections on other SAPIs are permitted */
		if (sapi != 0 && msg->lchan->sapis[0] != LCHAN_SAPI_MS) {
			LOG_LCHAN(msg->lchan, LOGL_NOTICE,
				  "MS attempted to establish DCCH on SAPI=%d (expected SAPI=0)\n",
				  sapi);
			/* Note: We do not need to close the channel,
			 * since we might still get a proper Establish Ind.
			 * If not, T3101 will close the channel on timeout. */
			break;
		}

		/* Note: Check for MF SACCH on SAPI=0 (not permitted). By
		 * definition we establish a link in multiframe (MF) mode.
		 * (see also 3GPP TS 48.058, chapter 3.1. However, on SAPI=0
		 * SACCH is only allowed in UL mode, not in MF mode.
		 * (see also 3GPP TS 44.005, figure 5) So we have to drop such
		 * Establish Indications */
		if (sapi == 0 && (rllh->link_id >> 6 & 0x03) == 1) {
			LOG_LCHAN(msg->lchan, LOGL_NOTICE,
				  "MS attempted to establish an SACCH in MF mode on SAPI=0 (not permitted)\n");

			/* Note: We do not need to close the channel,
			 * since we might still get a proper Establish Ind.
			 * If not, T3101 will close the channel on timeout. */
			break;
		}

		msg->lchan->sapis[sapi] = LCHAN_SAPI_MS;
		osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_RLL_ESTABLISH_IND, msg);

		if (msgb_l2len(msg) >
		    sizeof(struct abis_rsl_common_hdr) + sizeof(*rllh) &&
		    rllh->data[0] == RSL_IE_L3_INFO) {
			msg->l3h = &rllh->data[3];
			return gsm0408_rcvmsg(msg, rllh->link_id);
		}
		break;
	case RSL_MT_EST_CONF:
		LOG_LCHAN(msg->lchan, LOGL_ERROR, "SAPI=%u ESTABLISH CONFIRM\n", sapi);
		msg->lchan->sapis[sapi] = LCHAN_SAPI_NET;
		rll_indication(msg->lchan, rllh->link_id,
				  BSC_RLLR_IND_EST_CONF);
		break;
	case RSL_MT_REL_IND:
		/* BTS informs us of having received  DISC from MS */
		osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_RLL_REL_IND, &rllh->link_id);
		break;
	case RSL_MT_REL_CONF:
		/* BTS informs us of having received UA from MS,
		 * in response to DISC that we've sent earlier */
		osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_RLL_REL_CONF, &rllh->link_id);
		break;
	case RSL_MT_ERROR_IND:
		LOG_LCHAN(msg->lchan, LOGL_DEBUG, "SAPI=%u ERROR INDICATION\n", sapi);
		rc = rsl_rx_rll_err_ind(msg);
		break;
	case RSL_MT_UNIT_DATA_IND:
		LOG_LCHAN(msg->lchan, LOGL_NOTICE, "SAPI=%u UNIT DATA INDICATION:"
			  " unimplemented Abis RLL message type 0x%02x\n", sapi, rllh->c.msg_type);
		break;
	default:
		LOG_LCHAN(msg->lchan, LOGL_NOTICE, "SAPI=%u Unknown Abis RLL message type 0x%02x\n",
			  sapi, rllh->c.msg_type);
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_UNKNOWN]);
	}
	return rc;
}

/* Return an ip.access BTS speech mode value (uint8_t) or negative on error. */
int ipacc_speech_mode(enum gsm48_chan_mode tch_mode, enum gsm_chan_t type)
{
	switch (tch_mode) {
	case GSM48_CMODE_SPEECH_V1:
		switch (type) {
		case GSM_LCHAN_TCH_F:
			return 0x00;
		case GSM_LCHAN_TCH_H:
			return 0x03;
		default:
			break;
		}
		break;
	case GSM48_CMODE_SPEECH_EFR:
		switch (type) {
		case GSM_LCHAN_TCH_F:
			return 0x01;
		/* there's no half-rate EFR */
		default:
			break;
		}
		break;
	case GSM48_CMODE_SPEECH_AMR:
		switch (type) {
		case GSM_LCHAN_TCH_F:
			return 0x02;
		case GSM_LCHAN_TCH_H:
			return 0x05;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return -EINVAL;
}

void ipacc_speech_mode_set_direction(uint8_t *speech_mode, bool send)
{
	const uint8_t recv_only_flag = 0x10;
	if (send)
		*speech_mode = *speech_mode & ~recv_only_flag;
	else
		*speech_mode = *speech_mode | recv_only_flag;
}

/* Return an ip.access BTS payload type value (uint8_t) or negative on error. */
int ipacc_payload_type(enum gsm48_chan_mode tch_mode, enum gsm_chan_t type)
{
	switch (tch_mode) {
	case GSM48_CMODE_SPEECH_V1:
		switch (type) {
		case GSM_LCHAN_TCH_F:
			return RTP_PT_GSM_FULL;
		case GSM_LCHAN_TCH_H:
			return RTP_PT_GSM_HALF;
		default:
			break;
		}
		break;
	case GSM48_CMODE_SPEECH_EFR:
		switch (type) {
		case GSM_LCHAN_TCH_F:
			return RTP_PT_GSM_EFR;
		/* there's no half-rate EFR */
		default:
			break;
		}
		break;
	case GSM48_CMODE_SPEECH_AMR:
		switch (type) {
		case GSM_LCHAN_TCH_F:
		case GSM_LCHAN_TCH_H:
			return RTP_PT_AMR;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return -EINVAL;
}

static const char *ip_to_a(uint32_t ip)
{
	struct in_addr ia;
	ia.s_addr = htonl(ip);
	return inet_ntoa(ia);
}

/* ip.access specific RSL extensions */
static void ipac_parse_rtp(struct gsm_lchan *lchan, struct tlv_parsed *tv, const char *label)
{
	struct in_addr ip;
	uint16_t port, conn_id;

	if (TLVP_PRESENT(tv, RSL_IE_IPAC_LOCAL_IP)) {
		ip.s_addr = tlvp_val32_unal(tv, RSL_IE_IPAC_LOCAL_IP);
		lchan->abis_ip.bound_ip = ntohl(ip.s_addr);
	}

	if (TLVP_PRESENT(tv, RSL_IE_IPAC_LOCAL_PORT)) {
		port = tlvp_val16_unal(tv, RSL_IE_IPAC_LOCAL_PORT);
		port = ntohs(port);
		lchan->abis_ip.bound_port = port;
	}

	if (TLVP_PRESENT(tv, RSL_IE_IPAC_CONN_ID)) {
		conn_id = tlvp_val16_unal(tv, RSL_IE_IPAC_CONN_ID);
		conn_id = ntohs(conn_id);
		lchan->abis_ip.conn_id = conn_id;
	}

	if (TLVP_PRESENT(tv, RSL_IE_IPAC_RTP_PAYLOAD2)) {
		lchan->abis_ip.rtp_payload2 =
				*TLVP_VAL(tv, RSL_IE_IPAC_RTP_PAYLOAD2);
	}

	if (TLVP_PRESENT(tv, RSL_IE_IPAC_SPEECH_MODE)) {
		lchan->abis_ip.speech_mode =
				*TLVP_VAL(tv, RSL_IE_IPAC_SPEECH_MODE);
	}

	/* Why would we receive the MGW IP and port back from the BTS, and why would we care?? */
	if (TLVP_PRESENT(tv, RSL_IE_IPAC_REMOTE_IP)) {
		ip.s_addr = tlvp_val32_unal(tv, RSL_IE_IPAC_REMOTE_IP);
		lchan->abis_ip.connect_ip = ntohl(ip.s_addr);
	}
	if (TLVP_PRESENT(tv, RSL_IE_IPAC_REMOTE_PORT)) {
		port = tlvp_val16_unal(tv, RSL_IE_IPAC_REMOTE_PORT);
		port = ntohs(port);
		lchan->abis_ip.connect_port = port;
	}

	LOG_LCHAN(lchan, LOGL_DEBUG, "Rx IPACC %s ACK:"
		  " BTS=%s:%u conn_id=%u rtp_payload2=0x%02x speech_mode=0x%02x",
		  label, ip_to_a(lchan->abis_ip.bound_ip), lchan->abis_ip.bound_port,
		  lchan->abis_ip.conn_id, lchan->abis_ip.rtp_payload2, lchan->abis_ip.speech_mode);
}

/*! Send Issue IPA RSL CRCX to configure the RTP port of the BTS.
 * \param[in] lchan Logical Channel for which we issue CRCX
 */
int rsl_tx_ipacc_crcx(struct gsm_lchan *lchan)
{
	struct msgb *msg = rsl_msgb_alloc();
	struct abis_rsl_dchan_hdr *dh;

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_IPAC_CRCX);
	dh->c.msg_discr = ABIS_RSL_MDISC_IPACCESS;
	dh->chan_nr = gsm_lchan2chan_nr(lchan);

	/* 0x1- == receive-only, 0x-1 == EFR codec */
	msgb_tv_put(msg, RSL_IE_IPAC_SPEECH_MODE, lchan->abis_ip.speech_mode);
	msgb_tv_put(msg, RSL_IE_IPAC_RTP_PAYLOAD, lchan->abis_ip.rtp_payload);

	LOG_LCHAN(lchan, LOGL_DEBUG, "Sending IPACC CRCX to BTS: speech_mode=0x%02x RTP_PAYLOAD=%d\n",
		  lchan->abis_ip.speech_mode, lchan->abis_ip.rtp_payload);

	msg->dst = lchan->ts->trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

/*! Send IPA RSL MDCX to configure the RTP port the BTS sends to (MGW).
 * \param[in] lchan Logical Channel for which we issue MDCX
 * Remote (MGW) IP address, port and payload types for RTP are determined from lchan->abis_ip.
 */
int rsl_tx_ipacc_mdcx(struct gsm_lchan *lchan)
{
	struct msgb *msg = rsl_msgb_alloc();
	struct abis_rsl_dchan_hdr *dh;
	uint32_t *att_ip;

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_IPAC_MDCX);
	dh->c.msg_discr = ABIS_RSL_MDISC_IPACCESS;
	dh->chan_nr = gsm_lchan2chan_nr(lchan);

	msgb_tv16_put(msg, RSL_IE_IPAC_CONN_ID, lchan->abis_ip.conn_id);
	msgb_v_put(msg, RSL_IE_IPAC_REMOTE_IP);
	att_ip = (uint32_t *)msgb_put(msg, sizeof(uint32_t));
	*att_ip = htonl(lchan->abis_ip.connect_ip);
	msgb_tv16_put(msg, RSL_IE_IPAC_REMOTE_PORT, lchan->abis_ip.connect_port);
	msgb_tv_put(msg, RSL_IE_IPAC_SPEECH_MODE, lchan->abis_ip.speech_mode);
	msgb_tv_put(msg, RSL_IE_IPAC_RTP_PAYLOAD, lchan->abis_ip.rtp_payload);
	if (lchan->abis_ip.rtp_payload2)
		msgb_tv_put(msg, RSL_IE_IPAC_RTP_PAYLOAD2, lchan->abis_ip.rtp_payload2);

	msg->dst = lchan->ts->trx->rsl_link;

	LOG_LCHAN(lchan, LOGL_DEBUG, "Sending IPACC MDCX to BTS:"
		  " %s:%u rtp_payload=%u rtp_payload2=%u conn_id=%u speech_mode=0x%02x",
		  ip_to_a(lchan->abis_ip.connect_ip),
		  lchan->abis_ip.connect_port,
		  lchan->abis_ip.rtp_payload,
		  lchan->abis_ip.rtp_payload2,
		  lchan->abis_ip.conn_id,
		  lchan->abis_ip.speech_mode);

	return abis_rsl_sendmsg(msg);
}

static int abis_rsl_rx_ipacc_crcx_ack(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	struct tlv_parsed tv;
	struct gsm_lchan *lchan = msg->lchan;

	/* the BTS has acknowledged a local bind, it now tells us the IP
	* address and port number to which it has bound the given logical
	* channel */

	rsl_tlv_parse(&tv, dh->data, msgb_l2len(msg)-sizeof(*dh));
	if (!TLVP_PRESENT(&tv, RSL_IE_IPAC_LOCAL_PORT) ||
	    !TLVP_PRESENT(&tv, RSL_IE_IPAC_LOCAL_IP) ||
	    !TLVP_PRESENT(&tv, RSL_IE_IPAC_CONN_ID)) {
		LOGP(DRSL, LOGL_NOTICE, "mandatory IE missing");
		return -EINVAL;
	}

	ipac_parse_rtp(lchan, &tv, "CRCX");

	osmo_fsm_inst_dispatch(lchan->fi, LCHAN_EV_IPACC_CRCX_ACK, 0);

	return 0;
}

static int abis_rsl_rx_ipacc_mdcx_ack(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	struct tlv_parsed tv;
	struct gsm_lchan *lchan = msg->lchan;

	/* the BTS has acknowledged a remote connect request and
	 * it now tells us the IP address and port number to which it has
	 * connected the given logical channel */

	rsl_tlv_parse(&tv, dh->data, msgb_l2len(msg)-sizeof(*dh));
	ipac_parse_rtp(lchan, &tv, "MDCX");

	osmo_fsm_inst_dispatch(lchan->fi, LCHAN_EV_IPACC_MDCX_ACK, 0);

	return 0;
}

static int abis_rsl_rx_ipacc_dlcx_ind(struct msgb *msg)
{
	struct abis_rsl_dchan_hdr *dh = msgb_l2(msg);
	struct tlv_parsed tv;

	rsl_tlv_parse(&tv, dh->data, msgb_l2len(msg)-sizeof(*dh));
	LOG_LCHAN(msg->lchan, LOGL_NOTICE, "Rx IPACC DLCX IND%s",
		  rsl_cause_name(&tv, NULL));

	return 0;
}

static int abis_rsl_rx_ipacc(struct msgb *msg)
{
	struct e1inp_sign_link *sign_link = msg->dst;
	struct abis_rsl_rll_hdr *rllh = msgb_l2(msg);
	int rc = 0;

	msg->lchan = lchan_lookup(sign_link->trx, rllh->chan_nr,
				  "Abis RSL rx IPACC: ");

	if (!msg->lchan) {
		LOGP(DRSL, LOGL_ERROR,
		     "Rx RSL IPACC: unable to match RSL message to an lchan: chan_nr=0x%x\n",
		     rllh->chan_nr);
		return -EINVAL;
	}

	if (!msg->lchan->fi) {
		LOG_LCHAN(msg->lchan, LOGL_ERROR, "Rx RSL IPACC: RSL message for unconfigured lchan");
		return -EINVAL;
	}

	LOG_LCHAN(msg->lchan, LOGL_DEBUG, "Rx %s", rsl_or_ipac_msg_name(rllh->c.msg_type));

	switch (rllh->c.msg_type) {
	case RSL_MT_IPAC_CRCX_ACK:
		rc = abis_rsl_rx_ipacc_crcx_ack(msg);
		break;
	case RSL_MT_IPAC_CRCX_NACK:
		/* somehow the BTS was unable to bind the lchan to its local
		 * port?!? */
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_IPA_NACK]);
		osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_IPACC_CRCX_NACK, 0);
		break;
	case RSL_MT_IPAC_MDCX_ACK:
		/* the BTS tells us that a connect operation was successful */
		rc = abis_rsl_rx_ipacc_mdcx_ack(msg);
		break;
	case RSL_MT_IPAC_MDCX_NACK:
		/* somehow the BTS was unable to connect the lchan to a remote
		 * port */
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_IPA_NACK]);
		osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_IPACC_MDCX_NACK, 0);
		break;
	case RSL_MT_IPAC_DLCX_IND:
		rc = abis_rsl_rx_ipacc_dlcx_ind(msg);
		break;
	default:
		LOG_LCHAN(msg->lchan, LOGL_NOTICE, "Unknown ip.access msg_type 0x%02x\n",
			  rllh->c.msg_type);
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_UNKNOWN]);
		break;
	}

	return rc;
}

/*! Tx simplified channel (de-)activation message for non-standard Osmocom dyn TS PDCH type. */
static int send_osmocom_style_pdch_chan_act(struct gsm_bts_trx_ts *ts, bool activate)
{
	struct msgb *msg;
	struct abis_rsl_dchan_hdr *dh;

	msg = rsl_msgb_alloc();
	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, activate ? RSL_MT_CHAN_ACTIV : RSL_MT_RF_CHAN_REL);

	dh->chan_nr = RSL_CHAN_OSMO_PDCH | (ts->nr & ~RSL_CHAN_NR_MASK);

	if (activate) {
		msgb_tv_put(msg, RSL_IE_ACT_TYPE, RSL_ACT_OSMO_PDCH);

		if (ts->trx->bts->type == GSM_BTS_TYPE_RBS2000
		    && ts->trx->bts->rbs2000.use_superchannel) {
			const uint8_t eric_pgsl_tmr[] = { 30, 1 };
			msgb_tv_fixed_put(msg, RSL_IE_ERIC_PGSL_TIMERS,
					  sizeof(eric_pgsl_tmr), eric_pgsl_tmr);
		}
	}

	msg->dst = ts->trx->rsl_link;
	return abis_rsl_sendmsg(msg);
}

/*! Tx simplified channel (de-)activation message for non-standard ip.access dyn TS PDCH type. */
static int send_ipacc_style_pdch_act(struct gsm_bts_trx_ts *ts, bool activate)
{
	struct msgb *msg = rsl_msgb_alloc();
	struct abis_rsl_dchan_hdr *dh;

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(msg, sizeof(*dh));
	init_dchan_hdr(dh, activate ? RSL_MT_IPAC_PDCH_ACT : RSL_MT_IPAC_PDCH_DEACT);
	dh->c.msg_discr = ABIS_RSL_MDISC_DED_CHAN;
	dh->chan_nr = gsm_pchan2chan_nr(GSM_PCHAN_TCH_F, ts->nr, 0);

	msg->dst = ts->trx->rsl_link;
	return abis_rsl_sendmsg(msg);
}

int rsl_tx_dyn_ts_pdch_act_deact(struct gsm_bts_trx_ts *ts, bool activate)
{
	int rc;
	const char *what;
	const char *act;

	switch (ts->pchan_on_init) {
	case GSM_PCHAN_TCH_F_TCH_H_PDCH:
		what = "Osmocom dyn TS";
		act = activate? "PDCH Chan Activ" : "PDCH Chan RF Release";

		rc = send_osmocom_style_pdch_chan_act(ts, activate);
		break;

	case GSM_PCHAN_TCH_F_PDCH:
		what = "ip.access dyn TS";
		act = activate? "PDCH ACT" : "PDCH DEACT";

		rc = send_ipacc_style_pdch_act(ts, activate);
		break;

	default:
		what = "static timeslot";
		act = activate? "dynamic PDCH activation" : "dynamic PDCH deactivation";
		rc = -EINVAL;
		break;
	}

	if (rc)
		LOG_TS(ts, LOGL_ERROR, "Tx FAILED: %s: %s: %d (%s)",
		       what, act, rc, strerror(-rc));
	else
		LOG_TS(ts, LOGL_DEBUG, "Tx: %s: %s", what, act);
	return rc;
}

/* Entry-point where L2 RSL from BTS enters */
int abis_rsl_rcvmsg(struct msgb *msg)
{
	struct e1inp_sign_link *sign_link;
	struct abis_rsl_common_hdr *rslh;
	int rc = 0;

	if (!msg) {
		DEBUGP(DRSL, "Empty RSL msg?..\n");
		return -1;
	}

	if (msgb_l2len(msg) < sizeof(*rslh)) {
		DEBUGP(DRSL, "Truncated RSL message with l2len: %u\n", msgb_l2len(msg));
		msgb_free(msg);
		return -1;
	}

	sign_link = msg->dst;
	rslh = msgb_l2(msg);

	switch (rslh->msg_discr & 0xfe) {
	case ABIS_RSL_MDISC_RLL:
		rc = abis_rsl_rx_rll(msg);
		break;
	case ABIS_RSL_MDISC_DED_CHAN:
		rc = abis_rsl_rx_dchan(msg);
		break;
	case ABIS_RSL_MDISC_COM_CHAN:
		rc = abis_rsl_rx_cchan(msg);
		break;
	case ABIS_RSL_MDISC_TRX:
		rc = abis_rsl_rx_trx(msg);
		break;
	case ABIS_RSL_MDISC_LOC:
		LOGP(DRSL, LOGL_NOTICE, "unimplemented RSL msg disc 0x%02x\n",
			rslh->msg_discr);
		break;
	case ABIS_RSL_MDISC_IPACCESS:
		rc = abis_rsl_rx_ipacc(msg);
		break;
	default:
		LOGP(DRSL, LOGL_NOTICE, "unknown RSL message discriminator "
			"0x%02x\n", rslh->msg_discr);
		rate_ctr_inc(&sign_link->trx->bts->bts_ctrs->ctr[BTS_CTR_RSL_UNKNOWN]);
		rc = -EINVAL;
	}
	msgb_free(msg);
	return rc;
}

int rsl_sms_cb_command(struct gsm_bts *bts, uint8_t chan_number,
		       struct rsl_ie_cb_cmd_type cb_command,
		       const uint8_t *data, int len)
{
	struct abis_rsl_dchan_hdr *dh;
	struct msgb *cb_cmd;

	cb_cmd = rsl_msgb_alloc();
	if (!cb_cmd)
		return -1;

	dh = (struct abis_rsl_dchan_hdr *) msgb_put(cb_cmd, sizeof(*dh));
	init_dchan_hdr(dh, RSL_MT_SMS_BC_CMD);
	dh->c.msg_discr = ABIS_RSL_MDISC_COM_CHAN;
	dh->chan_nr = chan_number; /* TODO: check the chan config */

	msgb_tv_put(cb_cmd, RSL_IE_CB_CMD_TYPE, *(uint8_t*)&cb_command);
	msgb_tlv_put(cb_cmd, RSL_IE_SMSCB_MSG, len, data);

	cb_cmd->dst = bts->c0->rsl_link;

	return abis_rsl_sendmsg(cb_cmd);
}

int rsl_nokia_si_begin(struct gsm_bts_trx *trx)
{
	struct abis_rsl_common_hdr *ch;
	struct msgb *msg = rsl_msgb_alloc();

	ch = (struct abis_rsl_common_hdr *) msgb_put(msg, sizeof(*ch));
	ch->msg_discr = ABIS_RSL_MDISC_TRX;
	ch->msg_type = 0x40; /* Nokia SI Begin */

	msg->dst = trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_nokia_si_end(struct gsm_bts_trx *trx)
{
	struct abis_rsl_common_hdr *ch;
	struct msgb *msg = rsl_msgb_alloc();

	ch = (struct abis_rsl_common_hdr *) msgb_put(msg, sizeof(*ch));
	ch->msg_discr = ABIS_RSL_MDISC_TRX;
	ch->msg_type = 0x41;  /* Nokia SI End */

	msgb_tv_put(msg, 0xFD, 0x00); /* Nokia Pagemode Info, No paging reorganisation required */

	msg->dst = trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

int rsl_bs_power_control(struct gsm_bts_trx *trx, uint8_t channel, uint8_t reduction)
{
	struct abis_rsl_common_hdr *ch;
	struct msgb *msg = rsl_msgb_alloc();

	ch = (struct abis_rsl_common_hdr *) msgb_put(msg, sizeof(*ch));
	ch->msg_discr = ABIS_RSL_MDISC_DED_CHAN;
	ch->msg_type = RSL_MT_BS_POWER_CONTROL;

	msgb_tv_put(msg, RSL_IE_CHAN_NR, channel);
	msgb_tv_put(msg, RSL_IE_BS_POWER, reduction); /* reduction in 2dB steps */

	msg->dst = trx->rsl_link;

	return abis_rsl_sendmsg(msg);
}

static void handle_classmark_chg(struct gsm_subscriber_connection *conn,
				 struct msgb *msg)
{
	struct gsm48_hdr *gh = msgb_l3(msg);
	unsigned int payload_len = msgb_l3len(msg) - sizeof(*gh);
	uint8_t cm2_len, cm3_len = 0;
	uint8_t *cm2, *cm3 = NULL;


	/* classmark 2 */
	cm2_len = gh->data[0];
	cm2 = &gh->data[1];

	if (cm2_len > 3) {
		LOG_LCHAN(msg->lchan, LOGL_ERROR, "CLASSMARK CHANGE: CM2 too long: %u", cm2_len);
		return;
	}

	if (payload_len > cm2_len + 1) {
		/* we must have a classmark3 */
		if (gh->data[cm2_len+1] != 0x20) {
			LOG_LCHAN(msg->lchan, LOGL_ERROR, "CLASSMARK CHANGE: invalid CM3 TAG\n");
			return;
		}

		cm3_len = gh->data[cm2_len+2];
		cm3 = &gh->data[cm2_len+3];
		if (cm3_len > 14) {
			LOG_LCHAN(msg->lchan, LOGL_ERROR, "CLASSMARK CHANGE: CM3 too long: %u",
				  cm3_len);
			return;
		}
	}

	LOG_LCHAN(msg->lchan, LOGL_DEBUG, "CLASSMARK CHANGE CM2(len=%u) CM3(len=%u)",
		  cm2_len, cm3_len);
	bsc_cm_update(conn, cm2, cm2_len, cm3, cm3_len);
}

static void dispatch_dtap(struct gsm_subscriber_connection *conn,
			  uint8_t link_id, struct msgb *msg)
{
	struct gsm48_hdr *gh;
	uint8_t pdisc;
	uint8_t msg_type;
	int rc;

	if (msgb_l3len(msg) < sizeof(*gh)) {
		LOG_LCHAN(msg->lchan, LOGL_ERROR,
			  "Message too short for a GSM48 header (%u)", msgb_l3len(msg));
		return;
	}

	gh = msgb_l3(msg);
	pdisc = gsm48_hdr_pdisc(gh);
	msg_type = gsm48_hdr_msg_type(gh);

	/* the idea is to handle all RR messages here, and only hand
	 * MM/CC/SMS-CP/LCS up to the MSC.  Some messages like PAGING
	 * RESPONSE or CM SERVICE REQUEST will not be covered here, as
	 * they are only possible in the first L3 message of each L2
	 * channel, i.e. 'conn' will not exist and gsm0408_rcvmsg()
	 * will call api->compl_l3() for it */
	switch (pdisc) {
	case GSM48_PDISC_RR:
		LOG_LCHAN(msg->lchan, LOGL_DEBUG, "Rx %s", gsm48_rr_msg_name(msg_type));
		switch (msg_type) {
		case GSM48_MT_RR_GPRS_SUSP_REQ:
			/* do something? */
			break;
		case GSM48_MT_RR_STATUS:
			LOG_LCHAN(msg->lchan, LOGL_NOTICE, "RR Status: %s", rr_cause_name(gh->data[0]));
			/* do something? */
			break;
		case GSM48_MT_RR_MEAS_REP:
			/* This shouldn't actually end up here, as RSL treats
			* L3 Info of 08.58 MEASUREMENT REPORT different by calling
			* directly into gsm48_parse_meas_rep */
			LOG_LCHAN(msg->lchan, LOGL_ERROR, "DIRECT GSM48 MEASUREMENT REPORT ?!?\n");
			gsm48_tx_rr_status(conn, GSM48_RR_CAUSE_MSG_TYPE_N_COMPAT);
			break;
		case GSM48_MT_RR_HANDO_COMPL:
			/* Chapter 9.1.16 Handover complete */
			if (!conn->ho.fi)
				LOG_LCHAN(msg->lchan, LOGL_ERROR,
					  "Rx RR Handover Complete, but no handover is ongoing\n");
			else
				osmo_fsm_inst_dispatch(conn->ho.fi, HO_EV_RR_HO_COMPLETE, msg);
			break;
		case GSM48_MT_RR_HANDO_FAIL:
			/* Chapter 9.1.17 Handover Failure */
			if (!conn->ho.fi)
				LOG_LCHAN(msg->lchan, LOGL_ERROR,
					  "Rx RR Handover Fail, but no handover is ongoing\n");
			else
				osmo_fsm_inst_dispatch(conn->ho.fi, HO_EV_RR_HO_FAIL, msg);
			break;
		case GSM48_MT_RR_CIPH_M_COMPL:
			bsc_cipher_mode_compl(conn, msg, conn->lchan->encr.alg_id);
			break;
		case GSM48_MT_RR_ASS_COMPL:
			if (conn->assignment.fi)
				osmo_fsm_inst_dispatch(conn->assignment.fi,
						       ASSIGNMENT_EV_RR_ASSIGNMENT_COMPLETE, msg);
			else
				LOGPLCHAN(msg->lchan, DRR, LOGL_ERROR,
					  "Rx RR Assignment Complete, but no assignment is ongoing\n");
			break;
		case GSM48_MT_RR_ASS_FAIL:
			if (conn->assignment.fi)
				osmo_fsm_inst_dispatch(conn->assignment.fi,
						       ASSIGNMENT_EV_RR_ASSIGNMENT_FAIL, msg);
			else
				LOGPLCHAN(msg->lchan, DRR, LOGL_ERROR,
					  "Rx RR Assignment Failure, but no assignment is ongoing\n");
			break;
		case GSM48_MT_RR_CHAN_MODE_MODIF_ACK:
			rc = gsm48_rx_rr_modif_ack(msg);
			if (rc < 0)
				osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_CHAN_MODE_MODIF_ERROR, &rc);
			else
				osmo_fsm_inst_dispatch(msg->lchan->fi, LCHAN_EV_CHAN_MODE_MODIF_ACK, msg);
			break;
		case GSM48_MT_RR_CLSM_CHG:
			handle_classmark_chg(conn, msg);
			break;
		case GSM48_MT_RR_APP_INFO:
			/* Passing RR APP INFO to MSC, not quite
			 * according to spec */
			bsc_dtap(conn, link_id, msg);
			break;
		default:
			/* Drop unknown RR message */
			LOG_LCHAN(msg->lchan, LOGL_NOTICE, "Unknown RR message: %s",
				  gsm48_rr_msg_name(msg_type));
			gsm48_tx_rr_status(conn, GSM48_RR_CAUSE_MSG_TYPE_N);
			break;
		}
		break;
	default:
		bsc_dtap(conn, link_id, msg);
		break;
	}
}

/*! \brief RSL has received a DATA INDICATION with L3 from MS */
int gsm0408_rcvmsg(struct msgb *msg, uint8_t link_id)
{
	struct gsm_lchan *lchan;

	lchan = msg->lchan;
	if (!lchan_may_receive_data(lchan)) {
		LOG_LCHAN(msg->lchan, LOGL_INFO, "Got data in non active state, discarding.\n");
		return -1;
	}

	if (lchan->conn) {
		/* if we already have a connection, forward via DTAP to
		 * MSC */
		dispatch_dtap(lchan->conn, link_id, msg);
	} else {
		/* allocate a new connection */
		lchan->conn = bsc_subscr_con_allocate(msg->lchan->ts->trx->bts->network);
		if (!lchan->conn) {
			lchan_release(lchan, false, true, RSL_ERR_EQUIPMENT_FAIL);
			return -1;
		}
		lchan->conn->lchan = lchan;

		/* fwd via bsc_api to send COMPLETE L3 INFO to MSC */
		bsc_compl_l3(lchan->conn, msg, 0);
		/* conn shall release lchan on teardown, also if this Layer 3 Complete is rejected. */
	}

	return 0;
}

/* 9.1.29 RR Status */
struct msgb *gsm48_create_rr_status(uint8_t cause)
{
	struct msgb *msg;
	struct gsm48_hdr *gh;

	msg = gsm48_msgb_alloc_name("GSM 04.08 RR STATUS");
	if (!msg)
		return NULL;

	gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh) + 1);
	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_STATUS;
	gh->data[0] = cause;

	return msg;
}

/* 9.1.29 RR Status */
int gsm48_tx_rr_status(struct gsm_subscriber_connection *conn, uint8_t cause)
{
	struct msgb *msg = gsm48_create_rr_status(cause);
	if (!msg)
		return -1;
	gscon_submit_rsl_dtap(conn, msg, 0, 0);
	return 0;
}

int send_siemens_mrpci(struct gsm_lchan *lchan,
		       uint8_t *classmark2_lv)
{
	struct rsl_mrpci mrpci;

	if (classmark2_lv[0] < 2)
		return -EINVAL;

	mrpci.power_class = classmark2_lv[1] & 0x7;
	mrpci.vgcs_capable = classmark2_lv[2] & (1 << 1);
	mrpci.vbs_capable = classmark2_lv[2] & (1 <<2);
	mrpci.gsm_phase = (classmark2_lv[1]) >> 5 & 0x3;

	return rsl_siemens_mrpci(lchan, &mrpci);
}

int gsm48_rx_rr_modif_ack(struct msgb *msg)
{
	int rc;
	struct gsm48_hdr *gh = msgb_l3(msg);
	struct gsm48_chan_mode_modify *mod =
				(struct gsm48_chan_mode_modify *) gh->data;

	LOG_LCHAN(msg->lchan, LOGL_DEBUG, "CHANNEL MODE MODIFY ACK for %s",
		  gsm48_chan_mode_name(mod->mode));

	if (mod->mode != msg->lchan->tch_mode) {
		LOG_LCHAN(msg->lchan, LOGL_ERROR,
			  "CHANNEL MODE MODIFY ACK has wrong mode: Wanted: %s Got: %s\n",
			  gsm48_chan_mode_name(msg->lchan->tch_mode),
			  gsm48_chan_mode_name(mod->mode));
		return -1;
	}

	/* update the channel type */
	switch (mod->mode) {
	case GSM48_CMODE_SIGN:
		msg->lchan->rsl_cmode = RSL_CMOD_SPD_SIGN;
		break;
	case GSM48_CMODE_SPEECH_V1:
	case GSM48_CMODE_SPEECH_EFR:
	case GSM48_CMODE_SPEECH_AMR:
		msg->lchan->rsl_cmode = RSL_CMOD_SPD_SPEECH;
		break;
	case GSM48_CMODE_DATA_14k5:
	case GSM48_CMODE_DATA_12k0:
	case GSM48_CMODE_DATA_6k0:
	case GSM48_CMODE_DATA_3k6:
		msg->lchan->rsl_cmode = RSL_CMOD_SPD_DATA;
		break;
	}

	/* We've successfully modified the MS side of the channel,
	 * now go on to modify the BTS side of the channel */
	rc = rsl_chan_mode_modify_req(msg->lchan);

	/* FIXME: we not only need to do this after mode modify, but
	 * also after channel activation */
	if (is_ipaccess_bts(msg->lchan->ts->trx->bts) && mod->mode != GSM48_CMODE_SIGN)
		rsl_tx_ipacc_crcx(msg->lchan);
	return rc;
}

int gsm48_sendmsg(struct msgb *msg)
{
	if (msg->lchan)
		msg->dst = msg->lchan->ts->trx->rsl_link;

	msg->l3h = msg->data;
	return rsl_data_request(msg, 0);
}

/* 7.1.7 and 9.1.7: RR CHANnel RELease */
int gsm48_send_rr_release(struct gsm_lchan *lchan)
{
	struct msgb *msg = gsm48_msgb_alloc_name("GSM 04.08 RR REL");
	struct gsm48_hdr *gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh));
	uint8_t *cause;

	msg->lchan = lchan;
	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_CHAN_REL;

	cause = msgb_put(msg, 1);
	cause[0] = GSM48_RR_CAUSE_NORMAL;

	DEBUGP(DRR, "Sending Channel Release: Chan: Number: %d Type: %d\n",
		lchan->nr, lchan->type);

	/* Send actual release request to MS */
	return gsm48_sendmsg(msg);
}

/* Chapter 9.1.9: Ciphering Mode Command */
int gsm48_send_rr_ciph_mode(struct gsm_lchan *lchan, int want_imeisv)
{
	struct msgb *msg = gsm48_msgb_alloc_name("GSM 04.08 CIPH");
	struct gsm48_hdr *gh;
	uint8_t ciph_mod_set;

	msg->lchan = lchan;

	DEBUGP(DRR, "TX CIPHERING MODE CMD\n");

	if (lchan->encr.alg_id <= RSL_ENC_ALG_A5(0))
		ciph_mod_set = 0;
	else
		ciph_mod_set = (lchan->encr.alg_id-2)<<1 | 1;

	gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh) + 1);
	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_CIPH_M_CMD;
	gh->data[0] = (want_imeisv & 0x1) << 4 | (ciph_mod_set & 0xf);

	return rsl_encryption_cmd(msg);
}

static void gsm48_cell_desc(struct gsm48_cell_desc *cd,
			    const struct gsm_bts *bts)
{
	cd->ncc = (bts->bsic >> 3 & 0x7);
	cd->bcc = (bts->bsic & 0x7);
	cd->arfcn_hi = bts->c0->arfcn >> 8;
	cd->arfcn_lo = bts->c0->arfcn & 0xff;
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

int gsm48_send_ho_cmd(struct gsm_lchan *old_lchan, struct gsm_lchan *new_lchan,
		      uint8_t power_command, uint8_t ho_ref)
{
	struct msgb *msg = gsm48_make_ho_cmd(new_lchan, power_command, ho_ref);
	if (!msg)
		return -EINVAL;
	msg->lchan = old_lchan;
	return gsm48_sendmsg(msg);
}

static void mr_config_for_ms(struct gsm_lchan *lchan, struct msgb *msg)
{
	if (lchan->tch_mode == GSM48_CMODE_SPEECH_AMR)
		msgb_tlv_put(msg, GSM48_IE_MUL_RATE_CFG, lchan->mr_ms_lv[0],
			lchan->mr_ms_lv + 1);
}

/* Chapter 9.1.2: Assignment Command */
int gsm48_send_rr_ass_cmd(struct gsm_lchan *dest_lchan, struct gsm_lchan *lchan, uint8_t power_command)
{
	struct msgb *msg = gsm48_msgb_alloc_name("GSM 04.08 ASS CMD");
	struct gsm48_hdr *gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh));
	struct gsm48_ass_cmd *ass =
		(struct gsm48_ass_cmd *) msgb_put(msg, sizeof(*ass));

	DEBUGP(DRR, "-> ASSIGNMENT COMMAND tch_mode=0x%02x\n", lchan->tch_mode);

	msg->lchan = dest_lchan;
	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_ASS_CMD;

	/*
	 * fill the channel information element, this code
	 * should probably be shared with rsl_rx_chan_rqd(),
	 * gsm48_lchan_modify(). But beware that 10.5.2.5
	 * 10.5.2.5.a have slightly different semantic for
	 * the chan_desc. But as long as multi-slot configurations
	 * are not used we seem to be fine.
	 */
	gsm48_lchan2chan_desc(&ass->chan_desc, lchan);
	ass->power_command = power_command;

	/* optional: cell channel description */

	msgb_tv_put(msg, GSM48_IE_CHANMODE_1, lchan->tch_mode);

	/* mobile allocation in case of hopping */
	if (lchan->ts->hopping.enabled) {
		msgb_tlv_put(msg, GSM48_IE_MA_BEFORE, lchan->ts->hopping.ma_len,
			     lchan->ts->hopping.ma_data);
	}

	/* in case of multi rate we need to attach a config */
	mr_config_for_ms(lchan, msg);

	return gsm48_sendmsg(msg);
}

/* 9.1.5 Channel mode modify: Modify the mode on the MS side */
int gsm48_lchan_modify(struct gsm_lchan *lchan, uint8_t mode)
{
	struct msgb *msg = gsm48_msgb_alloc_name("GSM 04.08 CHN MOD");
	struct gsm48_hdr *gh = (struct gsm48_hdr *) msgb_put(msg, sizeof(*gh));
	struct gsm48_chan_mode_modify *cmm =
		(struct gsm48_chan_mode_modify *) msgb_put(msg, sizeof(*cmm));

	DEBUGP(DRR, "-> CHANNEL MODE MODIFY mode=0x%02x\n", mode);

	lchan->tch_mode = mode;
	msg->lchan = lchan;
	gh->proto_discr = GSM48_PDISC_RR;
	gh->msg_type = GSM48_MT_RR_CHAN_MODE_MODIF;

	/* fill the channel information element, this code
	 * should probably be shared with rsl_rx_chan_rqd() */
	gsm48_lchan2chan_desc(&cmm->chan_desc, lchan);
	cmm->mode = mode;

	/* in case of multi rate we need to attach a config */
	mr_config_for_ms(lchan, msg);

	return gsm48_sendmsg(msg);
}