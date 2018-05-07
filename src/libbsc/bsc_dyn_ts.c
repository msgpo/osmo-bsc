/* Dynamic PDCH initialisation implementation shared across NM and RSL */

/* (C) 2016 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
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

#include <osmocom/core/logging.h>
#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/gsm_data.h>
#include <osmocom/bsc/abis_rsl.h>

static void tchf_pdch_ts_init(struct gsm_bts_trx_ts *ts)
{
	int rc;

	rc = rsl_ipacc_pdch_activate(ts, 1);
	if (rc != 0 && rc != -ENOTSUP)
		LOGP(DRSL, LOGL_ERROR, "%s %s: PDCH ACT failed\n",
		     gsm_ts_name(ts), gsm_pchan_name(ts->pchan));
}

static void tchf_tchh_pdch_ts_init(struct gsm_bts_trx_ts *ts)
{
	dyn_ts_switchover_start(ts, GSM_PCHAN_PDCH);
}

void dyn_ts_init(struct gsm_bts_trx_ts *ts)
{
	/* Clear all TCH/F_PDCH flags */
	ts->flags &= ~(TS_F_PDCH_PENDING_MASK | TS_F_PDCH_ACTIVE);

	/* Clear TCH/F_TCH/H_PDCH state */
	ts->dyn.pchan_is = ts->dyn.pchan_want = GSM_PCHAN_NONE;
	ts->dyn.pending_chan_activ = NULL;

	switch (ts->pchan) {
	case GSM_PCHAN_TCH_F_PDCH:
		tchf_pdch_ts_init(ts);
		break;
	case GSM_PCHAN_TCH_F_TCH_H_PDCH:
		tchf_tchh_pdch_ts_init(ts);
		break;
	default:
		break;
	}
}
