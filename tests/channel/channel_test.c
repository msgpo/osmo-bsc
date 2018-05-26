/*
 * (C) 2009 by Holger Hans Peter Freyther <zecke@selfish.org>
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
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <assert.h>

#include <osmocom/core/application.h>
#include <osmocom/core/select.h>

#include <osmocom/bsc/abis_rsl.h>
#include <osmocom/bsc/debug.h>

void *ctx = NULL;

void test_bts_debug_print(void)
{
	struct gsm_network *network;
	struct gsm_bts *bts;
	struct gsm_bts_trx *trx;

	printf("Testing the lchan printing:");

	/* Create a dummy network */
	network = gsm_network_init(ctx);
	if (!network)
		exit(1);
	/* Add a BTS with some reasonanbly non-zero id */
	bts = gsm_bts_alloc(network, 45);
	/* Add a second TRX to test on multiple TRXs */
	gsm_bts_trx_alloc(bts);

	llist_for_each_entry(trx, &bts->trx_list, list) {
		char *name = gsm_lchan_name(&trx->ts[3].lchan[4]);

		if (name)
			printf(" %s", name);
		else
			printf("NULL name");
	}
	printf("\n");
}


void test_dyn_ts_subslots(void)
{
	struct gsm_bts_trx_ts ts;

	printf("Testing subslot numbers for pchan types\n");

	ts.pchan = GSM_PCHAN_TCH_F;
	OSMO_ASSERT(ts_subslots(&ts) == 1);

	ts.pchan = GSM_PCHAN_TCH_H;
	OSMO_ASSERT(ts_subslots(&ts) == 2);

	ts.pchan = GSM_PCHAN_PDCH;
	OSMO_ASSERT(ts_subslots(&ts) == 0);

	ts.pchan = GSM_PCHAN_TCH_F_PDCH;
	ts.flags = 0; /* TCH_F mode */
	OSMO_ASSERT(ts_subslots(&ts) == 1);
	ts.flags = TS_F_PDCH_ACTIVE;
	OSMO_ASSERT(ts_subslots(&ts) == 0);

	ts.pchan = GSM_PCHAN_TCH_F_TCH_H_PDCH;
	ts.dyn.pchan_is = GSM_PCHAN_TCH_F;
	OSMO_ASSERT(ts_subslots(&ts) == 1);
	ts.dyn.pchan_is = GSM_PCHAN_TCH_H;
	OSMO_ASSERT(ts_subslots(&ts) == 2);
	ts.dyn.pchan_is = GSM_PCHAN_PDCH;
	OSMO_ASSERT(ts_subslots(&ts) == 0);
}

static const struct log_info_cat log_categories[] = {
};

static const struct log_info log_info = {
	.cat = log_categories,
	.num_cat = ARRAY_SIZE(log_categories),
};

int main(int argc, char **argv)
{
	ctx = talloc_named_const(NULL, 0, "channel_test");
	osmo_init_logging2(ctx, &log_info);

	test_dyn_ts_subslots();
	test_bts_debug_print();

	return EXIT_SUCCESS;
}

bool on_gsm_ts_init(struct gsm_bts_trx_ts *ts) { return true; }
