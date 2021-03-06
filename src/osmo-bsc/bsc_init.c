/* A hackish minimal BSC (+MSC +HLR) implementation */

/* (C) 2008-2018 by Harald Welte <laforge@gnumonks.org>
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
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <osmocom/bsc/gsm_data.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/bsc/abis_rsl.h>
#include <osmocom/bsc/debug.h>
#include <osmocom/bsc/misdn.h>
#include <osmocom/bsc/system_information.h>
#include <osmocom/bsc/paging.h>
#include <osmocom/bsc/signal.h>
#include <osmocom/bsc/chan_alloc.h>
#include <osmocom/core/talloc.h>
#include <osmocom/bsc/ipaccess.h>
#include <osmocom/gsm/sysinfo.h>
#include <osmocom/bsc/pcu_if.h>
#include <osmocom/bsc/bsc_msc_data.h>
#include <osmocom/bsc/handover_cfg.h>
#include <osmocom/bsc/gsm_04_08_rr.h>
#include <osmocom/bsc/neighbor_ident.h>
#include <osmocom/bsc/bts.h>

#include <osmocom/bsc/smscb.h>
#include <osmocom/gsm/protocol/gsm_48_049.h>

#include <time.h>
#include <limits.h>
#include <stdbool.h>

static const struct osmo_stat_item_desc bsc_stat_desc[] = {
	{ "num_bts:total", "Number of configured BTS for this BSC", "", 16, 0 },
};

static const struct osmo_stat_item_group_desc bsc_statg_desc = {
	.group_name_prefix = "bsc",
	.group_description = "base station controller",
	.class_id = OSMO_STATS_CLASS_GLOBAL,
	.num_items = ARRAY_SIZE(bsc_stat_desc),
	.item_desc = bsc_stat_desc,
};

int bsc_shutdown_net(struct gsm_network *net)
{
	struct gsm_bts *bts;

	llist_for_each_entry(bts, &net->bts_list, list) {
		LOGP(DNM, LOGL_NOTICE, "shutting down OML for BTS %u\n", bts->nr);
		osmo_signal_dispatch(SS_L_GLOBAL, S_GLOBAL_BTS_CLOSE_OM, bts);
	}

	return 0;
}

/* XXX hard-coded for now */
#define T3122_CHAN_LOAD_SAMPLE_INTERVAL 1 /* in seconds */

static void update_t3122_chan_load_timer(void *data)
{
	struct gsm_network *net = data;
	struct gsm_bts *bts;

	llist_for_each_entry(bts, &net->bts_list, list)
		bts_update_t3122_chan_load(bts);

	/* Keep this timer ticking. */
	osmo_timer_schedule(&net->t3122_chan_load_timer, T3122_CHAN_LOAD_SAMPLE_INTERVAL, 0);
}

static struct gsm_network *bsc_network_init(void *ctx)
{
	struct gsm_network *net = gsm_network_init(ctx);

	net->cbc = talloc_zero(net, struct bsc_cbc_link);
	if (!net->cbc) {
		talloc_free(net);
		return NULL;
	}

	/* Init back pointer */
	net->auto_off_timeout = -1;
	INIT_LLIST_HEAD(&net->mscs);

	net->ho = ho_cfg_init(net, NULL);
	net->hodec2.congestion_check_interval_s = HO_CFG_CONGESTION_CHECK_DEFAULT;
	net->neighbor_bss_cells = neighbor_ident_init(net);

	/* init statistics */
	net->bsc_ctrs = rate_ctr_group_alloc(net, &bsc_ctrg_desc, 0);
	if (!net->bsc_ctrs) {
		talloc_free(net);
		return NULL;
	}
	net->bsc_statg = osmo_stat_item_group_alloc(net, &bsc_statg_desc, 0);
	if (!net->bsc_statg) {
		rate_ctr_group_free(net->bsc_ctrs);
		talloc_free(net);
		return NULL;
	}

	INIT_LLIST_HEAD(&net->bts_rejected);
	gsm_net_update_ctype(net);

	/*
	 * At present all BTS in the network share one channel load timeout.
	 * If this becomes a problem for networks with a lot of BTS, this
	 * code could be refactored to run the timeout individually per BTS.
	 */
	osmo_timer_setup(&net->t3122_chan_load_timer, update_t3122_chan_load_timer, net);
	osmo_timer_schedule(&net->t3122_chan_load_timer, T3122_CHAN_LOAD_SAMPLE_INTERVAL, 0);

	net->cbc->net = net;
	/* no cbc_hostname: client not started by default */
	net->cbc->config.cbc_port = CBSP_TCP_PORT;
	/* listen_port == -1: server not started by default */
	net->cbc->config.listen_port = -1;
	net->cbc->config.listen_hostname = talloc_strdup(net->cbc, "127.0.0.1");

	return net;
}

int bsc_network_alloc(void)
{
	/* initialize our data structures */
	bsc_gsmnet = bsc_network_init(tall_bsc_ctx);
	if (!bsc_gsmnet)
		return -ENOMEM;

	return 0;
}

struct gsm_bts *bsc_bts_alloc_register(struct gsm_network *net, enum gsm_bts_type type, uint8_t bsic)
{
	struct gsm_bts *bts = gsm_bts_alloc_register(net, type, bsic);
	OSMO_ASSERT(bts != NULL);

	bts->ho = ho_cfg_init(bts, net->ho);

	return bts;
}
