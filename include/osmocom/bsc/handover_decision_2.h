/* Handover Decision Algorithm 2 for intra-BSC (inter-BTS) handover, public API for OsmoBSC */

#pragma once
struct gsm_bts;

void handover_decision_2_init(struct gsm_network *net);

void handover_decision_2_bts_congestion_check(struct gsm_bts *bts);
