#pragma once

struct gsm_lchan;
struct gsm_bts;
struct gsm_subscriber_connection;

int bsc_handover_start(struct gsm_lchan *old_lchan, struct gsm_bts *bts);
int bsc_handover_start_lchan_change(struct gsm_lchan *old_lchan, struct gsm_bts *bts,
				    enum gsm_chan_t new_lchan_type);
void bsc_clear_handover(struct gsm_subscriber_connection *conn, int free_lchan);
struct gsm_lchan *bsc_handover_pending(struct gsm_lchan *new_lchan);

int bsc_ho_count(struct gsm_bts *bts, bool inter_cell);
