#pragma once

#include <stdint.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/timer.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/gsm/gsm0808.h>

#include <osmocom/bsc/neighbor_ident.h>
#include <osmocom/bsc/gsm_data.h>

struct gsm_network;
struct gsm_lchan;
struct gsm_bts;
struct gsm_subscriber_connection;
struct gsm_meas_rep mr;

#define LOG_FMT_BTS "bts=%d:lac=%d,ci=%d,arfcn=%d,bsic=%d"
#define LOG_ARGS_BTS(bts) \
		(bts) ? (bts)->nr : -1, \
		(bts) ? (bts)->location_area_code : -1, \
		(bts) ? (bts)->cell_identity : -1, \
		(bts) ? (bts)->c0->arfcn : -1, \
		(bts) ? (bts)->bsic : -1

#define LOG_FMT_LCHAN LOG_FMT_BTS "trx=%d,ts=%d,ss=%d,%s"
#define LOG_ARGS_LCHAN(lchan) \
		LOG_ARGS_BTS(lchan ? lchan->ts->trx->bts : NULL), \
		lchan ? lchan->ts->trx->nr : -1, \
		lchan ? lchan->ts->nr : -1, \
		lchan ? lchan->nr : -1, \
		lchan ? gsm_pchan_name(lchan->ts->pchan) : "-"

#define LOG_FMT_HO "(subscr %s) %s"
#define LOG_ARGS_HO(ho) \
	     bsc_subscr_name(ho->conn? ho->conn->bsub : NULL), \
	     handover_scope_name(ho->scope)

#define LOGPHO(HO, level, fmt, args ...) \
	do { \
	if ((HO)->scope & (HO_INTRA_CELL | HO_INTRA_BSC)) { \
		if ((HO)->mt.new_lchan) \
			LOGP(DHODEC, level, "("LOG_FMT_LCHAN")->(" LOG_FMT_LCHAN ") " LOG_FMT_HO ": " fmt, \
			     LOG_ARGS_LCHAN((HO)->mo.old_lchan), \
			     LOG_ARGS_LCHAN((HO)->mt.new_lchan), \
			     LOG_ARGS_HO(HO), ## args); \
		else if ((HO)->mt.new_bts) \
			LOGP(DHODEC, level, "("LOG_FMT_LCHAN")->("LOG_FMT_BTS",%s) " LOG_FMT_HO ": " fmt, \
			     LOG_ARGS_LCHAN((HO)->mo.old_lchan), \
			     LOG_ARGS_BTS((HO)->mt.new_bts), \
			     gsm_lchant_name((HO)->new_lchan_type), \
			     LOG_ARGS_HO(HO), ## args); \
		else \
			LOGP(DHODEC, level, "("LOG_FMT_LCHAN")->(?) " LOG_FMT_HO ": " fmt, \
			     LOG_ARGS_LCHAN((HO)->mo.old_lchan), \
			     LOG_ARGS_HO(HO), ## args); \
	} else if ((HO)->scope & HO_INTER_BSC_MO) \
		LOGP(DHODEC, level, "("LOG_FMT_LCHAN")->(%s) " LOG_FMT_HO ": " fmt, \
		     LOG_ARGS_LCHAN((HO)->mo.old_lchan), \
		     neighbor_ident_key_name(&(HO)->mo.target_cell), \
		     LOG_ARGS_HO(HO), ## args); \
	else if ((HO)->scope & HO_INTER_BSC_MT) { \
		if ((HO)->mt.new_lchan) \
			LOGP(DHODEC, level, "(remote:%s)->(local:%s|"LOG_FMT_LCHAN") " LOG_FMT_HO ": " fmt, \
			     gsm0808_cell_id_name(&(HO)->mt.inter_bsc.cell_id_serving), \
			     gsm0808_cell_id_name2(&(HO)->mt.inter_bsc.cell_id_target), \
			     LOG_ARGS_LCHAN((HO)->mt.new_lchan), \
			     LOG_ARGS_HO(HO), ## args); \
		else if ((HO)->mt.new_bts) \
			LOGP(DHODEC, level, "(remote:%s)->(local:%s|"LOG_FMT_BTS",%s) " LOG_FMT_HO ": " fmt, \
			     gsm0808_cell_id_name(&(HO)->mt.inter_bsc.cell_id_serving), \
			     gsm0808_cell_id_name2(&(HO)->mt.inter_bsc.cell_id_target), \
			     LOG_ARGS_BTS((HO)->mt.new_bts), \
			     gsm_lchant_name((HO)->new_lchan_type), \
			     LOG_ARGS_HO(HO), ## args); \
		else  \
			LOGP(DHODEC, level, "(remote:%s)->(local:%s,%s) " LOG_FMT_HO ": " fmt, \
			     gsm0808_cell_id_name(&(HO)->mt.inter_bsc.cell_id_serving), \
			     gsm0808_cell_id_name2(&(HO)->mt.inter_bsc.cell_id_target), \
			     gsm_lchant_name((HO)->new_lchan_type), \
			     LOG_ARGS_HO(HO), ## args); \
	} else \
		LOGP(DHODEC, level, LOG_FMT_HO ": " fmt, LOG_ARGS_HO(HO), ## args); \
	} while(0)


enum hodec_id {
	HODEC_NONE,
	HODEC1 = 1,
	HODEC2 = 2,
};

struct bsc_handover {
	struct llist_head list;

	/* Initial details of what is requested */
	struct gsm_lchan *old_lchan;
	struct gsm_bts *new_bts;
	enum gsm_chan_t new_lchan_type;
	bool async;

	/* Derived and resulting state */
	bool inter_cell;
	uint8_t ho_ref;
	enum hodec_id from_hodec_id;
	struct gsm_lchan *new_lchan;
	struct osmo_timer_list T3103;
};

int bsc_handover_start(enum hodec_id from_hodec_id, struct gsm_lchan *old_lchan, struct gsm_bts *new_bts,
		       enum gsm_chan_t new_lchan_type);
int bsc_handover_start_gscon(struct gsm_subscriber_connection *conn);
void bsc_clear_handover(struct gsm_subscriber_connection *conn, int free_lchan);
struct gsm_lchan *bsc_handover_pending(struct gsm_lchan *new_lchan);

int bsc_ho_count(struct gsm_bts *bts, bool inter_cell);

/* Handover decision algorithms' actions to take on incoming handover-relevant events.
 *
 * All events that are interesting for handover decision are actually communicated by S_LCHAN_* signals,
 * so theoretically, each handover algorithm could evaluate those.  However, handover_logic.c cleans up
 * handover operation state upon receiving some of these signals. To allow a handover decision algorithm
 * to take advantage of e.g. the struct bsc_handover before it is discarded, the handover decision event
 * handler needs to be invoked before handover_logic.c discards the state. For example, if the handover
 * decision wants to place a penalty timer upon a handover failure, it still needs to know which target
 * cell the handover failed for; handover_logic.c erases that knowledge on handover failure, since it
 * needs to clean up the lchan's handover state.
 *
 * The most explicit and safest way to ensure the correct order of event handling is to invoke the
 * handover decision algorithm's actions from handover_logic.c itself, before cleaning up. This struct
 * provides the callback functions for this purpose.
 *
 * For consistency, also handle signals in this way that aren't actually in danger of interference from
 * handover_logic.c (which also saves repeated lookup of handover state for lchans). Thus, handover
 * decision algorithms should not register any signal handler at all.
 */
struct handover_decision_callbacks {
	struct llist_head entry;

	int hodec_id;

	void (*on_measurement_report)(struct gsm_meas_rep *mr);
	void (*on_ho_chan_activ_nack)(struct bsc_handover *ho);
	void (*on_ho_failure)(struct bsc_handover *ho);
};

void handover_decision_callbacks_register(struct handover_decision_callbacks *hdc);
struct handover_decision_callbacks *handover_decision_callbacks_get(int hodec_id);

int bsc_handover_inter_bsc_start(enum hodec_id from_hodec_id, struct gsm_lchan *old_lchan,
				 struct gsm0808_cell_id_list2 *target_cells,
				 enum gsm_chan_t new_lchan_type);
int bsc_send_handover_required(struct gsm_lchan *lchan,
			       const struct gsm0808_handover_required *params);

struct gsm_bts *bts_by_neighbor_ident(const struct gsm_network *net,
				      const struct neighbor_ident_key *search_for);
struct neighbor_ident_key *bts_ident_key(const struct gsm_bts *bts);

int handover_to_neighbor_ident(enum hodec_id from_hodec_id, struct gsm_lchan *lchan,
			       struct neighbor_ident_key *ni, enum gsm_chan_t new_lchan_type);
