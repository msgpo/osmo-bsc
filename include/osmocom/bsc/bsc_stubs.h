/* libbsc currently references some BSSMAP functions only implemented within osmo-bsc/. Until that gets
 * cleaned up properly, various utils and C tests need these stubs to successfully link a complete
 * binary. This nonsense should disappear as soon as we get time to fix it. */

#include <osmocom/bsc/handover.h>

struct gsm_subscriber_connection;
struct msgb;
struct gsm_lchan;
struct gsm0808_cell_id_list2;
struct gsm0808_handover_required;
struct handover;

int osmo_bsc_sigtran_send(struct gsm_subscriber_connection *conn, struct msgb *msg)
{ OSMO_ASSERT(false); }

int osmo_bsc_sigtran_open_conn(struct gsm_subscriber_connection *conn, struct msgb *msg)
{ return 0; }

int bsc_handover_inter_bsc_start(enum hodec_id from_hodec_id, struct gsm_lchan *old_lchan,
				 struct gsm0808_cell_id_list2 *target_cells,
				 enum gsm_chan_t new_lchan_type)
{ OSMO_ASSERT(0); }

int bsc_send_handover_required(struct gsm_lchan *lchan, const struct gsm0808_handover_required *params)
{ OSMO_ASSERT(0); }

int bsc_send_handover_request_ack(struct handover *ho, struct msgb *rr_ho_command)
{ OSMO_ASSERT(0); }
