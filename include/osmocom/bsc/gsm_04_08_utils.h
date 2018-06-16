#pragma once

void gsm_net_update_ctype(struct gsm_network *network);
enum gsm_chan_t get_ctype_by_chreq(struct gsm_network *network, uint8_t ra);
int get_reason_by_chreq(uint8_t ra, int neci);
int gsm48_handle_paging_resp(struct gsm_subscriber_connection *conn,
			     struct msgb *msg, struct bsc_subscr *bsub);
void gsm48_lchan2chan_desc(struct gsm48_chan_desc *cd,
			   const struct gsm_lchan *lchan);
int gsm48_multirate_config(uint8_t *lv, const struct amr_multirate_conf *mr, const struct amr_mode *modes);
int gsm48_parse_meas_rep(struct gsm_meas_rep *rep, struct msgb *msg);
int gsm48_tx_mm_serv_ack(struct gsm_subscriber_connection *conn);
int gsm48_tx_mm_serv_rej(struct gsm_subscriber_connection *conn,
			 enum gsm48_reject_value value);

struct msgb *gsm48_create_mm_serv_rej(enum gsm48_reject_value value);
int gsm48_extract_mi(uint8_t *classmark2_lv, int length, char *mi_string, uint8_t *mi_type);
int gsm48_paging_extract_mi(struct gsm48_pag_resp *resp, int length,
			    char *mi_string, uint8_t *mi_type);
struct msgb *gsm48_create_loc_upd_rej(uint8_t cause);

#define GSM48_ALLOC_SIZE        2048
#define GSM48_ALLOC_HEADROOM    256

static inline struct msgb *gsm48_msgb_alloc_name(const char *name)
{
        return msgb_alloc_headroom(GSM48_ALLOC_SIZE, GSM48_ALLOC_HEADROOM,
                                   name);
}

uint64_t str_to_imsi(const char *imsi_str);
