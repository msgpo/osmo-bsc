/* Manage identity of neighboring BSS cells for inter-BSC handover */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/linuxlist.h>

struct vty;
struct neighbor_ident_list;
struct gsm0808_cell_id_list2;

enum bsic_kind {
	BSIC_NONE,
	BSIC_6BIT,
	BSIC_9BIT,
};

struct neighbor_ident_key {
	uint16_t arfcn;
	enum bsic_kind bsic_kind;
	uint16_t bsic;
};

const char *neighbor_ident_key_name(const struct neighbor_ident_key *ni_key);

struct neighbor_ident_list *neighbor_ident_init(void *talloc_ctx);
void neighbor_ident_free(struct neighbor_ident_list *nil);

bool neighbor_ident_key_match(const struct neighbor_ident_key *entry,
			      const struct neighbor_ident_key *search_for,
			      bool exact_bsic_kind);

int neighbor_ident_add(struct neighbor_ident_list *nil, const struct neighbor_ident_key *key,
		       const struct gsm0808_cell_id_list2 *val);
const struct gsm0808_cell_id_list2 *neighbor_ident_get(const struct neighbor_ident_list *nil,
						       const struct neighbor_ident_key *key);
bool neighbor_ident_del(struct neighbor_ident_list *nil, const struct neighbor_ident_key *key);
void neighbor_ident_clear(struct neighbor_ident_list *nil);

void neighbor_ident_iter(const struct neighbor_ident_list *nil,
			 bool (* iter_cb )(const struct neighbor_ident_key *key,
					   const struct gsm0808_cell_id_list2 *val,
					   void *cb_data),
			 void *cb_data);

void neighbor_ident_vty_init(struct neighbor_ident_list *nil, int parent_node);
void neighbor_ident_vty_write(const struct neighbor_ident_list *nil, struct vty *vty, const char *indent);

#define NEIGHBOR_IDENT_VTY_KEY_PARAMS "arfcn <0-1023> (bsic|bsic9) (<0-511>|any)"
#define NEIGHBOR_IDENT_VTY_KEY_DOC \
	"ARFCN of neighbor cell\n" "ARFCN value\n" \
	"BSIC of neighbor cell\n" "9-bit BSIC of neighbor cell\n" "BSIC value\n" \
	"for all BSICs / use any BSIC in this ARFCN\n"
bool neighbor_ident_vty_parse_key_params(struct vty *vty, const char **argv, struct neighbor_ident_key *key);
