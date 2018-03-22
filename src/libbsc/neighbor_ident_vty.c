/* Quagga VTY implementation to manage identity of neighboring BSS cells for inter-BSC handover. */
/* (C) 2018 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 *
 * All Rights Reserved
 *
 * Author: Neels Hofmeyr <nhofmeyr@sysmocom.de>
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <osmocom/vty/command.h>
#include <osmocom/gsm/gsm0808.h>

#include <osmocom/bsc/neighbor_ident.h>

struct neighbor_ident_list *g_nil = NULL;

bool parse_key(struct vty *vty, const char **argv, struct neighbor_ident_key *key)
{
	*key = (struct neighbor_ident_key){
		.arfcn = atoi(argv[0]),
	};
	const char *bsic_kind = argv[1];
	const char *bsic_str = argv[2];

	if (!strcmp(bsic_str, "any"))
		key->bsic_kind = BSIC_NONE;
	else {
		key->bsic_kind = (!strcmp(bsic_kind, "bsic9")) ? BSIC_9BIT : BSIC_6BIT;
		key->bsic = atoi(bsic_str);
		if (key->bsic_kind == BSIC_6BIT && key->bsic > 0x3f) {
			vty_out(vty, "%% Error: BSIC value surpasses 6-bit range: %u, use 'bsic9' instead%s",
				key->bsic, VTY_NEWLINE);
			return false;
		}
	}
	return true;
}

int add(struct vty *vty, const char **argv, struct gsm0808_cell_id_list2 *cil)
{
	int rc;
	struct neighbor_ident_key key;
	
	if (!parse_key(vty, argv, &key))
		return CMD_WARNING;

	rc = neighbor_ident_add(g_nil, &key, cil);

	if (rc < 0) {
		const char *reason;
		switch (rc) {
		case -EINVAL:
			reason = ": mismatching type between current and newly added cell identifier";
			break;
		case -ENOSPC:
			reason = ": list is full";
			break;
		default:
			reason = "";
			break;
		}

		vty_out(vty, "%% Error adding neighbor-BSS Cell Identifier%s%s",
			reason, VTY_NEWLINE);
		return CMD_WARNING;
	}

	vty_out(vty, "%% Neighbor-BSS %s now has %d Cell Identifier List entries%s",
		neighbor_ident_key_name(&key), rc, VTY_NEWLINE);
	return CMD_SUCCESS;
}

#define NEIGH_BSS_CELL_CMD \
		"neighbor-bss-cell arfcn <0-1023> (bsic|bsic9) (<0-511>|any)"
#define NEIGH_BSS_CELL_ADD_CMD \
		NEIGH_BSS_CELL_CMD " add "
#define NEIGH_BSS_CELL_CMD_DOC \
		"Neighboring BSS cell list\n" \
		"ARFCN of neighbor cell\n" "ARFCN value\n" \
		"BSIC of neighbor cell\n" "9-bit BSIC of neighbor cell\n" "BSIC value\n" \
		"use the same identifier list for all BSIC in this ARFCN\n"
#define NEIGH_BSS_CELL_ADD_CMD_DOC \
		NEIGH_BSS_CELL_CMD_DOC \
		"Add identification of cell in a neighboring BSS\n"

DEFUN(cfg_neighbor_bss_add_cgi, cfg_neighbor_bss_add_cgi_cmd,
      NEIGH_BSS_CELL_ADD_CMD "cgi <0-999> <0-999> <0-65535> <0-255>",
      NEIGH_BSS_CELL_ADD_CMD_DOC
      "Set neighboring identification as Cell Global Identification (MCC, MNC, LAC, CI)\n"
      "MCC\n" "MNC\n" "LAC\n" "CI\n")
{
	struct gsm0808_cell_id_list2 cil = {
		.id_discr = CELL_IDENT_WHOLE_GLOBAL,
		.id_list_len = 1,
	};
	const char *mcc = argv[3];
	const char *mnc = argv[4];
	struct osmo_cell_global_id *cgi = &cil.id_list[0].global;
	cgi->lai.lac = atoi(argv[5]);
	cgi->cell_identity = atoi(argv[6]);

	if (osmo_mcc_from_str(mcc, &cgi->lai.plmn.mcc)) {
		vty_out(vty, "%% Error decoding MCC: %s%s", mcc, VTY_NEWLINE);
		return CMD_WARNING;
	}

	if (osmo_mnc_from_str(mnc, &cgi->lai.plmn.mnc, &cgi->lai.plmn.mnc_3_digits)) {
		vty_out(vty, "%% Error decoding MNC: %s%s", mnc, VTY_NEWLINE);
		return CMD_WARNING;
	}

	return add(vty, argv, &cil);
}

DEFUN(cfg_neighbor_bss_del, cfg_neighbor_bss_del_cmd,
      NEIGH_BSS_CELL_CMD " del",
      NEIGH_BSS_CELL_CMD_DOC
      "Delete cell identifier entry\n")
{
	struct neighbor_ident_key key;

	if (!parse_key(vty, argv, &key))
		return CMD_WARNING;

	if (!neighbor_ident_del(g_nil, &key)) {
		vty_out(vty, "%% Error deleting cell identification, entry does not exists%s",
			VTY_NEWLINE);
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}


struct write_neighbor_ident_entry_data {
	struct vty *vty;
	const char *indent;
};

static bool write_neighbor_ident_entry(const struct neighbor_ident_key *key,
				       const struct gsm0808_cell_id_list2 *val,
				       void *cb_data)
{
	struct write_neighbor_ident_entry_data *d = cb_data;
	struct vty *vty = d->vty;
	int i;

#define NEIGH_BSS_WRITE(fmt, args...) do { \
		vty_out(vty, "%sneighbor-bss-cell arfcn %u ", d->indent, key->arfcn); \
		switch (key->bsic_kind) { \
		default: \
		case BSIC_NONE: \
			vty_out(vty, "bsic any"); \
			break; \
		case BSIC_6BIT: \
			vty_out(vty, "bsic %u", key->bsic & 0x3f); \
			break; \
		case BSIC_9BIT: \
			vty_out(vty, "bsic9 %u", key->bsic & 0x1ff); \
			break; \
		} \
		vty_out(vty, " add " fmt "%s", ## args, VTY_NEWLINE); \
	} while(0)

	switch (val->id_discr) {
	case CELL_IDENT_WHOLE_GLOBAL:
		for (i = 0; i < val->id_list_len; i++) {
			const struct osmo_cell_global_id *cgi = &val->id_list[i].global;
			NEIGH_BSS_WRITE("cgi %s %s %u %u",
					osmo_mcc_name(cgi->lai.plmn.mcc),
					osmo_mnc_name(cgi->lai.plmn.mnc, cgi->lai.plmn.mnc_3_digits),
					cgi->lai.lac, cgi->cell_identity);
		}
		break;
	default:
		vty_out(vty, "%% Unsupported Cell Identity%s", VTY_NEWLINE);
	}
#undef NEIGH_BSS_WRITE

	return true;
}

void neighbor_ident_vty_write(const struct neighbor_ident_list *nil, struct vty *vty, const char *indent)
{
	struct write_neighbor_ident_entry_data d = {
		.vty = vty,
		.indent = indent,
	};

	neighbor_ident_iter(nil, write_neighbor_ident_entry, &d);
}

DEFUN(cfg_neighbor_bss_resolve, cfg_neighbor_bss_resolve_cmd,
      NEIGH_BSS_CELL_CMD " resolve",
      NEIGH_BSS_CELL_CMD_DOC
      "Query which Cell Identifier List would be used for this ARFCN/BSIC\n")
{
	struct neighbor_ident_key key;
	struct gsm0808_cell_id_list2 *res;
	struct write_neighbor_ident_entry_data d = {
		.vty = vty,
		.indent = "% ",
	};

	if (!parse_key(vty, argv, &key))
		return CMD_WARNING;

	res = neighbor_ident_get(g_nil, &key);
	if (!res)
		vty_out(vty, "%% No entry for %s%s", neighbor_ident_key_name(&key), VTY_NEWLINE);
	else
		write_neighbor_ident_entry(&key, res, &d);

	return CMD_SUCCESS;
}

void neighbor_ident_vty_init(struct neighbor_ident_list *nil, int parent_node)
{
	g_nil = nil;
	install_element(parent_node, &cfg_neighbor_bss_add_cgi_cmd);
	install_element(parent_node, &cfg_neighbor_bss_del_cmd);
	install_element(parent_node, &cfg_neighbor_bss_resolve_cmd);
}
