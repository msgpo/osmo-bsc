/*
 * (C) 2010 by Holger Hans Peter Freyther <zecke@selfish.org>
 * (C) 2010 by on-waves.com
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef BSC_NAT_H
#define BSC_NAT_H

#include <sys/types.h>
#include <sccp/sccp_types.h>
#include "msgb.h"

#define FILTER_NONE	0
#define FILTER_TO_BSC	1
#define FILTER_TO_MSC	2
#define FILTER_TO_BOTH	3

/*
 * For the NAT we will need to analyze and later patch
 * the received message. This would require us to parse
 * the IPA and SCCP header twice. Instead of doing this
 * we will have one analyze structure and have the patching
 * and filter operate on the same structure.
 */
struct bsc_nat_parsed {
	/* ip access prototype */
	int ipa_proto;

	/* source local reference */
	struct sccp_source_reference *src_local_ref;

	/* destination local reference */
	struct sccp_source_reference *dest_local_ref;

	/* called ssn number */
	int called_ssn;

	/* calling ssn number */
	int calling_ssn;

	/* sccp message type */
	int sccp_type;

	/* bssap type, e.g. 0 for BSS Management */
	int bssap;

	/* the gsm0808 message type */
	int gsm_type;
};

/**
 * parse the given message into the above structure
 */
struct bsc_nat_parsed *bsc_nat_parse(struct msgb *msg);

/**
 * filter based on IP Access header in both directions
 */
int bsc_nat_filter_ipa(struct msgb *msg, struct bsc_nat_parsed *parsed);

#endif
