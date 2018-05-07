/* osmo-bsc API to allocate an lchan, complete with dyn TS switchover and MGCP communication to allocate
 * RTP endpoints.
 *
 * (C) 2017 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 *
 * Author: Neels Hofmeyr <neels@hofmeyr.de>
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

enum lchan_alloc_fsm_states {
	LA_ST_ALLOCATION_REQUESTED,
	LA_ST_DYN_TS_WAIT_PDCH_DEACT,
	LA_ST_LCHAN_CHOSEN,

	/* wait for MGW response to CRCX for BTS */
	LA_ST_WAIT_CRCX_BTS,
	/* wait for MGW response to MDCX for BTS */
	LA_ST_WAIT_MDCX_BTS,
	/* wait for MGW response to CRCX for MSC */
	LA_ST_WAIT_CRCX_MSC,

	LA_ST_DYN_TS_WAIT_DEACT_TCH,
	LA_ST_DYN_TS_WAIT_ACT_PDCH,

	LA_ST_END,
};
