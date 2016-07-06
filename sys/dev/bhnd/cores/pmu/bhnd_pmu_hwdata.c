/*
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2010 Broadcom Corporation
 * 
 * This file was initially derived from Broadcom's initial brcm80211 Linux
 * driver release of hndpmu.c, as contributed to the Linux staging repository.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>

#include "bhnd_pmu_hwdata.h"
#include "bhnd_pmureg.h"

#define	PMURES_BIT(_bit)	\
	(1 << (BHND_PMU_ ## _bit))

const struct bhnd_pmu_cfg bhnd_pmu_cfg_table[] = {
	{
		.match = { BHND_CHIP_ID(BCM4315) },
		.deps = (const struct bhnd_pmu_res_depend[]) {
			/* Adjust OTP PU resource dependencies - remove BB BURST */
			{
				PMURES_BIT(RES4325_OTP_PU), 
				BHND_PMU_DEPEND_REMOVE,
				PMURES_BIT(RES4325_BUCK_BOOST_BURST),
				BHND_MATCH_ANY
			},
		#if 0
			/* Adjust ALP/HT Avail resource dependencies - bring up BB along if it is used. */
			{
			PMURES_BIT(RES4325_ALP_AVAIL) | PMURES_BIT(RES4325_HT_AVAIL),
				BHND_PMU_DEPEND_ADD,
				PMURES_BIT(RES4325_BUCK_BOOST_BURST) |
				PMURES_BIT(RES4325_BUCK_BOOST_PWM), bhnd_pmu_res_depfltr_bb},
			/* Adjust HT Avail resource dependencies - bring up RF switches along with HT. */
			{
			PMURES_BIT(RES4325_HT_AVAIL),
				BHND_PMU_DEPEND_ADD,
				PMURES_BIT(RES4325_RX_PWRSW_PU) |
				PMURES_BIT(RES4325_TX_PWRSW_PU) |
				PMURES_BIT(RES4325_LOGEN_PWRSW_PU) |
				PMURES_BIT(RES4325_AFE_PWRSW_PU), NULL},
			/* Adjust ALL resource dependencies - remove CBUCK dependancies if it is not used. */
			{
			PMURES_BIT(RES4325_ILP_REQUEST) |
				PMURES_BIT(RES4325_ABUCK_BURST) |
				PMURES_BIT(RES4325_ABUCK_PWM) |
				PMURES_BIT(RES4325_LNLDO1_PU) |
				PMURES_BIT(RES4325C1_LNLDO2_PU) |
				PMURES_BIT(RES4325_XTAL_PU) |
				PMURES_BIT(RES4325_ALP_AVAIL) |
				PMURES_BIT(RES4325_RX_PWRSW_PU) |
				PMURES_BIT(RES4325_TX_PWRSW_PU) |
				PMURES_BIT(RES4325_RFPLL_PWRSW_PU) |
				PMURES_BIT(RES4325_LOGEN_PWRSW_PU) |
				PMURES_BIT(RES4325_AFE_PWRSW_PU) |
				PMURES_BIT(RES4325_BBPLL_PWRSW_PU) |
				PMURES_BIT(RES4325_HT_AVAIL), BHND_PMU_DEPEND_REMOVE,
				PMURES_BIT(RES4325B0_CBUCK_LPOM) |
				PMURES_BIT(RES4325B0_CBUCK_BURST) |
				PMURES_BIT(RES4325B0_CBUCK_PWM), bhnd_pmu_res_depfltr_ncb}
			},
		#endif
		},
		.updown = (const struct bhnd_pmu_res_updown[]) {
			{ BHND_PMU_RES4315_XTAL_PU, 0x2501 }
		}
	}
};
