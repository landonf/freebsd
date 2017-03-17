/*-
 * Copyright (c) 2016-2017 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (C) 2010, Broadcom Corporation.
 * All rights reserved.
 *
 * This file is derived from the hndpmu.c source contributed by Broadcom 
 * to to the Linux staging repository, as well as later revisions of hndpmu.c
 * distributed with the Asus RT-N16 firmware source code release.
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

#include <machine/_inttypes.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_pmu0reg.h"
#include "bhnd_pmureg.h"
#include "bhnd_pmuvar.h"

#include "bhnd_pmu_private.h"

#define	PMU0_XTAL0_DEFAULT_FREQ	20000	/* 20Mhz */

struct bhnd_pmu0_xtaltab0;

static const struct bhnd_pmu0_xtaltab0	*bhnd_pmu0_xtaltab_get_default(
					     struct bhnd_pmu_query *query);
static const struct bhnd_pmu0_xtaltab0	*bhnd_pmu0_xtaltab_findfreq(
					     struct bhnd_pmu_query *query,
					     uint32_t xtalfreq);
static const struct bhnd_pmu0_xtaltab0	*bhnd_pmu0_xtaltab_findxf(
					     struct bhnd_pmu_query *query,
					     uint8_t xf);

static bhnd_pmu_op_probe		 bhnd_pmu0_probe;
static bhnd_pmu_op_init			 bhnd_pmu0_init;
static bhnd_pmu_op_clock_get		 bhnd_pmu0_cpuclk;
static bhnd_pmu_op_clock_get		 bhnd_pmu0_alpclk;

/** BHND PMU (hwrev 0) */
static const struct bhnd_pmu_ops bhnd_pmu0_ops = {
	.probe		= bhnd_pmu0_probe,
	.init		= bhnd_pmu0_init,
	.get_bp_clock	= bhnd_pmu0_cpuclk,
	.get_mem_clock	= bhnd_pmu0_cpuclk,
	.get_cpu_clock	= bhnd_pmu0_cpuclk,
	.get_alp_clock	= bhnd_pmu0_alpclk
};

BHND_PMU_OPS_DECLARE(bhnd_pmu0_ops);

/* Supported hardware configurations */
static const struct bhnd_pmu_hwcfg bhnd_pmu0_devices[] = {
	/* BCM4328 */
	{
		.ops	= &bhnd_pmu0_ops,
		.hwreq	= { BHND_CHIP_ID(BCM4328) },
		.quirks	= BPMU_QUIRK_CLKCTL_CCS0,
		.rsrc_map = ((const struct bhnd_pmu_rsrc_map []) {
			{ BHND_PMU_RSRC_PLL_PU,		BHND_PMU0_RES4328_BB_PLL_PU },
			{ BHND_PMU_RSRC_PA_REF_LDO,	BHND_PMU0_RES4328_PA_REF_LDO },
			BHND_PMU_RSRC_MAP_END
		})
	},

	/* BCM5354 */
	{
		.ops	= &bhnd_pmu0_ops,
		.hwreq	= { BHND_CHIP_ID(BCM5354) },
		.quirks	= BPMU_QUIRK_CLKCTL_CCS0,
		.xtalfreq = 25 * 1000 /* 25MHz */,
		.cpufreq  = 240 * 1000 /* 240MHz */,
		.rsrc_map = ((const struct bhnd_pmu_rsrc_map []) {
			{ BHND_PMU_RSRC_PLL_PU,		BHND_PMU0_RES5354_BB_PLL_PU },
			{ BHND_PMU_RSRC_PA_REF_LDO,	BHND_PMU0_RES5354_PA_REF_LDO },
			BHND_PMU_RSRC_MAP_END
		})
	},

	BHND_PMU_HWCFG_END
};

/**
 * PLL initialization table. Assumes 880MHz FVCO.
 */
static const struct bhnd_pmu0_xtaltab0 {
	uint16_t	freq;		/* kHz */
	uint8_t		xf;		/* frequency ID */
	uint8_t		wbint;
	uint32_t	wbfrac;
} bhnd_pmu0_xtaltab0[] = {
	{ 12000,	1,	73,	349525 },
	{ 13000,	2,	67,	725937},
	{ 14400,	3,	61,	116508},
	{ 15360,	4,	57,	305834},
	{ 16200,	5,	54,	336579},
	{ 16800,	6,	52,	399457},
	{ 19200,	7,	45,	873813},
	{ 19800,	8,	44,	466033},
	{ 20000,	9,	44,	0},
	{ 25000,	10,	70,	419430},
	{ 26000,	11,	67,	725937},
	{ 30000,	12,	58,	699050},
	{ 38400,	13,	45,	873813},
	{ 40000,	14,	45,	0},
	{ 0,		0,	0,	0}
};

static int
bhnd_pmu0_probe(const struct bhnd_pmu_hwcfg **hwcfg,
    const struct bhnd_pmu_io *io, void *io_ctx, const struct bhnd_chipid *cid)
{
	const struct bhnd_pmu_hwcfg *cfg;

	cfg = bhnd_pmu_hwcfg_lookup(cid, bhnd_pmu0_devices,
	    sizeof(*bhnd_pmu0_devices));
	if (cfg == NULL)
		return (ENXIO);

	*hwcfg = cfg;
	return (BUS_PROBE_DEFAULT);
}

/**
 * Return the default PLL initialization table entry for @p query.
 * 
 * @param query	The PMU query instance.
 */
static const struct bhnd_pmu0_xtaltab0 *
bhnd_pmu0_xtaltab_get_default(struct bhnd_pmu_query *query)
{
	const struct bhnd_pmu0_xtaltab0	*xt;
	uint32_t			 xtalfreq;

	if (query->hwcfg->xtalfreq == 0)
		xtalfreq = PMU0_XTAL0_DEFAULT_FREQ;
	else
		xtalfreq = query->hwcfg->xtalfreq;

	xt = bhnd_pmu0_xtaltab_findfreq(query, PMU0_XTAL0_DEFAULT_FREQ);
	if (xt == NULL) {
		panic("missing default %" PRIu32 " kHz xtalfreq entry\n",
		    xtalfreq);
	}

	return (xt);
}

/**
 * Return the PLL initialization table entry for the given target frequency
 * (in kHz), or NULL if not found.
 * 
 * @param query		The PMU query instance.
 * @param xtalfreq	The target frequency (in kHz) to look up.
 * 
 * @retval non-NULL	success
 * @retval NULL		if not found
 */
static const struct bhnd_pmu0_xtaltab0 *
bhnd_pmu0_xtaltab_findfreq(struct bhnd_pmu_query *query, uint32_t xtalfreq)
{
	const struct bhnd_pmu0_xtaltab0 *xt;

	/* If the device has a fixed xtalfreq, treat all other xtalfreq values
	 * as undefined */
	if (query->hwcfg->xtalfreq != 0 && xtalfreq != query->hwcfg->xtalfreq)
		return (NULL);

	/* Search for matching entry */
	for (xt = bhnd_pmu0_xtaltab0; xt->freq != 0; xt++) {
		if (xt->freq == xtalfreq)
			return (xt);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return the PLL initialization table entry for the given frequency identifier,
 * or NULL if not found.
 * 
 * @param query	The PMU query instance.
 * @param xf	The frequency identifier to look up.
 * 
 * @retval non-NULL	success
 * @retval NULL		if not found
 */
static const struct bhnd_pmu0_xtaltab0 *
bhnd_pmu0_xtaltab_findxf(struct bhnd_pmu_query *query, uint8_t xf)
{
	const struct bhnd_pmu0_xtaltab0 *xt;

	for (xt = bhnd_pmu0_xtaltab0; xt->freq != 0; xt++) {
		if (xt->xf != xf)
			continue;

		/* If the device has a fixed xtalfreq, filter all other
		 * values */
		if (query->hwcfg->xtalfreq != 0) {
			if (xt->freq != query->hwcfg->xtalfreq)
				continue;
		}

		/* Found */
		return (xt);
	}

	/* Not found */
	return (NULL);
}

static int
bhnd_pmu0_init(struct bhnd_pmu_softc *sc, uint32_t xtalfreq)
{
	const struct bhnd_pmu0_xtaltab0	*xt;
	const struct bhnd_pmu_hwcfg	*hwcfg;
	uint32_t			 pll_data, pll_mask;
	uint32_t			 pmu_ctrl;
	uint32_t			 xf;
	uint8_t				 pll_res;

	hwcfg = sc->query.hwcfg;

	/* Find the PLL table entry for the requested xtalfreq */
	xt = bhnd_pmu0_xtaltab_findfreq(&sc->query, xtalfreq);
	if (xt == NULL) {
		xt = bhnd_pmu0_xtaltab_get_default(&sc->query);
		BHND_PMU_LOG(sc, "unsupported xtalfreq %d kHz requested, using "
		    "default %d kHz\n", xtalfreq, xt->freq);
	}

	BHND_PMU_DEBUG(sc, "XTAL %d.%d MHz (%d)\n", xtalfreq / 1000,
	    xtalfreq % 1000,  xt->xf);

	/* Check current PLL state */
	pmu_ctrl = BHND_PMU_READ_4(sc, BHND_PMU_CTRL);
	xf = BHND_PMU_GET_BITS(pmu_ctrl, BHND_PMU_CTRL_XTALFREQ);
	if (xf == xt->xf) {
#ifdef BCMUSBDEV
		if (sc->cid.chip_id == BHND_CHIPID_BCM4328) {
			bhnd_pmu0_sbclk4328(sc,
			    BHND_PMU0_PLL0_PC0_DIV_ARM_88MHZ);
			return;
		}
#endif	/* BCMUSBDEV */

		BHND_PMU_DEBUG(sc, "PLL already programmed for %d.%d MHz\n",
		         xt->freq / 1000, xt->freq % 1000);
		return (0);
	}

	if (xf != 0) {
		const struct bhnd_pmu0_xtaltab0 *xt_cur;

		/* Find current xtaltab entry */
		xt_cur = bhnd_pmu0_xtaltab_findxf(&sc->query, xf);
		if (xt_cur != NULL) {
			BHND_PMU_DEBUG(sc,
			    "reprogramming PLL for %d.%d MHz (was %d.%dMHz)\n",
			    xt->freq / 1000, xt->freq % 1000,
			    xt_cur->freq / 1000, xt_cur->freq % 1000);
		} else {
			BHND_PMU_DEBUG(sc, "reprogramming PLL for %d.%d MHz "
			    "(was xf=%" PRIu32 ")\n", xt->freq / 1000,
			    xt->freq % 1000, xf);
		}
	} else {
		BHND_PMU_DEBUG(sc, "programming PLL for %d.%d MHz\n",
		    xt->freq / 1000, xt->freq % 1000);
	}

	/* Make sure the PLL is off */
	if (!bhnd_pmu_rsrc_lookup(hwcfg, BHND_PMU_RSRC_PLL_PU, &pll_res)) {
		BHND_PMU_LOG(sc, "no PLL resource found\n");
		return (ENODEV);
	}

	BHND_PMU_AND_4(sc, BHND_PMU_MIN_RES_MASK, ~(BHND_PMURES_BIT(pll_res)));
	BHND_PMU_AND_4(sc, BHND_PMU_MAX_RES_MASK, ~(BHND_PMURES_BIT(pll_res)));

	/* Wait for HT clock to shutdown. */
	if (!BHND_PMU_WAIT_CLKST(sc, 0, BHND_CCS_HTAVAIL)) {
		BHND_PMU_LOG(sc, "timeout waiting for PLL disable\n");
		return (ETIMEDOUT);
	}

	BHND_PMU_DEBUG(sc, "PLL disabled\n");

	/* Write PDIV in pllcontrol[0] */
	if (xt->freq >= BHND_PMU0_PLL0_PC0_PDIV_FREQ) {
		BHND_PMU_PLL_WRITE(sc, BHND_PMU0_PLL0_PLLCTL0,
		    BHND_PMU0_PLL0_PC0_PDIV_MASK, BHND_PMU0_PLL0_PC0_PDIV_MASK);
	} else {
		BHND_PMU_PLL_WRITE(sc, BHND_PMU0_PLL0_PLLCTL0, 0,
		    BHND_PMU0_PLL0_PC0_PDIV_MASK);
	}

	/* Write WILD in pllcontrol[1] */
	pll_data =
	    BHND_PMU_SET_BITS(xt->wbint, BHND_PMU0_PLL0_PC1_WILD_INT) |
	    BHND_PMU_SET_BITS(xt->wbfrac, BHND_PMU0_PLL0_PC1_WILD_FRAC);

	if (xt->wbfrac == 0) {
		pll_data |= BHND_PMU0_PLL0_PC1_STOP_MOD;
	} else {
		pll_data &= ~BHND_PMU0_PLL0_PC1_STOP_MOD;
	}
	
	pll_mask = 
	    BHND_PMU0_PLL0_PC1_WILD_INT_MASK |
	    BHND_PMU0_PLL0_PC1_WILD_FRAC_MASK;

	BHND_PMU_PLL_WRITE(sc, BHND_PMU0_PLL0_PLLCTL1, pll_data, pll_mask);

	/* Write WILD in pllcontrol[2] */
	pll_data = BHND_PMU_SET_BITS(xt->wbint, BHND_PMU0_PLL0_PC2_WILD_INT);
	pll_mask = BHND_PMU0_PLL0_PC2_WILD_INT_MASK;
	BHND_PMU_PLL_WRITE(sc, BHND_PMU0_PLL0_PLLCTL2, pll_data, pll_mask);

	BHND_PMU_DEBUG(sc, "PLL initialized\n");

	/* Write XtalFreq. Set the divisor also. */
	pmu_ctrl = BHND_PMU_READ_4(sc, BHND_PMU_CTRL);
	pmu_ctrl &= ~(BHND_PMU_CTRL_ILP_DIV_MASK|BHND_PMU_CTRL_XTALFREQ_MASK);

	pmu_ctrl |= BHND_PMU_SET_BITS(((xt->freq + 127) / 128) - 1,
	    BHND_PMU_CTRL_ILP_DIV);
	pmu_ctrl |= BHND_PMU_SET_BITS(xt->xf, BHND_PMU_CTRL_XTALFREQ);

	BHND_PMU_WRITE_4(sc, BHND_PMU_CTRL, pmu_ctrl);

	return (0);
}

/* query alp/xtal clock frequency */
static int
bhnd_pmu0_alpclk(struct bhnd_pmu_query *sc, uint32_t *hz)
{
	const struct bhnd_pmu0_xtaltab0	*xt;
	uint32_t			 xf;

	if (sc->hwcfg->alpfreq != 0) {
		*hz = sc->hwcfg->alpfreq * 1000;
		return (0);
	}

	/* Find the frequency in the table */
	xf = BHND_PMU_READ_4(sc, BHND_PMU_CTRL);
	xf = BHND_PMU_GET_BITS(xf, BHND_PMU_CTRL_XTALFREQ);
	for (xt = bhnd_pmu0_xtaltab0; xt->freq; xt++)
		if (xt->xf == xf)
			break;

	/* PLL must be configured before */
	if (xt == NULL || xt->freq == 0) {
		BHND_PMU_LOG(sc, "unsupported frequency: %u", xf);
		return (ENXIO);
	}

	*hz = xt->freq * 1000;
	return (0);
}

/* query CPU clock frequency */
static int
bhnd_pmu0_cpuclk(struct bhnd_pmu_query *sc, uint32_t *hz)
{
	uint32_t tmp, divarm;

	if (sc->hwcfg->cpufreq != 0) {
		*hz = sc->hwcfg->cpufreq * 1000;
		return (0);
	}

	/* Read divarm from pllcontrol[0] */
	tmp = BHND_PMU_PLL_READ(sc, BHND_PMU0_PLL0_PLLCTL0);
	divarm = BHND_PMU_GET_BITS(tmp, BHND_PMU0_PLL0_PC0_DIV_ARM);

	/* Return ARM/SB clock */
	*hz = FVCO_880 / (divarm + BHND_PMU0_PLL0_PC0_DIV_ARM_BASE) * 1000;
	return (0);
}
