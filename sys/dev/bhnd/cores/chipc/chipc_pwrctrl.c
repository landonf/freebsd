/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2010, Broadcom Corporation.
 * All rights reserved.
 * 
 * This file is derived from the siutils.c source distributed with the
 * Asus RT-N16 firmware source code release.
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
 *
 * $Id: siutils.c,v 1.821.2.48 2011-02-11 20:59:28 Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndb/bhndb_pcireg.h>

#include <dev/bhnd/cores/pmu/bhnd_pmuvar.h>
#include <dev/bhnd/cores/pmu/bhnd_pmureg.h>

#include "bhnd_chipc_if.h"
#include "bhnd_pmu_if.h"

#include "chipcreg.h"
#include "chipcvar.h"

/*
 * ChipCommon Power/Clock Control driver.
 * 
 * Provides a bhnd_pmu_if-compatible interface to device clocking and
 * power management on non-PMU chipsets.
 */

struct chipc_pwrctrl_softc {
	device_t		 dev;
	device_t		 chipc_dev;	/**< chipc device */
	device_t		 pci_dev;	/**< host PCI device, or NULL */

	uint16_t		 chipc_rev;	/**< chipc core revision */
	struct bhnd_resource	*res;		/**< chipc register block. */
};

static int
chipc_pwrctrl_probe(device_t dev)
{
	struct chipc_caps	*ccaps;
	device_t		 chipc;

	/* Look for chipc parent */
	chipc = device_get_parent(dev);
	if (device_get_devclass(chipc) != devclass_find("bhnd_chipc"))
		return (ENXIO);

	/* Verify chipc capability flags */
	ccaps = BHND_CHIPC_GET_CAPS(chipc);
	if (ccaps->pmu || !ccaps->pwr_ctrl)
		return (ENXIO);

	device_set_desc(dev, "Broadcom ChipCommon Power Control");
	return (BUS_PROBE_NOWILDCARD);
}

static int
chipc_pwrctrl_attach(device_t dev)
{
	struct chipc_pwrctrl_softc	*sc;
	struct chipc_softc		*chipc_sc;

	sc = device_get_softc(dev);

	sc->dev = dev;
	sc->chipc_dev = device_get_parent(dev);
	sc->chipc_rev = bhnd_get_hwrev(sc->chipc_dev);

	/* Fetch core register block from ChipCommon parent */
	chipc_sc = device_get_softc(sc->chipc_dev);
	sc->res = chipc_sc->core;

	/* On early PCI devices, the slowclk registers are mapped via PCI
	 * config space */
	if (sc->chipc_rev < 6) {
		sc->pci_dev = bhnd_find_bridge_root(dev, devclass_find("pci"));
		if (sc->pci_dev == NULL) {
			device_printf(dev,
			    "parent pci bridge device not found\n");
			return (ENXIO);
		}
	}

	return (0);
}

static int
chipc_pwrctrl_detach(device_t dev)
{
	// TODO
	return (0);
}

static int
chipc_pwrctrl_suspend(device_t dev)
{
	// TODO
	return (0);
}

static int
chipc_pwrctrl_resume(device_t dev)
{
	// TODO
	return (0);
}

static uint32_t
chipc_pwrctrl_factor6(uint32_t x)
{
	switch (x) {
	case CHIPC_F6_2:	
		return (2);
	case CHIPC_F6_3:	
		return (3);
	case CHIPC_F6_4:	
		return (4);
	case CHIPC_F6_5:	
		return (5);
	case CHIPC_F6_6:
		return (6);
	case CHIPC_F6_7:
		return (7);
	default:
		return (0);
	}
}

/* Calculate the clock speed given a set of clockcontrol values */
static uint32_t
si_clock_rate(uint32_t pll_type, uint32_t n, uint32_t m)
{
	uint32_t clk_base;
	uint32_t n1, n2, clock, m1, m2, m3, mc;

	n1 = CHIPC_GET_BITS(n, CHIPC_CN_N1);
	n2 = CHIPC_GET_BITS(n, CHIPC_CN_N2);

	switch (pll_type) {
	case CHIPC_PLL_TYPE1:
	case CHIPC_PLL_TYPE3:
	case CHIPC_PLL_TYPE4:
	case CHIPC_PLL_TYPE7:
		n1 = chipc_pwrctrl_factor6(n1);
		n2 += CHIPC_F5_BIAS;
		break;

	case CHIPC_PLL_TYPE2:
		n1 += CHIPC_T2_BIAS;
		n2 += CHIPC_T2_BIAS;
		KASSERT(n1 >= 2 && n1 <= 7, ("invalid n1 value"));
		KASSERT(n2 >= 5 && n2 <= 23, ("invalid n2 value"));
		break;
		
	case CHIPC_PLL_TYPE5:
		return (100000000);

	case CHIPC_PLL_TYPE6:
		if (m & CHIPC_T6_MMASK)
			return (CHIPC_T6_M1);
		else
			return (CHIPC_T6_M0);

	default:
		printf("unsupported PLL type %u\n", pll_type);
		return (0);
	}

	/* PLL types 3 and 7 use BASE2 (25Mhz) */
	if (pll_type == CHIPC_PLL_TYPE3 || pll_type == CHIPC_PLL_TYPE7) {
		clk_base = CHIPC_CLOCK_BASE2;
	} else {
		clk_base = CHIPC_CLOCK_BASE1;
	}

	clock = clk_base * n1 * n2;

	if (clock == 0)
		return (0);

	m1 = CHIPC_GET_BITS(m, CHIPC_M1);
	m2 = CHIPC_GET_BITS(m, CHIPC_M2);
	m3 = CHIPC_GET_BITS(m, CHIPC_M3);
	mc = CHIPC_GET_BITS(m, CHIPC_MC);

	switch (pll_type) {
	case CHIPC_PLL_TYPE1:
	case CHIPC_PLL_TYPE3:
	case CHIPC_PLL_TYPE4:
	case CHIPC_PLL_TYPE7:
		m1 = chipc_pwrctrl_factor6(m1);
		if (pll_type == CHIPC_PLL_TYPE1 || pll_type == CHIPC_PLL_TYPE3)
			m2 += CHIPC_F5_BIAS;
		else
			m2 = chipc_pwrctrl_factor6(m2);

		m3 = chipc_pwrctrl_factor6(m3);

		switch (mc) {
		case CHIPC_MC_BYPASS:	
			return (clock);
		case CHIPC_MC_M1:	
			return (clock / m1);
		case CHIPC_MC_M1M2:	
			return (clock / (m1 * m2));
		case CHIPC_MC_M1M2M3:
			return (clock / (m1 * m2 * m3));
		case CHIPC_MC_M1M3:
			return (clock / (m1 * m3));
		default:
			printf("unsupported pwrctl mc %#x\n", mc);
			return (0);
		}
	case CHIPC_PLL_TYPE2:
		m1 += CHIPC_T2_BIAS;
		m2 += CHIPC_T2M2_BIAS;
		m3 += CHIPC_T2_BIAS;
		KASSERT(m1 >= 2 && m1 <= 7, ("invalid m1 value"));
		KASSERT(m2 >= 3 && m2 <= 10, ("invalid m2 value"));
		KASSERT(m3 >= 2 && m3 <= 7, ("invalid m3 value"));

		if ((mc & CHIPC_T2MC_M1BYP) == 0)
			clock /= m1;
		if ((mc & CHIPC_T2MC_M2BYP) == 0)
			clock /= m2;
		if ((mc & CHIPC_T2MC_M3BYP) == 0)
			clock /= m3;

		return (clock);
	default:
		panic("unhandled PLL type %u\n", pll_type);
	}
}

static uint32_t
si_clock(struct chipc_pwrctrl_softc *sc)
{
	struct chipc_caps	*ccaps;
	uint32_t 		 n, m;
	uint32_t 		 rate;

	ccaps = BHND_CHIPC_GET_CAPS(sc->chipc_dev);

	n = bhnd_bus_read_4(sc->res, CHIPC_CLKC_N);
	switch (ccaps->pll_type) {
	case CHIPC_PLL_TYPE6:
		m = bhnd_bus_read_4(sc->res, CHIPC_CLKC_M3);
		break;
	case CHIPC_PLL_TYPE3:
		m = bhnd_bus_read_4(sc->res, CHIPC_CLKC_M2);
		break;
	default:
		m = bhnd_bus_read_4(sc->res, CHIPC_CLKC_SB);
		break;
	}

	/* calculate rate */
	rate = si_clock_rate(ccaps->pll_type, n, m);

	if (ccaps->pll_type == CHIPC_PLL_TYPE3)
		rate /= 2;

	return (rate);
}

static uint32_t
si_alp_clock(struct chipc_pwrctrl_softc *sc)
{
	return (BHND_PMU_ALP_CLOCK);
}

static uint32_t
si_ilp_clock(struct chipc_pwrctrl_softc *sc)
{
	return (BHND_PMU_ILP_CLOCK);
}


/* return the slow clock source - (CHIPC_SCC_SS_*)  */
static uint32_t
si_slowclk_src(struct chipc_pwrctrl_softc *sc)
{
	uint32_t clkreg;

	if (sc->chipc_rev < 6) {
		KASSERT(sc->pci_dev != NULL, ("missing PCI bridge"));

		clkreg = pci_read_config(sc->pci_dev, BHNDB_PCI_GPIO_OUT, 4);
		if (clkreg & BHNDB_PCI_GPIO_SCS)
			return (CHIPC_SCC_SS_PCI);
		else
			return (CHIPC_SCC_SS_XTAL);
	} else if (sc->chipc_rev < 10) {
		clkreg = bhnd_bus_read_4(sc->res, CHIPC_PLL_SLOWCLK_CTL);
		return (clkreg & CHIPC_SCC_SS_MASK);
	} else {
		/* Instaclock */
		return (CHIPC_SCC_SS_XTAL);
	}
}

/* return the ILP (slowclock) min or max frequency */
static uint32_t
si_slowclk_freq(struct chipc_pwrctrl_softc *sc, bool max_freq)
{
	uint32_t slowclk;
	uint32_t div;
	uint32_t hz;

	slowclk = si_slowclk_src(sc);

	/* Determine clock divisor */
	if (sc->chipc_rev < 6) {
		if (slowclk == CHIPC_SCC_SS_PCI)
			div = 64;
		else
			div = 32;
	} else if (sc->chipc_rev < 10) {
		div = bhnd_bus_read_4(sc->res, CHIPC_PLL_SLOWCLK_CTL);
		div = CHIPC_GET_BITS(div, CHIPC_SCC_CD);
		div *= 4;
	} else {
		/* Chipc rev 10 is InstaClock */
		if (max_freq) {
			div = 1;
		} else {
			div = bhnd_bus_read_4(sc->res, CHIPC_SYS_CLK_CTL);
			div = CHIPC_GET_BITS(div, CHIPC_SYCC_CD);
			div = 4 * (div + 1);
		}
	}

	/* Determine clock frequency */
	switch (slowclk) {
	case CHIPC_SCC_SS_LPO:
		hz = max_freq ? CHIPC_LPOMAXFREQ : CHIPC_LPOMINFREQ;
		break;
	case CHIPC_SCC_SS_XTAL:
		hz = max_freq ? CHIPC_XTALMAXFREQ : CHIPC_XTALMINFREQ;
		break;
	case CHIPC_SCC_SS_PCI:
		hz = max_freq ? CHIPC_PCIMAXFREQ : CHIPC_PCIMINFREQ;
		break;
	default:
		device_printf(sc->dev, "unknown slowclk source %#x\n", slowclk);
		return (0);
	}

	return (hz / div);
}

/* initialize power control delay registers */
static void
si_clkctl_init(struct chipc_pwrctrl_softc *sc)
{
	uint32_t clkctl;
	uint32_t pll_delay, slowclk, slowmaxfreq;
	uint32_t pll_on_delay, fref_sel_delay;

	pll_delay = CHIPC_PLL_DELAY;

	/* set all Instaclk chip ILP to 1 MHz */
	if (sc->chipc_rev >= 10) {
		clkctl = (CHIPC_ILP_DIV_1MHZ << CHIPC_SYCC_CD_SHIFT);
		clkctl &= CHIPC_SYCC_CD_MASK;
		bhnd_bus_write_4(sc->res, CHIPC_SYS_CLK_CTL, clkctl);
	}


	/* 
	 * Initialize PLL/FREF delays.
	 * 
	 * If the slow clock is not sourced by the xtal, include the
	 * delay required to bring it up.
	 */
	slowclk = si_slowclk_src(sc);
	if (slowclk != CHIPC_SCC_SS_XTAL)
		pll_delay += CHIPC_XTAL_ON_DELAY;

	/* Starting with 4318 it is ILP that is used for the delays */
	slowmaxfreq = si_slowclk_freq(sc, (sc->chipc_rev >= 10) ? false : true);

	pll_on_delay = ((slowmaxfreq * pll_delay) + 999999) / 1000000;
	fref_sel_delay = ((slowmaxfreq * CHIPC_FREF_DELAY) + 999999) / 1000000;

	bhnd_bus_write_4(sc->res, CHIPC_PLL_ON_DELAY, pll_on_delay);
	bhnd_bus_write_4(sc->res, CHIPC_PLL_FREFSEL_DELAY, fref_sel_delay);
}

/* return the value suitable for writing to the dot11 core
 * FAST_PWRUP_DELAY register */
static uint16_t
si_clkctl_fast_pwrup_delay(struct chipc_pwrctrl_softc *sc)
{
	uint32_t pll_on_delay, slowminfreq;
	uint16_t fpdelay;

	fpdelay = 0;

	slowminfreq = si_slowclk_freq(sc, false);

	pll_on_delay = bhnd_bus_read_4(sc->res, CHIPC_PLL_ON_DELAY) + 2;
	pll_on_delay *= 1000000;
	pll_on_delay += (slowminfreq - 1);
	fpdelay = pll_on_delay / slowminfreq;

	return (fpdelay);
}

static bool	_si_clkctl_cc(struct chipc_pwrctrl_softc *sc, bhnd_clock clock);

/*
 *  clock control policy function throught chipcommon
 *
 *    set dynamic clk control mode (forceslow, forcefast, dynamic)
 *    returns true if we are forcing fast clock
 *    this is a wrapper over the next internal function
 *      to allow flexible policy settings for outside caller
 */
static bool
si_clkctl_cc(struct chipc_pwrctrl_softc *sc, bhnd_clock clock)
{
	const struct bhnd_chipid	*cid;
	bhnd_devclass_t			 hostb_class;
	device_t			 hostb_dev;
	bool				 force_ht;

	cid = bhnd_get_chipid(sc->chipc_dev);

	/* chipcommon cores prior to rev6 don't support dynamic clock control */
	if (sc->chipc_rev < 6)
		return (false);

	hostb_class = BHND_DEVCLASS_INVALID;
	hostb_dev = bhnd_find_hostb_device(device_get_parent(sc->chipc_dev));
	if (hostb_dev != NULL)
		hostb_class = bhnd_get_class(hostb_dev);

	/* Ignore requests for non-HT clocking? */
	force_ht = false;
	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4311:
		if (cid->chip_rev > 1)
			break;
		if (hostb_class != BHND_DEVCLASS_PCIE)
			break;

		force_ht = true;
		break;

	case BHND_CHIPID_BCM4321:
		if (hostb_class != BHND_DEVCLASS_PCIE ||
		    hostb_class != BHND_DEVCLASS_PCI)
			break;
		force_ht = true;
		break;

	case BHND_CHIPID_BCM4716:
		if (hostb_class != BHND_DEVCLASS_PCIE)
			break;
		force_ht = true;
		break;

	default:
		break;
	}
		
	if (force_ht)
		return (clock == BHND_CLOCK_HT);

	return (_si_clkctl_cc(sc, clock));
}

static void
si_clkctl_xtal(struct chipc_pwrctrl_softc *sc, bool enable)
{
	KASSERT(sc->chipc_rev >= 6 && sc->chipc_rev <= 9,
	    ("force_xtal unsupported on rev %hu", sc->chipc_rev));

	if (sc->pci_dev == NULL)
		return;

	// TODO: We need to force the XTAL on/off via PCI config space.
}

/* clk control mechanism through chipcommon, no policy checking */
static bool
_si_clkctl_cc(struct chipc_pwrctrl_softc *sc, bhnd_clock clock)
{
	uint32_t	scc;

	/* chipcommon cores prior to rev6 don't support dynamic clock control */
	if (sc->chipc_rev < 6)
		return (false);

	/* Chips with ccrev 10 are EOL and they don't have SYCC_HR used below */
	if (sc->chipc_rev == 10)
		return (false);

	scc = bhnd_bus_read_4(sc->res, CHIPC_PLL_SLOWCLK_CTL);

	switch (clock) {
	case BHND_CLOCK_HT:	/* FORCEHT, fast (pll) clock */
		if (sc->chipc_rev >= 6 && sc->chipc_rev <= 9) {
			scc &= ~(CHIPC_SCC_XC | CHIPC_SCC_FS | CHIPC_SCC_IP);
			scc |= CHIPC_SCC_IP;

			/* force xtal back on before clearing SCC_DYN_XTAL.. */
			si_clkctl_xtal(sc, true);
		} else if (sc->chipc_rev >= 10) {
			scc |= CHIPC_SYCC_HR;
		}

		bhnd_bus_write_4(sc->res, CHIPC_PLL_SLOWCLK_CTL, scc);
		DELAY(CHIPC_PLL_DELAY);
		break;

	case BHND_CLOCK_DYN:	/* enable dynamic clock control */
		if (sc->chipc_rev >= 6 && sc->chipc_rev <= 9) {
			scc &= ~(CHIPC_SCC_FS | CHIPC_SCC_IP | CHIPC_SCC_XC);
			if ((scc & CHIPC_SCC_SS_MASK) != CHIPC_SCC_SS_XTAL)
				scc |= CHIPC_SCC_XC;
	
			bhnd_bus_write_4(sc->res, CHIPC_PLL_SLOWCLK_CTL, scc);

			/* for dynamic control, we have to release our xtal_pu
			 * "force on" */
			if (scc & CHIPC_SCC_XC)
				si_clkctl_xtal(sc, false);
		} else if (sc->chipc_rev >= 10) {
			/* Instaclock */
			scc &= ~CHIPC_SYCC_HR;
			bhnd_bus_write_4(sc->res, CHIPC_SYS_CLK_CTL, scc);
		}

		break;

	default:
		// TODO: return real error values
		panic("unsupported request");
	}

	return (clock == BHND_CLOCK_HT);
}

static device_method_t chipc_pwrctrl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		chipc_pwrctrl_probe),
	DEVMETHOD(device_attach,	chipc_pwrctrl_attach),
	DEVMETHOD(device_detach,	chipc_pwrctrl_detach),
	DEVMETHOD(device_suspend,	chipc_pwrctrl_suspend),
	DEVMETHOD(device_resume,	chipc_pwrctrl_resume),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pmu, chipc_pwrctrl_driver, chipc_pwrctrl_methods,
    sizeof(struct chipc_pwrctrl_softc));
EARLY_DRIVER_MODULE(chipc_pwrctrl, bhnd_chipc, chipc_pwrctrl_driver,
    bhnd_pmu_devclass, NULL, NULL, BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);

MODULE_DEPEND(chipc_pwrctrl, bhnd, 1, 1, 1);
MODULE_VERSION(chipc_pwrctrl, 1);
