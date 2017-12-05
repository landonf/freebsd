/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2017 The FreeBSD Foundation
 * All rights reserved.
 * 
 * Portions of this software were developed by Landon Fuller
 * under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_llc.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_phy.h>
#include <net80211/ieee80211_ratectl.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/siba/sibareg.h>

#include <dev/bhnd/cores/chipc/chipc.h>
#include <dev/bhnd/cores/pci/bhnd_pcireg.h>
#include <dev/bhnd/cores/pmu/bhnd_pmu.h>

#include "bhnd_nvram_map.h"

#include "if_bwn_siba_compat.h"

static int		bwn_bhnd_populate_nvram_data(device_t dev,
			    struct bwn_bhnd_ctx *ctx);

#define	BWN_ASSERT_VALID_REG(_dev, _offset)				\
	KASSERT(!bwn_bhnd_is_siba_reg(_dev, _offset),			\
	    ("%s: accessing siba-specific register %#jx", __FUNCTION__,	\
		(uintmax_t)(_offset)));

static int
bwn_bhnd_bus_ops_init(device_t dev)
{
	struct bwn_bhnd_ctx	*ctx;
	struct bwn_softc	*sc;
	const struct chipc_caps	*ccaps;
	int			 error;

	sc = device_get_softc(dev);
	ctx = NULL;

	/* Allocate our context */
	ctx = malloc(sizeof(struct bwn_bhnd_ctx), M_DEVBUF, M_WAITOK|M_ZERO);

	/* Locate the ChipCommon device */
	ctx->chipc_dev = bhnd_retain_provider(dev, BHND_SERVICE_CHIPC);
	if (ctx->chipc_dev == NULL) {
		device_printf(dev, "ChipCommon not found\n");
		error = ENXIO;
		goto failed;
	}

	/* Locate the PMU device (if any) */
	ccaps = BHND_CHIPC_GET_CAPS(ctx->chipc_dev);
	if (ccaps->pmu) {
		ctx->pmu_dev = bhnd_retain_provider(dev, BHND_SERVICE_PMU);
		if (ctx->pmu_dev == NULL) {
			device_printf(dev, "PMU not found\n");
			error = ENXIO;
			goto failed;
		}
	}

	/* Populate NVRAM data */
	if ((error = bwn_bhnd_populate_nvram_data(dev, ctx)))
		goto failed;

	/* Initialize bwn_softc */
	sc->sc_bus_ctx = ctx;
	return (0);

failed:
	if (ctx != NULL) {
		if (ctx->chipc_dev != NULL) {
			bhnd_release_provider(dev, ctx->chipc_dev,
			    BHND_SERVICE_CHIPC);
		}

		if (ctx->pmu_dev != NULL) {
			bhnd_release_provider(dev, ctx->pmu_dev,
			    BHND_SERVICE_PMU);
		}

		free(ctx, M_DEVBUF);
	}

	return (error);
}

static void
bwn_bhnd_bus_ops_fini(device_t dev)
{
	struct bwn_bhnd_ctx	*ctx;
	struct bwn_softc	*sc;

	sc = device_get_softc(dev);
	ctx = sc->sc_bus_ctx;

	bhnd_release_provider(dev, ctx->chipc_dev, BHND_SERVICE_CHIPC);

	if (ctx->pmu_dev != NULL)
		bhnd_release_provider(dev, ctx->pmu_dev, BHND_SERVICE_PMU);

	free(ctx, M_DEVBUF);
	sc->sc_bus_ctx = NULL;
}

/* Populate SPROM values from NVRAM */
static int
bwn_bhnd_populate_nvram_data(device_t dev, struct bwn_bhnd_ctx *ctx)
{
	int	error;

	/* Fetch SROM revision */
	error = bhnd_nvram_getvar_uint8(dev, BHND_NVAR_SROMREV, &ctx->sromrev);
	if (error) {
		device_printf(dev, "error reading %s: %d\n", BHND_NVAR_SROMREV,
		    error);
		return (error);
	}

	/* Fetch pa0maxpwr; bwn(4) expects to be able to modify it */
	if ((ctx->sromrev >= 1 && ctx->sromrev <= 3) ||
	    (ctx->sromrev >= 8 && ctx->sromrev <= 10))
	{
		error = bhnd_nvram_getvar_uint8(dev, BHND_NVAR_PA0MAXPWR,
		     &ctx->pa0maxpwr);
		if (error) {
			device_printf(dev, "error reading %s: %d\n",
			    BHND_NVAR_PA0MAXPWR, error);
			return (error);
		}
	}

	return (0);
}

/**
 * Return the PCI bridge root device.
 * 
 * Will panic if a PCI bridge root device is not found.
 */
static device_t
bwn_bhnd_get_pci_dev(device_t dev)
{	device_t bridge_root;

	bridge_root = bhnd_find_bridge_root(dev, devclass_find("pci"));
	if (bridge_root == NULL)
		panic("not a PCI device");

	return (bridge_root);
}

/*
 * siba_get_pci_vendor()
 *
 * Referenced by:
 *   bwn_sprom_bugfixes()
 */
static uint16_t
bhnd_compat_get_pci_vendor(device_t dev)
{
	return (pci_get_vendor(bwn_bhnd_get_pci_dev(dev)));
}

/*
 * siba_get_pci_device()
 *
 * Referenced by:
 *   bwn_attach()
 *   bwn_attach_core()
 *   bwn_nphy_op_prepare_structs()
 *   bwn_sprom_bugfixes()
 */
static uint16_t
bhnd_compat_get_pci_device(device_t dev)
{
	return (pci_get_device(bwn_bhnd_get_pci_dev(dev)));
}

/*
 * siba_get_type()
 *
 * Referenced by:
 *   bwn_core_init()
 *   bwn_dma_attach()
 *   bwn_nphy_op_prepare_structs()
 *   bwn_sprom_bugfixes()
 */
static enum siba_type
bhnd_compat_get_type(device_t dev)
{
	device_t		bus, hostb;
	bhnd_devclass_t		hostb_devclass;

	bus = device_get_parent(dev);
	hostb = bhnd_bus_find_hostb_device(bus);

	if (hostb == NULL)
		return (SIBA_TYPE_SSB);

	hostb_devclass = bhnd_get_class(hostb);
	switch (hostb_devclass) {
	case BHND_DEVCLASS_PCCARD:
		return (SIBA_TYPE_PCMCIA);
	case BHND_DEVCLASS_PCI:
	case BHND_DEVCLASS_PCIE:
		return (SIBA_TYPE_PCI);
	default:
		panic("unsupported hostb devclass: %d\n", hostb_devclass);
	}
}

/*
 * siba_sprom_get_ccode()
 *
 * Referenced by:
 *   bwn_phy_g_switch_chan()
 */
static uint8_t
bhnd_compat_sprom_get_ccode(device_t dev)
{
	/* This has been replaced with 'ccode' in later SPROM
	 * revisions, but this API is only called on devices with
	 * spromrev 1. */
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_CC);
}

/*
 * siba_sprom_get_ant_a()
 *
 * Referenced by:
 *   bwn_antenna_sanitize()
 */
static uint8_t
bhnd_compat_sprom_get_ant_a(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_AA5G);
}

/*
 * siba_sprom_get_ant_bg()
 *
 * Referenced by:
 *   bwn_antenna_sanitize()
 */
static uint8_t
bhnd_compat_sprom_get_ant_bg(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_AA2G);
}

/*
 * siba_sprom_get_maxpwr_bg()
 *
 * Referenced by:
 *   bwn_phy_g_recalc_txpwr()
 */
static uint16_t
bhnd_compat_sprom_get_maxpwr_bg(device_t dev)
{
	return (bwn_bhnd_get_ctx(dev)->pa0maxpwr);
}

/*
 * siba_sprom_set_maxpwr_bg()
 *
 * Referenced by:
 *   bwn_phy_g_recalc_txpwr()
 */
static void
bhnd_compat_sprom_set_maxpwr_bg(device_t dev, uint16_t t)
{
	KASSERT(t <= UINT8_MAX, ("invalid maxpwr value %hu", t));
	bwn_bhnd_get_ctx(dev)->pa0maxpwr = t;
}

/*
 * siba_sprom_get_rxpo2g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rxpo2g(device_t dev)
{
	/* Should be signed, but bwn(4) expects an unsigned value */
	BWN_BHND_NVRAM_RETURN_VAR(dev, int8, BHND_NVAR_RXPO2G);
}

/*
 * siba_sprom_get_rxpo5g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rxpo5g(device_t dev)
{
	/* Should be signed, but bwn(4) expects an unsigned value */
	BWN_BHND_NVRAM_RETURN_VAR(dev, int8, BHND_NVAR_RXPO5G);
}

/*
 * siba_sprom_get_tri2g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_tri2g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TRI2G);
}

/*
 * siba_sprom_get_tri5gl()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_tri5gl(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TRI5GL);
}

/*
 * siba_sprom_get_tri5g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_tri5g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TRI5G);
}

/*
 * siba_sprom_get_tri5gh()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_tri5gh(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TRI5GH);
}

/*
 * siba_sprom_get_rssisav2g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rssisav2g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_RSSISAV2G);
}

/*
 * siba_sprom_get_rssismc2g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rssismc2g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_RSSISMC2G);
}

/*
 * siba_sprom_get_rssismf2g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rssismf2g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_RSSISMF2G);
}

/*
 * siba_sprom_get_bxa2g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_bxa2g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_BXA2G);
}

/*
 * siba_sprom_get_rssisav5g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rssisav5g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_RSSISAV5G);
}

/*
 * siba_sprom_get_rssismc5g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rssismc5g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_RSSISMC5G);
}

/*
 * siba_sprom_get_rssismf5g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_rssismf5g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_RSSISMF5G);
}

/*
 * siba_sprom_get_bxa5g()
 *
 * Referenced by:
 *   bwn_phy_lp_readsprom()
 */
static uint8_t
bhnd_compat_sprom_get_bxa5g(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_BXA5G);
}

/*
 * siba_sprom_get_cck2gpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static uint16_t
bhnd_compat_sprom_get_cck2gpo(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint16, BHND_NVAR_CCK2GPO);
}

/*
 * siba_sprom_get_ofdm2gpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static uint32_t
bhnd_compat_sprom_get_ofdm2gpo(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint32, BHND_NVAR_OFDM2GPO);
}

/*
 * siba_sprom_get_ofdm5glpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static uint32_t
bhnd_compat_sprom_get_ofdm5glpo(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint32, BHND_NVAR_OFDM5GLPO);
}

/*
 * siba_sprom_get_ofdm5gpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static uint32_t
bhnd_compat_sprom_get_ofdm5gpo(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint32, BHND_NVAR_OFDM5GPO);
}

/*
 * siba_sprom_get_ofdm5ghpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static uint32_t
bhnd_compat_sprom_get_ofdm5ghpo(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint32, BHND_NVAR_OFDM5GHPO);
}

/*
 * siba_sprom_get_fem_2ghz_tssipos()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_ctl_setup()
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_tssipos(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TSSIPOS2G);
}

/*
 * siba_sprom_get_fem_2ghz_extpa_gain()
 *
 * Referenced by:
 *   bwn_nphy_op_prepare_structs()
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_extpa_gain(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_EXTPAGAIN2G);
}

/*
 * siba_sprom_get_fem_2ghz_pdet_range()
 *
 * Referenced by:
 *   bwn_nphy_workarounds_rev3plus()
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_pdet_range(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_PDETRANGE2G);
}

/*
 * siba_sprom_get_fem_2ghz_tr_iso()
 *
 * Referenced by:
 *   bwn_nphy_get_gain_ctl_workaround_ent()
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_tr_iso(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TRISO2G);
}

/*
 * siba_sprom_get_fem_2ghz_antswlut()
 *
 * Referenced by:
 *   bwn_nphy_tables_init_rev3()
 *   bwn_nphy_tables_init_rev7_volatile()
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_antswlut(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_ANTSWCTL2G);
}

/*
 * siba_sprom_get_fem_5ghz_extpa_gain()
 *
 * Referenced by:
 *   bwn_nphy_get_tx_gain_table()
 *   bwn_nphy_op_prepare_structs()
 */
static uint8_t
bhnd_compat_sprom_get_fem_5ghz_extpa_gain(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_EXTPAGAIN5G);
}

/*
 * siba_sprom_get_fem_5ghz_pdet_range()
 *
 * Referenced by:
 *   bwn_nphy_workarounds_rev3plus()
 */
static uint8_t
bhnd_compat_sprom_get_fem_5ghz_pdet_range(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_PDETRANGE5G);
}

/*
 * siba_sprom_get_fem_5ghz_antswlut()
 *
 * Referenced by:
 *   bwn_nphy_tables_init_rev3()
 *   bwn_nphy_tables_init_rev7_volatile()
 */
static uint8_t
bhnd_compat_sprom_get_fem_5ghz_antswlut(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_ANTSWCTL5G);
}

/*
 * siba_sprom_get_txpid_2g_0()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_2g_0(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID2GA0);
}

/*
 * siba_sprom_get_txpid_2g_1()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_2g_1(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID2GA1);
}

/*
 * siba_sprom_get_txpid_5gl_0()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gl_0(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID5GLA0);
}

/*
 * siba_sprom_get_txpid_5gl_1()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gl_1(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID5GLA1);
}

/*
 * siba_sprom_get_txpid_5g_0()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5g_0(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID5GA0);
}

/*
 * siba_sprom_get_txpid_5g_1()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5g_1(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID5GA1);
}

/*
 * siba_sprom_get_txpid_5gh_0()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gh_0(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID5GHA0);
}

/*
 * siba_sprom_get_txpid_5gh_1()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_fix()
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gh_1(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint8, BHND_NVAR_TXPID5GHA1);
}

/*
 * siba_sprom_get_stbcpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static uint16_t
bhnd_compat_sprom_get_stbcpo(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint16, BHND_NVAR_STBCPO);
}

/*
 * siba_sprom_get_cddpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static uint16_t
bhnd_compat_sprom_get_cddpo(device_t dev)
{
	BWN_BHND_NVRAM_RETURN_VAR(dev, uint16, BHND_NVAR_CDDPO);
}

/*
 * siba_sprom_get_mcs2gpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static int
bhnd_compat_sprom_get_mcs2gpo(device_t dev, uint16_t *c)
{
	static const char *varnames[] = {
		BHND_NVAR_MCS2GPO0,
		BHND_NVAR_MCS2GPO1,
		BHND_NVAR_MCS2GPO2,
		BHND_NVAR_MCS2GPO3,
		BHND_NVAR_MCS2GPO4,
		BHND_NVAR_MCS2GPO5,
		BHND_NVAR_MCS2GPO6,
		BHND_NVAR_MCS2GPO7
	};

	for (size_t i = 0; i < nitems(varnames); i++) {
		const char *name = varnames[i];
		BWN_BHND_NVRAM_FETCH_VAR(dev, uint16, name, &c[i]);
	}

	return (0);
}

/*
 * siba_sprom_get_mcs5glpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static int
bhnd_compat_sprom_get_mcs5glpo(device_t dev, uint16_t *c)
{
	static const char *varnames[] = {
		BHND_NVAR_MCS5GLPO0,
		BHND_NVAR_MCS5GLPO1,
		BHND_NVAR_MCS5GLPO2,
		BHND_NVAR_MCS5GLPO3,
		BHND_NVAR_MCS5GLPO4,
		BHND_NVAR_MCS5GLPO5,
		BHND_NVAR_MCS5GLPO6,
		BHND_NVAR_MCS5GLPO7
	};

	for (size_t i = 0; i < nitems(varnames); i++) {
		const char *name = varnames[i];
		BWN_BHND_NVRAM_FETCH_VAR(dev, uint16, name, &c[i]);
	}

	return (0);
}

/*
 * siba_sprom_get_mcs5gpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static int
bhnd_compat_sprom_get_mcs5gpo(device_t dev, uint16_t *c)
{
	static const char *varnames[] = {
		BHND_NVAR_MCS5GPO0,
		BHND_NVAR_MCS5GPO1,
		BHND_NVAR_MCS5GPO2,
		BHND_NVAR_MCS5GPO3,
		BHND_NVAR_MCS5GPO4,
		BHND_NVAR_MCS5GPO5,
		BHND_NVAR_MCS5GPO6,
		BHND_NVAR_MCS5GPO7
	};

	for (size_t i = 0; i < nitems(varnames); i++) {
		const char *name = varnames[i];
		BWN_BHND_NVRAM_FETCH_VAR(dev, uint16, name, &c[i]);
	}

	return (0);
}

/*
 * siba_sprom_get_mcs5ghpo()
 *
 * Referenced by:
 *   bwn_ppr_load_max_from_sprom()
 */
static int
bhnd_compat_sprom_get_mcs5ghpo(device_t dev, uint16_t *c)
{
	static const char *varnames[] = {
		BHND_NVAR_MCS5GHPO0,
		BHND_NVAR_MCS5GHPO1,
		BHND_NVAR_MCS5GHPO2,
		BHND_NVAR_MCS5GHPO3,
		BHND_NVAR_MCS5GHPO4,
		BHND_NVAR_MCS5GHPO5,
		BHND_NVAR_MCS5GHPO6,
		BHND_NVAR_MCS5GHPO7
	};

	for (size_t i = 0; i < nitems(varnames); i++) {
		const char *name = varnames[i];
		BWN_BHND_NVRAM_FETCH_VAR(dev, uint16, name, &c[i]);
	}

	return (0);
}

const struct bwn_bus_ops bwn_bhnd_bus_ops = {
	.init				= bwn_bhnd_bus_ops_init,
	.fini				= bwn_bhnd_bus_ops_fini,
	.get_pci_vendor			= bhnd_compat_get_pci_vendor,
	.get_pci_device			= bhnd_compat_get_pci_device,
	.get_type			= bhnd_compat_get_type,
	.sprom_get_ccode		= bhnd_compat_sprom_get_ccode,
	.sprom_get_ant_a		= bhnd_compat_sprom_get_ant_a,
	.sprom_get_ant_bg		= bhnd_compat_sprom_get_ant_bg,
	.sprom_get_maxpwr_bg		= bhnd_compat_sprom_get_maxpwr_bg,
	.sprom_set_maxpwr_bg		= bhnd_compat_sprom_set_maxpwr_bg,
	.sprom_get_rxpo2g		= bhnd_compat_sprom_get_rxpo2g,
	.sprom_get_rxpo5g		= bhnd_compat_sprom_get_rxpo5g,
	.sprom_get_tri2g		= bhnd_compat_sprom_get_tri2g,
	.sprom_get_tri5gl		= bhnd_compat_sprom_get_tri5gl,
	.sprom_get_tri5g		= bhnd_compat_sprom_get_tri5g,
	.sprom_get_tri5gh		= bhnd_compat_sprom_get_tri5gh,
	.sprom_get_rssisav2g		= bhnd_compat_sprom_get_rssisav2g,
	.sprom_get_rssismc2g		= bhnd_compat_sprom_get_rssismc2g,
	.sprom_get_rssismf2g		= bhnd_compat_sprom_get_rssismf2g,
	.sprom_get_bxa2g		= bhnd_compat_sprom_get_bxa2g,
	.sprom_get_rssisav5g		= bhnd_compat_sprom_get_rssisav5g,
	.sprom_get_rssismc5g		= bhnd_compat_sprom_get_rssismc5g,
	.sprom_get_rssismf5g		= bhnd_compat_sprom_get_rssismf5g,
	.sprom_get_bxa5g		= bhnd_compat_sprom_get_bxa5g,
	.sprom_get_cck2gpo		= bhnd_compat_sprom_get_cck2gpo,
	.sprom_get_ofdm2gpo		= bhnd_compat_sprom_get_ofdm2gpo,
	.sprom_get_ofdm5glpo		= bhnd_compat_sprom_get_ofdm5glpo,
	.sprom_get_ofdm5gpo		= bhnd_compat_sprom_get_ofdm5gpo,
	.sprom_get_ofdm5ghpo		= bhnd_compat_sprom_get_ofdm5ghpo,
	.sprom_get_fem_2ghz_tssipos	= bhnd_compat_sprom_get_fem_2ghz_tssipos,
	.sprom_get_fem_2ghz_extpa_gain	= bhnd_compat_sprom_get_fem_2ghz_extpa_gain,
	.sprom_get_fem_2ghz_pdet_range	= bhnd_compat_sprom_get_fem_2ghz_pdet_range,
	.sprom_get_fem_2ghz_tr_iso	= bhnd_compat_sprom_get_fem_2ghz_tr_iso,
	.sprom_get_fem_2ghz_antswlut	= bhnd_compat_sprom_get_fem_2ghz_antswlut,
	.sprom_get_fem_5ghz_extpa_gain	= bhnd_compat_sprom_get_fem_5ghz_extpa_gain,
	.sprom_get_fem_5ghz_pdet_range	= bhnd_compat_sprom_get_fem_5ghz_pdet_range,
	.sprom_get_fem_5ghz_antswlut	= bhnd_compat_sprom_get_fem_5ghz_antswlut,
	.sprom_get_txpid_2g_0		= bhnd_compat_sprom_get_txpid_2g_0,
	.sprom_get_txpid_2g_1		= bhnd_compat_sprom_get_txpid_2g_1,
	.sprom_get_txpid_5gl_0		= bhnd_compat_sprom_get_txpid_5gl_0,
	.sprom_get_txpid_5gl_1		= bhnd_compat_sprom_get_txpid_5gl_1,
	.sprom_get_txpid_5g_0		= bhnd_compat_sprom_get_txpid_5g_0,
	.sprom_get_txpid_5g_1		= bhnd_compat_sprom_get_txpid_5g_1,
	.sprom_get_txpid_5gh_0		= bhnd_compat_sprom_get_txpid_5gh_0,
	.sprom_get_txpid_5gh_1		= bhnd_compat_sprom_get_txpid_5gh_1,
	.sprom_get_stbcpo		= bhnd_compat_sprom_get_stbcpo,
	.sprom_get_cddpo		= bhnd_compat_sprom_get_cddpo,
	.sprom_get_mcs2gpo		= bhnd_compat_sprom_get_mcs2gpo,
	.sprom_get_mcs5glpo		= bhnd_compat_sprom_get_mcs5glpo,
	.sprom_get_mcs5gpo		= bhnd_compat_sprom_get_mcs5gpo,
	.sprom_get_mcs5ghpo		= bhnd_compat_sprom_get_mcs5ghpo,
};
