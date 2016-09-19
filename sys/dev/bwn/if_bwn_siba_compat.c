/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

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

#include "if_bwn_siba_compat.h"

#define	BWN_ASSERT_VALID_REG(_dev, _offset)	\
	KASSERT(!bwn_bhnd_is_siba_reg(_dev, _offset), \
	    ("accessing siba-specific register %#jx", (uintmax_t)(_offset)));

/**
 * Return true if @p offset is within a siba-specific configuration register
 * block.
 */
static inline bool
bwn_bhnd_is_siba_reg(device_t dev, uint16_t offset)
{
	if (offset >= SIBA_CFG0_OFFSET &&
	    offset <= SIBA_CFG0_OFFSET + SIBA_CFG_SIZE)
		return (true);

	if (offset >= SIBA_CFG1_OFFSET &&
	    offset <= SIBA_CFG1_OFFSET + SIBA_CFG_SIZE)
		return (true);
	
	return (false);
}


static int
bwn_bhnd_bus_ops_init(device_t dev)
{
	struct bwn_bhnd_ctx	*ctx;
	struct bwn_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	sc->sc_mem_rid = 0;
	sc->sc_mem_res = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_mem_rid, RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		return (ENXIO);
	}

	/* Allocate PMU state */
	if ((error = bhnd_alloc_pmu(dev))) {
		device_printf(dev, "PMU allocation failed: %d\n", error);
		goto failed;
	}

	/* Allocate our context */
	ctx = malloc(sizeof(struct bwn_bhnd_ctx), M_DEVBUF, M_WAITOK|M_ZERO);

	/* Initialize bwn_softc */
	sc->sc_bus_ctx = ctx;
	return (0);

failed:
	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->sc_mem_rid,
	    sc->sc_mem_res);
	return (error);
}

static void
bwn_bhnd_bus_ops_fini(device_t dev)
{
	struct bwn_softc	*sc;

	sc = device_get_softc(dev);

	bhnd_release_pmu(dev);
	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->sc_mem_rid,
	    sc->sc_mem_res);
}

/*
 * Disable PCI-specific MSI interrupt allocation handling
 */

/*
 * pci_find_cap()
 *
 * Referenced by:
 *   bwn_attach()
 */
static int
bhnd_compat_pci_find_cap(device_t dev, int capability, int *capreg)
{
	return (ENODEV);
}

/*
 * pci_alloc_msi()
 *
 * Referenced by:
 *   bwn_attach()
 */
static int
bhnd_compat_pci_alloc_msi(device_t dev, int *count)
{
	return (ENODEV);
}

/*
 * pci_release_msi()
 *
 * Referenced by:
 *   bwn_attach()
 *   bwn_detach()
 */
static int
bhnd_compat_pci_release_msi(device_t dev)
{
	return (ENODEV);
}

/*
 * pci_msi_count()
 *
 * Referenced by:
 *   bwn_attach()
 */
static int
bhnd_compat_pci_msi_count(device_t dev)
{
	return (0);
}

/*
 * siba_get_vendor()
 *
 * Referenced by:
 *   bwn_probe()
 */
static uint16_t
bhnd_compat_get_vendor(device_t dev)
{
	uint16_t vendor = bhnd_get_vendor(dev);

	switch (vendor) {
	case BHND_MFGID_BCM:
		return (SIBA_VID_BROADCOM);
	default:
		return (0x0000);
	}
}

/*
 * siba_get_device()
 *
 * Referenced by:
 *   bwn_probe()
 */
static uint16_t
bhnd_compat_get_device(device_t dev)
{
	return (bhnd_get_device(dev));
}

/*
 * siba_get_revid()
 *
 * Referenced by:
 *   bwn_attach()
 *   bwn_attach_core()
 *   bwn_chip_init()
 *   bwn_chiptest()
 *   bwn_core_init()
 *   bwn_core_start()
 *   bwn_pio_idx2base()
 *   bwn_pio_set_txqueue()
 *   bwn_pio_tx_start()
 *   bwn_probe()
 * ... and 19 others
 * 
 */
static uint8_t
bhnd_compat_get_revid(device_t dev)
{
	return (bhnd_get_hwrev(dev));
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
 * siba_get_pci_subvendor()
 *
 * Referenced by:
 *   bwn_led_attach()
 *   bwn_nphy_op_prepare_structs()
 *   bwn_phy_g_prepare_hw()
 *   bwn_phy_hwpctl_init()
 *   bwn_phy_init_b5()
 *   bwn_phy_initn()
 *   bwn_phy_txpower_check()
 *   bwn_radio_init2055_post()
 *   bwn_sprom_bugfixes()
 *   bwn_wa_init()
 */
static uint16_t
bhnd_compat_get_pci_subvendor(device_t dev)
{
	return (pci_get_subvendor(bwn_bhnd_get_pci_dev(dev)));
}

/*
 * siba_get_pci_subdevice()
 *
 * Referenced by:
 *   bwn_nphy_workarounds_rev1_2()
 *   bwn_phy_g_prepare_hw()
 *   bwn_phy_hwpctl_init()
 *   bwn_phy_init_b5()
 *   bwn_phy_initn()
 *   bwn_phy_lp_bbinit_r01()
 *   bwn_phy_txpower_check()
 *   bwn_radio_init2055_post()
 *   bwn_sprom_bugfixes()
 *   bwn_wa_init()
 */
static uint16_t
bhnd_compat_get_pci_subdevice(device_t dev)
{
	return (pci_get_subdevice(bwn_bhnd_get_pci_dev(dev)));
}

/*
 * siba_get_pci_revid()
 *
 * Referenced by:
 *   bwn_phy_g_prepare_hw()
 *   bwn_phy_lp_bbinit_r2()
 *   bwn_sprom_bugfixes()
 *   bwn_wa_init()
 */
static uint8_t
bhnd_compat_get_pci_revid(device_t dev)
{
	return (pci_get_revid(bwn_bhnd_get_pci_dev(dev)));
}

/*
 * siba_get_chipid()
 *
 * Referenced by:
 *   bwn_attach()
 *   bwn_gpio_init()
 *   bwn_mac_switch_freq()
 *   bwn_phy_g_attach()
 *   bwn_phy_g_init_sub()
 *   bwn_phy_g_prepare_hw()
 *   bwn_phy_getinfo()
 *   bwn_phy_lp_calib()
 *   bwn_set_opmode()
 *   bwn_sprom_bugfixes()
 * ... and 9 others
 * 
 */
static uint16_t
bhnd_compat_get_chipid(device_t dev)
{
	panic("siba_get_chipid() unimplemented");
}

/*
 * siba_get_chiprev()
 *
 * Referenced by:
 *   bwn_phy_getinfo()
 *   bwn_phy_lp_bbinit_r2()
 *   bwn_phy_lp_tblinit_r2()
 *   bwn_set_opmode()
 */
static uint16_t
bhnd_compat_get_chiprev(device_t dev)
{
	panic("siba_get_chiprev() unimplemented");
}

/*
 * siba_get_chippkg()
 *
 * Referenced by:
 *   bwn_phy_g_init_sub()
 *   bwn_phy_lp_bbinit_r01()
 *   bwn_radio_2056_setup()
 */
static uint8_t
bhnd_compat_get_chippkg(device_t dev)
{
	panic("siba_get_chippkg() unimplemented");
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
	hostb = bhnd_find_hostb_device(bus);

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
 * siba_get_cc_pmufreq()
 *
 * Referenced by:
 *   bwn_phy_lp_b2062_init()
 *   bwn_phy_lp_b2062_switch_channel()
 *   bwn_phy_lp_b2063_switch_channel()
 *   bwn_phy_lp_rxcal_r2()
 */
static uint32_t
bhnd_compat_get_cc_pmufreq(device_t dev)
{
	panic("siba_get_cc_pmufreq() unimplemented");
}

/*
 * siba_get_cc_caps()
 *
 * Referenced by:
 *   bwn_phy_lp_b2062_init()
 */
static uint32_t
bhnd_compat_get_cc_caps(device_t dev)
{
	panic("siba_get_cc_caps() unimplemented");
}

/*
 * siba_get_cc_powerdelay()
 *
 * Referenced by:
 *   bwn_chip_init()
 */
static uint16_t
bhnd_compat_get_cc_powerdelay(device_t dev)
{
	panic("siba_get_cc_powerdelay() unimplemented");
}

/*
 * siba_get_pcicore_revid()
 *
 * Referenced by:
 *   bwn_core_init()
 */
static uint8_t
bhnd_compat_get_pcicore_revid(device_t dev)
{
	panic("siba_get_pcicore_revid() unimplemented");
}

/*
 * siba_sprom_get_rev()
 *
 * Referenced by:
 *   bwn_nphy_op_prepare_structs()
 *   bwn_nphy_tx_power_ctl_setup()
 *   bwn_nphy_tx_power_fix()
 *   bwn_nphy_workarounds_rev7plus()
 */
static uint8_t
bhnd_compat_sprom_get_rev(device_t dev)
{
	panic("siba_sprom_get_rev() unimplemented");
}

/*
 * siba_sprom_get_mac_80211bg()
 *
 * Referenced by:
 *   bwn_attach_post()
 */
static uint8_t *
bhnd_compat_sprom_get_mac_80211bg(device_t dev)
{
	panic("siba_sprom_get_mac_80211bg() unimplemented");
}

/*
 * siba_sprom_get_mac_80211a()
 *
 * Referenced by:
 *   bwn_attach_post()
 */
static uint8_t *
bhnd_compat_sprom_get_mac_80211a(device_t dev)
{
	panic("siba_sprom_get_mac_80211a() unimplemented");
}

/*
 * siba_sprom_get_brev()
 *
 * Referenced by:
 *   bwn_radio_init2055_post()
 */
static uint8_t
bhnd_compat_sprom_get_brev(device_t dev)
{
	panic("siba_sprom_get_brev() unimplemented");
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
	panic("siba_sprom_get_ccode() unimplemented");
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
	panic("siba_sprom_get_ant_a() unimplemented");
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
	panic("siba_sprom_get_ant_bg() unimplemented");
}

/*
 * siba_sprom_get_pa0b0()
 *
 * Referenced by:
 *   bwn_phy_g_attach()
 */
static uint16_t
bhnd_compat_sprom_get_pa0b0(device_t dev)
{
	panic("siba_sprom_get_pa0b0() unimplemented");
}

/*
 * siba_sprom_get_pa0b1()
 *
 * Referenced by:
 *   bwn_phy_g_attach()
 */
static uint16_t
bhnd_compat_sprom_get_pa0b1(device_t dev)
{
	panic("siba_sprom_get_pa0b1() unimplemented");
}

/*
 * siba_sprom_get_pa0b2()
 *
 * Referenced by:
 *   bwn_phy_g_attach()
 */
static uint16_t
bhnd_compat_sprom_get_pa0b2(device_t dev)
{
	panic("siba_sprom_get_pa0b2() unimplemented");
}

/*
 * siba_sprom_get_gpio0()
 *
 * Referenced by:
 *   bwn_led_attach()
 */
static uint8_t
bhnd_compat_sprom_get_gpio0(device_t dev)
{
	panic("siba_sprom_get_gpio0() unimplemented");
}

/*
 * siba_sprom_get_gpio1()
 *
 * Referenced by:
 *   bwn_led_attach()
 */
static uint8_t
bhnd_compat_sprom_get_gpio1(device_t dev)
{
	panic("siba_sprom_get_gpio1() unimplemented");
}

/*
 * siba_sprom_get_gpio2()
 *
 * Referenced by:
 *   bwn_led_attach()
 */
static uint8_t
bhnd_compat_sprom_get_gpio2(device_t dev)
{
	panic("siba_sprom_get_gpio2() unimplemented");
}

/*
 * siba_sprom_get_gpio3()
 *
 * Referenced by:
 *   bwn_led_attach()
 */
static uint8_t
bhnd_compat_sprom_get_gpio3(device_t dev)
{
	panic("siba_sprom_get_gpio3() unimplemented");
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
	panic("siba_sprom_get_maxpwr_bg() unimplemented");
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
	panic("siba_sprom_set_maxpwr_bg() unimplemented");
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
	panic("siba_sprom_get_rxpo2g() unimplemented");
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
	panic("siba_sprom_get_rxpo5g() unimplemented");
}

/*
 * siba_sprom_get_tssi_bg()
 *
 * Referenced by:
 *   bwn_phy_g_attach()
 */
static uint8_t
bhnd_compat_sprom_get_tssi_bg(device_t dev)
{
	panic("siba_sprom_get_tssi_bg() unimplemented");
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
	panic("siba_sprom_get_tri2g() unimplemented");
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
	panic("siba_sprom_get_tri5gl() unimplemented");
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
	panic("siba_sprom_get_tri5g() unimplemented");
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
	panic("siba_sprom_get_tri5gh() unimplemented");
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
	panic("siba_sprom_get_rssisav2g() unimplemented");
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
	panic("siba_sprom_get_rssismc2g() unimplemented");
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
	panic("siba_sprom_get_rssismf2g() unimplemented");
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
	panic("siba_sprom_get_bxa2g() unimplemented");
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
	panic("siba_sprom_get_rssisav5g() unimplemented");
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
	panic("siba_sprom_get_rssismc5g() unimplemented");
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
	panic("siba_sprom_get_rssismf5g() unimplemented");
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
	panic("siba_sprom_get_bxa5g() unimplemented");
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
	panic("siba_sprom_get_cck2gpo() unimplemented");
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
	panic("siba_sprom_get_ofdm2gpo() unimplemented");
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
	panic("siba_sprom_get_ofdm5glpo() unimplemented");
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
	panic("siba_sprom_get_ofdm5gpo() unimplemented");
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
	panic("siba_sprom_get_ofdm5ghpo() unimplemented");
}

/*
 * siba_sprom_set_bf_lo()
 *
 * Referenced by:
 *   bwn_sprom_bugfixes()
 */
static void
bhnd_compat_sprom_set_bf_lo(device_t dev, uint16_t t)
{
	panic("siba_sprom_set_bf_lo() unimplemented");
}

/*
 * siba_sprom_get_bf_lo()
 *
 * Referenced by:
 *   bwn_bt_enable()
 *   bwn_core_init()
 *   bwn_gpio_init()
 *   bwn_loopback_calcgain()
 *   bwn_phy_g_init_sub()
 *   bwn_phy_g_recalc_txpwr()
 *   bwn_phy_g_set_txpwr()
 *   bwn_phy_g_task_60s()
 *   bwn_rx_rssi_calc()
 *   bwn_sprom_bugfixes()
 * ... and 11 others
 * 
 */
static uint16_t
bhnd_compat_sprom_get_bf_lo(device_t dev)
{
	panic("siba_sprom_get_bf_lo() unimplemented");
}

/*
 * siba_sprom_get_bf_hi()
 *
 * Referenced by:
 *   bwn_nphy_gain_ctl_workarounds_rev3()
 *   bwn_phy_lp_bbinit_r01()
 *   bwn_phy_lp_tblinit_txgain()
 */
static uint16_t
bhnd_compat_sprom_get_bf_hi(device_t dev)
{
	panic("siba_sprom_get_bf_hi() unimplemented");
}

/*
 * siba_sprom_get_bf2_lo()
 *
 * Referenced by:
 *   bwn_nphy_op_prepare_structs()
 *   bwn_nphy_workarounds_rev1_2()
 *   bwn_nphy_workarounds_rev3plus()
 *   bwn_phy_initn()
 *   bwn_radio_2056_setup()
 *   bwn_radio_init2055_post()
 */
static uint16_t
bhnd_compat_sprom_get_bf2_lo(device_t dev)
{
	panic("siba_sprom_get_bf2_lo() unimplemented");
}

/*
 * siba_sprom_get_bf2_hi()
 *
 * Referenced by:
 *   bwn_nphy_workarounds_rev7plus()
 *   bwn_phy_initn()
 *   bwn_radio_2056_setup()
 */
static uint16_t
bhnd_compat_sprom_get_bf2_hi(device_t dev)
{
	panic("siba_sprom_get_bf2_hi() unimplemented");
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
	panic("siba_sprom_get_fem_2ghz_tssipos() unimplemented");
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
	panic("siba_sprom_get_fem_2ghz_extpa_gain() unimplemented");
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
	panic("siba_sprom_get_fem_2ghz_pdet_range() unimplemented");
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
	panic("siba_sprom_get_fem_2ghz_tr_iso() unimplemented");
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
	panic("siba_sprom_get_fem_2ghz_antswlut() unimplemented");
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
	panic("siba_sprom_get_fem_5ghz_extpa_gain() unimplemented");
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
	panic("siba_sprom_get_fem_5ghz_pdet_range() unimplemented");
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
	panic("siba_sprom_get_fem_5ghz_antswlut() unimplemented");
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
	panic("siba_sprom_get_txpid_2g_0() unimplemented");
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
	panic("siba_sprom_get_txpid_2g_1() unimplemented");
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
	panic("siba_sprom_get_txpid_5gl_0() unimplemented");
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
	panic("siba_sprom_get_txpid_5gl_1() unimplemented");
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
	panic("siba_sprom_get_txpid_5g_0() unimplemented");
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
	panic("siba_sprom_get_txpid_5g_1() unimplemented");
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
	panic("siba_sprom_get_txpid_5gh_0() unimplemented");
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
	panic("siba_sprom_get_txpid_5gh_1() unimplemented");
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
	panic("siba_sprom_get_stbcpo() unimplemented");
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
	panic("siba_sprom_get_cddpo() unimplemented");
}

/*
 * siba_powerup()
 *
 * Referenced by:
 *   bwn_attach_core()
 *   bwn_core_init()
 */
static void
bhnd_compat_powerup(device_t dev, int dynamic)
{
	bhnd_clock	clock;
	int		error;

	// XXX TODO: Should we bring up the core clock in bwn_reset_core()
	// instead?

	/* On bcma(4) devices, the core must be brought out of reset before
	 * accessing PMU clock request registers */
	if ((error = bhnd_reset_hw(dev, 0, 0))) {
		device_printf(dev, "core reset failed: %d\n", error);
		return;
	}

	/* Issue a PMU clock request */
	if (dynamic)
		clock = BHND_CLOCK_DYN;
	else
		clock = BHND_CLOCK_HT;

	if ((error = bhnd_request_clock(dev, clock))) {
		device_printf(dev, "%d clock request failed: %d\n",
		    clock, error);
	}

}

/*
 * siba_powerdown()
 *
 * Referenced by:
 *   bwn_attach_core()
 *   bwn_core_exit()
 *   bwn_core_init()
 */
static int
bhnd_compat_powerdown(device_t dev)
{
	int	error;

	/* Release any outstanding clock request */
	if ((error = bhnd_request_clock(dev, BHND_CLOCK_DYN)))
		return (error);

	/* Suspend the core */
	if ((error = bhnd_suspend_hw(dev, 0)))
		return (error);

	return (0);
}

/*
 * siba_read_2()
 *
 * Referenced by:
 *   bwn_chip_init()
 *   bwn_chiptest()
 *   bwn_dummy_transmission()
 *   bwn_gpio_init()
 *   bwn_phy_getinfo()
 *   bwn_pio_read_2()
 *   bwn_shm_read_2()
 *   bwn_shm_read_4()
 *   bwn_wme_init()
 *   bwn_wme_loadparams()
 * ... and 23 others
 * 
 */
static uint16_t
bhnd_compat_read_2(device_t dev, uint16_t offset)
{
	struct bwn_softc *sc = device_get_softc(dev);

	BWN_ASSERT_VALID_REG(dev, offset);

	return (bhnd_bus_read_2(sc->sc_mem_res, offset));
}

/*
 * siba_write_2()
 *
 * Referenced by:
 *   bwn_chip_init()
 *   bwn_chiptest()
 *   bwn_crypt_init()
 *   bwn_gpio_init()
 *   bwn_phy_getinfo()
 *   bwn_pio_tx_start()
 *   bwn_set_opmode()
 *   bwn_shm_write_2()
 *   bwn_shm_write_4()
 *   bwn_wme_init()
 * ... and 43 others
 * 
 */
static void
bhnd_compat_write_2(device_t dev, uint16_t offset, uint16_t value)
{
	struct bwn_softc *sc = device_get_softc(dev);

	BWN_ASSERT_VALID_REG(dev, offset);

	return (bhnd_bus_write_2(sc->sc_mem_res, offset, value));
}

/*
 * siba_read_4()
 *
 * Referenced by:
 *   bwn_attach_core()
 *   bwn_chip_init()
 *   bwn_chiptest()
 *   bwn_core_exit()
 *   bwn_core_init()
 *   bwn_core_start()
 *   bwn_pio_init()
 *   bwn_pio_tx_start()
 *   bwn_reset_core()
 *   bwn_shm_read_4()
 * ... and 42 others
 * 
 */
static uint32_t
bhnd_compat_read_4(device_t dev, uint16_t offset)
{
	struct bwn_softc *sc = device_get_softc(dev);

	BWN_ASSERT_VALID_REG(dev, offset);

	return (bhnd_bus_read_4(sc->sc_mem_res, offset));
}

/*
 * siba_write_4()
 *
 * Referenced by:
 *   bwn_chip_init()
 *   bwn_chiptest()
 *   bwn_core_exit()
 *   bwn_core_start()
 *   bwn_dma_mask()
 *   bwn_dma_rxdirectfifo()
 *   bwn_pio_init()
 *   bwn_reset_core()
 *   bwn_shm_ctlword()
 *   bwn_shm_write_4()
 * ... and 37 others
 * 
 */
static void
bhnd_compat_write_4(device_t dev, uint16_t offset, uint32_t value)
{
	struct bwn_softc *sc = device_get_softc(dev);

	BWN_ASSERT_VALID_REG(dev, offset);

	return (bhnd_bus_write_4(sc->sc_mem_res, offset, value));
}

/*
 * siba_dev_up()
 *
 * Referenced by:
 *   bwn_reset_core()
 */
static void
bhnd_compat_dev_up(device_t dev, uint32_t flags)
{
	panic("siba_dev_up() unimplemented");
}

/*
 * siba_dev_down()
 *
 * Referenced by:
 *   bwn_attach_core()
 *   bwn_core_exit()
 */
static void
bhnd_compat_dev_down(device_t dev, uint32_t flags)
{
	panic("siba_dev_down() unimplemented");
}

/*
 * siba_dev_isup()
 *
 * Referenced by:
 *   bwn_core_init()
 */
static int
bhnd_compat_dev_isup(device_t dev)
{
	panic("siba_dev_isup() unimplemented");
}

/*
 * siba_pcicore_intr()
 *
 * Referenced by:
 *   bwn_core_init()
 */
static void
bhnd_compat_pcicore_intr(device_t dev)
{
	panic("siba_pcicore_intr() unimplemented");
}

/*
 * siba_dma_translation()
 *
 * Referenced by:
 *   bwn_dma_32_setdesc()
 *   bwn_dma_64_setdesc()
 *   bwn_dma_setup()
 */
static uint32_t
bhnd_compat_dma_translation(device_t dev)
{
	panic("siba_dma_translation() unimplemented");
}

/*
 * siba_read_multi_2()
 *
 * Referenced by:
 *   bwn_pio_rxeof()
 */
static void
bhnd_compat_read_multi_2(device_t dev, void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_read_multi_2() unimplemented");
}

/*
 * siba_read_multi_4()
 *
 * Referenced by:
 *   bwn_pio_rxeof()
 */
static void
bhnd_compat_read_multi_4(device_t dev, void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_read_multi_4() unimplemented");
}

/*
 * siba_write_multi_2()
 *
 * Referenced by:
 *   bwn_pio_write_multi_2()
 */
static void
bhnd_compat_write_multi_2(device_t dev, const void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_write_multi_2() unimplemented");
}

/*
 * siba_write_multi_4()
 *
 * Referenced by:
 *   bwn_pio_write_multi_4()
 */
static void
bhnd_compat_write_multi_4(device_t dev, const void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_write_multi_4() unimplemented");
}

/*
 * siba_barrier()
 *
 * Referenced by:
 *   bwn_intr()
 *   bwn_intrtask()
 *   bwn_ram_write()
 */
static void
bhnd_compat_barrier(device_t dev, int flags)
{
	panic("siba_barrier() unimplemented");
}

/*
 * siba_cc_pmu_set_ldovolt()
 *
 * Referenced by:
 *   bwn_phy_lp_bbinit_r01()
 */
static void
bhnd_compat_cc_pmu_set_ldovolt(device_t dev, int id, uint32_t volt)
{
	panic("siba_cc_pmu_set_ldovolt() unimplemented");
}

/*
 * siba_cc_pmu_set_ldoparef()
 *
 * Referenced by:
 *   bwn_phy_lp_bbinit_r01()
 */
static void
bhnd_compat_cc_pmu_set_ldoparef(device_t dev, uint8_t on)
{
	panic("siba_cc_pmu_set_ldoparef() unimplemented");
}

/*
 * siba_gpio_set()
 *
 * Referenced by:
 *   bwn_chip_exit()
 *   bwn_chip_init()
 *   bwn_gpio_init()
 *   bwn_nphy_superswitch_init()
 */
static void
bhnd_compat_gpio_set(device_t dev, uint32_t value)
{
	panic("siba_gpio_set() unimplemented");
}

/*
 * siba_gpio_get()
 *
 * Referenced by:
 *   bwn_gpio_init()
 */
static uint32_t
bhnd_compat_gpio_get(device_t dev)
{
	panic("siba_gpio_get() unimplemented");
}

/*
 * siba_fix_imcfglobug()
 *
 * Referenced by:
 *   bwn_core_init()
 */
static void
bhnd_compat_fix_imcfglobug(device_t dev)
{
	panic("siba_fix_imcfglobug() unimplemented");
}

/*
 * siba_sprom_get_core_power_info()
 *
 * Referenced by:
 *   bwn_nphy_tx_power_ctl_setup()
 *   bwn_ppr_load_max_from_sprom()
 */
static int
bhnd_compat_sprom_get_core_power_info(device_t dev, int core,
    struct siba_sprom_core_pwr_info *c)
{
	panic("siba_sprom_get_core_power_info() unimplemented");
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
	panic("siba_sprom_get_mcs2gpo() unimplemented");
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
	panic("siba_sprom_get_mcs5glpo() unimplemented");
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
	panic("siba_sprom_get_mcs5gpo() unimplemented");
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
	panic("siba_sprom_get_mcs5ghpo() unimplemented");
}

/*
 * siba_pmu_spuravoid_pllupdate()
 *
 * Referenced by:
 *   bwn_nphy_pmu_spur_avoid()
 */
static void
bhnd_compat_pmu_spuravoid_pllupdate(device_t dev, int spur_avoid)
{
	panic("siba_pmu_spuravoid_pllupdate() unimplemented");
}

/*
 * siba_cc_set32()
 *
 * Referenced by:
 *   bwn_phy_initn()
 *   bwn_wireless_core_phy_pll_reset()
 */
static void
bhnd_compat_cc_set32(device_t dev, uint32_t reg, uint32_t val)
{
	panic("siba_cc_set32() unimplemented");
}

/*
 * siba_cc_mask32()
 *
 * Referenced by:
 *   bwn_wireless_core_phy_pll_reset()
 */
static void
bhnd_compat_cc_mask32(device_t dev, uint32_t reg, uint32_t mask)
{
	panic("siba_cc_mask32() unimplemented");
}

/*
 * siba_cc_write32()
 *
 * Referenced by:
 *   bwn_wireless_core_phy_pll_reset()
 */
static void
bhnd_compat_cc_write32(device_t dev, uint32_t reg, uint32_t val)
{
	panic("siba_cc_write32() unimplemented");
}

const struct bwn_bus_ops bwn_bhnd_bus_ops = {
	.init				= bwn_bhnd_bus_ops_init,
	.fini				= bwn_bhnd_bus_ops_fini,
	.pci_find_cap			= bhnd_compat_pci_find_cap,
	.pci_alloc_msi			= bhnd_compat_pci_alloc_msi,
	.pci_release_msi		= bhnd_compat_pci_release_msi,
	.pci_msi_count			= bhnd_compat_pci_msi_count,
	.get_vendor			= bhnd_compat_get_vendor,
	.get_device			= bhnd_compat_get_device,
	.get_revid			= bhnd_compat_get_revid,
	.get_pci_vendor			= bhnd_compat_get_pci_vendor,
	.get_pci_device			= bhnd_compat_get_pci_device,
	.get_pci_subvendor		= bhnd_compat_get_pci_subvendor,
	.get_pci_subdevice		= bhnd_compat_get_pci_subdevice,
	.get_pci_revid			= bhnd_compat_get_pci_revid,
	.get_chipid			= bhnd_compat_get_chipid,
	.get_chiprev			= bhnd_compat_get_chiprev,
	.get_chippkg			= bhnd_compat_get_chippkg,
	.get_type			= bhnd_compat_get_type,
	.get_cc_pmufreq			= bhnd_compat_get_cc_pmufreq,
	.get_cc_caps			= bhnd_compat_get_cc_caps,
	.get_cc_powerdelay		= bhnd_compat_get_cc_powerdelay,
	.get_pcicore_revid		= bhnd_compat_get_pcicore_revid,
	.sprom_get_rev			= bhnd_compat_sprom_get_rev,
	.sprom_get_mac_80211bg		= bhnd_compat_sprom_get_mac_80211bg,
	.sprom_get_mac_80211a		= bhnd_compat_sprom_get_mac_80211a,
	.sprom_get_brev			= bhnd_compat_sprom_get_brev,
	.sprom_get_ccode		= bhnd_compat_sprom_get_ccode,
	.sprom_get_ant_a		= bhnd_compat_sprom_get_ant_a,
	.sprom_get_ant_bg		= bhnd_compat_sprom_get_ant_bg,
	.sprom_get_pa0b0		= bhnd_compat_sprom_get_pa0b0,
	.sprom_get_pa0b1		= bhnd_compat_sprom_get_pa0b1,
	.sprom_get_pa0b2		= bhnd_compat_sprom_get_pa0b2,
	.sprom_get_gpio0		= bhnd_compat_sprom_get_gpio0,
	.sprom_get_gpio1		= bhnd_compat_sprom_get_gpio1,
	.sprom_get_gpio2		= bhnd_compat_sprom_get_gpio2,
	.sprom_get_gpio3		= bhnd_compat_sprom_get_gpio3,
	.sprom_get_maxpwr_bg		= bhnd_compat_sprom_get_maxpwr_bg,
	.sprom_set_maxpwr_bg		= bhnd_compat_sprom_set_maxpwr_bg,
	.sprom_get_rxpo2g		= bhnd_compat_sprom_get_rxpo2g,
	.sprom_get_rxpo5g		= bhnd_compat_sprom_get_rxpo5g,
	.sprom_get_tssi_bg		= bhnd_compat_sprom_get_tssi_bg,
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
	.sprom_get_bf_lo		= bhnd_compat_sprom_get_bf_lo,
	.sprom_set_bf_lo		= bhnd_compat_sprom_set_bf_lo,
	.sprom_get_bf_hi		= bhnd_compat_sprom_get_bf_hi,
	.sprom_get_bf2_lo		= bhnd_compat_sprom_get_bf2_lo,
	.sprom_get_bf2_hi		= bhnd_compat_sprom_get_bf2_hi,
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
	.powerup			= bhnd_compat_powerup,
	.powerdown			= bhnd_compat_powerdown,
	.read_2				= bhnd_compat_read_2,
	.write_2			= bhnd_compat_write_2,
	.read_4				= bhnd_compat_read_4,
	.write_4			= bhnd_compat_write_4,
	.dev_up				= bhnd_compat_dev_up,
	.dev_down			= bhnd_compat_dev_down,
	.dev_isup			= bhnd_compat_dev_isup,
	.pcicore_intr			= bhnd_compat_pcicore_intr,
	.dma_translation		= bhnd_compat_dma_translation,
	.read_multi_2			= bhnd_compat_read_multi_2,
	.read_multi_4			= bhnd_compat_read_multi_4,
	.write_multi_2			= bhnd_compat_write_multi_2,
	.write_multi_4			= bhnd_compat_write_multi_4,
	.barrier			= bhnd_compat_barrier,
	.cc_pmu_set_ldovolt		= bhnd_compat_cc_pmu_set_ldovolt,
	.cc_pmu_set_ldoparef		= bhnd_compat_cc_pmu_set_ldoparef,
	.gpio_set			= bhnd_compat_gpio_set,
	.gpio_get			= bhnd_compat_gpio_get,
	.fix_imcfglobug			= bhnd_compat_fix_imcfglobug,
	.sprom_get_core_power_info	= bhnd_compat_sprom_get_core_power_info,
	.sprom_get_mcs2gpo		= bhnd_compat_sprom_get_mcs2gpo,
	.sprom_get_mcs5glpo		= bhnd_compat_sprom_get_mcs5glpo,
	.sprom_get_mcs5gpo		= bhnd_compat_sprom_get_mcs5gpo,
	.sprom_get_mcs5ghpo		= bhnd_compat_sprom_get_mcs5ghpo,
	.pmu_spuravoid_pllupdate	= bhnd_compat_pmu_spuravoid_pllupdate,
	.cc_set32			= bhnd_compat_cc_set32,
	.cc_mask32			= bhnd_compat_cc_mask32,
	.cc_write32			= bhnd_compat_cc_write32,
};
