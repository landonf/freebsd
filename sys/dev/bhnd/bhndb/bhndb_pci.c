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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * PCI-specific implementation for the BHNDB bridge driver.
 * 
 * Provides support for bridging from a PCI parent bus to a BHND-compatible
 * bus (e.g. bcma or siba) via a Broadcom PCI core configured in end-point
 * mode.
 * 
 * This driver handles all interactions with the PCI bridge core. On the
 * bridged bhnd bus, the PCI core device will be claimed by a simple
 * bhnd_hostb driver.
 */

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

#include <dev/bhnd/cores/pci/bhnd_pcireg.h>
#include <dev/bhnd/cores/pci/mdio_pcievar.h>

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"
#include "bhndb_private.h"

static int	bhndb_enable_pci_clocks(struct bhndb_pci_softc *sc);
static int	bhndb_disable_pci_clocks(struct bhndb_pci_softc *sc);

static int	bhndb_pci_compat_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);
static int	bhndb_pci_fast_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);

static uint32_t	bhndb_pcie_read_proto_reg(struct bhndb_pci_softc *sc,
		    uint32_t addr);
static void	bhndb_pcie_write_proto_reg(struct bhndb_pci_softc *sc,
		    uint32_t addr, uint32_t val);

static uint32_t	bhndb_pci_get_hostb_quirks(struct bhndb_pci_softc *,
		    const struct bhndb_pci_id *);

static const struct bhndb_pci_id *bhndb_pci_find_core_id(
				      struct bhnd_core_info *core);

#define	BHNDB_PCI_QUIRK(_sc, _name)	\
    ((_sc)->quirks & BHNDB_PCI_QUIRK_ ## _name)

#define	BHNDB_PCIE_QUIRK(_sc, _name)	\
    ((_sc)->quirks & BHNDB_PCIE_QUIRK_ ## _name)

#define	BHNDB_PCI_READ_2(_sc, _reg)		\
	bus_read_2((_sc)->mem_res, (_sc)->mem_off + (_reg))
#define	BHNDB_PCI_READ_4(_sc, _reg)		\
	bus_read_4((_sc)->mem_res, (_sc)->mem_off + (_reg))

#define	BHNDB_PCI_WRITE_2(_sc, _reg, _val)	\
	bus_write_2((_sc)->mem_res, (_sc)->mem_off +  (_reg), (_val))
#define	BHNDB_PCI_WRITE_4(_sc, _reg, _val)	\
	bus_write_4((_sc)->mem_res, (_sc)->mem_off +  (_reg), (_val))

/*
 * Supported PCI bridge cores.
 *
 * This table defines quirks specific to core hwrev ranges; see also
 * bhndb_pci_get_hostb_quirks() for additional quirk detection.
 */
static const struct bhndb_pci_id bhndb_pci_ids[] = {
	/* PCI */
	BHNDB_PCI_ID(PCI,
	    BHND_QUIRK_HWREV_RANGE	(0, 5,	BHNDB_PCI_QUIRK_SBINTVEC),
	    BHND_QUIRK_HWREV_GTE	(0,	BHNDB_PCI_QUIRK_SBTOPCI2_PREF_BURST),
	    BHND_QUIRK_HWREV_GTE	(11,	BHNDB_PCI_QUIRK_SBTOPCI2_READMULTI | BHNDB_PCI_QUIRK_CLKRUN_DSBL),
	    BHND_QUIRK_HWREV_END
	),

	/* PCI Gen 1 */
	BHNDB_PCI_ID(PCIE,
	    BHND_QUIRK_HWREV_EQ		(0,	BHNDB_PCIE_QUIRK_SDR9_L0s_HANG),
	    BHND_QUIRK_HWREV_RANGE	(0, 1,	BHNDB_PCIE_QUIRK_UR_STATUS_FIX),
	    BHND_QUIRK_HWREV_EQ		(1,	BHNDB_PCIE_QUIRK_PCIPM_REQEN),
	    BHND_QUIRK_HWREV_RANGE	(3, 5,	BHNDB_PCIE_QUIRK_ASPM_OVR | BHNDB_PCIE_QUIRK_SDR9_POLARITY),
	    BHND_QUIRK_HWREV_LTE	(6,	BHNDB_PCIE_QUIRK_L1_IDLE_THRESH),
	    BHND_QUIRK_HWREV_GTE	(6,	BHNDB_PCIE_QUIRK_SPROM_L23_PCI_RESET),
	    BHND_QUIRK_HWREV_EQ		(7,	BHNDB_PCIE_QUIRK_SERDES_NOPLLDOWN),
	    BHND_QUIRK_HWREV_GTE	(8,	BHNDB_PCIE_QUIRK_L1_TIMER_PERF),
	    BHND_QUIRK_HWREV_GTE	(10,	BHNDB_PCIE_QUIRK_SD_C22_EXTADDR),

	    BHND_QUIRK_HWREV_END
	),

	{ BHND_COREID_INVALID, BHND_PCI_REGFMT_PCI, NULL }
};

/** 
 * Default bhndb_pci implementation of device_probe().
 * 
 * Verifies that the parent is a PCI/PCIe device.
 */
static int
bhndb_pci_probe(device_t dev)
{
	device_t	parent;
	devclass_t	parent_bus;
	devclass_t	pci;

	/* Our parent must be a PCI/PCIe device. */
	pci = devclass_find("pci");
	parent = device_get_parent(dev);
	parent_bus = device_get_devclass(device_get_parent(parent));

	if (parent_bus != pci)
		return (ENXIO);

	device_set_desc(dev, "PCI-BHND bridge");

	return (BUS_PROBE_DEFAULT);
}

static int
bhndb_pci_attach(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error, reg;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Enable PCI bus mastering */
	pci_enable_busmaster(device_get_parent(dev));

	/* Determine our bridge device class */
	sc->pci_devclass = BHND_DEVCLASS_PCI;
	if (pci_find_cap(device_get_parent(dev), PCIY_EXPRESS, &reg) == 0)
		sc->pci_devclass = BHND_DEVCLASS_PCIE;

	/* Enable PCI clocks */
	if ((error = bhndb_enable_pci_clocks(sc))) {
		device_printf(dev, "clock initialization failed\n");
		return (error);
	}

	/* Use siba(4)-compatible regwin handling until we know
	 * what kind of bus is attached */
	sc->set_regwin = bhndb_pci_compat_setregwin;

	/* Perform full bridge attach */
	if ((error = bhndb_attach(dev, sc->pci_devclass)))
		return (error);

	/* If supported, switch to the faster regwin handling */
	if (sc->bhndb.chipid.chip_type != BHND_CHIPTYPE_SIBA) {
		atomic_store_rel_ptr((volatile void *) &sc->set_regwin,
		    (uintptr_t) &bhndb_pci_fast_setregwin);
	}

	return (0);
}

#if 0
/*
 * Quirk: N/A
 * Applies to all PCI/PCIe revisions.
 * 
 * On devices without a SROM, the PCI(e) cores will be initialized with
 * their Power-on-Reset defaults; this can leave the the BAR0 PCIe windows
 * potentially mapped to the wrong core.
 * 
 * This WAR updates the BAR0 defaults to point at the current PCIe core.
 */
static void
bcm_pci_sromless_defaults_war(struct bhnd_pci_hostb_softc *sc)
{
	bus_size_t	sprom_addr;
	u_int		sprom_core_idx;
	u_int		pci_core_idx;
	uint16_t	val;

	/* Fetch the SPROM's configured core index */
	sprom_addr = BPCI_COMMON_REG_OFFSET(SPROM_SHADOW, SRSH_PI_OFFSET);
	val = bhnd_bus_read_2(sc->dev, sc->core, sprom_addr);

	/* If it doesn't match our core index, update the index value */
	sprom_core_idx = BPCI_COMMON_REG_GET(val, SRSH_PI);
	pci_core_idx = bhnd_get_core_index(sc->dev);

	if (sprom_core_idx != pci_core_idx) {
		BPCI_COMMON_REG_SET(val, SRSH_PI, pci_core_idx);
		bhnd_bus_write_2(sc->dev, sc->core, sprom_addr, val);
	}
}

/* Hardware Setup WARs */
static void
bcm_pci_hw_wars(struct bhnd_pci_hostb_softc *sc)
{
	/* Fix Power-on-Reset defaults on SROM-less PCI/PCIe devices. */
	// TODO - This must be lifted out into bridge-level initialization
	bcm_pci_sromless_defaults_war(sc);

	/* Enable PCI prefetch/burst/MRM support */
	if (BHND_PCI_QUIRK(sc, SBTOPCI2_PREF_BURST) ||
	    BHND_PCI_QUIRK(sc, SBTOPCI2_READMULTI))
	{
		uint32_t sbp2;
		sbp2 = bhnd_bus_read_4(sc->dev, sc->core, BHND_PCI_SBTOPCI2);

		if (BHND_PCI_QUIRK(sc, SBTOPCI2_PREF_BURST))
			sbp2 |= (BHND_PCI_SBTOPCI_PREF|BHND_PCI_SBTOPCI_BURST);
		
		if (BHND_PCI_QUIRK(sc, SBTOPCI2_READMULTI))
			sbp2 |= BHND_PCI_SBTOPCI_RC_READMULTI;

		bhnd_bus_write_4(sc->dev, sc->core, BHND_PCI_SBTOPCI2, sbp2);
	}
	
	/* Disable PCI CLKRUN# */
	if (BHND_PCI_QUIRK(sc, CLKRUN_DSBL)) {
		uint32_t ctl;
	
		ctl = bhnd_bus_read_4(sc->dev, sc->core, BHND_PCI_CLKRUN_CTL);
		ctl |= BHND_PCI_CLKRUN_DSBL;
		bhnd_bus_write_4(sc->dev, sc->core, BHND_PCI_CLKRUN_CTL, ctl);
	}

	/* Force correct SerDes polarity */
	if (BHND_PCIE_QUIRK(sc, SDR9_POLARITY)) {
		device_printf(sc->dev, "SDR9_POLARITY\n");
		uint16_t	rxctl;

		rxctl = MDIO_READREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CTRL);

		rxctl |= BHND_PCIE_SDR9_RX_CTRL_FORCE;
		if (sc->polarity_inv)
			rxctl |= BHND_PCIE_SDR9_RX_CTRL_POLARITY_INV;
		else
			rxctl &= ~BHND_PCIE_SDR9_RX_CTRL_POLARITY_INV;

		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CTRL, rxctl);
	}

	/* Enable TLP unmatched address handling work-around */
	if (BHND_PCIE_QUIRK(sc, UR_STATUS_FIX)) {
		device_printf(sc->dev, "UR_STATUS_FIX\n");
		uint32_t wrs;
		wrs = bcm_pcie_read_proto_reg(sc, BHND_PCIE_TLP_WORKAROUNDSREG);
		wrs |= BHND_PCIE_TLP_WORKAROUND_URBIT;
		bcm_pcie_write_proto_reg(sc, BHND_PCIE_TLP_WORKAROUNDSREG, wrs);
	}

	/* Explicitly enable PCI-PM */
	if (BHND_PCIE_QUIRK(sc, PCIPM_REQEN)) {
		device_printf(sc->dev, "PCIPM_REQEN\n");
		uint32_t lcreg;
		lcreg = bcm_pcie_read_proto_reg(sc, BHND_PCIE_DLLP_LCREG);
		lcreg |= BHND_PCIE_DLLP_LCREG_PCIPM_EN;
		bcm_pcie_write_proto_reg(sc, BHND_PCIE_DLLP_LCREG, lcreg);
	}

	/* Adjust SerDes CDR tuning to ensure that CDR is stable before sending
	 * data during L0s to L0 exit transitions. */
	if (BHND_PCIE_QUIRK(sc, SDR9_L0s_HANG)) {
		device_printf(sc->dev, "SDR9_L0s_HANG\n");
		uint16_t sdv;

		/* Set RX track/acquire timers to 2.064us/40.96us */
		sdv = 0;
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_TIMER1_LKTRK, (2064/16));
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_TIMER1_LKACQ, (40960/1024));
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		BHND_PCIE_SDR9_RX_TIMER1, sdv);

		/* Apply CDR frequency workaround */
		sdv = BHND_PCIE_SDR9_RX_CDR_FREQ_OVR_EN;
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDR_FREQ_OVR, 0x0);
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		BHND_PCIE_SDR9_RX_CDR, sdv);

		/* Apply CDR BW tunings */
		sdv = 0;
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_INTGTRK, 0x2);
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_INTGACQ, 0x4);
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_PROPTRK, 0x6);
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_PROPACQ, 0x6);
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		BHND_PCIE_SDR9_RX_CDRBW, sdv);
	}

	/* Enable L23READY_EXIT_NOPRST if not already set in SPROM. */
	if (BHND_PCIE_QUIRK(sc, SPROM_L23_PCI_RESET)) {
		bus_size_t	reg;
		uint16_t	cfg;

		/* Fetch the misccfg offset from SPROM */
		reg = BHND_PCIE_SPROM_SHADOW + BHND_PCIE_SRSH_PCIE_MISC_CONFIG;
		cfg = bhnd_bus_read_2(sc->dev, sc->core, reg);

		/* Set NOPRST enable flag if not already set */
		if (!(cfg & BHND_PCIE_SRSH_L23READY_EXIT_NOPRST)) {
			cfg |= BHND_PCIE_SRSH_L23READY_EXIT_NOPRST;
			bhnd_bus_write_2(sc->dev, sc->core, reg, cfg);
		}
	}
}
#endif

/**
 * Initialize the full bridge configuration.
 * 
 */
static int
bhndb_pci_init_full_config(device_t dev, device_t child,
    const struct bhndb_hw_priority *prio_table)
{
	struct bhnd_core_info		 core;
	const struct bhndb_pci_id	*id;
	struct bhndb_pci_softc		*sc;
	struct bhndb_region		*pcir;
	bhnd_addr_t			 pcir_addr;
	bhnd_size_t			 pcir_size;
	int				 error;

	sc = device_get_softc(dev);

	/* Let bhndb perform full initialization */
	if ((error = bhndb_generic_init_full_config(dev, child, prio_table)))
		return (error);

	/* Identify our bridge core and register family. */
	KASSERT(sc->bhndb.hostb_dev,
	    ("missing hostb device\n"));

	core = bhnd_get_core_info(sc->bhndb.hostb_dev);
	id = bhndb_pci_find_core_id(&core);
	if (id == NULL) {
		device_printf(dev, "%s %s hostb core is not recognized\n",
		    bhnd_vendor_name(core.vendor), bhnd_core_name(&core));
	}

	/* Fetch register family and device quirks */
	sc->regfmt = id->regfmt;
	sc->quirks = bhndb_pci_get_hostb_quirks(sc, id);

	/* Apply any early quirk workarounds */
	if (BHNDB_PCIE_QUIRK(sc, SDR9_POLARITY)) {
		uint32_t st;
		bool inv;

		/* Determine correct polarity by observing the attach-time PCIe PHY
		 * link status. This is used later to reset/force the SerDes
		 * polarity */
		st = bhndb_pcie_read_proto_reg(sc, BHND_PCIE_PLP_STATUSREG);
		inv = ((st & BHND_PCIE_PLP_POLARITY_INV) != 0);
		sc->sdr9_quirk_polarity.inv = inv;
	}
	
	/* Find the PCI core register block address/size. */
	error = bhnd_get_region_addr(sc->bhndb.hostb_dev, BHND_PORT_DEVICE, 0,
	    0, &pcir_addr, &pcir_size);
	if (error) {
		device_printf(dev,
		    "failed to locate usable hostb core registers\n");
		return (error);
	}

	/* Find the bhndb_region that statically maps this block */
	pcir = bhndb_find_resource_region(sc->bhndb.bus_res, pcir_addr,
	    pcir_size);
	if (pcir == NULL || pcir->static_regwin == NULL) {
		device_printf(dev,
		    "missing static PCI hostb register window\n");
		return (ENXIO);
	}

	/* Save borrowed reference to the mapped PCI core registers */
	sc->mem_off = pcir->static_regwin->win_offset;
	sc->mem_res = bhndb_find_regwin_resource(sc->bhndb.bus_res,
	    pcir->static_regwin);
	if (sc->mem_res == NULL || !(rman_get_flags(sc->mem_res) & RF_ACTIVE)) {
		device_printf(dev,
		    "no active resource maps the hostb register window\n");
		return (ENXIO);
	}

	sc->bhnd_mem_res = (struct bhnd_resource) {
		.res = sc->mem_res,
		.direct = true
	};

	/* Attach our MMIO device */
	if (sc->pci_devclass == BHND_DEVCLASS_PCIE) {
		device_t child = device_add_child(dev,
		    devclass_get_name(bhnd_mdio_pci_devclass), 0);
		if (child == NULL)
			return (ENXIO);

		if ((error = device_probe_and_attach(child))) {
			device_printf(dev, "failed to attach MDIO device\n");
			return (error);
		}
	}

	return (0);
}

static int
bhndb_pci_detach(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bhndb_generic_detach(dev)))
		return (error);

	if ((error = bhndb_disable_pci_clocks(sc))) {
		device_printf(dev, "failed to disable clocks\n");
		return (error);
	}

	/* Disable PCI bus mastering */
	pci_disable_busmaster(device_get_parent(dev));

	return (0);
}

static int
bhndb_pci_suspend(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bhndb_generic_suspend(dev)))
		return (error);

	if ((error = bhndb_disable_pci_clocks(sc))) {
		device_printf(dev, "suspend failed to disable clocks\n");
		return (error);
	}

	return (0);
}

static int
bhndb_pci_resume(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bhndb_enable_pci_clocks(sc))) {
		device_printf(dev, "resume failed to enable clocks\n");
		return (error);
	}
	
	if ((error = bhndb_generic_resume(dev)))
		return (error);

	return (0);
}

static int
bhndb_pci_set_window_addr(device_t dev, const struct bhndb_regwin *rw,
    bhnd_addr_t addr)
{
	struct bhndb_pci_softc *sc = device_get_softc(dev);
	return (sc->set_regwin(sc, rw, addr));
}

/**
 * A siba(4) and bcma(4)-compatible bhndb_set_window_addr implementation.
 * 
 * On siba(4) devices, it's possible that writing a PCI window register may
 * not succeed; it's necessary to immediately read the configuration register
 * and retry if not set to the desired value.
 * 
 * This is not necessary on bcma(4) devices, but other than the overhead of
 * validating the register, there's no harm in performing the verification.
 */
static int
bhndb_pci_compat_setregwin(struct bhndb_pci_softc *sc,
    const struct bhndb_regwin *rw, bhnd_addr_t addr)
{
	device_t	parent;
	int		error;

	parent = sc->bhndb.parent_dev;

	if (rw->win_type != BHNDB_REGWIN_T_DYN)
		return (ENODEV);

	for (u_int i = 0; i < BHNDB_PCI_BARCTRL_WRITE_RETRY; i++) {
		if ((error = bhndb_pci_fast_setregwin(sc, rw, addr)))
			return (error);

		if (pci_read_config(parent, rw->dyn.cfg_offset, 4) == addr)
			return (0);

		DELAY(10);
	}

	/* Unable to set window */
	return (ENODEV);
}

/**
 * A bcma(4)-only bhndb_set_window_addr implementation.
 */
static int
bhndb_pci_fast_setregwin(struct bhndb_pci_softc *sc,
    const struct bhndb_regwin *rw, bhnd_addr_t addr)
{
	device_t parent = sc->bhndb.parent_dev;

	/* The PCI bridge core only supports 32-bit addressing, regardless
	 * of the bus' support for 64-bit addressing */
	if (addr > UINT32_MAX)
		return (ERANGE);

	switch (rw->win_type) {
	case BHNDB_REGWIN_T_DYN:
		/* Addresses must be page aligned */
		if (addr % rw->win_size != 0)
			return (EINVAL);

		pci_write_config(parent, rw->dyn.cfg_offset, addr, 4);
		break;
	default:
		return (ENODEV);
	}

	return (0);
}


/**
 * Read a 32-bit PCIe TLP/DLLP/PLP protocol register.
 * 
 * @param sc The bhndb_pci driver state.
 * @param addr The protocol register offset.
 */
static uint32_t
bhndb_pcie_read_proto_reg(struct bhndb_pci_softc *sc, uint32_t addr)
{
	uint32_t val;

	KASSERT(bhnd_get_class(sc->bhndb.hostb_dev) == BHND_DEVCLASS_PCIE,
	    ("not a pcie device!"));

	BHNDB_LOCK(&sc->bhndb);
	BHNDB_PCI_WRITE_4(sc, BHND_PCIE_IND_ADDR, addr);
	val = BHNDB_PCI_READ_4(sc, BHND_PCIE_IND_DATA);
	BHNDB_UNLOCK(&sc->bhndb);

	return (val);
}

/**
 * Write a 32-bit PCIe TLP/DLLP/PLP protocol register value.
 * 
 * @param sc The bhndb_pci driver state.
 * @param addr The protocol register offset.
 * @param val The value to write to @p addr.
 */
static void
bhndb_pcie_write_proto_reg(struct bhndb_pci_softc *sc, uint32_t addr,
    uint32_t val)
{
	KASSERT(bhnd_get_class(sc->bhndb.hostb_dev) == BHND_DEVCLASS_PCIE,
	    ("not a pcie device!"));

	BHNDB_LOCK(&sc->bhndb);
	BHNDB_PCI_WRITE_4(sc, BHND_PCIE_IND_ADDR, addr);
	BHNDB_PCI_WRITE_4(sc, BHND_PCIE_IND_DATA, val);
	BHNDB_UNLOCK(&sc->bhndb);
}


/**
 * If supported (and required), enable any externally managed
 * clocks. 
 * 
 * @param sc Bridge driver state.
 */
static int
bhndb_enable_pci_clocks(struct bhndb_pci_softc *sc)
{
	device_t		pci_parent;
	uint32_t		gpio_in, gpio_out, gpio_en;
	uint32_t		gpio_flags;
	uint16_t		pci_status;

	pci_parent = device_get_parent(sc->dev);

	/* Only Broadcom's PCI devices require external clock gating */
	if (sc->pci_devclass != BHND_DEVCLASS_PCI)
		return (0);

	/* Read state of XTAL pin */
	gpio_in = pci_read_config(pci_parent, BHNDB_PCI_GPIO_IN, 4);
	if (gpio_in & BHNDB_PCI_GPIO_XTAL_ON)
		return (0); /* already enabled */

	/* Fetch current config */
	gpio_out = pci_read_config(pci_parent, BHNDB_PCI_GPIO_OUT, 4);
	gpio_en = pci_read_config(pci_parent, BHNDB_PCI_GPIO_OUTEN, 4);

	/* Set PLL_OFF/XTAL_ON pins to HIGH and enable both pins */
	gpio_flags = (BHNDB_PCI_GPIO_PLL_OFF|BHNDB_PCI_GPIO_XTAL_ON);
	gpio_out |= gpio_flags;
	gpio_en |= gpio_flags;

	pci_write_config(pci_parent, BHNDB_PCI_GPIO_OUT, gpio_out, 4);
	pci_write_config(pci_parent, BHNDB_PCI_GPIO_OUTEN, gpio_en, 4);
	DELAY(1000);

	/* Reset PLL_OFF */
	gpio_out &= ~BHNDB_PCI_GPIO_PLL_OFF;
	pci_write_config(pci_parent, BHNDB_PCI_GPIO_OUT, gpio_out, 4);
	DELAY(5000);

	/* Clear any PCI 'sent target-abort' flag. */
	pci_status = pci_read_config(pci_parent, PCIR_STATUS, 2);
	pci_status &= ~PCIM_STATUS_STABORT;
	pci_write_config(pci_parent, PCIR_STATUS, pci_status, 2);

	return (0);
}

/**
 * If supported (and required), disable any externally managed
 * clocks.
 */
static int
bhndb_disable_pci_clocks(struct bhndb_pci_softc *sc)
{
	device_t	parent_dev;
	uint32_t	gpio_out, gpio_en;

	parent_dev = device_get_parent(sc->dev);

	/* Only Broadcom's PCI devices require external clock gating */
	if (sc->pci_devclass != BHND_DEVCLASS_PCI)
		return (0);

	// TODO: Check board flags for BFL2_XTALBUFOUTEN?
	// TODO: Check PCI core revision?
	// TODO: Switch to 'slow' clock?

	/* Fetch current config */
	gpio_out = pci_read_config(parent_dev, BHNDB_PCI_GPIO_OUT, 4);
	gpio_en = pci_read_config(parent_dev, BHNDB_PCI_GPIO_OUTEN, 4);

	/* Set PLL_OFF to HIGH, XTAL_ON to LOW. */
	gpio_out &= ~BHNDB_PCI_GPIO_XTAL_ON;
	gpio_out |= BHNDB_PCI_GPIO_PLL_OFF;
	pci_write_config(parent_dev, BHNDB_PCI_GPIO_OUT, gpio_out, 4);

	/* Enable both output pins */
	gpio_en |= (BHNDB_PCI_GPIO_PLL_OFF|BHNDB_PCI_GPIO_XTAL_ON);
	pci_write_config(parent_dev, BHNDB_PCI_GPIO_OUTEN, gpio_en, 4);

	return (0);
}


/**
 * Find the identification table entry for a core descriptor.
 * 
 * @param sc bhndb PCI driver state.
 */
static const struct bhndb_pci_id *
bhndb_pci_find_core_id(struct bhnd_core_info *core)
{
	const struct bhndb_pci_id *id;

	for (id = bhndb_pci_ids; id->device != BHND_COREID_INVALID; id++) {
		if (core->vendor == BHND_MFGID_BCM && 
		    core->device == id->device)
			return (id);
	}

	return (NULL);
}


/**
 * Return all quirks applicable to the host bridge core.
 * 
 * @param sc bhndb PCI driver state.
 * @param id The host bridge core's identification table entry.
 * 
 * @return Returns the set of quirks applicable to the PCI bridge core,
 * or BHNDB_PCI_QUIRK_INVALID if the host bridge core is not recognized.
 */
static uint32_t 
bhndb_pci_get_hostb_quirks(struct bhndb_pci_softc *sc,
    const struct bhndb_pci_id *id)
{
	struct bhnd_device_quirk	*qt;
	uint32_t			 quirks;
	uint8_t				 hwrev;

	/* Collect all the hwrev quirk flags */
	hwrev = bhnd_get_hwrev(sc->bhndb.hostb_dev);
	quirks = BHNDB_PCI_QUIRK_NONE;
	for (qt = id->quirks; qt->quirks != 0; qt++) {
		if (bhnd_hwrev_matches(hwrev, &qt->hwrev))
			quirks |= qt->quirks;
	};

	// TODO: Additional quirk matching

	return (quirks);
}

/*
 * Support for attaching the PCIe-Gen1 MDIO driver to a parent bhndb PCIe
 * bridge device. 
 */
static int
bhndb_mdio_pcie_probe(device_t dev)
{
	struct bhndb_softc	*psc;
	device_t		 parent;

	/* Parent must be a bhndb_pcie instance */
	parent = device_get_parent(dev);
	if (device_get_driver(parent) != &bhndb_pci_driver)
		return (ENXIO);

	/* Parent must have PCIe-Gen1 hostb device */
	psc = device_get_softc(parent);
	if (psc->hostb_dev == NULL)
		return (ENXIO);

	if (bhnd_get_vendor(psc->hostb_dev) != BHND_MFGID_BCM ||
	    bhnd_get_device(psc->hostb_dev) != BHND_COREID_PCIE)
	{
		return (ENXIO);
	}

	device_quiet(dev);
	return (BUS_PROBE_NOWILDCARD);
}

static int
bhndb_mdio_pcie_attach(device_t dev)
{
	struct bhndb_pci_softc	*psc;
	
	psc = device_get_softc(device_get_parent(dev));

	return (bhnd_mdio_pcie_attach(dev,
	    &psc->bhnd_mem_res, -1,
	    psc->mem_off + BHND_PCIE_MDIO_CTL,
	    (psc->quirks & BHNDB_PCIE_QUIRK_SD_C22_EXTADDR) != 0));

	return (ENXIO);
}

static device_method_t bhnd_mdio_pcie_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhndb_mdio_pcie_probe),
	DEVMETHOD(device_attach,		bhndb_mdio_pcie_attach),
	DEVMETHOD_END
};

static device_method_t bhndb_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhndb_pci_probe),
	DEVMETHOD(device_attach,		bhndb_pci_attach),
	DEVMETHOD(device_detach,		bhndb_pci_detach),
	DEVMETHOD(device_suspend,		bhndb_pci_suspend),
	DEVMETHOD(device_resume,		bhndb_pci_resume),

	/* BHNDB interface */
	DEVMETHOD(bhndb_init_full_config,	bhndb_pci_init_full_config),
	DEVMETHOD(bhndb_set_window_addr,	bhndb_pci_set_window_addr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhndb, bhndb_pci_driver, bhndb_pci_methods,
    sizeof(struct bhndb_pci_softc), bhndb_driver);

DEFINE_CLASS_1(bhnd_mdio_pci, bhndb_mdio_pcie_driver, bhnd_mdio_pcie_methods, 
    sizeof(struct bhnd_mdio_pcie_softc), bhnd_mdio_pcie_driver);

DRIVER_MODULE(bhnd_mdio_pcie, bhndb, bhndb_mdio_pcie_driver,
    bhnd_mdio_pci_devclass, NULL, NULL);

MODULE_VERSION(bhndb_pci, 1);
MODULE_DEPEND(bhndb_pci, bhnd_pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
