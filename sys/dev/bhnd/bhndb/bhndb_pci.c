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

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"
#include "bhndb_private.h"

static int	bhndb_enable_pci_clocks(struct bhndb_pci_softc *sc);
static int	bhndb_disable_pci_clocks(struct bhndb_pci_softc *sc);

static int	bhndb_pci_compat_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);
static int	bhndb_pci_fast_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);

static uint32_t	bhndb_pci_get_hostb_quirks(struct bhndb_pci_softc *,
		    const struct bhndb_pci_id *);

static const struct bhndb_pci_id *bhndb_pci_find_core_id(
				      struct bhnd_core_info *core);


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

	{ BHND_COREID_INVALID, BHND_PCI_REGS_PCI, NULL }
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
	sc->regf = id->regf;
	sc->quirks = bhndb_pci_get_hostb_quirks(sc, id);


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

	/* Save reference to the mapped PCI core registers */
	sc->res_offset = pcir->static_regwin->win_offset;
	sc->res = bhndb_find_regwin_resource(sc->bhndb.bus_res,
	    pcir->static_regwin);
	if (sc->res == NULL || !(rman_get_flags(sc->res) & RF_ACTIVE)) {
		device_printf(dev,
		    "no active resource maps the hostb register window\n");
		return (ENXIO);
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

MODULE_VERSION(bhndb_pci, 1);
MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
