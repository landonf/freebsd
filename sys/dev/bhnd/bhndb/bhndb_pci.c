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
 * This driver handles all host-level PCI interactions with a PCI/PCIe bridge
 * core operating in endpoint mode. On the bridged bhnd bus, the PCI core
 * device will be managed by a bhnd_pci_hostb driver.
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

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"
#include "bhndb_private.h"

static int	bhndb_enable_pci_clocks(struct bhndb_pci_softc *sc);
static int	bhndb_disable_pci_clocks(struct bhndb_pci_softc *sc);

static int	bhndb_pci_compat_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);
static int	bhndb_pci_fast_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);

static void	bhndb_init_sromless_pci_config(struct bhndb_pci_softc *sc);

#if 0 // TODO!
	// This gets applied to all PCI devices, without fail

	BHND_PCI_QUIRK_EXT_CLOCK_GATING		= (1<<1),

	// This should only apply to siba(4) BCM4303 devices (devices with
	// PCI cores rev < 6)
	// Ideally, we can handle this in the siba_bhndb code itself, and
	// avoid making a mess of the common code by adding legacy work-arounds.
	//
	// Quoth the Broadcom sources: "PCI rev 2.3 support was added in pci
	// core rev 6 and things changed."
	//
	// We may also be able to look at the PCI revision (?)
	/**
	 * Interrupt masking is handled via the interconnect configuration
	 * registers (SBINTVEC on siba), rather than the PCI_INT_MASK
	 * config register.
	 */
	BHND_PCI_QUIRK_SBINTVEC			= (1<<4),
#endif

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

	/* Enable clocks (if supported by this hardware) */
	if ((error = bhndb_enable_pci_clocks(sc)))
		return (error);

	/* Use siba(4)-compatible regwin handling until we know
	 * what kind of bus is attached */
	sc->set_regwin = bhndb_pci_compat_setregwin;

	/* Perform full bridge attach. This should call back into our
	 * bhndb_pci_init_full_config() implementation once the bridged
	 * bhnd(4) bus has been enumerated, but before any devices have been
	 * probed or attached. */
	if ((error = bhndb_attach(dev, sc->pci_devclass)))
		return (error);

	/* If supported, switch to the faster regwin handling */
	if (sc->bhndb.chipid.chip_type != BHND_CHIPTYPE_SIBA) {
		atomic_store_rel_ptr((volatile void *) &sc->set_regwin,
		    (uintptr_t) &bhndb_pci_fast_setregwin);
	}

	return (0);
}

/*
 * On devices without a SROM, the PCI(e) cores will be initialized with
 * their Power-on-Reset defaults; this can leave the the BAR0 PCI windows
 * potentially mapped to the wrong core index.
 * 
 * This function updates the PCI core's BAR0 PCI configuration to point at the
 * current PCI core.
 * 
 * Applies to all PCI/PCIe revisions. Must be applied before bus devices
 * are probed/attached or the SPROM is parsed.
 */
static void
bhndb_init_sromless_pci_config(struct bhndb_pci_softc *sc)
{
	// TODO: Generic register API? Call a bhnd_pci_hostb function?
#if 0
	bus_size_t	sprom_addr;
	u_int		sprom_core_idx;
	u_int		pci_core_idx;
	uint16_t	val;

	/* Fetch the SPROM's configured core index */
	sprom_addr = BPCI_COMMON_REG_OFFSET(SPROM_SHADOW, SRSH_PI_OFFSET);
	val = BHNDB_PCI_READ_2(sc, sprom_addr);

	/* If it doesn't match host bridge's core index, update the index
	 * value */
	sprom_core_idx = BPCI_COMMON_REG_EXTRACT(val, SRSH_PI);
	pci_core_idx = bhnd_get_core_index(sc->bhndb.hostb_dev);

	if (sprom_core_idx != pci_core_idx) {
		val = BPCI_COMMON_REG_INSERT(val, SRSH_PI, pci_core_idx);
		BHNDB_PCI_WRITE_2(sc, sprom_addr, val);
	}
#endif
}

static int
bhndb_pci_resume(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	
	/* Enable clocks (if supported by this hardware) */
	if ((error = bhndb_enable_pci_clocks(sc)))
		return (error);

	/* Perform resume */
	return (bhndb_generic_resume(dev));
}

static int
bhndb_pci_suspend(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	
	/* Disable clocks (if supported by this hardware) */
	if ((error = bhndb_disable_pci_clocks(sc)))
		return (error);

	/* Perform suspend */
	return (bhndb_generic_suspend(dev));
}

static int
bhndb_pci_detach(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	/* Disable clocks (if supported by this hardware) */
	if ((error = bhndb_disable_pci_clocks(sc)))
		return (error);

	/* Perform detach */
	if ((error = bhndb_generic_detach(dev)))
		return (error);

	/* Disable PCI bus mastering */
	pci_disable_busmaster(device_get_parent(dev));

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
 * Enable externally managed clocks, if required.
 * 
 * Some PCI chipsets (BCM4306, possibly others) chips do not support
 * the idle low-power clock. Clocking must be bootstrapped at
 * attach/resume by directly adjusting GPIO registers exposed in the
 * PCI config space, and correspondingly, explicitly shutdown at
 * detach/suspend.
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

	/* Only supported and required on PCI devices */
	if (sc->pci_devclass != BHND_DEVCLASS_PCI)
		return (0);

	pci_parent = device_get_parent(sc->dev);

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
 * Disable externally managed clocks, if required.
 * 
 * @param sc Bridge driver state.
 */
static int
bhndb_disable_pci_clocks(struct bhndb_pci_softc *sc)
{
	device_t	parent_dev;
	uint32_t	gpio_out, gpio_en;

	/* Only supported and required on PCI devices */
	if (sc->pci_devclass != BHND_DEVCLASS_PCI)
		return (0);

	parent_dev = device_get_parent(sc->dev);

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

static device_method_t bhndb_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhndb_pci_probe),
	DEVMETHOD(device_attach,		bhndb_pci_attach),
	DEVMETHOD(device_resume,		bhndb_pci_resume),
	DEVMETHOD(device_suspend,		bhndb_pci_suspend),
	DEVMETHOD(device_detach,		bhndb_pci_detach),

	/* BHNDB interface */
	DEVMETHOD(bhndb_set_window_addr,	bhndb_pci_set_window_addr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhndb, bhndb_pci_driver, bhndb_pci_methods,
    sizeof(struct bhndb_pci_softc), bhndb_driver);

MODULE_VERSION(bhndb_pci, 1);
MODULE_DEPEND(bhndb_pci, bhnd_pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
