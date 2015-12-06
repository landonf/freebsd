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
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhnd.h>

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"

static int	bhndb_enable_pci_clocks(device_t dev);
static int	bhndb_disable_pci_clocks(device_t dev);

static int	bhndb_pci_compat_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);
static int	bhndb_pci_fast_setregwin(struct bhndb_pci_softc *,
		    const struct bhndb_regwin *, bhnd_addr_t);

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
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bhndb_enable_pci_clocks(dev))) {
		device_printf(dev, "clock initialization failed\n");
		return (error);
	}

	/* Use siba(4)-compatible regwin handling until we know
	 * what kind of bus is attached */
	sc->set_regwin = bhndb_pci_compat_setregwin;

	/* Perform full bridge attach */
	if ((error = bhndb_generic_attach(dev)))
		return (error);

	/* If supported, switch to the faster regwin handling */
	if (sc->bhndb.chipid.chip_type != BHND_CHIPTYPE_SIBA) {
		atomic_store_rel_ptr((volatile void *) &sc->set_regwin,
		    (uintptr_t) &bhndb_pci_fast_setregwin);
	}

	return (0);
}

static int
bhndb_pci_detach(device_t dev)
{
	int error;

	if ((error = bhndb_generic_detach(dev)))
		return (error);

	if ((error = bhndb_disable_pci_clocks(dev))) {
		device_printf(dev, "failed to disable clocks\n");
		return (error);
	}

	return (0);
}

static int
bhndb_pci_suspend(device_t dev)
{
	int error;

	if ((error = bhndb_generic_suspend(dev)))
		return (error);

	if ((error = bhndb_disable_pci_clocks(dev))) {
		device_printf(dev, "suspend failed to disable clocks\n");
		return (error);
	}

	return (0);
}

static int
bhndb_pci_resume(device_t dev)
{
	int error;

	if ((error = bhndb_enable_pci_clocks(dev))) {
		device_printf(dev, "resume failed to enable clocks\n");
		return (error);
	}
	
	if ((error = bhndb_generic_resume(dev)))
		return (error);

	return (0);
}

static bhnd_devclass_t
bhndb_pci_get_bridge_devclass(device_t dev)
{
	int reg;

	/* Check for a PCIe parent device */
	if (pci_find_cap(device_get_parent(dev), PCIY_EXPRESS, &reg) == 0)
		return (BHND_DEVCLASS_PCIE);
	else
		return (BHND_DEVCLASS_PCI);
}

static int
bhndb_pci_set_window_addr(device_t dev, const struct bhndb_regwin *rw,
    bhnd_addr_t addr)
{
	struct bhndb_pci_softc *sc = device_get_softc(dev);
	return (sc->set_regwin(sc, rw, addr));
}

/* Common read_indirect/write_indirect implementation */
static int
perform_indirect_io_op(device_t dev, bhnd_addr_t addr, uint32_t *value,
    bool write)
{
	struct bhndb_softc		*sc;
	const struct bhndb_regwin	*indrw;
	device_t			 pci_parent;

	sc = device_get_softc(dev);
	pci_parent = sc->parent_dev;
	indrw = sc->ind_regwin;

	if (indrw == NULL)
		return (ENXIO);

	KASSERT(indrw->win_type == BHNDB_REGWIN_T_INDIRECT,
	    ("unsupported regwin type\n"));

	KASSERT(indrw->win_size == sizeof(uint32_t),
	    ("unsupported regwin size\n"));

	/* The PCI bridge only supports 32-bit addressing. */
	if (addr > UINT32_MAX)
		return (ERANGE);

	/* Set the target address */
	device_printf(dev, "ADDR=0x%x\n", pci_read_config(pci_parent, indrw->ind.addr_offset, 4));
	
	pci_write_config(pci_parent, indrw->ind.addr_offset, addr, 4);

	device_printf(dev, "ADDR=0x%x\n", pci_read_config(pci_parent, indrw->ind.addr_offset, 4));

	/* Perform the op */
	if (write) {
		pci_write_config(pci_parent, indrw->ind.data_offset, *value, 4);
	} else {
		*value = pci_read_config(pci_parent, indrw->ind.data_offset, 4);
	}

	return (0);
}

static int
bhndb_pci_read_indirect(device_t dev, bhnd_addr_t addr, uint32_t *value)
{
	return (perform_indirect_io_op(dev, addr, value, false));
}

static int
bhndb_pci_write_indirect(device_t dev, bhnd_addr_t addr, uint32_t value)
{
	return (perform_indirect_io_op(dev, addr, &value, true));
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
		pci_write_config(parent, rw->dyn.cfg_offset, addr, 4);
		break;
	default:
		return (ENODEV);
	}

	return (0);
}

/**
 * If supported (and required) by @p dev, enable any externally managed
 * clocks. 
 */
static int
bhndb_enable_pci_clocks(device_t dev)
{
	device_t		pci_parent;
	uint32_t		gpio_in, gpio_out, gpio_en;
	uint32_t		gpio_flags;
	uint16_t		pci_status;

	/* This may be called prior to softc initialization;
         * we must be able to operate without driver state. */
	pci_parent = device_get_parent(dev);

	/* Only Broadcom's PCI devices require external clock gating */
	if (BHNDB_GET_BRIDGE_DEVCLASS(dev) != BHND_DEVCLASS_PCI)
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
 * If supported (and required) by @p dev, disable any externally managed
 * clocks.
 */
static int
bhndb_disable_pci_clocks(device_t dev)
{
	struct bhndb_softc	*sc;
	uint32_t		gpio_out, gpio_en;

	sc = device_get_softc(dev);

	/* Only Broadcom's PCI devices require external clock gating */
	if (BHNDB_GET_BRIDGE_DEVCLASS(dev) != BHND_DEVCLASS_PCI)
		return (0);

	// TODO: Check board flags for BFL2_XTALBUFOUTEN?
	// TODO: Check PCI core revision?
	// TODO: Switch to 'slow' clock?

	/* Fetch current config */
	gpio_out = pci_read_config(sc->parent_dev, BHNDB_PCI_GPIO_OUT, 4);
	gpio_en = pci_read_config(sc->parent_dev, BHNDB_PCI_GPIO_OUTEN, 4);

	/* Set PLL_OFF to HIGH, XTAL_ON to LOW. */
	gpio_out &= ~BHNDB_PCI_GPIO_XTAL_ON;
	gpio_out |= BHNDB_PCI_GPIO_PLL_OFF;
	pci_write_config(sc->parent_dev, BHNDB_PCI_GPIO_OUT, gpio_out, 4);

	/* Enable both output pins */
	gpio_en |= (BHNDB_PCI_GPIO_PLL_OFF|BHNDB_PCI_GPIO_XTAL_ON);
	pci_write_config(sc->parent_dev, BHNDB_PCI_GPIO_OUTEN, gpio_en, 4);

	return (0);
}


static device_method_t bhndb_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhndb_pci_probe),
	DEVMETHOD(device_attach,		bhndb_pci_attach),
	DEVMETHOD(device_detach,		bhndb_pci_detach),
	DEVMETHOD(device_suspend,		bhndb_pci_suspend),
	DEVMETHOD(device_resume,		bhndb_pci_resume),

	/* BHNDB interface */
	DEVMETHOD(bhndb_get_bridge_devclass,	bhndb_pci_get_bridge_devclass),
	DEVMETHOD(bhndb_set_window_addr,	bhndb_pci_set_window_addr),
	DEVMETHOD(bhndb_read_indirect,		bhndb_pci_read_indirect),
	DEVMETHOD(bhndb_write_indirect,		bhndb_pci_write_indirect),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhndb, bhndb_pci_driver, bhndb_pci_methods,
    sizeof(struct bhndb_pci_softc), bhndb_driver);

MODULE_VERSION(bhndb_pci, 1);
MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
