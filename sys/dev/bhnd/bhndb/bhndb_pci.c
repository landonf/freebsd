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

/** Returns true if @p parent is pcie */
static bool
parent_is_pcie(device_t parent)
{
	int reg;
	return (pci_find_cap(parent, PCIY_EXPRESS, &reg) == 0);
}

// TODO - swap out bcma/siba implementations as required.
static int
bhndb_pci_set_window_addr(device_t dev, const struct bhndb_regwin *rw,
    bhnd_addr_t addr)
{
	struct bhndb_softc *sc = device_get_softc(dev);

	/* The PCI bridge core only supports 32-bit addressing, regardless
	 * of the bus' support for 64-bit addressing */
	if (addr > UINT32_MAX)
		return (ERANGE);

	switch (rw->win_type) {
	case BHNDB_REGWIN_T_DYN:
		pci_write_config(sc->parent_dev, rw->dyn.cfg_offset, addr, 4);
		break;
	default:
		return (ENODEV);
	}

	return (0);
}

/** 
 * Standard bhndb_pci implementation of bhndb_enable_clocks().
 */
static int
bhndb_pci_enable_clocks(device_t dev)
{
	device_t		pci_parent;
	uint32_t		gpio_in, gpio_out, gpio_en;
	uint32_t		gpio_flags;
	uint16_t		pci_status;

	/* This may be called prior to attach(), in which case
	 * we must be able to operate without driver state. */

	/* Broadcom's PCIe devices do not require external clock gating */
	pci_parent = device_get_parent(dev);
	if (parent_is_pcie(pci_parent))
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
 * Standard bhndb_pci implementation of bhndb_disable_clocks().
 */
static int
bhndb_pci_disable_clocks(device_t dev)
{
	struct bhndb_softc	*sc;
	uint32_t		gpio_out, gpio_en;

	sc = device_get_softc(dev);

	KASSERT(sc->parent_dev != NULL,
	    (("called prior to attach()")));

	/* Broadcom's PCIe bridges do not require external clock gate config */
	if (parent_is_pcie(sc->parent_dev))
		return (0);

	// TODO: Check ChipCommon board flags for BFL2_XTALBUFOUTEN?
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

static int
compare_core_index(const void *lhs, const void *rhs)
{
	u_int left = bhnd_get_core_index(*(const device_t *) lhs);
	u_int right = bhnd_get_core_index(*(const device_t *) rhs);

	if (left < right)
		return (-1);
	else if (left > right)
		return (1);
	else
		return (0);
}

/**
 * Default bhndb_pci implementation of bhnd_is_hostb_device().
 * 
 * Returns true if @p child:
 * 
 * - is a Broadcom PCI core attached to the bhnd bus.
 * - is the first core on the bus matching the PCI type (PCI or PCIe) of the
 *   bhndb parent device.
 * 
 * This heuristic should be valid on all currently known PCI/PCIe-bridged
 * devices.
 */
static bool
bhndb_pci_is_hostb_device(device_t dev, device_t child) {
	struct bhndb_softc	*sc;
	struct bhnd_core_match	 md;
	bhnd_devclass_t		 pci_cls;
	device_t		 bhnd_bus;
	device_t		 hostb_dev;
	device_t		*devlist;
	int			 devcnt, error;

	sc = device_get_softc(dev);
	bhnd_bus = BHNDB_GET_ATTACHED_BUS(dev);
	
	/* Requestor must be attached to the bhnd bus */
	if (device_get_parent(child) != bhnd_bus)
		return (false);

	/* Determine required PCI class */
	pci_cls = BHND_DEVCLASS_PCI;
	if (parent_is_pcie(sc->parent_dev))
		pci_cls = BHND_DEVCLASS_PCIE;

	/* Pre-screen the device before searching over the full device list. */
	md = (struct bhnd_core_match) {
		.vendor = BHND_MFGID_BCM,
		.device = BHND_COREID_INVALID,
		.hwrev = { BHND_HWREV_INVALID, BHND_HWREV_INVALID },
		.class = pci_cls,
		.unit = 0
	};

	if (!bhnd_device_matches(child, &md))
		return (false);

	/*
	 * Confirm that this is the absolute first matching device on the bus.
	 */
	if ((error = device_get_children(bhnd_bus, &devlist, &devcnt)))
		return (false);

	/* Sort by core index value, ascending */
	qsort(devlist, devcnt, sizeof(*devlist), compare_core_index);

	/* Find the actual hostb device */
	hostb_dev = NULL;
	for (int i = 0; i < devcnt; i++) {
		if (bhnd_device_matches(devlist[i], &md)) {
			hostb_dev = devlist[i];
			break;
		}
	}

	/* Clean up */
	free(devlist, M_TEMP);

	return (child == hostb_dev);
}

static device_method_t bhndb_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhndb_pci_probe),

	/* BHNDB interface */
	DEVMETHOD(bhndb_set_window_addr,	bhndb_pci_set_window_addr),

	/* BHND interface */
	DEVMETHOD(bhnd_is_hostb_device,		bhndb_pci_is_hostb_device),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhndb, bhndb_pci_driver, bhndb_pci_methods,
    sizeof(struct bhndb_softc), bhndb_driver);

MODULE_VERSION(bhndb_pci, 1);
MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
