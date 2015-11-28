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

#include "bhndb_pcivar.h"

/** 
 * Default bhndb_pci implementation of device_probe().
 * 
 * Verifies that the parent is a PCI/PCIe device.
 */
int
bhndb_pci_generic_probe(device_t dev)
{
	device_t	parent;
	devclass_t	parent_bus;
	devclass_t	pci;

	/* Our parent must be a PCI device. */
	pci = devclass_find("pci");
	parent = device_get_parent(dev);
	parent_bus = device_get_devclass(device_get_parent(parent));

	if (parent_bus != pci) {
		device_printf(dev, "attached to non-PCI parent %s\n",
		    device_get_nameunit(parent));
		return (ENXIO);
	}

	return (BUS_PROBE_NOWILDCARD);
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
bool
bhndb_pci_generic_is_hostb_device(device_t dev, device_t child) {
	struct bhndb_pci_softc	*sc;
	struct bhnd_core_match	 md;
	bhnd_devclass_t		 pci_cls;
	device_t		 bhnd_bus;
	device_t		 hostb_dev;
	device_t		*devlist;
	int			 devcnt, error, pcireg;

	sc = device_get_softc(dev);
	bhnd_bus = BHNDB_GET_ATTACHED_BUS(dev);
	
	/* Requestor must be attached to the bhnd bus */
	if (device_get_parent(child) != bhnd_bus)
		return (false);

	/* Determine required PCI class */
	pci_cls = BHND_DEVCLASS_PCI;
	if (pci_find_cap(sc->bhndb_sc.parent_dev, PCIY_EXPRESS, &pcireg) == 0)
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
	DEVMETHOD(device_probe,		bhndb_pci_generic_probe),

	/* BHND interface */
	DEVMETHOD(bhnd_is_hostb_device,	bhndb_pci_generic_is_hostb_device),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhndb_pci, bhndb_pci_driver, bhndb_pci_methods,
    sizeof(struct bhndb_pci_softc), bhndb_driver);

MODULE_VERSION(bhndb_pci, 1);
MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
