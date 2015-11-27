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
 * Broadcom PCI-BHND Host Bridge.
 * 
 * This driver is used to "eat" PCI(e) cores operating in endpoint mode when
 * they're attached to a bhndb_pci driver on the host side.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndb/bhndbvar.h>

#include "bhnd_pci_hostb.h"

struct bhnd_pci_hostb_softc {};

static const struct pci_hostb_device {
	uint16_t	 vendor;
	uint16_t	 device;
	const char	*desc;
} pci_hostb_devs[] = {
	{ BHND_MFGID_BCM,	BHND_COREID_PCI,	"PCI-BHND Bridge" },
	{ BHND_MFGID_BCM,	BHND_COREID_PCIE,	"PCI-BHND Bridge (PCIe Gen1)" },
	{ BHND_MFGID_BCM,	BHND_COREID_PCIE2,	"PCI-BHND Bridge (PCIe Gen2)" },
	{ BHND_MFGID_INVALID,	BHND_COREID_INVALID,	NULL }
};

static int
bhnd_pci_hostb_probe(device_t dev)
{
	const struct pci_hostb_device *id;

	/* Query hostb status from parent bridge */
	if (!bhndb_is_hostb_device(dev))
		return (ENXIO);

	for (id = pci_hostb_devs; id->device != BHND_COREID_INVALID; id++) {
		if (bhnd_get_vendor(dev) != id->vendor)
			continue;

		if (bhnd_get_device(dev) != id->device)
			continue;

		device_set_desc(dev, id->desc);
		return (BUS_PROBE_SPECIFIC);
	}

	return (ENXIO);
}

static int
bhnd_pci_hostb_attach(device_t dev)
{
	// TODO - Quirks
	return (0);
}

static int
bhnd_pci_hostb_detach(device_t dev)
{
	return (0);
}

static int
bhnd_pci_hostb_suspend(device_t dev)
{
	return (0);
}

static int
bhnd_pci_hostb_resume(device_t dev)
{
	return (0);
}

static device_method_t bhnd_pci_hostb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_pci_hostb_probe),
	DEVMETHOD(device_attach,	bhnd_pci_hostb_attach),
	DEVMETHOD(device_detach,	bhnd_pci_hostb_detach),
	DEVMETHOD(device_suspend,	bhnd_pci_hostb_suspend),
	DEVMETHOD(device_resume,	bhnd_pci_hostb_resume),
	DEVMETHOD_END
};

static driver_t bhnd_pci_hostb_driver = {
	BHND_HOSTB_DEVNAME,
	bhnd_pci_hostb_methods,
	sizeof(struct bhnd_pci_hostb_softc)
};

static devclass_t bhnd_pci_hostb_devclass;

DRIVER_MODULE(bhnd_pci_hostb, bcma, bhnd_pci_hostb_driver, bhnd_pci_hostb_devclass, 0, 0);
DRIVER_MODULE(bhnd_pci_hostb, siba, bhnd_pci_hostb_driver, bhnd_pci_hostb_devclass, 0, 0);