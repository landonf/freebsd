/*-
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
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

#include <dev/bhnd/cores/pci/bhnd_pci_hostbvar.h>

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"

struct bhndb_pci_sprom_softc {};

static int
bhndb_pci_sprom_probe(device_t dev)
{
	device_t	bridge, bus;

	/* Our parent must be a PCI-BHND bridge with an attached bhnd bus */
	bridge = device_get_parent(dev);
	if (device_get_driver(bridge) != &bhndb_pci_driver)
		return (ENXIO);
	
	bus = device_find_child(bridge, devclass_get_name(bhnd_devclass), 0);
	if (bus == NULL)
		return (ENXIO);

	/* Found */
	device_set_desc(dev, "PCI-BHNDB SPROM/OTP");
	if (!bootverbose)
		device_quiet(dev);

	/* Refuse wildcard attachments */
	return (BUS_PROBE_NOWILDCARD);
}

static int
bhndb_pci_sprom_attach(device_t dev)
{
	// TODO
	struct resource *r;
	int rid;

	rid = 0;
	r = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (r == NULL) {
		device_printf(dev, "failed to allocate resources\n");
		return (ENXIO);
	}

	device_printf(dev, "%ju byte SROM\n",
	    rman_get_size(r));
	bus_release_resource(dev, SYS_RES_MEMORY, rid, r);

	return (0);
}

static int
bhndb_pci_sprom_resume(device_t dev)
{
	return (0);
}

static int
bhndb_pci_sprom_suspend(device_t dev)
{
	return (0);
}

static int
bhndb_pci_sprom_detach(device_t dev)
{
	return (0);
}


static device_method_t bhndb_pci_sprom_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhndb_pci_sprom_probe),
	DEVMETHOD(device_attach,		bhndb_pci_sprom_attach),
	DEVMETHOD(device_resume,		bhndb_pci_sprom_resume),
	DEVMETHOD(device_suspend,		bhndb_pci_sprom_suspend),
	DEVMETHOD(device_detach,		bhndb_pci_sprom_detach),

	/* NVRAM interface */
	// TODO

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram, bhndb_pci_sprom_driver, bhndb_pci_sprom_methods, sizeof(struct bhndb_pci_sprom_softc));

DRIVER_MODULE(bhndb_pci_sprom, bhndb, bhndb_pci_sprom_driver, bhnd_nvram_devclass, NULL, NULL);
