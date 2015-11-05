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
 * XXX: This is a temporary testing stub that will need to be
 * merged into if_bwn.c, or split off into a new driver.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bcma/bcmab_pcivar.h>

#include "if_bwn_pcivar.h"

static const struct bwn_pci_device {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} bwn_pci_devices[] = {
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N,		"Broadcom BCM4331 802.11a/b/g/n Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N2G,	"Broadcom BCM4331 802.11b/g/n (2GHz) Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N5G,	"Broadcom BCM4331 802.11a/b/g/n (5GHz) Wireless" },
	{ 0, 0, NULL }
};

static int
bwn_pci_probe(device_t dev)
{
	const struct bwn_pci_device *ident;
		
	for (ident = bwn_pci_devices; ident->vendor != 0; ident++) {
		if (pci_get_vendor(dev) == ident->vendor && pci_get_device(dev) == ident->device) {
			device_set_desc(dev, ident->desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
bwn_pci_attach(device_t dev)
{
	struct bwn_pci_softc	*sc;

	sc = device_get_softc(dev);
	sc->bwn_dev = dev;

	/* Attach our bcma bus */
	sc->bhnd_dev = device_add_child(dev, BCMAB_DEVNAME, 0);
	if (sc->bhnd_dev == NULL)
		return (ENXIO);

	/* Let the generic implementation probe all added children. */
	return (bus_generic_attach(dev));
}

static int
bwn_pci_detach(device_t dev)
{
	return (bus_generic_detach(dev));
}

static void
bwn_pci_probe_nomatch(device_t dev, device_t child)
{
	const char *name;

	name = device_get_name(child);
	if (name == NULL)
		name = "unknown device";

	device_printf(dev, "<%s> (no driver attached)\n", name);
}

static device_method_t bwn_pci_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			bwn_pci_probe),
	DEVMETHOD(device_attach,		bwn_pci_attach),
	DEVMETHOD(device_detach,		bwn_pci_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bus_generic_suspend), // TODO
	DEVMETHOD(device_resume,		bus_generic_resume), // TODO
	
	/* Bus interface */
	DEVMETHOD(bus_probe_nomatch,		bwn_pci_probe_nomatch),

	DEVMETHOD_END
};

static devclass_t bwn_devclass;

static driver_t bwn_pci_driver = {
    "bwn",
    bwn_pci_methods,
    sizeof(struct bwn_pci_softc)
};

DRIVER_MODULE(bwn_pci, pci, bwn_pci_driver, bwn_devclass, NULL, NULL);
DRIVER_MODULE(bcmab, bwn, bcmab_pci_driver, bcmab_devclass, NULL, NULL);

MODULE_DEPEND(bwn_pci, pci, 1, 1, 1);
MODULE_DEPEND(bwn_pci, bcma, 1, 1, 1);
MODULE_DEPEND(bwn_pci, siba, 1, 1, 1);