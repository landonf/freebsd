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
 * Generic PCI BCMA Device Support
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhnd_core.h>
#include <dev/bhnd/bhnd_device_ids.h>

#include "bcma_pcivar.h"

static const struct bcma_pci_device {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} bcma_pci_devices[] = {
	{ PCI_VENDOR_BROADCOM,	PCI_BCM4331_D11N_ID,	"Broadcom BCM4331 802.11a/b/g/n Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_BCM4331_D11N2G_ID,	"Broadcom BCM4331 802.11b/g/n (2GHz) Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_BCM4331_D11N5G_ID,	"Broadcom BCM4331 802.11a/b/g/n (5GHz) Wireless" },
	{ 0, 0, NULL }
};

static struct resource_spec bcma_pci_bmem_spec[] = {
	{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
	{ SYS_RES_MEMORY,	PCIR_BAR(1),	RF_ACTIVE },
	{ -1,			0,		0 }
};

static int
bcma_pci_probe(device_t dev)
{
	const struct bcma_pci_device *ident;
		
	for (ident = bcma_pci_devices; ident->vendor != 0; ident++) {
		if (pci_get_vendor(dev) == ident->vendor && pci_get_device(dev) == ident->device) {
			device_set_desc(dev, ident->desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
bcma_pci_attach(device_t dev)
{
	struct bcma_pci_softc *sc = device_get_softc(dev);
	
	sc->bcma_dev = dev;
	
	pci_enable_busmaster(dev);
	
	/*
	 * Map control/status registers.
	 */
		
	/* Backplane address space */
	if (bus_alloc_resources(dev, bcma_pci_bmem_spec, sc->bmem_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}
	
	return (0);
}

static int
bcma_pci_detach(device_t dev)
{
	struct bcma_pci_softc *sc = device_get_softc(dev);

	bus_release_resources(dev, bcma_pci_bmem_spec, sc->bmem_res);

	return (0);
}

static int
bcma_pci_suspend(device_t dev)
{
	return (ENXIO);
}

static int
bcma_pci_resume(device_t dev)
{
	return (ENXIO);
}

static int
bcma_pci_print_child(device_t dev, device_t child)
{
	return (ENOENT);
}

static void
bcma_pci_probe_nomatch(device_t dev, device_t child)
{
}

static int
bcma_pci_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	return (ENOENT);
}

static int
bcma_pci_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	return (ENOENT);
}


static device_method_t bcma_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcma_pci_probe),
	DEVMETHOD(device_attach,	bcma_pci_attach),
	DEVMETHOD(device_detach,	bcma_pci_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD(device_suspend,	bcma_pci_suspend),
	DEVMETHOD(device_resume,	bcma_pci_resume),
	
	/* Bus interface */
	DEVMETHOD(bus_print_child,	bcma_pci_print_child),
	DEVMETHOD(bus_probe_nomatch,	bcma_pci_probe_nomatch),
	DEVMETHOD(bus_read_ivar,	bcma_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,	bcma_pci_write_ivar),
	
	// TODO: Additional bus_* methods required.
	
	DEVMETHOD_END
};
static driver_t bcma_pci_driver = {
	"bcma",
	bcma_pci_methods,
	sizeof(struct bcma_pci_softc)
};
static devclass_t bhnd_devclass;
DRIVER_MODULE(bcma_pci, pci, bcma_pci_driver, bhnd_devclass, 0, 0);
MODULE_DEPEND(bcma_pci, pci, 1, 1, 1);
MODULE_DEPEND(bcma_pci, bhnd, 1, 1, 1);