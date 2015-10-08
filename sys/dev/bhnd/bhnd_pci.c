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
 * Common BHND PCI Support
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhndvar.h>

// TODO
struct bhnd_pci_softc {};

#define	PCI_VENDOR_BROADCOM		0x14e4

static const struct bhnd_pci_device {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} bhnd_pci_devices[] = {
	{ PCI_VENDOR_BROADCOM,	0x4331,	"Broadcom BCM4331 802.11a/b/g/n Wireless" },
	{ 0, 0, NULL }
};

static int
bhnd_pci_probe(device_t dev)
{
	const struct bhnd_pci_device *ident;
		
	for (ident = bhnd_pci_devices; ident->vendor != 0; ident++) {
		if (pci_get_vendor(dev) == ident->vendor && pci_get_device(dev) == ident->device) {
			device_set_desc(dev, ident->desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
bhnd_pci_attach(device_t dev)
{
	// struct bhnd_pci_softc *sc = device_get_softc(dev);
	
	// TODO
	
	return (0);
}

static int
bhnd_pci_detach(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_pci_suspend(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_pci_resume(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_pci_print_child(device_t dev, device_t child)
{
	return (ENOENT);
}

static void
bhnd_pci_probe_nomatch(device_t dev, device_t child)
{
}

static int
bhnd_pci_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	return (ENOENT);
}

static int
bhnd_pci_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	return (ENOENT);
}


static device_method_t bhnd_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_pci_probe),
	DEVMETHOD(device_attach,	bhnd_pci_attach),
	DEVMETHOD(device_detach,	bhnd_pci_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD(device_suspend,	bhnd_pci_suspend),
	DEVMETHOD(device_resume,	bhnd_pci_resume),
	
	/* Bus interface */
	DEVMETHOD(bus_print_child,	bhnd_pci_print_child),
	DEVMETHOD(bus_probe_nomatch,	bhnd_pci_probe_nomatch),
	DEVMETHOD(bus_read_ivar,	bhnd_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,	bhnd_pci_write_ivar),
	
	// TODO: Additional bus_* methods required.
	
	DEVMETHOD_END
};
static driver_t bhnd_pci_driver = {
	"bhnd",
	bhnd_pci_methods,
	sizeof(struct bhnd_pci_softc)
};
static devclass_t bhnd_devclass;
DRIVER_MODULE(bhnd_pci, pci, bhnd_pci_driver, bhnd_devclass, 0, 0);
MODULE_DEPEND(bhnd_pci, pci, 1, 1, 1);