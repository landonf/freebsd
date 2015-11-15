 
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
 * Generic BHND PCI Device Support
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

#include <dev/bhnd/bhnd_private.h>
#include <dev/bhnd/bhndb_pcivar.h>

#include <dev/bhnd/cores/bhnd_pci_hostb.h>

#include "bhndb_bus_if.h"

#include "bcmab_pcivar.h"

static int
bcmab_pci_probe(device_t dev)
{
	return (BUS_PROBE_NOWILDCARD);
}

static int
bcmab_pci_attach(device_t dev)
{
	/* Call core attach */
	return bhndb_pci_attach(dev);
}

static int
bcmab_pci_detach(device_t dev)
{
	/* Call core detach */
	return bhndb_pci_detach(dev);
}

static int
bcmab_pci_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	switch (index) {
	case BHNDB_IVAR_DEV_BASE_ADDR:
		// TODO
		*result = 0;
	default:
		return (bhndb_pci_read_ivar(dev, child, index, result));
	}
}

static int
bcmab_pci_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	switch (index) {
	case BHNDB_IVAR_DEV_BASE_ADDR:
		return (EINVAL);
	default:
		return (bhndb_pci_write_ivar(dev, child, index, value));
	}
}


static int
bcmab_pci_get_core_table(device_t dev, struct bhnd_core_info **cores,
	size_t *count)
{
	panic("unimplemented");
}

static int
bcmab_pci_set_window_register(device_t dev, bus_size_t reg,
	uint32_t addr)
{
	panic("unimplemented");
}

static device_method_t bcmab_pci_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			bcmab_pci_probe),
	DEVMETHOD(device_attach,		bcmab_pci_attach),
	DEVMETHOD(device_detach,		bcmab_pci_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,		bcmab_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,		bcmab_pci_write_ivar),

	/* BHND-HW interface */
	DEVMETHOD(bhndb_get_core_table,		bcmab_pci_get_core_table),
	DEVMETHOD(bhndb_set_window_register,	bcmab_pci_set_window_register),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bcmab, bcmab_pci_driver, bcmab_pci_methods,
    sizeof(struct bhndb_pci_softc), bhndb_pci_driver);