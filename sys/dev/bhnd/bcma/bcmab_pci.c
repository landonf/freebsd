 
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

#include <dev/bhnd/bridge/bhndb_pcireg.h>
#include <dev/bhnd/bridge/bhndb_pcivar.h>

#include "bhndb_bus_if.h"

#include "bcmab_pcivar.h"

static int
bcmab_pci_probe(device_t dev)
{
	int error;

	/* Call core probe */
	if ((error = bhndb_pci_probe(dev)) > 0)
		return (error);
	
	device_set_desc(dev, "PCI-BCMA Bridge");
	return (error);
}

static int
bcmab_pci_attach(device_t dev)
{
	/* Attach bridged bcma(4) bus */
	if (device_add_child(dev, devclass_get_name(bcma_devclass), 0) == NULL)
		return (ENXIO);

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

MODULE_VERSION(bcmab_pci, 1);
MODULE_DEPEND(bcmab_pci, pci, 1, 1, 1);
MODULE_DEPEND(bcmab_pci, bhndb, 1, 1, 1);

// TODO: move to proper home
static const struct resource_spec bhnd_pci_common_res[] = {
	{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
	{ -1,			0,		0 }
};

static const struct bhndb_regwin bhnd_pci_v1_common_regwin[] = {
	{
		.win_type	= BHNDB_REGWIN_T_DYN,
		.win_offset	= BHNDB_PCI_V1_BAR0_WIN0_OFFSET,
		.win_size	= BHNDB_PCI_V1_BAR0_WIN0_SIZE,
		.dyn.cfg_offset = BHNDB_PCI_V1_BAR0_WIN0_CONTROL,
		.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
	},
	{
		.win_type	= BHNDB_REGWIN_T_CORE,
		.win_offset	= BHNDB_PCI_V1_BAR0_CCREGS_OFFSET,
		.win_size	= BHNDB_PCI_V1_BAR0_CCREGS_SIZE,
		.core = {
			.class	= BHND_DEVCLASS_CC,
			.unit	= 0,
			.port	= 0,
			.region	= 0 
		},
		.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
	},

	BHNDB_REGWIN_TABLE_END
};

const struct bhndb_hwcfg bhnd_pci_v1_common_hwcfg = {
	.resource_specs		= bhnd_pci_common_res,
	.register_windows	= bhnd_pci_v1_common_regwin
};
