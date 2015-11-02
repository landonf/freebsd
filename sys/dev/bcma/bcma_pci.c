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
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhnd_core.h>
#include <dev/bhnd/bhnd_device_ids.h>

#include <dev/bhnd/bhnd_pci.h>
#include <dev/bhnd/bhnd_pcireg.h>

#include "bcmavar.h"
#include "bcmareg.h"

#include "bcma_pcivar.h"

static const struct bcma_pci_device {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} bcma_pci_devices[] = {
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N,		"Broadcom BCM4331 802.11a/b/g/n Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N2G,	"Broadcom BCM4331 802.11b/g/n (2GHz) Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N5G,	"Broadcom BCM4331 802.11a/b/g/n (5GHz) Wireless" },
	{ 0, 0, NULL }
};

static struct resource_spec bcma_pci_res_spec[BCMA_PCI_MAX_RES+1] = {
	{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
	{ SYS_RES_MEMORY,	PCIR_BAR(1),	RF_ACTIVE },
	{ -1,			0,		0 }
};


#define BPCI_RES_BAR0	0	/* bar0 pci_res index */
#define BPCI_RES_BAR1	1	/* bar1 pci_res index */

#if 0
static const struct bcma_pci_regwin bcma_pci_v0_regwin[] = {
	{
		.pci_res	= BPCI_RES_BAR0,
		.win_type	= BCMA_PCI_WINTYPE_DYN,
		.cfg_offset	= BHND_PCI_BAR0_WIN0,
		.win_offset	= BHND_PCI_V0_BAR0_WIN0_OFFSET,
		.win_size	= BHND_PCI_V0_BAR0_WIN0_SIZE
	},
	{
		.bhnd_class	= BHND_DEVCLASS_PCI,
		.win_type	= BCMA_PCI_WINTYPE_FIXED,
		.port_num	= 0,
		.region_num	= 0,
		.pci_res	= BPCI_RES_BAR0,
		.win_offset	= BHND_PCI_V0_BAR0_PCIREGS_OFFSET,
		.win_size	= BHND_PCI_V0_BAR0_PCIREGS_SIZE
	}
};
#define	BCMA_PCI_V0_REGWIN_NUM \
    (sizeof(bcma_pci_v0_regwin) / sizeof(bcma_pci_v0_regwin[0])

static const struct bcma_pci_regwin bcma_pci_v1_windows[] = {
	{
		.pci_res	= BPCI_RES_BAR0,
		.win_type	= BCMA_PCI_WINTYPE_DYN,
		.cfg_offset	= BHND_PCI_BAR0_WIN0,
		.win_offset	= BHND_PCI_V1_BAR0_WIN0_OFFSET,
		.win_size	= BHND_PCI_V1_BAR0_WIN0_SIZE
	},
	{
		.bhnd_class	= BHND_DEVCLASS_PCI,
		.win_type	= BCMA_PCI_WINTYPE_FIXED,
		.port_num	= 0,
		.region_num	= 0,
		.pci_res	= BPCI_RES_BAR0,
		.win_offset	= BHND_PCI_V1_BAR0_PCIREGS_OFFSET,
		.win_size	= BHND_PCI_V1_BAR0_PCIREGS_SIZE
	}
};
#define	BCMA_PCI_V1_REGWIN_NUM \
    (sizeof(bcma_pci_v1_regwin) / sizeof(bcma_pci_v1_regwin[0])

static const struct bcma_pci_regwin bcma_pci_v2_windows[] = {
	{
		.pci_res	= BPCI_RES_BAR0,
		.win_type	= BCMA_PCI_WINTYPE_DYN,
		.cfg_offset	= BHND_PCI_BAR0_WIN0,
		.win_offset	= BHND_PCI_V2_BAR0_WIN0_OFFSET,
		.win_size	= BHND_PCI_V2_BAR0_WIN0_SIZE
	},
	{
		.pci_res	= BPCI_RES_BAR0,
		.win_type	= BCMA_PCI_WINTYPE_DYN,
		.cfg_offset	= BHND_PCI_BAR0_WIN1,
		.win_offset	= BHND_PCI_V2_BAR0_WIN1_OFFSET,
		.win_size	= BHND_PCI_V2_BAR0_WIN1_SIZE
	},
	{
		.bhnd_class	= BHND_DEVCLASS_PCI,
		.win_type	= BCMA_PCI_WINTYPE_FIXED,
		.port_num	= 0,
		.region_num	= 0,
		.pci_res	= BPCI_RES_BAR0,
		.win_offset	= BHND_PCI_V2_BAR0_PCIREGS_OFFSET,
		.win_size	= BHND_PCI_V2_BAR0_PCIREGS_SIZE
	},
	{
		.bhnd_class	= BHND_DEVCLASS_CC,
		.win_type	= BCMA_PCI_WINTYPE_FIXED,
		.port_num	= 0,
		.region_num	= 0,
		.pci_res	= BPCI_RES_BAR0,
		.win_offset	= BHND_PCI_V2_BAR0_CCREGS_OFFSET,
		.win_size	= BHND_PCI_V2_BAR0_CCREGS_SIZE
	}
};
#define	BCMA_PCI_V2_REGWIN_NUM \
    (sizeof(bcma_pci_v2_regwin) / sizeof(bcma_pci_v2_regwin[0])
#endif

#define BMEM_RES_CHIPC	BPCI_RES_BAR0

#define bcma_bar_read(res, base, offset, size) \
	bus_read_ ## size (res, (base) + (offset))

#define bcma_read_chipc(sc, offset, size) \
	bcma_bar_read(sc->pci_res[BPCI_RES_BAR0], BHND_PCI_V2_BAR0_CCREGS_OFFSET, offset, size)

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
	struct bcma_pci_softc	*sc;
	device_t		 pci_core;
	uint32_t		 erom_table;
	int			 error;
	bool			 free_mem_rman = false;
	bool			 free_pci_res = false;

	sc = device_get_softc(dev);
	sc->bcma_dev = dev;
	pci_enable_busmaster(dev);
	
	/* Set up a resource manager for the device's address space. */
	sc->mem_rman.rm_start = 0;
	sc->mem_rman.rm_end = BUS_SPACE_MAXADDR_32BIT;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "bmca bus addresses";
	
	if (rman_init(&sc->mem_rman) ||
	    rman_manage_region(&sc->mem_rman, 0, BUS_SPACE_MAXADDR_32BIT))
	{
		device_printf(dev, "could not initialize mem_rman\n");
		return (ENXIO);
	} else {
		free_mem_rman = true;
	}


	/* Map our PCI device resources. */
	error = bus_alloc_resources(dev, bcma_pci_res_spec, sc->pci_res);
	if (error) {
		device_printf(dev, "could not allocate PCI resources\n");
		goto failed;
	} else {
		free_pci_res = true;
	}


	/* Locate and map the enumeration table into WIN1. A pointer to the
	 * table can be found within the ChipCommon register map. */
	erom_table = bcma_read_chipc(sc, BCMA_CC_EROM_ADDR, 4);
	pci_write_config(dev, BHND_PCI_BAR0_WIN1, erom_table, 4);

	/* Enumerate and register all bus devices. */
	error = bcma_scan_erom(dev, bhnd_generic_probecfg_table,
	    sc->pci_res[BMEM_RES_CHIPC], BHND_PCI_V2_BAR0_WIN1_OFFSET);
	if (error)
		goto failed;
	

	// TODO - determine PCI core configuration
	pci_core = bhnd_find_child(dev, BHND_DEVCLASS_PCI);
	if (pci_core == NULL) {
		device_printf(dev, "could not find a PCI core\n");
		return (ENXIO);
	}
	device_printf(pci_core, "PCI revision: %hhu\n", bhnd_get_revid(pci_core));

	/* Let the generic implementation probe all added children. */
	return (bus_generic_attach(dev));

failed:
	if (free_mem_rman)
		rman_fini(&sc->mem_rman);

	if (free_pci_res)
		bus_release_resources(dev, bcma_pci_res_spec, sc->pci_res);

	return (error);
}

static int
bcma_pci_detach(device_t dev)
{
	struct bcma_pci_softc *sc = device_get_softc(dev);

	bus_release_resources(dev, bcma_pci_res_spec, sc->pci_res);

	return (0);
}

static struct rman *
bcma_pci_get_rman(device_t dev, int type)
{
	struct bcma_pci_softc *sc = device_get_softc(dev);

	switch (type) {
	case SYS_RES_MEMORY:
		return &sc->mem_rman;
	case SYS_RES_IRQ:
		// TODO
		// return &sc->irq_rman;
		return (NULL);
	default:
		return (NULL);
	};
}

/* delegate all remaining driver methods to generic bcma implementations */
#define	bcma_pci_read_ivar		bcma_generic_read_ivar
#define	bcma_pci_write_ivar		bcma_generic_write_ivar
#define	bcma_pci_child_deleted		bcma_generic_child_deleted
#define	bcma_pci_get_resource_list	bcma_generic_get_resource_list
#define	bcma_pci_get_port_rid		bcma_generic_get_port_rid

/* declare our bcma_pci driver */
BHND_PCI_DECLARE_DRIVER(bcma_pci, bcma);