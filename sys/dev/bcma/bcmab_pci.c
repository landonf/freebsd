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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhndb_pci.h>
#include <dev/bhnd/bhnd_pcireg.h>

#include "bcmareg.h"
#include "bcmavar.h"
#include "bcmab_pci.h"

struct bcmab_pci_softc {
	struct bhndb_pci_softc	bhnd_softc;	/**< required bhndb_pci state */
	device_t bcma_bus;			/**< the bridged bcma(4) bus */
};

static int
bcmab_pci_probe(device_t dev)
{
	int error;
	
	if ((error = bhndb_pci_generic_probe(dev)) > 0)
		return (error);

	device_set_desc(dev, "PCI-BCMA bridge");
	return (error);
}

static int
bcmab_pci_attach(device_t dev)
{
	struct bcmab_pci_softc	*sc;
	struct resource		*regs;
	device_t		 pci_dev;
	bus_addr_t		 erom_table;
	int			 error;
	int			 rid;

	sc = device_get_softc(dev);
	pci_dev = device_get_parent(dev);

	/* Allocate and map BAR0, which we'll use to enumerate all attached
	 * devices. */
	rid = PCIR_BAR(0);
	regs = bus_alloc_resource_any(pci_dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (regs == NULL) {
		device_printf(dev, "failed to allocate PCI BAR0\n");
		return (ENXIO);
	}
	
	/* Create the child bcma bus device */
	sc->bcma_bus = device_add_child(dev, BCMA_DEVNAME, -1);
	if (sc->bcma_bus == NULL) {
		device_printf(dev, "failed to add bcma device\n");
		return (ENXIO);
	}

	/* Locate and map the bcma bus enumeration table into WIN0, which
	 * should be available on all bcma devices. A pointer to the table can
	 * be found within the ChipCommon register map. */
	erom_table = bus_read_4(regs, BHND_PCI_V2_BAR0_CCREGS_OFFSET +
	    BCMA_CC_EROM_ADDR);
	pci_write_config(pci_dev, BHND_PCI_BAR0_WIN0, erom_table, 4);

	/* Enumerate and register all devices on the bcma bus before
	 * discarding our BAR allocation. */
	error = bcma_scan_erom(sc->bcma_bus, bhnd_generic_probecfg_table, regs,
	    BHND_PCI_V2_BAR0_WIN0_OFFSET);

	bus_release_resource(pci_dev, SYS_RES_MEMORY, rid, regs);

	if (error) {
		device_printf(dev, "erom scan failed\n");
		return (error);
	}

	/* Delegate remainder to the generic implementation */ 
	return (bhndb_pci_generic_attach(dev));
}

static int
bcmab_pci_detach(device_t dev)
{
	return (bhndb_pci_generic_detach(dev));
}

BHND_PCIEB_DECLARE_DRIVER(bcmab_pci, bcmab);
