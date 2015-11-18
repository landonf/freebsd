 
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
 * PCI-BCMA Bridge Device Support
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

#include <dev/bhnd/cores/bhnd_chipcreg.h>

#include <dev/bhnd/bridge/bhndb_pcireg.h>
#include <dev/bhnd/bridge/bhndb_pcivar.h>

#include "bhndb_bus_if.h"
#include "bhndb_if.h"

#include "bcmab_pcivar.h"

#include "bcma_eromreg.h"
#include "bcma_eromvar.h"

struct bcmab_pci_softc {
	struct bhndb_pci_softc	bhndb_softc;	/**< parent softc */
	device_t		pci_dev;	/**< parent PCI device */
	bus_addr_t		erom_addr;	/**< EROM's base address */
};


/**
 * Generic PCI-BCMA bridge configuration usable with all known bcma(4)-based
 * PCI devices; this configuration is adequate for enumerating a bridged
 * bcma(4) bus to determine the full hardware configuration.
 * 
 * Compatible with PCI_V0, PCI_V1, PCI_V2, and PCI_V3 devices.
 * 
 * This configuration defines the full set of register windows required
 * for both siba(4) and bcma(4) bus enumeration, but note that this
 * configuration is not compatible with siba(4) devices bridged via a PCI_V0
 * core.
 */
const struct bhndb_hwcfg bcmab_pci_generic_hwcfg = {
	.resource_specs		= (const struct resource_spec[]) {
		{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
		{ -1,			0,		0 }
	},

	.register_windows	= (const struct bhndb_regwin[]) {
		/* bar0+0x0000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V1_BAR0_WIN0_OFFSET,
			.win_size	= BHNDB_PCI_V1_BAR0_WIN0_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V1_BAR0_WIN0_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},

		/* bar0+0x3000: chipc core registers */
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
	},
};

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

/* Map the ChipCommon core using our bus-provided register windows 
 * and fetch the EROM address. */
static int
bcmab_find_erom_addr(device_t dev, bus_addr_t *erom_addr)
{
	struct bcmab_pci_softc		*sc;
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*cc_win;
	struct resource			*res_mem;
	int				 rid, rtype;

	sc = device_get_softc(dev);

	/* Fetch the initial hardware configuration */
	cfg = BHNDB_BUS_GET_HWCFG(sc->pci_dev, dev);

	/* Find the chipc register window */
	cc_win = bhndb_regwin_find_core(cfg->register_windows,
	    BHND_DEVCLASS_CC, 0, 0, 0);
	if (cc_win == NULL) {
		device_printf(dev, "missing chipcommon register window\n");
		return (0);
	}

	/* Allocate the ChipCommon window resource and fetch the EROM table
	 * address. */
	rid = cc_win->res.rid;
	rtype = cc_win->res.type;
	res_mem = bus_alloc_resource_any(sc->pci_dev, rtype, &rid, RF_ACTIVE);
	if (res_mem == NULL) {
		device_printf(dev,
		    "failed to allocate bcmab_pci chipc resource\n");
		return (ENXIO);
	}
	
	*erom_addr = bus_read_4(res_mem, cc_win->win_offset + BCMA_CC_EROM_ADDR);

	/* Clean up */
	bus_release_resource(sc->pci_dev, rtype, rid, res_mem);
	return (0);
}

static int
bcmab_pci_attach(device_t dev)
{
	struct bcmab_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	sc->pci_dev = device_get_parent(dev);

	/* Extract the EROM address for later use */
	if ((error = bcmab_find_erom_addr(dev, &sc->erom_addr)))
		return (error);

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
	struct bcmab_pci_softc	*sc;
	
	sc = device_get_softc(dev);

	switch (index) {
	case BHNDB_IVAR_DEV_BASE_ADDR:
		*result = sc->erom_addr;
		return (0);
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
bcmab_pci_set_window_register(device_t dev, const struct bhndb_regwin *rw,
	uint32_t addr)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);

	KASSERT(rw->win_type == BHNDB_REGWIN_T_DYN,
	    ("non-dynamic register window type %d", rw->win_type));

	pci_write_config(sc->pci_dev, rw->dyn.cfg_offset, addr, 4);
	return (0);
}

static int
bcmab_pci_get_core_table(device_t dev, struct bhnd_core_info **cores,
	u_int *count)
{
	struct bcmab_pci_softc		*sc;
	struct bcma_erom		 erom;
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*erom_win;
	struct resource			*res_mem;
	int				 error;
	int				 rid;

	sc = device_get_softc(dev);
	cfg = BHNDB_BUS_GET_HWCFG(sc->pci_dev, dev);

	/* Prefer a fixed EROM mapping window. */
	erom_win = bhndb_regwin_find_core(cfg->register_windows,
	    BHND_DEVCLASS_EROM, 0, 0, 0);

	/* Fall back to a dynamic window */
	if (erom_win == NULL) {
		erom_win = bhndb_regwin_find_type(cfg->register_windows,
		    BHNDB_REGWIN_T_DYN);
		
		if (erom_win == NULL) {
			device_printf(dev,
			    "no usable dynamic register window\n");
			return (ENXIO);
		}
	
		/* Configure the dynamic window's base address. */
		error = BHNDB_SET_WINDOW_REGISTER(dev, erom_win, sc->erom_addr);
		if (error)
			return (error);
	}

	/* Allocate the EROM window resource  */
	rid = erom_win->res.rid;
	res_mem = bus_alloc_resource_any(sc->pci_dev, erom_win->res.type, &rid,
	    RF_ACTIVE);
	if (res_mem == NULL) {
		device_printf(dev,
		    "failed to allocate bcmab_pci EROM resource\n");
		return (ENXIO);
	}

	/* Perform the read */
	error = bcma_erom_open(res_mem, BCMA_EROM_TABLE_START, &erom);
	if (error)
		goto cleanup;

	bcma_erom_get_core_info(&erom, cores, count);

cleanup:
	bus_release_resource(sc->pci_dev, erom_win->res.type, rid, res_mem);
	return (error);
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
    sizeof(struct bcmab_pci_softc), bhndb_pci_driver);

MODULE_VERSION(bcmab_pci, 1);
MODULE_DEPEND(bcmab_pci, pci, 1, 1, 1);
MODULE_DEPEND(bcmab_pci, bhndb, 1, 1, 1);
