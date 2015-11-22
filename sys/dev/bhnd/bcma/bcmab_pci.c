 
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
#include <dev/bhnd/bhndb/bhndb_private.h>

#include <dev/bhnd/cores/bhnd_chipcreg.h>

#include <dev/bhnd/bhndb/bhndb_pcireg.h>
#include <dev/bhnd/bhndb/bhndbvar.h>

#include "bhndb_bus_if.h"
#include "bhndb_if.h"

#include "bcmab_pcivar.h"

#include "bcma_eromvar.h"

static int	bcmab_find_erom_addr(struct bcmab_pci_softc *sc,
		    bhndb_addr_t *erom_addr);

/* Map the ChipCommon core using our bus-provided register windows 
 * and fetch the EROM address. */
static int
bcmab_find_erom_addr(struct bcmab_pci_softc *sc, bhndb_addr_t *erom_addr)
{
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*cc_win;
	struct resource			*res_mem;
	int				 rid, rtype;

	/* Fetch the initial hardware configuration */
	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);
	device_printf(sc->dev, "cfg=%p rwin=%p\n", cfg, cfg->register_windows);

	/* Find the chipc register window */
	cc_win = bhndb_regwin_find_core(cfg->register_windows,
	    BHND_DEVCLASS_CC, 0, 0, 0);
	if (cc_win == NULL) {
		device_printf(sc->dev, "missing chipcommon register window\n");
		return (0);
	}

	/* Allocate the ChipCommon window resource and fetch the EROM table
	 * address. */
	rid = cc_win->res.rid;
	rtype = cc_win->res.type;
	res_mem = bus_alloc_resource_any(sc->parent_dev, rtype, &rid, RF_ACTIVE);
	if (res_mem == NULL) {
		device_printf(sc->dev,
		    "failed to allocate bcmab_pci chipc resource\n");
		return (ENXIO);
	}
	
	*erom_addr = bus_read_4(res_mem, cc_win->win_offset + BCMA_CC_EROM_ADDR);

	/* Clean up */
	bus_release_resource(sc->parent_dev, rtype, rid, res_mem);
	return (0);
}

static int
bcmab_pci_probe(device_t dev)
{	
	device_t	parent;
	devclass_t	parent_bus;
	devclass_t	pci;
	int		error;

	/* Our parent must be a PCI device. */
	pci = devclass_find("pci");
	parent = device_get_parent(dev);
	parent_bus = device_get_devclass(device_get_parent(parent));

	if (parent_bus != pci) {
		device_printf(dev, "attached to non-PCI parent %s\n",
		    device_get_nameunit(parent));
		return (ENXIO);
	}
	

	/* Call default probe implementation */
	if ((error = bhndb_gen_probe(dev)) > 0)
		return (error);
	
	device_set_desc(dev, "PCI-BCMA Bridge");
	return (error);
}

static int
bcmab_pci_attach(device_t dev)
{
	struct bcmab_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	sc->dev = dev;
	sc->parent_dev = device_get_parent(dev);

	/* Extract the EROM address for later use */
	if ((error = bcmab_find_erom_addr(sc, &sc->erom_addr)))
		return (error);

	/* Attach bridged bcma(4) bus */
	if (BUS_ADD_CHILD(dev, 0, devclass_get_name(bcma_devclass), 0) == NULL)
		return (ENXIO);

	/* Call default attach implementation */
	return bhndb_gen_attach(dev);
}

static int
bcmab_pci_detach(device_t dev)
{
	/* Call default detach implementation */
	return bhndb_gen_detach(dev);
}

static bhndb_addr_t
bcmab_pci_get_enum_addr(device_t dev, device_t child)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);
	return (sc->erom_addr);
}

static int
bcmab_pci_get_window_addr(device_t dev, const struct bhndb_regwin *rw,
	bhndb_addr_t *addr)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);

	switch (rw->win_type) {
	case BHNDB_REGWIN_T_CORE:
		// TODO
		return (ENODEV);
	case BHNDB_REGWIN_T_DYN:
		*addr = pci_read_config(sc->parent_dev, rw->dyn.cfg_offset, 4);
		break;
	default:
		return (ENODEV);
	}

	return (0);
}

static int
bcmab_pci_set_window_addr(device_t dev, const struct bhndb_regwin *rw,
	bhndb_addr_t addr)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);

	switch (rw->win_type) {
	case BHNDB_REGWIN_T_DYN:
		pci_write_config(sc->parent_dev, rw->dyn.cfg_offset, addr, 4);
		break;
	default:
		return (ENODEV);
	}

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
	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, dev);

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
		error = BHNDB_SET_WINDOW_ADDR(dev, erom_win, sc->erom_addr);
		if (error)
			return (error);
	}

	/* Allocate the EROM window resource  */
	rid = erom_win->res.rid;
	res_mem = bus_alloc_resource_any(sc->parent_dev, erom_win->res.type, &rid,
	    RF_ACTIVE);
	if (res_mem == NULL) {
		device_printf(dev,
		    "failed to allocate bcmab_pci EROM resource\n");
		return (ENXIO);
	}

	/* Perform the read */
	error = bcma_erom_open(res_mem, erom_win->win_offset, &erom);
	if (error)
		goto cleanup;

	if ((error = bcma_erom_get_core_info(&erom, cores, count)))
		goto cleanup;

cleanup:
	bus_release_resource(sc->parent_dev, erom_win->res.type, rid, res_mem);
	return (error);
}



static device_method_t bcmab_pci_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			bcmab_pci_probe),
	DEVMETHOD(device_attach,		bcmab_pci_attach),
	DEVMETHOD(device_detach,		bcmab_pci_detach),

	/* BHNDB interface */
	DEVMETHOD(bhndb_get_core_table,		bcmab_pci_get_core_table),
	DEVMETHOD(bhndb_get_enum_addr,		bcmab_pci_get_enum_addr),
	DEVMETHOD(bhndb_get_window_addr,	bcmab_pci_get_window_addr),
	DEVMETHOD(bhndb_set_window_addr,	bcmab_pci_set_window_addr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bcmab, bcmab_pci_driver, bcmab_pci_methods,
    sizeof(struct bcmab_pci_softc), bhndb_driver);
// 
MODULE_VERSION(bcmab_pci, 1);
MODULE_DEPEND(bcmab_pci, pci, 1, 1, 1);
MODULE_DEPEND(bcmab_pci, bhndb, 1, 1, 1);
