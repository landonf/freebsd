 
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

#include <dev/bhnd/bhndb/bhndbvar.h>
#include <dev/bhnd/bhndb/bhndb_pcireg.h>

#include <dev/bhnd/cores/bhnd_chipcreg.h>

#include "bhndb_bus_if.h"
#include "bhndb_if.h"

#include "bcmab_pcivar.h"

#include "bcma_eromvar.h"

static int		 bcmab_find_erom_addr(struct bcmab_pci_softc *sc,
			     bhnd_addr_t *erom_addr);

static int		 bcmab_get_erom_port_addr(struct bcmab_pci_softc *sc,
			     const struct bhndb_regwin *rw,
			     bhnd_addr_t *region_addr,
			     bhnd_size_t *region_size);

static struct resource	*bcmab_alloc_erom_resource(struct bcmab_pci_softc *sc,
			     struct bcma_erom *erom, int *rid, int *type);

/* Map the ChipCommon core using our bus-provided register windows 
 * and fetch the EROM address. */
static int
bcmab_find_erom_addr(struct bcmab_pci_softc *sc, bhnd_addr_t *erom_addr)
{
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*cc_win;
	struct resource			*res_mem;
	int				 rid, rtype;

	/* Fetch the initial hardware configuration */
	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);

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

/**
 * Select (and, if necessary, configure) an EROM-mapping register window and
 * use it to allocate and return a parent resource usable for immediate EROM
 * parsing.
 * 
 * The returned resource should be deallocated using bus_release_resource().
 * 
 * This will fail if the parent resource has already been exclusively acquired,
 * as will be the case once the bridge driver is initialized.
 * 
 * @param sc bcmab driver state.
 * @param[out] erom The open EROM resource.
 * @param[out] rid id of the allocated resource; must be passed to
 * bus_release_resource().
 * @param[out] type type of the allocated resource; must be passed to
 * bus_release_resource().
 * 
 * @retval resource A resource usable for EROM parsing.
 * @retval NULL If an error occured allocating the resource.
 */
static struct resource *
bcmab_alloc_erom_resource(struct bcmab_pci_softc *sc, struct bcma_erom *erom,
    int *rid, int *type)
{
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*rw;
	struct resource			*erom_res;
	int				 error;

	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);

	/* Prefer a fixed EROM mapping window. */
	rw = bhndb_regwin_find_core(cfg->register_windows, BHND_DEVCLASS_EROM,
	    0, 0, 0);

	/* Fall back to a dynamic window */
	if (rw == NULL) {
		rw = bhndb_regwin_find_type(cfg->register_windows,
		    BHNDB_REGWIN_T_DYN);
		
		if (rw == NULL) {
			device_printf(sc->dev,
			    "no usable dynamic register window\n");
			return (NULL);
		}
	}

	/* Allocate the EROM window resource  */
	*rid = rw->res.rid;
	*type = rw->res.type;
	erom_res = bus_alloc_resource_any(sc->parent_dev, rw->res.type,
	    rid, RF_ACTIVE);
	if (erom_res == NULL) {
		device_printf(sc->dev,
		    "failed to allocate bcmab_pci EROM resource\n");
		return (NULL);
	}
	
	/* Configure the dynamic window's base address. This is done after
	 * resource allocation has succeeded to ensure that we do not modify
	 * a window backed by an in-use resource. */
	if (rw->win_type == BHNDB_REGWIN_T_DYN) {
		if ((error = BHNDB_SET_WINDOW_ADDR(sc->dev, rw, sc->erom_addr)))
			goto failed;
	}

	/* Open the EROM for reading */
	if ((error = bcma_erom_open(erom_res, rw->win_offset, erom)))
		goto failed;

	return (erom_res);

failed:
	if (erom_res != NULL)
		bus_release_resource(sc->parent_dev, *type, *rid,
		    erom_res);

	return (NULL);
}

/**
 * Parse the EROM, saving the core information to @p sc.
 * 
 * @param sc bcmab driver state.
 */
static int
bcmab_save_core_info(struct bcmab_pci_softc *sc)
{
	struct bcma_erom		 erom;
	struct resource			*erom_res;
	int				 error;
	int				 rid, type;

	/* Allocate resource for EROM parsing */
	erom_res = bcmab_alloc_erom_resource(sc, &erom, &rid, &type);
	if (erom_res == NULL)
		return (ENODEV);

	/* Cache the core table */
	error = bcma_erom_get_core_info(&erom, &sc->cores, &sc->num_cores);
	if (error)
		goto cleanup;

cleanup:
	bus_release_resource(sc->parent_dev, type, rid, erom_res);
	return (error);
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
	if ((error = bhndb_generic_probe(dev)) > 0)
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
		goto failed;

	/* Cache core info from EROM */
	if ((error = bcmab_save_core_info(sc)))
		goto failed;

	/* Attach bridged bcma(4) bus */
	sc->bus_dev = BUS_ADD_CHILD(dev, 0, devclass_get_name(bcma_devclass), 0);
	if (sc->bus_dev == NULL) {
		error = ENXIO;
		goto failed;
	}

	/* Call default attach implementation */
	return bhndb_generic_attach(dev);

failed:
	if (sc->cores != NULL)
		free(sc->cores, M_BHND);

	return (error);
}

static int
bcmab_pci_detach(device_t dev)
{
	struct bcmab_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	/* Call default implementation before discarding local driver state. */
	if ((error = bhndb_generic_detach(dev)))
		return (error);

	/* Clean up */
	free(sc->cores, M_BHND);

	return (0);
}

static bhnd_addr_t
bcmab_pci_get_enum_addr(device_t dev, device_t child)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);
	return (sc->erom_addr);
}

/**
 * Consult the EROM to find the base address and size of the port region
 * referenced by @p rw.
 * 
 * @param sc Driver state.
 * @param rw A BHNDB_REGWIN_T_CORE register window.
 * @param[out] region_addr If found, the port region's base address.
 * @param[out] region_size If found, the port region's size.
 * 
 * @retval 0 success
 * @retval non-zero not found 
 */
static int bcmab_get_erom_port_addr(struct bcmab_pci_softc *sc,
    const struct bhndb_regwin *rw, bhnd_addr_t *region_addr,
    bhnd_size_t *region_size)
{
	struct bcma_corecfg	*cfg;
	struct bcma_erom	 erom;
	struct bcma_map		*map;
	struct bhnd_core_info	*core;
	struct resource		*erom_res;
	int			 error;
	int			 rid, type;

	KASSERT(rw->win_type == BHNDB_REGWIN_T_CORE, ("unsupported win_type"));
	
	core = NULL;
	cfg = NULL;

	/* Find a core matching the given register window */
	for (u_int i = 0; i < sc->num_cores; i++) {
		struct bhnd_core_info *c = &sc->cores[i];
		if (rw->core.class == bhnd_core_class(c->vendor, c->device) &&
		    rw->core.unit == c->unit)
		{
			core = c;
			break;
		}
	}

	if (core == NULL)
		return (ENOENT);

	/* Allocate EROM parsing state */
	erom_res = bcmab_alloc_erom_resource(sc, &erom, &rid, &type);
	if (erom_res == NULL)
		return (ENODEV);

	/* Parse the corecfg */
	if ((error = bcma_erom_seek_core_index(&erom, core->core_id))) {
		device_printf(sc->dev, "seek to coreid failed!\n");
		goto cleanup;
	}

	if ((error = bcma_erom_parse_corecfg(&erom, &cfg))) {
				device_printf(sc->dev, "corecfg parse failed!\n");

		goto cleanup;
	}

	/* Find the register window's defined region */
	map = bcma_corecfg_find_region_map(cfg, BCMA_SPORT_TYPE_DEVICE,
	    rw->core.port, rw->core.region);
	if (map == NULL) {
		error = ENOENT;
		goto cleanup;
	}

	*region_addr = map->m_base;
	*region_size = map->m_size;

cleanup:
	bus_release_resource(sc->parent_dev, type, rid, erom_res);
	
	if (cfg != NULL)
		bcma_free_corecfg(cfg);

	return (error);
}

/**
 * Find the core/port/region triplet corresponding to @p rw, if any, and
 * write the region's base address to @p addr.
 * 
 * @param sc Driver state.
 * @param rw A BHNDB_REGWIN_T_CORE register window.
 * @param[out] addr If found, the port region's base address.
 *
 * @retval 0 success
 * @retval non-zero not found 
 */
static int bcmab_get_core_regwin_addr(struct bcmab_pci_softc *sc,
    const struct bhndb_regwin *regwin, bhnd_addr_t *addr)
{
	bhnd_addr_t	region_addr;
	bhnd_size_t	region_size;
	int		error;

	KASSERT(regwin->win_type == BHNDB_REGWIN_T_CORE,
	    ("unsupported window type"));

	if (device_is_attached(sc->bus_dev)) {
		/* Prefer fetching device info from the bus device. */
		device_t child;

		/* Find attached device */
		child = bhnd_find_child(sc->bus_dev, regwin->core.class,
		    regwin->core.unit);
		if (child == NULL)
			return (ENOENT);
		
		error = bhnd_get_port_addr(child, regwin->core.port,
		    regwin->core.region, &region_addr, &region_size);
		if (error)
			return (error);
	} else {
		/* If our bus is unavailable, we're running during our own
		 * attachment and must consult the EROM directly */
		error = bcmab_get_erom_port_addr(sc, regwin, &region_addr,
		    &region_size);
		if (error)
			return (error);
	}

	/* The window definition must fit in the mapped region */
	if (regwin->win_size > region_size) {
		device_printf(sc->dev,
		    "register window extends beyond port region at %lx\n",
		    region_addr);
		return (ENODEV);
	}

	*addr = region_addr;
	return (0);
}

static int
bcmab_pci_get_window_addr(device_t dev, const struct bhndb_regwin *rw,
    bhnd_addr_t *addr)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);

	switch (rw->win_type) {
	case BHNDB_REGWIN_T_CORE:
		return (bcmab_get_core_regwin_addr(sc, rw, addr));
	case BHNDB_REGWIN_T_DYN:
		*addr = pci_read_config(sc->parent_dev, rw->dyn.cfg_offset, 4);
		return (0);
	default:
		return (ENODEV);
	}
}

static int
bcmab_pci_set_window_addr(device_t dev, const struct bhndb_regwin *rw,
    bhnd_addr_t addr)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);

	/* The PCI bridge core only supports 32-bit addressing, regardless
	 * of the bus' support for 64-bit addressing */
	if (addr > UINT32_MAX)
		return (ERANGE);

	switch (rw->win_type) {
	case BHNDB_REGWIN_T_DYN:
		pci_write_config(sc->parent_dev, rw->dyn.cfg_offset, addr, 4);
		break;
	default:
		return (ENODEV);
	}

	return (0);
}

static device_t
bcmab_pci_get_attached_bus(device_t dev)
{
	struct bcmab_pci_softc *sc = device_get_softc(dev);
	return (sc->bus_dev);
}

static int
bcmab_pci_get_core_table(device_t dev, struct bhnd_core_info **cores,
	u_int *count)
{
	struct bcmab_pci_softc		*sc;
	struct bhnd_core_info		*result;

	sc = device_get_softc(dev);

	/* Copy out cached core table */
	result = malloc(sizeof(sc->cores[0]) * sc->num_cores, M_BHND, M_WAITOK);
	if (result == NULL)
		return (ENOMEM);

	memcpy(result, sc->cores, sizeof(sc->cores[0]) * sc->num_cores);

	*cores = result;
	*count = sc->num_cores;
	return (0);
}



static device_method_t bcmab_pci_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			bcmab_pci_probe),
	DEVMETHOD(device_attach,		bcmab_pci_attach),
	DEVMETHOD(device_detach,		bcmab_pci_detach),

	/* BHNDB interface */
	DEVMETHOD(bhndb_get_core_table,		bcmab_pci_get_core_table),
	DEVMETHOD(bhndb_get_enum_addr,		bcmab_pci_get_enum_addr),
	DEVMETHOD(bhndb_get_attached_bus,	bcmab_pci_get_attached_bus),
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
