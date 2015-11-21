 
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

#include "bhndb_private.h"

#include "bhndb_bus_if.h"

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"
#include "bhndb_private.h"

#define	BHNDB_RES_LOCK_INIT(sc) \
	mtx_init(&(sc)->res_mtx, device_get_nameunit((sc)->dev), \
	    "bhndb_pci resource allocator lock", MTX_DEF)
#define	BHNDB_RES_LOCK(sc)		mtx_lock(&(sc)->res_mtx)
#define	BHNDB_RES_UNLOCK(sc)		mtx_unlock(&(sc)->res_mtx)
#define	BHNDB_RES_LOCK_ASSERT(sc)	mtx_assert(&(sc)->res_mtx, MA_OWNED)
#define	BHNDB_RES_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->res_mtx)

/**
 * A register window managed by our window allocator. 
 */
struct bhndb_pci_regwin_region {
	uintptr_t			 vaddr;	/**< virtual address of the window */
	const struct bhndb_regwin	*win;	/**< window definition */
	struct resource			*res;	/**< enclosing resource */
};

/** bhndb implementation of device_probe(). */
int
bhndb_pci_probe(device_t dev)
{
	device_t	parent;
	devclass_t	parent_bus;
	devclass_t	pci;

	/* Our parent must be a PCI device. */
	pci = devclass_find("pci");
	parent = device_get_parent(dev);
	parent_bus = device_get_devclass(device_get_parent(parent));

	if (parent_bus != pci) {
		device_printf(dev, "bhndb_pci attached to non-PCI parent %s\n",
		    device_get_nameunit(parent));
		return (ENXIO);
	}
	
	return (BUS_PROBE_NOWILDCARD);
}

static void
bhndb_pci_probe_nomatch(device_t dev, device_t child)
{
	const char *name;

	name = device_get_name(child);
	if (name == NULL)
		name = "unknown device";

	device_printf(dev, "<%s> (no driver attached)\n", name);
}

static int
bhndb_pci_child_pnpinfo_str(device_t bus, device_t child, char *buf,
    size_t buflen)
{
	// TODO
	return (ENXIO);
}

static int
bhndb_pci_child_location_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	// TODO
	return (ENXIO);
}

/**
 * Return true if @p cores matches the @p hw specification.
 * 
 * @param dev The bhndb device.
 * @param cores A core table fetched from @p dev.
 * @param num_cores The number of cores in @p cores.
 * @param hw The hardware description to be matched against.
 */
static bool
bhndb_pci_hw_matches(device_t dev, struct bhnd_core_info *cores,
    u_int num_cores, const struct bhndb_hw *hw)
{
	for (u_int i = 0; i < hw->num_hw_reqs; i++) {
		const struct bhnd_core_match *match =  &hw->hw_reqs[i];

		if (bhnd_match_core(cores, num_cores, match) == NULL)
			return (false);
	}

	return (true);
}

/**
 * Find a hardware configuration for @p dev.
 * 
 * @param dev The bhndb device.
 * @param[out] hwconfig On success, the hardware configuration to be used
 * with @p dev.
 * 
 * @retval 0 success
 * @retval non-zero if an error occurs fetching device info for comparison.
 */
static int
bhndb_pci_find_hwcfg(device_t dev, const struct bhndb_hwcfg **hwcfg)
{
	struct bhndb_pci_softc	*sc;
	const struct bhndb_hw	*hw;
	struct bhnd_core_info	*cores;
	u_int			 num_cores;
	int			 error;

	sc = device_get_softc(dev);
	error = 0;

	/* Fetch our core table */
	if ((error = BHNDB_GET_CORE_TABLE(dev, &cores, &num_cores)))
		return (error);
	
	/* Search for the first matching hardware config */
	for (hw = bhndb_pci_hw; hw->hw_reqs != NULL; hw++) {
		if (bhndb_pci_hw_matches(dev, cores, num_cores, hw)) {
			device_printf(dev, "%s register map\n",
			    hw->name);
			*hwcfg = hw->cfg;
			goto finished;
		}
	}

	/* Not found */
	error = ENOENT;

finished:
	free(cores, M_BHND);
	return (error);
}

/** Allocate and initialize the BHNDB_REGWIN_T_DYN region allocator state. */
static int
init_dw_region_allocator(struct bhndb_pci_softc *sc)
{
	const struct resource_spec	*rspecs;
	const struct bhndb_regwin	*rws;
	const struct bhndb_regwin	*win;
	u_int				 region_idx;
	int				 error;
	
	rws = sc->hwcfg->register_windows;
	rspecs = sc->hwcfg->resource_specs;

	sc->dw_regions = NULL;

	/* Fetch and verify the dynamic regwin count does not exceed
	 * what is representable via our bitfield freelist. */
	sc->dw_count = bhndb_regwin_count(rws, BHNDB_REGWIN_T_DYN);
	if (sc->dw_count >= (8 * sizeof(sc->dw_free_list))) {
		device_printf(sc->dev, "max dynamic regwin count exceeded\n");
		return (ENOMEM);
	}
	
	/* Allocate the region table. */
	sc->dw_regions = malloc(sizeof(struct bhndb_pci_regwin_region) * 
	    sc->dw_count, M_BHND, M_WAITOK);
	if (sc->dw_regions == NULL) {
		return (ENOMEM);
	}

	/* Initialize the region table and freelist. */
	sc->dw_free_list = 0;
	region_idx = 0;
	for (win = rws; win->win_type != BHNDB_REGWIN_T_INVALID; win++)
	{
		struct bhndb_pci_regwin_region *region;

		/* Skip non-DYN windows */
		if (win->win_type != BHNDB_REGWIN_T_DYN)
			continue;

		region = &sc->dw_regions[region_idx];
		region->win = win;
		region->res = NULL;

		/* Find the corresponding resource. */
		for (u_int rsi = 0; rspecs[rsi].type != -1; rsi++) {			
			if (win->res.type != rspecs[rsi].type)
				continue;

			if (win->res.rid != rspecs[rsi].rid)
				continue;

			/* Found matching rspec/resource */
			region->res = sc->res[rsi];
			break;
		}

		/* Invalid hwcfg? */
		if (region->res == NULL) {
			device_printf(sc->dev,
			    "missing regwin resource spec (type=%d, rid=%d)\n",
			    win->res.type, win->res.rid);
			error = EINVAL;
			goto failed;
		}

		/* Cache the resource vaddr */
		region->vaddr = (uintptr_t) rman_get_virtual(region->res);
		region->vaddr += win->win_offset;

		/* Add to freelist */
		sc->dw_free_list |= (1 << region_idx);

		region_idx++;
	}

	return (0);

failed:
	if (sc->dw_regions != NULL)
		free(sc->dw_regions, M_BHND);

	return (error);
}

/** bhndb implementation of device_attach(). */
int
bhndb_pci_attach(device_t dev)
{
	struct bhndb_pci_softc		*sc;
	int				 error;
	bool				 free_mem_rman = false;
	bool				 free_pci_res = false;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->pci_dev = device_get_parent(dev);

	/* Find our register window configuration */
	if ((error = bhndb_pci_find_hwcfg(dev, &sc->hwcfg)))
		return (error);

	/* Initialize locking */
	BHNDB_RES_LOCK_INIT(sc);

	/* Set up a resource manager for the device's address space. */
	sc->mem_rman.rm_start = 0;
	sc->mem_rman.rm_end = BUS_SPACE_MAXADDR_32BIT;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "bhnd device address space";
	
	if (rman_init(&sc->mem_rman) ||
	    rman_manage_region(&sc->mem_rman, 0, BUS_SPACE_MAXADDR_32BIT))
	{
		device_printf(dev, "could not initialize mem_rman\n");
		return (ENXIO);
	} else {
		free_mem_rman = true;
	}


	/* Determine our PCI resource count from the hardware config. */
	sc->res_count = 0;
	for (size_t i = 0; sc->hwcfg->resource_specs[i].type != -1; i++)
		sc->res_count++;

	/* Allocate space for a non-const copy of our resource_spec
	 * table; this will be updated with the RIDs assigned by
	 * bus_alloc_resources. */
	sc->res_spec = malloc(sizeof(struct resource_spec) * (sc->res_count + 1),
	    M_BHND, M_WAITOK);
	if (sc->res_spec == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Initialize and terminate the table */
	for (size_t i = 0; i < sc->res_count; i++)
		sc->res_spec[i] = sc->hwcfg->resource_specs[i];
	
	sc->res_spec[sc->res_count].type = -1;

	/* Allocate space for our resource references */
	sc->res = malloc(sizeof(struct resource) * sc->res_count,
	    M_BHND, M_WAITOK);
	if (sc->res == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Allocate resources */
	error = bus_alloc_resources(sc->pci_dev, sc->res_spec, sc->res);
	if (error) {
		device_printf(dev,
		    "could not allocate PCI resources on %s: %d\n",
		    device_get_nameunit(sc->pci_dev), error);
		goto failed;
	} else {
		free_pci_res = true;
	}

	/* Initialize dynamic window allocator state */
	if ((error = init_dw_region_allocator(sc)))
		goto failed;

	return (bus_generic_attach(dev));

failed:
	if (free_mem_rman)
		rman_fini(&sc->mem_rman);

	if (free_pci_res)
		bus_release_resources(dev, sc->res_spec, sc->res);

	if (sc->res != NULL)
		free(sc->res, M_BHND);

	if (sc->res_spec != NULL)
		free(sc->res_spec, M_BHND);
	
	BHNDB_RES_LOCK_DESTROY(sc);

	return (error);
}

/** bhndb implementation of device_detach(). */
int
bhndb_pci_detach(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	
	/* Detach children */
	error = bus_generic_detach(dev);

	/* Clean up */
	rman_fini(&sc->mem_rman);

	bus_release_resources(dev, sc->res_spec, sc->res);
	free(sc->res, M_BHND);
	free(sc->res_spec, M_BHND);

	free(sc->dw_regions, M_BHND);
	BHNDB_RES_LOCK_DESTROY(sc);

	return (error);
}

/** bhndb implementation of device_suspend(). */
int
bhndb_pci_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

/** bhndb implementation of device_resume(). */
int
bhndb_pci_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

/** bhndb implementation of bus_read_ivar(). */
int
bhndb_pci_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	switch (index) {
	default:
		return (ENOENT);
	}
}

/** bhndb implementation of bus_write_ivar(). */
int
bhndb_pci_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	switch (index) {
	default:
		return (ENOENT);
	}
}

static struct rman *
bhndb_pci_get_rman(device_t dev, int type)
{
	struct bhndb_pci_softc *sc = device_get_softc(dev);

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

static device_t
bhndb_pci_add_child(device_t dev, u_int order, const char *name, int unit)
{
	struct bhndb_devinfo	*dinfo;
	device_t		 child;
	
	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return (NULL);

	dinfo = malloc(sizeof(struct bhndb_devinfo), M_BHND, M_WAITOK);
	if (dinfo == NULL) {
		device_delete_child(dev, child);
		return (NULL);
	}

	resource_list_init(&dinfo->resources);

	device_set_ivars(child, dinfo);

	return (child);
}

static void
bhndb_pci_child_deleted(device_t dev, device_t child)
{
	struct bhndb_devinfo *dinfo = device_get_ivars(child);
	if (dinfo != NULL)
		resource_list_free(&dinfo->resources);

	device_set_ivars(child, NULL);
}

static struct resource *
bhndb_pci_alloc_resource(device_t dev, device_t child, int type,
    int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct resource_list_entry	*rle;
	struct resource			*rv;
	struct rman			*rm;
	int				 error;
	bool				 immed_child, defaults;

	immed_child = (device_get_parent(child) == dev);
	defaults = (start == 0ULL && end == ~0ULL);
	rle = NULL;

	/* Populate defaults */
	if (immed_child && defaults) {
		/* Fetch the resource list entry. */
		rle = resource_list_find(BUS_GET_RESOURCE_LIST(dev, child),
		    type, *rid);
		if (rle == NULL) {
			device_printf(dev,
			    "default resource %#x type %d for child %s "
			    "not found\n", *rid, type,
			    device_get_nameunit(child));
			
			return (NULL);
		}
		
		if (rle->res != NULL) {
			device_printf(dev,
			    "resource entry %#x type %d for child %s is busy\n",
			    *rid, type, device_get_nameunit(child));
			
			return (NULL);
		}

		start = rle->start;
		end = rle->end;
		if (count < rle->count)
			count = rle->count;
	}

	/* Validate resource addresses */
	if (start > end || end < start || count > (end - start))
		return (NULL);
	
	/* Fetch the resource manager */
	rm = bhndb_pci_get_rman(dev, type);
	if (rm == NULL)
		return (NULL);

	/* Make our reservation */
	rv = rman_reserve_resource(rm, start, end, count, flags & ~RF_ACTIVE,
	    child);
	if (rv == NULL)
		return (NULL);
	
	rman_set_rid(rv, *rid);

	/* Activate */
	if (flags & RF_ACTIVE) {
		error = bus_activate_resource(child, type, *rid, rv);
		if (error) {
			device_printf(dev,
			    "failed to activate entry %#x type %d for "
				"child %s\n",
			     *rid, type, device_get_nameunit(child));

			rman_release_resource(rv);

			return (NULL);
		}
	}

	/* Update child's resource list entry */
	if (rle != NULL) {
		rle->res = rv;
		rle->start = rman_get_start(rv);
		rle->end = rman_get_end(rv);
		rle->count = rman_get_size(rv);
	}

	return (rv);
}

static int
bhndb_pci_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	int error;

	/* Release any reserved bridged resources */
	if (rman_get_flags(r) & RF_ACTIVE) {
		error = BUS_DEACTIVATE_RESOURCE(dev, child, type, rid, r);
		if (error)
			return (error);
	}

	if ((error = rman_release_resource(r)))
		return (error);

	return (0);
}

static int
bhndb_pci_adjust_resource(device_t dev, device_t child, int type,
    struct resource *r, u_long start, u_long end)
{
	// TODO
	return (EINVAL);
}

static int
bhndb_pci_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	struct bhndb_pci_softc		*sc;
	struct bhndb_pci_regwin_region	*region;
	int				 region_ctz;
	int				 error;

	sc = device_get_softc(dev);
	
	BHNDB_RES_LOCK(sc);

	/* No windows available? */
	if (sc->dw_free_list == 0) {
		BHNDB_RES_UNLOCK(sc);
		return (ENOMEM);
	}

	/* Claim the first free region */
	region_ctz = __builtin_ctz(sc->dw_free_list);
	sc->dw_free_list &= ~(1 << region_ctz);
	region = &sc->dw_regions[region_ctz];

	// TODO - track RID to support deallocation?

	BHNDB_RES_UNLOCK(sc);

	/* Set the target window */
	error = BHNDB_SET_WINDOW_ADDR(dev, region->win, rman_get_start(r));
	if (error)
		goto failed;

	/* Configure resource with its real bus values. */
	rman_set_virtual(r, (void *) region->vaddr);
	rman_set_bustag(r, rman_get_bustag(region->res));
	rman_set_bushandle(r, rman_get_bushandle(region->res));

	/* Mark active */
	if ((error = rman_activate_resource(r)))
		goto failed;

	return (0);
failed:
	/* Release our region allocation. */
	BHNDB_RES_LOCK(sc);
	sc->dw_free_list |= (1 << region_ctz);
	BHNDB_RES_UNLOCK(sc);

	return (error);
}

static int
bhndb_pci_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	// TODO - window resource deactivations
	return (0);
}


static struct resource_list *
bhndb_pci_get_resource_list(device_t dev, device_t child)
{
	struct bhndb_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

static struct bhnd_resource *
bhndb_pci_alloc_bhnd_resource(device_t dev, device_t child, int type,
     int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	// struct bhnd_resource *r;

	// TODO
	return (NULL);
}

static int
bhndb_pci_release_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	// int error;

	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_pci_activate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	// TODO
	return (EOPNOTSUPP);
};

static int
bhndb_pci_deactivate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	// TODO
	return (EOPNOTSUPP);
};

static int
bhndb_pci_setup_intr(device_t dev, device_t child, struct resource *r,
    int flags, driver_filter_t filter, driver_intr_t handler, void *arg,
    void **cookiep)
{
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_pci_teardown_intr(device_t dev, device_t child, struct resource *r,
    void *cookie)
{
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_pci_config_intr(device_t dev, int irq, enum intr_trigger trig,
    enum intr_polarity pol)
{
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_pci_bind_intr(device_t dev, device_t child, struct resource *r, int cpu) {
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_pci_describe_intr(device_t dev, device_t child, struct resource *irq, void *cookie,
    const char *descr)
{
	// TODO
	return (EOPNOTSUPP);
}

static bus_dma_tag_t
bhndb_pci_get_dma_tag(device_t dev, device_t child)
{
	// TODO
	return (NULL);
}

static device_method_t bhndb_pci_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			bhndb_pci_probe),
	DEVMETHOD(device_attach,		bhndb_pci_attach),
	DEVMETHOD(device_detach,		bhndb_pci_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bhndb_pci_suspend),
	DEVMETHOD(device_resume,		bhndb_pci_resume),

	/* Bus interface */
	DEVMETHOD(bus_probe_nomatch,		bhndb_pci_probe_nomatch),
	DEVMETHOD(bus_child_pnpinfo_str,	bhndb_pci_child_pnpinfo_str),
	DEVMETHOD(bus_child_location_str,	bhndb_pci_child_location_str),
	DEVMETHOD(bus_add_child,		bhndb_pci_add_child),
	DEVMETHOD(bus_child_deleted,		bhndb_pci_child_deleted),

	DEVMETHOD(bus_alloc_resource,		bhndb_pci_alloc_resource),
	DEVMETHOD(bus_release_resource,		bhndb_pci_release_resource),
	DEVMETHOD(bus_activate_resource,	bhndb_pci_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bhndb_pci_deactivate_resource),

	DEVMETHOD(bus_setup_intr,		bhndb_pci_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bhndb_pci_teardown_intr),
	DEVMETHOD(bus_config_intr,		bhndb_pci_config_intr),
	DEVMETHOD(bus_bind_intr,		bhndb_pci_bind_intr),
	DEVMETHOD(bus_describe_intr,		bhndb_pci_describe_intr),

	DEVMETHOD(bus_get_dma_tag,		bhndb_pci_get_dma_tag),

	DEVMETHOD(bus_adjust_resource,		bhndb_pci_adjust_resource),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_get_resource_list,	bhndb_pci_get_resource_list),

	DEVMETHOD(bus_read_ivar,		bhndb_pci_read_ivar),
	DEVMETHOD(bus_write_ivar,		bhndb_pci_write_ivar),

	/* BHND interface */
	DEVMETHOD(bhnd_alloc_resource,		bhndb_pci_alloc_bhnd_resource),
	DEVMETHOD(bhnd_release_resource,	bhndb_pci_release_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_pci_activate_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_pci_deactivate_bhnd_resource),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhndb, bhndb_pci_driver, bhndb_pci_methods, sizeof(struct bhndb_pci_softc));

MODULE_VERSION(bhndb_pci, 1);

MODULE_DEPEND(bhndb_pci, pci, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhnd, 1, 1, 1);
MODULE_DEPEND(bhndb_pci, bhndb, 1, 1, 1);
