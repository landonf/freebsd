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
 * Abstract BHND Bridge Device Driver
 * 
 * Provides generic support for bridging from a parent bus (such as PCI) to
 * a BHND-compatible bus (e.g. bcma or siba).
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd_private.h>

#include "bhndbvar.h"
#include "bhndb_bus_if.h"
#include "bhndb_private.h"

#define	BHNDB_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), \
	    "bhndb_gen resource allocator lock", MTX_DEF)
#define	BHNDB_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	BHNDB_UNLOCK(sc)	mtx_unlock(&(sc)->sc_mtx)
#define	BHNDB_LOCK_ASSERT(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)
#define	BHNDB_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->sc_mtx)

/**
 * Mark a dynamic window region as free.
 */
#define	BHNDB_DW_REGION_RELEASE(sc, rnid)	do {		\
	KASSERT((sc)->dw_regions[rnid].child_res != NULL &&	\
	    !BHNDB_DW_REGION_IS_FREE((sc), (rnid)),		\
	    (("dw_region double free")));			\
								\
	(sc)->dw_freelist |= (1 << (rnid));			\
	(sc)->dw_regions[rnid].child_res = NULL;		\
} while(0)

/**
 * Mark a dynamic window region as reserved.
 */
#define	BHNDB_DW_REGION_RESERVE(sc, rnid, cr)	do {		\
	KASSERT((sc)->dw_regions[rnid].child_res == NULL &&	\
	    BHNDB_DW_REGION_IS_FREE((sc), (rnid)),		\
	    (("dw_region is busy")));				\
								\
	(sc)->dw_freelist &= ~(1 << (rnid));			\
	(sc)->dw_regions[rnid].child_res = cr;			\
} while(0)

/**
 * Return non-zero value if a dynamic window region is marked as free.
 */
#define	BHNDB_DW_REGION_IS_FREE(sc, rnid) \
	((sc)->dw_freelist & (1 << (rnid)))

/** bhndb child instance state */
struct bhndb_devinfo {
        struct resource_list    resources;	/**< child resources. */
};

/**
 * A register window allocation record. 
 */
struct bhndb_regwin_region {
	uintptr_t			vaddr;		/**< virtual address of the window */
	const struct bhndb_regwin	*win;		/**< window definition */
	struct resource			*res;		/**< enclosing resource */
	struct resource			*child_res;	/**< associated child resource, or NULL */
	u_int				 rnid;		/**< region identifier */
};

static bool				 bhndb_hw_matches(
					     struct bhnd_core_info *cores,
					     u_int num_cores,
					     const struct bhndb_hw *hw);

static int				 bhndb_find_hwspec(
					     struct bhndb_softc *sc,
					     const struct bhndb_hw **hw);

static int				 bhndb_init_dw_region_allocator(
					     struct bhndb_softc *sc);

static struct rman			*bhndb_get_rman(struct bhndb_softc *sc,
					     int type);

static struct bhndb_regwin_region 	*bhndb_find_resource_region(
					     struct bhndb_softc *sc,
					     device_t child, int type, int rid,
					     struct resource *r);

/** 
 * Default bhnd implementation of device_probe().
 */
int
bhndb_gen_probe(device_t dev)
{
	return (BUS_PROBE_NOWILDCARD);
}

static void
bhndb_probe_nomatch(device_t dev, device_t child)
{
	const char *name;

	name = device_get_name(child);
	if (name == NULL)
		name = "unknown device";

	device_printf(dev, "<%s> (no driver attached)\n", name);
}

static int
bhndb_print_child(device_t dev, device_t child)
{
	struct bhndb_softc	*sc;
	struct resource_list	*rl;
	bhndb_addr_t		 addr;
	int			retval = 0;

	sc = device_get_softc(dev);
	addr = BHNDB_GET_ENUM_ADDR(dev, child);

	retval += bus_print_child_header(dev, child);

	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl != NULL) {
		retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY,
		    "%#lx");
		retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ,
		    "%ld");
	}

	retval += bus_print_child_domain(dev, child);
	retval += bus_print_child_footer(dev, child);

	return (retval);
}

static int
bhndb_child_pnpinfo_str(device_t bus, device_t child, char *buf,
    size_t buflen)
{
	*buf = '\0';
	return (0);
}

static int
bhndb_child_location_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	bhndb_addr_t addr;

	addr = BHNDB_GET_ENUM_ADDR(dev, child);
	snprintf(buf, buflen, "enum_base=0x%llx", (unsigned long long) addr);
	return (0);
}

/**
 * Return true if @p cores matches the @p hw specification.
 * 
 * @param cores A core table to match against.
 * @param num_cores The number of cores in @p cores.
 * @param hw The hardware description to be matched against.
 */
static bool
bhndb_hw_matches(struct bhnd_core_info *cores,
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
 * Find a hardware specification for @p dev.
 * 
 * @param dev The bhndb device.
 * @param[out] hw On success, the matched hardware specification.
 * with @p dev.
 * 
 * @retval 0 success
 * @retval non-zero if an error occurs fetching device info for comparison.
 */
static int
bhndb_find_hwspec(struct bhndb_softc *sc, const struct bhndb_hw **hw)
{
	const struct bhndb_hw	*next, *hw_table;
	struct bhnd_core_info	*cores;
	u_int			 num_cores;
	int			 error;

	error = 0;

	/* Fetch our core table */
	if ((error = BHNDB_GET_CORE_TABLE(sc->dev, &cores, &num_cores)))
		return (error);
	
	/* Search for the first matching hardware config. */
	hw_table = BHNDB_BUS_GET_HARDWARE_TABLE(sc->parent_dev, sc->dev);
	for (next = hw_table; next->hw_reqs != NULL; next++) {
		if (bhndb_hw_matches(cores, num_cores, next)) {
			*hw = next;
			goto done;
		}
	}

	/* Not found */
	error = ENOENT;

done:
	free(cores, M_BHND);
	return (error);
}

/** Allocate and initialize the BHNDB_REGWIN_T_DYN region allocator state. */
static int
bhndb_init_dw_region_allocator(struct bhndb_softc *sc)
{
	const struct bhndb_hwcfg	*cfg;
	const struct resource_spec	*rspecs;
	const struct bhndb_regwin	*rws;
	const struct bhndb_regwin	*win;
	u_int				 rnid;
	int				 error;

	cfg = sc->hw->cfg;
	rws = cfg->register_windows;
	rspecs = cfg->resource_specs;

	sc->dw_regions = NULL;

	/* Fetch and verify the dynamic regwin count does not exceed
	 * what is representable via our freelist bitmask. */
	sc->dw_count = bhndb_regwin_count(rws, BHNDB_REGWIN_T_DYN);
	if (sc->dw_count >= (8 * sizeof(sc->dw_freelist))) {
		device_printf(sc->dev, "max dynamic regwin count exceeded\n");
		return (ENOMEM);
	}
	
	/* Allocate the region table. */
	sc->dw_regions = malloc(sizeof(struct bhndb_regwin_region) * 
	    sc->dw_count, M_BHND, M_WAITOK);
	if (sc->dw_regions == NULL) {
		return (ENOMEM);
	}

	/* Initialize the region table and freelist. */
	sc->dw_freelist = 0;
	rnid = 0;
	for (win = rws; win->win_type != BHNDB_REGWIN_T_INVALID; win++)
	{
		struct bhndb_regwin_region *region;

		/* Skip non-DYN windows */
		if (win->win_type != BHNDB_REGWIN_T_DYN)
			continue;

		region = &sc->dw_regions[rnid];
		region->win = win;
		region->res = NULL;
		region->child_res = NULL;
		region->rnid = rnid;

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
		sc->dw_freelist |= (1 << rnid);

		rnid++;
	}

	return (0);

failed:
	if (sc->dw_regions != NULL)
		free(sc->dw_regions, M_BHND);

	return (error);
}

/**
 * Default bhndb implementation of device_attach().
 */
int
bhndb_gen_attach(device_t dev)
{
	struct bhndb_softc		*sc;
	int				 error;
	bool				 free_mem_rman = false;
	bool				 free_parent_res = false;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->parent_dev = device_get_parent(dev);

	BHNDB_LOCK_INIT(sc);

	/* Find our register window configuration */
	if ((error = bhndb_find_hwspec(sc, &sc->hw)))
		return (error);

	if (bootverbose)
		device_printf(dev, "%s register map\n", sc->hw->name);

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


	/* Determine our bridge resource count from the hardware config. */
	sc->res_count = 0;
	for (size_t i = 0; sc->hw->cfg->resource_specs[i].type != -1; i++)
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
		sc->res_spec[i] = sc->hw->cfg->resource_specs[i];
	
	sc->res_spec[sc->res_count].type = -1;

	/* Allocate space for our resource references */
	sc->res = malloc(sizeof(struct resource) * sc->res_count,
	    M_BHND, M_WAITOK);
	if (sc->res == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Allocate resources */
	error = bus_alloc_resources(sc->parent_dev, sc->res_spec, sc->res);
	if (error) {
		device_printf(dev,
		    "could not allocate bridge resources on %s: %d\n",
		    device_get_nameunit(sc->parent_dev), error);
		goto failed;
	} else {
		free_parent_res = true;
	}

	/* Initialize dynamic window allocator state */
	if ((error = bhndb_init_dw_region_allocator(sc)))
		goto failed;

	return (bus_generic_attach(dev));

failed:
	if (free_mem_rman)
		rman_fini(&sc->mem_rman);

	if (free_parent_res)
		bus_release_resources(dev, sc->res_spec, sc->res);

	if (sc->res != NULL)
		free(sc->res, M_BHND);

	if (sc->res_spec != NULL)
		free(sc->res_spec, M_BHND);
	
	BHNDB_LOCK_DESTROY(sc);

	return (error);
}

/** Default bhndb implementation of device_detach(). */
int
bhndb_gen_detach(device_t dev)
{
	struct bhndb_softc	*sc;
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
	BHNDB_LOCK_DESTROY(sc);

	return (error);
}

/** Default bhndb implementation of device_suspend(). */
int
bhndb_gen_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

/** Default bhndb implementation of device_resume(). */
int
bhndb_gen_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

/** Default bhndb implementation of bus_read_ivar(). */
int
bhndb_gen_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	return (ENOENT);
}

/** Default bhndb implementation of bus_write_ivar(). */
int
bhndb_gen_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	return (ENOENT);
}

static struct rman *
bhndb_get_rman(struct bhndb_softc *sc, int type)
{
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
bhndb_add_child(device_t dev, u_int order, const char *name, int unit)
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
bhndb_child_deleted(device_t dev, device_t child)
{
	struct bhndb_devinfo *dinfo = device_get_ivars(child);
	if (dinfo != NULL) {
		resource_list_free(&dinfo->resources);
		free(dinfo, M_BHND);
	}

	device_set_ivars(child, NULL);
}

static struct resource *
bhndb_alloc_resource(device_t dev, device_t child, int type,
    int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct bhndb_softc		*sc;
	struct resource_list_entry	*rle;
	struct resource			*rv;
	struct rman			*rm;
	int				 error;
	bool				 immed_child, defaults;

	sc = device_get_softc(dev);
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
	rm = bhndb_get_rman(sc, type);
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
bhndb_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	int error;

	/* Deactivate resources */
	if (rman_get_flags(r) & RF_ACTIVE) {
		error = BUS_DEACTIVATE_RESOURCE(dev, child, type, rid, r);
		if (error)
			return (error);
	}

	if ((error = rman_release_resource(r)))
		return (error);

	return (0);
}


/**
 * Find the region allocated for @p r, if any.
 */
static struct bhndb_regwin_region *
bhndb_find_resource_region(struct bhndb_softc *sc, device_t child, int type,
    int rid, struct resource *r)
{
	struct bhndb_regwin_region	*region, *ret;
	struct rman			*rm;
	
	BHNDB_LOCK_ASSERT(sc);

	if ((rm = bhndb_get_rman(sc, type)) == NULL)
		return (NULL);

	ret = NULL;
	for (size_t i = 0; i < sc->dw_count; i++) {
		region = &sc->dw_regions[i];

		/* Skip free regions */
		if (BHNDB_DW_REGION_IS_FREE(sc, i))
			continue;

		/* Match dev/type-rm/rid triplet */
		if (rman_get_device(region->child_res) != child)
			continue;

		if (!rman_is_region_manager(region->child_res, rm))
			continue;

		if (rman_get_rid(region->child_res) != rid)
			continue;

		/* Matching region found */
		ret = region;
		break;
	}

	return (ret);
}

static int
bhndb_adjust_resource(device_t dev, device_t child, int type,
    struct resource *r, u_long start, u_long end)
{
	struct bhndb_softc		*sc;
	struct rman			*rm;
	struct bhndb_regwin_region	*region;
	int				 error;
	
	sc = device_get_softc(dev);
	error = 0;

	/* Fetch resource manager */
	rm = bhndb_get_rman(sc, type);
	if (rm == NULL)
		return (ENXIO);

	if (!rman_is_region_manager(r, rm))
		return (ENXIO);

	/* If active, adjustment is limited by the assigned window. */
	BHNDB_LOCK(sc);
	region = NULL;
	if (rman_get_flags(r) & RF_ACTIVE) {
		region = bhndb_find_resource_region(sc, child, type,
			rman_get_rid(r), r);
		if (region == NULL) {
			error = ENXIO;
			goto finish;
		}
	}

	/* If active, the base address is fixed, and the end address cannot
	 * grow past the end of the allocated window. */
	if (region != NULL) {
		if (rman_get_start(r) != start  ||
		    end < start ||
		    end - start > region->win->win_size)
		{
			error = EINVAL;
			goto finish;
		}
	}

finish:
	BHNDB_UNLOCK(sc);
	if (!error)
		error = rman_adjust_resource(r, start, end);

	return (error);
}

static int
bhndb_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	struct bhndb_softc		*sc;
	struct bhndb_regwin_region	*region;
	uint32_t			 freelist;
	int				 rnid;
	int				 error;

	sc = device_get_softc(dev);

	/*
	 * Find the first free window of sufficient size.
	 * 
	 * On all currently known hardware, dynamic windows are 4K and 
	 * register blocks are 4K or smaller; this check should always
	 * succeed.
	 */
	BHNDB_LOCK(sc);
	freelist = sc->dw_freelist;
	do {
		/* No windows available? */
		if (freelist == 0) {
			BHNDB_UNLOCK(sc);
			return (ENOMEM);
		}
		
		/* Find the next free window */
		rnid = __builtin_ctz(freelist);
		freelist &= ~(1 << rnid);

		region = &sc->dw_regions[rnid];

	} while (region->win->win_size < rman_get_size(r));

	/* Mark region as allocated */
	BHNDB_DW_REGION_RESERVE(sc, rnid, r);
	BHNDB_UNLOCK(sc);


	/* Set the target window */
	error = BHNDB_SET_WINDOW_ADDR(dev, region->win, rman_get_start(r));
	if (error)
		goto failed;

	/* Mark active */
	if ((error = rman_activate_resource(r)))
		goto failed;

	/* Configure resource with its real bus values. */
	rman_set_virtual(r, (void *) region->vaddr);
	rman_set_bustag(r, rman_get_bustag(region->res));
	rman_set_bushandle(r, rman_get_bushandle(region->res));

	return (0);

failed:
	/* Release our region allocation. */
	BHNDB_LOCK(sc);
	BHNDB_DW_REGION_RELEASE(sc, rnid);
	BHNDB_UNLOCK(sc);

	return (error);
}

static int
bhndb_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	struct bhndb_regwin_region	*region;
	struct bhndb_softc		*sc;
	struct rman			*rm;
	int				 error;

	sc = device_get_softc(dev);

	if ((rm = bhndb_get_rman(sc, type)) == NULL)
		return (EINVAL);

	/* Mark inactive */
	if ((error = rman_deactivate_resource(r)))
		return (error);

	/* Free any dynamic window allocation. */
	BHNDB_LOCK(sc);
	region = bhndb_find_resource_region(sc, child, type, rid, r);
	if (region != NULL)
		BHNDB_DW_REGION_RELEASE(sc, region->rnid);
	BHNDB_UNLOCK(sc);

	return (0);
}


static struct resource_list *
bhndb_get_resource_list(device_t dev, device_t child)
{
	struct bhndb_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

static struct bhnd_resource *
bhndb_alloc_bhnd_resource(device_t dev, device_t child, int type,
     int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct bhnd_resource	*br;
	
	br = malloc(sizeof(struct bhnd_resource), M_BHND, M_WAITOK);
	if (br == NULL)
		return (NULL);

	// TODO - support hwcfg specification of indirect targets.
	br->_direct = true;
	br->_res = bus_alloc_resource(child, type, rid, start, end, count,
	    flags & ~RF_ACTIVE);
	if (br->_res == NULL)
		goto failed;
	

	if (flags & RF_ACTIVE) {
		if (bhnd_activate_resource(child, type, *rid, br))
			goto failed;
	}

	return (br);

failed:
	if (br->_res != NULL)
		bus_release_resource(child, type, *rid, br->_res);

	free(br, M_BHND);
	return (NULL);
}

static int
bhndb_release_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	int error;

	if ((error = bus_release_resource(child, type, rid, r->_res)))
		return (error);

	free(r, M_BHND);
	return (0);
}

static int
bhndb_activate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	/* Indirect resources don't require activation */
	if (!r->_direct)
		return (0);

	return (bus_activate_resource(child, type, rid, r->_res));
};

static int
bhndb_deactivate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	/* Indirect resources don't require activation */
	if (!r->_direct)
		return (0);

	return (bus_deactivate_resource(child, type, rid, r->_res));
};

static int
bhndb_setup_intr(device_t dev, device_t child, struct resource *r,
    int flags, driver_filter_t filter, driver_intr_t handler, void *arg,
    void **cookiep)
{
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_teardown_intr(device_t dev, device_t child, struct resource *r,
    void *cookie)
{
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_config_intr(device_t dev, int irq, enum intr_trigger trig,
    enum intr_polarity pol)
{
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_bind_intr(device_t dev, device_t child, struct resource *r, int cpu)
{
	// TODO
	return (EOPNOTSUPP);
}

static int
bhndb_describe_intr(device_t dev, device_t child, struct resource *irq, void *cookie,
    const char *descr)
{
	// TODO
	return (EOPNOTSUPP);
}

static bus_dma_tag_t
bhndb_get_dma_tag(device_t dev, device_t child)
{
	// TODO
	return (NULL);
}

static device_method_t bhndb_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			bhndb_gen_probe),
	DEVMETHOD(device_attach,		bhndb_gen_attach),
	DEVMETHOD(device_detach,		bhndb_gen_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bhndb_gen_suspend),
	DEVMETHOD(device_resume,		bhndb_gen_resume),

	/* Bus interface */
	DEVMETHOD(bus_probe_nomatch,		bhndb_probe_nomatch),
	DEVMETHOD(bus_print_child,		bhndb_print_child),
	DEVMETHOD(bus_child_pnpinfo_str,	bhndb_child_pnpinfo_str),
	DEVMETHOD(bus_child_location_str,	bhndb_child_location_str),
	DEVMETHOD(bus_add_child,		bhndb_add_child),
	DEVMETHOD(bus_child_deleted,		bhndb_child_deleted),

	DEVMETHOD(bus_alloc_resource,		bhndb_alloc_resource),
	DEVMETHOD(bus_release_resource,		bhndb_release_resource),
	DEVMETHOD(bus_activate_resource,	bhndb_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bhndb_deactivate_resource),

	DEVMETHOD(bus_setup_intr,		bhndb_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bhndb_teardown_intr),
	DEVMETHOD(bus_config_intr,		bhndb_config_intr),
	DEVMETHOD(bus_bind_intr,		bhndb_bind_intr),
	DEVMETHOD(bus_describe_intr,		bhndb_describe_intr),

	DEVMETHOD(bus_get_dma_tag,		bhndb_get_dma_tag),

	DEVMETHOD(bus_adjust_resource,		bhndb_adjust_resource),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_get_resource_list,	bhndb_get_resource_list),

	DEVMETHOD(bus_read_ivar,		bhndb_gen_read_ivar),
	DEVMETHOD(bus_write_ivar,		bhndb_gen_write_ivar),

	/* BHND interface */
	DEVMETHOD(bhnd_alloc_resource,		bhndb_alloc_bhnd_resource),
	DEVMETHOD(bhnd_release_resource,	bhndb_release_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_activate_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_deactivate_bhnd_resource),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhndb, bhndb_driver, bhndb_methods, sizeof(struct bhndb_softc));

MODULE_VERSION(bhndb, 1);
MODULE_DEPEND(bhndb, bhnd, 1, 1, 1);
