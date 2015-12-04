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

#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhndreg.h>

#include <dev/bhnd/cores/bhnd_chipcreg.h>

#include "bhndbvar.h"
#include "bhndb_bus_if.h"
#include "bhndb_private.h"

static bool				 bhndb_hw_matches(
					     struct bhnd_core_info *cores,
					     u_int num_cores,
					     const struct bhndb_hw *hw);

static int				 bhndb_find_hwspec(
					     struct bhndb_softc *sc,
					     const struct bhndb_hw **hw);

static struct resource			*bhndb_find_parent_resource(
					     struct bhndb_softc *sc,
					     const struct bhndb_regwin *win);

static int				 bhndb_init_dw_region_allocator(
					     struct bhndb_softc *sc);

static struct rman			*bhndb_get_rman(struct bhndb_softc *sc,
					     int type);

static struct bhndb_regwin_region 	*bhndb_find_resource_region(
					     struct bhndb_softc *sc,
					     device_t child, int type, int rid,
					     struct resource *r);

static int				 bhndb_init_child_resource(
					     struct resource *r,
					     struct resource *parent,
					     bhnd_size_t offset,
					     bhnd_size_t size);

static int				 bhndb_try_activate_static_window(
					     struct bhndb_softc *sc, 
					     device_t child, int type, int rid,
					     struct resource *r);

/** 
 * Default bhndb implementation of device_probe().
 */
int
bhndb_generic_probe(device_t dev)
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
	int			 retval = 0;

	sc = device_get_softc(dev);

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
	struct bhndb_softc *sc;

	sc = device_get_softc(dev);

	snprintf(buf, buflen, "base=0x%llx",
	    (unsigned long long) sc->chipid.enum_addr);
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

/*
 * regwin-based bus i/o implementation for bhndb_read_core_table()
 */
struct bhnd_regwin_bus_ctx {
	struct bhndb_softc		*sc;
	struct resource			*r;
	const struct bhndb_regwin	*regwin;
	bhnd_addr_t			 addr;
};

static uint32_t
bhndb_regwin_bus_read4(void *handle, bhnd_addr_t addr)
{
	struct bhnd_regwin_bus_ctx	*ctx;
	struct bhndb_softc		*sc;
	const struct bhndb_regwin	*rw;
	bus_addr_t			 base, offset;
	
	ctx = (struct bhnd_regwin_bus_ctx *) handle;
	sc = ctx->sc;
	rw = ctx->regwin;

	/* Adjust window on-demand */
	if (addr < ctx->addr || addr > (ctx->addr + rw->win_size)) {
		/* maintain window alignment */
		base = addr - (addr % rw->win_size);
		
		/* The 4 byte request may be impossible to fill within
		 * an aligned window. */
		if (addr > (base + rw->win_size - sizeof(uint32_t)))
			return (UINT32_MAX);

		if (BHNDB_SET_WINDOW_ADDR(sc->dev, ctx->regwin, base))
			return (UINT32_MAX);

		ctx->addr = base;
	}
	
	

	/* Perform the read, relative to the resource's window mapping. */
	offset = rw->win_offset + (addr - ctx->addr);
	return (bus_read_4(ctx->r, offset));
}

/**
 * Look for a bhnd(4)-compatible bus driver capable of reading our
 * bridged device's core table, and then have it do so.
 * 
 * The memory allocated for the table should be freed using
 * `free(*cores, M_BHND)`.
 * 
 * @param sc The bhndb device state
 * @param[out] cores The bridged device's core table.
 * @param[out] count The number of cores in @p cores.
 */
static int
bhndb_read_core_table(struct bhndb_softc *sc, struct bhnd_core_info **cores,
    u_int *count)
{
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*regwin;
	struct bhnd_bus_ctx		bus_ctx;
	struct resource			*r;
	driver_t			**drvs;
	int				 num_drvs;
	int				 error, rid, rtype;

	drvs = NULL;
	r = NULL;

	/* Find a dynamic register window capable of 4 byte reads. */
	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);
	regwin = bhndb_regwin_find_type(cfg->register_windows,
	    BHNDB_REGWIN_T_DYN, sizeof(uint32_t));

	if (regwin == NULL) {
		device_printf(sc->dev, "no usable dynamic register window\n");
		return (0);
	}

	/* Allocate the register window resource */
	rid = regwin->res.rid;
	rtype = regwin->res.type;
	r = bus_alloc_resource_any(sc->parent_dev, rtype, &rid, RF_ACTIVE);
	if (r == NULL) {
		device_printf(sc->dev,
		    "failed to allocate bhndb core table resource\n");
		return (ENXIO);
	}

	/* Initialize the register window */
	error = BHNDB_SET_WINDOW_ADDR(sc->dev, regwin, sc->chipid.enum_addr);
	if (error)
		goto cleanup;

	/* Configure our bus I/O context */
	bus_ctx = (struct bhnd_bus_ctx) {
		.dev = sc->dev,
		.ops = &(struct bhnd_bus_ops) {
			.read4 = bhndb_regwin_bus_read4,
			.write4 = NULL,
		},
		.context = &(struct bhnd_regwin_bus_ctx) {
			.sc = sc,
			.r = r,
			.regwin = regwin,
			.addr = sc->chipid.enum_addr
		}
	};

	/* Look for the first driver that'll handle our core table */
	if ((error = devclass_get_drivers(bhndb_devclass, &drvs, &num_drvs)))
		goto cleanup;

	for (int i = 0; i < num_drvs; i++) {
		error = BHND_READ_CORE_TABLE(drvs[i], &sc->chipid, &bus_ctx,
		    cores, count);
		if (!error)
			break;
	}

cleanup:
	/* Clean up */
	if (r != NULL)
		bus_release_resource(sc->parent_dev, rtype, rid, r);

	if (drvs != NULL)
		free(drvs, M_TEMP);

	return (error);
}



/**
 * Find a hardware specification for @p dev.
 * 
 * @param sc The bhndb device state
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
	if ((error = bhndb_read_core_table(sc, &cores, &num_cores)))
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

/**
 * Find the resource containing @p win.
 * 
 * @param sc The bhndb device state
 * @param win A register window.
 * 
 * @retval resource the resource containing @p win.
 * @retval NULL if no resource containing @p win can be found.
 */
static struct resource *
bhndb_find_parent_resource(struct bhndb_softc *sc,
    const struct bhndb_regwin *win)
{
	const struct resource_spec *rspecs;

	rspecs = sc->hw->cfg->resource_specs;
	for (u_int i = 0; rspecs[i].type != -1; i++) {			
		if (win->res.type != rspecs[i].type)
			continue;

		if (win->res.rid != rspecs[i].rid)
			continue;

		/* Found declared resource */
		return (sc->res[i]);
	}

	device_printf(sc->dev,
	    "missing regwin resource spec (type=%d, rid=%d)\n",
	    win->res.type, win->res.rid);
	return (NULL);
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
		region->parent_res = NULL;
		region->child_res = NULL;
		region->rnid = rnid;

		/* Find and validate corresponding resource. */
		region->parent_res = bhndb_find_parent_resource(sc, win);
		if (region->parent_res == NULL) {
			error = EINVAL;
			goto failed;
		}

		if (rman_get_size(region->parent_res) < win->win_offset +
		    win->win_size)
		{
			device_printf(sc->dev, "resource %d too small for "
			    "register window with offset %llx and size %llx\n",
			    rman_get_rid(region->parent_res),
			    (unsigned long long) win->win_offset,
			    (unsigned long long) win->win_size);

			error = EINVAL;
			goto failed;
		}

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
 * Read the ChipCommon identification data for this device.
 * 
 * @param sc bhndb device state.
 * @param[out] result the chip identification data.
 * 
 * @retval 0 success
 * @retval non-zero if the ChipCommon identification data could not be read.
 */
static int
bhndb_read_chipid(struct bhndb_softc *sc, struct bhnd_chipid *result)
{
	const struct bhnd_chipid	*parent_cid;
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*cc_win;
	struct resource_spec		 rs;
	int				 error;

	/* Let our parent device override the discovery process */
	parent_cid = BHNDB_BUS_GET_CHIPID(sc->parent_dev, sc->dev);
	if (parent_cid != NULL) {
		*result = *parent_cid;
		return (0);
	}

	/* Try to find a register window we can use to map the first
	 * CHIPC_CHIPID_SIZE of ChipCommon registers. */
	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);
	cc_win = bhndb_regwin_find_core_or_dyn(cfg->register_windows,
	    BHND_DEVCLASS_CC, 0, 0, 0, CHIPC_CHIPID_SIZE);
	if (cc_win == NULL) {
		device_printf(sc->dev, "no chipcommon register window\n");
		return (0);
	}

	/* We can assume a device without a static ChipCommon window uses the
	 * default ChipCommon address. */
	if (cc_win->win_type == BHNDB_REGWIN_T_DYN) {
		error = BHNDB_SET_WINDOW_ADDR(sc->dev, cc_win,
		    BHND_CHIPC_DEFAULT_ADDR);
		if (error) {
			device_printf(sc->dev,
			     "failed to set chipcommon register window\n");
			return (error);
		}
	}

	/* Let the default bhnd implemenation alloc/release the resource and
	 * perform the read */
	rs.type = cc_win->res.type;
	rs.rid = cc_win->res.rid;
	rs.flags = RF_ACTIVE;

	return (bhnd_read_chipid(sc->parent_dev, &rs, cc_win->win_offset,
	    result));
}

/**
 * Default bhndb implementation of device_attach().
 */
int
bhndb_generic_attach(device_t dev)
{
	struct bhndb_softc		*sc;
	int				 error;
	bool				 free_mem_rman = false;
	bool				 free_parent_res = false;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->parent_dev = device_get_parent(dev);

	BHNDB_LOCK_INIT(sc);

	/* Fetch our chip identification data */
	if ((error = bhndb_read_chipid(sc, &sc->chipid)))
		return (error);

	/* Find our register window configuration */
	if ((error = bhndb_find_hwspec(sc, &sc->hw))) {
		device_printf(dev, "no usable hardware spec found\n");
		return (error);
	}

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

	/* Attach our bridged bus device */
	sc->bus_dev = device_add_child(dev, devclass_get_name(bhnd_devclass),
	    -1);
	if (sc->bus_dev == NULL) {
		error = ENXIO;
		goto failed;
	}

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
bhndb_generic_detach(device_t dev)
{
	struct bhndb_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	
	/* Detach children */
	if ((error = bus_generic_detach(dev)))
		return (error);

	/* Clean up */
	rman_fini(&sc->mem_rman);

	bus_release_resources(dev, sc->res_spec, sc->res);
	free(sc->res, M_BHND);
	free(sc->res_spec, M_BHND);

	free(sc->dw_regions, M_BHND);
	BHNDB_LOCK_DESTROY(sc);

	return (0);
}

/** Default bhndb implementation of device_suspend(). */
int
bhndb_generic_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

/** Default bhndb implementation of device_resume(). */
int
bhndb_generic_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

/** Default bhndb implementation of bus_read_ivar(). */
int
bhndb_generic_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	return (ENOENT);
}

/** Default bhndb implementation of bus_write_ivar(). */
int
bhndb_generic_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
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

static struct bhnd_chipid
bhndb_get_chipid(device_t dev, device_t child)
{
	struct bhndb_softc *sc = device_get_softc(dev);
	return (sc->chipid);
}

static device_t
bhndb_get_attached_bus(device_t dev)
{
	struct bhndb_softc *sc = device_get_softc(dev);
	return (sc->bus_dev);
}

static bool
bhndb_is_hw_populated(device_t dev, device_t child) {
	struct bhnd_core_info core;

	/* We can only handle bhnd-attached devices */
	if (device_get_parent(child) != BHNDB_GET_ATTACHED_BUS(dev))
		return (bhnd_generic_is_hw_populated(dev, child));

	/* Ask our bhndb_bus parent */
	core = bhnd_get_core_info(child);
	if (!BHNDB_BUS_IS_CORE_POPULATED(device_get_parent(dev), dev, &core))
		return (false);

	/* Treat bridge-capable cores as unpopulated if they're not the
	 * configured host bridge. This will need to be revisited if there's
	 * ever a Broadcom chip that operates PCI/PCIe/etc cores in host
	 * mode (ie presenting a functional PCI-BHND-PCI bridge). */
	if (BHND_DEVCLASS_SUPPORTS_HOSTB(bhnd_core_class(&core)))
		return (bhnd_is_hostb_device(child));

	/* Otherwise, assume the core is populated */
	return (true);
}

/* bhnd core index sorting used by bhndb_is_hostb_device() */ 
static int
compare_core_index(const void *lhs, const void *rhs)
{
	u_int left = bhnd_get_core_index(*(const device_t *) lhs);
	u_int right = bhnd_get_core_index(*(const device_t *) rhs);

	if (left < right)
		return (-1);
	else if (left > right)
		return (1);
	else
		return (0);
}

/**
 * Default implementation of bhnd_is_hostb_device that applies a common
 * heuristic for detecting bhndb bridge cores.
 * 
 * Returns true if @p child:
 * 
 * - is a Broadcom core matching the devclass returned by
 *   BHNDB_GET_BRIDGE_DEVCLASS()
 * - is the first core on the bus with this device class.
 * 
 * This heuristic should be valid on all known PCI/PCIe/PCMCIA-bridged
 * devices.
 */
static bool
bhndb_is_hostb_device(device_t dev, device_t child) {
	struct bhndb_softc	*sc;
	struct bhnd_core_match	 md;
	bhnd_devclass_t		 bridge_cls;
	device_t		 bhnd_bus;
	device_t		 hostb_dev;
	device_t		*devlist;
	int			 devcnt, error;

	sc = device_get_softc(dev);
	bhnd_bus = BHNDB_GET_ATTACHED_BUS(dev);
	
	/* Requestor must be attached to the bhnd bus */
	if (device_get_parent(child) != bhnd_bus)
		return (false);

	/* Determine required device class */
	bridge_cls = BHNDB_GET_BRIDGE_DEVCLASS(dev);

	/* Pre-screen the device before searching over the full device list. */
	md = (struct bhnd_core_match) {
		.vendor = BHND_MFGID_BCM,
		.device = BHND_COREID_INVALID,
		.hwrev = { BHND_HWREV_INVALID, BHND_HWREV_INVALID },
		.class = bridge_cls,
		.unit = 0
	};

	if (!bhnd_device_matches(child, &md))
		return (false);

	/*
	 * Confirm that this is the absolute first matching device on the bus.
	 */
	if ((error = device_get_children(bhnd_bus, &devlist, &devcnt)))
		return (false);

	/* Sort by core index value, ascending */
	qsort(devlist, devcnt, sizeof(*devlist), compare_core_index);

	/* Find the actual hostb device */
	hostb_dev = NULL;
	for (int i = 0; i < devcnt; i++) {
		if (bhnd_device_matches(devlist[i], &md)) {
			hostb_dev = devlist[i];
			break;
		}
	}

	/* Clean up */
	free(devlist, M_TEMP);

	return (child == hostb_dev);
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
	defaults = (start == 0UL && end == ~0UL);
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
		count = ulmax(count, rle->count);
	}

	/* Validate resource addresses */
	if (start > end || end < start || count > ((end - start) + 1))
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
 * 
 * @param sc bhndb device state.
 * @param child The child device making a resource request on @p r.
 * @param type The resource type.
 * @param rid The resource ID.
 * @param r The resource to search for.
 * 
 * @retval bhndb_regwin_region The region allocated for @p r.
 * @retval NULL if no region is allocated for @p r.
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
		
		/* If active, the base address is fixed, and the end address
		 * cannot grow past the end of the allocated window.
		 * 
		 * In theory we could support adjusting the base address, but
		 * that would require that we keep track of the window's
		 * absolute start and end addresses.
		 */
		if (rman_get_start(r) != start  || end < start ||
		    (end - start) + 1 > region->win->win_size)
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

/**
 * Initialize child resource @p r with a virtual address, tag, and handle
 * copied from @p parent, adjusted to contain only the range defined by @p win.
 * 
 * @param r The register to be initialized.
 * @param parent The parent bus resource that fully contains the subregion.
 * @param offset The subregion offset within @p parent.
 * @param size The subregion size.
 * @p r.
 */
static int
bhndb_init_child_resource(struct resource *r,
    struct resource *parent, bhnd_size_t offset, bhnd_size_t size)
{

	bus_space_handle_t	bh, child_bh;
	bus_space_tag_t		bt;
	uintptr_t		vaddr;
	int			error;

	/* Fetch the parent resource's real bus values */
	vaddr = (uintptr_t) rman_get_virtual(parent);
	bt = rman_get_bustag(parent);
	bh = rman_get_bushandle(parent);

	/* Configure child resource with window-adjusted real bus values */
	vaddr += offset;
	error = bus_space_subregion(bt, bh, offset, size, &child_bh);
	if (error)
		return (error);

	rman_set_virtual(r, (void *) vaddr);
	rman_set_bustag(r, bt);
	rman_set_bushandle(r, child_bh);

	return (0);
}

/**
 * Attempt activation of a fixed register window mapping for @p child.
 * 
 * @param sc BHNDB device state.
 * @param child A child requesting resource activation.
 * @param type Resource type.
 * @param rid Resource identifier.
 * @param r Resource to be activated.
 * 
 * @retval 0 if @p r was activated successfully
 * @retval ENOENT if no fixed register window was found.
 * @retval non-zero if @p r could not be activated.
 */
static int
bhndb_try_activate_static_window(struct bhndb_softc *sc, device_t child,
    int type, int rid, struct resource *r)
{
	struct resource			*parent_res;
	const struct bhndb_regwin	*win;
	bhnd_addr_t			 addr;
	bhnd_size_t			 parent_offset, size;
	u_long				 r_start, r_end;
	int				 error;

	r_start = rman_get_start(r);
	r_end = rman_get_end(r);

	/* Look for a core window enclosing the resource range. */
	for (win = sc->hw->cfg->register_windows;
	    win->win_type != BHNDB_REGWIN_T_INVALID; win++)
	{
		device_t bhnd_child;

		if (win->win_type != BHNDB_REGWIN_T_CORE)
			continue;

		/* Find the core's device on the bhnd bus */
		bhnd_child = bhnd_find_child(sc->bus_dev, win->core.class,
		    win->core.unit);
		if (bhnd_child == NULL)
			continue;

		/* Fetch the address and size of the core's mapped port. */
		error = bhnd_get_port_addr(bhnd_child, win->core.port,
		    win->core.region, &addr, &size);
		if (error)
			return (error);

		/* The size cannot be larger than the register window */
		size = ulmin(win->win_size, size);
	
		/* Resource must fit within the mapped range */
		if (r_start < addr || r_start > addr + size)
			continue;

		if ((r_end + 1) > addr + size)
			continue;

		/* Calculate subregion offset within the parent resource */
		parent_offset = r_start - addr;
		parent_offset += win->win_offset;

		/* Find the corresponding bridge resource */
		parent_res = bhndb_find_parent_resource(sc, win);
		if (parent_res == NULL)
			return (ENXIO);

		/* Configure resource with its real bus values. */
		error = bhndb_init_child_resource(r, parent_res, parent_offset,
		    size);
		if (error)
			return (error);

		/* Mark active */
		if ((error = rman_activate_resource(r)))
			return (error);
		
		return (0);
	}

	return (ENOENT);
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

	// TODO
	if (type != SYS_RES_MEMORY)
		return (ENXIO);

	/* Try static mappings before consuming a dynamic register window. */
	error = bhndb_try_activate_static_window(sc, child, type, rid, r);
	if (!error)
		return (0);

	/*
	 * Find the first free dynamic window of sufficient size.
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

	/* Configure resource with its real bus values. */
	error = bhndb_init_child_resource(r, region->parent_res,
	    region->win->win_offset, region->win->win_size);
	if (error)
		goto failed;

	/* Mark active */
	if ((error = rman_activate_resource(r)))
		goto failed;


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
	struct bhndb_softc	*sc;
	struct bhnd_resource	*br;

	sc = device_get_softc(dev);

	/* Allocate resource wrapper */
	br = malloc(sizeof(struct bhnd_resource), M_BHND, M_WAITOK|M_ZERO);
	if (br == NULL)
		return (NULL);

	/* Configure */
	// TODO - `_direct` must be set at activation time, not allocation.
	br->direct = true;
	br->res = bus_alloc_resource(child, type, rid, start, end, count,
	    flags & ~RF_ACTIVE);
	if (br->res == NULL)
		goto failed;
	

	if (flags & RF_ACTIVE) {
		if (bhnd_activate_resource(child, type, *rid, br))
			goto failed;
	}

	return (br);

failed:
	if (br->res != NULL)
		bus_release_resource(child, type, *rid, br->res);

	free(br, M_BHND);
	return (NULL);
}

static int
bhndb_release_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	int error;

	if ((error = bus_release_resource(child, type, rid, r->res)))
		return (error);

	free(r, M_BHND);
	return (0);
}

static int
bhndb_activate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	/* Indirect resources don't require activation */
	if (!r->direct)
		return (0);

	return (bus_activate_resource(child, type, rid, r->res));
};

static int
bhndb_deactivate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	/* Indirect resources don't require activation */
	if (!r->direct)
		return (0);

	return (bus_deactivate_resource(child, type, rid, r->res));
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
	DEVMETHOD(device_probe,			bhndb_generic_probe),
	DEVMETHOD(device_attach,		bhndb_generic_attach),
	DEVMETHOD(device_detach,		bhndb_generic_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bhndb_generic_suspend),
	DEVMETHOD(device_resume,		bhndb_generic_resume),

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

	DEVMETHOD(bus_read_ivar,		bhndb_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,		bhndb_generic_write_ivar),

	/* BHNDB interface */
	DEVMETHOD(bhndb_get_chipid,		bhndb_get_chipid),
	DEVMETHOD(bhndb_get_attached_bus,	bhndb_get_attached_bus),

	/* BHND interface */
	DEVMETHOD(bhnd_is_hw_populated,		bhndb_is_hw_populated),
	DEVMETHOD(bhnd_is_hostb_device,		bhndb_is_hostb_device),
	DEVMETHOD(bhnd_alloc_resource,		bhndb_alloc_bhnd_resource),
	DEVMETHOD(bhnd_release_resource,	bhndb_release_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_activate_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_deactivate_bhnd_resource),

	DEVMETHOD_END
};

devclass_t bhndb_devclass;

DEFINE_CLASS_0(bhndb, bhndb_driver, bhndb_methods, sizeof(struct bhndb_softc));

MODULE_VERSION(bhndb, 1);
MODULE_DEPEND(bhndb, bhnd, 1, 1, 1);
