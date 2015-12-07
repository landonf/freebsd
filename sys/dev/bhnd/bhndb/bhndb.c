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

static bool				 bhndb_hw_matches(device_t *devlist,
					     int num_devs,
					     const struct bhndb_hw *hw);

static int				 bhndb_find_hwspec(
					     struct bhndb_softc *sc,
					     const struct bhndb_hw **hw);

static struct resource			*bhndb_find_parent_resource(
					     struct bhndb_softc *sc,
					     const struct bhndb_regwin *win);

static int				 bhndb_init_dw_region_allocator(
					     struct bhndb_softc *sc);

static int				 bhndb_read_chipid(
					     struct bhndb_softc *sc,
					     struct bhnd_chipid *result);

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
 * Return true if @p devlist matches the @p hw specification.
 * 
 * @param devlist A device table to match against.
 * @param num_devs The number of devices in @p devlist.
 * @param hw The hardware description to be matched against.
 */
static bool
bhndb_hw_matches(device_t *devlist, int num_devs, const struct bhndb_hw *hw)
{
	for (u_int i = 0; i < hw->num_hw_reqs; i++) {
		const struct bhnd_core_match	*match;
		bool				 found;

		match =  &hw->hw_reqs[i];
		found = false;

		for (int d = 0; d < num_devs; d++) {
			if (!bhnd_device_matches(devlist[d], match))
				continue;

			found = true;
			break;
		}

		if (!found)
			return (false);
	}

	return (true);
}

/**
 * Find a hardware specification for @p dev.
 * 
 * @param sc The bhndb device state
 * @param cores The core table to match against.
 * @param num_cores The length of @p num_cores
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
	device_t		*devlist;
	int			 error, num_devs;

	/* Fetch the full set of attached devices */
	if ((error = device_get_children(sc->bus_dev, &devlist, &num_devs)))
		return (error);

	/* Search for the first matching hardware config. */
	error = ENOENT;

	hw_table = BHNDB_BUS_GET_HARDWARE_TABLE(sc->parent_dev, sc->dev);
	for (next = hw_table; next->hw_reqs != NULL; next++) {
		if (!bhndb_hw_matches(devlist, num_devs, next))
			continue;

		/* Found */
		*hw = next;
		error = 0;
		goto cleanup;
	}

cleanup:
	free(devlist, M_TEMP);
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

	rspecs = sc->cfg->resource_specs;
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

	cfg = sc->cfg;
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
	    sc->dw_count, M_BHND, M_NOWAIT);
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

	sc->dw_count = 0;
	sc->dw_freelist = 0;
	sc->dw_regions = NULL;

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
	const struct bhndb_regwin	*cc_win;
	struct resource_spec		 rs;
	int				 error;

	/* Let our parent device override the discovery process */
	parent_cid = BHNDB_BUS_GET_CHIPID(sc->parent_dev, sc->dev);
	if (parent_cid != NULL) {
		*result = *parent_cid;
		return (0);
	}

	/* Find a register window we can use to map the first CHIPC_CHIPID_SIZE
	 * of ChipCommon registers. */
	cc_win = bhndb_regwin_find_best(sc->cfg->register_windows,
	    BHND_DEVCLASS_CC, 0, BHND_PORT_DEVICE, 0, 0, CHIPC_CHIPID_SIZE);
	if (cc_win == NULL) {
		device_printf(sc->dev, "no chipcommon register window\n");
		return (0);
	}

	/* We can assume a device without a static ChipCommon window uses the
	 * default ChipCommon address. */
	if (cc_win->win_type == BHNDB_REGWIN_T_DYN) {
		error = BHNDB_SET_WINDOW_ADDR(sc->dev, cc_win,
		    BHND_DEFAULT_CHIPC_ADDR);

		if (error) {
			device_printf(sc->dev, "failed to set chipcommon "
			    "register window\n");
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
 * Free any bridged resources and reset all associated state.
 * 
 * @param sc Bridge driver state.
 * 
 * @retval 0 success
 * @retval non-zero failure.
 */
static int
bhndb_reset_bridged_resources(struct bhndb_softc *sc)
{
	/* No window regions may still be held */
	if (__builtin_popcount(sc->dw_freelist) != sc->dw_count) {
		panic("bridge resource reset with %llu leaked regions\n",
		    (unsigned long long) sc->dw_count - sc->dw_freelist);
	}

	/* Release resources allocated through our parent. */
	bus_release_resources(sc->parent_dev, sc->res_spec, sc->res);

	/* Free (and reset) resource state */
	free(sc->res, M_BHND);
	sc->res_count = 0;
	sc->res = NULL;

	free(sc->res_spec, M_BHND);
	sc->res_spec = NULL;

	free(sc->dw_regions, M_BHND);
	sc->dw_regions = NULL;
	sc->dw_count = 0;
	sc->dw_freelist = 0;

	return (0);
}

/**
 * Allocate resources from our parent and initialize our bridge resource
 * allocation state.
 */
static int
bhndb_init_bridge_resources(struct bhndb_softc *sc)
{
	int	error;
	bool	free_parent_res;

	KASSERT(sc->res_count == 0 && 
		sc->res_spec == NULL && 
		sc->res == NULL &&
		sc->dw_regions == NULL &&
		sc->dw_freelist == 0 &&
		sc->dw_count == 0, ("bhndb already initialized\n"));
	
	free_parent_res = false;

	/* Determine our bridge resource count from the hardware config. */
	sc->res_count = 0;
	for (size_t i = 0; sc->cfg->resource_specs[i].type != -1; i++)
		sc->res_count++;

	/* Allocate space for a non-const copy of our resource_spec
	 * table; this will be updated with the RIDs assigned by
	 * bus_alloc_resources. */
	sc->res_spec = malloc(sizeof(struct resource_spec) * (sc->res_count + 1),
	    M_BHND, M_NOWAIT);
	if (sc->res_spec == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Initialize and terminate the table */
	for (size_t i = 0; i < sc->res_count; i++)
		sc->res_spec[i] = sc->cfg->resource_specs[i];
	
	sc->res_spec[sc->res_count].type = -1;

	/* Allocate space for our resource references */
	sc->res = malloc(sizeof(struct resource) * sc->res_count,
	    M_BHND, M_NOWAIT);
	if (sc->res == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Allocate resources */
	error = bus_alloc_resources(sc->parent_dev, sc->res_spec, sc->res);
	if (error) {
		device_printf(sc->dev,
		    "could not allocate bridge resources on %s: %d\n",
		    device_get_nameunit(sc->parent_dev), error);
		goto failed;
	} else {
		free_parent_res = true;
	}

	/* Initialize dynamic window allocator state */
	if ((error = bhndb_init_dw_region_allocator(sc)))
		goto failed;

	return (0);

failed:
	if (free_parent_res)
		bus_release_resources(sc->parent_dev, sc->res_spec, sc->res);

	if (sc->res != NULL) {
		free(sc->res, M_BHND);
		sc->res = NULL;
	}

	if (sc->res_spec != NULL) {
		free(sc->res_spec, M_BHND);
		sc->res_spec = NULL;
	}

	return (error);
}

/**
 * Default bhndb implementation of device_attach().
 * 
 * @param dev Bridge device.
 * @param bridge_devclass The device class of the bridging core. This is used
 * to automatically detect the bridge core, and to disable additional bridge
 * cores (e.g. PCMCIA on a PCIe device).
 */
int
bhndb_attach(device_t dev, bhnd_devclass_t bridge_devclass)
{
	struct bhndb_softc		*sc;
	int				 error;
	bool				 free_mem_rman = false;
	bool				 free_bridge_res = false;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->parent_dev = device_get_parent(dev);
	sc->bridge_class = bridge_devclass;
	sc->cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);

	BHNDB_LOCK_INIT(sc);
	
	/* Read our chip identification data */
	if ((error = bhndb_read_chipid(sc, &sc->chipid)))
		return (error);

	/* Initialize resource allocation state. */
	if ((error = bhndb_init_bridge_resources(sc))) {
		goto failed;
	} else {
		free_bridge_res = true;
	}

	/* Set up a resource manager for the device's address space. */
	sc->mem_rman.rm_start = 0;
	sc->mem_rman.rm_end = BUS_SPACE_MAXADDR_32BIT;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "bhnd device address space";
	
	if ((error = rman_init(&sc->mem_rman))) {
		device_printf(dev, "could not initialize mem_rman\n");
		goto failed;
	} else {
		free_mem_rman = true;
	}

	error = rman_manage_region(&sc->mem_rman, 0, BUS_SPACE_MAXADDR_32BIT);
	if (error) {
		device_printf(dev, "could not configure mem_rman\n");
		goto failed;
	}

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

	if (free_bridge_res)
		bhndb_reset_bridged_resources(sc);

	BHNDB_LOCK_DESTROY(sc);

	return (error);
}

static int
bhndb_init_full_config(device_t dev, device_t child)
{
	struct bhndb_softc		*sc;
	const struct bhndb_hw		*hw;
	int				 error;

	sc = device_get_softc(dev);
	error = 0;

	BHNDB_LOCK(sc);

	/* Reset existing resource state */
	if ((error = bhndb_reset_bridged_resources(sc)))
		goto cleanup;

	/* Find our full register window configuration */
	if ((error = bhndb_find_hwspec(sc, &hw))) {
		device_printf(sc->dev, "no usable hardware spec found\n");
		goto cleanup;
	}

	if (bootverbose)
		device_printf(sc->dev, "%s register map\n", hw->name);

	/* Reinitialize our resource state */
	sc->cfg = hw->cfg;
	if ((error = bhndb_init_bridge_resources(sc)))
		goto cleanup;

cleanup:
	BHNDB_UNLOCK(sc);
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

	/* Clean up our driver state. */
	rman_fini(&sc->mem_rman);
	bhndb_reset_bridged_resources(sc);
	
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

	dinfo = malloc(sizeof(struct bhndb_devinfo), M_BHND, M_NOWAIT);
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

static bool
bhndb_is_hw_disabled(device_t dev, device_t child) {
	struct bhndb_softc	*sc;
	struct bhnd_core_info	 core;

	sc = device_get_softc(dev);

	/* Requestor must be attached to the bhnd bus */
	if (device_get_parent(child) != sc->bus_dev) {
		return (BHND_IS_HW_DISABLED(device_get_parent(dev), child));
	}

	/* Fetch core info */
	core = bhnd_get_core_info(child);

	/* Try to defer to the bhndb bus parent */
	if (BHNDB_BUS_IS_CORE_DISABLED(sc->parent_dev, dev, &core))
		return (true);

	/* Otherwise, we treat bridge-capable cores as unpopulated if they're
	 * not the configured host bridge */
	if (BHND_DEVCLASS_SUPPORTS_HOSTB(bhnd_core_class(&core)))
		return (!BHND_IS_HOSTB_DEVICE(dev, child));

	/* Otherwise, assume the core is populated */
	return (false);
}

/* ascending core index comparison used by bhndb_is_hostb_device() */ 
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
 * BHND_IS_HOSTB_DEVICE implementation that uses a heuristic valid on all known
 * PCI/PCIe/PCMCIA-bridged bhnd(4) devices:
 * 
 * - The core must have a Broadcom vendor ID.
 * - The core devclass must match the bridge type.
 * - The core must be the first device on the bus with the bridged device
 *   class.
 * 
 * @param sc The bridge device state.
 * @param cores The table of bridge-enumerated cores.
 * @param num_cores The length of @p cores.
 * @param core The core to check.
 */
static bool
bhndb_is_hostb_device(device_t dev, device_t child) {
	struct bhndb_softc	*sc;
	struct bhnd_core_match	 md;
	device_t		 hostb_dev, *devlist;
	int                      devcnt, error;

	
	sc = device_get_softc(dev);

	/* Requestor must be attached to the bhnd bus */
	if (device_get_parent(child) != sc->bus_dev)
		return (BHND_IS_HOSTB_DEVICE(device_get_parent(dev), child));

	/* Determine required device class and set up a match descriptor. */
	md = (struct bhnd_core_match) {
		.vendor = BHND_MFGID_BCM,
		.device = BHND_COREID_INVALID,
		.hwrev = { BHND_HWREV_INVALID, BHND_HWREV_INVALID },
		.class = sc->bridge_class,
		.unit = 0
	};

	/* Pre-screen the device before searching over the full device list. */
	if (!bhnd_device_matches(child, &md))
		return (false);
	
	/* Must be the absolute first matching device on the bus. */
	if ((error = device_get_children(sc->bus_dev, &devlist, &devcnt)))
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
	bhnd_port_type			 port_type;
	u_int				 port, region;
	u_long				 r_start, r_end;
	int				 error;

	r_start = rman_get_start(r);
	r_end = rman_get_end(r);

	/* Only core register windows are currently supported; this requires a
	 * bus-attached device. */
	if (device_get_parent(child) != sc->bus_dev)
		return (ENOENT);

	/* Try to map the resource back to a port triplet */
	error = bhnd_decode_port_rid(child, type, rid, &port_type, &port,
	    &region);
	if (error)
		return (error);

	/* Look for a core window matching this port triplet. */
	for (win = sc->cfg->register_windows;
	    win->win_type != BHNDB_REGWIN_T_INVALID; win++)
	{
		if (win->win_type != BHNDB_REGWIN_T_CORE)
			continue;

		if (win->core.type != port_type ||
		    win->core.port != port ||
		    win->core.region != region)
		{
			continue;
		}

		/* Fetch the address and size of the core's mapped port. */
		error = bhnd_get_port_addr(child, win->core.type,
		    win->core.port, win->core.region, &addr, &size);
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
	br = malloc(sizeof(struct bhnd_resource), M_BHND, M_NOWAIT|M_ZERO);
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
	DEVMETHOD(bhndb_init_full_config,	bhndb_init_full_config),

	/* BHND interface */
	DEVMETHOD(bhnd_is_hw_disabled,		bhndb_is_hw_disabled),
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
