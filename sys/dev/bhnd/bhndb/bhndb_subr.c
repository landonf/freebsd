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
#include <sys/kernel.h>

#include "bhndb_private.h"
#include "bhndbvar.h"

/**
 * Attach a BHND bridge device to @p parent.
 * 
 * @param parent A parent PCI device.
 * @param[out] bhndb On success, the probed and attached bhndb bridge device.
 * @param unit The device unit number, or -1 to select the next available unit
 * number.
 * 
 * @retval 0 success
 * @retval non-zero Failed to attach the bhndb device.
 */
int
bhndb_attach_bridge(device_t parent, device_t *bhndb, int unit)
{
	int error;

	*bhndb = device_add_child(parent, devclass_get_name(bhndb_devclass),
	    unit);
	if (*bhndb == NULL)
		return (ENXIO);

	if (!(error = device_probe_and_attach(*bhndb)))
		return (0);

	if ((device_delete_child(parent, *bhndb)))
		device_printf(parent, "failed to detach bhndb child\n");

	return (error);
}

/**
 * Find the resource containing @p win.
 * 
 * @param res The bhndb resource state to search.
 * @param win A register window.
 * 
 * @retval resource the resource containing @p win.
 * @retval NULL if no resource containing @p win can be found.
 */
struct resource *
bhndb_find_regwin_resource(struct bhndb_resources *r,
    const struct bhndb_regwin *win)
{
	const struct resource_spec *rspecs;

	rspecs = r->cfg->resource_specs;
	for (u_int i = 0; rspecs[i].type != -1; i++) {			
		if (win->res.type != rspecs[i].type)
			continue;

		if (win->res.rid != rspecs[i].rid)
			continue;

		/* Found declared resource */
		return (r->res[i]);
	}

	device_printf(r->dev,
	    "missing regwin resource spec (type=%d, rid=%d)\n",
	    win->res.type, win->res.rid);

	return (NULL);
}

/**
 * Allocate and initialize a new resource state structure, allocating
 * bus resources from @p parent_dev according to @p cfg.
 * 
 * @param dev The bridge device.
 * @param parent_dev The parent device from which resources will be allocated.
 * @param cfg The hardware configuration to be used.
 */
struct bhndb_resources *
bhndb_alloc_resources(device_t dev, device_t parent_dev,
    const struct bhndb_hwcfg *cfg)
{
	struct bhndb_resources		*r;
	const struct bhndb_regwin	*win;
	bus_size_t			 last_window_size;
	size_t				 res_num;
	u_int				 rnid;
	int				 error;
	bool				 free_parent_res;

	free_parent_res = false;

	r = malloc(sizeof(*r), M_BHND, M_WAITOK|M_ZERO);
	if (r == NULL)
		return (NULL);

	/* Basic initialization */
	r->dev = dev;
	r->parent_dev = parent_dev;
	r->cfg = cfg;
	r->min_prio = BHNDB_PRIORITY_NONE;
	STAILQ_INIT(&r->bus_regions);
	
	/* Determine our bridge resource count from the hardware config. */
	res_num = 0;
	for (size_t i = 0; cfg->resource_specs[i].type != -1; i++)
		res_num++;

	/* Allocate space for a non-const copy of our resource_spec
	 * table; this will be updated with the RIDs assigned by
	 * bus_alloc_resources. */
	r->res_spec = malloc(sizeof(r->res_spec[0]) * (res_num + 1), M_BHND,
	    M_WAITOK);
	if (r->res_spec == NULL)
		goto failed;

	/* Initialize and terminate the table */
	for (size_t i = 0; i < res_num; i++)
		r->res_spec[i] = cfg->resource_specs[i];
	
	r->res_spec[res_num].type = -1;

	/* Allocate space for our resource references */
	r->res = malloc(sizeof(r->res[0]) * res_num, M_BHND, M_WAITOK);
	if (r->res == NULL)
		goto failed;

	/* Allocate resources */
	error = bus_alloc_resources(r->parent_dev, r->res_spec, r->res);
	if (error) {
		device_printf(r->dev,
		    "could not allocate bridge resources on %s: %d\n",
		    device_get_nameunit(r->parent_dev), error);
		goto failed;
	} else {
		free_parent_res = true;
	}

	/* Fetch the dynamic regwin count and verify that it does not exceed
	 * what is representable via our freelist bitmask. */
	r->dw_count = bhndb_regwin_count(cfg->register_windows,
	    BHNDB_REGWIN_T_DYN);
	if (r->dw_count >= (8 * sizeof(r->dw_freelist))) {
		device_printf(r->dev, "max dynamic regwin count exceeded\n");
		goto failed;
	}
	
	/* Allocate the dynamic region table. */
	r->dw_regions = malloc(sizeof(r->dw_regions[0]) * r->dw_count, M_BHND,
	    M_WAITOK);
	if (r->dw_regions == NULL)
		goto failed;

	/* Initialize the dynamic region table and freelist. */
	r->dw_freelist = 0;
	rnid = 0;
	last_window_size = 0;
	for (win = cfg->register_windows;
	    win->win_type != BHNDB_REGWIN_T_INVALID; win++)
	{
		struct bhndb_dw_region *region;

		/* Skip non-DYN windows */
		if (win->win_type != BHNDB_REGWIN_T_DYN)
			continue;

		/* Validate the window size */
		if (win->win_size == 0) {
			device_printf(r->dev, "ignoring zero-length dynamic "
			    "register window\n");
			continue;
		} else if (last_window_size == 0) {
			last_window_size = win->win_size;
		} else if (last_window_size != win->win_size) {
			/* 
			 * No existing hardware should trigger this.
			 * 
			 * If you run into this in the future, the dynamic
			 * window allocator and the resource priority system
			 * will need to be extended to support multiple register
			 * window allocation pools. 
			 */
			device_printf(r->dev, "devices that vend multiple "
			    "dynamic register window sizes are not currently "
			    "supported\n");
			goto failed;
		}

		region = &r->dw_regions[rnid];
		region->win = win;
		region->parent_res = NULL;
		region->child_res = NULL;
		region->rnid = rnid;

		/* Find and validate corresponding resource. */
		region->parent_res = bhndb_find_regwin_resource(r, win);
		if (region->parent_res == NULL)
			goto failed;

		if (rman_get_size(region->parent_res) < win->win_offset +
		    win->win_size)
		{
			device_printf(r->dev, "resource %d too small for "
			    "register window with offset %llx and size %llx\n",
			    rman_get_rid(region->parent_res),
			    (unsigned long long) win->win_offset,
			    (unsigned long long) win->win_size);

			error = EINVAL;
			goto failed;
		}

		/* Add to freelist */
		r->dw_freelist |= (1 << rnid);

		rnid++;
	}

	return (r);

failed:
	if (free_parent_res)
		bus_release_resources(r->parent_dev, r->res_spec, r->res);

	if (r->res != NULL)
		free(r->res, M_BHND);

	if (r->res_spec != NULL)
		free(r->res_spec, M_BHND);

	if (r->dw_regions != NULL)
		free(r->dw_regions, M_BHND);

	free (r, M_BHND);

	return (NULL);
}

/**
 * Deallocate the given bridge resource structure and any associated resources.
 * 
 * @param res Resource state to be deallocated.
 */
void
bhndb_free_resources(struct bhndb_resources *r)
{
	struct bhndb_region *region, *r_next;

	/* No window regions may still be held */
	if (__builtin_popcount(r->dw_freelist) != r->dw_count) {
		device_printf(r->dev, "leaked %llu dynamic register regions\n",
		    (unsigned long long) r->dw_count - r->dw_freelist);
	}

	/* Release resources allocated through our parent. */
	bus_release_resources(r->parent_dev, r->res_spec, r->res);

	/* Free resource state structures */
	free(r->res, M_BHND);
	free(r->res_spec, M_BHND);
	free(r->dw_regions, M_BHND);
	
	STAILQ_FOREACH_SAFE(region, &r->bus_regions, link, r_next) {
		STAILQ_REMOVE(&r->bus_regions, region, bhndb_region, link);
		free(region, M_BHND);
	}
}

/**
 * Add a bus region entry to @p r for the given port defined on @p dev,
 * with @p priority.
 * 
 * @param r The resource state to which the bus region entry will be added.
 * @param dev A bhnd device defining the given port/region.
 * @param port_type The port type of @p port.
 * @param port The port on @p dev.
 * @param region The region mapped to @p port.
 * @param static_regwin The static register window for this bus region entry,
 * or NULL.
 * @param priority The resource priority to be assigned to the bus region
 * entry.
 * 
 * @retval 0 success
 * @retval non-zero if adding the bus region fails.
 */
int
bhndb_resources_add_device_region(struct bhndb_resources *r, device_t dev,
    bhnd_port_type port_type, u_int port, u_int region,
    const struct bhndb_regwin *static_regwin, bhndb_priority_t priority)
{
	struct bhndb_region	*reg;
	bhnd_addr_t		 addr;
	bhnd_size_t		 size;
	int			 error;

	/* Fetch the address and size of the mapped port. */
	error = bhnd_get_region_addr(dev, port_type, port, region, &addr, &size);
	if (error)
		return (error);

	/* Insert in the bus resource list */
	reg = malloc(sizeof(*reg), M_BHND, M_WAITOK);
	if (reg == NULL)
		return (ENOMEM);

	*reg = (struct bhndb_region) {
		.addr = addr,
		.size = size,
		.priority = priority,
		.static_regwin = static_regwin
	};

	STAILQ_INSERT_HEAD(&r->bus_regions, reg, link);

	return (0);
}

/**
 * Find a bus region that maps the address range at @p addr of @p size.
 * 
 * @param r The resource state to search.
 * @param addr The requested starting address.
 * @param size The requested size.
 * 
 * @retval bhndb_region A region that fully contains the requested range.
 * @retval NULL If no mapping region can be found.
 */
struct bhndb_region *
bhndb_resources_find_region(struct bhndb_resources *r, bhnd_addr_t addr,
    bhnd_size_t size)
{
	struct bhndb_region *region;

	STAILQ_FOREACH(region, &r->bus_regions, link) {
		/* Request must fit within the region's mapping  */
		if (addr < region->addr)
			continue;

		if (addr + size > region->addr + region->size)
			continue;

		return (region);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return the count of @p type register windows in @p table.
 * 
 * @param table The table to search.
 * @param type The required window type, or BHNDB_REGWIN_T_INVALID to
 * count all register window types.
 */
size_t
bhndb_regwin_count(const struct bhndb_regwin *table,
    bhndb_regwin_type_t type)
{
	const struct bhndb_regwin	*rw;
	size_t				 count;

	count = 0;
	for (rw = table; rw->win_type != BHNDB_REGWIN_T_INVALID; rw++) {
		if (type == BHNDB_REGWIN_T_INVALID || rw->win_type == type)
			count++;
	}

	return (count);
}

/**
 * Search @p table for the first window with the given @p type.
 * 
 * @param table The table to search.
 * @param type The required window type.
 * @param min_size The minimum window size.
 * 
 * @retval bhndb_regwin The first matching window.
 * @retval NULL If no window of the requested type could be found. 
 */
const struct bhndb_regwin *
bhndb_regwin_find_type(const struct bhndb_regwin *table,
    bhndb_regwin_type_t type, bus_size_t min_size)
{
	const struct bhndb_regwin *rw;

	for (rw = table; rw->win_type != BHNDB_REGWIN_T_INVALID; rw++)
	{
		if (rw->win_type == type && rw->win_size >= min_size)
			return (rw);
	}

	return (NULL);
}

/**
 * Search @p windows for the first matching core window.
 * 
 * @param table The table to search.
 * @param class The required core class.
 * @param unit The required core unit, or -1.
 * @param port_type The required port type.
 * @param port The required port.
 * @param region The required region.
 *
 * @retval bhndb_regwin The first matching window.
 * @retval NULL If no matching window was found. 
 */
const struct bhndb_regwin *
bhndb_regwin_find_core(const struct bhndb_regwin *table, bhnd_devclass_t class,
    int unit, bhnd_port_type port_type, u_int port, u_int region)
{
	const struct bhndb_regwin *rw;
	
	for (rw = table; rw->win_type != BHNDB_REGWIN_T_INVALID; rw++)
	{
		if (rw->win_type != BHNDB_REGWIN_T_CORE)
			continue;

		if (rw->core.class != class)
			continue;
		
		if (unit != -1 && rw->core.unit != unit)
			continue;

		if (rw->core.port_type != port_type)
			continue;

		if (rw->core.port != port)
			continue;
		
		if (rw->core.region != region)
			continue;

		return (rw);
	}

	return (NULL);
}

/**
 * Search @p windows for the best available window of at least @p min_size.
 * 
 * Search order:
 * - BHND_REGWIN_T_CORE
 * - BHND_REGWIN_T_DYN
 * 
 * @param table The table to search.
 * @param class The required core class.
 * @param unit The required core unit, or -1.
 * @param port_type The required port type.
 * @param port The required port.
 * @param region The required region.
 * @param min_size The minimum window size.
 *
 * @retval bhndb_regwin The first matching window.
 * @retval NULL If no matching window was found. 
 */
const struct bhndb_regwin *
bhndb_regwin_find_best(const struct bhndb_regwin *table,
    bhnd_devclass_t class, int unit, bhnd_port_type port_type, u_int port,
    u_int region, bus_size_t min_size)
{
	const struct bhndb_regwin *rw;

	/* Prefer a fixed core mapping */
	rw = bhndb_regwin_find_core(table, class, unit, port_type,
	    port, region);
	if (rw != NULL)
		return (rw);

	/* Fall back on a generic dynamic window */
	return (bhndb_regwin_find_type(table, BHNDB_REGWIN_T_DYN, min_size));
}

/**
 * Return true if @p regw defines a static port register window, and
 * the mapped port is actually defined on @p dev.
 * 
 * @param regw A register window to match against.
 * @param dev A bhnd(4) bus device.
 */
bool
bhndb_regwin_matches_device(const struct bhndb_regwin *regw, device_t dev)
{
	/* Only core windows are supported */
	if (regw->win_type != BHNDB_REGWIN_T_CORE)
		return (false);

	/* Device class must match */
	if (bhnd_get_class(dev) != regw->core.class)
		return (false);

	/* Device unit must match */
	if (bhnd_get_core_unit(dev) != regw->core.unit)
		return (false);
	
	/* The regwin port must be defined. */
	if (regw->core.port > bhnd_get_port_count(dev, regw->core.port_type))
		return (false);

	/* The regwin region must be defined. */
	if (regw->core.region > bhnd_get_region_count(dev, regw->core.port_type,
	    regw->core.port))
	{
		return (false);
	}

	/* Matches */
	return (true);
}

/**
 * Search for a core resource priority descriptor in @p table that matches
 * @p device.
 * 
 * @param table The table to search.
 * @param device A bhnd(4) bus device.
 */
const struct bhndb_hw_priority *
bhndb_hw_priority_find_device(const struct bhndb_hw_priority *table,
    device_t device)
{
	const struct bhndb_hw_priority *hp;

	for (hp = table; hp->ports != NULL; hp++) {
		if (bhnd_device_matches(device, &hp->match))
			return (hp);
	}

	/* not found */
	return (NULL);
}
