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
#include "bhndb_hwdata.h"
#include "bhndb_private.h"

/* Debugging flags */
static u_long bhndb_debug = 0;
TUNABLE_ULONG("hw.bhndb.debug", &bhndb_debug);

enum {
	BHNDB_DEBUG_PRIO = 1 << 0,
};

#define	BHNDB_DEBUG(_type)	(BHNDB_DEBUG_ ## _type & bhndb_debug)

static bool			 bhndb_hw_matches(device_t *devlist,
				     int num_devs,
				     const struct bhndb_hw *hw);

static int			 bhndb_find_hwspec(struct bhndb_softc *sc,
				     const struct bhndb_hw **hw);

static int			 bhndb_read_chipid(struct bhndb_softc *sc,
				     const struct bhndb_hwcfg *cfg,
				     struct bhnd_chipid *result);

static struct rman		*bhndb_get_rman(struct bhndb_softc *sc,
				     int type);

static struct bhndb_dw_region 	*bhndb_find_dw_region(struct bhndb_softc *sc,
				     device_t child, int type, int rid,
				     struct resource *r);

static int			 bhndb_init_child_resource(struct resource *r,
				     struct resource *parent,
				     bhnd_size_t offset,
				     bhnd_size_t size);

static int			 bhndb_activate_static_region(
				     struct bhndb_softc *sc,
				     struct bhndb_region *region, 
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
 * Initialize the region maps and priority configuration in @p r using
 * the provided priority @p table and the set of devices attached to
 * the bridged @p bus_dev .
 * 
 * @param bus_dev The bridged bhnd bus.
 * @param table Hardware priority table to be used to determine the relative
 * priorities of per-core port resources.
 * @param r The resource state to be configured.
 */
static int
bhndb_initialize_region_cfg(device_t bus_dev,
    const struct bhndb_hw_priority *table, struct bhndb_resources *r)
{
	const struct bhndb_hw_priority	*hp;
	device_t			*devices;
	bhnd_addr_t			 addr;
	bhnd_size_t			 size;
	size_t				 prio_low, prio_default, prio_high;
	int				 ndevs;
	int				 error;

	/* Fetch the full set of attached devices */
	if ((error = device_get_children(bus_dev, &devices, &ndevs)))
		return (error);

	/* The number of port regions per priority band that must be accessible
	 * via dynamic register windows */
	prio_low = 0;
	prio_default = 0;
	prio_high = 0;

	/* 
	 * Register bridge regions covering all statically mapped ports.
	 */
	for (int i = 0; i < ndevs; i++) {
		const struct bhndb_regwin	*regw;
		device_t			 child;

		child = devices[i];

		for (regw = r->cfg->register_windows;
		    regw->win_type != BHNDB_REGWIN_T_INVALID; regw++)
		{
			/* Only core windows are supported */
			if (regw->win_type != BHNDB_REGWIN_T_CORE)
				continue;

			/* Skip non-applicable register windows. */
			if (!bhndb_regwin_matches_device(regw, child))
				continue;
			
			/* Fetch the base address of the mapped port. */
			error = bhnd_get_region_addr(child, 
			    regw->core.port_type, regw->core.port, 
			    regw->core.region, &addr, &size);
			if (error)
			    return (error);

			/*
			 * Always defer to the register window's size.
			 * 
			 * If the port size is smaller than the window size,
			 * this ensures that we fully utilize register windows
			 * larger than the referenced port.
			 * 
			 * If the port size is larger than the window size, this
			 * ensures that we do not directly map the allocations
			 * within the region to a too-small window.
			 */
			size = regw->win_size;

			/*
			 * Add to the bus region list.
			 * 
			 * The window priority for a statically mapped
			 * region is always HIGH.
			 */
			error = bhndb_add_resource_region(r, addr, size,
			    BHNDB_PRIORITY_HIGH, regw);
			if (error)
				return (error);
		}
	}

	/*
	 * Perform priority accounting and register bridge regions for all
	 * ports defined in the priority table
	 */
	for (int i = 0; i < ndevs; i++) {
		struct bhndb_region	*region;
		device_t		 child;

		child = devices[i];

		/* 
		 * Skip priority accounting for cores that ...
		 */
		
		/* ... do not require bridge resources */
		if (bhnd_is_hw_disabled(child) || !device_is_enabled(child))
			continue;

		/* ... do not have a priority table entry */
		hp = bhndb_hw_priority_find_device(table, child);
		if (hp == NULL)
			continue;

		/* ... are explicitly disabled in the priority table. */
		if (hp->priority == BHNDB_PRIORITY_NONE)
			continue;

		/* Determine the number of dynamic windows required and
		 * register their bus_region entries. */
		for (u_int i = 0; i < hp->num_ports; i++) {
			const struct bhndb_port_priority *pp;

			pp = &hp->ports[i];
			
			/* Skip ports not defined on this device */
			if (pp->port >= bhnd_get_port_count(child, pp->type) ||
			    pp->region >= bhnd_get_region_count(child, pp->type,
				pp->port))
			{
				continue;
			}

			/* Fetch the address+size of the mapped port. */
			error = bhnd_get_region_addr(child, pp->type, pp->port,
			    pp->region, &addr, &size);
			if (error)
			    return (error);

			/* Skip ports with an existing static mapping */
			region = bhndb_find_resource_region(r, addr, size);
			if (region != NULL && region->static_regwin != NULL)
				continue;

			/* Define a dynamic region for this port */
			error = bhndb_add_resource_region(r, addr, size,
			    pp->priority, NULL);
			if (error)
				return (error);

			/* Update port mapping counts */
			switch (pp->priority) {
			case BHNDB_PRIORITY_NONE:
				break;
			case BHNDB_PRIORITY_LOW:
				prio_low++;
				break;
			case BHNDB_PRIORITY_DEFAULT:
				prio_default++;
				break;
			case BHNDB_PRIORITY_HIGH:
				prio_high++;
				break;
			}
		}
	}

	/* Determine the minimum priority at which we'll allocate direct
	 * register windows from our dynamic pool */
	size_t prio_total = prio_low + prio_default + prio_high;
	if (prio_total <= r->dw_count) {
		/* low+default+high priority regions get windows */
		r->min_prio = BHNDB_PRIORITY_LOW;

	} else if (prio_default + prio_high <= r->dw_count) {
		/* default+high priority regions get windows */
		r->min_prio = BHNDB_PRIORITY_DEFAULT;

	} else {
		/* high priority regions get windows */
		r->min_prio = BHNDB_PRIORITY_HIGH;
	}

	if (BHNDB_DEBUG(PRIO)) {
		struct bhndb_region	*region;
		const char		*direct_msg, *type_msg;
		bhndb_priority_t	 prio, prio_min;

		prio_min = r->min_prio;
		device_printf(r->dev, "min_prio: %d\n", prio_min);

		STAILQ_FOREACH(region, &r->bus_regions, link) {
			prio = region->priority;

			direct_msg = prio >= prio_min ? "direct" : "indirect";
			type_msg = region->static_regwin ? "static" : "dynamic";
	
			device_printf(r->dev, "region 0x%llx+0x%llx priority "
			    "%u %s/%s\n",
			    (unsigned long long) region->addr, 
			    (unsigned long long) region->size,
			    region->priority,
			    direct_msg, type_msg);
		}
	}

	free(devices, M_TEMP);
	return (0);
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
 * Read the ChipCommon identification data for this device.
 * 
 * @param sc bhndb device state.
 * @param cfg The hardware configuration to use when mapping the ChipCommon
 * registers.
 * @param[out] result the chip identification data.
 * 
 * @retval 0 success
 * @retval non-zero if the ChipCommon identification data could not be read.
 */
static int
bhndb_read_chipid(struct bhndb_softc *sc, const struct bhndb_hwcfg *cfg,
    struct bhnd_chipid *result)
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
	cc_win = bhndb_regwin_find_best(cfg->register_windows,
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
	const struct bhndb_hwcfg	*cfg;
	int				 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->parent_dev = device_get_parent(dev);
	sc->bridge_class = bridge_devclass;

	BHNDB_LOCK_INIT(sc);
	
	/* Read our chip identification data */
	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);
	if ((error = bhndb_read_chipid(sc, cfg, &sc->chipid)))
		return (error);

	/* Set up a resource manager for the device's address space. */
	sc->mem_rman.rm_start = 0;
	sc->mem_rman.rm_end = BUS_SPACE_MAXADDR_32BIT;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "BHND I/O memory addresses";
	
	if ((error = rman_init(&sc->mem_rman))) {
		device_printf(dev, "could not initialize mem_rman\n");
		return (error);
	}

	error = rman_manage_region(&sc->mem_rman, 0, BUS_SPACE_MAXADDR_32BIT);
	if (error) {
		device_printf(dev, "could not configure mem_rman\n");
		goto failed;
	}

	/* Initialize basic resource allocation state. */
	sc->bus_res = bhndb_alloc_resources(dev, sc->parent_dev, cfg);
	if (sc->bus_res == NULL) {
		error = ENXIO;
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
	BHNDB_LOCK_DESTROY(sc);

	rman_fini(&sc->mem_rman);

	if (sc->bus_res != NULL)
		bhndb_free_resources(sc->bus_res);

	return (error);
}

static int
bhndb_init_full_config(device_t dev, device_t child,
    const struct bhndb_hw_priority *hw_prio_table)
{
	struct bhndb_softc		*sc;
	const struct bhndb_hw		*hw;
	struct bhndb_resources		*r;
	int				 error;

	sc = device_get_softc(dev);
	error = 0;
	
	/* Find our full register window configuration */
	if ((error = bhndb_find_hwspec(sc, &hw))) {
		device_printf(sc->dev, "unable to identify device, "
			" using generic bridge resource definitions\n");
		return (0);
	}

	if (bootverbose)
		device_printf(sc->dev, "%s resource configuration\n", hw->name);

	/* Release existing resource state */
	BHNDB_LOCK(sc);
	bhndb_free_resources(sc->bus_res);
	sc->bus_res = NULL;
	BHNDB_UNLOCK(sc);

	/* Allocate new resource state */
	r = bhndb_alloc_resources(dev, sc->parent_dev, hw->cfg);
	if (r == NULL)
		return (ENXIO);

	/* Initialize our resource priority configuration */
	error = bhndb_initialize_region_cfg(sc->bus_dev, hw_prio_table, r);
	if (error) {
		bhndb_free_resources(r);
		return (error);
	}

	/* Reinitialize our resource state */
	BHNDB_LOCK(sc);
	sc->bus_res = r;
	BHNDB_UNLOCK(sc);

	return (0);
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
	bhndb_free_resources(sc->bus_res);
	
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
	struct bhndb_softc	*sc;
	struct bhndb_resources	*bus_res;
	struct bhndb_dw_region	*dw_region;
	int			 error;

	sc = device_get_softc(dev);
	bus_res = sc->bus_res;

	/* Guarantee that all in-use dynamic register windows are mapped to
	 * their previously configured target address. */
	BHNDB_LOCK(sc);
	for (size_t i = 0; i < bus_res->dw_count; i++) {
		dw_region = &bus_res->dw_regions[i];
	
		/* If not in-use, return the region to its initial state */
		if (BHNDB_DW_REGION_IS_FREE(bus_res, i)) {
			dw_region->target = 0x0;
			continue;
		}

		/* Otherwise, ensure the register window is correct before
		 * any children attempt MMIO */
		error = BHNDB_SET_WINDOW_ADDR(dev, dw_region->win, 
		    dw_region->target);
		if (error)
			break;
	}
	BHNDB_UNLOCK(sc);

	/* Error restoring hardware state; children cannot be safely resumed */
	if (error) {
		device_printf(dev, "Unable to restore hardware configuration; "
		    "cannot resume: %d\n", error);
		return (error);
	}

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

static const struct bhnd_chipid *
bhndb_get_chipid(device_t dev, device_t child)
{
	struct bhndb_softc *sc = device_get_softc(dev);
	return (&sc->chipid);
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
 * Find the dynamic region allocated for @p r, if any.
 * 
 * @param sc bhndb device state.
 * @param child The child device making a resource request on @p r.
 * @param type The resource type.
 * @param rid The resource ID.
 * @param r The resource to search for.
 * 
 * @retval bhndb_dw_region The region allocated for @p r.
 * @retval NULL if no region is allocated for @p r.
 */
static struct bhndb_dw_region *
bhndb_find_dw_region(struct bhndb_softc *sc, device_t child, int type,
    int rid, struct resource *r)
{
	struct bhndb_dw_region	*region, *ret;
	struct rman		*rm;
	
	BHNDB_LOCK_ASSERT(sc, MA_OWNED);

	if ((rm = bhndb_get_rman(sc, type)) == NULL)
		return (NULL);

	ret = NULL;
	for (size_t i = 0; i < sc->bus_res->dw_count; i++) {
		region = &sc->bus_res->dw_regions[i];

		/* Skip free regions */
		if (BHNDB_DW_REGION_IS_FREE(sc->bus_res, i))
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

	// TODO: Currently unsupported
	error = ENODEV;

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
 * @param region The static region definition capable of mapping @p r.
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
bhndb_activate_static_region(struct bhndb_softc *sc,
    struct bhndb_region *region, device_t child, int type, int rid,
    struct resource *r)
{
	struct resource			*bridge_res;
	const struct bhndb_regwin	*win;
	bhnd_size_t			 parent_offset;
	u_long				 r_start, r_size;
	int				 error;

	win = region->static_regwin;

	KASSERT(win != NULL && BHNDB_REGWIN_T_IS_STATIC(win->win_type),
	    ("can't activate non-static region"));

	r_start = rman_get_start(r);
	r_size = rman_get_size(r);

	/* Find the corresponding bridge resource */
	bridge_res = bhndb_find_regwin_resource(sc->bus_res, win);
	if (bridge_res == NULL)
		return (ENXIO);
	
	/* Calculate subregion offset within the parent resource */
	parent_offset = r_start - region->addr;
	parent_offset += win->win_offset;

	/* Configure resource with its real bus values. */
	error = bhndb_init_child_resource(r, bridge_res, parent_offset, r_size);
	if (error)
		return (error);

	/* Mark active */
	if ((error = rman_activate_resource(r)))
		return (error);

	return (0);
}

/**
 * Configure a dynamic window region to map @p size bytes at @p addr. This
 * will apply any necessary window alignment and verify that the window
 * is capable of mapping the requested range. 
 * 
 * @param sc BHNDB driver state.
 * @param dw_region The region to be configured.
 * @param addr The address to be mapped via @p dw_region.
 * @param size The number of bytes to be mapped at @p addr.
 * @param r A resource to be mapped via a dynamic register window.
 * @param[out] region On success, the allocated region.
 * @param[out] indirect On error and if not NULL, will be set to 'true' if the
 * caller should instead use an indirect resource mapping.
 *
 * @retval 0 success
 * @retval non-zero no usable register window available.
 */
static int
bhndb_set_dwr_target(struct bhndb_softc *sc, struct bhndb_dw_region *dw_region,
    bus_addr_t addr, bus_size_t size, bool *indirect)
{
	const struct bhndb_regwin	*rw;
	u_long				 offset;
	int				 error;

	BHNDB_LOCK_ASSERT(sc, MA_OWNED);

	rw = dw_region->win;
	if (indirect != NULL)
		*indirect = false;

	/* Page-align the region target address */
	offset = addr % rw->win_size;
	dw_region->target = addr - offset;

	/* Verify that the window is large enough for the full target */
	if (rw->win_size - offset < size) {
		/* The caller can still use indirect mapping */
		if (indirect != NULL)
			*indirect = true;

		return (ENOMEM);
	}
	
	/* Update the window target */
	error = BHNDB_SET_WINDOW_ADDR(sc->dev, dw_region->win,
	    dw_region->target);
	if (error) {
		dw_region->target = 0x0;
		return (error);
	}

	return (0);
}

/**
 * Activate a resource using any viable static or dynamic register window.
 * 
 * @param sc The bhndb driver state.
 * @param child The child holding ownership of @p r.
 * @param type The type of the resource to be activated.
 * @param rid The resource ID of @p r.
 * @param r The resource to be activated
 * @param[out] indirect On error and if not NULL, will be set to 'true' if
 * the caller should instead use an indirect resource mapping.
 * 
 * @retval 0 success
 * @retval non-zero activation failed.
 */
static int
bhndb_try_activate_resource(struct bhndb_softc *sc, device_t child, int type,
    int rid, struct resource *r, bool *indirect)
{
	struct bhndb_region	*region;
	struct bhndb_dw_region	*dw_region;
	bhndb_priority_t	 dw_priority;
	u_long			 r_start, r_size;
	u_long			 parent_offset;
	int			 rnid;
	int			 error;

	// TODO - IRQs
	if (type != SYS_RES_MEMORY)
		return (ENXIO);
	
	if (indirect)
		*indirect = false;

	/* Default to low priority */
	dw_priority = BHNDB_PRIORITY_LOW;

	/* Look for a bus region matching the resource's address range */
	r_start = rman_get_start(r);
	r_size = rman_get_size(r);
	region = bhndb_find_resource_region(sc->bus_res, r_start, r_size);
	if (region != NULL)
		dw_priority = region->priority;

	/* Prefer static mappings over consuming a dynamic windows. */
	if (region && region->static_regwin) {
		error = bhndb_activate_static_region(sc, region, child, type,
		    rid, r);
		if (error)
			device_printf(sc->dev, "static window allocation "
			     "for 0x%llx-0x%llx failed\n",
			     (unsigned long long) r_start,
			     (unsigned long long) r_start + r_size - 1);
		return (error);
	}

	/* A dynamic window will be required; is this resource high enough
	 * priority to be reserved a dynamic window? */
	if (dw_priority < sc->bus_res->min_prio) {
		if (indirect)
			*indirect = true;

		return (ENOMEM);
	}

	/* Find and claim next free window */
	BHNDB_LOCK(sc); {
		if (BHNDB_DW_REGION_EXHAUSTED(sc->bus_res)) {
			BHNDB_UNLOCK(sc);
			if (indirect)
				*indirect = true;

			return (ENOMEM);
		}

		/* Reserve a free region */
		rnid = BHNDB_DW_REGION_NEXT_FREE(sc->bus_res);
		BHNDB_DW_REGION_RESERVE(sc->bus_res, rnid, r);
		dw_region = &sc->bus_res->dw_regions[rnid];
		
		/* Set the region target */
		error = bhndb_set_dwr_target(sc, dw_region, rman_get_start(r),
		    rman_get_size(r), indirect);
		if (error) {
			BHNDB_UNLOCK(sc);
			BHNDB_DW_REGION_RELEASE(sc->bus_res, rnid);
			device_printf(sc->dev, "dynamic window initialization "
			     "for 0x%llx-0x%llx failed\n",
			     (unsigned long long) r_start,
			     (unsigned long long) r_start + r_size - 1);
			
			return (error);
		}
	} BHNDB_UNLOCK(sc);

	/* Configure resource with its real bus values. */
	parent_offset = dw_region->win->win_offset;
	parent_offset += r_start - dw_region->target;

	error = bhndb_init_child_resource(r, dw_region->parent_res,
	    parent_offset, dw_region->win->win_size);
	if (error)
		goto failed;

	/* Mark active */
	if ((error = rman_activate_resource(r)))
		goto failed;


	return (0);

failed:
	/* Release our region allocation. */
	BHNDB_LOCK(sc);
	BHNDB_DW_REGION_RELEASE(sc->bus_res, dw_region->rnid);
	BHNDB_UNLOCK(sc);

	return (error);
}

/**
 * Implementation of BUS_ACTIVATE_RESOURCE() that maps resources to any usable
 * static or dynamic register window.
 * 
 *  * @param[out] indirect On error, will be set to 'true' if the caller should
 * instead use an indirect resource mapping.
 * 
 * @retval 0 success
 * @retval non-zero activation failed.
 */
static int
bhndb_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	struct bhndb_softc *sc = device_get_softc(dev);

	return (bhndb_try_activate_resource(sc, child, type, rid, r, NULL));
}

static int
bhndb_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	struct bhndb_dw_region	*region;
	struct bhndb_softc	*sc;
	struct rman		*rm;
	int			 error;

	sc = device_get_softc(dev);

	if ((rm = bhndb_get_rman(sc, type)) == NULL)
		return (EINVAL);

	/* Mark inactive */
	if ((error = rman_deactivate_resource(r)))
		return (error);

	/* Free any dynamic window allocation. */
	BHNDB_LOCK(sc);
	region = bhndb_find_dw_region(sc, child, type, rid, r);
	if (region != NULL)
		BHNDB_DW_REGION_RELEASE(sc->bus_res, region->rnid);
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
	br->direct = false;
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
	struct bhndb_softc	*sc;
	struct bhndb_region	*region;
	bhndb_priority_t	 r_prio;
	u_long			 r_start, r_size;
	int 			 error;
	bool			 indirect;

	KASSERT(!r->direct,
	    ("direct flag set on inactive resource"));
	
	KASSERT(!(rman_get_flags(r->res) & RF_ACTIVE),
	    ("RF_ACTIVE set on inactive resource"));

	sc = device_get_softc(dev);

	/* Fetch the address range's resource priority */
	r_start = rman_get_start(r->res);
	r_size = rman_get_size(r->res);
	r_prio = BHNDB_PRIORITY_NONE;

	region = bhndb_find_resource_region(sc->bus_res, r_start, r_size);
	if (region != NULL)
		r_prio = region->priority;
	
	/* If less than the minimum dynamic window priority, this
	 * resource should always be indirect. */
	if (r_prio < sc->bus_res->min_prio)
		return (0);

	/* Attempt direct activation */
	error = bhndb_try_activate_resource(sc, child, type, rid, r->res,
	    &indirect);
	if (!error) {
		r->direct = true;
	} else if (indirect) {
		/* The request was valid, but no viable register window is
		 * available; indirection must be employed. */
		error = 0;
		r->direct = false;
	}

	if (BHNDB_DEBUG(PRIO)) {
		device_printf(child, "activated 0x%llx-0x%llx as %s "
		    "resource\n",
		    (unsigned long long) r_start, 
		    (unsigned long long) r_start + r_size - 1,
		    r->direct ? "direct" : "indirect");
	}

	return (error);
};

static int
bhndb_deactivate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	int error;

	/* Indirect resources don't require activation */
	if (!r->direct)
		return (0);

	KASSERT(rman_get_flags(r->res) & RF_ACTIVE,
	    ("RF_ACTIVE not set on direct resource"));

	/* Perform deactivation */
	error = bus_deactivate_resource(child, type, rid, r->res);
	if (!error)
		r->direct = false;

	return (error);
};

/* slow path for bhndb_io_resource; iterates over the existing allocated
 * dw_regions looking for a viable in-use region */
static struct bhndb_dw_region *
bhndb_io_resource_slow(struct bhndb_softc *sc, bus_addr_t addr,
    bus_size_t size, bus_size_t *offset)
{
	struct bhndb_resources	*br;
	struct bhndb_dw_region	*dw_region;

	BHNDB_LOCK_ASSERT(sc, MA_OWNED);

	br = sc->bus_res;

	/* Search for an existing dynamic mapping of this address range.
	 * Static regions are not searched, as a statically mapped
	 * region would never be allocated as an indirect resource. */
	for (size_t i = 0; i < br->dw_count; i++) {
		const struct bhndb_regwin *win;

		dw_region = &br->dw_regions[i];
		win = dw_region->win;

		KASSERT(win->win_type == BHNDB_REGWIN_T_DYN,
			("invalid register window type"));

		/* Verify the range */
		if (addr < dw_region->target)
			continue;

		if (addr + size > dw_region->target + win->win_size)
			continue;

		/* Found */
		*offset = dw_region->win->win_offset;
		*offset += addr - dw_region->target;

		return (dw_region);
	}

	/* not found */
	return (NULL);
}

/**
 * Find the bridge resource to be used for I/O requests.
 * 
 * @param sc Bridge driver state.
 * @param addr The I/O target address.
 * @param size The size of the I/O operation to be performed at @p addr. 
 * @param[out] offset The offset within the returned resource at which
 * to perform the I/O request.
 */
static inline struct bhndb_dw_region *
bhndb_io_resource(struct bhndb_softc *sc, bus_addr_t addr, bus_size_t size,
    bus_size_t *offset)
{
	struct bhndb_resources	*br;
	struct bhndb_dw_region	*dwr;
	u_int			 rnid;
	int			 error;

	BHNDB_LOCK_ASSERT(sc, MA_OWNED);

	br = sc->bus_res;

	/*
	 * If no dynamic regions are available, look for an existing
	 * region that maps the target range. 
	 * 
	 * If none are found, this is a child driver bug -- our window
	 * over-commit should only fail in the case where a child driver leaks
	 * resources, or perform operations out-of-order.
	 * 
	 * Broadcom HND chipsets are designed to not require register window
	 * swapping during execution; as long as the child devices are
	 * attached/detached correctly, using the hardware's required order
	 * of operations, there should always be a window available for the
	 * current operation.
	 */
	if (BHNDB_DW_REGION_EXHAUSTED(br)) {
		dwr = bhndb_io_resource_slow(sc, addr, size, offset);
		if (dwr == NULL) {
			panic("register windows exhausted attempting to map "
			    "0x%llx-0x%llx\n", 
			    (unsigned long long) addr,
			    (unsigned long long) addr+size-1);
		}

		return (dwr);
	}

	/* Fetch a free window */
	rnid = BHNDB_DW_REGION_NEXT_FREE(br);
	dwr = &br->dw_regions[rnid];

	/* Adjust the window if the I/O request won't fit in the current
	 * target range. */
	if (addr < dwr->target || 
	   (dwr->target + dwr->win->win_size) - addr < size)
	{
		if ((error = bhndb_set_dwr_target(sc, dwr, addr, size, NULL))) {
		    panic("failed to set register window target mapping "
			    "0x%llx-0x%llx\n", 
			    (unsigned long long) addr,
			    (unsigned long long) addr+size-1);
		}
	}

	/* Calculate the offset and return */
	*offset = (addr - dwr->target) + dwr->win->win_offset;
	return (dwr);
}

/* bhndb_bus_(read|write) common implementation */
#define	BHNDB_IO_COMMON_SETUP(_io_size)				\
	struct bhndb_softc	*sc;				\
	struct bhndb_dw_region	*dwr;				\
	struct resource		*io_res;			\
	bus_size_t		 io_offset;			\
								\
	sc = device_get_softc(dev);				\
								\
	BHNDB_LOCK(sc);						\
	dwr = bhndb_io_resource(sc, rman_get_start(r->res) +	\
	    offset, _io_size, &io_offset);			\
	io_res = dwr->parent_res;				\
								\
	KASSERT(!r->direct,					\
	    ("bhnd_bus slow path used for direct resource"));	\
								\
	KASSERT(rman_get_flags(io_res) & RF_ACTIVE,		\
	    ("i/o resource is not active"));

#define	BHNDB_IO_COMMON_TEARDOWN()				\
	BHNDB_UNLOCK(sc);

/* Defines a bhndb_bus_read_* method implementation */
#define	BHNDB_IO_READ(_type, _size)				\
static _type							\
bhndb_bus_read_ ## _size (device_t dev, device_t child,		\
    struct bhnd_resource *r, bus_size_t offset)			\
{								\
	_type v;						\
	BHNDB_IO_COMMON_SETUP(sizeof(_type));			\
	v = bus_read_ ## _size (io_res, io_offset);		\
	BHNDB_IO_COMMON_TEARDOWN();				\
								\
	return (v);						\
}

/* Defines a bhndb_bus_write_* method implementation */
#define	BHNDB_IO_WRITE(_type, _size)				\
static void							\
bhndb_bus_write_ ## _size (device_t dev, device_t child,	\
    struct bhnd_resource *r, bus_size_t offset, _type value)	\
{								\
	BHNDB_IO_COMMON_SETUP(sizeof(_type));			\
	bus_write_ ## _size (io_res, io_offset, value);		\
	BHNDB_IO_COMMON_TEARDOWN();				\
}

BHNDB_IO_READ(uint8_t, 1);
BHNDB_IO_READ(uint16_t, 2);
BHNDB_IO_READ(uint32_t, 4);

BHNDB_IO_WRITE(uint8_t, 1);
BHNDB_IO_WRITE(uint16_t, 2);
BHNDB_IO_WRITE(uint32_t, 4);

static void 
bhndb_bus_barrier(device_t dev, device_t child, struct bhnd_resource *r,
    bus_size_t offset, bus_size_t length, int flags)
{
	bus_size_t remain;

	BHNDB_IO_COMMON_SETUP(length);

	/* TODO: It's unclear whether we need a barrier implementation,
	 * and if we do, what it needs to actually do. This may need
	 * revisiting once we have a better idea of requirements after
	 * porting the core drivers. */
	panic("implementation incorrect");

	/* Use 4-byte reads where possible */
	remain = length % sizeof(uint32_t);
	for (bus_size_t i = 0; i < (length - remain); i += 4)
		bus_read_4(io_res, io_offset + offset + i);

	/* Use 1 byte reads for the remainder */
	for (bus_size_t i = 0; i < remain; i++)
		bus_read_1(io_res, io_offset + offset + length + i);

	BHNDB_IO_COMMON_TEARDOWN();
}

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
	DEVMETHOD(bhnd_get_chipid,		bhndb_get_chipid),
	DEVMETHOD(bhnd_alloc_resource,		bhndb_alloc_bhnd_resource),
	DEVMETHOD(bhnd_release_resource,	bhndb_release_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_activate_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhndb_deactivate_bhnd_resource),
	DEVMETHOD(bhnd_bus_read_1,		bhndb_bus_read_1),
	DEVMETHOD(bhnd_bus_read_2,		bhndb_bus_read_2),
	DEVMETHOD(bhnd_bus_read_4,		bhndb_bus_read_4),
	DEVMETHOD(bhnd_bus_write_1,		bhndb_bus_write_1),
	DEVMETHOD(bhnd_bus_write_2,		bhndb_bus_write_2),
	DEVMETHOD(bhnd_bus_write_4,		bhndb_bus_write_4),
	DEVMETHOD(bhnd_bus_barrier,		bhndb_bus_barrier),

	DEVMETHOD_END
};

devclass_t bhndb_devclass;

DEFINE_CLASS_0(bhndb, bhndb_driver, bhndb_methods, sizeof(struct bhndb_softc));

MODULE_VERSION(bhndb, 1);
MODULE_DEPEND(bhndb, bhnd, 1, 1, 1);
