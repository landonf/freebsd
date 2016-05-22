/*-
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
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
 * Broadcom ChipCommon driver.
 * 
 * With the exception of some very early chipsets, the ChipCommon core
 * has been included in all HND SoCs and chipsets based on the siba(4) 
 * and bcma(4) interconnects, providing a common interface to chipset 
 * identification, bus enumeration, UARTs, clocks, watchdog interrupts, GPIO, 
 * flash, etc.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndvar.h>

#include "chipcreg.h"
#include "chipcvar.h"

devclass_t bhnd_chipc_devclass;	/**< bhnd(4) chipcommon device class */

static struct bhnd_device_quirk chipc_quirks[];
static struct bhnd_chip_quirk chipc_chip_quirks[];

/* Supported device identifiers */
static const struct bhnd_device chipc_devices[] = {
	BHND_DEVICE(CC, "CC", chipc_quirks, chipc_chip_quirks),
	BHND_DEVICE_END
};


/* Device quirks table */
static struct bhnd_device_quirk chipc_quirks[] = {
	{ BHND_HWREV_GTE	(32),	CHIPC_QUIRK_SUPPORTS_SPROM },
	{ BHND_HWREV_GTE	(35),	CHIPC_QUIRK_SUPPORTS_NFLASH },
	BHND_DEVICE_QUIRK_END
};

/* Chip-specific quirks table */
static struct bhnd_chip_quirk chipc_chip_quirks[] = {
	/* 4331 12x9 packages */
	{{ BHND_CHIP_IP(4331, 4331TN) },
		CHIPC_QUIRK_4331_GPIO2_5_MUX_SPROM
	},
	{{ BHND_CHIP_IP(4331, 4331TNA0) },
		CHIPC_QUIRK_4331_GPIO2_5_MUX_SPROM
	},

	/* 4331 12x12 packages */
	{{ BHND_CHIP_IPR(4331, 4331TT, HWREV_GTE(1)) },
		CHIPC_QUIRK_4331_EXTPA2_MUX_SPROM
	},

	/* 4331 (all packages/revisions) */
	{{ BHND_CHIP_ID(4331) },
		CHIPC_QUIRK_4331_EXTPA_MUX_SPROM
	},

	/* 4360 family (all revs <= 2) */
	{{ BHND_CHIP_IR(4352, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },
	{{ BHND_CHIP_IR(43460, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },
	{{ BHND_CHIP_IR(43462, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },
	{{ BHND_CHIP_IR(43602, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },

	BHND_CHIP_QUIRK_END
};

static struct chipc_region	*chipc_alloc_region(struct chipc_softc *sc,
				     bhnd_port_type type, u_int port,
				     u_int region);
static void			 chipc_free_region(struct chipc_softc *sc,
				     struct chipc_region *cr);
static struct chipc_region	*chipc_find_region(struct chipc_softc *sc,
				     rman_res_t start, rman_res_t end);
static struct chipc_region	*chipc_find_region_by_rid(struct chipc_softc *sc,
				     int rid);

int				 chipc_retain_region(struct chipc_softc *sc,
				     struct chipc_region *cr, int flags);
int				 chipc_release_region(struct chipc_softc *sc,
				     struct chipc_region *cr, int flags);

static int			 chipc_try_activate_resource(
				    struct chipc_softc *sc, device_t child,
				    int type, int rid, struct resource *r,
				    bool req_direct);

static int			 chipc_init_rman(struct chipc_softc *sc);
static void			 chipc_free_rman(struct chipc_softc *sc);
static struct rman		*chipc_get_rman(struct chipc_softc *sc,
				     int type);

static int			 chipc_nvram_attach(struct chipc_softc *sc);
static bhnd_nvram_src_t		 chipc_nvram_identify(struct chipc_softc *sc);
static bool			 chipc_should_enable_sprom(
				     struct chipc_softc *sc);

/* quirk and capability flag convenience macros */
#define	CHIPC_QUIRK(_sc, _name)	\
    ((_sc)->quirks & CHIPC_QUIRK_ ## _name)
    
#define CHIPC_CAP(_sc, _name)	\
    ((_sc)->caps & CHIPC_ ## _name)

#define	CHIPC_ASSERT_QUIRK(_sc, name)	\
    KASSERT(CHIPC_QUIRK((_sc), name), ("quirk " __STRING(_name) " not set"))

#define	CHIPC_ASSERT_CAP(_sc, name)	\
    KASSERT(CHIPC_CAP((_sc), name), ("capability " __STRING(_name) " not set"))

static int
chipc_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, chipc_devices, sizeof(chipc_devices[0]));
	if (id == NULL)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

/* Allocate region records for the given port, and add the port's memory
 * range to the mem_rman */
static int
chipc_rman_init_regions (struct chipc_softc *sc, bhnd_port_type type,
    u_int port)
{
	struct	chipc_region	*cr;
	rman_res_t		 start, end;
	u_int			 num_regions;
	int			 error;

	num_regions = bhnd_get_region_count(sc->dev, port, port);
	for (u_int region = 0; region < num_regions; region++) {
		/* Allocate new region record */
		cr = chipc_alloc_region(sc, type, port, region);
		if (cr == NULL)
			return (ENODEV);

		/* Can't manage regions that cannot be allocated */
		if (cr->cr_rid < 0) {
			BHND_DEBUG_DEV(sc->dev, "no rid for chipc region "
			    "%s%u.%u", bhnd_port_type_name(type), port, region);
			chipc_free_region(sc, cr);
			continue;
		}

		/* Add to rman's managed range */
		start = cr->cr_addr;
		end = cr->cr_end;
		if ((error = rman_manage_region(&sc->mem_rman, start, end))) {
			chipc_free_region(sc, cr);
			return (error);
		}

		/* Add to region list */
		STAILQ_INSERT_TAIL(&sc->mem_regions, cr, cr_link);
	}

	return (0);
}

/* Initialize memory state for all chipc port regions */
static int
chipc_init_rman(struct chipc_softc *sc)
{
	u_int	num_ports;
	int	error;

	/* Port types for which we'll register chipc_region mappings */
	bhnd_port_type types[] = {
	    BHND_PORT_DEVICE
	};

	/* Initialize resource manager */
	sc->mem_rman.rm_start = 0;
	sc->mem_rman.rm_end = BUS_SPACE_MAXADDR;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "ChipCommon Device Memory";
	if ((error = rman_init(&sc->mem_rman))) {
		device_printf(sc->dev, "could not initialize mem_rman: %d\n",
		    error);
		return (error);
	}

	/* Populate per-port-region state */
	for (u_int i = 0; i < nitems(types); i++) {
		num_ports = bhnd_get_port_count(sc->dev, types[i]);
		for (u_int port = 0; port < num_ports; port++) {
			error = chipc_rman_init_regions(sc, types[i], port);
			if (error) {
				device_printf(sc->dev,
				    "region init failed for %s%u: %d\n",
				     bhnd_port_type_name(types[i]), port,
				     error);

				goto failed;
			}
		}
	}

	return (0);

failed:
	chipc_free_rman(sc);
	return (error);
}

/* Free memory management state */
static void
chipc_free_rman(struct chipc_softc *sc)
{
	struct chipc_region *cr, *cr_next;

	STAILQ_FOREACH_SAFE(cr, &sc->mem_regions, cr_link, cr_next)
		chipc_free_region(sc, cr);

	rman_fini(&sc->mem_rman);
}

/**
 * Return the rman instance for a given resource @p type, if any.
 * 
 * @param sc The chipc device state.
 * @param type The resource type (e.g. SYS_RES_MEMORY, SYS_RES_IRQ, ...)
 */
static struct rman *
chipc_get_rman(struct chipc_softc *sc, int type)
{	
	switch (type) {
	case SYS_RES_MEMORY:
		return (&sc->mem_rman);

	case SYS_RES_IRQ:
		/* IRQs can be used with RF_SHAREABLE, so we don't perform
		 * any local proxying of resource requests. */
		return (NULL);

	default:
		return (NULL);
	};
}

static int
chipc_attach(device_t dev)
{
	struct chipc_softc		*sc;
	bhnd_addr_t			 enum_addr;
	uint32_t			 ccid_reg;
	uint8_t				 chip_type;
	int				 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = bhnd_device_quirks(dev, chipc_devices,
	    sizeof(chipc_devices[0]));
	sc->sprom_refcnt = 0;

	CHIPC_LOCK_INIT(sc);
	STAILQ_INIT(&sc->mem_regions);

	/* Set up resource management */
	if ((error = chipc_init_rman(sc))) {
		device_printf(sc->dev,
		    "failed to initialize chipc resource state: %d\n", error);
		goto failed;
	}

	/* Allocate the region containing our core registers */
	if ((sc->core_region = chipc_find_region_by_rid(sc, 0)) == NULL) {
		error = ENXIO;
		goto failed;
	}

	error = chipc_retain_region(sc, sc->core_region,
	    RF_ALLOCATED|RF_ACTIVE);
	if (error) {
		sc->core_region = NULL;
		goto failed;
	} else {
		sc->core = sc->core_region->cr_res;
	}

	/* Fetch our chipset identification data */
	ccid_reg = bhnd_bus_read_4(sc->core, CHIPC_ID);
	chip_type = CHIPC_GET_ATTR(ccid_reg, ID_BUS);

	switch (chip_type) {
	case BHND_CHIPTYPE_SIBA:
		/* enumeration space starts at the ChipCommon register base. */
		enum_addr = rman_get_start(sc->core->res);
		break;
	case BHND_CHIPTYPE_BCMA:
	case BHND_CHIPTYPE_BCMA_ALT:
		enum_addr = bhnd_bus_read_4(sc->core, CHIPC_EROMPTR);
		break;
	default:
		device_printf(dev, "unsupported chip type %hhu\n", chip_type);
		error = ENODEV;
		goto failed;
	}

	sc->ccid = bhnd_parse_chipid(ccid_reg, enum_addr);

	/* Fetch capability and status register values */
	sc->caps = bhnd_bus_read_4(sc->core, CHIPC_CAPABILITIES);
	sc->cst = bhnd_bus_read_4(sc->core, CHIPC_CHIPST);

	/* Identify NVRAM source and add child device. */
	sc->nvram_src = chipc_nvram_identify(sc);
	if ((error = chipc_nvram_attach(sc)))
		goto failed;

	/* Standard bus probe */
	if ((error = bus_generic_attach(dev)))
		goto failed;

	return (0);
	
failed:
	if (sc->core_region != NULL) {
		chipc_release_region(sc, sc->core_region,
		    RF_ALLOCATED|RF_ACTIVE);
	}

	chipc_free_rman(sc);
	CHIPC_LOCK_DESTROY(sc);
	return (error);
}

static int
chipc_detach(device_t dev)
{
	struct chipc_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bus_generic_detach(dev)))
		return (error);

	chipc_release_region(sc, sc->core_region, RF_ALLOCATED|RF_ACTIVE);
	chipc_free_rman(sc);
	bhnd_sprom_fini(&sc->sprom);

	CHIPC_LOCK_DESTROY(sc);

	return (0);
}

static int
chipc_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

static int
chipc_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

static void
chipc_probe_nomatch(device_t dev, device_t child)
{
	struct resource_list	*rl;
	const char		*name;

	name = device_get_name(child);
	if (name == NULL)
		name = "unknown device";

	device_printf(dev, "<%s> at", name);

	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl != NULL) {
		resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#jx");
		resource_list_print_type(rl, "irq", SYS_RES_IRQ, "%jd");
	}

	printf(" (no driver attached)\n");
}

static int
chipc_print_child(device_t dev, device_t child)
{
	struct resource_list	*rl;
	int			 retval = 0;

	retval += bus_print_child_header(dev, child);

	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl != NULL) {
		retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY,
		    "%#jx");
		retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ,
		    "%jd");
	}

	retval += bus_print_child_domain(dev, child);
	retval += bus_print_child_footer(dev, child);

	return (retval);
}

static int
chipc_child_pnpinfo_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	if (buflen == 0)
		return (EOVERFLOW);

	*buf = '\0';
	return (0);
}

static int
chipc_child_location_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	if (buflen == 0)
		return (EOVERFLOW);

	*buf = '\0';
	return (ENXIO);
}

static device_t
chipc_add_child(device_t dev, u_int order, const char *name, int unit)
{
	struct chipc_devinfo	*dinfo;
	device_t		 child;

	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return (NULL);

	dinfo = malloc(sizeof(struct chipc_devinfo), M_BHND, M_NOWAIT);
	if (dinfo == NULL) {
		device_delete_child(dev, child);
		return (NULL);
	}

	resource_list_init(&dinfo->resources);

	device_set_ivars(child, dinfo);

	return (child);
}

static void
chipc_child_deleted(device_t dev, device_t child)
{
	struct chipc_devinfo *dinfo = device_get_ivars(child);

	if (dinfo != NULL) {
		resource_list_free(&dinfo->resources);
		free(dinfo, M_BHND);
	}

	device_set_ivars(child, NULL);
}

static struct resource_list *
chipc_get_resource_list(device_t dev, device_t child)
{
	struct chipc_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

/**
 * Allocate and initialize new region record.
 * 
 * @param sc Driver instance state.
 * @param type The port type to query.
 * @param port The port number to query.
 * @param region The region number to query.
 */
static struct chipc_region *
chipc_alloc_region(struct chipc_softc *sc, bhnd_port_type type,
    u_int port, u_int region)
{
	struct chipc_region	*cr;
	int			 error;

	/* Don't bother allocating a chipc_region if init will fail */
	if (!bhnd_is_region_valid(sc->dev, type, port, region))
		return (NULL);

	/* Allocate and initialize region info */
	cr = malloc(sizeof(*cr), M_BHND, M_NOWAIT);
	if (cr == NULL)
		return (NULL);

	cr->cr_port_type = type;
	cr->cr_port_num = port;
	cr->cr_region_num = region;
	cr->cr_res = NULL;
	cr->cr_refs = 0;
	cr->cr_act_refs = 0;

	error = bhnd_get_region_addr(sc->dev, type, port, region, &cr->cr_addr,
	    &cr->cr_count);
	if (error) {
		device_printf(sc->dev,
		    "fetching chipc region address failed: %d\n", error);
		goto failed;
	}

	cr->cr_end = cr->cr_addr + cr->cr_count - 1;

	/* Note that not all regions have an assigned rid, in which case
	 * this will return -1 */
	cr->cr_rid = bhnd_get_port_rid(sc->dev, type, port, region);
	return (cr);

failed:
	device_printf(sc->dev, "chipc region alloc failed for %s%u.%u\n",
	    bhnd_port_type_name(type), port, region);
	free(cr, M_BHND);
	return (NULL);
}

/**
 * Deallocate the given region record and its associated resource, if any.
 *
 * @param sc Driver instance state.
 * @param cr Region record to be deallocated.
 */
static void
chipc_free_region(struct chipc_softc *sc, struct chipc_region *cr)
{
	KASSERT(cr->cr_refs == 0,
	    ("chipc %s%u.%u region has %u active references",
	     bhnd_port_type_name(cr->cr_port_type), cr->cr_port_num,
	     cr->cr_region_num, cr->cr_refs));

	if (cr->cr_res != NULL) {
		bhnd_release_resource(sc->dev, SYS_RES_MEMORY, cr->cr_rid,
		    cr->cr_res);
	}

	free(cr, M_BHND);
}

/**
 * Locate the region mapping the given range, if any. Returns NULL if no
 * valid region is found.
 * 
 * @param sc Driver instance state.
 * @param start start of address range.
 * @param end end of address range.
 */
static struct chipc_region *
chipc_find_region(struct chipc_softc *sc, rman_res_t start, rman_res_t end)
{
	struct chipc_region *cr;

	if (start > end)
		return (NULL);

	STAILQ_FOREACH(cr, &sc->mem_regions, cr_link) {
		if (start < cr->cr_addr || end > cr->cr_end)
			continue;

		/* Found */
		return (cr);
	}

	/* Not found */
	return (NULL);
}

/**
 * Locate a region mapping by its bhnd-assigned resource id (as returned by
 * bhnd_get_port_rid).
 * 
 * @param sc Driver instance state.
 * @param rid Resource ID to query for.
 */
static struct chipc_region *
chipc_find_region_by_rid(struct chipc_softc *sc, int rid)
{
	struct chipc_region	*cr;
	int			 port_rid;

	STAILQ_FOREACH(cr, &sc->mem_regions, cr_link) {
		port_rid = bhnd_get_port_rid(sc->dev, cr->cr_port_type,
		    cr->cr_port_num, cr->cr_region_num);
		if (port_rid == -1 || port_rid != rid)
			continue;

		/* Found */
		return (cr);
	}

	/* Not found */
	return (NULL);
}

/* Retain a reference to the region, allocating/activating if necessary */
int
chipc_retain_region(struct chipc_softc *sc, struct chipc_region *cr, int flags)
{
	int error;

	KASSERT(!(flags &~ (RF_ACTIVE|RF_ALLOCATED)), ("unsupported flags"));

	CHIPC_LOCK(sc);

	/* Handle allocation */
	if (flags & RF_ALLOCATED) {
		/* If this is the first reference, allocate the resource */
		if (cr->cr_refs == 0) {
			KASSERT(cr->cr_res == NULL,
			    ("non-NULL resource has refcount"));

			cr->cr_res = bhnd_alloc_resource(sc->dev,
			    SYS_RES_MEMORY, &cr->cr_rid, cr->cr_addr,
			    cr->cr_end, cr->cr_count, 0);

			if (cr->cr_res == NULL) {
				CHIPC_UNLOCK(sc);
				return (ENXIO);
			}
		}
		
		/* Increment allocation refcount */
		cr->cr_refs++;
	}


	/* Handle activation */
	if (flags & RF_ACTIVE) {
		KASSERT(cr->cr_refs > 0,
		    ("cannot activate unallocated resource"));

		/* If this is the first reference, activate the resource */
		if (cr->cr_act_refs == 0) {
			error = bhnd_activate_resource(sc->dev, SYS_RES_MEMORY,
			    cr->cr_rid, cr->cr_res);
			if (error) {
				/* Drop any allocation reference acquired
				 * above */
				CHIPC_UNLOCK(sc);
				chipc_release_region(sc, cr,
				    flags &~ RF_ACTIVE);
				return (error);
			}
		}

		/* Increment activation refcount */
		cr->cr_act_refs++;
	}

	CHIPC_UNLOCK(sc);
	return (0);
}

int
chipc_release_region(struct chipc_softc *sc, struct chipc_region *cr,
    int flags)
{
	int	error;

	CHIPC_LOCK(sc);
	error = 0;

	if (flags & RF_ACTIVE) {
		KASSERT(cr->cr_act_refs > 0, ("RF_ACTIVE over-released"));
		KASSERT(cr->cr_act_refs <= cr->cr_refs,
		     ("RF_ALLOCATED released with RF_ACTIVE held"));

		/* If this is the last reference, deactivate the resource */
		if (cr->cr_act_refs == 1) {
			error = bhnd_deactivate_resource(sc->dev,
			    SYS_RES_MEMORY, cr->cr_rid, cr->cr_res);
			if (error)
				goto done;
		}

		/* Drop our activation refcount */
		cr->cr_act_refs--;
	}

	if (flags & RF_ALLOCATED) {
		KASSERT(cr->cr_refs > 0, ("overrelease of refs"));

		/* If this is the last reference, release the resource */
		if (cr->cr_refs == 1) {
			error = bhnd_release_resource(sc->dev,
			    SYS_RES_MEMORY, cr->cr_rid, cr->cr_res);
			if (error)
				goto done;

			cr->cr_res = NULL;
			cr->cr_rid = -1;
		}

		/* Drop our allocation refcount */
		cr->cr_refs--;
	}

done:
	CHIPC_UNLOCK(sc);
	return (error);
}

static struct resource *
chipc_alloc_resource(device_t dev, device_t child, int type,
    int *rid, rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct chipc_softc		*sc;
	struct chipc_region		*cr;
	struct resource_list_entry	*rle;
	struct resource			*rv;
	struct rman			*rm;
	int				 error;
	bool				 passthrough, isdefault;

	sc = device_get_softc(dev);
	passthrough = (device_get_parent(child) != dev);
	isdefault = RMAN_IS_DEFAULT_RANGE(start, end);
	rle = NULL;

	/* Fetch the resource manager, delegate request if necessary */
	rm = chipc_get_rman(sc, type);
	if (rm == NULL) {
		/* Requested resource type is delegated to our parent */
		rv = bus_generic_rl_alloc_resource(dev, child, type, rid,
		    start, end, count, flags);
		return (rv);
	}

	/* Populate defaults */
	if (!passthrough && isdefault) {
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

	/* Locate a mapping region */
	if ((cr = chipc_find_region(sc, start, end)) == NULL) {
		device_printf(dev,
		    "no mapping region found for %#x type %d for "
		    "child %s\n",
		    *rid, type, device_get_nameunit(child));

		return (NULL);
	}

	/* Try to retain a region reference */
	if ((error = chipc_retain_region(sc, cr, RF_ALLOCATED))) {
		CHIPC_UNLOCK(sc);
		return (NULL);
	}

	/* Make our rman reservation */
	rv = rman_reserve_resource(rm, start, end, count, flags & ~RF_ACTIVE,
	    child);
	if (rv == NULL) {
		chipc_release_region(sc, cr, RF_ALLOCATED);
		return (NULL);
	}

	rman_set_rid(rv, *rid);

	/* Activate */
	if (flags & RF_ACTIVE) {
		error = bus_activate_resource(child, type, *rid, rv);
		if (error) {
			device_printf(dev,
			    "failed to activate entry %#x type %d for "
				"child %s: %d\n",
			     *rid, type, device_get_nameunit(child), error);

			chipc_release_region(sc, cr, RF_ALLOCATED);
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
chipc_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	struct chipc_softc	*sc;
	struct chipc_region	*cr;
	struct rman		*rm;
	int			 error;

	sc = device_get_softc(dev);

	/* Handled by parent bus? */
	rm = chipc_get_rman(sc, type);
	if (rm == NULL || !rman_is_region_manager(r, rm)) {
		return (bus_generic_rl_release_resource(dev, child, type, rid,
		    r));
	}

	/* Locate the mapping region */
	cr = chipc_find_region(sc, rman_get_start(r), rman_get_end(r));
	if (cr == NULL)
		return (EINVAL);

	/* Deactivate resources */
	if (rman_get_flags(r) & RF_ACTIVE) {
		error = BUS_DEACTIVATE_RESOURCE(dev, child, type, rid, r);
		if (error)
			return (error);
	}

	if ((error = rman_release_resource(r)))
		return (error);

	/* Drop allocation reference */
	chipc_release_region(sc, cr, RF_ALLOCATED);

	return (0);
}

static int
chipc_adjust_resource(device_t dev, device_t child, int type,
    struct resource *r, rman_res_t start, rman_res_t end)
{
	struct chipc_softc		*sc;
	struct chipc_region		*cr;
	struct rman			*rm;
	
	sc = device_get_softc(dev);

	/* Handled by parent bus? */
	rm = chipc_get_rman(sc, type);
	if (rm == NULL || !rman_is_region_manager(r, rm)) {
		return (bus_generic_adjust_resource(dev, child, type, r, start,
		    end));
	}

	/* The range is limited to the existing region mapping */
	cr = chipc_find_region(sc, rman_get_start(r), rman_get_end(r));
	if (cr == NULL)
		return (EINVAL);
	
	if (end <= start)
		return (EINVAL);

	if (start < cr->cr_addr || end > cr->cr_end)
		return (EINVAL);

	/* Range falls within the existing region */
	return (rman_adjust_resource(r, start, end));
}

/**
 * Initialize child resource @p r with a virtual address, tag, and handle
 * copied from @p parent, adjusted to contain only the range defined by
 * @p offsize and @p size.
 * 
 * @param r The register to be initialized.
 * @param parent The parent bus resource that fully contains the subregion.
 * @param offset The subregion offset within @p parent.
 * @param size The subregion size.
 */
static int
chipc_init_child_resource(struct resource *r,
    struct resource *parent, bhnd_size_t offset, bhnd_size_t size)
{
	bus_space_handle_t	bh, child_bh;
	bus_space_tag_t		bt;
	uintptr_t		vaddr;
	int			error;

	/* Fetch the parent resource's bus values */
	vaddr = (uintptr_t) rman_get_virtual(parent);
	bt = rman_get_bustag(parent);
	bh = rman_get_bushandle(parent);

	/* Configure child resource with offset-adjusted values */
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
 * Retain an RF_ACTIVE reference to the region mapping @p r, and
 * configure @p r with its subregion values.
 *
 * @param sc Driver instance state.
 * @param child Requesting child device.
 * @param type resource type of @p r.
 * @param rid resource id of @p r
 * @param r resource to be activated.
 * @param req_direct If true, failure to allocate a direct bhnd resource
 * will be treated as an error. If false, the resource will not be marked
 * as RF_ACTIVE if bhnd direct resource allocation fails.
 */
static int
chipc_try_activate_resource(struct chipc_softc *sc, device_t child, int type,
    int rid, struct resource *r, bool req_direct)
{
	struct rman		*rm;
	struct chipc_region	*cr;
	bhnd_size_t		 cr_offset;
	rman_res_t		 r_start, r_end, r_size;
	int			 error;

	rm = chipc_get_rman(sc, type);
	if (rm == NULL || !rman_is_region_manager(r, rm))
		return (EINVAL);

	r_start = rman_get_start(r);
	r_end = rman_get_end(r);
	r_size = rman_get_size(r);

	/* Find the corresponding chipc region */
	cr = chipc_find_region(sc, r_start, r_end);
	if (cr == NULL)
		return (EINVAL);
	
	/* Calculate subregion offset within the chipc region */
	cr_offset = r_start - cr->cr_addr;

	/* Retain (and activate, if necessary) the chipc region */
	if ((error = chipc_retain_region(sc, cr, RF_ACTIVE)))
		return (error);

	/* Configure child resource with its subregion values. */
	if (cr->cr_res->direct) {
		error = chipc_init_child_resource(r, cr->cr_res->res,
		    cr_offset, r_size);
		if (error)
			goto cleanup;

		/* Mark active */
		if ((error = rman_activate_resource(r)))
			goto cleanup;
	} else if (req_direct) {
		error = ENOMEM;
		goto cleanup;
	}

	return (0);

cleanup:
	chipc_release_region(sc, cr, RF_ACTIVE);
	return (error);
}

static int
chipc_activate_bhnd_resource(device_t dev, device_t child, int type,
    int rid, struct bhnd_resource *r)
{
	struct chipc_softc	*sc;
	struct rman		*rm;
	int			 error;

	sc = device_get_softc(dev);
	
	/* Delegate non-locally managed resources to parent */
	rm = chipc_get_rman(sc, type);
	if (rm == NULL || !rman_is_region_manager(r->res, rm)) {
		return (bhnd_bus_generic_activate_resource(dev, child, type,
		    rid, r));
	}

	/* Try activating the chipc region resource */
	error = chipc_try_activate_resource(sc, child, type, rid, r->res,
	    false);
	if (error)
		return (error);

	/* Mark the child resource as direct according to the returned resource
	 * state */
	if (rman_get_flags(r->res) & RF_ACTIVE)
		r->direct = true;

	return (0);
}

static int
chipc_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	struct chipc_softc	*sc;
	struct rman		*rm;

	sc = device_get_softc(dev);

	/* Delegate non-locally managed resources to parent */
	rm = chipc_get_rman(sc, type);
	if (rm == NULL || !rman_is_region_manager(r, rm)) {
		return (bus_generic_activate_resource(dev, child, type, rid,
		    r));
	}

	/* Try activating the chipc region-based resource */
	return (chipc_try_activate_resource(sc, child, type, rid, r, true));
}

/**
 * Default bhndb(4) implementation of BUS_DEACTIVATE_RESOURCE().
 */
static int
chipc_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	struct chipc_softc	*sc;
	struct chipc_region	*cr;
	struct rman		*rm;
	int			 error;

	sc = device_get_softc(dev);

	/* Handled by parent bus? */
	rm = chipc_get_rman(sc, type);
	if (rm == NULL || !rman_is_region_manager(r, rm)) {
		return (bus_generic_deactivate_resource(dev, child, type, rid,
		    r));
	}

	/* Find the corresponding chipc region */
	cr = chipc_find_region(sc, rman_get_start(r), rman_get_end(r));
	if (cr == NULL)
		return (EINVAL);

	/* Mark inactive */
	if ((error = rman_deactivate_resource(r)))
		return (error);

	/* Drop associated RF_ACTIVE reference */
	chipc_release_region(sc, cr, RF_ACTIVE);

	return (0);
}

/**
 * If supported, add an appropriate NVRAM child device.
 */
static int
chipc_nvram_attach(struct chipc_softc *sc)
{
	device_t	 nvram_dev;
	rman_res_t	 start;
	int		 error;

	switch (sc->nvram_src) {
	case BHND_NVRAM_SRC_OTP:
	case BHND_NVRAM_SRC_SPROM:
		/* Add OTP/SPROM device */
		nvram_dev = BUS_ADD_CHILD(sc->dev, 0, "bhnd_nvram", -1);
		if (nvram_dev == NULL) {
			device_printf(sc->dev, "failed to add NVRAM device\n");
			return (ENXIO);
		}

		start = rman_get_start(sc->core->res) + CHIPC_SPROM_OTP;
		error = bus_set_resource(nvram_dev, SYS_RES_MEMORY, 0, start,
		    CHIPC_SPROM_OTP_SIZE);
		return (error);

	case BHND_NVRAM_SRC_NFLASH:
		// TODO (requires access to NFLASH hardware)
		device_printf(sc->dev, "NVRAM-NFLASH unsupported\n");
		return (0);

	case BHND_NVRAM_SRC_UNKNOWN:
		/* Handled externally */
		return (0);

	default:
		device_printf(sc->dev, "invalid nvram source: %u\n",
		     sc->nvram_src);
		return (ENXIO);
	}
}

/**
 * Determine the NVRAM data source for this device.
 *
 * @param sc chipc driver state.
 */
static bhnd_nvram_src_t
chipc_nvram_identify(struct chipc_softc *sc)
{
	uint32_t		 srom_ctrl;

	/* Very early devices vend SPROM/OTP/CIS (if at all) via the
	 * host bridge interface instead of ChipCommon. */
	if (!CHIPC_QUIRK(sc, SUPPORTS_SPROM))
		return (BHND_NVRAM_SRC_UNKNOWN);

	/*
	 * Later chipset revisions standardized the SPROM capability flags and
	 * register interfaces.
	 * 
	 * We check for hardware presence in order of precedence. For example,
	 * SPROM is is always used in preference to internal OTP if found.
	 */
	if (CHIPC_CAP(sc, CAP_SPROM)) {
		srom_ctrl = bhnd_bus_read_4(sc->core, CHIPC_SPROM_CTRL);
		if (srom_ctrl & CHIPC_SRC_PRESENT)
			return (BHND_NVRAM_SRC_SPROM);
	}

	/* Check for OTP */
	if (CHIPC_CAP(sc, CAP_OTP_SIZE))
		return (BHND_NVRAM_SRC_OTP);

	/*
	 * Finally, Northstar chipsets (and possibly other chipsets?) support
	 * external NAND flash. 
	 */
	if (CHIPC_QUIRK(sc, SUPPORTS_NFLASH) && CHIPC_CAP(sc, CAP_NFLASH))
		return (BHND_NVRAM_SRC_NFLASH);

	/* No NVRAM hardware capability declared */
	return (BHND_NVRAM_SRC_UNKNOWN);
}

/**
 * Examine bus state and make a best effort determination of whether it's
 * likely safe to enable the muxed SPROM pins.
 * 
 * On devices that do not use SPROM pin muxing, always returns true.
 * 
 * @param sc chipc driver state.
 */
static bool
chipc_should_enable_sprom(struct chipc_softc *sc)
{
	device_t	*devs;
	device_t	 hostb;
	device_t	 parent;
	int		 devcount;
	int		 error;
	bool		 result;

	mtx_assert(&Giant, MA_OWNED);	/* for newbus */

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return (true);

	parent = device_get_parent(sc->dev);
	hostb = bhnd_find_hostb_device(parent);

	if ((error = device_get_children(parent, &devs, &devcount)))
		return (false);

	/* Reject any active devices other than ChipCommon, or the
	 * host bridge (if any). */
	result = true;
	for (int i = 0; i < devcount; i++) {
		if (devs[i] == hostb || devs[i] == sc->dev)
			continue;

		if (!device_is_attached(devs[i]))
			continue;

		if (device_is_suspended(devs[i]))
			continue;

		/* Active device; assume SPROM is busy */
		result = false;
		break;
	}

	free(devs, M_TEMP);
	return (result);
}

/**
 * If required by this device, enable access to the SPROM.
 * 
 * @param sc chipc driver state.
 */
static int
chipc_enable_sprom_pins(device_t dev)
{
	struct chipc_softc	*sc;
	uint32_t		 cctrl;
	int			 error;

	sc = device_get_softc(dev);

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return (0);

	/* Make sure we're holding Giant for newbus */
	mtx_lock(&Giant);
	CHIPC_LOCK(sc);

	/* Already enabled? */
	if (sc->sprom_refcnt >= 1) {
		error = 0;
		goto finished;
	}

	/* Check whether bus is busy */
	if (!chipc_should_enable_sprom(sc)) {
		error = EBUSY;
		goto finished;
	}

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);

	/* 4331 devices */
	if (CHIPC_QUIRK(sc, 4331_EXTPA_MUX_SPROM)) {
		cctrl &= ~CHIPC_CCTRL4331_EXTPA_EN;

		if (CHIPC_QUIRK(sc, 4331_GPIO2_5_MUX_SPROM))
			cctrl &= ~CHIPC_CCTRL4331_EXTPA_ON_GPIO2_5;

		if (CHIPC_QUIRK(sc, 4331_EXTPA2_MUX_SPROM))
			cctrl &= ~CHIPC_CCTRL4331_EXTPA_EN2;

		bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);
		error = 0;
		goto finished;
	}

	/* 4360 devices */
	if (CHIPC_QUIRK(sc, 4360_FEM_MUX_SPROM)) {
		/* Unimplemented */
	}

	/* Refuse to proceed on unsupported devices with muxed SPROM pins */
	device_printf(sc->dev, "muxed sprom lines on unrecognized device\n");
	error = ENXIO;

finished:
	/* Bump the reference count */
	if (error == 0)
		sc->sprom_refcnt++;

	CHIPC_UNLOCK(sc);
	mtx_unlock(&Giant);

	return (error);
}

/**
 * If required by this device, revert any GPIO/pin configuration applied
 * to allow SPROM access.
 * 
 * @param sc chipc driver state.
 */
static void
chipc_disable_sprom_pins(device_t dev)
{
	struct chipc_softc	*sc;
	uint32_t		 cctrl;

	sc = device_get_softc(dev);

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return;

	CHIPC_LOCK(sc);

	/* Check reference count, skip disable if in-use. */
	KASSERT(sc->sprom_refcnt > 0, ("sprom refcnt overrelease"));
	sc->sprom_refcnt--;
	if (sc->sprom_refcnt > 0)
		goto finished;

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);

	/* 4331 devices */
	if (CHIPC_QUIRK(sc, 4331_EXTPA_MUX_SPROM)) {
		cctrl |= CHIPC_CCTRL4331_EXTPA_EN;

		if (CHIPC_QUIRK(sc, 4331_GPIO2_5_MUX_SPROM))
			cctrl |= CHIPC_CCTRL4331_EXTPA_ON_GPIO2_5;

		if (CHIPC_QUIRK(sc, 4331_EXTPA2_MUX_SPROM))
			cctrl |= CHIPC_CCTRL4331_EXTPA_EN2;

		bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);
		goto finished;
	}

	/* 4360 devices */
	if (CHIPC_QUIRK(sc, 4360_FEM_MUX_SPROM)) {
		/* Unimplemented */
	}

finished:
	CHIPC_UNLOCK(sc);
}

static bhnd_nvram_src_t
chipc_nvram_src(device_t dev)
{
	struct chipc_softc *sc = device_get_softc(dev);
	return (sc->nvram_src);
}

static void
chipc_write_chipctrl(device_t dev, uint32_t value, uint32_t mask)
{
	struct chipc_softc	*sc;
	uint32_t		 cctrl;

	sc = device_get_softc(dev);

	CHIPC_LOCK(sc);

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);
	cctrl = (cctrl & ~mask) | (value | mask);
	bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);

	CHIPC_UNLOCK(sc);
}

static device_method_t chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			chipc_probe),
	DEVMETHOD(device_attach,		chipc_attach),
	DEVMETHOD(device_detach,		chipc_detach),
	DEVMETHOD(device_suspend,		chipc_suspend),
	DEVMETHOD(device_resume,		chipc_resume),

	/* Bus interface */
	DEVMETHOD(bus_probe_nomatch,		chipc_probe_nomatch),
	DEVMETHOD(bus_print_child,		chipc_print_child),
	DEVMETHOD(bus_child_pnpinfo_str,	chipc_child_pnpinfo_str),
	DEVMETHOD(bus_child_location_str,	chipc_child_location_str),

	DEVMETHOD(bus_add_child,		chipc_add_child),
	DEVMETHOD(bus_child_deleted,		chipc_child_deleted),

	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_alloc_resource,		chipc_alloc_resource),
	DEVMETHOD(bus_release_resource,		chipc_release_resource),
	DEVMETHOD(bus_adjust_resource,		chipc_adjust_resource),
	DEVMETHOD(bus_activate_resource,	chipc_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	chipc_deactivate_resource),
	DEVMETHOD(bus_get_resource_list,	chipc_get_resource_list),

	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),
	DEVMETHOD(bus_config_intr,		bus_generic_config_intr),
	DEVMETHOD(bus_bind_intr,		bus_generic_bind_intr),
	DEVMETHOD(bus_describe_intr,		bus_generic_describe_intr),

	/* BHND bus inteface */
	DEVMETHOD(bhnd_bus_activate_resource,	chipc_activate_bhnd_resource),

	/* ChipCommon interface */
	DEVMETHOD(bhnd_chipc_nvram_src,		chipc_nvram_src),
	DEVMETHOD(bhnd_chipc_write_chipctrl,	chipc_write_chipctrl),
	DEVMETHOD(bhnd_chipc_enable_sprom,	chipc_enable_sprom_pins),
	DEVMETHOD(bhnd_chipc_disable_sprom,	chipc_disable_sprom_pins),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_chipc, chipc_driver, chipc_methods, sizeof(struct chipc_softc));
DRIVER_MODULE(bhnd_chipc, bhnd, chipc_driver, bhnd_chipc_devclass, 0, 0);
MODULE_DEPEND(bhnd_chipc, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_chipc, 1);
