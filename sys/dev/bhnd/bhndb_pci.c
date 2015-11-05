 
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

#include "bhnd.h"
#include "bhndvar.h"
#include "bhndreg.h"

#include "bhndb_pcivar.h"
#include "bhnd_pcireg.h"

devclass_t bhndb_devclass;

static struct resource_spec bhndb_pci_res_spec[] = {
	{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
	{ SYS_RES_MEMORY,	PCIR_BAR(1),	RF_ACTIVE },
	{ -1,			0,		0 }
};
#define bhndb_pci_res_count \
	(sizeof(bhndb_pci_res_spec) / sizeof(bhndb_pci_res_spec[0]))

int
bhndb_pci_generic_probe(device_t dev)
{
	return (BUS_PROBE_NOWILDCARD);
}

int
bhndb_pci_generic_attach(device_t dev)
{
	struct bhndb_pci_softc	*sc;
	int			 error;
	bool			 free_mem_rman = false;
	bool			 free_pci_res = false;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->pci_dev = device_get_parent(dev);
	
	/* Allocate our resource table */
	sc->pci_res = malloc(sizeof(struct resource) * bhndb_pci_res_count,
	    M_BHND, M_WAITOK);
	if (sc->pci_res == NULL)
		return (ENOMEM);
	

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

	/* Map our PCI device resources. */
	error = bus_alloc_resources(sc->pci_dev, bhndb_pci_res_spec, sc->pci_res);
	if (error) {
		device_printf(dev, "could not allocate PCI resources on %s\n",
		device_get_nameunit(sc->pci_dev));
		goto failed;
	} else {
		free_pci_res = true;
	}

	return (bus_generic_attach(dev));

failed:
	if (free_mem_rman)
		rman_fini(&sc->mem_rman);

	if (free_pci_res)
		bus_release_resources(dev, bhndb_pci_res_spec, sc->pci_res);
	
	free(sc->pci_res, M_BHND);
	return (error);
}

int
bhndb_pci_generic_detach(device_t dev)
{
	struct bhndb_pci_softc *sc = device_get_softc(dev);

	bus_release_resources(dev, bhndb_pci_res_spec, sc->pci_res);
	free(sc->pci_res, M_BHND);

	return (bus_generic_detach(dev));
}

int
bhndb_pci_generic_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

int
bhndb_pci_generic_resume(device_t dev)
{
	return (bus_generic_resume(dev));
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

/**
 * Helper function for implementing BUS_ALLOC_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BHNDBUS_GET_RMAN() and BUS_GET_RESOURCE_LIST()
 * to fetch resource state for allocation.
 */
struct resource *
bhndb_pci_generic_alloc_resource(device_t dev, device_t child, int type,
    int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	struct rman			*rm;
	int				 error;

	if (device_get_parent(child) != dev) {
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), child,
		    type, rid, start, end, count, flags));
	}

	/* Fetch the resource manager */
	rm = bhndb_pci_get_rman(dev, type);
	if (rm == NULL)
		return (NULL);

	/* Fetch the resource list entry */
	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl == NULL)
		return (NULL);

	rle = resource_list_find(rl, type, *rid);
	if (rle == NULL)
		return (NULL);

	/* Validate the resource addresses */
	if (start == 0ULL && end == ~0ULL && count <= rle->count) {
		start = rle->start;
		end = rle->end;
		count = rle->count;
	} else {
		if (start < rle->start || start > end)
			return NULL;
		
		if (end < start || end > rle->end)
			return NULL;
		
		if (count > end - start)
			return NULL;
	}

	/* Check for existing allocation */
	if (rle->res != NULL) {
		device_printf(dev,
		    "resource entry %#x type %d for child %s is busy\n", *rid,
			type, device_get_nameunit(child));
		return (NULL);
	}

	/* Make our reservation */
	rle->res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (rle->res == NULL)
		return (NULL);

	rman_set_rid(rle->res, *rid);

	/* Activate */
	if (flags & RF_ACTIVE) {
		error = bus_activate_resource(child, type, *rid, rle->res);
		if (error) {
			device_printf(dev,
			    "failed to activate entry %#x type %d for "
				"child %s\n",
			     *rid, type, device_get_nameunit(child));

			rman_release_resource(rle->res);
			rle->res = NULL;
			return (NULL);
		}
	}

	return (rle->res);
}

/**
 * Helper function for implementing BUS_RELEASE_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BUS_GET_RESOURCE_LIST() to fetch resource
 * state.
 */
int
bhndb_pci_generic_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *res)
{
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	int				 error;

	if (device_get_parent(child) != dev) {
		return (BUS_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, res));
	}

	/* Fetch the resource list entry */
	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl == NULL)
		return (EINVAL);

	rle = resource_list_find(rl, type, rid);
	if (rle == NULL)
		panic("missing resource list entry");

	if (rle->res != res)
		panic("resource list entry does not match resource");

	/* Release the resource */
	if ((error = rman_release_resource(res)))
		return (error);

	rle->res = NULL;
	return (0);
}

/**
 * Helper function for implementing BUS_ACTIVATE_RESOURCE() on bhnd pci hosts.
 */
int
bhndb_pci_generic_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	// TODO - window resource activations
	return (EINVAL);
}

/**
 * Helper function for implementing BUS_ACTIVATE_RESOURCE() on bhnd pci hosts.
 */
int
bhndb_pci_generic_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	// TODO - window resource deactivations
	return (EINVAL);
}

/**
 * Helper function for implementing BHNDBUS_ALLOC_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_ALLOC_RESOURCE().
 */
struct bhnd_resource *
bhndb_pci_generic_alloc_bhnd_resource(device_t dev, device_t child, int type,
     int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	// struct bhnd_resource *r;
	
	if (device_get_parent(child) != dev)
		return (BHNDBUS_ALLOC_RESOURCE(device_get_parent(dev), child,
		    type, rid, start, end, count, flags));

	// TODO
	return (NULL);
}

/**
 * Helper function for implementing BHNDBUS_RELEASE_RESOURCE().
 */
int
bhndb_pci_generic_release_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	// int error;
	
	if (device_get_parent(child) != dev)
		return (BHNDBUS_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	// TODO
	return (EOPNOTSUPP);
}

/**
 * Helper function for implementing BHNDBUS_ACTIVATE_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_ACTIVATE_RESOURCE().
 */
int
bhndb_pci_generic_activate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	if (device_get_parent(child) != dev)
		return (BHNDBUS_ACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	// TODO
	return (EOPNOTSUPP);
};

/**
 * Helper function for implementing BHNDBUS_DEACTIVATE_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_DEACTIVATE_RESOURCE().
 */
int
bhndb_pci_generic_deactivate_bhnd_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	if (device_get_parent(child) != dev)
		return (BHNDBUS_DEACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	// TODO
	return (EOPNOTSUPP);
};