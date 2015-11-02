 
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

#include "bhnd_pci.h"
#include "bhnd_pcireg.h"

int
bhnd_pci_attach(device_t dev)
{
	// TODO
	return (ENXIO);
}

int
bhnd_pci_detach(device_t dev)
{
	// TODO
	return (ENXIO);
}

int
bhnd_pci_suspend(device_t dev)
{
	// TODO
	return (ENXIO);
}

int
bhnd_pci_resume(device_t dev)
{
	// TODO
	return (ENXIO);
}

/**
 * Helper function for implementing BUS_ALLOC_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BHNDBUS_GET_RMAN() and BUS_GET_RESOURCE_LIST()
 * to fetch resource state for allocation.
 */
struct resource *
bhnd_pci_alloc_resource(device_t dev, device_t child, int type,
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
	rm = BHNDBUS_GET_RMAN(dev, type);
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
 * Helper function for implementing BUS_SET_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BUS_GET_RESOURCE_LIST() to fetch resource
 * state.
 */
int
bhnd_pci_set_resource(device_t dev, device_t child, int type, int rid,
    u_long start, u_long count)
{
	return bus_generic_rl_set_resource(dev, child, type, rid, start, count);
}

/**
 * Helper function for implementing BUS_GET_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BUS_GET_RESOURCE_LIST() to fetch resource
 * state.
 */
int
bhnd_pci_get_resource(device_t dev, device_t child, int type, int rid,
    u_long *startp, u_long *countp)
{
	return bus_generic_rl_get_resource(dev, child, type, rid, startp,
	    countp);
}

/**
 * Helper function for implementing BUS_DELETE_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BUS_GET_RESOURCE_LIST() to fetch resource
 * state.
 */
void
bhnd_pci_delete_resource(device_t dev, device_t child, int type, int rid)
{
	return bus_generic_rl_delete_resource(dev, child, type, rid);
}

/**
 * Helper function for implementing BUS_RELEASE_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BUS_GET_RESOURCE_LIST() to fetch resource
 * state.
 */
int
bhnd_pci_release_resource(device_t dev, device_t child, int type, int rid,
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
 * Helper function for implementing BUS_ADJUST_RESOURCE() on bhnd pci hosts.
 * 
 * This simple implementation uses BUS_GET_RESOURCE_LIST() to fetch resource
 * state.
 */
int
bhnd_pci_adjust_resource(device_t dev, device_t child, int type,
    struct resource *res, u_long start, u_long end)
{
	if (device_get_parent(child) != dev) {
		return (BUS_ADJUST_RESOURCE(device_get_parent(dev), child,
		    type, res, start, end));
	}

	return (rman_adjust_resource(res, start, end));
}

/**
 * Helper function for implementing BUS_ACTIVATE_RESOURCE() on bhnd pci hosts.
 */
int
bhnd_pci_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	// TODO - window resource activations
	return (EINVAL);
}

/**
 * Helper function for implementing BUS_ACTIVATE_RESOURCE() on bhnd pci hosts.
 */
int
bhnd_pci_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	// TODO - window resource deactivations
	return (EINVAL);
}
