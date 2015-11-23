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
 * Broadcom Home Networking Division (HND) Bus Driver.
 * 
 * The Broadcom HND family of devices consists of both SoCs and host-connected
 * networking chipsets containing a common family of Broadcom IP cores,
 * including an integrated MIPS and/or ARM cores.
 * 
 * HND devices expose a nearly identical interface whether accessible over a 
 * native SoC interconnect, or when connected via a host interface such as 
 * PCIe. As a result, the majority of hardware support code should be re-usable 
 * across host drivers for HND networking chipsets, as well as FreeBSD support 
 * for Broadcom MIPS/ARM HND SoCs.
 * 
 * Earlier HND models used the siba(4) on-chip interconnect, while later models
 * use bcma(4); the programming model is almost entirely independent
 * of the actual underlying interconect.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include "bhnd.h"
#include "bhndvar.h"

MALLOC_DEFINE(M_BHND, "bhnd", "bhnd bus data structures");
MODULE_VERSION(bhnd, 1);

/**
 * bhnd_generic_probe_nomatch() reporting configuration.
 */
static const struct bhnd_nomatch {
	uint16_t	vendor;		/**< core designer */
	uint16_t	device;		/**< core id */
	bool		report;		/**< always (true) or only
					     bootverbose (false) */
} bhnd_nomatch_table[] = {
	{ BHND_MFGID_ARM,	BHND_COREID_AXI_UNMAPPED,	false },
	{ 0,			BHND_COREID_INVALID,		false }
};

/**
 * Helper function for implementing BUS_PRINT_CHILD().
 * 
 * This implementation requests the device's struct resource_list via
 * BUS_GET_RESOURCE_LIST.
 */
int
bhnd_generic_print_child(device_t dev, device_t child)
{
	struct resource_list	*rl;
	int			retval = 0;

	retval += bus_print_child_header(dev, child);

	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl != NULL)
		retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#lx");

	retval += printf(" at core %u", bhnd_get_core_index(child));

	retval += bus_print_child_domain(dev, child);
	retval += bus_print_child_footer(dev, child);

	return (retval);
}

/**
 * Helper function for implementing BUS_PRINT_CHILD().
 * 
 * This implementation requests the device's struct resource_list via
 * BUS_GET_RESOURCE_LIST.
 */
void
bhnd_generic_probe_nomatch(device_t dev, device_t child)
{
	const struct bhnd_nomatch	*nm;
	struct resource_list		*rl;
	bool				report;

	/* Fetch reporting configuration for this device */
	report = true;
	for (nm = bhnd_nomatch_table; nm->device != BHND_COREID_INVALID; nm++) {
		if (nm->vendor == bhnd_get_vendor(child) &&
		    nm->device == bhnd_get_device(child))
		{
			report = nm->report;
		}
	}

	/* Print the non-matched device info */
	if (report || bootverbose) {
		device_printf(dev, "<%s %s>", bhnd_get_vendor_name(child),
		    bhnd_get_device_name(child));

		rl = BUS_GET_RESOURCE_LIST(dev, child);
		if (rl != NULL)
			resource_list_print_type(rl, "mem", SYS_RES_MEMORY,
			    "%#lx");

		printf(" at core %u (no driver attached)\n",
		    bhnd_get_core_index(child));
	}
}

/**
 * Helper function for implementing BHND_ALLOC_RESOURCE().
 * 
 * This simple implementation of BHND_ALLOC_RESOURCE() determines
 * any default values via BUS_GET_RESOURCE_LIST(), and calls
 * BHND_ALLOC_RESOURCE() method of the parent of @p dev.
 * 
 * If no parent device is available, the request is instead delegated to
 * BUS_ALLOC_RESOURCE().
 */
struct bhnd_resource *
bhnd_generic_alloc_bhnd_resource(device_t dev, device_t child, int type,
	int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct bhnd_resource		*r;
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	bool				 isdefault;
	bool				 passthrough;

	passthrough = (device_get_parent(child) != dev);
	isdefault = (start == 0ULL && end == ~0ULL);

	/* Determine locally-known defaults before delegating the request. */
	if (!passthrough && isdefault) {
		/* fetch resource list from child's bus */
		rl = BUS_GET_RESOURCE_LIST(dev, child);
		if (rl == NULL)
			return (NULL); /* no resource list */

		/* look for matching type/rid pair */
		rle = resource_list_find(BUS_GET_RESOURCE_LIST(dev, child),
		    type, *rid);
		if (rle == NULL)
			return (NULL);

		/* set default values */
		start = rle->start;
		end = rle->end;
		count = ulmax(count, rle->count);
	}

	/* Try to delegate to our parent. */
	if (device_get_parent(dev) != NULL) {
		return (BHND_ALLOC_RESOURCE(device_get_parent(dev), child, type,
		    rid, start, end, count, flags));
	}

	/* If this is the bus root, use a real bus-allocated resource */
	r = malloc(sizeof(struct bhnd_resource), M_BHND, M_WAITOK);
	if (r == NULL)
		return NULL;

	/* Allocate the bus resource, marking it as 'direct' (not requiring
	 * any bus window remapping to perform I/O) */
	r->_direct = true;
	r->_res = BUS_ALLOC_RESOURCE(dev, child, type, rid, start, end,
	    count, flags);

	if (r->_res == NULL) {
		free(r, M_BHND);
		return NULL;
	}

	return (r);
}

/**
 * Helper function for implementing BHND_RELEASE_RESOURCE().
 * 
 * This simple implementation of BHND_RELEASE_RESOURCE() simply calls the
 * BHND_RELEASE_RESOURCE() method of the parent of @p dev.
 * 
 * If no parent device is available, the request is delegated to
 * BUS_RELEASE_RESOURCE().
 */
int
bhnd_generic_release_bhnd_resource(device_t dev, device_t child, int type,
    int rid, struct bhnd_resource *r)
{
	int error;

	/* Try to delegate to the parent. */
	if (device_get_parent(dev) != NULL)
		return (BHND_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	/* Release the resource directly */
	if (!r->_direct)
		panic("bhnd indirect resource released without bhnd parent bus");

	error = BUS_RELEASE_RESOURCE(dev, child, type, rid, r->_res);
	if (error)
		return (error);

	free(r, M_BHND);
	return (0);
}

/**
 * Helper function for implementing BHND_ACTIVATE_RESOURCE().
 * 
 * This simple implementation of BHND_ACTIVATE_RESOURCE() simply calls the
 * BHND_ACTIVATE_RESOURCE() method of the parent of @p dev.
 * 
 * If no parent device is available, the request is delegated to
 * BUS_ACTIVATE_RESOURCE().
 */
int
bhnd_generic_activate_bhnd_resource(device_t dev, device_t child, int type,
	int rid, struct bhnd_resource *r)
{
	/* Try to delegate to the parent */
	if (device_get_parent(dev) != NULL)
		return (BHND_ACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	/* Activate the resource directly */
	if (!r->_direct)
		panic("bhnd indirect resource activated without bhnd parent bus");

	return (BUS_ACTIVATE_RESOURCE(dev, child, type, rid, r->_res));
};

/**
 * Helper function for implementing BHND_DEACTIVATE_RESOURCE().
 * 
 * This simple implementation of BHND_ACTIVATE_RESOURCE() simply calls the
 * BHND_ACTIVATE_RESOURCE() method of the parent of @p dev.
 * 
 * If no parent device is available, the request is delegated to
 * BUS_DEACTIVATE_RESOURCE().
 */
int
bhnd_generic_deactivate_bhnd_resource(device_t dev, device_t child, int type,
	int rid, struct bhnd_resource *r)
{
	if (device_get_parent(dev) != NULL)
		return (BHND_DEACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	/* De-activate the resource directly */
	if (!r->_direct)
		panic("bhnd indirect resource deactivated without bhnd parent bus");

	return (BUS_DEACTIVATE_RESOURCE(dev, child, type, rid, r->_res));
};
