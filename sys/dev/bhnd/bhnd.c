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

/**
 * bhnd_generic_probe_nomatch() reporting configuration.
 */
static const struct bhnd_nomatch {
	uint16_t	vendor;		/**< core designer */
	uint16_t	device;		/**< core id */
	bool		if_verbose;	/**< print when bootverbose is set. */
} bhnd_nomatch_table[] = {
	{ BHND_MFGID_ARM,	BHND_COREID_OOB_ROUTER,		true	},
	{ BHND_MFGID_ARM,	BHND_COREID_EROM,		true	},
	{ BHND_MFGID_ARM,	BHND_COREID_PL301,		true	},
	{ BHND_MFGID_ARM,	BHND_COREID_APB_BRIDGE,		true	},
	{ BHND_MFGID_ARM,	BHND_COREID_AXI_UNMAPPED,	false	},

	{ BHND_MFGID_INVALID,	BHND_COREID_INVALID,		false	}
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
	struct resource_list		*rl;
	const struct bhnd_nomatch	*nm;
	bool				 report;

	/* Fetch reporting configuration for this device */
	report = true;
	for (nm = bhnd_nomatch_table; nm->device != BHND_COREID_INVALID; nm++) {
		if (nm->vendor != bhnd_get_vendor(child))
			continue;

		if (nm->device != bhnd_get_device(child))
			continue;

		report = false;
		if (bootverbose && nm->if_verbose)
			report = true;
		break;
	}
	
	if (!report)
		return;

	/* Print the non-matched device info */
	device_printf(dev, "<%s %s>", bhnd_get_vendor_name(child),
		bhnd_get_device_name(child));

	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl != NULL)
		resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#lx");

	printf(" at core %u (no driver attached)\n", bhnd_get_core_index(child));
}

/**
 * Default implementation of BUS_CHILD_PNPINFO_STR().
 */
static int
bhnd_child_pnpinfo_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	if (device_get_parent(child) != dev) {
		return (BUS_CHILD_PNPINFO_STR(device_get_parent(dev), child,
		    buf, buflen));
	}

	snprintf(buf, buflen, "vendor=0x%hx device=0x%hx rev=0x%hhx",
	    bhnd_get_vendor(child), bhnd_get_device(child),
	    bhnd_get_hwrev(child));

	return (0);
}

/**
 * Default implementation of implementing BUS_PRINT_CHILD().
 */
static int
bhnd_child_location_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	bhnd_addr_t	addr;
	bhnd_size_t	size;
	
	if (device_get_parent(child) != dev) {
		return (BUS_CHILD_LOCATION_STR(device_get_parent(dev), child,
		    buf, buflen));
	}


	if (bhnd_get_port_addr(child, 0, 0, &addr, &size)) {
		/* No device default port/region */
		if (buflen > 0)
			*buf = '\0';
		return (0);
	}

	snprintf(buf, buflen, "port0.0=0x%llx", (unsigned long long) addr);
	return (0);
}

/**
 * Helper function for implementing BHND_IS_HOSTB_DEVICE().
 * 
 * If a parent device is available, this implementation delegates the
 * request to the BHND_IS_HOSTB_DEVICE() method on the parent of @p dev.
 * 
 * If no parent device is available (i.e. on a the bus root), false
 * is returned.
 */
bool
bhnd_generic_is_hostb_device(device_t dev, device_t child) {
	if (device_get_parent(dev) != NULL)
		return (BHND_IS_HOSTB_DEVICE(device_get_parent(dev), child));

	return (false);
}

/**
 * Helper function for implementing BHND_IS_HW_DISABLED().
 * 
 * If a parent device is available, this implementation delegates the
 * request to the BHND_IS_HW_DISABLED() method on the parent of @p dev.
 * 
 * If no parent device is available (i.e. on a the bus root), the hardware
 * is assumed to be usable and false is returned.
 */
bool
bhnd_generic_is_hw_disabled(device_t dev, device_t child)
{
	if (device_get_parent(dev) != NULL)
		return (BHND_IS_HW_DISABLED(device_get_parent(dev), child));

	return (false);
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
	isdefault = (start == 0UL && end == ~0UL);

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
	r->direct = true;
	r->res = BUS_ALLOC_RESOURCE(dev, child, type, rid, start, end,
	    count, flags);

	if (r->res == NULL) {
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
	if (!r->direct)
		panic("bhnd indirect resource released without bhnd parent bus");

	error = BUS_RELEASE_RESOURCE(dev, child, type, rid, r->res);
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
	if (!r->direct)
		panic("bhnd indirect resource activated without bhnd parent bus");

	return (BUS_ACTIVATE_RESOURCE(dev, child, type, rid, r->res));
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
	if (!r->direct)
		panic("bhnd indirect resource deactivated without bhnd parent bus");

	return (BUS_DEACTIVATE_RESOURCE(dev, child, type, rid, r->res));
};

static device_method_t bhnd_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_attach,		bus_generic_attach),
	DEVMETHOD(device_detach,		bus_generic_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bus_generic_suspend),
	DEVMETHOD(device_resume,		bus_generic_resume),

	/* Bus interface */
	DEVMETHOD(bus_probe_nomatch,		bhnd_generic_probe_nomatch),
	DEVMETHOD(bus_print_child,		bhnd_generic_print_child),
	DEVMETHOD(bus_child_pnpinfo_str,	bhnd_child_pnpinfo_str),
	DEVMETHOD(bus_child_location_str,	bhnd_child_location_str),

	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_alloc_resource,		bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_adjust_resource,		bus_generic_adjust_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_rl_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),

	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),
	DEVMETHOD(bus_config_intr,		bus_generic_config_intr),
	DEVMETHOD(bus_bind_intr,		bus_generic_bind_intr),
	DEVMETHOD(bus_describe_intr,		bus_generic_describe_intr),

	DEVMETHOD(bus_get_dma_tag,		bus_generic_get_dma_tag),

	/* BHND interface */
	DEVMETHOD(bhnd_alloc_resource,		bhnd_generic_alloc_bhnd_resource),
	DEVMETHOD(bhnd_release_resource,	bhnd_generic_release_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhnd_generic_activate_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhnd_generic_deactivate_bhnd_resource),

	DEVMETHOD_END
};

devclass_t bhnd_devclass;

DEFINE_CLASS_0(bhnd, bhnd_driver, bhnd_methods, sizeof(struct bhnd_softc));
MODULE_VERSION(bhnd, 1);
