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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_BHNDVAR_H_
#define _BHND_BHNDVAR_H_

#include <sys/types.h>
#include <sys/malloc.h>

struct bhnd_resource;
struct bhnd_probecfg;

#include "bhnd_if.h"

MALLOC_DECLARE(M_BHND);


/*
 * Device Identification
 */

/** BHND Device Classes. */
typedef enum {
	BHND_DEVCLASS_CC,	/**< chipcommon i/o controller */
	BHND_DEVCLASS_PCI,	/**< pci/pcie host/device bridge */
	BHND_DEVCLASS_MEM,	/**< internal RAM/SRAM */
	BHND_DEVCLASS_MEMC,	/**< memory controller */
	BHND_DEVCLASS_MAC,	/**< 802.3 MAC */
	BHND_DEVCLASS_PHY,	/**< 802.3 PHY */
	BHND_DEVCLASS_MPHY,	/**< 802.3 MAC/PHY */
	BHND_DEVCLASS_MAC_W,	/**< 802.11 MAC */
	BHND_DEVCLASS_PHY_W,	/**< 802.11 PHY */
	BHND_DEVCLASS_MPHY_W,	/**< 802.11 mac/phy */
	BHND_DEVCLASS_CPU,	/**< cpu core */
	BHND_DEVCLASS_SOCI,	/**< interconnect */
	BHND_DEVCLASS_SOCB,	/**< interconnect bridge/socket */
	BHND_DEVCLASS_OTHER,	/**< other / unknown */

	BHND_DEVCLASS_INVALID	/**< no/invalid class */
} bhnd_devclass_t;

const char		*bhnd_vendor_name(uint16_t vendor);
const char 		*bhnd_core_name(uint16_t vendor, uint16_t device);
bhnd_devclass_t		 bhnd_core_class(uint16_t vendor, uint16_t device);


/*
 * Device Matching
 */

/** A bhnd device match descriptor. */
struct bhnd_matchdesc {
	uint16_t	md_vendor;	/**< required JEP106 device vendor or JEDEC_MFGID_INVALID. */
	uint16_t	md_device;	/**< required core ID or BHND_COREID_INVALID */
	uint16_t	md_revid;	/**< required revision or BHND_HWREV_INVALID */
	bhnd_devclass_t	md_class;	/**< required class or BHND_DEVCLASS_INVALID */
};

bool			bhnd_device_matches(device_t dev,
			    struct bhnd_matchdesc *desc);
device_t		bhnd_find_child(device_t dev,
			    bhnd_devclass_t class);
device_t		bhnd_match_child(device_t dev,
			    struct bhnd_matchdesc *desc);


/*
 * Generic Bus Methods.
 */

struct bhnd_resource	*bhnd_generic_alloc_bhnd_resource (device_t dev,
			     device_t child, int type, int *rid, u_long start,
			     u_long end, u_long count, u_int flags);

int			 bhnd_generic_release_bhnd_resource (device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);

int			 bhnd_generic_activate_bhnd_resource (device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);

int			 bhnd_generic_deactivate_bhnd_resource (device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);



/*
 * Bus Resource Management
 */

/**
* A bhnd(4) bus resource.
* 
* This provides an abstract interface to per-core resources that may require
* bus-level remapping of address windows prior to access.
*/
struct bhnd_resource {
	struct resource	*_res;		/**< the system resource. */
	bool		 _direct;	/**< true if the resource requires
					*   bus window remapping before it
					*   is MMIO accessible. */
};

/**
 * Allocate a resource from a device's parent bhnd(4) bus.
 * 
 * @param dev The device requesting resource ownership.
 * @param type The type of resource to allocate. This may be any type supported
 * by the standard bus APIs.
 * @param rid The bus-specific handle identifying the resource being allocated.
 * @param start The start address of the resource.
 * @param end The end address of the resource.
 * @param count The size of the resource.
 * @param flags The flags for the resource to be allocated. These may be any
 * values supported by the standard bus APIs.
 * 
 * To request the resource's default addresses, pass @p start and
 * @p end values of @c 0UL and @c ~0UL, respectively, and
 * a @p count of @c 1.
 * 
 * @retval NULL The resource could not be allocated.
 * @retval resource The allocated resource.
 */
static inline struct bhnd_resource *
bhnd_alloc_resource (device_t dev, int type, int *rid, u_long start,
    u_long end, u_long count, u_int flags)
{
	return BHND_ALLOC_RESOURCE(device_get_parent(dev), dev, type, rid,
	    start, end, count, flags);
};

/**
 * Allocate a resource from a device's parent bhnd(4) bus.
 * 
 * This is a convenience wrapper for bhnd_alloc_resource; the default
 * start, end, and count values for the resource will be requested.
 * 
 * @param dev The device requesting resource ownership.
 * @param type The type of resource to allocate. This may be any type supported
 * by the standard bus APIs.
 * @param rid The bus-specific handle identifying the resource being allocated.
 * @param flags The flags for the resource to be allocated. These may be any
 * values supported by the standard bus APIs.
 * 
 * @retval NULL The resource could not be allocated.
 * @retval resource The allocated resource.
 */
static inline struct bhnd_resource *
bhnd_alloc_resource_any (device_t dev, int type, int *rid, u_int flags)
{
	return bhnd_alloc_resource(dev, type, rid, 0ULL, ~0ULL, 1, flags);
};

/**
 * Activate a previously allocated bhnd resource.
 *
 * @param dev The device holding ownership of the allocated resource.
 * @param type The type of the resource. 
 * @param rid The bus-specific handle identifying the resource.
 * @param r A pointer to the resoruce returned by bhnd_alloc_resource or
 * BHND_ALLOC_RESOURCE.
 * 
 * @retval 0 success
 * @retval non-zero an error occured while activating the resource.
 */
static inline int
bhnd_activate_resource (device_t dev, int type, int rid,
   struct bhnd_resource *r)
{
	return BHND_ACTIVATE_RESOURCE(device_get_parent(dev), dev, type, rid, r);
};

/**
 * Deactivate a previously activated bhnd resource.
 *
 * @param dev The device holding ownership of the activated resource.
 * @param type The type of the resource. 
 * @param rid The bus-specific handle identifying the resource.
 * @param r A pointer to the resoruce returned by bhnd_alloc_resource or
 * BHND_ALLOC_RESOURCE.
 * 
 * @retval 0 success
 * @retval non-zero an error occured while activating the resource.
 */
static inline int
bhnd_deactivate_resource (device_t dev, int type, int rid,
   struct bhnd_resource *r)
{
	return BHND_DEACTIVATE_RESOURCE(device_get_parent(dev), dev, type, rid, r);
};

/**
 * Return the resource-ID for a memory region on the given device port.
 *
 * @param dev The device being queried.
 * @param port The index of the device's interconnect port.
 * @param region The index of the port's mapped address region.
 *
 * @retval int The RID for the given @p port and @p region on @p device.
 * @retval -1 No such port/region found.
 */
static inline int
bhnd_get_port_rid(device_t dev, u_int port, u_int region)
{
    return BHND_GET_PORT_RID(device_get_parent(dev), dev, port, region);
}



/*
 * Bus Probe Configuration
 */

/**
 * A bus device probe configuration record.
 */
struct bhnd_probecfg {
	bhnd_devclass_t	 devclass;	/**< core device class to match. */
	int		 probe_order;	/**< device probe order for this core. */
	const char	*probe_name;	/**< device name for probe, or NULL. */
};

struct bhnd_probecfg	*bhnd_find_probecfg(struct bhnd_probecfg table[],
			    uint16_t vendor, uint16_t device);

/* default table */
extern struct bhnd_probecfg		bhnd_generic_probecfg_table[];

/**
 * Entry designating EOF in a bhnd_probecfg table.
 */
#define	BHND_PROBECFG_TABLE_END		{ BHND_DEVCLASS_INVALID, 0, NULL }

/**
 * bhnd device probe priority.
 */
enum {
	BHND_PROBE_ORDER_FIRST		= 0,	/**< probe first */
	BHND_PROBE_ORDER_EARLY		= 10,	/**< probe early */
	BHND_PROBE_ORDER_DEFAULT	= 20,	/**< default probe priority */
	BHND_PROBE_ORDER_LAST		= 30,	/**< probe last */
};


/*
 * Child Instance Variables
 */

enum bhnd_device_vars {
	/** Core designer's JEP-106 manufacturer ID. */ 
	BHND_IVAR_VENDOR,
	
	/** Core part number (aka core ID). */
	BHND_IVAR_DEVICE,
	
	/** Core revision identifier. */
	BHND_IVAR_REVID,

	/** Core class. */
	BHND_IVAR_DEVICE_CLASS,
	
	/** Vendor name. */
	BHND_IVAR_VENDOR_NAME,
	
	/** Core name. */
	BHND_IVAR_DEVICE_NAME,

	/** Core number */
	BHND_IVAR_CORE_INDEX
};

/*
 * Simplified accessors for bhnd device ivars
 */
#define	BHND_ACCESSOR(var, ivar, type) \
	__BUS_ACCESSOR(bhnd, var, BHND, ivar, type)

BHND_ACCESSOR(vendor,		VENDOR,		uint16_t);
BHND_ACCESSOR(device,		DEVICE,		uint16_t);
BHND_ACCESSOR(revid,		REVID,		uint8_t);
BHND_ACCESSOR(class,		DEVICE_CLASS,	bhnd_devclass_t);
BHND_ACCESSOR(vendor_name,	VENDOR_NAME,	const char *);
BHND_ACCESSOR(device_name,	DEVICE_NAME,	const char *);
BHND_ACCESSOR(core_index,	CORE_INDEX,	u_int);

#undef	BHND_ACCESSOR

#endif /* _BHND_BHNDVAR_H_ */
