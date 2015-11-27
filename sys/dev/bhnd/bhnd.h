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

#ifndef _BHND_BHND_H_
#define _BHND_BHND_H_

#include <sys/types.h>

#include "bhnd_ids.h"
#include "bhnd_types.h"
#include "bhnd_if.h"

/**
 * bhnd child instance variables
 */
enum bhnd_device_vars {
	BHND_IVAR_VENDOR,	/**< Designer's JEP-106 manufacturer ID. */
	BHND_IVAR_DEVICE,	/**< Part number */
	BHND_IVAR_REVID,	/**< Core revision */
	BHND_IVAR_DEVICE_CLASS,	/**< Core class (@sa bhnd_devclass_t) */
	BHND_IVAR_VENDOR_NAME,	/**< Core vendor name */
	BHND_IVAR_DEVICE_NAME,	/**< Core name */
	BHND_IVAR_CORE_INDEX,	/**< Bus-assigned core number */
	BHND_IVAR_CORE_UNIT,	/**< Bus-assigned core unit number,
				     assigned sequentially (starting at 0) for
				     each vendor/device pair. */
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
BHND_ACCESSOR(core_unit,	CORE_UNIT,	int);

#undef	BHND_ACCESSOR

const char			*bhnd_vendor_name(uint16_t vendor);
const char 			*bhnd_core_name(uint16_t vendor, uint16_t device);
bhnd_devclass_t			 bhnd_core_class(uint16_t vendor, uint16_t device);

device_t			 bhnd_match_child(device_t dev,
				     const struct bhnd_core_match *desc);

device_t			 bhnd_find_child(device_t dev,
				     bhnd_devclass_t class, int unit);

const struct bhnd_core_info	*bhnd_match_core(
				     const struct bhnd_core_info *cores,
				     u_int num_cores,
				     const struct bhnd_core_match *desc);

const struct bhnd_core_info	*bhnd_find_core(
				     const struct bhnd_core_info *cores,
				     u_int num_cores, bhnd_devclass_t class);

bool				 bhnd_core_matches(
				     const struct bhnd_core_info *core,
				     const struct bhnd_core_match *desc);

bool				 bhnd_device_matches(device_t dev,
				     const struct bhnd_core_match *desc);

/**
 * Return true if @p dev is serving as a host bridge for its parent bhnd
 * bus.
 *
 * @param dev A bhnd bus child device.
 */
static inline bool
bhnd_is_hostb_device(device_t dev) {
	return (BHND_IS_HOSTB_DEVICE(device_get_parent(dev), dev));
}

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
	return bhnd_alloc_resource(dev, type, rid, 0UL, ~0UL, 1, flags);
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
 * @param port The port identifier.
 * @param region The identifier of the memory region on @p port.
 * 
 * @retval int The RID for the given @p port and @p region on @p device.
 * @retval -1 No such port/region found.
 */
static inline int
bhnd_get_port_rid(device_t dev, u_int port, u_int region)
{
	return BHND_GET_PORT_RID(device_get_parent(dev), dev, port, region);
}

/**
 * Decode a port / region pair on @p dev defined by @p rid.
 *
 * @param dev The device being queried.
 * @param type The resource type.
 * @param rid The resource identifier.
 * @param[out] port The decoded port identifier.
 * @param[out] region The decoded region identifier.
 *
 * @retval 0 success
 * @retval non-zero No matching port/region found.
 */
static inline int
bhnd_decode_port_rid(device_t dev, int type, int rid, u_int *port,
    u_int *region)
{
	return BHND_DECODE_PORT_RID(device_get_parent(dev), dev, type, rid,
	    port, region);
}

/**
 * Get the address and size of @p region on @p port.
 *
 * @param dev The device being queried.
 * @param port The port identifier.
 * @param region The identifier of the memory region on @p port.
 * @param[out] region_addr The region's base address.
 * @param[out] region_size The region's size.
 *
 * @retval 0 success
 * @retval non-zero No matching port/region found.
 */
static inline int
bhnd_get_port_addr(device_t dev, u_int port, u_int region,
   bhnd_addr_t *region_addr, bhnd_size_t *region_size)
{
	return BHND_GET_PORT_ADDR(device_get_parent(dev), dev, port, region,
	    region_addr, region_size);
}

#endif /* _BHND_BHND_H_ */
