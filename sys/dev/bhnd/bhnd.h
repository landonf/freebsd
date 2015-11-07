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
#include <sys/malloc.h>

#include "bhnd_if.h"

#include "bhnd_devinfo.h"
#include "bhndvar.h"

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

#endif /* _BHND_BHND_H_ */
