#-
# Copyright (c) 2015 Landon Fuller <landon@landonf.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# $FreeBSD$

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/bhnd/bhndvar.h>

INTERFACE bhnd;

HEADER {
	struct bhnd_resource;
}

CODE {
	#include <dev/bhnd/bhnd_private.h>

	static int
	bhnd_null_get_port_rid(device_t dev, device_t child, u_int port_num,
	    u_int region_num)
	{
		return (-1);
	}
	
	static int
	bhnd_null_decode_port_rid(device_t dev, device_t child, int type,
	    int rid, u_int *port_num, u_int *region_num, u_long *region_addr,
	    u_long *region_size)
	{
		return (ENOENT);
	}
}

/**
 * Allocate a bhnd resource.
 *
 * This method's semantics are functionally identical to the bus API of the same
 * name; refer to BUS_ALLOC_RESOURCE for complete documentation.
 */
METHOD struct bhnd_resource * alloc_resource {
	device_t dev;
	device_t child;
	int type;
	int *rid;
	u_long start;
	u_long end;
	u_long count;
	u_int flags;
} DEFAULT bhnd_generic_alloc_bhnd_resource;

/**
 * Release a bhnd resource.
 *
 * This method's semantics are functionally identical to the bus API of the same
 * name; refer to BUS_RELEASE_RESOURCE for complete documentation.
 */
METHOD int release_resource {
	device_t dev;
	device_t child;
	int type;
	int rid;
	struct bhnd_resource *res;
} DEFAULT bhnd_generic_release_bhnd_resource;

/**
 * Activate a bhnd resource.
 *
 * This method's semantics are functionally identical to the bus API of the same
 * name; refer to BUS_ACTIVATE_RESOURCE for complete documentation.
 */
METHOD int activate_resource {
	device_t dev;
        device_t child;
	int type;
        int rid;
        struct bhnd_resource *r;
} DEFAULT bhnd_generic_activate_bhnd_resource;

/**
 * Deactivate a bhnd resource.
 *
 * This method's semantics are functionally identical to the bus API of the same
 * name; refer to BUS_DEACTIVATE_RESOURCE for complete documentation.
 */
METHOD int deactivate_resource {
        device_t dev;
        device_t child;
        int type;
	int rid;
        struct bhnd_resource *r;
} DEFAULT bhnd_generic_deactivate_bhnd_resource;

/**
 * Return the SYS_RES_MEMORY resource-ID for a port /region pair attached to
 * @p child.
 *
 * @param dev The bus device.
 * @param child The bhnd child.
 * @param port_num The index of the child interconnect port.
 * @param region_num The index of the port-mapped address region.
 *
 * @retval -1 No such port/region found.
 */
METHOD int get_port_rid {
	device_t dev;
	device_t child;
	u_int port_num;
	u_int region_num;
} DEFAULT bhnd_null_get_port_rid;


/**
 * Decode a port / region pair on @p child from @p rid.
 *
 * @param dev The bus device.
 * @param child The bhnd child.
 * @param type The resource type.
 * @param rid The resource identifier.
 * @param[out] port_num The decoded port number.
 * @param[out] region_num The decoded region number.
 * @param[out] region_addr The decoded region address.
 * @param[out] region_size The decoded region size.
 *
 * @retval 0 success
 * @retval non-zero No matching port/region found.
 */
METHOD int decode_port_rid {
	device_t dev;
	device_t child;
	int type;
	int rid;
	u_int *port_num;
	u_int *region_num;
	u_long *region_addr;
	u_long *region_size;
} DEFAULT bhnd_null_decode_port_rid;