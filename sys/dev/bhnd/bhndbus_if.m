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

INTERFACE bhndbus;

HEADER {
	struct bhnd_resource;
}

CODE {
	static struct rman *
	bhnd_null_get_rman(device_t dev, int type)
	{
		return (NULL);
	}

        /* delegate enumeration to the device parent. */
        static int 
        bhnd_delegate_enumerate_children(device_t dev, device_t child)
        {
                return BHNDBUS_ENUMERATE_CHILDREN(device_get_parent(dev), dev);
        }
}

/**
 * Enumerate all devices on this bus, calling BUS_ADD_CHILD for each discovered
 * core.
 *
 * @param dev The bus parent.
 * @param child The bus device.
 */
METHOD int enumerate_children {
        device_t dev;
        device_t child;
} DEFAULT bhnd_delegate_enumerate_children;

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
 * Return the resource manager for the given resource type.
 *
 * Used by drivers which use bhnd_pci_generic_alloc_resource() etc. to
 * implement their resource handling.
 *
 * @param dev The bus device.
 * @param type The resource type (e.g. SYS_RES_MEMORY, SYS_REQ_IRQ).
 */
METHOD struct rman * get_rman {
        device_t dev;
	int type;
} DEFAULT bhnd_null_get_rman;


/**
 * Return the resource-ID for a port / region pair
 *
 * @param dev The parent device of @p child.
 * @param child The child being queried.
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
};
