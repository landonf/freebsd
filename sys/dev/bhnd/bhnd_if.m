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

#include <dev/bhnd/bhnd_types.h>

INTERFACE bhnd;

#
# bhnd(4) bus interface
#

CODE {
	#include <dev/bhnd/bhndvar.h>
	
	static int
	bhnd_null_read_core_table(kobj_class_t driver, device_t dev,
	    const struct bhnd_chipid *chipid, const void *ioh,
	    const struct bhnd_iosw *iosw, struct bhnd_core_info **cores,
	    u_int *num_cores)
	{
		return (ENXIO);
	}

	static int
	bhnd_null_get_port_rid(device_t dev, device_t child, u_int port,
	    u_int region)
	{
		return (-1);
	}
	
	static int
	bhnd_null_decode_port_rid(device_t dev, device_t child, int type,
	    int rid, u_int *port, u_int *region)
	{
		return (ENOENT);
	}
	
	static int
	bhnd_null_get_port_addr(device_t dev, device_t child, u_int port,
	    u_int region, bhnd_addr_t *addr, bhnd_size_t *size)
	{
		return (ENOENT);
	}
}

/**
 * Probe the bus defined by @p chipid, and if a supported bus type, enumerate
 * all devices on the bus.
 * 
 * The memory allocated for the table should be freed using
 * `free(*cores, M_BHND)`. @p cores and @p num_cores are not changed
 * when an error is returned.
 * 
 * @param	driver		the driver class.
 * @param	chipid		the chip's identification info.
  * @param	ioh		device i/o handle.
 * @param	iosw		device i/o callbacks.
 * @param[out]	cores		the table of parsed core descriptors.
 * @param[out]	num_cores	the number of core records in @p cores.
 * 
 * @retval 0		success
 * @retval ENXIO	if the bhnd(4) bus type is not supported.
 * @retval non-zero	if an error occurs during the open, a regular UNIX
 *			error code should be returned.
 */
STATICMETHOD int read_core_table {
	kobj_class_t driver;
	device_t dev;
	const struct bhnd_chipid *chipid;
	const void *ioh;
	const struct bhnd_iosw *iosw;
	struct bhnd_core_info **cores;
	u_int *num_cores;
} DEFAULT bhnd_null_read_core_table;

/**
 * Returns true if @p child is serving as a host bridge for the bhnd
 * bus.
 *
 * The default implementation will walk the parent device tree until
 * the root node is hit, returning false.
 *
 * @param dev The device whose child is being examined.
 * @param child The child device.
 */
METHOD bool is_hostb_device {
	device_t dev;
	device_t child;
} DEFAULT bhnd_generic_is_hostb_device;

/**
 * Return true if the hardware components required by @p child are populated on
 * the hardware board.
 *
 * In some cases, enumerated devices may have pins that are left floating, or
 * the hardware may otherwise be non-functional; this method allows a parent
 * device to explicitly specify if a successfully enumerated @p child should
 * be disabled.
 *
 * @param dev The device whose child is being examined.
 * @param child The child device.
 */
METHOD bool is_hw_populated {
	device_t dev;
	device_t child;
} DEFAULT bhnd_generic_is_hw_populated;

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
 * Decode a port / region pair on @p child defined by @p rid.
 *
 *
 * @param dev The bus device.
 * @param child The bhnd child.
 * @param port The port identifier.
 * @param region The identifier of the memory region on @p port.
 * 
 * @retval int The RID for the given @p port and @p region on @p device.
 * @retval non-zero No matching port/region found.
 */
METHOD int decode_port_rid {
	device_t dev;
	device_t child;
	int type;
	int rid;
	u_int *port;
	u_int *region;
} DEFAULT bhnd_null_decode_port_rid;

/**
 * Get the address and size of @p region on @p port.
 *
 * @param dev The bus device.
 * @param child The bhnd child.
 * @param port The port identifier.
 * @param region The identifier of the memory region on @p port.
 * @param[out] region_addr The region's base address.
 * @param[out] region_size The region's size.
 *
 * @retval 0 success
 * @retval non-zero No matching port/region found.
 */
METHOD int get_port_addr {
	device_t dev;
	device_t child;
	u_int port;
	u_int region;
	bhnd_addr_t *region_addr;
	bhnd_size_t *region_size;
} DEFAULT bhnd_null_get_port_addr;


/** An implementation of bus_read_1() compatible with bhnd_resource */
METHOD uint8_t bus_read_1 {
	device_t dev;
	device_t child;
	struct bhnd_resource *r;
	bus_size_t offset;
}

/** An implementation of bus_read_2() compatible with bhnd_resource */
METHOD uint16_t bus_read_2 {
	device_t dev;
	device_t child;
	struct bhnd_resource *r;
	bus_size_t offset
}

/** An implementation of bus_read_4() compatible with bhnd_resource */
METHOD uint32_t bus_read_4 {
	device_t dev;
	device_t child;
	struct bhnd_resource *r;
	bus_size_t offset;
}

/** An implementation of bus_write_1() compatible with bhnd_resource */
METHOD void bus_write_1 {
	device_t dev;
	device_t child;
	struct bhnd_resource *r;
	bus_size_t offset;
	uint8_t value;
}

/** An implementation of bus_write_2() compatible with bhnd_resource */
METHOD void bhnd_bus_write_2 {
	device_t dev;
	device_t child;
	struct bhnd_resource *r;
	bus_size_t offset;
	uint16_t value;
}

/** An implementation of bus_write_4() compatible with bhnd_resource */
METHOD void bhnd_bus_write_4 {
	device_t dev;
	device_t child;
	struct bhnd_resource *r;
	bus_size_t offset;
	uint32_t value;
}

/** An implementation of bus_barrier() compatible with bhnd_resource */
METHOD void bhnd_bus_barrier {
	device_t dev;
	device_t child;
	struct bhnd_resource *r;
	bus_size_t offset;
	bus_size_t length;
	int flags;
}
