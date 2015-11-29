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

#include <sys/param.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#
# bhndb bridge device interface.
#

INTERFACE bhndb;

HEADER {
	struct bhndb_regwin;
	struct bhndb_hw;
}

CODE {
	#include <sys/systm.h>
	#include <dev/bhnd/bhndb/bhndbvar.h>
	
	static int
	bhndb_null_enable_clocks(device_t dev)
	{
		return (0);
	}
	
	static int
	bhndb_null_disable_clocks(device_t dev)
	{
		return (0);
	}

	static bhnd_addr_t
	bhndb_null_get_enum_addr(device_t dev, device_t child)
	{
		panic("bhndb_get_enum_addr unimplemented\n");
	}

	static int
	bhndb_null_get_core_table(device_t dev, struct bhnd_core_info **cores,
	    u_int *count)
	{
		panic("bhndb_get_core_table unimplemented");
	}

	static int
	bhndb_null_get_window_addr(device_t dev,
	    const struct bhndb_regwin *rw, bhnd_addr_t *addr)
	{
		panic("bhndb_get_window_addr unimplemented");
	}
	
	static int
	bhndb_null_set_window_addr(device_t dev,
	    const struct bhndb_regwin *rw, bhnd_addr_t addr)
	{
		panic("bhndb_set_window_addr unimplemented");
	}

	static device_t
	bhndb_null_get_attached_bus(device_t dev)
	{
		panic("bhndb_get_attached_bus unimplemented");
	}
}

/**
 * Return the base address at which @p child should perform device enumeration.
 *
 * @param dev The parent device of @p child.
 * @param child The bhndb child performing bus enumeration.
 */
METHOD bhnd_addr_t get_enum_addr {
	device_t dev;
	device_t child;
} DEFAULT bhndb_null_get_enum_addr;


/**
 * Enable clocks required prior to accessing mapped core registers. This must 
 * be called early in device_attach() and device_resume().
 * 
 * @param dev The bridge device.
 *
 * @retval 0 success
 * @retval non-zero Enabling clocks failed; clocks may be in unknown state.
 */
METHOD int enable_clocks {
	device_t dev;
} DEFAULT bhndb_null_enable_clocks;

/**
 * Disable clocks required for accessing mapped core registers. 
 * @retval 0 success
 * @retval non-zero Disabling clocks failed; clocks may be in unknown state.
 */
METHOD int disable_clocks {
	device_t dev;
} DEFAULT bhndb_null_disable_clocks;

/**
 * Retrieve the list of all cores enumerated by @p dev.
 *
 * This method may return an error if requisite bus resources
 * have already been claimed by the bridge.
 * 
 * The memory allocated for the table should be freed using
 * `free(*cores, M_BHND)`. @p cores and @p num_cores are not changed
 * when an error is returned.
 * 
 * @param dev The bridge device.
 * @param[out] cores the table of parsed core descriptors.
 * @param[out] num_cores the number of core records in @p cores.
 */
METHOD int get_core_table {
	device_t dev;
	struct bhnd_core_info **cores;
	u_int *count;
} DEFAULT bhndb_null_get_core_table;

/**
 * Return the bhnd-compatible child bus device attached to this bridge.
 *
 * @param dev The bridge device.
 *
 * @retval device_t The attached bus device.
 * @retval NULL The bus device was not found.
 */
METHOD device_t get_attached_bus {
	device_t dev;
} DEFAULT bhndb_null_get_attached_bus;

/**
 * Read a given register window's base address.
 *
 * @param dev The bridge device.
 * @param win The register window.
 * @param[out] addr On success, the bus address of @p win.
 *
 * @retval 0 success
 * @retval ENODEV The provided @p win is not memory-mapped on the bus. 
 * @retval non-zero an error occured determining the window address.
 */
METHOD int get_window_addr {
	device_t dev;
	const struct bhndb_regwin *win;
	bhnd_addr_t *addr;
} DEFAULT bhndb_null_get_window_addr;

/**
 * Set a given register window's base address.
 *
 * @param dev The bridge device.
 * @param win The register window.
 * @param addr The address to be configured for @p win.
 *
 * @retval 0 success
 * @retval ENODEV The provided @p win is not memory-mapped on the bus or does
 * not support setting a base address.
 * @retval non-zero failure
 */
METHOD int set_window_addr {
	device_t dev;
	const struct bhndb_regwin *dynwin;
	bhnd_addr_t addr;
} DEFAULT bhndb_null_set_window_addr;
