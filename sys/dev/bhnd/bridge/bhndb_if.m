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

#
# bhndb bridge device interface.
#

INTERFACE bhndb;

HEADER {
	struct bhnd_core_info;
	struct bhndb_regwin;
}

CODE {
	#include <sys/systm.h>

	static int
	bhndb_null_get_core_table(device_t dev, struct bhnd_core_info **cores,
	    u_int *count)
	{
		panic("bhndb_get_core_table unimplemented");
	}

	static int
	bhndb_null_get_window_addr(device_t dev,
	    const struct bhndb_regwin *rw, uint32_t *addr)
	{
		panic("bhndb_get_window_addr unimplemented");
	}
	
	static int
	bhndb_null_set_window_addr(device_t dev,
	    const struct bhndb_regwin *rw, uint32_t addr)
	{
		panic("bhndb_set_window_addr unimplemented");
	}
}

/**
 * Retrieve the list of all cores enumerated by @p dev.
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
	uint32_t *addr;
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
	uint32_t addr;
} DEFAULT bhndb_null_set_window_addr;
