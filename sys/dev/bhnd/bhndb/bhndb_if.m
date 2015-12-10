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

	static const struct bhnd_chipid *
	bhndb_null_get_chipid(device_t dev, device_t child)
	{
		panic("bhndb_get_chipid unimplemented\n");
	}

	static int
	bhndb_null_init_full_config(device_t dev, device_t child)
	{
		panic("bhndb_init_full_config unimplemented\n");
	}

	static int
	bhndb_null_set_window_addr(device_t dev,
	    const struct bhndb_regwin *rw, bhnd_addr_t addr)
	{
		panic("bhndb_set_window_addr unimplemented");
	}
}

/**
 * Return the chip identification information for @p child.
 *
 * @param dev The parent device of @p child.
 * @param child The bhndb-attached device.
 */
METHOD const struct bhnd_chipid * get_chipid {
	device_t dev;
	device_t child;
} DEFAULT bhndb_null_get_chipid;

/**
 * Perform final bridge hardware configuration after @p child has fully
 * enumerated its children.
 *
 * This must be called by any bhndb-attached bridge devices; this allows the
 * bridge to perform final configuration based on the hardware information
 * enumerated by the child bus.
 *
 * When calling this method:
 * - Any bus resources previously allocated by @p child must be deallocated.
 * - The @p child bus must have performed initial enumeration -- but not
 *   probe or attachment -- of its children.
 */
METHOD int init_full_config {
	device_t dev;
	device_t child;
} DEFAULT bhndb_null_init_full_config;

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
	const struct bhndb_regwin *win;
	bhnd_addr_t addr;
} DEFAULT bhndb_null_set_window_addr;
