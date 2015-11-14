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
# Internal bhndb bridge hardware interface.
#

INTERFACE bhndb_hw;

HEADER {
	struct bhndb_core_info;
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
	struct bhndb_core_info **cores;
	size_t *count;
}

/**
 * Write a new base address to a bridge window register.
 *
 * @param dev The bhndb bridge device.
 * @param reg The offset of the bridge window register.
 * @param addr The address to be written to @p reg.
 *
 * @retval 0 success
 * @retval non-zero failure
 */
METHOD int set_window_register {
	device_t dev;
	bus_size_t reg;
	uint32_t addr;
};
