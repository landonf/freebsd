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

#include <dev/bhnd/bhnd.h>

INTERFACE bhnd_erom;

#
# Common interface to bhnd(4) device enumeration
#

/**
 * Open an EROM resource for reading.
 *
 * @param self	the EROM instance.
 * @param bus	the bhnd(4) bus.
 * @param iosw	bus i/o callbacks.
 * @param chipc	the ChipCommon base address.
 *
 * @retval 0		success
 * @retval ENXIO	if the bhnd(4) bus type is not supported.
 * @retval non-zero	if an error occurs during the open, a regular UNIX
 *			error code should be returned.
 */
METHOD int open {
	kobj_t self;
	device_t bus;
	struct bhnd_iosw *iosw;
	bhnd_addr_t chipc;
};

/**
 * Close an EROM resource and release any associated resources.
 *
 * @param self the EROM instance.
 *
 * @retval 0		success
 * @retval non-zero	if an error occurs during the open, a regular UNIX
 *			error code should be returned.
 */
METHOD int close {
	kobj_t self;
};

/**
 * Parse all cores descriptors from @p self and return the array
 * in @p cores and the count in @p num_cores.
 * 
 * The memory allocated for the table should be freed using
 * `free(*cores, M_BHND)`. @p cores and @p num_cores are not changed
 * when an error is returned.
 * 
 * @param	self		the EROM instance.
 * @param[out]	cores		the table of parsed core descriptors.
 * @param[out]	num_cores	the number of core records in @p cores.
 */
METHOD int read_core_table {
	kobj_t self;
	struct bhnd_core_info **cores;
	u_int *num_cores;
};
