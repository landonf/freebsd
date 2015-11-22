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

#
# Parent bus interface required by attached bhndb bridge devices.
#

INTERFACE bhndb_bus;

HEADER {
	struct bhndb_hwcfg;
	struct bhndb_hw;
};

CODE {
	#include <sys/systm.h>

	static const struct bhndb_hwcfg *
	bhndb_null_get_generic_hwcfg(device_t dev, device_t child)
	{
		panic("bhndb_get_generic_hwcfg unimplemented");
	}

	static const struct bhndb_hw *
	bhndb_null_get_hardware_table(device_t dev, device_t child)
	{
		panic("bhndb_get_hardware_table unimplemented");
	}
}

/**
 * Return a generic hardware configuration to be used by
 * the bhndb bridge device to enumerate attached devices.
 *
 * @param dev The parent device.
 * @param child The attached bhndb device.
 *
 * @retval bhndb_hwcfg The configuration to use for bus enumeration.
 */
METHOD const struct bhndb_hwcfg * get_generic_hwcfg {
	device_t dev;
	device_t child;
} DEFAULT bhndb_null_get_generic_hwcfg;


/**
 * Return the hardware specification table to be used when identifying the
 * bridge's full hardware configuration.
 *
 * @param dev The parent device.
 * @param child The attached bhndb device.
 */
 METHOD const struct bhndb_hw * get_hardware_table {
	device_t dev;
	device_t child;
} DEFAULT bhndb_null_get_hardware_table;
