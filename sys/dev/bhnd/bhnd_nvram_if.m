#-
# Copyright (c) 2016 Landon Fuller <landon@landonf.org>
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

#include <dev/bhnd/bhnd.h>

INTERFACE bhnd_nvram;

#
# bhnd(4) NVRAM device interface.
#
# Provides a shared interface to HND NVRAM, OTP, and SPROM devices that provide
# access to a common set of hardware/device configuration variables.
#


/**
 * Return the size of an NVRAM variable.
 *
 * @param	dev	The NVRAM device.
 * @param	name	The NVRAM variable name.
 * @param[out]	size	On success, the size of @p name.
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable was not found.
 * @retval non-zero	If determining the size of @p name otherwise fails, a
 *			regular unix error code will be returned.
 */
METHOD int get_var_size {
	device_t	 dev;
	const char	*name;
	size_t		*size;
};


/**
 * Read an NVRAM variable.
 *
 * @param	dev	The NVRAM device.
 * @param	name	The NVRAM variable name.
 * @param[out]	buf	On success, the NVRAM variable will be read into this
 *			buffer.
 * @param	size	Size of buffer.
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable was not found.
 * @retval ENOMEM	A buffer of @p size is too small to hold the requested
 *			value.
 * @retval non-zero	If determining the size of @p name otherwise fails, a
 *			regular unix error code will be returned.
 */
METHOD int read_var {
	device_t	dev;
	const char	*name;
	void		*buf;
	size_t		size;
};