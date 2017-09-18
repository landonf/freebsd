#-
# Copyright (c) 2017 The FreeBSD Foundation
# All rights reserved.
#
# This software was developed by Landon Fuller under sponsorship from
# the FreeBSD Foundation.
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

#include <dev/bhnd/bhnd.h>

INTERFACE bhnd_hostb;

#
# bhnd(4) host bridge interface.
#
# Provides a common host bridge interface.
#

/**
 * Enable routing of @p core's backplane interrupt line(s) to the host
 * via @p hostb.
 *
 * @param hostb	The host bridge device.
 * @param core	A core bridged by @p hostb.
 *
 * @retval 0           success
 * @retval non-zero    if an error occurs routing the interrupt line(s),
 *                     a regular unix error code will be returned.
 */
METHOD int enable_core_interrupts {
	device_t	hostb;
	device_t	core;
};

/**
 * Disable routing of @p core's backplane interrupt line(s) to the host
 * via @p hostb.
 *
 * @param hostb	The host bridge device.
 * @param core	A core bridged by @p hostb.
 *
 * @retval 0           success
 * @retval non-zero    if an error occurs disabling core interrupt routing, a
 *                     regular unix error code will be returned.
 */
METHOD int disable_core_interrupts {
	device_t	hostb;
	device_t	core;
};
