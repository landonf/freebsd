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

INTERFACE bhnd_pic;

#
# bhnd(4) interrupt controller interface
#

/**
 * Activate an interrupt for the given bhnd(4) @p core.
 * 
 * @param dev	The PIC device.
 * @param core	The bhnd(4) core for which @p rid should be activated.
 * @param rid	The SYS_RES_IRQ resource identifier to be activated.
 *
 * @retval 0		success
 * @retval non-zero	if an error occurs activating the interrupt,
 *			a regular unix error code will be returned.
 */
METHOD int activate_intr {
	device_t	dev;
	device_t	core;
	int		rid;
};

/**
 * Deactivate an interrupt for the given bhnd(4) @p core.
 * 
 * @param dev	The PIC device.
 * @param core	The bhnd(4) core for which @p rid should be deactivated.
 * @param rid	The SYS_RES_IRQ resource identifier to be deactivated.
 *
 * @retval 0		success
 * @retval non-zero	if an error occurs deactivating the interrupt,
 *			a regular unix error code will be returned.
 */
METHOD int deactivate_intr {
	device_t	dev;
	device_t	core;
	int		rid;
};
