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
# Provides a common interface for implementing core-specific host bridge
# behavior on behalf of a parent bhndb(4) bridge.
#

HEADER {
	#include <sys/param.h>
	#include <sys/bus.h>
}

CODE {
	static int
	bhnd_hostb_null_setup_intr(device_t hostb, device_t parent,
	    device_t child, bus_setup_intr_t *setup_intr, struct resource *r,
	    int flags, driver_filter_t filter, driver_intr_t handler, void *arg,
	    void **cookiep)
	{
		return (setup_intr(parent, child, r, flags, filter, handler,
		    arg, cookiep));
	}

	static int
	bhnd_hostb_null_teardown_intr(device_t hostb, device_t parent,
	    device_t child, bus_teardown_intr_t *teardown_intr,
	    struct resource *r, void *cookiep)
	{
		return (teardown_intr(parent, child, r, cookiep));
	}
}

/**
 * Create and attach an interrupt handler using @p setup_intr on behalf of
 * @p parent for @p child, as per the API defined in bus_setup_intr(9),
 * performing any filtering, modification, or additional configuration as
 * required by the host bridge.
 *
 * To allow the host bridge to manage interrupt handler state, the interrupt
 * must be detached by calling BHND_HOSTB_TEARDOWN_INTR().
 *
 * @param	hostb		The host bridge device.
 * @param	parent		The requesting parent of @p child.
 * @param	child		The child for which an interrupt handler will be
 *				created and attached.
 * @param	setup_intr	The function to be called by the host bridge
 *				after filtering the request.
 * @param	r		The interrupt resource.
 * @param	flags		Interrupt handler flags (e.g. INTR_EXCL, etc).
 * @param	filter		The interrupt filter routine, or NULL.
 * @param	ithread		The interrupt ithread, or NULL.
 * @param	arg		The argument to be passed to the interrupt
 *				handler.
 * @param[out]	cookiep		On success, a non-NULL value uniquely identifying
 *				this interrupt handler will be written to
 *				@p cookiep. This must be passed to
 *				BHND_HOSTB_TEARDOWN_INTR() to identify the
 *				correct interrupt handler.
 *
 * @retval 0           success
 * @retval non-zero    if an error occurs setting up the interrupt handler,
 *                     a regular unix error code will be returned.
 */
METHOD int setup_intr {
	device_t		 hostb;
	device_t		 parent;
	device_t		 child;
	bus_setup_intr_t	*setup_intr;
	struct resource		*r;
	int			 flags;
	driver_filter_t		 filter;
	driver_intr_t		 ithread;
	void			*arg;
	void			**cookiep;
} DEFAULT bhnd_hostb_null_setup_intr;

/**
 * Detach an interrupt handler previously attached via BHND_HOSTB_SETUP_INTR(),
 * using @p teardown_intr on behalf of @p parent for @p child, as per the API
 * defined in bus_teardown_intr(9), performing any filtering, modification, or
 * additional configuration as required by the host bridge.
 *
 * @param hostb		The host bridge device.
 * @param parent	The requesting parent of @p child.
 * @param child		The child for which an interrupt handler will be created
 *			and attached.
 * @param teardown_intr	The function to be called by the host bridge after
 *			filtering the request.
 * @param r		The interrupt resource.
 * @param cookiep	The cookiep value uniquely identifying this interrupt
 *			handler.
 *
 * @retval 0           success
 * @retval non-zero    if an error occurs setting up the interrupt handler,
 *                     a regular unix error code will be returned.
 */
METHOD int teardown_intr {
	device_t		 hostb;
	device_t		 parent;
	device_t		 child;
	bus_teardown_intr_t	*teardown_intr;
	struct resource		*r;
	void			*cookiep;
} DEFAULT bhnd_hostb_null_teardown_intr;
