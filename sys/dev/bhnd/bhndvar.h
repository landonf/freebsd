/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 * 
 * $FreeBSD$
 */

#ifndef _BHND_BHNDVAR_H_
#define _BHND_BHNDVAR_H_

#include <sys/param.h>
#include <sys/types.h>
#include <sys/malloc.h>

#include <sys/bus.h>

#include "bhnd_devinfo.h"

#include "bhnd_if.h"

MALLOC_DECLARE(M_BHND);


int			 bhnd_generic_print_child(device_t dev,
			     device_t child);
void			 bhnd_generic_probe_nomatch(device_t dev,
			     device_t child);

struct bhnd_resource	*bhnd_generic_alloc_bhnd_resource (device_t dev,
			     device_t child, int type, int *rid, u_long start,
			     u_long end, u_long count, u_int flags);

int			 bhnd_generic_release_bhnd_resource (device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);

int			 bhnd_generic_activate_bhnd_resource (device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);

int			 bhnd_generic_deactivate_bhnd_resource (device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);

/**
 * bhnd child instance variables
 */
enum bhnd_device_vars {
	/** Core designer's JEP-106 manufacturer ID. */ 
	BHND_IVAR_VENDOR,
	
	/** Core part number (aka core ID). */
	BHND_IVAR_DEVICE,
	
	/** Core revision identifier. */
	BHND_IVAR_REVID,

	/** Core class. */
	BHND_IVAR_DEVICE_CLASS,
	
	/** Vendor name. */
	BHND_IVAR_VENDOR_NAME,
	
	/** Core name. */
	BHND_IVAR_DEVICE_NAME,

	/** Core number */
	BHND_IVAR_CORE_INDEX
};

/*
 * Simplified accessors for bhnd device ivars
 */
#define	BHND_ACCESSOR(var, ivar, type) \
	__BUS_ACCESSOR(bhnd, var, BHND, ivar, type)

BHND_ACCESSOR(vendor,		VENDOR,		uint16_t);
BHND_ACCESSOR(device,		DEVICE,		uint16_t);
BHND_ACCESSOR(revid,		REVID,		uint8_t);
BHND_ACCESSOR(class,		DEVICE_CLASS,	bhnd_devclass_t);
BHND_ACCESSOR(vendor_name,	VENDOR_NAME,	const char *);
BHND_ACCESSOR(device_name,	DEVICE_NAME,	const char *);
BHND_ACCESSOR(core_index,	CORE_INDEX,	u_int);

#undef	BHND_ACCESSOR

#endif /* _BHND_BHNDVAR_H_ */
