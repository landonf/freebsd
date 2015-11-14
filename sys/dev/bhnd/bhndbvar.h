/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 * $FreeBSD$
 */

#ifndef _BHND_BHNDBVAR_H_
#define _BHND_BHNDBVAR_H_

#include "bhnd.h"

extern devclass_t bhndb_devclass;
struct bhndb_hw_class;


/**
 * bhndb child instance variables
 */
enum bhndb_device_vars {
	BHNDB_IVAR_DEV_BASE_ADDR,	/**< device enumeration base address */
};

/*
 * Simplified accessors for bhndb device ivars
 */
#define	BHND_ACCESSOR(var, ivar, type) \
	__BUS_ACCESSOR(bhndb, var, BHNDB, ivar, type)

BHND_ACCESSOR(dev_base_addr,	DEV_BASE_ADDR,		bus_addr_t);

#undef	BHND_ACCESSOR


#endif /* _BHND_BHNDBVAR_H_ */
