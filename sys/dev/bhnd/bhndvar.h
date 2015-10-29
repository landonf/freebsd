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

#include "bhnd_if.h"

/**
 * A bus device probe configuration record.
 */
struct bhnd_probecfg {
	uint16_t	 vendor;	/**< JEP106 device vendor to match on. */
	uint16_t	 device;	/**< device identifier to match on. */

	int		 probe_order;	/**< device probe order for this core. */
	const char	*probe_name;	/**< device name for probe, or NULL. */
};
extern struct bhnd_probecfg bhnd_generic_probecfg_table[];


const char		*bhnd_vendor_name(uint16_t vendor);
const char 		*bhnd_core_name(uint16_t vendor, uint16_t device);
struct bhnd_probecfg	*bhnd_find_probecfg(struct bhnd_probecfg table[],
			    uint16_t vendor, uint16_t device);

/** Represents EOF in a bhnd_probecfg table. */
#define	BHND_PROBECFG_TABLE_END	{ 0, BHND_COREID_NODEV, 0, NULL }

/* Standard device probe ordering. */
#define	BHND_PROBE_ORDER_FIRST		 0
#define	BHND_PROBE_ORDER_EARLY		 10
#define	BHND_PROBE_ORDER_DEFAULT	 20
#define	BHND_PROBE_ORDER_LAST		 30

enum bhnd_device_vars {
	/** Core designer's JEP-106 manufacturer ID. */ 
	BHND_IVAR_VENDOR,
	
	/** Core part number (aka core ID). */
	BHND_IVAR_DEVICE,
	
	/** Core revision identifier. */
	BHND_IVAR_REVID,
	
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
BHND_ACCESSOR(vendor_name,	VENDOR_NAME,	const char *);
BHND_ACCESSOR(device_name,	DEVICE_NAME,	const char *);
BHND_ACCESSOR(core_index,	CORE_INDEX,	u_int);

#undef	BHND_ACCESSOR

static __inline uint32_t
bhnd_get_port_rid(device_t dev, u_int port, u_int region)
{
    return BHND_GET_PORT_RID(device_get_parent(dev), dev, port, region);
}


#endif /* _BHND_BHNDVAR_H_ */