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

#ifndef _BHND_BHND_DEVINFO_H_
#define _BHND_BHND_DEVINFO_H_

#include <sys/param.h>

#include "bhnd_devids.h"

/** BHND Device Classes. */
typedef enum {
	BHND_DEVCLASS_CC,	/**< chipcommon i/o controller */
	BHND_DEVCLASS_PCI,	/**< pci/pcie host/device bridge */
	BHND_DEVCLASS_MEM,	/**< internal RAM/SRAM */
	BHND_DEVCLASS_MEMC,	/**< memory controller */
	BHND_DEVCLASS_MAC,	/**< 802.3 MAC */
	BHND_DEVCLASS_PHY,	/**< 802.3 PHY */
	BHND_DEVCLASS_MPHY,	/**< 802.3 MAC/PHY */
	BHND_DEVCLASS_MAC_W,	/**< 802.11 MAC */
	BHND_DEVCLASS_PHY_W,	/**< 802.11 PHY */
	BHND_DEVCLASS_MPHY_W,	/**< 802.11 mac/phy */
	BHND_DEVCLASS_CPU,	/**< cpu core */
	BHND_DEVCLASS_SOCI,	/**< interconnect */
	BHND_DEVCLASS_SOCB,	/**< interconnect bridge/socket */
	BHND_DEVCLASS_OTHER,	/**< other / unknown */

	BHND_DEVCLASS_INVALID	/**< no/invalid class */
} bhnd_devclass_t;

/**
 * A bus device probe configuration record.
 */
struct bhnd_probecfg {
	bhnd_devclass_t	 devclass;	/**< core device class to match. */
	int		 probe_order;	/**< device probe order for this core. */
	const char	*probe_name;	/**< device name for probe, or NULL. */
};

/**
 * A hardware revision match descriptor.
 */
struct bhnd_hwrev_match {
	uint16_t	start;	/**< first revision, or BHND_HWREV_INVALID
					     to match on any revision. */
	uint16_t	end;	/**< last revision, or BHND_HWREV_INVALID
					     to match on any revision. */
};

/** A core match descriptor. */
struct bhnd_core_match {
	uint16_t		vendor;	/**< required JEP106 device vendor or BHND_MFGID_INVALID. */
	uint16_t		device;	/**< required core ID or BHND_COREID_INVALID */
	struct bhnd_hwrev_match	hwrev;	/**< matching revisions. */
	bhnd_devclass_t		class;	/**< required class or BHND_DEVCLASS_INVALID */
	int			unit;	/**< required core unit, or -1 */
};

const char		*bhnd_vendor_name(uint16_t vendor);
const char 		*bhnd_core_name(uint16_t vendor, uint16_t device);
bhnd_devclass_t		 bhnd_core_class(uint16_t vendor, uint16_t device);

bool			 bhnd_device_matches(device_t dev,
			     struct bhnd_core_match *desc);

struct bhnd_probecfg	*bhnd_find_probecfg(struct bhnd_probecfg table[],
			    uint16_t vendor, uint16_t device);

/**
 * Entry designating end of bhnd_probecfg table.
 */
#define	BHND_PROBECFG_TABLE_END		{ BHND_DEVCLASS_INVALID, 0, NULL }

/**
 * bhnd device probe priority.
 */
enum {
	BHND_PROBE_ORDER_FIRST		= 0,	/**< probe first */
	BHND_PROBE_ORDER_EARLY		= 10,	/**< probe early */
	BHND_PROBE_ORDER_DEFAULT	= 20,	/**< default probe priority */
	BHND_PROBE_ORDER_LAST		= 30,	/**< probe last */
};

/* default table */
extern struct bhnd_probecfg		bhnd_generic_probecfg_table[];

#endif /* _BHND_BHND_H_ */
