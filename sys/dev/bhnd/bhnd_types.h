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

#ifndef _BHND_BHND_TYPES_H_
#define _BHND_BHND_TYPES_H_

#include <sys/types.h>

/** BHND Device Classes. */
typedef enum {
	BHND_DEVCLASS_CC,		/**< chipcommon i/o controller */
	BHND_DEVCLASS_PCI,		/**< pci host/device bridge */
	BHND_DEVCLASS_PCIE,		/**< pcie host/device bridge */
	BHND_DEVCLASS_PCCARD,		/**< pcmcia host/device bridge */
	BHND_DEVCLASS_MEM,		/**< internal RAM/SRAM */
	BHND_DEVCLASS_MEMC,		/**< memory controller */
	BHND_DEVCLASS_ENET,		/**< 802.3 MAC/PHY */
	BHND_DEVCLASS_ENET_MAC,		/**< 802.3 MAC */
	BHND_DEVCLASS_ENET_PHY,		/**< 802.3 PHY */
	BHND_DEVCLASS_WLAN,		/**< 802.11 MAC/PHY/Radio */
	BHND_DEVCLASS_WLAN_MAC,		/**< 802.11 MAC */
	BHND_DEVCLASS_WLAN_PHY,		/**< 802.11 PHY */
	BHND_DEVCLASS_CPU,		/**< cpu core */
	BHND_DEVCLASS_SOCI,		/**< interconnect */
	BHND_DEVCLASS_SOCB,		/**< interconnect bridge/socket */
	BHND_DEVCLASS_EROM,		/**< bus device enumeration ROM */
	BHND_DEVCLASS_OTHER,		/**< other / unknown */

	BHND_DEVCLASS_INVALID		/**< no/invalid class */
} bhnd_devclass_t;

/** Evaluates to true if @p cls is a device class that can be configured
 *  as a host bridge device. */
#define	BHND_DEVCLASS_SUPPORTS_HOSTB(cls)					\
	((cls) == BHND_DEVCLASS_PCI || (cls) == BHND_DEVCLASS_PCIE ||	\
	 (cls) == BHND_DEVCLASS_PCCARD)

/**
 * BHND bus address.
 * 
 * @note While the interconnect may support 64-bit addressing, not
 * all bridges and SoC CPUs will.
 */
typedef uint64_t	bhnd_addr_t;
#define	BHND_ADDR_MAX	UINT64_MAX	/**< Maximum bhnd_addr_t value */

/** BHND bus size. */
typedef uint64_t	bhnd_size_t;
#define	BHND_SIZE_MAX	UINT64_MAX	/**< Maximum bhnd_size_t value */

/**
 * bhnd device probe priority.
 */
enum {
	BHND_PROBE_ORDER_FIRST		= 0,	/**< probe first */
	BHND_PROBE_ORDER_EARLY		= 10,	/**< probe early */
	BHND_PROBE_ORDER_DEFAULT	= 20,	/**< default probe priority */
	BHND_PROBE_ORDER_LAST		= 30,	/**< probe last */
};

/* forward declaration; private to bhndvar.h */
struct bhnd_bus_ctx;

/**
 * Chip Identification
 * 
 * This is read from the ChipCommon ID register; on earlier bhnd(4) devices
 * where ChipCommon is unavailable, known values must be supplied.
 */
struct bhnd_chipid {
	uint16_t	chip_id;	/**< chip id (BHND_CHIPID_*) */
	uint8_t		chip_rev;	/**< chip revision */
	uint8_t		chip_pkg;	/**< chip package (BHND_PKGID_*) */
	uint8_t		chip_type;	/**< chip type (BHND_CHIPTYPE_*) */

	bhnd_addr_t	enum_addr;	/**< chip_type-specific enumeration
					  *  address; either the siba(4) base
					  *  core register block, or the bcma(4)
					  *  EROM core address. */

	uint8_t		ncores;		/**< number of cores, if known. 0 if
					  *  not available. */
};

/**
* A bhnd(4) bus resource.
* 
* This provides an abstract interface to per-core resources that may require
* bus-level remapping of address windows prior to access.
*/
struct bhnd_resource {
	struct resource	*res;		/**< the system resource. */
	bool		 direct;	/**< false if the resource requires
					 *   bus window remapping before it
					 *   is MMIO accessible. */
};

/**
 * A bhnd(4) core descriptor.
 */
struct bhnd_core_info {
	uint16_t	vendor;		/**< vendor */
	uint16_t	device;		/**< device */
	uint16_t	hwrev;		/**< hardware revision */
	u_int		core_id;	/**< bus-assigned core identifier */
	int		unit;		/**< bus-assigned core unit */
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

#endif /* _BHND_BHND_TYPES_H_ */
