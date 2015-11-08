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

#ifndef _BCMA_BCMA_PRIVATE_H_
#define _BCMA_BCMA_PRIVATE_H_

#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/limits.h>
#include <sys/rman.h>

/*
 * Internal bcma declarations for use by bcma/bcmab implementations.
 */

MALLOC_DECLARE(M_BCMA);

/** BCMA bus address. The backing bus supports 64-bit addressing. */
typedef uint64_t	bcma_addr_t;
#define	BCMA_ADDR_MAX	UINT64_MAX	/**< Maximum bcma_addr_t value */

/** BCMA bus size. */
typedef uint64_t	bcma_size_t;
#define	BCMA_SIZE_MAX	UINT64_MAX	/**< Maximum bcma_size_t value */

/** BCMA port identifier. */
typedef u_int		bcma_pid_t;
#define BCMA_PID_MAX	UINT_MAX	/**< Maximum bcma_pid_t value */

/** BCMA per-port region map identifier. */
typedef u_int		bcma_rmid_t;
#define	BCMA_RMID_MAX	UINT_MAX	/**< Maximum bcma_rmid_t value */

struct bcma_devinfo;
struct bcma_map;
struct bcma_mport;
struct bcma_sport;

/**
 * Slave port types.
 */
typedef enum {
	BCMA_SPORT_TYPE_DEVICE	= 0,	/**< device/core. */
	BCMA_SPORT_TYPE_BRIDGE	= 1,	/**< bridge. */
	BCMA_SPORT_TYPE_SWRAP	= 2,	/**< DMP agent/wrapper for master port */
	BCMA_SPORT_TYPE_MWRAP	= 3,	/**< DMP agent/wrapper for slave port */
} bcma_sport_type;

int			 bcma_generic_probe(device_t dev);
int			 bcma_generic_attach(device_t dev);
int			 bcma_generic_detach(device_t dev);
int			 bcma_generic_read_ivar(device_t dev, device_t child,
			     int index, uintptr_t *result);
int			 bcma_generic_write_ivar(device_t dev, device_t child,
			     int index, uintptr_t value);
void			 bcma_generic_child_deleted(device_t dev,
			     device_t child);
struct resource_list	*bcma_generic_get_resource_list(device_t dev,
			     device_t child);
int			 bcma_generic_get_port_rid(device_t dev, device_t child,
			     u_int port_num, u_int region_num);

int			 bcma_add_children(device_t bus,
			     struct resource *erom_res, bus_size_t erom_offset);

const char		*bcma_port_type_name (bcma_sport_type port_type);

struct bcma_devinfo	*bcma_alloc_dinfo(u_int core_index, int core_unit,
			     uint16_t vendor, uint16_t device, uint8_t revid);
void			 bcma_free_dinfo(struct bcma_devinfo *dinfo);

struct bcma_sport	*bcma_alloc_sport(bcma_pid_t port_num, bcma_sport_type port_type);
void			 bcma_free_sport(struct bcma_sport *sport);

/** BCMA master port descriptor */
struct bcma_mport {
	bcma_pid_t	mp_num;		/**< AXI port identifier (bus-unique) */
	bcma_pid_t	mp_vid;		/**< AXI master virtual ID (core-unique) */
	STAILQ_ENTRY(bcma_mport) mp_link;
};

/** BCMA memory region descriptor */
struct bcma_map {
	bcma_rmid_t	m_region_num;	/**< region identifier (port-unique). */
	bcma_addr_t	m_base;		/**< base address */
	bcma_size_t	m_size;		/**< size */
	int		m_rid;		/**< bus resource id, or -1. */

	STAILQ_ENTRY(bcma_map) m_link;
};

/** BCMA slave port descriptor */
struct bcma_sport {
	bcma_pid_t	sp_num;		/**< slave port number (core-unique) */
	bcma_sport_type	sp_type;	/**< port type */

	u_long		sp_num_maps;	/**< number of regions mapped to this port */
	STAILQ_HEAD(, bcma_map) sp_maps;
	STAILQ_ENTRY(bcma_sport) sp_link;
};

STAILQ_HEAD(bcma_mport_list, bcma_mport);
STAILQ_HEAD(bcma_sport_list, bcma_sport);

/** BCMA IP core/block configuration */
struct bcma_corecfg {
	uint16_t	vendor;		/**< IP designer's JEP-106 mfgid */
	uint16_t	device;		/**< IP core ID/part number */
	uint8_t		revid;		/**< IP core revision identifier */
	u_int		core_index;	/**< core index (bus-unique) */
	int		core_unit;	/**< core unit number */

	u_long		num_mports;	/**< number of master port descriptors. */
	struct bcma_mport_list	mports;	/**< master port descriptors */

	u_long		num_dports;	/**< number of device slave port descriptors. */
	struct bcma_sport_list	dports;	/**< device port descriptors */
	
	u_long		num_wports;	/**< number of wrapper slave port descriptors. */	
	struct bcma_sport_list	wports;	/**< wrapper port descriptors */	
};

/**
 * BCMA per-device info
 */
struct bcma_devinfo {
	struct resource_list	resources;	/**< Slave port memory regions. */
	struct bcma_corecfg	cfg;		/**< IP core/block config */
};

#endif /* _BCMA_BCMA_PRIVATE_H_ */