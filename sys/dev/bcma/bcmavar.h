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

#ifndef _BCMA_BCMAVAR_H_
#define _BCMA_BCMAVAR_H_

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/rman.h>

MALLOC_DECLARE(M_BCMA);

/*
 * Broadcom AMBA backplane types and data structures.
 */

/** BCMA bus address type. The backing bus supports 64-bit addressing. */
typedef uint64_t bcma_addr_t;

struct bcma_mport;
struct bcma_sport;
struct bcma_devinfo;

/**
 * Slave port types.
 */
typedef enum {
	BCMA_SPORT_TYPE_DEVICE	= 0,	/**< device/core. */
	BCMA_SPORT_TYPE_BRIDGE	= 1,	/**< bridge. */
	BCMA_SPORT_TYPE_SWRAP	= 2,	/**< DMP agent/wrapper for master port */
	BCMA_SPORT_TYPE_MWRAP	= 3,	/**< DMP agent/wrapper for slave port */
} bcma_sport_type;


int			 bcma_generic_print_child(device_t dev, device_t child);
void			 bcma_generic_probe_nomatch(device_t dev, device_t child);
int			 bcma_generic_read_ivar(device_t dev, device_t child, int index, uintptr_t *result);
int			 bcma_generic_write_ivar(device_t dev, device_t child, int index, uintptr_t value);
void			 bcma_generic_child_deleted(device_t dev, device_t child);
struct resource_list	*bcma_generic_get_resource_list(device_t dev, device_t child);

int			 bcma_scan_erom(device_t bus, struct resource *erom_res, bus_size_t erom_base);

struct bcma_devinfo	*bcma_alloc_dinfo(uint16_t designer, uint16_t core_id, uint8_t revision);
void			 bcma_free_dinfo(struct bcma_devinfo *dinfo);

struct bcma_sport	*bcma_alloc_sport(uint8_t port_num, bcma_sport_type port_type);
void			 bcma_free_sport(struct bcma_sport *sport);


/** BCMA master port descriptor */
struct bcma_mport {
	uint8_t		mp_num;		/**< AXI port identifier (bus-unique) */
	uint8_t		mp_vid;		/**< AXI master virtual ID (core-unique) */
	STAILQ_ENTRY(bcma_mport) mp_link;
};

/** BCMA memory region descriptor */
struct bcma_map {
	bcma_addr_t	m_base;	/**< base address */
	bcma_addr_t	m_size;	/**< size */
	STAILQ_ENTRY(bcma_map) m_link;
};

/** BCMA slave port descriptor */
struct bcma_sport {
	uint8_t		sp_num;		/**< slave port number (core-unique) */
	bcma_sport_type	sp_type;	/**< port type */
	uint16_t	sp_num_maps;	/**< number of regions mapped to this port. */

	STAILQ_HEAD(, bcma_map) sp_maps;
	STAILQ_ENTRY(bcma_sport) sp_link;
};

STAILQ_HEAD(bcma_mport_list, bcma_mport);
STAILQ_HEAD(bcma_sport_list, bcma_sport);

/** BCMA IP core/block configuration */
struct bcma_corecfg {
	uint16_t	designer;	/**< IP designer's JEP-106 mfgid */
	uint16_t	core_id;	/**< IP core ID/part number */
	uint8_t		revision;	/**< IP core hardware revision */

	uint16_t	num_mports;	/**< number of master port descriptors. */
	struct bcma_mport_list	mports;	/**< master port descriptors */

	uint16_t	num_dports;	/**< number of device slave port descriptors. */
	struct bcma_sport_list	dports;	/**< device port descriptors */
	
	uint16_t	num_wports;	/**< number of wrapper slave port descriptors. */	
	struct bcma_sport_list	wports;	/**< wrapper port descriptors */	
};

/**
 * BCMA per-device info
 */
struct bcma_devinfo {
	struct resource_list	resources;	/**< Slave port memory regions. */
	struct bcma_corecfg	cfg;		/**< IP core/block config */
};

#endif /* _BCMA_BCMAVAR_H_ */