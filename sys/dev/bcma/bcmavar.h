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

#include <sys/queue.h>

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


struct bcma_devinfo	*bcma_alloc_dinfo(uint8_t designer, uint8_t partnum,
    uint8_t revision);
void			 bcma_free_dinfo(struct bcma_devinfo *dinfo);

struct bcma_sport	*bcma_alloc_sport(uint8_t port_num,
    bcma_sport_type port_type);
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
	STAILQ_HEAD(, bcma_map) sp_maps;
	STAILQ_ENTRY(bcma_sport) sp_link;
};

STAILQ_HEAD(bcma_mport_list, bcma_mport);
STAILQ_HEAD(bcma_sport_list, bcma_sport);

/** BCMA IP core/block configuration */
struct bcma_corecfg {
	uint8_t		designer;	/**< IP designer's JEP-106 mfgid */
	uint8_t		partnum;	/**< IP core part number */
	uint8_t		revision;	/**< IP core hardware revision */

	struct bcma_mport_list	mports;	/**< master port descriptors */
	struct bcma_sport_list	sports;	/**< device port descriptors */
	struct bcma_sport_list	wports;	/**< wrapper port descriptors */	
};

/**
 * BCMA per-device info
 */
struct bcma_devinfo {
	struct bcma_corecfg	cfg;	/**< IP core/block config */
};

#endif /* _BCMA_BCMAVAR_H_ */