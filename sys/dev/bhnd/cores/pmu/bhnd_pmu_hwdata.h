/*
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2010 Broadcom Corporation
 * 
 * This file was initially derived from Broadcom's initial brcm80211 Linux
 * driver release of hndpmu.c, as contributed to the Linux staging repository.
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

#ifndef _BHND_CORES_PMU_BHND_HWDATA_H_
#define _BHND_CORES_PMU_BHND_HWDATA_H_

#include <dev/bhnd/bhnd.h>


/** Resource dependency operations */
typedef enum {
	BHND_PMU_DEPEND_SET	= 0,	/**< Override dependancies mask */
	BHND_PMU_DEPEND_ADD	= 1,	/**< Add to dependancies mask */
	BHND_PMU_DEPEND_REMOVE	= 2	/**< Remove from dependancies mask */
} bhnd_pmu_res_depend_op;

/**
 * PMU resource dependency override descriptor.
 */
struct bhnd_pmu_res_depend {
	uint32_t			res_mask;	/**< chip-specific resources for which this entry
							     defines dependency modifications */
     	bhnd_pmu_res_depend_op		action;		/**< dependency mask action */
	uint32_t			depend_mask;	/**< dependancy mask value */
	struct bhnd_device_match	match_req;	/**< additional device match requirements
							     that must be fufilled before applying this
							     entry, if any. */
};

/*
 * PMU resource up/down timer override descriptor.
 */
struct bhnd_pmu_res_updown {
	uint8_t		resnum;	/** resource number (BHND_PMU_RES_TABLE_SEL) */
	uint16_t	updown;	/** updown time override (BHND_PMU_RES_UPDN_TIMER) */
};

/**
 * PMU resource configuration.
 */
struct bhnd_pmu_cfg {
	const struct bhnd_device_match		 match;		/**< devices to which this configuration applies. */
	const struct bhnd_pmu_res_depend	*deps;		/**< dependency override table */
	size_t					 num_deps;	/**< number of dependency table entries */
	const struct bhnd_pmu_res_updown	*updown;	/**< updown override table */
	size_t					 num_updown;	/**< number of updown table entries */
};

#endif /* _BHND_CORES_PMU_BHND_HWDATA_H_ */
