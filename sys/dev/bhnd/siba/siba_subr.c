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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhndvar.h>

#include "sibareg.h"
#include "sibavar.h"

/**
 * Map a siba(4) OCP vendor code to its corresponding JEDEC JEP-106 vendor
 * code.
 * 
 * @param ocp_vendor An OCP vendor code.
 * @return The BHND_MFGID constant corresponding to @p ocp_vendor, or
 * BHND_MFGID_INVALID if the OCP vendor is unknown.
 */
uint16_t
siba_get_bhnd_mfgid(uint16_t ocp_vendor)
{
	switch (ocp_vendor) {
	case OCP_VENDOR_BCM:
		return (BHND_MFGID_BCM);
	default:
		return (BHND_MFGID_INVALID);
	}
}


/**
 * Determine the number of cores available on the bus.
 * 
 * Some devices require a hardcoded core count:
 * - Earlier ChipCommon revisions (chip_rev <= 4) did not include a core count.
 * - Earlier siba(4) devices did not include a ChipCommon core at all.
 */
uint8_t
siba_get_ncores(const struct bhnd_chipid *chipid)
{
	/* Use the real count if available. */
	if (chipid->ncores > 0)
		return (chipid->ncores);

	/*
	 * The magic constants below were copied from the previous
	 * siba driver implementation; their correctness has
	 * not been verified.
	 */
	switch (chipid->chip_id) {
		case 0x4401:	/* BCM4401 PCI ID? */
		case BHND_CHIPID_BCM4402:
			return (3);

		case 0x4301:	/* BCM4031 PCI ID? */
		case 0x4307:	/* BCM4307 PCI ID? */
			return (5);
			
		case BHND_CHIPID_BCM4306:
			return (6);
			
		case BHND_CHIPID_BCM5365:
			return (7);

		case 0x4310:	/* ??? */
			return (8);

		case 0x4610:	/* BCM4610 Sentry5 PCI Card? */
		case BHND_CHIPID_BCM4704:
		case BHND_CHIPID_BCM4710:
			return (9);

		default:
			return (0);
	}
}

/**
 * Allocate and initialize new device info structure, copying the
 * provided core info.
 * 
 * @param dev The requesting bus device.
 * @param core Device core info.
 */
struct siba_devinfo *
siba_alloc_dinfo(device_t bus, const struct bhnd_core_info *core_info)
{
	struct siba_devinfo *dinfo;
	
	dinfo = malloc(sizeof(struct siba_devinfo), M_BHND, M_WAITOK);
	if (dinfo == NULL)
		return NULL;

	dinfo->core_info = *core_info;

	resource_list_init(&dinfo->resources);

	return dinfo;
}

/**
 * Deallocate the given device info structure and any associated resources.
 * 
 * @param dinfo Device info to be deallocated.
 */
void
siba_free_dinfo(struct siba_devinfo *dinfo)
{
	resource_list_free(&dinfo->resources);
	free(dinfo, M_BHND);
}


