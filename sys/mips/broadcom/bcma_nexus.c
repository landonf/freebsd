/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/bhnd/bcma/bcma_eromreg.h>
#include <dev/bhnd/bcma/bcma_eromvar.h>

#include "bcm_machdep.h"
#include "bhnd_nexusvar.h"

#define	BCMFC_ERR(fmt, ...)	printf("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)

/* bcma-specific implementation of bcm_find_core() */
int
bcm_find_core_bcma(struct bhnd_chipid *chipid, bhnd_devclass_t devclass,
    int unit, struct bhnd_core_info *info, uintptr_t *addr)
{
	struct bcma_erom		erom;
	struct bcma_erom_core		core;
	struct bcma_erom_sport_region	region;
	bhnd_devclass_t			core_class;
	int				error;

	error = bhnd_erom_bus_space_open(&erom, NULL, mips_bus_space_generic,
	    (bus_space_handle_t) BCM_SOC_ADDR(chipid->enum_addr, 0), 0);
	if (error) {
		BCMFC_ERR("erom open failed: %d\n", error);
		return (error);
	}

	for (u_long core_index = 0; core_index < ULONG_MAX; core_index++) {
		/* Fetch next core record */
		if ((error = bcma_erom_seek_next_core(&erom)))
			return (error);

		if ((error = bcma_erom_parse_core(&erom, &core))) {
			BCMFC_ERR("core parse failed: %d\n", error);
			return (error);
		}

		/* Check for match */
		core_class = bhnd_find_core_class(core.vendor,
		    core.device);
		if (core_class != devclass)
			continue;

		/* Provide the basic core info */
		if (info != NULL)
			bcma_erom_to_core_info(&core, core_index, 0, info);

		/* Provide the core's device0.0 port address */
		error = bcma_erom_seek_core_sport_region(&erom, core_index,
		    BHND_PORT_DEVICE, 0, 0);
		if (error) {
			BCMFC_ERR("sport not found: %d\n", error);
			return (error);
		}

		if ((error = bcma_erom_parse_sport_region(&erom, &region))) {
			BCMFC_ERR("sport parse failed: %d\n", error);
			return (error);
		}

		if (addr != NULL)
			*addr = region.base_addr;

		return (0);
	}

	/* Not found */
	return (ENOENT);
}

static int
bcma_nexus_probe(device_t dev)
{
	int error;

	switch (bcm_get_platform()->id.chip_type) {
	case BHND_CHIPTYPE_BCMA:
	case BHND_CHIPTYPE_BCMA_ALT:
	case BHND_CHIPTYPE_UBUS:
		break;
	default:
		return (ENXIO);
	}

	if ((error = bcma_probe(dev)) > 0)
		return (error);

	/* Set device description */
	bhnd_set_default_bus_desc(dev, &bcm_get_platform()->id);

	return (BUS_PROBE_SPECIFIC);
}

static int
bcma_nexus_attach(device_t dev)
{
	struct bcm_platform	*bp;
	struct resource		*erom_res;
	int			 error;
	int			 rid;

	bp = bcm_get_platform();

	/* Map the EROM resource and enumerate the bus. */
	rid = 0;
	erom_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
	    bp->id.enum_addr, bp->id.enum_addr + BCMA_EROM_TABLE_SIZE,
	    BCMA_EROM_TABLE_SIZE, RF_ACTIVE);
	if (erom_res == NULL) {
		device_printf(dev, "failed to allocate EROM resource\n");
		return (ENXIO);
	}

	error = bcma_add_children(dev, erom_res, BCMA_EROM_TABLE_START);
	bus_release_resource(dev, SYS_RES_MEMORY, rid, erom_res);

	if (error)
		return (error);

	return (bcma_attach(dev));
}

static device_method_t bcma_nexus_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcma_nexus_probe),
	DEVMETHOD(device_attach,		bcma_nexus_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_2(bhnd, bcma_nexus_driver, bcma_nexus_methods,
    sizeof(struct bhnd_softc), bhnd_nexus_driver, bcma_driver);

EARLY_DRIVER_MODULE(bcma_nexus, nexus, bcma_nexus_driver, bhnd_devclass, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
