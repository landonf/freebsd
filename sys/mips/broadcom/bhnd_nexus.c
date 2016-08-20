/*-
 * Copyright (c) 2015-2016 Landon Fuller <landonf@FreeBSD.org>
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
#include <sys/bus.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd_ids.h>

#include "bcm_machdep.h"
#include "bhnd_nexusvar.h"

/*
 * bhnd(4) driver mix-in providing shared common implementation for
 * bhnd bus drivers attached to our MIPS root nexus.
 */

/**
 * Default bhnd_nexus implementation of BHND_BUS_GET_ATTACH_TYPE().
 */
static bhnd_attach_type
bhnd_nexus_get_attach_type(device_t dev, device_t child)
{
	return (BHND_ATTACH_NATIVE);
}

/**
 * Default bhnd_nexus implementation of BHND_BUS_ACTIVATE_RESOURCE().
 */
static int
bhnd_nexus_activate_resource(device_t dev, device_t child, int type, int rid,
    struct bhnd_resource *r)
{
	int error;

	/* Always direct */
	if ((error = bus_activate_resource(child, type, rid, r->res)))
		return (error);

	r->direct = true;
	return (0);
}

/**
 * Default bhnd_nexus implementation of BHND_BUS_DEACTIVATE_RESOURCE().
 */
static int
bhnd_nexus_deactivate_resource(device_t dev, device_t child,
    int type, int rid, struct bhnd_resource *r)
{
	int error;

	/* Always direct */
	KASSERT(r->direct, ("indirect resource delegated to bhnd_nexus\n"));

	if ((error = bus_deactivate_resource(child, type, rid, r->res)))
		return (error);

	r->direct = false;
	return (0);
}

/**
 * Default bhnd_nexus implementation of BHND_BUS_IS_HW_DISABLED().
 */
static bool
bhnd_nexus_is_hw_disabled(device_t dev, device_t child)
{
	struct bcm_platform	*bp;
	struct bhnd_chipid	*cid;

	bp = bcm_get_platform();
	cid = &bp->id;

	/* The BCM4706 low-cost package leaves secondary GMAC cores
	 * floating */
	if (cid->chip_id == BHND_CHIPID_BCM4706 &&
	    cid->chip_pkg == BHND_PKGID_BCM4706L &&
	    bhnd_get_device(child) == BHND_COREID_4706_GMAC &&
	    bhnd_get_core_unit(child) != 0)
	{
		return (true);
	}

	return (false);
}

/**
 * Default bhnd_nexus implementation of BHND_BUS_GET_CHIPID().
 */
static const struct bhnd_chipid *
bhnd_nexus_get_chipid(device_t dev, device_t child) {
	return (&bcm_get_platform()->id);
}

/**
 * Default bhnd_nexus implementation of BHND_BUS_GET_INTR_COUNT().
 */
static int
bhnd_nexus_get_intr_count(device_t dev, device_t child) {
	/* Don't waste an interrupt */
	if (!device_is_enabled(child))
		return (0);

	// TODO
	return (0);
}

/**
 * Default bhnd_nexus implementation of BHND_BUS_ASSIGN_INTR().
 */
static int
bhnd_nexus_assign_intr(device_t dev, device_t child, int rid)
{
	// TODO
	return (ENXIO);
}

static device_method_t bhnd_nexus_methods[] = {
	/* bhnd interface */
	DEVMETHOD(bhnd_bus_activate_resource,	bhnd_nexus_activate_resource),
	DEVMETHOD(bhnd_bus_deactivate_resource, bhnd_nexus_deactivate_resource),
	DEVMETHOD(bhnd_bus_is_hw_disabled,	bhnd_nexus_is_hw_disabled),
	DEVMETHOD(bhnd_bus_get_attach_type,	bhnd_nexus_get_attach_type),
	DEVMETHOD(bhnd_bus_get_chipid,		bhnd_nexus_get_chipid),
	DEVMETHOD(bhnd_bus_get_intr_count,	bhnd_nexus_get_intr_count),
	DEVMETHOD(bhnd_bus_assign_intr,		bhnd_nexus_assign_intr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhnd, bhnd_nexus_driver, bhnd_nexus_methods,
   sizeof(struct bhnd_softc), bhnd_driver);
