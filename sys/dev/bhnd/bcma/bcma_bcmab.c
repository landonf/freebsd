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
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <dev/bhnd/bhndb/bhndbvar.h>

#include "bcmavar.h"

#include "bcma_eromreg.h"
#include "bcma_eromvar.h"

/*
 * Supports attachment of bcma(4) bus devices via a bcmab bridge.
 */

static int
bcma_bcmab_probe(device_t dev)
{
	int error;
	
	if ((error = bcma_probe(dev)) > 0)
		return (error);

	return (BUS_PROBE_NOWILDCARD);
}

static int
bcma_bcmab_attach(device_t dev)
{
	struct resource	*erom_res;
	bus_addr_t	erom_addr;
	int		error;
	int		rid;

	erom_addr = BHNDB_GET_ENUM_ADDR(device_get_parent(dev), dev);

	/* Map the EROM resource and enumerate our children. */
	rid = 0;
	erom_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid, erom_addr,
		erom_addr + BCMA_EROM_TABLE_SIZE, BCMA_EROM_TABLE_SIZE,
		RF_ACTIVE);
	if (erom_res == NULL) {
		device_printf(dev, "failed to allocate EROM resource\n");
		return (ENXIO);
	}

	error = bcma_add_children(dev, erom_res, BCMA_EROM_TABLE_START);

	/* Clean up */
	bus_release_resource(dev, SYS_RES_MEMORY, rid, erom_res);
	if (error)
		return (error);


	/* Call our superclass' implementation */
	return (bcma_attach(dev));
}

static device_method_t bcma_bcmab_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcma_bcmab_probe),
	DEVMETHOD(device_attach,		bcma_bcmab_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bcma, bcma_bcmab_driver, bcma_bcmab_methods,
    sizeof(struct bcma_softc), bcma_driver);

DRIVER_MODULE(bcma_bcmab, bcmab, bcma_bcmab_driver, bcma_devclass, NULL, NULL);
 
MODULE_VERSION(bcma_bcmab, 1);
MODULE_DEPEND(bcma_bcmab, bcmab, 1, 1, 1);
