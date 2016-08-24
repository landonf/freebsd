/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
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
#include <sys/lock.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bcma_pl367reg.h"
#include "bcma_pl367var.h"

/*
 * BCMA ARM PL367 OOB Router driver.
 */

/* Supported device identifiers */
static const struct bhnd_device bcma_pl367_devices[] = {
	BHND_DEVICE(ARM, OOB_ROUTER, NULL, NULL),

	BHND_DEVICE_END
};

static int
bcma_pl367_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, bcma_pl367_devices,
	     sizeof(bcma_pl367_devices[0]));
	if (id == NULL)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
bcma_pl367_attach(device_t dev)
{
	struct bcma_pl367_softc	*sc;

	sc = device_get_softc(dev);

	/* Allocate register block */
	sc->mem_rid = 0;
	sc->mem_res = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "failed to allocate resources\n");
		return (ENXIO);
	}

	uint32_t bcfg;

	bcfg = bhnd_bus_read_4(sc->mem_res, BCMA_OOB_BUSCONFIG);
	device_printf(dev, "bcfg=%#x\n", bcfg);

	for (u_int bank = 0; bank < 4; bank++) {
		char ch = 'A' + bank;

		uint32_t st, en0, en1, en2, en3;

		st = bhnd_bus_read_4(sc->mem_res, BCMA_OOB_STATUSA + (bank * 4));
		en0 = bhnd_bus_read_4(sc->mem_res, BCMA_OOB_ENABLEA0 + (bank * 16));
		en1 = bhnd_bus_read_4(sc->mem_res, BCMA_OOB_ENABLEA1 + (bank * 16));
		en2 = bhnd_bus_read_4(sc->mem_res, BCMA_OOB_ENABLEA2 + (bank * 16));
		en3 = bhnd_bus_read_4(sc->mem_res, BCMA_OOB_ENABLEA3 + (bank * 16));

		printf("BANK-%c-STATUS: %#x\n", ch, st);
		printf("BANK-%c-EN0: %#x\n", ch, en0);
		printf("BANK-%c-EN1: %#x\n", ch, en1);
		printf("BANK-%c-EN2: %#x\n", ch, en2);
		printf("BANK-%c-EN3: %#x\n", ch, en3);
	}

	uint32_t p0, p1, p2, p3, p4, p5, p6, p7;
	uint32_t c0, c1, c2, c3;

	p0 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID0);
	p1 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID1);
	p2 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID2);
	p3 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID3);
	p4 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID4);
	p5 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID5);
	p6 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID6);
	p7 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_PERIPHERIALID7);

	uint16_t part = (p0 & 0x7F);
	part |= (p1 & 0x7) << 6;
	
	printf("part0=%#x, part1=%#x\n", (p0 & 0x7F), (p1 & 0x7));

	uint16_t jep106 = (p1 & 0xF0) >> 4;
	jep106 |= (p2 & 0x7) << 4;

	uint8_t rev = (p2 & 0xF0) >> 4;
	
	uint8_t mod = (p3 & 0x7);

	printf("%#x	%#x	%#x	%#x\n", p0, p1, p2, p3);
	printf("%#x	%#x	%#x	%#x\n", p4, p5, p6, p7);

	device_printf(dev, "mfg=%#hx part=%#hx rev=%hhu, mod=%hhx\n", jep106, part, rev, mod);

	c0 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_COMPONENTID0);
	c1 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_COMPONENTID1);
	c2 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_COMPONENTID2);
	c3 = bhnd_bus_read_4(sc->mem_res, BCMA_DMP_COMPONENTID3);
	
	printf("%#x	%#x	%#x	%#x\n", c0, c1, c2, c3);

	return (0);
}

static int
bcma_pl367_detach(device_t dev)
{
	struct bcma_pl367_softc	*sc;

	sc = device_get_softc(dev);

	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
	return (0);
}

static device_method_t bcma_pl367_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcma_pl367_probe),
	DEVMETHOD(device_attach,	bcma_pl367_attach),
	DEVMETHOD(device_detach,	bcma_pl367_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bcma_pl367, bcma_pl367_driver, bcma_pl367_methods,
    sizeof(struct bcma_pl367_softc));

devclass_t bcma_pl367_devclass;
EARLY_DRIVER_MODULE(bcma_pl367, bhnd, bcma_pl367_driver, bcma_pl367_devclass, 0, 0,
    BUS_PASS_INTERRUPT);

MODULE_DEPEND(bcma_pl367, bcma, 1, 1, 1);
MODULE_VERSION(bcma_pl367, 1);
