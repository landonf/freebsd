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

/*
 * ChipCommon attachment support for the bhnd(4) PMU driver.
 * 
 * Supports non-AOB ("Always-on Bus") devices that map the PMU register blocks
 * via the ChipCommon core, rather than vending a distinct PMU core on the
 * bhnd bus.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <dev/bhnd/bhnd.h>

#include <dev/bhnd/gpio/bhnd_gpiovar.h>
#include <dev/bhnd/gpio/bhnd_gpioreg.h>

#include "chipcvar.h"

static int
bhnd_gpio_chipc_probe(device_t dev)
{
	struct bhnd_gpio_softc	*sc;
	device_t		 chipc;
	int			 error;

	sc = device_get_softc(dev);

	/* Look for chipc parent */
	chipc = device_get_parent(dev);
	if (device_get_driver(chipc) != &bhnd_chipc_driver)
		return (ENXIO);

	/* Delegate to common driver implementation */
	if ((error = bhnd_gpio_probe(dev)) > 0)
		return (error);

	device_set_desc(dev, "ChipCommon GPIO");

	return (BUS_PROBE_NOWILDCARD);
}

static int
bhnd_gpio_chipc_attach(device_t dev)
{
	struct chipc_softc	*chipc_sc;
	struct bhnd_resource	*r;

	/* Fetch core registers from ChipCommon parent */
	chipc_sc = device_get_softc(device_get_parent(dev));
	r = chipc_sc->core;

	return (bhnd_gpio_attach(dev, r));
}

static device_method_t bhnd_gpio_chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_gpio_chipc_probe),
	DEVMETHOD(device_attach,		bhnd_gpio_chipc_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(gpio, bhnd_gpio_chipc_driver, bhnd_gpio_chipc_methods,
    sizeof(struct bhnd_gpio_softc), bhnd_gpio_driver);

static devclass_t bhnd_gpio_chipc_devclass;                                                                                                                                                                                              

EARLY_DRIVER_MODULE(bhnd_gpio_chipc, bhnd_chipc, bhnd_gpio_chipc_driver,
    bhnd_gpio_chipc_devclass, NULL, NULL, 
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);

MODULE_DEPEND(bhnd_gpio_chipc, bhnd, 1, 1, 1);
MODULE_DEPEND(bhnd_gpio_chipc, bhnd_gpio, 1, 1, 1);
MODULE_VERSION(bhnd_gpio_chipc, 1);
