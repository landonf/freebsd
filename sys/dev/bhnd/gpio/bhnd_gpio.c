/*-
 * Copyright (c) 2016 Landon Fuller <landon@landonf.org>
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
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_gpioreg.h"
#include "bhnd_gpiovar.h"

/*
 * BHND GPIO driver.
 * 
 * On modern BHND chipsets, a common GPIO register block is found within
 * the ChipCommon core.
 * 
 * On earlier chipsets, this register block is found at the same
 * offsets within either the PCI (not PCIe) core, or the EXTIF core.
 */

/**
 * Default bhnd_gpio driver implementation of DEVICE_PROBE().
 */
int
bhnd_gpio_probe(device_t dev)
{
	return (BUS_PROBE_DEFAULT);
}

/**
 * Default bhnd_gpio driver implementation of DEVICE_ATTACH().
 * 
 * @param dev GPIO device.
 * @param res The GPIO device registers. The driver will maintain a borrowed
 * reference to this resource for the lifetime of the device.
 */
int
bhnd_gpio_attach(device_t dev, struct bhnd_resource *res)
{
	// TODO
	return (ENXIO);
}

/**
 * Default bhnd_gpio driver implementation of DEVICE_DETACH().
 */
int
bhnd_gpio_detach(device_t dev)
{
	// TODO
	return (ENXIO);
}

/**
 * Default bhnd_gpio driver implementation of DEVICE_SUSPEND().
 */
int
bhnd_gpio_suspend(device_t dev)
{
	return (0);
}

/**
 * Default bhnd_gpio driver implementation of DEVICE_RESUME().
 */
int
bhnd_gpio_resume(device_t dev)
{
	return (0);
}

static device_method_t bhnd_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_gpio_probe),
	DEVMETHOD(device_detach,		bhnd_gpio_detach),
	DEVMETHOD(device_suspend,		bhnd_gpio_suspend),
	DEVMETHOD(device_resume,		bhnd_gpio_resume),

	/* GPIO interface */
	// TODO

	DEVMETHOD_END
};

DEFINE_CLASS_0(gpio, bhnd_gpio_driver, bhnd_gpio_methods, sizeof(struct bhnd_gpio_softc));

MODULE_DEPEND(bhnd_gpio, gpiobus, 1, 1, 1);
MODULE_VERSION(bhnd_gpio, 1);
