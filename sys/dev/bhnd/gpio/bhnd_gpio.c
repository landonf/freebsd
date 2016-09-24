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

#include <machine/_inttypes.h>

#include <dev/gpio/gpiobusvar.h>

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

MALLOC_DEFINE(M_BHND_GPIO, "bhnd_gpio", "bhnd gpio state");

#define	VALID_PIN(_sc, _pin)	((_pin) < (_sc)->npins)

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
	struct bhnd_gpio_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	
	sc->res = res;
	sc->npins = BHND_GPIO_NUMPINS;
	sc->bhnd_dev = device_get_parent(dev);

	BGPIO_LOCK_INIT(sc);

	sc->gpiobus_dev = gpiobus_attach_bus(dev);
	if (sc->gpiobus_dev == NULL) {
		error = ENXIO;
		goto failed;
	}

	return (0);

failed:
	device_delete_children(dev);
	BGPIO_LOCK_DESTROY(sc);

	return (error);
}

/**
 * Default bhnd_gpio driver implementation of DEVICE_DETACH().
 */
int
bhnd_gpio_detach(device_t dev)
{
	struct bhnd_gpio_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = gpiobus_detach_bus(dev)))
		return (error);

	if ((error = device_delete_children(dev)))
		return (error);

	BGPIO_LOCK_DESTROY(sc);

	return (bus_generic_detach(dev));
}

/**
 * Default bhnd_gpio driver implementation of DEVICE_SUSPEND().
 */
int
bhnd_gpio_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

/**
 * Default bhnd_gpio driver implementation of DEVICE_RESUME().
 */
int
bhnd_gpio_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

static device_t
bhnd_gpio_get_bus(device_t dev)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);
	return (sc->gpiobus_dev);
}

static int
bhnd_gpio_pin_max(device_t dev, int *maxpin)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);

	*maxpin = sc->npins - 1; /* npins - 1 == maximum pin number */
	return (0);
}

static int
bhnd_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(sc, pin))
		return (EINVAL);

	// TODO
	*caps = 0x0;
	return (0);
}

static int
bhnd_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(sc, pin))
		return (EINVAL);

	// TODO
	*flags = 0x0;
	return (0);
}

static int
bhnd_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(sc, pin))
		return (EINVAL);

	// TODO
	return (ENXIO);
}

static int
bhnd_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct bhnd_gpio_softc	*sc;
	char			 key[BHND_NVRAM_KEY_MAXLEN];
	char			 val[BHND_NVRAM_VAL_MAXLEN];
	char			*nv_name;
	int			 error;
	int			 len;

	sc = device_get_softc(dev);
	nv_name = val;

	if (!VALID_PIN(sc, pin))
		return (EINVAL);

	/* Compose the pin variable name */
	len = snprintf(key, sizeof(key), "gpio%" PRIu32, pin);
	if (len < 0)
		return (EINVAL);
	if (len >= sizeof(key))
		return (ENOMEM);

	/* Try to fetch the pin variable */
	error = bhnd_nvram_getvar_str(sc->bhnd_dev, key, val,
	    sizeof(val), NULL);

	switch (error) {
	case ENOENT:
		/* Use the generic 'gpioX' name if no explicit
		 * assignment exists */
		nv_name = key;
		break;
	case 0:
		/* Until gpio(4) grows support for something akin to aliased
		 * pin names, we can't handle 'muxed' pins with multiple
		 * assigned names */
		if (strchr(val, ',') != NULL) {
			device_printf(dev, "ignoring muxed NVRAM pin name for "
			    "gpio%" PRIu32 ": %s\n", pin, val);
			nv_name = key;
		}

		/* We're stuck with the GPIOMAXNAME limit; I'm not aware of any
		 * Broadcom pin names that will exceed this, however. */
		if (strlen(val)+1 > GPIOMAXNAME) {
			device_printf(dev, "ignoring pin name for gpio%" PRIu32
			    " (exceeds GPIOMAXNAME): %s\n", pin, val);
			nv_name = key;
		}
		break;

	default:
		/* Unexpected NVRAM error */
		device_printf(dev, "error fetching GPIO assignment '%s' from "
		    "NVRAM: %d\n", key, error);
		return (error);
	}

	strlcpy(name, nv_name, GPIOMAXNAME);
	return (0);
}

static int
bhnd_gpio_pin_get(device_t dev, uint32_t pin, uint32_t *value)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(sc, pin))
		return (EINVAL);

	// TODO
	*value = 0;
	return (0);
}

static int
bhnd_gpio_pin_set(device_t dev, uint32_t pin, uint32_t value)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(sc, pin))
		return (EINVAL);

	// TODO
	return (ENXIO);
}

static int
bhnd_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct bhnd_gpio_softc *sc = device_get_softc(dev);

	if (!VALID_PIN(sc, pin))
		return (EINVAL);

	// TODO
	return (ENXIO);
}

static device_method_t bhnd_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_gpio_probe),
	DEVMETHOD(device_detach,	bhnd_gpio_detach),
	DEVMETHOD(device_suspend,	bhnd_gpio_suspend),
	DEVMETHOD(device_resume,	bhnd_gpio_resume),


	/* GPIO interface */
	DEVMETHOD(gpio_get_bus,		bhnd_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		bhnd_gpio_pin_max),
	DEVMETHOD(gpio_pin_getcaps,	bhnd_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	bhnd_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	bhnd_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_getname,	bhnd_gpio_pin_getname),
	DEVMETHOD(gpio_pin_set,		bhnd_gpio_pin_set),
	DEVMETHOD(gpio_pin_get,		bhnd_gpio_pin_get),
	DEVMETHOD(gpio_pin_toggle,	bhnd_gpio_pin_toggle),

	DEVMETHOD_END
};

DEFINE_CLASS_0(gpio, bhnd_gpio_driver, bhnd_gpio_methods, sizeof(struct bhnd_gpio_softc));

MODULE_DEPEND(bhnd_gpio, bhnd, 1, 1, 1);
MODULE_DEPEND(bhnd_gpio, gpiobus, 1, 1, 1);
MODULE_VERSION(bhnd_gpio, 1);
