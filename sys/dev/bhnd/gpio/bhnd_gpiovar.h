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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_GPIO_BHND_GPIOVAR_H
#define _BHND_GPIO_BHND_GPIOVAR_H

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/malloc.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <sys/gpio.h>

#include <dev/bhnd/bhnd.h>

MALLOC_DECLARE(M_BHND_GPIO);
DECLARE_CLASS(bhnd_gpio_driver);

#define	BHND_GPIO_NUMPINS	32	/**< BHND GPIO pin count */

int	bhnd_gpio_probe(device_t dev);
int	bhnd_gpio_attach(device_t dev, struct bhnd_resource *res);
int	bhnd_gpio_detach(device_t dev);
int	bhnd_gpio_suspend(device_t dev);
int	bhnd_gpio_resume(device_t dev);

/**
 * bhnd_gpio driver instance state.
 */
struct bhnd_gpio_softc {
	device_t		 dev;
	device_t		 bhnd_dev;	/**< bhnd-attached parent */
	device_t		 gpiobus_dev;	/**< gpiobus child */

	struct bhnd_resource	*res;		/**< borrowed register block reference */
	int			 npins;		/**< pin count */

	struct mtx		 mtx;		/**< state mutex */
};


#define	BGPIO_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev), \
	    "BHND GPIO driver lock", MTX_DEF)
#define	BGPIO_LOCK(sc)				mtx_lock(&(sc)->mtx)
#define	BGPIO_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	BGPIO_LOCK_ASSERT(sc, what)		mtx_assert(&(sc)->mtx, what)
#define	BGPIO_LOCK_DESTROY(sc)			mtx_destroy(&(sc)->mtx)

#endif /* _BHND_GPIO_BHND_GPIOVAR_H */
