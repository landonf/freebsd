/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2017 The FreeBSD Foundation
 * All rights reserved.
 *
 * Portions of this software were developed by Landon Fuller
 * under sponsorship from the FreeBSD Foundation.
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

#include <machine/bus.h>
#include <sys/rman.h>

#include <machine/cpufunc.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bcma/bcma_dmp.h>

#include "bcm_mips74kreg.h"

/*
 * Broadcom MIPS74K Core
 *
 * These cores are only found on bcma(4) chipsets, allowing
 * us to assume the availability of bcma interrupt registers.
 */

struct bcm_mips74k_softc;

static int	bcm_mips74k_route_ivec(struct bcm_mips74k_softc *sc,
		    u_int ivec, u_int irq);

static const struct bhnd_device bcm_mips74k_devs[] = {
	BHND_DEVICE(MIPS, MIPS74K, NULL, NULL, BHND_DF_SOC),
	BHND_DEVICE_END
};

struct bcm_mips74k_softc {
	device_t		 dev;
	struct resource		*mem_res;
	int			 mem_rid;
	u_int			 timer_irq;	/**< CPU timer IRQ */
};

static int
bcm_mips74k_probe(device_t dev)
{
	const struct bhnd_device	*id;
	const struct bhnd_chipid	*cid;

	id = bhnd_device_lookup(dev, bcm_mips74k_devs,
	    sizeof(bcm_mips74k_devs[0]));
	if (id == NULL)
		return (ENXIO);

	/* Check the chip type; the MIPS74K core should only be found
	 * on bcma(4) chipsets (and we rely on bcma OOB interrupt
	 * routing). */
	cid = bhnd_get_chipid(dev);
	if (!BHND_CHIPTYPE_IS_BCMA_COMPATIBLE(cid->chip_type))
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
bcm_mips74k_attach(device_t dev)
{
	struct bcm_mips74k_softc	*sc;
	int				 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Allocate bus resources */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL)
		return (ENXIO);

	/* Fetch the assigned CPU timer IRQ */
	sc->timer_irq = (mips_rd_intctl() & MIPS_INTCTL_IPTI_MASK) >>
	    MIPS_INTCTL_IPTI_SHIFT;

	/* Route the CPU timer's bus interrupt vector */
	error = bcm_mips74k_route_ivec(sc, BCM_MIPS74K_TIMER_IVEC,
	    sc->timer_irq);
	if (error) {
		device_printf(dev, "error routing timer IRQ %u: %d\n",
		    sc->timer_irq, error);
		goto failed;
	}

	return (0);

failed:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
	return (error);
}

static int
bcm_mips74k_detach(device_t dev)
{
	struct bcm_mips74k_softc	*sc;

	sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);

	return (0);
}

/**
 * Route bus interrupt vector @p ivec to MIPS CPU @p irq.
 * 
 * @param sc	Driver state.
 * @param ivec	The OOB bus line to route to @p irq.
 * @param irq	The MIPS CPU IRQ to which @p ivec will be routed; MIPS hardware
 *		IRQs are numbered 0-5, software IRQs 6-7.
 * 
 * @retval 0		success
 * @retval EINVAL	if @p ivec or @p irq are invalid.
 */
static int
bcm_mips74k_route_ivec(struct bcm_mips74k_softc *sc, u_int ivec, u_int irq)
{
	if (irq >= BCM_MIPS74K_NUM_INTR)
		return (EINVAL);

	if (ivec >= BCMA_OOB_NUM_BUSLINES)
		return (EINVAL);

	bus_write_4(sc->mem_res, BCM_MIPS74K_INTR_SEL(irq),
	    BCM_MIPS74K_INTR_SEL_FLAG(ivec));

	return (0);
}

static device_method_t bcm_mips74k_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcm_mips74k_probe),
	DEVMETHOD(device_attach,		bcm_mips74k_attach),
	DEVMETHOD(device_detach,		bcm_mips74k_detach),
	
	DEVMETHOD_END
};

static devclass_t bcm_mips_devclass;

DEFINE_CLASS_0(bcm_mips, bcm_mips74k_driver, bcm_mips74k_methods, sizeof(struct bcm_mips74k_softc));
EARLY_DRIVER_MODULE(bcm_mips74k, bhnd, bcm_mips74k_driver, bcm_mips_devclass, 0, 0, BUS_PASS_CPU + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(bcm_mips74k, 1);
MODULE_DEPEND(bcm_mips74k, bhnd, 1, 1, 1);
