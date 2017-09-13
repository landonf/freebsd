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
#include <sys/bus.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <sys/rman.h>

#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/siba/sibareg.h>

#include "pic_if.h"

#include "bcm_mipsvar.h"

/*
 * Broadcom MIPS core driver.
 *
 * Abstract driver for Broadcom MIPS CPU/PIC cores.
 */

#define	PIC_INTR_ISRC(sc, irq)	(&(sc)->isrcs[(irq)].isrc)

/**
 * Register all interrupt source definitions.
 */
static int
bcm_mips_register_isrcs(struct bcm_mips_softc *sc)
{
	const char	*name;
	int		 error;

	name = device_get_nameunit(sc->dev);
	for (size_t ivec = 0; ivec < nitems(sc->isrcs); ivec++) {
		sc->isrcs[ivec].ivec = ivec;

		error = intr_isrc_register(PIC_INTR_ISRC(sc, ivec), sc->dev, 0,
		    "%s,%u", name, ivec);
		if (error) {
			for (size_t i = 0; i < ivec; i++)
				intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

			device_printf(sc->dev, "error registering IRQ %zu: "
			    "%d\n", ivec, error);
			return (error);
		}
	}

	return (0);
}

int
bcm_mips_attach(device_t dev)
{
	struct bcm_mips_softc	*sc;
	struct intr_pic		*pic;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	pic = NULL;

	/* Register our interrupt sources */
	if ((error = bcm_mips_register_isrcs(sc)))
		return (error);

	/* Allocate register block */
	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "failed to allocate register block\n");
		error = ENXIO;
		goto failed;
	}

	/* Allocate a MIPS HW IRQ for our nested interrupt handler */
	sc->irq_rid = bhnd_get_intr_count(dev); /* last bhnd-assigned RID + 1 */
	error = bus_set_resource(dev, SYS_RES_IRQ, sc->irq_rid,
	    BCM_MIPS_PIC_IRQ, 1);
	if (error) {
		device_printf(dev, "failed to set up shared interrupt: %d\n",
		    error);
	}
	
	sc->irq_rid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_SHAREABLE|RF_ACTIVE);
	if (sc->irq == NULL) {
		device_printf(dev, "failed to allocate shared interrupt\n");
		error = ENXIO;
		goto failed;
	}

	/* Register PIC */
	if ((pic = intr_pic_register(dev, 0)) == NULL) {
		device_printf(dev, "error registering PIC\n");
		error = ENXIO;
		goto failed;
	}

	return (0);

failed:
	if (pic != NULL)
		intr_pic_deregister(dev, 0);

	for (size_t i = 0; i < nitems(sc->isrcs); i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);

	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);

	return (error);
}

int
bcm_mips_detach(device_t dev)
{
	struct bcm_mips_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);

	for (size_t i = 0; i < nitems(sc->isrcs); i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	return (0);
}

// XXX TODO
#if 0
static int
bcm_mips_pic_intr(void *arg)
{

	struct bcm_mips_pic_softc *sc = arg;
	register_t cause, status;
	int i, intr;

	cause = mips_rd_cause();
	status = mips_rd_status();
	intr = (cause & MIPS_INT_MASK) >> 8;
	/*
	 * Do not handle masked interrupts. They were masked by
	 * pre_ithread function (mips_mask_XXX_intr) and will be
	 * unmasked once ithread is through with handler
	 */
	intr &= (status & MIPS_INT_MASK) >> 8;
	while ((i = fls(intr)) != 0) {
		i--; /* Get a 0-offset interrupt. */
		intr &= ~(1 << i);

		if (intr_isrc_dispatch(PIC_INTR_ISRC(sc, i),
		    curthread->td_intr_frame) != 0) {
			device_printf(sc->pic_dev,
			    "Stray interrupt %u detected\n", i);
			pic_irq_mask(sc, i);
			continue;
		}
	}

	KASSERT(i == 0, ("all interrupts handled"));

	return (FILTER_HANDLED);
}
#endif

static void
bcm_mips_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX TODO
#if 0
	u_int irq;

	irq = ((struct bcm_mips_irqsrc *)isrc)->irq;
	pic_irq_mask(device_get_softc(dev), irq);
#endif
}

static void
bcm_mips_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX TODO
#if 0
	u_int irq;

	irq = ((struct bcm_mips_irqsrc *)isrc)->irq;
	pic_irq_unmask(device_get_softc(dev), irq);
#endif
}

static int
bcm_mips_pic_map_intr(device_t dev, struct intr_map_data *d,
    struct intr_irqsrc **isrcp)
{
	struct bcm_mips_softc		*sc;
	struct bcm_mips_intr_map_data	*data;

	sc = device_get_softc(dev);

	if (d->type != INTR_MAP_DATA_BCM_MIPS)
		return (ENOTSUP);

	data = (struct bcm_mips_intr_map_data *)d;
	if (data->ivec < 0 || data->ivec >= nitems(sc->isrcs))
		return (EINVAL);

	*isrcp = PIC_INTR_ISRC(sc, data->ivec);
	return (0);
}

static void
bcm_mips_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX ???
	bcm_mips_pic_disable_intr(dev, isrc);
}

static void
bcm_mips_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX ???
	bcm_mips_pic_enable_intr(dev, isrc);
}

static void
bcm_mips_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
}

static device_method_t bcm_mips_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	bcm_mips_attach),
	DEVMETHOD(device_detach,	bcm_mips_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_intr,	bcm_mips_pic_disable_intr),
	DEVMETHOD(pic_enable_intr,	bcm_mips_pic_enable_intr),
	DEVMETHOD(pic_map_intr,		bcm_mips_pic_map_intr),
	DEVMETHOD(pic_pre_ithread,	bcm_mips_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	bcm_mips_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	bcm_mips_pic_post_filter),
	
	DEVMETHOD_END
};

DEFINE_CLASS_0(bcm_mips, bcm_mips_driver, bcm_mips_methods, sizeof(struct bcm_mips_softc));

MODULE_VERSION(bcm_mips, 1);
MODULE_DEPEND(bcm_mips, bhnd, 1, 1, 1);
