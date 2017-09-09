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

#include "bcm_bmipsreg.h"

/*
 * BMIPS32 and BMIPS3300 core driver.
 *
 * These cores are only found on siba(4) chipsets, allowing
 * us to assume the availability of siba interrupt registers.
 */

static const struct bhnd_device bcm_bmips_devs[] = {
	BHND_DEVICE(BCM, MIPS33, NULL, NULL, BHND_DF_SOC),
	BHND_DEVICE_END
};

struct bcm_bmips_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			ivec;	/**< bus interrupt vector */
};

struct bcm_bmips_softc {
	device_t		 dev;
	struct resource		*mem;
	int			 mem_rid;
	struct resource		*irqs[BCM_BMIPS_NUM_INTR];	/**< CPU IRQs */
	int			 irq_rids[BCM_BMIPS_NUM_INTR];	/**< CPU IRQ resource IDs */
	struct bcm_bmips_irqsrc	 isrcs[SIBA_MAX_INTR];
};

#define PIC_INTR_ISRC(sc, irq)	(&(sc)->isrcs[(irq)].isrc)

static int
bcm_bmips_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, bcm_bmips_devs,
	    sizeof(bcm_bmips_devs[0]));
	if (id == NULL)
		return (ENXIO);

	/* Check the chip type; should only be found on siba(4) chipsets */
	if (bhnd_get_chipid(dev)->chip_type != BHND_CHIPTYPE_SIBA)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
bcm_bmips_register_isrcs(struct bcm_bmips_softc *sc)
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

static int
bcm_bmips_attach(device_t dev)
{
	struct bcm_bmips_softc	*sc;
	struct intr_pic		*pic;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	pic = NULL;

	/* Register our interrupt sources */
	if ((error = bcm_bmips_register_isrcs(sc)))
		return (error);

	/* Allocate register block */
	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		error = ENXIO;
		goto failed;
	}

	/* Allocate MIPS IRQs */
	KASSERT(nitems(sc->irq_rids) == nitems(sc->irqs),
	    ("array size mismatch"));

	for (size_t i = 0; i < nitems(sc->irq_rids); i++) {
		sc->irq_rids[i] = i;

		if ((error = bus_set_resource(dev, SYS_RES_IRQ, i, i, 1)))
			goto failed;

		sc->irqs[i] = bus_alloc_resource(dev, SYS_RES_IRQ,
		    &sc->irq_rids[i], i, i, 1, RF_ACTIVE);

		if (sc->irqs[i] == NULL) {
			device_printf(dev, "error allocating MIPS IRQ %zu\n",
			    i);
			error = ENXIO;
			goto failed;
		}
	}

	/* Register PIC */
	if ((pic = intr_pic_register(dev, 0)) == NULL) {
		device_printf(dev, "error registering PIC\n");
		error = ENXIO;
		goto failed;
	}

	device_printf(dev, "GOT IRQs\n");
	for (size_t i = 0; i < BCM_BMIPS_NUM_INTR; i++) {
		device_printf(dev, "irq[%zu] = %p\n", i, sc->irqs[i]);
	}

	return (0);

failed:
	if (pic != NULL)
		intr_pic_deregister(dev, 0);

	for (size_t i = 0; i < nitems(sc->isrcs); i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);

	for (size_t i = 0; i < nitems(sc->irqs); i++) {
		if (sc->irqs[i] == NULL)
			continue;

		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rids[i],
		    sc->irqs[i]);
	}

	return (error);
}

static int
bcm_bmips_detach(device_t dev)
{
	struct bcm_bmips_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	for (size_t i = 0; i < nitems(sc->irqs); i++) {
		if (sc->irqs[i] == NULL)
			continue;

		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rids[i],
		    sc->irqs[i]);
	}

	for (size_t i = 0; i < nitems(sc->isrcs); i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	return (0);
}

// XXX TODO
#if 0
static int
bcm_bmips_pic_intr(void *arg)
{

	struct bcm_bmips_pic_softc *sc = arg;
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
bcm_bmips_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX TODO
#if 0
	u_int irq;

	irq = ((struct bcm_bmips_irqsrc *)isrc)->irq;
	pic_irq_mask(device_get_softc(dev), irq);
#endif
}

static void
bcm_bmips_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX TODO
#if 0
	u_int irq;

	irq = ((struct bcm_bmips_irqsrc *)isrc)->irq;
	pic_irq_unmask(device_get_softc(dev), irq);
#endif
}

static int
bcm_bmips_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct bcm_bmips_softc *sc;

	sc = device_get_softc(dev);

	if (data->type != INTR_MAP_DATA_PLAT_1)
		return (ENOTSUP);

	// XXX TODO
#if 0
	struct intr_map_data_bcm_bmips_pic *mpd;

	mpd = (struct intr_map_data_bcm_bmips_pic *)data;

	if (mpd->irq < 0 || mpd->irq >= sc->nirqs)
		return (EINVAL);

	*isrcp = PIC_INTR_ISRC(sc, mpd->irq);
	return (0);
#endif

	// XXX TODO
	return (ENOTSUP);
}

static void
bcm_bmips_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX ???
	bcm_bmips_pic_disable_intr(dev, isrc);
}

static void
bcm_bmips_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX ???
	bcm_bmips_pic_enable_intr(dev, isrc);
}

static void
bcm_bmips_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
}

static device_method_t bcm_bmips_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm_bmips_probe),
	DEVMETHOD(device_attach,	bcm_bmips_attach),
	DEVMETHOD(device_detach,	bcm_bmips_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_intr,	bcm_bmips_pic_disable_intr),
	DEVMETHOD(pic_enable_intr,	bcm_bmips_pic_enable_intr),
	DEVMETHOD(pic_map_intr,		bcm_bmips_pic_map_intr),
	DEVMETHOD(pic_pre_ithread,	bcm_bmips_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	bcm_bmips_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	bcm_bmips_pic_post_filter),
	
	DEVMETHOD_END
};

static devclass_t bcm_mips_devclass;

DEFINE_CLASS_0(bcm_mips, bcm_bmips_driver, bcm_bmips_methods, sizeof(struct bcm_bmips_softc));
EARLY_DRIVER_MODULE(bcm_bmips, bhnd, bcm_bmips_driver, bcm_mips_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

MODULE_VERSION(bcm_bmips, 1);
MODULE_DEPEND(bcm_bmips, bhnd, 1, 1, 1);
