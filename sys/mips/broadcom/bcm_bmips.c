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
#include "bcm_bmipsreg.h"

/*
 * BMIPS32 and BMIPS3300 core driver.
 *
 * These cores are only found on siba(4) chipsets, allowing
 * us to assume the availability of siba interrupt registers.
 */

#define	BCM_BMIPS_NCPU_IRQS	5	/**< MIPS HW IRQs 0-4 are assignable */
#define	BCM_BMIPS_TIMER_IRQ	5	/**< MIPS HW IRQ5 is always assigned to the timer */

static const struct bhnd_device bcm_bmips_devs[] = {
	BHND_DEVICE(BCM, MIPS33, NULL, NULL, BHND_DF_SOC),
	BHND_DEVICE_END
};

struct bcm_bmips_softc {
	struct bcm_mips_softc	 bcm_mips;	/**< parent softc */
	struct resource		*cfg_res;	/**< siba cfg0 register block */
	int			 cfg_rid;	/**< cfg register resource ID */
};

static int
bcm_bmips_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, bcm_bmips_devs, sizeof(bcm_bmips_devs[0]));
	if (id == NULL)
		return (ENXIO);

	/* Check the chip type; should only be found on siba(4) chipsets */
	if (bhnd_get_chipid(dev)->chip_type != BHND_CHIPTYPE_SIBA)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}


static int
bcm_bmips_attach(device_t dev)
{
	struct bcm_bmips_softc *sc = device_get_softc(dev);

	/* Determine the resource ID for our siba CFG0 registers */
	sc->cfg_rid = bhnd_get_port_rid(dev, BHND_PORT_AGENT, 0, 0);
	if (sc->cfg_rid == -1) {
		device_printf(dev, "missing required CFG0 register block\n");
		return (ENXIO);
	}

	/* Map our core's siba-specific CFG0 register block */
	sc->cfg_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->cfg_rid,
	    RF_ACTIVE|RF_SHAREABLE);
	if (sc->cfg_res == NULL) {
		device_printf(dev, "failed to map CFG0 register block\n");
		return (ENXIO);
	}

	return (bcm_mips_attach(dev, BCM_BMIPS_NCPU_IRQS, BCM_BMIPS_TIMER_IRQ));
}

static int
bcm_bmips_detach(device_t dev)
{
	struct bcm_bmips_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bcm_mips_detach(dev)))
		return (error);

	bus_release_resource(dev, SYS_RES_MEMORY, sc->cfg_rid, sc->cfg_res);
	return (0);
}

// XXX TODO
#if 0
static int
bcm_bmips_pic_intr(void *arg)
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

/* PIC_DISABLE_INTR() */
static void
bcm_bmips_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct bcm_bmips_softc	*sc;
	struct bcm_mips_irqsrc	*irqsrc;
	
	sc = device_get_softc(dev);
	irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
#if 0
	u_int irq;

	irq = ((struct bcm_mips_irqsrc *)isrc)->irq;
	pic_irq_mask(device_get_softc(dev), irq);
#endif
}

/* PIC_ENABLE_INTR() */
static void
bcm_bmips_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct bcm_bmips_softc	*sc;
	struct bcm_mips_irqsrc	*irqsrc;
	
	sc = device_get_softc(dev);
	irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
#if 0
	u_int irq;

	irq = ((struct bcm_mips_irqsrc *)isrc)->irq;
	pic_irq_mask(device_get_softc(dev), irq);
#endif
}

/* PIC_PRE_ITHREAD() */
static void
bcm_bmips_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	bcm_bmips_pic_disable_intr(dev, isrc);
}

/* PIC_POST_ITHREAD() */
static void
bcm_bmips_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	bcm_bmips_pic_enable_intr(dev, isrc);
}

/* PIC_POST_FILTER() */
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
	DEVMETHOD(pic_pre_ithread,	bcm_bmips_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	bcm_bmips_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	bcm_bmips_pic_post_filter),

	DEVMETHOD_END
};

static devclass_t bcm_mips_devclass;

DEFINE_CLASS_1(bcm_mips, bcm_bmips_driver, bcm_bmips_methods, sizeof(struct bcm_bmips_softc), bcm_mips_driver);
EARLY_DRIVER_MODULE(bcm_bmips, bhnd, bcm_bmips_driver, bcm_mips_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

MODULE_VERSION(bcm_bmips, 1);
MODULE_DEPEND(bcm_bmips, bhnd, 1, 1, 1);
