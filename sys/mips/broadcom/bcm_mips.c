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
#include <sys/limits.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/siba/sibareg.h>

#include "pic_if.h"

#include "bcm_mipsvar.h"

#define	DENTRY(dev, fmt, ...)				\
	printf("%s(%s, " fmt ")\n", __FUNCTION__,	\
	    device_get_nameunit(dev), ##__VA_ARGS__)

/*
 * Broadcom MIPS core driver.
 *
 * Abstract driver for Broadcom MIPS CPU/PIC cores.
 */

#define	PIC_IRQ_SHARED		0	/**< MIPS CPU IRQ reserved for shared interrupt handling */

#define	PIC_INTR_ISRC(sc, irq)	(&(sc)->isrcs[(irq)].isrc)

/** return our PIC's xref */
static uintptr_t
bcm_mips_pic_xref(struct bcm_mips_softc *sc)
{
	uintptr_t xref;

	/* Determine our interrupt domain */
	xref = BHND_BUS_GET_INTR_DOMAIN(device_get_parent(sc->dev), sc->dev,
	    true);
	KASSERT(xref != 0, ("missing interrupt domain"));

	return (xref);
}

/**
 * Register all interrupt source definitions.
 */
static int
bcm_mips_register_isrcs(struct bcm_mips_softc *sc)
{
	const char	*name;
	uintptr_t	 xref;
	int		 error;

	xref = bcm_mips_pic_xref(sc);

	name = device_get_nameunit(sc->dev);
	for (size_t ivec = 0; ivec < nitems(sc->isrcs); ivec++) {
		sc->isrcs[ivec].ivec = ivec;

		error = intr_isrc_register(PIC_INTR_ISRC(sc, ivec), sc->dev,
		    xref, "%s,%u", name, ivec);
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

/**
 * Allocate required resources and initialize the given @p cpuirq state.
 * 
 * @param sc		BHND MIPS driver instance state.
 * @param cpuirq	The CPU IRQ state to be initialized.
 * @param rid		The resource ID to be assigned for the CPU IRQ resource.
 * @param irq		The MIPS HW IRQ# to be allocated.
 *
 * @retval 0		success
 * @retval non-zero	if initializing @p cpuirq otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_mips_init_cpuirq(struct bcm_mips_softc *sc, struct bcm_mips_cpuirq *cpuirq,
    int rid, u_int irq)
{
	struct resource	*res;
	int		 error;

	KASSERT(cpuirq->sc == NULL, ("cpuirq already initialized"));

	/* Must fall within MIPS HW IRQ range */
	if (irq >= NHARD_IRQS)
		return (EINVAL);

	/* HW IRQs are numbered relative to SW IRQs */
	irq += NSOFT_IRQS;

	/* Try to assign and allocate the resource */
	BCM_MIPS_LOCK(sc);

	error = bus_set_resource(sc->dev, SYS_RES_IRQ, rid, irq, 1);
	if (error) {
		BCM_MIPS_UNLOCK(sc);
		device_printf(sc->dev, "failed to assign interrupt %u: %d\n",
		    irq, error);
		return (error);
	}

	res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE|RF_ACTIVE);
	if (res == NULL) {
		BCM_MIPS_UNLOCK(sc);
		device_printf(sc->dev, "failed to allocate interrupt %u "
		    "resource\n", irq);
		bus_delete_resource(sc->dev, SYS_RES_IRQ, rid);
		return (ENXIO);
	}

	/* Initialize CPU IRQ state */
	cpuirq->sc = sc;
	cpuirq->irq_rid = rid;
	cpuirq->irq_res = res;
	cpuirq->ivec_mask = 0x0;
	cpuirq->consumers = 0;

	BCM_MIPS_UNLOCK(sc);
	return (0);
}

/**
 * Free any resources associated with the given @p cpuirq state.
 * 
 * @param sc		BHND MIPS driver instance state.
 * @param cpuirq	A CPU IRQ instance previously successfully initialized
 *			via bcm_mips_init_cpuirq().
 *
 * @retval 0		success
 * @retval non-zero	if finalizing @p cpuirq otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_mips_fini_cpuirq(struct bcm_mips_softc *sc, struct bcm_mips_cpuirq *cpuirq)
{
	int error;

	BCM_MIPS_LOCK(sc);

	if (cpuirq->sc == NULL) {
		KASSERT(cpuirq->irq_res == NULL, ("leaking cpuirq resource"));

		BCM_MIPS_UNLOCK(sc);
		return (0);	/* not initialized */
	}

	if (cpuirq->consumers != 0) {
		BCM_MIPS_UNLOCK(sc);
		return (EBUSY);
	}

	error = bus_free_resource(sc->dev, SYS_RES_IRQ, cpuirq->irq_res);
	if (error) {
		BCM_MIPS_UNLOCK(sc);
		return (error);
	}

	bus_delete_resource(sc->dev, SYS_RES_IRQ, cpuirq->irq_rid);

	BCM_MIPS_UNLOCK(sc);

	return (0);
}

/**
 * BHND MIPS device attach.
 * 
 * This should be called from subclass drivers' DEVICE_ATTACH().
 * 
 * @param dev BHND MIPS device.
 * @param num_cpu_irqs The number of usable MIPS HW IRQs.
 * @param timer_irq The MIPS HW IRQ assigned to the MIPS CPU timer.
 */
int
bcm_mips_attach(device_t dev, u_int num_cpu_irqs, u_int timer_irq)
{
	struct bcm_mips_softc	*sc;
	struct intr_pic		*pic;
	uintptr_t		 xref;
	u_int			 irq_rid;
	rman_res_t		 irq;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->nirqs = num_cpu_irqs;
	sc->timer_irq = timer_irq;

	if (sc->nirqs > nitems(sc->irqs)) {
		device_printf(dev, "%u nirqs exceeds maximum supported %zu",
		    sc->nirqs, nitems(sc->irqs));
		return (ENXIO);
	}

	pic = NULL;
	xref = bcm_mips_pic_xref(sc);

	BCM_MIPS_LOCK_INIT(sc);

	/* Register our interrupt sources */
	if ((error = bcm_mips_register_isrcs(sc))) {
		BCM_MIPS_LOCK_DESTROY(sc);
		return (error);
	}

	/* Allocate register block */
	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "failed to allocate register block\n");
		error = ENXIO;
		goto failed;
	}

	/* Initialize our CPU interrupt state */
	irq_rid = bhnd_get_intr_count(dev); /* last bhnd-assigned RID + 1 */
	irq = 0;
	for (u_int i = 0; i < sc->nirqs; i++) {
		/* Let the CPU timer have a dedicated IRQ */
		if (irq == sc->timer_irq) {
			KASSERT(sc->nirqs > 0, ("nirq underflow"));
			sc->nirqs--;
			irq++;

			if (i >= sc->nirqs)
				break;
		}

		/* Must not overflow signed resource ID representation */
		if (irq_rid >= INT_MAX) {
			device_printf(dev, "exhausted IRQ resource IDs\n");
			error = ENOMEM;
			goto failed;
		}

		/* Initialize CPU IRQ state */
		error = bcm_mips_init_cpuirq(sc, &sc->irqs[i], irq_rid, irq);
		if (error)
			goto failed;

		/* Increment IRQ and resource ID for next allocation */
		irq_rid++;
		irq++;
	}

	/* Register PIC */
	if ((pic = intr_pic_register(dev, xref)) == NULL) {
		device_printf(dev, "error registering PIC\n");
		error = ENXIO;
		goto failed;
	}

	return (0);

failed:
	/* Deregister PIC before performing any other cleanup */
	if (pic != NULL)
		intr_pic_deregister(dev, 0);

	/* Deregister all interrupt sources */
	for (size_t i = 0; i < nitems(sc->isrcs); i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	/* Free our MIPS CPU interrupt handler state */
	for (u_int i = 0; i < sc->nirqs; i++)
		bcm_mips_fini_cpuirq(sc, &sc->irqs[i]);

	/* Free our register block mapping */
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);

	BCM_MIPS_LOCK_DESTROY(sc);
	return (error);
}

int
bcm_mips_detach(device_t dev)
{
	struct bcm_mips_softc *sc;

	sc = device_get_softc(dev);

	/* Deregister PIC before performing any other cleanup */
	intr_pic_deregister(dev, 0);

	/* Deregister all interrupt sources */
	for (size_t i = 0; i < nitems(sc->isrcs); i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	/* Free our MIPS CPU interrupt handler state */
	for (u_int i = 0; i < sc->nirqs; i++)
		bcm_mips_fini_cpuirq(sc, &sc->irqs[i]);

	/* Free our register block mapping */
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);

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

/* PIC_DISABLE_INTR() */
static void
bcm_mips_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct bcm_mips_irqsrc *irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
	DENTRY(dev, "ivec=%u\n", irqsrc->ivec);

#if 0
	u_int irq;

	irq = ((struct bcm_mips_irqsrc *)isrc)->irq;
	pic_irq_mask(device_get_softc(dev), irq);
#endif
}

/* PIC_ENABLE_INTR() */
static void
bcm_mips_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct bcm_mips_irqsrc *irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
	DENTRY(dev, "ivec=%u\n", irqsrc->ivec);

#if 0
	u_int irq;

	irq = ((struct bcm_mips_irqsrc *)isrc)->irq;
	pic_irq_unmask(device_get_softc(dev), irq);
#endif
}

/* PIC_MAP_INTR() */
static int
bcm_mips_pic_map_intr(device_t dev, struct intr_map_data *d,
    struct intr_irqsrc **isrcp)
{
	struct bcm_mips_softc		*sc;
	struct bcm_mips_intr_map_data	*data;

	sc = device_get_softc(dev);

	if (d->type != INTR_MAP_DATA_BCM_MIPS) {
		DENTRY(dev, "type=%d", d->type);
		return (ENOTSUP);
	}

	data = (struct bcm_mips_intr_map_data *)d;
	DENTRY(dev, "type=%d, ivec=%u", d->type, data->ivec);
	if (data->ivec < 0 || data->ivec >= nitems(sc->isrcs))
		return (EINVAL);

	*isrcp = PIC_INTR_ISRC(sc, data->ivec);
	return (0);
}

/* PIC_ACTIVATE_INTR() */
static int
bcm_mips_pic_activate_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct bcm_mips_irqsrc *irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
	DENTRY(dev, "ivec=%u, irq=%ju", irqsrc->ivec, rman_get_start(res));

	return (0);
}

/* PIC_DEACTIVATE_INTR() */
static int
bcm_mips_pic_deactivate_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct bcm_mips_irqsrc *irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
	DENTRY(dev, "ivec=%u, irq=%ju", irqsrc->ivec, rman_get_start(res));

	return (0);
}

/* PIC_SETUP_INTR() */
static int
bcm_mips_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct bcm_mips_irqsrc *irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
	DENTRY(dev, "ivec=%u, irq=%ju", irqsrc->ivec, rman_get_start(res));

	return (ENXIO);
}

/* PIC_TEARDOWN_INTR() */
static int
bcm_mips_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct bcm_mips_irqsrc *irqsrc = (struct bcm_mips_irqsrc *)isrc;

	// XXX TODO
	DENTRY(dev, "ivec=%u, irq=%ju", irqsrc->ivec, rman_get_start(res));

	return (ENXIO);
}

/* PIC_PRE_ITHREAD() */
static void
bcm_mips_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX ???
	bcm_mips_pic_disable_intr(dev, isrc);
}

/* PIC_POST_ITHREAD() */
static void
bcm_mips_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	// XXX ???
	bcm_mips_pic_enable_intr(dev, isrc);
}

/* PIC_POST_FILTER() */
static void
bcm_mips_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
}

static device_method_t bcm_mips_methods[] = {
	/* Device interface */
	DEVMETHOD(device_detach,	bcm_mips_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_intr,	bcm_mips_pic_disable_intr),
	DEVMETHOD(pic_enable_intr,	bcm_mips_pic_enable_intr),
	DEVMETHOD(pic_map_intr,		bcm_mips_pic_map_intr),
	DEVMETHOD(pic_activate_intr,	bcm_mips_pic_activate_intr),
	DEVMETHOD(pic_deactivate_intr,	bcm_mips_pic_deactivate_intr),
	DEVMETHOD(pic_setup_intr,	bcm_mips_pic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	bcm_mips_pic_teardown_intr),
	DEVMETHOD(pic_pre_ithread,	bcm_mips_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	bcm_mips_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	bcm_mips_pic_post_filter),
	
	DEVMETHOD_END
};

DEFINE_CLASS_0(bcm_mips, bcm_mips_driver, bcm_mips_methods, sizeof(struct bcm_mips_softc));

MODULE_VERSION(bcm_mips, 1);
MODULE_DEPEND(bcm_mips, bhnd, 1, 1, 1);
