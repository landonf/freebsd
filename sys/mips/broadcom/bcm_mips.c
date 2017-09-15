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

/*
 * Broadcom MIPS core driver.
 *
 * Abstract driver for Broadcom MIPS CPU/PIC cores.
 */

static uintptr_t	bcm_mips_pic_xref(struct bcm_mips_softc *sc);
static device_t		bcm_mips_find_bhnd_parent(device_t dev);

static const int bcm_mips_debug = 1;

#define	dprintf(fmt, ...) do {			\
	if (bcm_mips_debug)			\
		printf(fmt,  ##__VA_ARGS__);	\
} while (0)

#define	DENTRY(dev, fmt, ...)	\
	dprintf("%s(%s, " fmt ")\n", __FUNCTION__,	\
	    device_get_nameunit(dev), ##__VA_ARGS__)

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
		sc->isrcs[ivec].cpuirq = NULL;
		sc->isrcs[ivec].refs = 0;

		error = intr_isrc_register(&sc->isrcs[ivec].isrc, sc->dev,
		    xref, "%s,%u", name, ivec);
		if (error) {
			for (size_t i = 0; i < ivec; i++)
				intr_isrc_deregister(&sc->isrcs[i].isrc);

			device_printf(sc->dev, "error registering IRQ %zu: "
			    "%d\n", ivec, error);
			return (error);
		}
	}

	return (0);
}

/**
 * Initialize the given @p cpuirq state as unavailable.
 * 
 * @param sc		BHND MIPS driver instance state.
 * @param cpuirq	The CPU IRQ state to be initialized.
 *
 * @retval 0		success
 * @retval non-zero	if initializing @p cpuirq otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_mips_init_cpuirq_unavail(struct bcm_mips_softc *sc,
    struct bcm_mips_cpuirq *cpuirq)
{
	BCM_MIPS_LOCK(sc);

	KASSERT(cpuirq->sc == NULL, ("cpuirq already initialized"));
	cpuirq->sc = sc;
	cpuirq->intr = 0;
	cpuirq->irq_rid = -1;
	cpuirq->irq_res = NULL;
	cpuirq->irq_cookie = NULL;
	cpuirq->ivec_mask = 0x0;
	cpuirq->refs = 0;

	BCM_MIPS_UNLOCK(sc);

	return (0);
}

/**
 * Allocate required resources and initialize the given @p cpuirq state.
 * 
 * @param sc		BHND MIPS driver instance state.
 * @param cpuirq	The CPU IRQ state to be initialized.
 * @param rid		The resource ID to be assigned for the CPU IRQ resource,
 *			or -1 if no resource should be assigned.
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

	/* Must fall within MIPS HW IRQ range */
	if (irq >= NHARD_IRQS)
		return (EINVAL);

	/* HW IRQs are numbered relative to SW IRQs */
	irq += NSOFT_IRQS;

	/* Try to assign and allocate the resource */
	BCM_MIPS_LOCK(sc);

	KASSERT(cpuirq->sc == NULL, ("cpuirq already initialized"));

	error = bus_set_resource(sc->dev, SYS_RES_IRQ, rid, irq, 1);
	if (error) {
		BCM_MIPS_UNLOCK(sc);
		device_printf(sc->dev, "failed to assign interrupt %u: "
			"%d\n",
		irq, error);
		return (error);
	}

	res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &rid,
	RF_SHAREABLE|RF_ACTIVE);
	if (res == NULL) {
		BCM_MIPS_UNLOCK(sc);
		device_printf(sc->dev, "failed to allocate interrupt "
			"%u resource\n", irq);
		bus_delete_resource(sc->dev, SYS_RES_IRQ, rid);
		return (ENXIO);
	}

	/* Initialize CPU IRQ state */
	cpuirq->sc = sc;
	cpuirq->intr = irq;
	cpuirq->irq_rid = rid;
	cpuirq->irq_res = res;
	cpuirq->irq_cookie = NULL;
	cpuirq->ivec_mask = 0x0;
	cpuirq->refs = 0;

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

	if (cpuirq->refs != 0) {
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

static int
bcm_mips_attach_default(device_t dev)
{
	/* subclassing drivers must provide an implementation of
	 * DEVICE_ATTACH() */
	panic("device_attach() unimplemented");
}

/**
 * BHND MIPS device attach.
 * 
 * This must be called from subclass drivers' DEVICE_ATTACH().
 * 
 * @param dev BHND MIPS device.
 * @param num_cpuirqs The number of usable MIPS HW IRQs.
 * @param timer_irq The MIPS HW IRQ assigned to the MIPS CPU timer.
 */
int
bcm_mips_attach(device_t dev, u_int num_cpuirqs, u_int timer_irq)
{
	struct bcm_mips_softc	*sc;
	struct intr_pic		*pic;
	uintptr_t		 xref;
	u_int			 irq_rid;
	rman_res_t		 irq;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->num_cpuirqs = num_cpuirqs;
	sc->timer_irq = timer_irq;

	/* Must not exceed the actual size of our fixed IRQ array */
	if (sc->num_cpuirqs > nitems(sc->cpuirqs)) {
		device_printf(dev, "%u nirqs exceeds maximum supported %zu",
		    sc->num_cpuirqs, nitems(sc->cpuirqs));
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

	/* Initialize our CPU interrupt state */
	irq_rid = bhnd_get_intr_count(dev); /* last bhnd-assigned RID + 1 */
	irq = 0;
	for (u_int i = 0; i < sc->num_cpuirqs; i++) {
		/* Must not overflow signed resource ID representation */
		if (irq_rid >= INT_MAX) {
			device_printf(dev, "exhausted IRQ resource IDs\n");
			error = ENOMEM;
			goto failed;
		}

		if (irq == sc->timer_irq) {
			/* Mark the CPU timer's IRQ as unavailable */
			error = bcm_mips_init_cpuirq_unavail(sc,
			    &sc->cpuirqs[i]);
		} else {
			/* Initialize state */
			error = bcm_mips_init_cpuirq(sc, &sc->cpuirqs[i],
			    irq_rid, irq);
		}

		if (error)
			goto failed;

		/* Increment IRQ and resource ID for next allocation */
		irq_rid++;
		irq++;
	}

	/* Sanity check; our shared IRQ must be available */
	if (sc->num_cpuirqs <= BCM_MIPS_IRQ_SHARED)
		panic("missing shared interrupt %d\n", BCM_MIPS_IRQ_SHARED);

	if (sc->cpuirqs[BCM_MIPS_IRQ_SHARED].irq_rid == -1)
		panic("shared interrupt %d unavailable", BCM_MIPS_IRQ_SHARED);

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
		intr_isrc_deregister(&sc->isrcs[i].isrc);

	/* Free our MIPS CPU interrupt handler state */
	for (u_int i = 0; i < sc->num_cpuirqs; i++)
		bcm_mips_fini_cpuirq(sc, &sc->cpuirqs[i]);

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
		intr_isrc_deregister(&sc->isrcs[i].isrc);

	/* Free our MIPS CPU interrupt handler state */
	for (u_int i = 0; i < sc->num_cpuirqs; i++)
		bcm_mips_fini_cpuirq(sc, &sc->cpuirqs[i]);

	return (0);
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

	*isrcp = &sc->isrcs[data->ivec].isrc;
	return (0);
}

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
 * Walk up the device tree from @p dev until we find a bhnd-attached core,
 * returning either the core, or NULL if @p dev is not attached under a bhnd
 * bus.
 */    
static device_t
bcm_mips_find_bhnd_parent(device_t dev)
{
	device_t	core, bus;
	devclass_t	bhnd_class;

	bhnd_class = devclass_find("bhnd");
	core = dev;
	while ((bus = device_get_parent(core)) != NULL) {
		if (device_get_devclass(bus) == bhnd_class)
			return (core);

		core = bus;
	}

	/* Not found */
	return (NULL);
}

/**
 * Retain @p isrc and assign a MIPS CPU interrupt on behalf of @p res; if
 * the @p isrc already has a MIPS CPU interrupt assigned, the existing
 * reference will be left unmodified.
 * 
 * @param sc		BHND MIPS driver state.
 * @param isrc		The interrupt source corresponding to @p res.
 * @param res		The interrupt resource for which a MIPS CPU IRQ will be
 *			assigned.
 * @param filter	The interrupt filter to be used if a new MIPS CPU IRQ
 *			must be assigned to @p isrc. If a MIPS CPU IRQ has
 *			already been assigned, this argument will be ignored.
 * @param arg		An opaque argument to be passed to @p filter. If a MIPS
 *			CPU IRQ has already been assigned, this argument will
 *			be ignored.
 */
int
bcm_mips_retain_cpu_intr(struct bcm_mips_softc *sc,
    struct bcm_mips_irqsrc *isrc, struct resource *res, driver_filter_t filter,
    void *arg)
{
	struct bcm_mips_cpuirq	*cpuirq;
	bhnd_devclass_t		 devclass;
	device_t		 core;
	int			 error;

	BCM_MIPS_LOCK_ASSERT(sc, MA_OWNED);

	/* Prefer existing assignment */
	if (isrc->cpuirq != NULL) {
		KASSERT(isrc->cpuirq->refs > 0, ("assigned IRQ has no "
		    "references"));

		/* Increment our reference count */
		if (isrc->refs == UINT_MAX)
			return (ENOMEM);	/* would overflow */

		isrc->refs++;
		return (0);
	}

	/* Use the device class of the bhnd core to which the interrupt
	 * vector is routed to determine whether a shared interrupt should
	 * be preferred. */
	devclass = BHND_DEVCLASS_OTHER;
	core = bcm_mips_find_bhnd_parent(rman_get_device(res));
	if (core != NULL)
		devclass = bhnd_get_class(core);

	switch (devclass) {
	case BHND_DEVCLASS_CC:
	case BHND_DEVCLASS_CC_B:
	case BHND_DEVCLASS_PMU:
	case BHND_DEVCLASS_RAM:
	case BHND_DEVCLASS_MEMC:
	case BHND_DEVCLASS_CPU:
	case BHND_DEVCLASS_SOC_ROUTER:
	case BHND_DEVCLASS_SOC_BRIDGE:
	case BHND_DEVCLASS_EROM:
	case BHND_DEVCLASS_NVRAM:
		/* Always use a shared interrupt for these devices */
		cpuirq = &sc->cpuirqs[BCM_MIPS_IRQ_SHARED];
		break;

	case BHND_DEVCLASS_PCI:
	case BHND_DEVCLASS_PCIE:
	case BHND_DEVCLASS_PCCARD:	
	case BHND_DEVCLASS_ENET:
	case BHND_DEVCLASS_ENET_MAC:
	case BHND_DEVCLASS_ENET_PHY:
	case BHND_DEVCLASS_WLAN:
	case BHND_DEVCLASS_WLAN_MAC:
	case BHND_DEVCLASS_WLAN_PHY:
	case BHND_DEVCLASS_USB_HOST:
	case BHND_DEVCLASS_USB_DEV:
	case BHND_DEVCLASS_USB_DUAL:
	case BHND_DEVCLASS_OTHER:
	case BHND_DEVCLASS_INVALID:
	default:
		/* Fall back on a shared interrupt */
		cpuirq = &sc->cpuirqs[BCM_MIPS_IRQ_SHARED];

		/* Try to assign a dedicated MIPS HW interrupt */
		for (u_int i = 0; i < sc->num_cpuirqs; i++) {
			if (i == BCM_MIPS_IRQ_SHARED)
				continue;

			if (sc->cpuirqs[i].irq_rid == -1)
				continue; /* unavailable */

			if (sc->cpuirqs[i].refs != 0)
				continue; /* already assigned */

			/* Found an unused CPU IRQ */
			cpuirq = &sc->cpuirqs[i];
			break;
		}

		break;
	}

	dprintf("assigning ivec %u to MIPS IRQ %u\n", isrc->ivec, cpuirq->intr);

	KASSERT(isrc->cpuirq == NULL, ("CPU IRQ already assigned"));
	KASSERT(isrc->refs == 0, ("isrc has active references with no "
	    "assigned CPU IRQ"));


	/* Verify that bumping the cpuirq refcount below will not overflow */
	if (cpuirq->refs == UINT_MAX)
		return (ENOMEM);

	/* If this is the first isrc attached to the CPU IRQ, we need
	 * to set up our interrupt routine */
	if (cpuirq->refs == 0) {
		KASSERT(cpuirq->irq_res != NULL, ("missing IRQ resource"));
		KASSERT(cpuirq->irq_cookie == NULL, ("already setup"));
	
		error = bus_setup_intr(sc->dev, cpuirq->irq_res,
		    INTR_TYPE_MISC | INTR_MPSAFE, filter, NULL, arg,
		    &cpuirq->irq_cookie);
		if (error) {
			printf("failed to setup internal interrupt handler: "
			    "%d\n", error);
			cpuirq->irq_cookie = NULL;

			return (error);
		}
	}

	/* Increment cpuirq refcount on behalf of the isrc */
	cpuirq->refs++;

	/* Increment isrc refcount on behalf of the caller */
	isrc->refs++;

	/* Assign the IRQ to the isrc */
	isrc->cpuirq = cpuirq;

	return (0);
}

/**
 * Release the MIPS CPU interrupt assigned to @p isrc on behalf of @p res.
 *
 * @param sc	BHND MIPS driver state.
 * @param isrc	The interrupt source corresponding to @p res.
 * @param res	The interrupt resource being activated.
 */
int
bcm_mips_release_cpu_intr(struct bcm_mips_softc *sc,
    struct bcm_mips_irqsrc *isrc, struct resource *res)
{
	struct bcm_mips_cpuirq	*cpuirq;
	int			 error;

	BCM_MIPS_LOCK_ASSERT(sc, MA_OWNED);

	/* Decrement the refcount */
	KASSERT(isrc->refs > 0, ("isrc over-release"));
	isrc->refs--;

	/* Nothing else to do if the isrc is still actively referenced */
	if (isrc->refs > 0)
		return (0);

	/* Otherwise, we need to release our CPU IRQ reference */
	cpuirq = isrc->cpuirq;
	isrc->cpuirq = NULL;

	KASSERT(cpuirq != NULL, ("no assigned IRQ"));
	KASSERT(cpuirq->refs > 0, ("cpuirq over-release"));
	cpuirq->refs--;

	/* If this is the last isrc attached to the CPU IRQ, we need
	 * to tear down our interrupt routine */
	if (cpuirq->refs == 0) {
		KASSERT(cpuirq->irq_res != NULL, ("missing IRQ resource"));
		KASSERT(cpuirq->irq_cookie != NULL, ("not setup"));
	
		error = bus_teardown_intr(sc->dev, cpuirq->irq_res,
		    cpuirq->irq_cookie);
		if (error) {
			printf("failed to teardown internal interrupt handler: "
			    "%d\n", error);
			return (error);
		}

		/* Clear now invalid cookie */
		cpuirq->irq_cookie = NULL;
	}

	return (0);
}

static device_method_t bcm_mips_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	bcm_mips_attach_default),
	DEVMETHOD(device_detach,	bcm_mips_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_map_intr,		bcm_mips_pic_map_intr),
	
	DEVMETHOD_END
};

DEFINE_CLASS_0(bcm_mips, bcm_mips_driver, bcm_mips_methods, sizeof(struct bcm_mips_softc));

MODULE_VERSION(bcm_mips, 1);
MODULE_DEPEND(bcm_mips, bhnd, 1, 1, 1);
