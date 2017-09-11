/*-
 * Copyright (c) 2015 Alexander Kabaev
 * Copyright (c) 2006 Oleksandr Tymoshenko
 * Copyright (c) 2002-2004 Juli Mallett <jmallett@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_platform.h"
#include "opt_hwpmc_hooks.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/cpuset.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/smp.h>
#include <sys/sched.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>

#include <machine/bus.h>
#include <machine/hwfunc.h>
#include <machine/intr.h>
#include <machine/smp.h>

#ifdef FDT
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#ifndef FDT
#define	MIPS_PIC_FIXED_MAP	/**< preemptively populate fixed MIPS IRQ0-7 mappings */
#endif

#include "pic_if.h"

struct mips_pic_softc;

static int				 mips_pic_intr(void *);
static struct intr_map_data_mips	*cpu_create_intr_map(
					     struct mips_pic_softc *sc,
					     int irq);
static void				 cpu_establish_intr(
					     struct mips_pic_softc *sc,
					     const char *name,
					     driver_filter_t *filt,
					     void (*handler)(void*), void *arg,
					     int irq, int flags,
					     void **cookiep);

struct intr_map_data_mips {
	struct intr_map_data	 hdr;		/**< inherited map data */
	u_int			 mips_irq;	/**< MIPS IRQ# 0-7 */
	u_int			 intr_irq;	/**< INTRNG IRQ# */
	u_int			 consumers;	/**< INTRNG activation refcount */
	struct resource		*res;		/**< resource shared by all interrupt handlers registered via
						     cpu_establish_hardintr() or cpu_establish_softintr(); NULL
						     if no interrupt handlers are yet registered. */

	LIST_ENTRY(intr_map_data_mips) link;
};

struct mips_pic_irqsrc {
	struct intr_irqsrc	 isrc;	/**< inherited irqsrc state */
	u_int			 irq;	/**< MIPS IRQ# 0-7 */
};

struct mips_pic_softc {
	device_t			 pic_dev;
	struct mips_pic_irqsrc		 pic_isrcs[NREAL_IRQS];	/**< interrupt source entries */
	LIST_HEAD(,intr_map_data_mips)	 pic_data;		/**< MIPS IRQ mappings */
	struct mtx			 mutex;			/**< state mutex used to synchronize access to pic_data */
	u_int				 nirqs;			/**< IRQ count */
};

static struct mips_pic_softc *pic_sc;

#define PIC_INTR_ISRC(sc, irq)		(&(sc)->pic_isrcs[(irq)].isrc)

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"mti,cpu-interrupt-controller",	true},
	{NULL,					false}
};
#endif

#ifndef FDT
static void
mips_pic_identify(driver_t *drv, device_t parent)
{

	BUS_ADD_CHILD(parent, 0, "cpupic", 0);
}
#endif

static int
mips_pic_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);
#endif
	device_set_desc(dev, "MIPS32 Interrupt Controller");
	return (BUS_PROBE_DEFAULT);
}

static inline void
pic_irq_unmask(struct mips_pic_softc *sc, u_int irq)
{

	mips_wr_status(mips_rd_status() | ((1 << irq) << 8));
}

static inline void
pic_irq_mask(struct mips_pic_softc *sc, u_int irq)
{

	mips_wr_status(mips_rd_status() & ~((1 << irq) << 8));
}

static inline intptr_t
pic_xref(device_t dev)
{
#ifdef FDT
	return (OF_xref_from_node(ofw_bus_get_node(dev)));
#else
	return (0);
#endif
}

static int
mips_pic_register_isrcs(struct mips_pic_softc *sc)
{
	int error;

	for (u_int irq = 0; irq < sc->nirqs; irq++) {
		struct mips_pic_irqsrc		*isrc;
		char				*name;
		u_int				 nameirq;
#ifdef MIPS_PIC_FIXED_MAP
		struct intr_map_data_mips	*data;

		/*
		 * Non-FDT/OFW MIPS targets do not provide an equivalent to
		 * OFW_BUS_MAP_INTR(); it is instead necessary to reserve INTRNG
		 * IRQ# 0-7 for use by MIPS device drivers that assume INTRNG
		 * IRQs 0-7 are directly mapped to MIPS IRQs 0-7.
		 * 
		 * XXX: There is no support in INTRNG for reserving a fixed IRQ
		 * range; since we're the root PIC, we can work around this by
		 * iteratively allocating the required 0-7 MIP IRQ# range.
		 */
		data = cpu_create_intr_map(sc, irq);
		if (data->mips_irq != data->intr_irq) {
			panic("invalid IRQ mapping: %u->%u", data->mips_irq,
			    data->intr_irq);
		}
#endif /* MIPS_PIC_FIXED_MAP */

		/* Register our isrc for this IRQ */
		isrc = (struct mips_pic_irqsrc *)PIC_INTR_ISRC(sc, irq);
		isrc->irq = irq;

		/* sint.[0-2] or int.[0-5] */
		if (irq < NSOFT_IRQS) {
			name = "sint";
			nameirq = irq;
		} else {
			name = "int";
			nameirq = irq - NSOFT_IRQS;
		}

		error = intr_isrc_register(&isrc->isrc, sc->pic_dev, 0, "%s%u",
		    name, nameirq);
		if (error != 0) {
			for (u_int i = 0; i < irq; i++)
				intr_isrc_deregister(PIC_INTR_ISRC(sc, i));
	
			device_printf(sc->pic_dev, "%s failed", __func__);
			return (error);
		}
	}

	return (0);
}

static int
mips_pic_attach(device_t dev)
{
	struct		mips_pic_softc *sc;
	intptr_t	xref = pic_xref(dev);
	int		error;

	if (pic_sc != NULL)
		return (ENXIO);

	sc = device_get_softc(dev);
	sc->pic_dev = dev;

	/* Initialize mutex */
	mtx_init(&sc->mutex, "PIC lock", "", MTX_DEF);

	/* Set the number of interrupts */
	sc->nirqs = nitems(sc->pic_isrcs);

	LIST_INIT(&sc->pic_data);

	/* Register the interrupts */
	if ((error = mips_pic_register_isrcs(sc))) {
		device_printf(dev, "could not register PIC ISRCs: %d\n", error);
		mtx_destroy(&sc->mutex);
		return (error);
	}

	/*
	 * Now, when everything is initialized, it's right time to
	 * register interrupt controller to interrupt framefork.
	 */
	if (intr_pic_register(dev, xref) == NULL) {
		device_printf(dev, "could not register PIC\n");
		error = ENXIO;
		goto cleanup;
	}

	/* Claim our root controller role */
	if ((error = intr_pic_claim_root(dev, xref, mips_pic_intr, sc, 0))) {
		device_printf(dev, "could not set PIC as a root: %d\n", error);
		intr_pic_deregister(dev, xref);
		goto cleanup;
	}

	pic_sc = sc;
	return (0);

cleanup:
	for (u_int i = 0; i < sc->nirqs; i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	mtx_destroy(&sc->mutex);

	return (ENXIO);
}

static int
mips_pic_detach(device_t dev)
{
	struct		mips_pic_softc *sc;
	intptr_t	xref;
	int		error;

	sc = device_get_softc(dev);
	xref = pic_xref(dev);

	if ((error = intr_pic_deregister(dev, xref)))
		return (error);

	for (u_int i = 0; i < sc->nirqs; i++)
		intr_isrc_deregister(PIC_INTR_ISRC(sc, i));

	pic_sc = NULL;
	mtx_destroy(&sc->mutex);
	return (0);
}

int
mips_pic_intr(void *arg)
{
	struct mips_pic_softc *sc = arg;
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

#ifdef HWPMC_HOOKS
	if (pmc_hook && (PCPU_GET(curthread)->td_pflags & TDP_CALLCHAIN)) {
		struct trapframe *tf = PCPU_GET(curthread)->td_intr_frame;

		pmc_hook(PCPU_GET(curthread), PMC_FN_USER_CALLCHAIN, tf);
	}
#endif
	return (FILTER_HANDLED);
}

static void
mips_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq;

	irq = ((struct mips_pic_irqsrc *)isrc)->irq;
	pic_irq_mask(device_get_softc(dev), irq);
}

static void
mips_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	u_int irq;

	irq = ((struct mips_pic_irqsrc *)isrc)->irq;
	pic_irq_unmask(device_get_softc(dev), irq);
}

static int
mips_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct mips_pic_softc *sc;
	int res;

	sc = device_get_softc(dev);
	res = 0;
#ifdef FDT
	if (data->type == INTR_MAP_DATA_FDT) {
		struct intr_map_data_fdt *daf;

		daf = (struct intr_map_data_fdt *)data;

		if (daf->ncells != 1 || daf->cells[0] >= sc->nirqs)
			return (EINVAL);

		*isrcp = PIC_INTR_ISRC(sc, daf->cells[0]);
	} else
#endif
	if (data->type == INTR_MAP_DATA_MIPS) {
		struct intr_map_data_mips *mpd;

		mpd = (struct intr_map_data_mips *)data;

		if (mpd->mips_irq < 0 || mpd->mips_irq >= sc->nirqs)
			return (EINVAL);

		*isrcp = PIC_INTR_ISRC(sc, mpd->mips_irq);
	} else {
		res = ENOTSUP;
	}

	return (res);
}

static void
mips_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	mips_pic_disable_intr(dev, isrc);
}

static void
mips_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	mips_pic_enable_intr(dev, isrc);
}

static void
mips_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
}

static device_method_t mips_pic_methods[] = {
	/* Device interface */
#ifndef FDT
	DEVMETHOD(device_identify,	mips_pic_identify),
#endif
	DEVMETHOD(device_probe,		mips_pic_probe),
	DEVMETHOD(device_attach,	mips_pic_attach),
	DEVMETHOD(device_detach,	mips_pic_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_intr,	mips_pic_disable_intr),
	DEVMETHOD(pic_enable_intr,	mips_pic_enable_intr),
	DEVMETHOD(pic_map_intr,		mips_pic_map_intr),
	DEVMETHOD(pic_pre_ithread,	mips_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	mips_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	mips_pic_post_filter),

	{ 0, 0 }
};

static driver_t mips_pic_driver = {
	"cpupic",
	mips_pic_methods,
	sizeof(struct mips_pic_softc),
};

static devclass_t mips_pic_devclass;

#ifdef FDT
EARLY_DRIVER_MODULE(cpupic, ofwbus, mips_pic_driver, mips_pic_devclass, 0, 0,
    BUS_PASS_INTERRUPT);
#else
EARLY_DRIVER_MODULE(cpupic, nexus, mips_pic_driver, mips_pic_devclass, 0, 0,
    BUS_PASS_INTERRUPT);
#endif

void
cpu_init_interrupts(void)
{

}

/**
 * Return the interrupt map entry for MIPS @p irq, or NULL if no such
 * entry has been created.
 */
static struct intr_map_data_mips *
cpu_find_mips_irq_map(struct mips_pic_softc *sc, int irq)
{
	struct intr_map_data_mips *data;

	mtx_lock(&sc->mutex);
	LIST_FOREACH(data, &sc->pic_data, link) {
		if (data->mips_irq != irq)
			continue;

		mtx_unlock(&sc->mutex);
		return (data);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return the MIPS interrupt map entry for @p r, or NULL if no such entry has
 * been created.
 */
static struct intr_map_data_mips *
cpu_find_intr_irq_map(struct mips_pic_softc *sc, struct resource *r)
{
	struct intr_map_data_mips	*data;
	rman_res_t			 irq;

	irq = rman_get_start(r);
	if (irq != rman_get_end(r) || rman_get_size(r) != 1)
		return (NULL);

	mtx_lock(&sc->mutex);
	LIST_FOREACH(data, &sc->pic_data, link) {
		if (data->intr_irq != irq)
			continue;

		mtx_unlock(&sc->mutex);
		return (data);
	}

	/* Not found */
	return (NULL);
}

/**
 * Register and return an interrupt map entry for MIPS @p irq. If an entry
 * for @p irq is already registered, the existing entry will be returned.
 */
static struct intr_map_data_mips *
cpu_create_intr_map(struct mips_pic_softc *sc, int irq)
{
	struct intr_map_data_mips	*data;
	uintptr_t			 xref;

	xref = pic_xref(sc->pic_dev);

	/* Prefer existing mapping */
	if ((data = cpu_find_mips_irq_map(sc, irq)) != NULL)
		return (data);

	/* Register a new mapping */
	data = (struct intr_map_data_mips *)intr_alloc_map_data(
		INTR_MAP_DATA_MIPS, sizeof(*data), M_WAITOK | M_ZERO);
	data->mips_irq = irq;
	data->consumers = 0;
	data->res = NULL;
	data->intr_irq = intr_map_irq(sc->pic_dev, xref, &data->hdr);

	LIST_INSERT_HEAD(&sc->pic_data, data, link);

	mtx_unlock(&sc->mutex);

	return (data);
}

/**
 * If @p r references a MIPS interrupt mapped by the MIPS32 interrupt
 * controller, handle interrupt activation internally.
 *
 * Otherwise, delegate directly to intr_activate_irq().
 */
int
mips_pic_activate_intr(device_t child, struct resource *r)
{
	struct intr_map_data_mips	*data;
	int				 error;

	/* Is this one of our MIPS interrupt mappings? */
	if (pic_sc == NULL || (data = cpu_find_intr_irq_map(pic_sc, r)) == NULL)
		return (intr_activate_irq(child, r));

	/* Bump consumer count and request activation if required */
	mtx_lock(&pic_sc->mutex);
	if (data->consumers == UINT_MAX) {
		mtx_unlock(&pic_sc->mutex);
		return (ENOMEM);
	}

	if (data->consumers == 0) {
		if ((error = intr_activate_irq(child, r))) {
			mtx_unlock(&pic_sc->mutex);
			return (error);
		}
	}

	data->consumers++;
	mtx_unlock(&pic_sc->mutex);

	return (0);
}

/**
 * If @p r references a MIPS interrupt mapped by the MIPS32 interrupt
 * controller, handle interrupt deactivation internally.
 * 
 * Otherwise, delegate directly to intr_deactivate_irq().
 */
int
mips_pic_deactivate_intr(device_t child, struct resource *r)
{
	struct intr_map_data_mips	*data;
	int				 error;

	/* Is this one of our MIPS interrupt mappings? */
	if (pic_sc == NULL || (data = cpu_find_intr_irq_map(pic_sc, r)) == NULL)
		return (intr_deactivate_irq(child, r));

	/* Decrement consumer count and request deactivation if required */
	mtx_lock(&pic_sc->mutex);
	KASSERT(data->consumers > 0, ("refcount overrelease"));

	if (data->consumers == 1) {
		if ((error = intr_deactivate_irq(child, r))) {
			mtx_unlock(&pic_sc->mutex);
			return (error);
		}
	}
	data->consumers--;
	
	mtx_unlock(&pic_sc->mutex);
	return (0);
}

/**
 * Provide backwards-compatible support for registering a MIPS interrupt handler
 * directly, without allocating a bus resource. 
 */
static void
cpu_establish_intr(struct mips_pic_softc *sc, const char *name,
    driver_filter_t *filt, void (*handler)(void*), void *arg, int irq,
    int flags, void **cookiep)
{
	struct intr_map_data_mips	*data;
	struct resource			*res;
	int				 rid;
	int				 error;

	/*
	 * We have 6 levels, but thats 0 - 5 (not including 6)
	 */
	if (irq < 0 || irq >= NREAL_IRQS)
		panic("%s called for unknown intr %d", __func__, irq);

	data = cpu_create_intr_map(sc, irq);
	rid = -1;

	/* Fetch the backing resource, if any */
	mtx_lock(&sc->mutex);
	res = data->res;
	mtx_unlock(&sc->mutex);

	/* Allocate our IRQ resource */
	if (res == NULL) {
		/* Optimistically perform resource allocation */
		rid = data->intr_irq;
		res = bus_alloc_resource(sc->pic_dev, SYS_RES_IRQ, &rid,
		    data->intr_irq, data->intr_irq, 1, RF_SHAREABLE|RF_ACTIVE);

		if (res != NULL) {
			/* Try to update data->res */
			mtx_lock(&sc->mutex);
			if (data->res == NULL) {
				data->res = res;
			}
			mtx_unlock(&sc->mutex);

			/* If data->res was updated concurrently, free our local
			 * resource allocation */
			if (data->res != res) {
				bus_release_resource(sc->pic_dev, SYS_RES_IRQ,
				    rid, res);
			}
		} else {
			/* Maybe someone else allocated it? */
			mtx_lock(&sc->mutex);
			res = data->res;
			mtx_unlock(&sc->mutex);
		}
	
		if (res == NULL)
			panic("Unable to allocate IRQ %d resource", irq);
	}

	error = bus_setup_intr(sc->pic_dev, res, flags, filt, handler, arg,
	    cookiep);
	if (error)
		panic("Unable to add IRQ %d handler: %d", irq, error);
}

void
cpu_establish_hardintr(const char *name, driver_filter_t *filt,
    void (*handler)(void*), void *arg, int irq, int flags, void **cookiep)
{
	KASSERT(pic_sc != NULL, ("%s: no pic", __func__));

	if (irq < 0 || irq >= NHARD_IRQS)
		panic("%s called for unknown hard intr %d", __func__, irq);

	cpu_establish_intr(pic_sc, name, filt, handler, arg, irq+NSOFT_IRQS,
	    flags, cookiep);
}

void
cpu_establish_softintr(const char *name, driver_filter_t *filt,
    void (*handler)(void*), void *arg, int irq, int flags,
    void **cookiep)
{
	KASSERT(pic_sc != NULL, ("%s: no pic", __func__));

	if (irq < 0 || irq >= NSOFT_IRQS)
		panic("%s called for unknown soft intr %d", __func__, irq);

	cpu_establish_intr(pic_sc, name, filt, handler, arg, irq, flags,
	    cookiep);
}

