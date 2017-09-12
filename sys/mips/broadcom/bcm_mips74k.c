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
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bcma/bcma_dmp.h>

#include "bcm_machdep.h"

#include "bcm_mipsvar.h"
#include "bcm_mips74kreg.h"

/*
 * Broadcom MIPS74K Core
 *
 * These cores are only found on bcma(4) chipsets, allowing
 * us to assume the availability of bcma interrupt registers.
 */

// TODO
#if 0
static int	bcm_mips74k_route_ivec(struct bcm_mips_softc *sc,
		    u_int ivec, u_int irq);
#endif

static const struct bhnd_device bcm_mips74k_devs[] = {
	BHND_DEVICE(MIPS, MIPS74K, NULL, NULL, BHND_DF_SOC),
	BHND_DEVICE_END
};

/* Early routing of the CPU timer interrupt is required */
static void
bcm_mips74k_timer_init(void *unused)
{
	struct bcm_platform	*bp;
	u_int			 irq;
	uint32_t		 mask;

	bp = bcm_get_platform();

	/* Must be a MIPS74K core attached to a BCMA interconnect */
	if (!bhnd_core_matches(&bp->cpu_id, &(struct bhnd_core_match) {
		BHND_MATCH_CORE(BHND_MFGID_MIPS, BHND_COREID_MIPS74K)
	})) {
		if (bootverbose) {
			BCM_ERR("not a MIPS74K core: %s %s\n",
			    bhnd_vendor_name(bp->cpu_id.vendor),
			    bhnd_core_name(&bp->cpu_id));
		}

		return;
	}

	if (!BHND_CHIPTYPE_IS_BCMA_COMPATIBLE(bp->cid.chip_type)) {
		if (bootverbose)
			BCM_ERR("not a BCMA device\n");
		return;
	}

	/* Route the timer bus ivec to the CPU's timer IRQ, and disable any
	 * other vectors assigned to the IRQ. */
	irq = BCM_MIPS74K_GET_TIMER_IRQ();
	mask = BCM_MIPS74K_INTR_SEL_FLAG(BCM_MIPS74K_TIMER_IVEC);

	BCM_CPU_WRITE_4(bp, BCM_MIPS74K_INTR_SEL(irq), mask);
}

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
	return (bcm_mips_attach(dev));
}

static int
bcm_mips74k_detach(device_t dev)
{
	return (bcm_mips_detach(dev));
}

// TODO
#if 0
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
bcm_mips74k_route_ivec(struct bcm_mips_softc *sc, u_int ivec, u_int irq)
{
	uint32_t sel;

	if (irq >= BCM_MIPS74K_NUM_INTR)
		return (EINVAL);

	if (ivec >= BCMA_OOB_NUM_BUSLINES)
		return (EINVAL);

	sel = bus_read_4(sc->mem_res, BCM_MIPS74K_INTR_SEL(irq));
	sel |= BCM_MIPS74K_INTR_SEL_FLAG(ivec);
	bus_write_4(sc->mem_res, BCM_MIPS74K_INTR_SEL(irq), sel);

	return (0);
}
#endif

static device_method_t bcm_mips74k_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcm_mips74k_probe),
	DEVMETHOD(device_attach,		bcm_mips74k_attach),
	DEVMETHOD(device_detach,		bcm_mips74k_detach),
	
	DEVMETHOD_END
};

static devclass_t bcm_mips_devclass;

DEFINE_CLASS_1(bcm_mips, bcm_mips74k_driver, bcm_mips74k_methods, sizeof(struct bcm_mips_softc), bcm_mips_driver);
EARLY_DRIVER_MODULE(bcm_mips74k, bhnd, bcm_mips74k_driver, bcm_mips_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
SYSINIT(cpu_init, SI_SUB_CPU, SI_ORDER_FIRST, bcm_mips74k_timer_init, NULL);
MODULE_VERSION(bcm_mips74k, 1);
MODULE_DEPEND(bcm_mips74k, bhnd, 1, 1, 1);
