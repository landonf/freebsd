/*-
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
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
#include <sys/lock.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/cores/chipc/chipc.h>

#include "bhnd_nvram_map.h"

#include "bhnd_pmureg.h"
#include "bhnd_pmuvar.h"

#include "bhnd_pmu_private.h"

/*
 * Broadcom PMU driver.
 * 
 * On modern BHND chipsets, the PMU, GCI, and SRENG (Save/Restore Engine?)
 * register blocks are found within a dedicated PMU core (attached via
 * the AHB 'always on bus').
 * 
 * On earlier chipsets, these register blocks are found at the same
 * offsets within the ChipCommon core.
 */

devclass_t bhnd_pmu_devclass;	/**< bhnd(4) PMU device class */

static int	bhnd_pmu_sysctl_bus_freq(SYSCTL_HANDLER_ARGS);
static int	bhnd_pmu_sysctl_cpu_freq(SYSCTL_HANDLER_ARGS);
static int	bhnd_pmu_sysctl_mem_freq(SYSCTL_HANDLER_ARGS);

static uint32_t	bhnd_pmu_read_4(bus_size_t reg, void *ctx);
static void	bhnd_pmu_write_4(bus_size_t reg, uint32_t val, void *ctx);
static uint32_t	bhnd_pmu_read_chipst(void *ctx);

static const struct bhnd_pmu_io bhnd_pmu_res_io = {
	.rd4		= bhnd_pmu_read_4,
	.wr4		= bhnd_pmu_write_4,
	.rd_chipst	= bhnd_pmu_read_chipst
};

/**
 * Default bhnd_pmu driver implementation of DEVICE_PROBE().
 */
int
bhnd_pmu_probe(device_t dev)
{
	return (BUS_PROBE_DEFAULT);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_ATTACH().
 * 
 * @param dev PMU device.
 * @param res The PMU device registers. The driver will maintain a borrowed
 * reference to this resource for the lifetime of the device.
 */
int
bhnd_pmu_attach(device_t dev, struct bhnd_resource *res)
{
	struct bhnd_pmu_softc	*sc;
	struct sysctl_ctx_list	*ctx;
	struct sysctl_oid	*tree;
	devclass_t		 bhnd_class;
	device_t		 core, bus;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = 0;
	sc->res = res;

	/* Fetch capability flags */
	sc->caps = bhnd_bus_read_4(sc->res, BHND_PMU_CAP);

	/* Find the bus and bus-attached core */
	bhnd_class = devclass_find("bhnd");
	core = sc->dev;
	while ((bus = device_get_parent(core)) != NULL) {
		if (device_get_devclass(bus) == bhnd_class)
			break;

		core = bus;
	}

	if (core == NULL) {
		device_printf(sc->dev, "bhnd bus not found\n");
		return (ENXIO);
	}

	/* Fetch chip and board info */
	sc->cid = *bhnd_get_chipid(core);

	if ((error = bhnd_read_board_info(core, &sc->board))) {
		device_printf(sc->dev, "error fetching board info: %d\n",
		    error);
		return (ENXIO);
	}

	/* Locate ChipCommon device */
	sc->chipc_dev = bhnd_bus_find_child(bus, BHND_DEVCLASS_CC, 0);
	if (sc->chipc_dev == NULL) {
		device_printf(sc->dev, "chipcommon device not found\n");
		return (ENXIO);
	}

	/* Initialize query state */
	error = bhnd_pmu_query_init(&sc->query, dev, sc->cid, &bhnd_pmu_res_io,
	    sc);
	if (error)
		return (error);
	sc->io = sc->query.io; 
	sc->io_ctx = sc->query.io_ctx;

	BPMU_LOCK_INIT(sc);

	/* Set quirk flags, including CLKCTL quirks for the PMU core itself */
	sc->quirks = 0;
	sc->quirks |= bhnd_pmu_clkctl_quirks(core);

	/* Initialize PMU */
	if ((error = bhnd_pmu_init(sc))) {
		device_printf(sc->dev, "PMU init failed: %d\n", error);
		goto failed;
	}

	/* Register ourselves with the bus */
	if ((error = bhnd_register_provider(dev, BHND_SERVICE_PMU))) {
		device_printf(sc->dev, "failed to register PMU with bus : %d\n",
		    error);
		goto failed;
	}

	/* Set up sysctl nodes */
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "bus_freq", CTLTYPE_UINT | CTLFLAG_RD, sc, 0,
	    bhnd_pmu_sysctl_bus_freq, "IU", "Bus clock frequency");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "cpu_freq", CTLTYPE_UINT | CTLFLAG_RD, sc, 0,
	    bhnd_pmu_sysctl_cpu_freq, "IU", "CPU clock frequency");
	
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mem_freq", CTLTYPE_UINT | CTLFLAG_RD, sc, 0,
	    bhnd_pmu_sysctl_mem_freq, "IU", "Memory clock frequency");

	return (0);

failed:
	BPMU_LOCK_DESTROY(sc);
	bhnd_pmu_query_fini(&sc->query);
	return (error);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_DETACH().
 */
int
bhnd_pmu_detach(device_t dev)
{
	struct bhnd_pmu_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bhnd_deregister_provider(dev, BHND_SERVICE_ANY)))
		return (error);

	BPMU_LOCK_DESTROY(sc);
	bhnd_pmu_query_fini(&sc->query);

	return (0);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_SUSPEND().
 */
int
bhnd_pmu_suspend(device_t dev)
{
	return (0);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_RESUME().
 */
int
bhnd_pmu_resume(device_t dev)
{
	struct bhnd_pmu_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	/* Re-initialize PMU */
	if ((error = bhnd_pmu_init(sc))) {
		device_printf(sc->dev, "PMU init failed: %d\n", error);
		return (error);
	}

	return (0);
}

static int
bhnd_pmu_sysctl_bus_freq(SYSCTL_HANDLER_ARGS)
{
	struct bhnd_pmu_softc	*sc;
	uint32_t		 freq;
	
	sc = arg1;

	BPMU_LOCK(sc);
	freq = bhnd_pmu_si_clock(&sc->query);
	BPMU_UNLOCK(sc);

	return (sysctl_handle_32(oidp, NULL, freq, req));
}

static int
bhnd_pmu_sysctl_cpu_freq(SYSCTL_HANDLER_ARGS)
{
	struct bhnd_pmu_softc	*sc;
	uint32_t		 freq;
	
	sc = arg1;

	BPMU_LOCK(sc);
	freq = bhnd_pmu_cpu_clock(&sc->query);
	BPMU_UNLOCK(sc);

	return (sysctl_handle_32(oidp, NULL, freq, req));
}

static int
bhnd_pmu_sysctl_mem_freq(SYSCTL_HANDLER_ARGS)
{
	struct bhnd_pmu_softc	*sc;
	uint32_t		 freq;
	
	sc = arg1;

	BPMU_LOCK(sc);
	freq = bhnd_pmu_mem_clock(&sc->query);
	BPMU_UNLOCK(sc);

	return (sysctl_handle_32(oidp, NULL, freq, req));
}

static uint32_t
bhnd_pmu_read_4(bus_size_t reg, void *ctx)
{
	struct bhnd_pmu_softc *sc = ctx;
	return (bhnd_bus_read_4(sc->res, reg));
}

static void
bhnd_pmu_write_4(bus_size_t reg, uint32_t val, void *ctx)
{
	struct bhnd_pmu_softc *sc = ctx;
	return (bhnd_bus_write_4(sc->res, reg, val));
}

static uint32_t
bhnd_pmu_read_chipst(void *ctx)
{
	struct bhnd_pmu_softc *sc = ctx;
	return (BHND_CHIPC_READ_CHIPST(sc->chipc_dev));
}

static device_method_t bhnd_pmu_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_pmu_probe),
	DEVMETHOD(device_detach,		bhnd_pmu_detach),
	DEVMETHOD(device_suspend,		bhnd_pmu_suspend),
	DEVMETHOD(device_resume,		bhnd_pmu_resume),

	/* BHND PMU interface */
	// TODO

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pmu, bhnd_pmu_driver, bhnd_pmu_methods, sizeof(struct bhnd_pmu_softc));
MODULE_VERSION(bhnd_pmu, 1);
