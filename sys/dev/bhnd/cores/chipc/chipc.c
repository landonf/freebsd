/*-
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
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

/*
 * Broadcom ChipCommon driver.
 * 
 * With the exception of some very early chipsets, the ChipCommon core
 * has been included in all HND SoCs and chipsets based on the siba(4) 
 * and bcma(4) interconnects, providing a common interface to chipset 
 * identification, bus enumeration, UARTs, clocks, watchdog interrupts, GPIO, 
 * flash, etc.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "chipcreg.h"
#include "chipcvar.h"

devclass_t bhnd_chipc_devclass;	/**< bhnd(4) chipcommon device class */

static const struct resource_spec chipc_rspec[CHIPC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

static struct bhnd_device_quirk chipc_quirks[];
static struct bhnd_chip_quirk chipc_chip_quirks[];

/* Supported device identifiers */
static const struct bhnd_device chipc_devices[] = {
	BHND_DEVICE(CC, "CC", chipc_quirks, chipc_chip_quirks),
	BHND_DEVICE_END
};


/* Device quirks table */
static struct bhnd_device_quirk chipc_quirks[] = {
	{ BHND_HWREV_GTE	(32),	CHIPC_QUIRK_SUPPORTS_SPROM },
	{ BHND_HWREV_GTE	(35),	CHIPC_QUIRK_SUPPORTS_NFLASH },
	BHND_DEVICE_QUIRK_END
};

/* Chip-specific quirks table */
static struct bhnd_chip_quirk chipc_chip_quirks[] = {
	/* 4331 12x9 packages */
	{{ BHND_CHIP_IP(4331, 4331TN) },
		CHIPC_QUIRK_4331_GPIO2_5_MUX_SPROM
	},
	{{ BHND_CHIP_IP(4331, 4331TNA0) },
		CHIPC_QUIRK_4331_GPIO2_5_MUX_SPROM
	},

	/* 4331 12x12 packages */
	{{ BHND_CHIP_IPR(4331, 4331TT, HWREV_GTE(1)) },
		CHIPC_QUIRK_4331_EXTPA2_MUX_SPROM
	},

	/* 4331 (all packages/revisions) */
	{{ BHND_CHIP_ID(4331) },
		CHIPC_QUIRK_4331_EXTPA_MUX_SPROM
	},

	/* 4360 family (all revs <= 2) */
	{{ BHND_CHIP_IR(4352, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },
	{{ BHND_CHIP_IR(43460, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },
	{{ BHND_CHIP_IR(43462, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },
	{{ BHND_CHIP_IR(43602, HWREV_LTE(2)) },
		CHIPC_QUIRK_4360_FEM_MUX_SPROM },

	BHND_CHIP_QUIRK_END
};

static int		chipc_nvram_attach(struct chipc_softc *sc);
static bhnd_nvram_src_t	chipc_nvram_identify(struct chipc_softc *sc);

static bool		chipc_should_enable_sprom(struct chipc_softc *sc);

/* quirk and capability flag convenience macros */
#define	CHIPC_QUIRK(_sc, _name)	\
    ((_sc)->quirks & CHIPC_QUIRK_ ## _name)
    
#define CHIPC_CAP(_sc, _name)	\
    ((_sc)->caps & CHIPC_ ## _name)

#define	CHIPC_ASSERT_QUIRK(_sc, name)	\
    KASSERT(CHIPC_QUIRK((_sc), name), ("quirk " __STRING(_name) " not set"))

#define	CHIPC_ASSERT_CAP(_sc, name)	\
    KASSERT(CHIPC_CAP((_sc), name), ("capability " __STRING(_name) " not set"))

static int
chipc_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, chipc_devices, sizeof(chipc_devices[0]));
	if (id == NULL)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
chipc_attach(device_t dev)
{
	struct chipc_softc		*sc;
	bhnd_addr_t			 enum_addr;
	uint32_t			 ccid_reg;
	uint8_t				 chip_type;
	int				 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = bhnd_device_quirks(dev, chipc_devices,
	    sizeof(chipc_devices[0]));
	sc->sprom_refcnt = 0;

	CHIPC_LOCK_INIT(sc);

	/* Allocate bus resources */
	memcpy(sc->rspec, chipc_rspec, sizeof(sc->rspec));
	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	sc->core = sc->res[0];

	/* Map just the primary ChipCommon register block, leaving the
	 * remainder for our children */
	error = bus_adjust_resource(dev, SYS_RES_MEMORY, sc->core->res,
	    rman_get_start(sc->core->res), 
	    rman_get_start(sc->core->res) + CHIPC_CHIPID_SIZE - 1);
	if (error) {
		device_printf(dev, "failed to resize register block: %d\n",
		    error);
		goto cleanup;
	}

	/* Fetch our chipset identification data */
	ccid_reg = bhnd_bus_read_4(sc->core, CHIPC_ID);
	chip_type = CHIPC_GET_ATTR(ccid_reg, ID_BUS);

	switch (chip_type) {
	case BHND_CHIPTYPE_SIBA:
		/* enumeration space starts at the ChipCommon register base. */
		enum_addr = rman_get_start(sc->core->res);
		break;
	case BHND_CHIPTYPE_BCMA:
	case BHND_CHIPTYPE_BCMA_ALT:
		enum_addr = bhnd_bus_read_4(sc->core, CHIPC_EROMPTR);
		break;
	default:
		device_printf(dev, "unsupported chip type %hhu\n", chip_type);
		error = ENODEV;
		goto cleanup;
	}

	sc->ccid = bhnd_parse_chipid(ccid_reg, enum_addr);

	/* Fetch capability and status register values */
	sc->caps = bhnd_bus_read_4(sc->core, CHIPC_CAPABILITIES);
	sc->cst = bhnd_bus_read_4(sc->core, CHIPC_CHIPST);

	/* Identify NVRAM source and add child device. */
	sc->nvram_src = chipc_nvram_identify(sc);
	if ((error = chipc_nvram_attach(sc)))
		goto cleanup;

	/* Standard bus probe */
	if ((error = bus_generic_attach(dev)))
		goto cleanup;

	return (0);
	
cleanup:
	bhnd_release_resources(dev, sc->rspec, sc->res);
	CHIPC_LOCK_DESTROY(sc);
	return (error);
}

static int
chipc_detach(device_t dev)
{
	struct chipc_softc	*sc;
	int			 error;

	if ((error = bus_generic_detach(dev)))
		return (error);

	sc = device_get_softc(dev);
	bhnd_release_resources(dev, sc->rspec, sc->res);
	bhnd_sprom_fini(&sc->sprom);

	CHIPC_LOCK_DESTROY(sc);

	return (0);
}

static int
chipc_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

static int
chipc_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

static void
chipc_probe_nomatch(device_t dev, device_t child)
{
	struct resource_list	*rl;
	const char		*name;

	name = device_get_name(child);
	if (name == NULL)
		name = "unknown device";

	device_printf(dev, "<%s> at", name);

	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl != NULL) {
		resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#jx");
		resource_list_print_type(rl, "irq", SYS_RES_IRQ, "%jd");
	}

	printf(" (no driver attached)\n");
}

static int
chipc_print_child(device_t dev, device_t child)
{
	struct resource_list	*rl;
	int			 retval = 0;

	retval += bus_print_child_header(dev, child);

	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl != NULL) {
		retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY,
		    "%#jx");
		retval += resource_list_print_type(rl, "irq", SYS_RES_IRQ,
		    "%jd");
	}

	retval += bus_print_child_domain(dev, child);
	retval += bus_print_child_footer(dev, child);

	return (retval);
}

static int
chipc_child_pnpinfo_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	if (buflen == 0)
		return (EOVERFLOW);

	*buf = '\0';
	return (0);
}

static int
chipc_child_location_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{
	if (buflen == 0)
		return (EOVERFLOW);

	*buf = '\0';
	return (ENXIO);
}

static device_t
chipc_add_child(device_t dev, u_int order, const char *name, int unit)
{
	struct chipc_devinfo	*dinfo;
	device_t		 child;

	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return (NULL);

	dinfo = malloc(sizeof(struct chipc_devinfo), M_DEVBUF, M_NOWAIT);
	if (dinfo == NULL) {
		device_delete_child(dev, child);
		return (NULL);
	}

	resource_list_init(&dinfo->resources);

	device_set_ivars(child, dinfo);

	return (child);
}

static void
chipc_child_deleted(device_t dev, device_t child)
{
	struct chipc_devinfo *dinfo = device_get_ivars(child);

	if (dinfo != NULL) {
		resource_list_free(&dinfo->resources);
		free(dinfo, M_DEVBUF);
	}

	device_set_ivars(child, NULL);
}

static struct resource_list *
chipc_get_resource_list(device_t dev, device_t child)
{
	struct chipc_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

/**
 * If supported, add an appropriate NVRAM child device.
 */
static int
chipc_nvram_attach(struct chipc_softc *sc)
{
	device_t	 nvram_dev;
	rman_res_t	 start, end;
	int		 error;

	switch (sc->nvram_src) {
	case BHND_NVRAM_SRC_OTP:
	case BHND_NVRAM_SRC_SPROM:
		/* Add OTP/SPROM device */
		nvram_dev = BUS_ADD_CHILD(sc->dev, 0, "bhnd_nvram", -1);
		if (nvram_dev == NULL) {
			device_printf(sc->dev, "failed to add NVRAM device\n");
			return (ENXIO);
		}

		start = rman_get_start(sc->core->res) + CHIPC_SPROM_OTP;
		end = start + CHIPC_SPROM_OTP_SIZE - 1;
		error = bus_set_resource(nvram_dev, SYS_RES_MEMORY, 0, start,
		    end);
		return (error);
		
	case BHND_NVRAM_SRC_NFLASH:
		// TODO (requires access to NFLASH hardware)
		device_printf(sc->dev, "NVRAM-NFLASH unsupported\n");
		return (0);

	case BHND_NVRAM_SRC_UNKNOWN:
		/* Handled externally */
		return (0);

	default:
		device_printf(sc->dev, "invalid nvram source: %u\n",
		     sc->nvram_src);
		return (ENXIO);
	}
}

/**
 * Determine the NVRAM data source for this device.
 *
 * @param sc chipc driver state.
 */
static bhnd_nvram_src_t
chipc_nvram_identify(struct chipc_softc *sc)
{
	uint32_t		 srom_ctrl;

	/* Very early devices vend SPROM/OTP/CIS (if at all) via the
	 * host bridge interface instead of ChipCommon. */
	if (!CHIPC_QUIRK(sc, SUPPORTS_SPROM))
		return (BHND_NVRAM_SRC_UNKNOWN);

	/*
	 * Later chipset revisions standardized the SPROM capability flags and
	 * register interfaces.
	 * 
	 * We check for hardware presence in order of precedence. For example,
	 * SPROM is is always used in preference to internal OTP if found.
	 */
	if (CHIPC_CAP(sc, CAP_SPROM)) {
		srom_ctrl = bhnd_bus_read_4(sc->core, CHIPC_SPROM_CTRL);
		if (srom_ctrl & CHIPC_SRC_PRESENT)
			return (BHND_NVRAM_SRC_SPROM);
	}

	/* Check for OTP */
	if (CHIPC_CAP(sc, CAP_OTP_SIZE))
		return (BHND_NVRAM_SRC_OTP);

	/*
	 * Finally, Northstar chipsets (and possibly other chipsets?) support
	 * external NAND flash. 
	 */
	if (CHIPC_QUIRK(sc, SUPPORTS_NFLASH) && CHIPC_CAP(sc, CAP_NFLASH))
		return (BHND_NVRAM_SRC_NFLASH);

	/* No NVRAM hardware capability declared */
	return (BHND_NVRAM_SRC_UNKNOWN);
}

/**
 * Eexamine bus state and make a best effort determination of whether it's
 * likely safe to enable the muxed SPROM pins.
 * 
 * On devices that do not use SPROM pin muxing, always returns true.
 * 
 * @param sc chipc driver state.
 */
static bool
chipc_should_enable_sprom(struct chipc_softc *sc)
{
	device_t	*devs;
	device_t	 hostb;
	device_t	 parent;
	int		 devcount;
	int		 error;
	bool		 result;

	mtx_assert(&Giant, MA_OWNED);	/* for newbus */

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return (true);

	parent = device_get_parent(sc->dev);
	hostb = bhnd_find_hostb_device(parent);

	if ((error = device_get_children(parent, &devs, &devcount)))
		return (false);

	/* Reject any active devices other than ChipCommon, or the
	 * host bridge (if any). */
	result = true;
	for (int i = 0; i < devcount; i++) {
		if (devs[i] == hostb || devs[i] == sc->dev)
			continue;

		if (!device_is_attached(devs[i]))
			continue;

		if (device_is_suspended(devs[i]))
			continue;

		/* Active device; assume SPROM is busy */
		result = false;
		break;
	}

	free(devs, M_TEMP);
	return (result);
}

/**
 * If required by this device, enable access to the SPROM.
 * 
 * @param sc chipc driver state.
 */
static int
chipc_enable_sprom_pins(device_t dev)
{
	struct chipc_softc	*sc;
	uint32_t		 cctrl;
	int			 error;

	sc = device_get_softc(dev);

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return (0);

	/* Make sure we're holding Giant for newbus */
	mtx_lock(&Giant);
	CHIPC_LOCK(sc);

	/* Already enabled? */
	if (sc->sprom_refcnt >= 1) {
		error = 0;
		goto finished;
	}

	/* Check whether bus is busy */
	if (!chipc_should_enable_sprom(sc)) {
		error = EBUSY;
		goto finished;
	}

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);

	/* 4331 devices */
	if (CHIPC_QUIRK(sc, 4331_EXTPA_MUX_SPROM)) {
		cctrl &= ~CHIPC_CCTRL4331_EXTPA_EN;

		if (CHIPC_QUIRK(sc, 4331_GPIO2_5_MUX_SPROM))
			cctrl &= ~CHIPC_CCTRL4331_EXTPA_ON_GPIO2_5;

		if (CHIPC_QUIRK(sc, 4331_EXTPA2_MUX_SPROM))
			cctrl &= ~CHIPC_CCTRL4331_EXTPA_EN2;

		bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);
		error = 0;
		goto finished;
	}

	/* 4360 devices */
	if (CHIPC_QUIRK(sc, 4360_FEM_MUX_SPROM)) {
		/* Unimplemented */
	}

	/* Refuse to proceed on unsupported devices with muxed SPROM pins */
	device_printf(sc->dev, "muxed sprom lines on unrecognized device\n");
	error = ENXIO;

finished:
	/* Bump the reference count */
	if (error == 0)
		sc->sprom_refcnt++;

	CHIPC_UNLOCK(sc);
	mtx_unlock(&Giant);

	return (error);
}

/**
 * If required by this device, revert any GPIO/pin configuration applied
 * to allow SPROM access.
 * 
 * @param sc chipc driver state.
 */
static void
chipc_disable_sprom_pins(device_t dev)
{
	struct chipc_softc	*sc;
	uint32_t		 cctrl;

	sc = device_get_softc(dev);

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return;

	CHIPC_LOCK(sc);

	/* Check reference count, skip disable if in-use. */
	KASSERT(sc->sprom_refcnt > 0, ("sprom refcnt overrelease"));
	sc->sprom_refcnt--;
	if (sc->sprom_refcnt > 0)
		goto finished;

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);

	/* 4331 devices */
	if (CHIPC_QUIRK(sc, 4331_EXTPA_MUX_SPROM)) {
		cctrl |= CHIPC_CCTRL4331_EXTPA_EN;

		if (CHIPC_QUIRK(sc, 4331_GPIO2_5_MUX_SPROM))
			cctrl |= CHIPC_CCTRL4331_EXTPA_ON_GPIO2_5;

		if (CHIPC_QUIRK(sc, 4331_EXTPA2_MUX_SPROM))
			cctrl |= CHIPC_CCTRL4331_EXTPA_EN2;

		bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);
		goto finished;
	}

	/* 4360 devices */
	if (CHIPC_QUIRK(sc, 4360_FEM_MUX_SPROM)) {
		/* Unimplemented */
	}

finished:
	CHIPC_UNLOCK(sc);
}

static bhnd_nvram_src_t
chipc_nvram_src(device_t dev)
{
	struct chipc_softc *sc = device_get_softc(dev);
	return (sc->nvram_src);
}

static void
chipc_write_chipctrl(device_t dev, uint32_t value, uint32_t mask)
{
	struct chipc_softc	*sc;
	uint32_t		 cctrl;

	sc = device_get_softc(dev);

	CHIPC_LOCK(sc);

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);
	cctrl = (cctrl & ~mask) | (value | mask);
	bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);

	CHIPC_UNLOCK(sc);
}

static device_method_t chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			chipc_probe),
	DEVMETHOD(device_attach,		chipc_attach),
	DEVMETHOD(device_detach,		chipc_detach),
	DEVMETHOD(device_suspend,		chipc_suspend),
	DEVMETHOD(device_resume,		chipc_resume),

	/* Bus interface */
	DEVMETHOD(bus_probe_nomatch,		chipc_probe_nomatch),
	DEVMETHOD(bus_print_child,		chipc_print_child),
	DEVMETHOD(bus_child_pnpinfo_str,	chipc_child_pnpinfo_str),
	DEVMETHOD(bus_child_location_str,	chipc_child_location_str),

	DEVMETHOD(bus_add_child,		chipc_add_child),
	DEVMETHOD(bus_child_deleted,		chipc_child_deleted),

	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_alloc_resource,		bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_rl_release_resource),
	DEVMETHOD(bus_adjust_resource,		bus_generic_adjust_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_rl_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_get_resource_list,	chipc_get_resource_list),

	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),
	DEVMETHOD(bus_config_intr,		bus_generic_config_intr),
	DEVMETHOD(bus_bind_intr,		bus_generic_bind_intr),
	DEVMETHOD(bus_describe_intr,		bus_generic_describe_intr),

	/* ChipCommon interface */
	DEVMETHOD(bhnd_chipc_nvram_src,		chipc_nvram_src),
	DEVMETHOD(bhnd_chipc_write_chipctrl,	chipc_write_chipctrl),
	DEVMETHOD(bhnd_chipc_enable_sprom,	chipc_enable_sprom_pins),
	DEVMETHOD(bhnd_chipc_disable_sprom,	chipc_disable_sprom_pins),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_chipc, chipc_driver, chipc_methods, sizeof(struct chipc_softc));
DRIVER_MODULE(bhnd_chipc, bhnd, chipc_driver, bhnd_chipc_devclass, 0, 0);
MODULE_DEPEND(bhnd_chipc, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_chipc, 1);
