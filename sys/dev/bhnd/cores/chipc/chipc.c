/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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
 *
 * The purpose of this driver is resource management for ChipCommon drivers
 * like UART, PMU, flash. ChipCommon core has several resources:
 *  - several memory regions,
 *  - one or more IRQ lines.
 *
 * ChipCommon driver has 2 resource managers: for memory and for IRQ. Driver
 * gets information about BHND core ports/regions and map them
 * into drivers' resources.
 *
 * Here is overview of mapping:
 *
 * ------------------------------------------------------
 * | Port.Region| Purpose				|
 * ------------------------------------------------------
 * |	0.0	| main registers: SPI(0x40), UART(0x300)|
 * |	1.0	| ?					|
 * |	1.1	| MMIO flash (SPI & CFI)		|
 * ------------------------------------------------------
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndvar.h>

#include "bhnd_nvram_if.h"

#include "chipc.h"
#include "chipcreg.h"
#include "chipcvar.h"

devclass_t bhnd_chipc_devclass;	/**< bhnd(4) chipcommon device class */

static const struct resource_spec chipc_rspec[CHIPC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

static struct bhnd_device_quirk chipc_quirks[];

/* Supported device identifiers */
static const struct bhnd_device chipc_devices[] = {
	BHND_DEVICE(CC, "CC", chipc_quirks),
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

/* quirk and capability flag convenience macros */
#define	CHIPC_QUIRK(_sc, _name)	\
    ((_sc)->quirks & CHIPC_QUIRK_ ## _name)
    
#define	CHIPC_ASSERT_QUIRK(_sc, name)	\
    KASSERT(CHIPC_QUIRK((_sc), name), ("quirk " __STRING(_name) " not set"))

/*
 * Here is resource configuration for child devices
 *
 * [Flash] There are 2 flash resources:
 *  - resource ID (rid) = 0: memory-mapped flash memory
 *  - resource ID (rid) = 1: memory-mapped flash registers (i.e for SPI)
 *
 * [UART] Uses IRQ and memory resources:
 *  - resource ID (rid) = 0: memory-mapped registers
 *  - IRQ resource ID (rid) = 0: shared IRQ line for Tx/Rx.
 */
struct chipc_spec chipc_children_mem_specs[] = {
	{"uart", SYS_RES_MEMORY, 0, 0, 0, CHIPC_UART_BASE, CHIPC_UART_SIZE},
	{"uart", SYS_RES_IRQ, 	 0,-1,-1, 0, 1 },
	{"spi", SYS_RES_MEMORY,  0, 1, 1, 0, ~0},
	{"spi", SYS_RES_MEMORY,  1, 0, 0, CHIPC_FLASHBASE, CHIPC_FLASHREGSZ},
	{"cfi", SYS_RES_MEMORY,  0, 1, 1, 0, ~0},
	{"cfi", SYS_RES_MEMORY,  1, 0, 0, CHIPC_FLASHBASE, CHIPC_FLASHREGSZ},
	{NULL, 0, 0, 0, 0, 0, 0}
};

static struct resource_list*	chipc_get_resource_list(device_t dev,
							device_t child);
static struct resource*		chipc_alloc_resource(device_t bus,
							device_t child,
							int type,
							int *rid,
							rman_res_t start,
							rman_res_t end,
							rman_res_t count,
							u_int flags);

static void		chipc_probe_nomatch(device_t dev, device_t child);
static void		chipc_set_resources(device_t bus, device_t dev,
		    	    struct resource_list* rl);
static int		chipc_release_resource(device_t bus, device_t child,
		    	    int type, int rid, struct resource *r);
void			chipc_cleanup(device_t dev);

static bhnd_nvram_src_t	chipc_nvram_identify(struct chipc_softc *sc);
static int		chipc_sprom_init(struct chipc_softc *);
static int		chipc_enable_sprom_pins(struct chipc_softc *);
static int		chipc_disable_sprom_pins(struct chipc_softc *);
static int		chipc_init_rman(device_t dev);

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
	uint32_t		 	 caps;
	uint8_t				 chip_type;
	int				 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = bhnd_device_quirks(dev, chipc_devices,
	    sizeof(chipc_devices[0]));
	sc->quirks |= bhnd_chip_quirks(dev, chipc_chip_quirks);
	
	CHIPC_LOCK_INIT(sc);

	/* Allocate bus resources */
	memcpy(sc->rspec, chipc_rspec, sizeof(sc->rspec));
	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	sc->core = sc->res[0];
	
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
	caps = bhnd_bus_read_4(sc->core, CHIPC_CAPABILITIES);
	chipc_parse_capabilities(&sc->capabilities, caps);
	sc->cst = bhnd_bus_read_4(sc->core, CHIPC_CHIPST);

	/* Identify NVRAM source */
	sc->nvram_src = chipc_nvram_identify(sc);

	/* Read NVRAM data */
	switch (sc->nvram_src) {
	case BHND_NVRAM_SRC_OTP:
		// TODO (requires access to OTP hardware)
		device_printf(sc->dev, "NVRAM-OTP unsupported\n");
		break;

	case BHND_NVRAM_SRC_NFLASH:
		// TODO (requires access to NFLASH hardware)
		device_printf(sc->dev, "NVRAM-NFLASH unsupported\n");
		break;

	case BHND_NVRAM_SRC_SPROM:
		if ((error = chipc_sprom_init(sc)))
			goto cleanup;
		break;

	case BHND_NVRAM_SRC_UNKNOWN:
		/* Handled externally */
		break;
	}

	error = chipc_init_rman(dev);

	if (error != 0) {
		device_printf(dev,"init_rman failed with: %d\n", error);
		goto cleanup;
	}

	error = bus_generic_attach(dev);
	if (error != 0) {
		device_printf(dev, "bus_generic_attach failed: %d\n", error);
		goto cleanup;
	}

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

	sc = device_get_softc(dev);
	bhnd_release_resources(dev, sc->rspec, sc->res);
	bhnd_sprom_fini(&sc->sprom);
	chipc_cleanup(dev);

	CHIPC_LOCK_DESTROY(sc);

	return (0);
}

static int
chipc_suspend(device_t dev)
{
	return (0);
}

static int
chipc_resume(device_t dev)
{
	return (0);
}


static void
chipc_probe_nomatch(device_t dev, device_t child)
{
	device_printf(dev, "no found driver for %s\n", device_get_name(child));
}

/**
 * Initialize local SPROM shadow, if required.
 * 
 * @param sc chipc driver state.
 */
static int
chipc_sprom_init(struct chipc_softc *sc)
{
	int	error;

	KASSERT(sc->nvram_src == BHND_NVRAM_SRC_SPROM,
	    ("non-SPROM source (%u)\n", sc->nvram_src));

	/* Enable access to the SPROM */
	CHIPC_LOCK(sc);
	if ((error = chipc_enable_sprom_pins(sc)))
		goto failed;

	/* Initialize SPROM parser */
	error = bhnd_sprom_init(&sc->sprom, sc->core, CHIPC_SPROM_OTP);
	if (error) {
		device_printf(sc->dev, "SPROM identification failed: %d\n",
			error);

		chipc_disable_sprom_pins(sc);
		goto failed;
	}

	/* Drop access to the SPROM lines */
	if ((error = chipc_disable_sprom_pins(sc))) {
		bhnd_sprom_fini(&sc->sprom);
		goto failed;
	}
	CHIPC_UNLOCK(sc);

	return (0);

failed:
	CHIPC_UNLOCK(sc);
	return (error);
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
	if (sc->capabilities.sprom) {
		srom_ctrl = bhnd_bus_read_4(sc->core, CHIPC_SPROM_CTRL);
		if (srom_ctrl & CHIPC_SRC_PRESENT)
			return (BHND_NVRAM_SRC_SPROM);
	}

	/* Check for OTP */
	if (sc->capabilities.otp_size)
		return (BHND_NVRAM_SRC_OTP);

	/*
	 * Finally, Northstar chipsets (and possibly other chipsets?) support
	 * external NAND flash. 
	 */
	if (CHIPC_QUIRK(sc, SUPPORTS_NFLASH) && sc->capabilities.nflash)
		return (BHND_NVRAM_SRC_NFLASH);

	/* No NVRAM hardware capability declared */
	return (BHND_NVRAM_SRC_UNKNOWN);
}


/**
 * If required by this device, enable access to the SPROM.
 * 
 * @param sc chipc driver state.
 */
static int
chipc_enable_sprom_pins(struct chipc_softc *sc)
{
	uint32_t cctrl;
	
	CHIPC_LOCK_ASSERT(sc, MA_OWNED);

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return (0);

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);

	/* 4331 devices */
	if (CHIPC_QUIRK(sc, 4331_EXTPA_MUX_SPROM)) {
		cctrl &= ~CHIPC_CCTRL4331_EXTPA_EN;

		if (CHIPC_QUIRK(sc, 4331_GPIO2_5_MUX_SPROM))
			cctrl &= ~CHIPC_CCTRL4331_EXTPA_ON_GPIO2_5;

		if (CHIPC_QUIRK(sc, 4331_EXTPA2_MUX_SPROM))
			cctrl &= ~CHIPC_CCTRL4331_EXTPA_EN2;

		bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);
		return (0);
	}

	/* 4360 devices */
	if (CHIPC_QUIRK(sc, 4360_FEM_MUX_SPROM)) {
		/* Unimplemented */
	}

	/* Refuse to proceed on unsupported devices with muxed SPROM pins */
	device_printf(sc->dev, "muxed sprom lines on unrecognized device\n");
	return (ENXIO);
}

/**
 * If required by this device, revert any GPIO/pin configuration applied
 * to allow SPROM access.
 * 
 * @param sc chipc driver state.
 */
static int
chipc_disable_sprom_pins(struct chipc_softc *sc)
{
	uint32_t cctrl;

	CHIPC_LOCK_ASSERT(sc, MA_OWNED);

	/* Nothing to do? */
	if (!CHIPC_QUIRK(sc, MUX_SPROM))
		return (0);

	cctrl = bhnd_bus_read_4(sc->core, CHIPC_CHIPCTRL);

	/* 4331 devices */
	if (CHIPC_QUIRK(sc, 4331_EXTPA_MUX_SPROM)) {
		cctrl |= CHIPC_CCTRL4331_EXTPA_EN;

		if (CHIPC_QUIRK(sc, 4331_GPIO2_5_MUX_SPROM))
			cctrl |= CHIPC_CCTRL4331_EXTPA_ON_GPIO2_5;

		if (CHIPC_QUIRK(sc, 4331_EXTPA2_MUX_SPROM))
			cctrl |= CHIPC_CCTRL4331_EXTPA_EN2;

		bhnd_bus_write_4(sc->core, CHIPC_CHIPCTRL, cctrl);
		return (0);
	}

	/* 4360 devices */
	if (CHIPC_QUIRK(sc, 4360_FEM_MUX_SPROM)) {
		/* Unimplemented */
	}
	
	/* Refuse to proceed on unsupported devices with muxed SPROM pins */
	device_printf(sc->dev, "muxed sprom lines on unrecognized device\n");
	return (ENXIO);
}

static bhnd_nvram_src_t
chipc_nvram_src(device_t dev)
{
	struct chipc_softc *sc = device_get_softc(dev);
	return (sc->nvram_src);
}

static int
chipc_nvram_getvar(device_t dev, const char *name, void *buf, size_t *len)
{
	struct chipc_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	switch (sc->nvram_src) {
	case BHND_NVRAM_SRC_SPROM:
		CHIPC_LOCK(sc);
		error = bhnd_sprom_getvar(&sc->sprom, name, buf, len);
		CHIPC_UNLOCK(sc);
		return (error);

	case BHND_NVRAM_SRC_OTP:
	case BHND_NVRAM_SRC_NFLASH:
		/* Currently unsupported */
		return (ENXIO);

	case BHND_NVRAM_SRC_UNKNOWN:
		return (ENODEV);
	}

	/* Unknown NVRAM source */
	return (ENODEV);
}

static int
chipc_nvram_setvar(device_t dev, const char *name, const void *buf,
    size_t len)
{
	struct chipc_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	switch (sc->nvram_src) {
	case BHND_NVRAM_SRC_SPROM:
		CHIPC_LOCK(sc);
		error = bhnd_sprom_setvar(&sc->sprom, name, buf, len);
		CHIPC_UNLOCK(sc);
		return (error);

	case BHND_NVRAM_SRC_OTP:
	case BHND_NVRAM_SRC_NFLASH:
		/* Currently unsupported */
		return (ENXIO);

	case BHND_NVRAM_SRC_UNKNOWN:
	default:
		return (ENODEV);
	}

	/* Unknown NVRAM source */
	return (ENODEV);
}

static struct chipc_capabilities*
chipc_get_caps(device_t dev)
{
	struct chipc_softc	*sc;

	sc = device_get_softc(dev);
	return (&sc->capabilities);
}

static uint32_t
chipc_get_flash_cfg(device_t dev)
{
	struct chipc_softc	*sc;

	sc = device_get_softc(dev);
	return (bhnd_bus_read_4(sc->core, CHIPC_FLASH_CFG));
}

void
chipc_cleanup(device_t dev)
{
	struct chipc_softc *sc;
	sc = device_get_softc(dev);
	rman_fini(&sc->chipc_irq);
	rman_fini(&sc->chipc_mem);
}

static int
chipc_init_rman(device_t dev)
{
	int			 err;
	struct chipc_softc	*sc;
	bhnd_addr_t		 rg_start;
	bhnd_size_t		 rg_size;
	int			 rgcnt;

	BHND_DEBUG_DEV(dev, "init rman");

	sc = device_get_softc(dev);
	sc->chipc_irq.rm_start = 0;
	sc->chipc_irq.rm_end = NUM_IRQS - 1;
	sc->chipc_irq.rm_type = RMAN_ARRAY;
	sc->chipc_irq.rm_descr = "ChipCommon IRQs";
	err = rman_init(&sc->chipc_irq);
	if (err) {
		BHND_ERROR_DEV(dev, "error occurred during rman_init of "
				"IRQ rman: %d", err);
		goto error;
	}
	sc->chipc_mem.rm_start = 0;
	sc->chipc_mem.rm_end = BUS_SPACE_MAXADDR;
	sc->chipc_mem.rm_type = RMAN_ARRAY;
	sc->chipc_mem.rm_descr = "ChipCommon Memory";
	err = rman_init(&sc->chipc_mem);
	if (err) {
		BHND_ERROR_DEV(dev, "error occurred during init of "
				"MEMORY rman: %d", err);
		goto error;
	}

	/* Iterate over device ports & regions and fill instance variable */
	for(int i = 0; i < bhnd_get_port_count(dev, BHND_PORT_DEVICE); i++) {
		rgcnt = bhnd_get_region_count(dev, BHND_PORT_DEVICE, i);
		BHND_DEBUG_DEV(dev, "[%d] region count = %d", i, rgcnt);
		for (int j = 0; j < rgcnt; j++) {
			err = bhnd_get_region_addr(dev, BHND_PORT_DEVICE, i, j,
					&rg_start, &rg_size);
			BHND_DEBUG_DEV(dev, "[%d.%d] region addr = 0x%jx (%d)",
					i, j, rg_start, err);
			if (err != 0)
				continue;

			err = rman_manage_region(&sc->chipc_mem, rg_start,
					rg_start + rg_size - 1);
			if (err != 0) {
				BHND_ERROR_DEV(dev, "error occurred during "
					"rman_manage_region of MEMORY rman with "
					"params: 0x%jx - 0x%jx : err = %d"
					,rg_start, rg_start + rg_size - 1, err);
				goto error;
			}
		}
	}

/*
 * We're creating resource manager for all MIPS IRQs, but it's MIPS-ifed code
 * and it's better to get IRQ number from core info.
 * TODO: fetch IRQ number from core info to demipsify code
 */
	err = rman_manage_region(&sc->chipc_irq, 0, NUM_IRQS - 1);
	if (err) {
		BHND_ERROR_DEV(dev, "error occurred (code = %d) during "
		    "rman_manage_region of IRQ rman with params: 0x%d - 0x%d",
		    err, 0, NUM_IRQS);
		goto error;
	}

	BHND_DEBUG_DEV(dev, "rmans are initialized successfully");
	return (0);
error:
	BHND_ERROR_DEV(dev, "init_rman finished with error: %d", err);
	chipc_cleanup(dev);
	return (err);
}

static struct resource_list *
chipc_get_resource_list(device_t dev, device_t child)
{
	struct chipc_devinfo *dinfo;

	dinfo = device_get_ivars(child);

	if (dinfo == NULL) {
		/*
		 * Lazy way of resource assignment
		 */
		dinfo = malloc(sizeof(struct chipc_devinfo*), M_BHND, M_NOWAIT);

		if (dinfo == NULL) {
			BHND_ERROR_DEV(dev, "can't allocate memory for "
					"chipc_devinfo of %s",
					device_get_nameunit(child));
			return (NULL);
		}

		resource_list_init(&(dinfo->resources));
		chipc_set_resources(dev, child, &(dinfo->resources));
		device_set_ivars(child, dinfo);
	}

	return (&dinfo->resources);
}

static void
chipc_set_resources(device_t bus, device_t dev, struct resource_list* rl)
{
	const char		*devname;
	struct chipc_spec	*spec;
	rman_res_t		 start;
	rman_res_t		 end;
	bhnd_addr_t		 region_start;
	bhnd_addr_t		 region_end;
	bhnd_size_t		 region_size;
	rman_res_t		 count;
	int			 err;

	BHND_DEBUG_DEV(dev, "init resource list...");

	devname = device_get_name(dev);
	spec = chipc_children_mem_specs;
	for (;spec->name != NULL; spec++) {
		if (strcmp(spec->name, devname) != 0)
			continue;

		/* device name is matched */
		switch (spec->type){
		case SYS_RES_MEMORY:
			err = bhnd_get_region_addr(bus, BHND_PORT_DEVICE,
			    spec->port, spec->reg, &region_start, &region_size);
			if (err != 0) {
				BHND_WARN_DEV(bus, "region is not found "
				    "[type %d, rid %d, port.region %d.%d]",
				    spec->type, spec->rid, spec->port, spec->reg);
				continue;
			}

			region_end = region_start + region_size - 1;

			break;
		/*
		 * We're creating resource manager for all MIPS IRQs,
		 * but it's MIPS-ifed code and it's better to get IRQ number
		 * from core info.
		 *
		 * TODO: fetch IRQ number from core info to demipsify code
		 */
		case SYS_RES_IRQ:
			region_start = 2;
			region_end = 2;
			region_size = 1;
			break;
		default:
			continue;
		}

		start = MIN(region_start + spec->start, region_end);
		end = MIN(region_start + spec->start +
			  MIN(spec->size, region_size) - 1,
			  region_end);
		count = end - start + 1;
		resource_list_add(rl, spec->type, spec->rid, start, end, count);
	}
}

static struct resource *
chipc_alloc_resource(device_t bus, device_t child, int type, int *rid,
	rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct chipc_softc		*sc;
	struct rman			*rm;
	struct resource			*r;
	struct resource_list		*rl;
	struct resource_list_entry	*rle;

	BHND_DEBUG_DEV(child, "looking for allocation resource [%d]: %d, "
			"0x%jx, 0x%jx, 0x%jx", type, *rid, start, end, count);
	if (RMAN_IS_DEFAULT_RANGE(start, end) && count == 1) {
		rl = chipc_get_resource_list(bus, child);
		if (rl == NULL) {
			BHND_ERROR_DEV(bus, "there is no resource list for: %s",
					device_get_nameunit(child));
			return (NULL);
		}
		rle = resource_list_find(rl, type, *rid);
		if (rle == NULL) {
			BHND_DEBUG_DEV(child, "there is no resource in "
					"resource list: type %d rid %d",
					type, *rid);
			return (NULL);
		}
		start = rle->start;
		end = rle->end;
		count = rle->count;
	}

	sc = device_get_softc(bus);
	switch (type) {
	case SYS_RES_IRQ:
		rm = &sc->chipc_irq;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->chipc_mem;
		break;
	default:
		BHND_ERROR_DEV(child, "unknown resource type %d", type);
		return (NULL);
	}

	BHND_DEBUG_DEV(child, "ready for rman_reserve [%d]: %d, "
			"0x%jx, 0x%jx, 0x%jx", type, *rid, start, end, count);
	r = rman_reserve_resource(rm, start, end, count, flags, child);
	if (r == NULL) {
		BHND_ERROR_DEV(child, "could not reserve resource %d: "
				"0x%jx-0x%jx", *rid, start, end);
		return (NULL);
	}

	rman_set_rid(r, *rid);

	if (flags & RF_ACTIVE)
		if (bus_activate_resource(child, type, *rid, r)) {
			BHND_ERROR_DEV(child, "could not activate resource %d:"
					" 0x%jx-0x%jx", *rid, start, end);
			rman_release_resource(r);
			return (NULL);
		}

	return (r);
}

static int
chipc_release_resource(device_t dev, device_t child, int type, int rid,
		       struct resource *r)
{
	int error;

	if (rman_get_flags(r) & RF_ACTIVE) {
		error = bus_deactivate_resource(child, type, rid, r);
		if (error)
			return (error);
	}

	return (rman_release_resource(r));
}

static device_method_t chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		chipc_probe),
	DEVMETHOD(device_attach,	chipc_attach),
	DEVMETHOD(device_detach,	chipc_detach),
	DEVMETHOD(device_suspend,	chipc_suspend),
	DEVMETHOD(device_resume,	chipc_resume),

	/* Bus interface */
	DEVMETHOD(bus_add_child,		bus_generic_add_child),
	DEVMETHOD(bus_get_resource_list,	chipc_get_resource_list),
	DEVMETHOD(bus_alloc_resource,		chipc_alloc_resource),
	DEVMETHOD(bus_release_resource,		chipc_release_resource),
	DEVMETHOD(bus_activate_resource, 	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, 	bus_generic_deactivate_resource),

	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),
	DEVMETHOD(bus_config_intr,		bus_generic_config_intr),
	DEVMETHOD(bus_bind_intr,		bus_generic_bind_intr),
	DEVMETHOD(bus_describe_intr,		bus_generic_describe_intr),

	DEVMETHOD(bus_probe_nomatch,		chipc_probe_nomatch),

	/* ChipCommon interface */
	DEVMETHOD(bhnd_chipc_nvram_src,		chipc_nvram_src),
	DEVMETHOD(bhnd_chipc_get_capabilities, 	chipc_get_caps),
	DEVMETHOD(bhnd_chipc_get_flash_cfg,	chipc_get_flash_cfg),

	/* NVRAM interface */
	DEVMETHOD(bhnd_nvram_getvar,	chipc_nvram_getvar),
	DEVMETHOD(bhnd_nvram_setvar,	chipc_nvram_setvar),

	/*
	 * TODO: Add
	 * 	- bus_print_child,
	 * 	- bus_{get,set,delete}_resource,
	 * 	- bus_hinted_child
	 */

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_chipc, chipc_driver, chipc_methods, sizeof(struct chipc_softc));
EARLY_DRIVER_MODULE(bhnd_chipc, bhnd, chipc_driver, bhnd_chipc_devclass, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
MODULE_DEPEND(bhnd_chipc, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_chipc, 1);
