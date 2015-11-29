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
 * PCI-SIBA Bridge Device Support
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhndb/bhndb_pcivar.h>
#include <dev/bhnd/bhndb/bhndb_pcireg.h>

#include "bhndb_bus_if.h"
#include "bhndb_if.h"

#include "sibab_pcireg.h"
#include "sibab_pcivar.h"

#include "siba_configreg.h"

static int
sibab_pci_probe(device_t dev)
{	
	int error;

	/* Call default probe implementation */
	if ((error = bhndb_pci_generic_probe(dev)) > 0)
		return (error);
	
	device_set_desc(dev, "PCI-SIBA Bridge");
	return (error);
}

#define	CHIPC_CC		0x0

#define	CHIPC_CC_CHIPID_MASK	0x0000FFFF
#define	CHIPC_CC_CHIPID_SHIFT	0

#define	CHIPC_CC_REV_MASK	0x000F0000
#define	CHIPC_CC_REV_SHIFT	16

#define	CHIPC_CC_PKG_MASK	0x00F00000
#define	CHIPC_CC_PKG_SHIFT	20

#define	CHIPC_CC_NUMCORE_MASK	0x0F000000
#define	CHIPC_CC_NUMCORE_SHIFT	24

#define CHIPC_CC_BUS_T_MASK	0xF0000000
#define CHIPC_CC_BUS_T_SHIFT	28

#define	CHIPC_GET_ATTR(_entry, _attr) \
	((_entry & CHIPC_ ## _attr ## _MASK) >> CHIPC_ ## _attr ## _SHIFT)

static int
test_enumerate_cores(struct sibab_pci_softc *sc)
{
	const struct bhndb_hwcfg	*cfg;
	const struct bhndb_regwin	*rw;
	struct resource			*enum_res;
	int				 rid, type;
	int				 error;

	error = 0;
	cfg = BHNDB_BUS_GET_GENERIC_HWCFG(sc->parent_dev, sc->dev);

	/* Fetch and configurea register window */
	rw = bhndb_regwin_find_type(cfg->register_windows, BHNDB_REGWIN_T_DYN,
	    SIBA_CORE_SIZE);
	if (rw == NULL) {
		device_printf(sc->dev, "no usable dynamic register window\n");
		return (ENODEV);
	}

	/* Allocate regwin resource.*/
	rid = rw->res.rid;
	type = rw->res.type;
	enum_res = bus_alloc_resource_any(sc->parent_dev, type, &rid, RF_ACTIVE);
	if (enum_res == NULL) {
		device_printf(sc->dev, "failed to allocate sibab_pci "
		    "enumeration resource\n");
		return (ENXIO);
	}

	uint8_t ncore = 1;
	for (uint8_t i = 0; i < ncore; i++) {
		error = BHNDB_SET_WINDOW_ADDR(sc->dev, rw, SIBA_CORE_ADDR(i));
		if (error)
			goto failed;
		
		if (i == 0) {
			uint32_t chipc	= bus_read_4(enum_res, CHIPC_CC);
			uint16_t chip	= CHIPC_GET_ATTR(chipc, CC_CHIPID);
			uint8_t rev	= CHIPC_GET_ATTR(chipc, CC_REV);
			uint8_t pkg	= CHIPC_GET_ATTR(chipc, CC_PKG);
			ncore		= CHIPC_GET_ATTR(chipc, CC_NUMCORE);
			uint8_t bus	= CHIPC_GET_ATTR(chipc, CC_BUS_T);
			device_printf(sc->dev, "chip=0x%hx rev=0x%hhx pkg=0x%hhx ncore=0x%hhu bus=0x%hhx\n", chip, rev, pkg, ncore, bus);
		}
		
		uint32_t idL = bus_read_4(enum_res, SBIDLOW);
		uint32_t idH = bus_read_4(enum_res, SBIDHIGH);

		uint8_t cspace = SB_REG_GET(idL, IDL_CS);
		uint8_t arange = SB_REG_GET(idL, IDL_ADDR_RANGE);
		uint8_t minlat = SB_REG_GET(idL, IDL_MINLAT);
		uint8_t manlat = SB_REG_GET(idL, IDL_MAXLAT);
		uint8_t first = SB_REG_GET(idL, IDL_FIRST);
		uint8_t ccw = SB_REG_GET(idL, IDL_CW);
		uint8_t tpc = SB_REG_GET(idL, IDL_TP);
		uint8_t ipc = SB_REG_GET(idL, IDL_IP);
		uint8_t sbrev = SB_REG_GET(idL, IDL_SBREV);

		uint16_t vendor	= SB_REG_GET(idH, IDH_VENDOR);
		uint16_t device	= SB_REG_GET(idH, IDH_DEVICE);
		uint16_t hwrev	= SB_CORE_REV(idH);


		device_printf(sc->dev, "vendor=0x%hx device=0x%hx hwrev=0x%hu\n", vendor, device, hwrev);
		
		device_printf(sc->dev, "cspace=0x%hhx arange=0x%hhx minlat=0x%hhx manlat=0x%hhx first=0x%hhx ccw=0x%hhu tpc=0x%hhu ipc=0x%hhu sbrev=0x%hhx\n",cspace, arange, minlat, manlat, first, ccw, tpc, ipc, sbrev);

	}

failed:
	if (enum_res != NULL)
		bus_release_resource(sc->parent_dev, type, rid, enum_res);

	return (error);
}


static int
sibab_pci_attach(device_t dev)
{
	struct sibab_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	sc->dev = dev;
	sc->parent_dev = device_get_parent(dev);

	/* Enable clocks. */
	if ((error = BHNDB_ENABLE_CLOCKS(dev)))
		return (error);

	// TODO
	test_enumerate_cores(sc);

	/* Attach bridged siba(4) bus */
	sc->bus_dev = BUS_ADD_CHILD(dev, 0, devclass_get_name(siba_devclass), 0);
	if (sc->bus_dev == NULL) {
		error = ENXIO;
		goto failed;
	}

	/* Call default attach implementation */
	return bhndb_generic_attach(dev);

failed:
	return (error);
}

static int
sibab_pci_detach(device_t dev)
{
	struct sibab_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	/* Call default implementation before discarding local driver state. */
	if ((error = bhndb_generic_detach(dev)))
		return (error);

	return (0);
}

static bhnd_addr_t
sibab_pci_get_enum_addr(device_t dev, device_t child)
{
	return (SIBA_ENUM_ADDR);
}

static int
sibab_pci_get_window_addr(device_t dev, const struct bhndb_regwin *rw,
    bhnd_addr_t *addr)
{
	struct sibab_pci_softc *sc = device_get_softc(dev);

	switch (rw->win_type) {
	case BHNDB_REGWIN_T_CORE:
		// TODO
		return (ENXIO);
	case BHNDB_REGWIN_T_DYN:
		*addr = pci_read_config(sc->parent_dev, rw->dyn.cfg_offset, 4);
		return (0);
	default:
		return (ENODEV);
	}
}

static int
sibab_pci_set_window_addr(device_t dev, const struct bhndb_regwin *rw,
    bhnd_addr_t addr)
{
	struct sibab_pci_softc	*sc;
	int			 reg;

	sc = device_get_softc(dev);

	/* Mutation requires a dynamic window. */
	if (rw->win_type != BHNDB_REGWIN_T_DYN)
		return (ENODEV);

	/* The PCI bridge core only supports 32-bit addressing. */
	if (addr > UINT32_MAX)
		return (ERANGE);

	/* The write may fail and must be retried */
	reg = rw->dyn.cfg_offset;
	for (u_int i = 0; i < SIBAB_PCI_BARCTRL_WRITE_RETRY; i++) {
		pci_write_config(sc->parent_dev, reg, addr, 4);

		if (pci_read_config(sc->parent_dev, reg, 4) == addr)
			return (0);

		DELAY(10);
	}

	/* Unable to set window */
	return (ENODEV);
}

static device_t
sibab_pci_get_attached_bus(device_t dev)
{
	struct sibab_pci_softc *sc = device_get_softc(dev);
	return (sc->bus_dev);
}

static int
sibab_pci_get_core_table(device_t dev, struct bhnd_core_info **cores,
	u_int *count)
{
	// TODO
	return (ENXIO);
}



static device_method_t sibab_pci_methods[] = {
	/* Device interface */ \
	DEVMETHOD(device_probe,			sibab_pci_probe),
	DEVMETHOD(device_attach,		sibab_pci_attach),
	DEVMETHOD(device_detach,		sibab_pci_detach),

	/* BHNDB interface */
	DEVMETHOD(bhndb_get_core_table,		sibab_pci_get_core_table),
	DEVMETHOD(bhndb_get_enum_addr,		sibab_pci_get_enum_addr),
	DEVMETHOD(bhndb_get_attached_bus,	sibab_pci_get_attached_bus),
	DEVMETHOD(bhndb_get_window_addr,	sibab_pci_get_window_addr),
	DEVMETHOD(bhndb_set_window_addr,	sibab_pci_set_window_addr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(sibab, sibab_pci_driver, sibab_pci_methods,
    sizeof(struct sibab_pci_softc), bhndb_pci_driver);

MODULE_VERSION(sibab_pci, 1);
MODULE_DEPEND(sibab_pci, pci, 1, 1, 1);
MODULE_DEPEND(sibab_pci, bhndb, 1, 1, 1);
