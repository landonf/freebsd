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
 * Broadcom PCI-BHND Host Bridge.
 * 
 * This driver is used to "eat" PCI(e) cores operating in endpoint mode when
 * they're attached to a bhndb_pci driver on the host side.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_pcireg.h"
#include "bhnd_pci_hostbvar.h"

/*
 * Supported PCI bridge cores
 */
static const struct bhnd_hostb_device bhnd_hostb_devs[] = {
	/* PCI */
	BHND_HOSTB_DEV(PCI,	"PCI",
	    BHND_QUIRK_HWREV_RANGE	(0, 5,	BHND_PCI_QUIRK_SBINTVEC),
	    BHND_QUIRK_HWREV_GTE	(0,	BHND_PCI_QUIRK_SBTOPCI2_PREF_BURST),
	    BHND_QUIRK_HWREV_GTE	(11,	BHND_PCI_QUIRK_SBTOPCI2_READMULTI),
	    BHND_QUIRK_HWREV_END
	),

	/* PCI Gen 1 */
	BHND_HOSTB_DEV(PCIE,	"PCIe",
	    BHND_QUIRK_HWREV_EQ		(0,	BHND_PCIE_QUIRK_PCIPM_REQEN | BHND_PCIE_QUIRK_SERDES_L0s_HANG),
	    BHND_QUIRK_HWREV_RANGE	(0, 1,	BHND_PCIE_QUIRK_IGNORE_VDM),
	    BHND_QUIRK_HWREV_RANGE	(3, 5,	BHND_PCIE_QUIRK_ASPM_OVR | BHND_PCIE_QUIRK_SERDES_POLARITY),
	    BHND_QUIRK_HWREV_LTE	(6,	BHND_PCIE_QUIRK_L1_IDLE_THRESH),
	    BHND_QUIRK_HWREV_GTE	(6,	BHND_PCIE_QUIRK_SPROM_L23_PCI_RESET),
	    BHND_QUIRK_HWREV_EQ		(7,	BHND_PCIE_QUIRK_SERDES_NOPLLDOWN),
	    BHND_QUIRK_HWREV_GTE	(8,	BHND_PCIE_QUIRK_L1_TIMER_PERF),

	    BHND_QUIRK_HWREV_END
	),

	{ BHND_COREID_INVALID, NULL, BHND_PCI_REGS_PCI, NULL }
};

/* Standard core resource specification */
static const struct resource_spec bhnd_pci_hostb_rspec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};
#define	CORE_RES_IDX	0

/**
 * Collect all quirks defined in @p id that match @p dev.
 */
static uint32_t 
pcihb_get_quirks(device_t dev, const struct bhnd_hostb_device *id)
{
	struct bhnd_device_quirk	*dq;
	uint32_t			 quirks;
	uint16_t			 hwrev;

	hwrev = bhnd_get_hwrev(dev);
	quirks = BHND_PCI_QUIRK_NONE;

	for (dq = id->quirks; dq->quirks != 0; dq++) {
		if (bhnd_hwrev_matches(hwrev, &dq->hwrev))
			quirks |= dq->quirks;
	};

	return (quirks);
}

/**
 * Find the device table entry for @p dev, if any.
 */
static const struct bhnd_hostb_device *
pcihb_find_dev_entry(device_t dev)
{
	const struct bhnd_hostb_device *id;

	for (id = bhnd_hostb_devs; id->device != BHND_COREID_INVALID; id++) {
		if (bhnd_get_vendor(dev) == BHND_MFGID_BCM &&
		    bhnd_get_device(dev) == id->device)
			return (id);
	}

	return (NULL);
}

static int
bhnd_pci_hostb_probe(device_t dev)
{
	const struct bhnd_hostb_device *id;

	/* Ignore PCI cores not in host bridge mode. */
	if (!bhnd_is_hostb_device(dev))
		return (ENXIO);

	if ((id = pcihb_find_dev_entry(dev)) == NULL)
		return (ENXIO);

	device_set_desc(dev, id->desc);
	return (BUS_PROBE_DEFAULT);
}


/*
* Ensure that the correct PCI core's SPROM shadow is mapped to the first
* PCI core.
*/
static void
bhndb_pci_sprom_target_war(struct bhnd_pci_hostb_softc *sc)
{
	bus_size_t	sprom_addr;
	u_int		sprom_core_idx;
	u_int		pci_core_idx;
	uint16_t	val;

	/* Fetch the SPROM's configured core index */
	sprom_addr = BPCI_COMMON_REG_OFFSET(SPROM_SHADOW, SRSH_PI_OFFSET);
	val = bhnd_bus_read_2(sc->dev, sc->core, sprom_addr);

	/* If it doesn't match our core index, update the index value */
	sprom_core_idx = BPCI_COMMON_REG_GET(val, SRSH_PI);
	pci_core_idx = bhnd_get_core_index(sc->dev);

	if (sprom_core_idx != pci_core_idx) {
		val = BPCI_COMMON_REG_SET(val, SRSH_PI, pci_core_idx);
		bhnd_bus_write_2(sc->dev, sc->core, sprom_addr, val);
	}
}

static int
bhnd_pci_hostb_attach(device_t dev)
{
	const struct bhnd_hostb_device	*id;
	struct bhnd_pci_hostb_softc	*sc;
	int				 error;

	id = pcihb_find_dev_entry(dev);
	KASSERT(id != NULL, ("device entry went missing"));

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->regs = id->regs;
	sc->quirks = pcihb_get_quirks(dev, id);

	device_printf(dev, "QUIRKS: 0x%x\n", sc->quirks);

	/*
	 * Map our PCI core registers
	 */
	memcpy(sc->rspec, bhnd_pci_hostb_rspec, sizeof(sc->rspec));
	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	sc->core = sc->res[CORE_RES_IDX];

	/* Apply SPROM shadow work-around */
	bhndb_pci_sprom_target_war(sc);

	return (0);
}

static int
bhnd_pci_hostb_detach(device_t dev)
{
	struct bhnd_pci_hostb_softc	*sc;

	sc = device_get_softc(dev);
	bhnd_release_resources(dev, sc->rspec, sc->res);

	return (0);
}

static int
bhnd_pci_hostb_suspend(device_t dev)
{
	return (0);
}

static int
bhnd_pci_hostb_resume(device_t dev)
{
	return (0);
}

static device_method_t bhnd_pci_hostb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_pci_hostb_probe),
	DEVMETHOD(device_attach,	bhnd_pci_hostb_attach),
	DEVMETHOD(device_detach,	bhnd_pci_hostb_detach),
	DEVMETHOD(device_suspend,	bhnd_pci_hostb_suspend),
	DEVMETHOD(device_resume,	bhnd_pci_hostb_resume),
	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_hostb, bhnd_pci_hostb_driver, bhnd_pci_hostb_methods, sizeof(struct bhnd_pci_hostb_softc));

DRIVER_MODULE(bhnd_pci_hostb, bhnd, bhnd_pci_hostb_driver, bhnd_hostb_devclass, 0, 0);
