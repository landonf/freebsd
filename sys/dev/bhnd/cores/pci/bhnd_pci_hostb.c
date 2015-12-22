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

#include "mdio_if.h"

#include "bhnd_pcireg.h"
#include "bhnd_pci_hostbvar.h"

static uint32_t				 bcm_pci_get_quirks(device_t dev,
					     const struct bhnd_pci_device *id);

static const struct bhnd_pci_device	*bcm_pci_find_dev_entry(device_t dev);

static uint32_t				 bcm_pcie_read_proto_reg(
					     struct bhnd_pci_hostb_softc *sc,
					     uint32_t addr);
static void				 bcm_pcie_write_proto_reg(
					     struct bhnd_pci_hostb_softc *sc,
					     uint32_t addr, uint32_t val);

static void				 bcm_pci_hw_wars(
					     struct bhnd_pci_hostb_softc *sc);

/*
 * Supported PCI bridge cores
 */
static const struct bhnd_pci_device bhnd_pci_devs[] = {
	/* PCI */
	BHND_HOSTB_DEV(PCI,	"PCI",
	    BHND_QUIRK_HWREV_RANGE	(0, 5,	BHND_PCI_QUIRK_SBINTVEC),
	    BHND_QUIRK_HWREV_GTE	(0,	BHND_PCI_QUIRK_SBTOPCI2_PREF_BURST),
	    BHND_QUIRK_HWREV_GTE	(11,	BHND_PCI_QUIRK_SBTOPCI2_READMULTI | BHND_PCI_QUIRK_CLKRUN_DSBL),
	    BHND_QUIRK_HWREV_END
	),

	/* PCI Gen 1 */
	BHND_HOSTB_DEV(PCIE,	"PCIe",
	    BHND_QUIRK_HWREV_EQ		(0,	BHND_PCIE_QUIRK_SDR9_L0s_HANG),
	    BHND_QUIRK_HWREV_RANGE	(0, 1,	BHND_PCIE_QUIRK_UR_STATUS_FIX),
	    BHND_QUIRK_HWREV_EQ		(1,	BHND_PCIE_QUIRK_PCIPM_REQEN),
	    BHND_QUIRK_HWREV_RANGE	(3, 5,	BHND_PCIE_QUIRK_ASPM_OVR | BHND_PCIE_QUIRK_SDR9_POLARITY),
	    BHND_QUIRK_HWREV_LTE	(6,	BHND_PCIE_QUIRK_L1_IDLE_THRESH),
	    BHND_QUIRK_HWREV_GTE	(6,	BHND_PCIE_QUIRK_SPROM_L23_PCI_RESET),
	    BHND_QUIRK_HWREV_EQ		(7,	BHND_PCIE_QUIRK_SERDES_NOPLLDOWN),
	    BHND_QUIRK_HWREV_GTE	(8,	BHND_PCIE_QUIRK_L1_TIMER_PERF),
	    BHND_QUIRK_HWREV_GTE	(10,	BHND_PCIE_QUIRK_SD_C22_EXTADDR),

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

#define	BHND_PCI_QUIRK(_sc, _name)	\
    ((_sc)->quirks & BHND_PCI_QUIRK_ ## _name)

#define	BHND_PCIE_QUIRK(_sc, _name)	\
    ((_sc)->quirks & BHND_PCIE_QUIRK_ ## _name)

/**
 * Collect all quirks defined in @p id that match @p dev.
 */
static uint32_t 
bcm_pci_get_quirks(device_t dev, const struct bhnd_pci_device *id)
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
static const struct bhnd_pci_device *
bcm_pci_find_dev_entry(device_t dev)
{
	const struct bhnd_pci_device *id;

	for (id = bhnd_pci_devs; id->device != BHND_COREID_INVALID; id++) {
		if (bhnd_get_vendor(dev) == BHND_MFGID_BCM &&
		    bhnd_get_device(dev) == id->device)
			return (id);
	}

	return (NULL);
}

static int
bhnd_pci_hostb_probe(device_t dev)
{
	const struct bhnd_pci_device *id;

	/* Ignore PCI cores not in host bridge mode. */
	if (!bhnd_is_hostb_device(dev))
		return (ENXIO);

	if ((id = bcm_pci_find_dev_entry(dev)) == NULL)
		return (ENXIO);

	device_set_desc(dev, id->desc);
	return (BUS_PROBE_DEFAULT);
}

static int
bhnd_pci_hostb_attach(device_t dev)
{
	const struct bhnd_pci_device	*id;
	struct bhnd_pci_hostb_softc	*sc;
	int				 error;

	id = bcm_pci_find_dev_entry(dev);
	KASSERT(id != NULL, ("device entry went missing"));

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->regs = id->regs;
	sc->quirks = bcm_pci_get_quirks(dev, id);
	BHND_PCI_LOCK_INIT(sc);

	/*
	 * Map our PCI core registers
	 */
	memcpy(sc->rspec, bhnd_pci_hostb_rspec, sizeof(sc->rspec));
	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	sc->core = sc->res[CORE_RES_IDX];

	/* Attach PCIe MDIO interface */
	bus_generic_probe(dev);
	sc->mdio = device_find_child(dev, devclass_get_name(bhnd_mdio_pci), 0);

	if (bhnd_get_class(dev) == BHND_DEVCLASS_PCIE && sc->mdio == NULL) {
		device_printf(dev, "failed to attach MDIO device\n");
		error = ENXIO;
		goto failed;
	}

	if ((error = bus_generic_attach(dev)))
		goto failed;

	/* Determine correct polarity by observing the attach-time PCIe PHY
	 * link status. This is used later to reset/force the SerDes
	 * polarity */
	if (BHND_PCIE_QUIRK(sc, SDR9_POLARITY)) {
		uint32_t st;

		st = bcm_pcie_read_proto_reg(sc, BHND_PCIE_PLP_STATUSREG);
		sc->polarity_inv = ((st & BHND_PCIE_PLP_POLARITY_INV) != 0);
	}

	/* Apply hardware work-arounds */
	bcm_pci_hw_wars(sc);

	return (error);

failed:
	bhnd_release_resources(dev, sc->rspec, sc->res);
	return (error);
}

static int
bhnd_pci_hostb_detach(device_t dev)
{
	struct bhnd_pci_hostb_softc *sc = device_get_softc(dev);

	bhnd_release_resources(dev, sc->rspec, sc->res);
	BHND_PCI_LOCK_DESTROY(sc);

	return (bus_generic_detach(dev));
}

static int
bhnd_pci_hostb_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

static int
bhnd_pci_hostb_resume(device_t dev)
{
	struct bhnd_pci_hostb_softc	*sc;
	int				 error;
	
	sc = device_get_softc(dev);

	if ((error = bus_generic_resume(dev)))
		return (error);

	/* Apply hardware work-arounds */
	bcm_pci_hw_wars(sc);

	return (0);
}

/*
 * PCIe Protocol Register Access
 */

static uint32_t
bcm_pcie_read_proto_reg(struct bhnd_pci_hostb_softc *sc, uint32_t addr)
{
	uint32_t val;

	KASSERT(bhnd_get_class(sc->dev) == BHND_DEVCLASS_PCIE,
	    ("not a pcie device!"));

	BHND_PCI_LOCK(sc);	
	bhnd_bus_write_4(sc->dev, sc->core, BHND_PCIE_IND_ADDR, addr);
	val = bhnd_bus_read_4(sc->dev, sc->core, BHND_PCIE_IND_DATA);
	BHND_PCI_UNLOCK(sc);

	return (val);
}

static void
bcm_pcie_write_proto_reg(struct bhnd_pci_hostb_softc *sc, uint32_t addr,
    uint32_t val)
{
	KASSERT(bhnd_get_class(sc->dev) == BHND_DEVCLASS_PCIE,
	    ("not a pcie device!"));

	BHND_PCI_LOCK(sc);	
	bhnd_bus_write_4(sc->dev, sc->core, BHND_PCIE_IND_ADDR, addr);
	bhnd_bus_write_4(sc->dev, sc->core, BHND_PCIE_IND_DATA, val);
	BHND_PCI_UNLOCK(sc);
}

/*
 * Hardware Workarounds (WARs)
 */

/*
 * Quirk: N/A
 * Applies to all PCI/PCIe revisions.
 * 
 * On devices without a SROM, the PCI(e) cores will be initialized with
 * their Power-on-Reset defaults; this can leave the the BAR0 PCIe windows
 * potentially mapped to the wrong core.
 * 
 * This WAR updates the BAR0 defaults to point at the current PCIe core.
 */
static void
bcm_pci_sromless_defaults_war(struct bhnd_pci_hostb_softc *sc)
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
		BPCI_COMMON_REG_SET(val, SRSH_PI, pci_core_idx);
		bhnd_bus_write_2(sc->dev, sc->core, sprom_addr, val);
	}
}

/* Hardware Setup WARs */
static void
bcm_pci_hw_wars(struct bhnd_pci_hostb_softc *sc)
{
	/* Fix Power-on-Reset defaults on SROM-less PCI/PCIe devices. */
	// TODO - This must be lifted out into bridge-level initialization
	bcm_pci_sromless_defaults_war(sc);

	/* Enable PCI prefetch/burst/MRM support */
	if (BHND_PCI_QUIRK(sc, SBTOPCI2_PREF_BURST) ||
	    BHND_PCI_QUIRK(sc, SBTOPCI2_READMULTI))
	{
		uint32_t sbp2;
		sbp2 = bhnd_bus_read_4(sc->dev, sc->core, BHND_PCI_SBTOPCI2);

		if (BHND_PCI_QUIRK(sc, SBTOPCI2_PREF_BURST))
			sbp2 |= (BHND_PCI_SBTOPCI_PREF|BHND_PCI_SBTOPCI_BURST);
		
		if (BHND_PCI_QUIRK(sc, SBTOPCI2_READMULTI))
			sbp2 |= BHND_PCI_SBTOPCI_RC_READMULTI;

		bhnd_bus_write_4(sc->dev, sc->core, BHND_PCI_SBTOPCI2, sbp2);
	}
	
	/* Disable PCI CLKRUN# */
	if (BHND_PCI_QUIRK(sc, CLKRUN_DSBL)) {
		uint32_t ctl;
	
		ctl = bhnd_bus_read_4(sc->dev, sc->core, BHND_PCI_CLKRUN_CTL);
		ctl |= BHND_PCI_CLKRUN_DSBL;
		bhnd_bus_write_4(sc->dev, sc->core, BHND_PCI_CLKRUN_CTL, ctl);
	}

	/* Force correct SerDes polarity */
	if (BHND_PCIE_QUIRK(sc, SDR9_POLARITY)) {
		device_printf(sc->dev, "SDR9_POLARITY\n");
		uint16_t	rxctl;

		rxctl = MDIO_READREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CTRL);

		rxctl |= BHND_PCIE_SDR9_RX_CTRL_FORCE;
		if (sc->polarity_inv)
			rxctl |= BHND_PCIE_SDR9_RX_CTRL_POLARITY_INV;
		else
			rxctl &= ~BHND_PCIE_SDR9_RX_CTRL_POLARITY_INV;

		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CTRL, rxctl);
	}

	/* Enable TLP unmatched address handling work-around */
	if (BHND_PCIE_QUIRK(sc, UR_STATUS_FIX)) {
		device_printf(sc->dev, "UR_STATUS_FIX\n");
		uint32_t wrs;
		wrs = bcm_pcie_read_proto_reg(sc, BHND_PCIE_TLP_WORKAROUNDSREG);
		wrs |= BHND_PCIE_TLP_WORKAROUND_URBIT;
		bcm_pcie_write_proto_reg(sc, BHND_PCIE_TLP_WORKAROUNDSREG, wrs);
	}

	/* Explicitly enable PCI-PM */
	if (BHND_PCIE_QUIRK(sc, PCIPM_REQEN)) {
		device_printf(sc->dev, "PCIPM_REQEN\n");
		uint32_t lcreg;
		lcreg = bcm_pcie_read_proto_reg(sc, BHND_PCIE_DLLP_LCREG);
		lcreg |= BHND_PCIE_DLLP_LCREG_PCIPM_EN;
		bcm_pcie_write_proto_reg(sc, BHND_PCIE_DLLP_LCREG, lcreg);
	}

	/* Adjust SerDes CDR tuning to ensure that CDR is stable before sending
	 * data during L0s to L0 exit transitions. */
	if (BHND_PCIE_QUIRK(sc, SDR9_L0s_HANG)) {
		device_printf(sc->dev, "SDR9_L0s_HANG\n");
		uint16_t sdv;

		/* Set RX track/acquire timers to 2.064us/40.96us */
		sdv = 0;
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_TIMER1_LKTRK, (2064/16));
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_TIMER1_LKACQ, (40960/1024));
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		BHND_PCIE_SDR9_RX_TIMER1, sdv);

		/* Apply CDR frequency workaround */
		sdv = BHND_PCIE_SDR9_RX_CDR_FREQ_OVR_EN;
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDR_FREQ_OVR, 0x0);
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		BHND_PCIE_SDR9_RX_CDR, sdv);

		/* Apply CDR BW tunings */
		sdv = 0;
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_INTGTRK, 0x2);
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_INTGACQ, 0x4);
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_PROPTRK, 0x6);
		BPCI_REG_SET(sdv, PCIE_SDR9_RX_CDRBW_PROPACQ, 0x6);
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		BHND_PCIE_SDR9_RX_CDRBW, sdv);
	}

	/* Enable L23READY_EXIT_NOPRST if not already set in SPROM. */
	if (BHND_PCIE_QUIRK(sc, SPROM_L23_PCI_RESET)) {
		bus_size_t	reg;
		uint16_t	cfg;

		/* Fetch the misccfg offset from SPROM */
		reg = BHND_PCIE_SPROM_SHADOW + BHND_PCIE_SRSH_PCIE_MISC_CONFIG;
		cfg = bhnd_bus_read_2(sc->dev, sc->core, reg);

		/* Set NOPRST enable flag if not already set */
		if (!(cfg & BHND_PCIE_SRSH_L23READY_EXIT_NOPRST)) {
			cfg |= BHND_PCIE_SRSH_L23READY_EXIT_NOPRST;
			bhnd_bus_write_2(sc->dev, sc->core, reg, cfg);
		}
	}
}

static device_method_t bhnd_pci_hostb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_pci_hostb_probe),
	DEVMETHOD(device_attach,	bhnd_pci_hostb_attach),
	DEVMETHOD(device_detach,	bhnd_pci_hostb_detach),
	DEVMETHOD(device_suspend,	bhnd_pci_hostb_suspend),
	DEVMETHOD(device_resume,	bhnd_pci_hostb_resume),

	/* Bus interface */
	DEVMETHOD(bus_add_child,	device_add_child_ordered),

	DEVMETHOD_END
};

static devclass_t bhnd_pci_hostb_devclass;

DEFINE_CLASS_0(bhnd_pci_hostb, bhnd_pci_hostb_driver, bhnd_pci_hostb_methods, 
    sizeof(struct bhnd_pci_hostb_softc));

DRIVER_MODULE(bhnd_pci_hostb, bhnd, bhnd_pci_hostb_driver,
    bhnd_pci_hostb_devclass, 0, 0);

MODULE_VERSION(bhnd_pci_hostb, 1);
MODULE_DEPEND(bhnd_pci_hostb, pci, 1, 1, 1);
MODULE_DEPEND(bhnd_pci_hostb, bhnd_pci, 1, 1, 1);