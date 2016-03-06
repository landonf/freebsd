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
 * Broadcom BHND PCI/PCIe-Gen1 Host Bridge.
 * 
 * This driver handles all interactions with the PCI bridge core operating in
 * endpoint mode.
 * 
 * Host-level PCI operations are handled at the bhndb bridge level by the
 * bhndb_pci driver.
 */

#include <sys/param.h>
#include <sys/kernel.h>

#include <sys/malloc.h>

#include <sys/bus.h>
#include <sys/module.h>

#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_pcie_mdiovar.h"
#include "bhnd_pcireg.h"
#include "bhnd_pci_hostbvar.h"

struct bhnd_pci_device;


static const struct bhnd_pci_device	*bhnd_pci_device_find(
					     struct bhnd_core_info *core);
static uint32_t				 bhnd_pcie_read_proto_reg(
					     struct bhnd_pcihb_softc *sc,
					     uint32_t addr);
static void				 bhnd_pcie_write_proto_reg(
					     struct bhnd_pcihb_softc *sc,
					     uint32_t addr, uint32_t val);

/* quirk convenience macros */
#define	BHND_PCI_QUIRK(_sc, _name)	\
    ((_sc)->quirks & BHND_PCI_QUIRK_ ## _name)
#define	BHND_PCIE_QUIRK(_sc, _name)	\
    ((_sc)->quirks & BHND_PCIE_QUIRK_ ## _name)

#define	BHND_PCI_ASSERT_QUIRK(_sc, name)	\
    KASSERT(BHND_PCI_QUIRK((_sc), name), ("quirk " __STRING(_name) " not set"))
#define	BHND_PCIE_ASSERT_QUIRK(_sc, name)	\
    KASSERT(BHND_PCIE_QUIRK((_sc), name), ("quirk " __STRING(_name) " not set"))

/* Declare a bhnd_pci_device entry */
#define	BHND_PCI_DEV(_device, _desc, ...)	{	\
	BHND_COREID_ ## _device, 			\
	BHND_PCI_REGFMT_ ## _device,			\
	(struct bhnd_device_quirk[]) {			\
		__VA_ARGS__				\
	}						\
}

static const struct resource_spec bhnd_pci_rspec[BHND_PCIHB_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

/*
 * Supported PCI bridge cores.
 *
 * This table defines quirks specific to core hwrev ranges; see also
 * bhndb_pci_discover_quirks() for additional quirk detection.
 */
static const struct bhnd_pci_device {
	uint16_t			 device;	/**< bhnd device ID */
	bhnd_pci_regfmt_t		 regfmt;	/**< register format */
	struct bhnd_device_quirk	*quirks;	/**< quirks table */
} bhnd_pci_devices[] = {
	/* PCI */
	BHND_PCI_DEV(PCI,
		BHND_QUIRK_HWREV_GTE	(11,	BHND_PCI_QUIRK_SBTOPCI2_READMULTI |
						BHND_PCI_QUIRK_CLKRUN_DSBL),
		BHND_QUIRK_HWREV_END
	),

	/* PCI Gen 1 */
	BHND_PCI_DEV(PCIE,
		BHND_QUIRK_HWREV_EQ	(0,	BHND_PCIE_QUIRK_SDR9_L0s_HANG),
		BHND_QUIRK_HWREV_RANGE	(0, 1,	BHND_PCIE_QUIRK_UR_STATUS_FIX),
		BHND_QUIRK_HWREV_EQ	(1,	BHND_PCIE_QUIRK_PCIPM_REQEN),
		BHND_QUIRK_HWREV_RANGE	(3, 5,	BHND_PCIE_QUIRK_ASPM_OVR |
						BHND_PCIE_QUIRK_SDR9_POLARITY |
						BHND_PCIE_QUIRK_SDR9_NO_FREQRETRY),
		BHND_QUIRK_HWREV_LTE	(6,	BHND_PCIE_QUIRK_L1_IDLE_THRESH),
		BHND_QUIRK_HWREV_GTE	(6,	BHND_PCIE_QUIRK_SPROM_L23_PCI_RESET),
		BHND_QUIRK_HWREV_EQ	(7,	BHND_PCIE_QUIRK_SERDES_NOPLLDOWN),
		BHND_QUIRK_HWREV_GTE	(8,	BHND_PCIE_QUIRK_L1_TIMER_PERF),
		BHND_QUIRK_HWREV_GTE	(10,	BHND_PCIE_QUIRK_SD_C22_EXTADDR),
		BHND_QUIRK_HWREV_END
	),

	{ BHND_COREID_INVALID, BHND_PCI_REGFMT_PCI, NULL }
};

#if 0 // TODO

	// TODO This gets applied to *all* PCI core revisions
	/**
	 * SBTOPCI_PREF and SBTOPCI_BURST must be set on the
	 * SSB_PCICORE_SBTOPCI2 register.
	 */
	BHND_PCI_QUIRK_SBTOPCI2_PREF_BURST	= (1<<2),

#endif


#if 0
static int	bhndb_pci_wars_register_access(struct bhnd_pcihb_softc *sc);
static int	bhndb_pci_wars_early_once(struct bhnd_pcihb_softc *sc);
static int	bhndb_pci_wars_hwup(struct bhnd_pcihb_softc *sc);
static int	bhndb_pci_wars_hwdown(struct bhnd_pcihb_softc *sc);

static uint32_t	bhndb_pci_discover_quirks(struct bhnd_pcihb_softc *,
		    const struct bhndb_pci_id *);
#endif

static int
bhnd_pci_hostb_probe(device_t dev)
{
	struct bhnd_core_info	cid;
	int			error;

	cid = bhnd_get_core_info(dev);

	/* Look for a device table entry */
	if (bhnd_pci_device_find(&cid) == NULL)
		return (ENXIO);

	/* Core must be the host bridge device */
	if (!bhnd_is_hostb_device(dev))
		return (ENXIO);

	/* Identify children */
	if ((error = bus_generic_probe(dev)))
		return (error);

	bhnd_set_generic_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
bhnd_pci_hostb_attach(device_t dev)
{
	struct bhnd_pcihb_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Allocate bus resources */
	memcpy(sc->rspec, bhnd_pci_rspec, sizeof(sc->rspec));
	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	sc->core = sc->res[0];

	BHND_PCI_LOCK_INIT(sc);

	if ((error = bus_generic_attach(dev)))
		goto cleanup;

	sc->mdio = device_find_child(dev,
	    devclass_get_name(bhnd_pcie_mdio_devclass), 0);
	if (bhnd_get_class(dev) == BHND_DEVCLASS_PCIE && sc->mdio == NULL) {
		device_printf(dev, "failed to attach MDIO device\n");
		error = ENXIO;
		goto cleanup;
	}

cleanup:
	BHND_PCI_LOCK_DESTROY(sc);
	bhnd_release_resources(dev, sc->rspec, sc->res);

	return (error);
#if 0
	
	/*
	 * Attach MDIO device (if this is a PCIe device), which is used for
	 * access to the PCIe SerDes required by the quirk workarounds.
	 */
	if (sc->pci_devclass == BHND_DEVCLASS_PCIE) {
		sc->mdio = BUS_ADD_CHILD(dev, 0,
		    devclass_get_name(bhnd_mdio_pci_devclass), 0);
		if (sc->mdio == NULL)
			return (ENXIO);
		
		error = bus_set_resource(sc->mdio, SYS_RES_MEMORY, 0,
		    rman_get_start(sc->mem_res) + sc->mem_off +
		    BHND_PCIE_MDIO_CTL, sizeof(uint32_t)*2);
		if (error) {
			device_printf(dev, "failed to set MDIO resource\n");
			return (error);
		}

		if ((error = device_probe_and_attach(sc->mdio))) {
			device_printf(dev, "failed to attach MDIO device\n");
			return (error);
		}
	}
#endif
}

static int
bhnd_pci_hostb_detach(device_t dev)
{
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
	return (bus_generic_resume(dev));
}

static struct resource_list *
bhnd_pci_get_resource_list(device_t dev, device_t child)
{
	struct bhnd_pci_devinfo *dinfo;

	if (device_get_parent(child) != dev)
		return (NULL);

	dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

static device_t
bhnd_pci_add_child(device_t dev, u_int order, const char *name, int unit)
{
	struct bhnd_pci_devinfo	*dinfo;
	device_t		 child;
	
	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return (NULL);

	dinfo = malloc(sizeof(struct bhnd_pci_devinfo), M_DEVBUF, M_NOWAIT);
	if (dinfo == NULL) {
		device_delete_child(dev, child);
		return (NULL);
	}

	resource_list_init(&dinfo->resources);
	
	device_set_ivars(child, dinfo);
	return (child);
}

static void
bhnd_pci_child_deleted(device_t dev, device_t child)
{
	struct bhnd_pci_devinfo *dinfo;

	if (device_get_parent(child) != dev)
		return;

	dinfo = device_get_ivars(child);
	if (dinfo != NULL) {
		resource_list_free(&dinfo->resources);
		free(dinfo, M_DEVBUF);
	}

	device_set_ivars(child, NULL);
}

/**
 * Find the identification table entry for a core descriptor.
 */
static const struct bhnd_pci_device *
bhnd_pci_device_find(struct bhnd_core_info *core)
{
	const struct bhnd_pci_device *id;

	for (id = bhnd_pci_devices; id->device != BHND_COREID_INVALID; id++) {
		if (core->vendor == BHND_MFGID_BCM && 
		    core->device == id->device)
			return (id);
	}

	return (NULL);
}

// Quirk TODO
// WARs for the following are not yet implemented:
// - BHND_PCIE_QUIRK_ASPM_OVR
// - BHND_PCIE_QUIRK_SERDES_NOPLLDOWN
// Quirks (and WARs) for the following are not yet defined:
// - Power savings via MDIO BLK1/PWR_MGMT3 on PCIe hwrev 15-20, 21-22
// - WOWL PME enable/disable
// - 4360 PCIe SerDes Tx amplitude/deemphasis (vendor Apple, boards
//   BCM94360X51P2, BCM94360X51A).
// - PCI latency timer (boards CB2_4321_BOARD, CB2_4321_AG_BOARD)
// - Max SerDes TX drive strength (vendor Apple, pcie >= rev10,
//   board BCM94322X9)
// - 700mV SerDes TX drive strength (chipid BCM4331, boards BCM94331X19,
//   BCM94331X28, BCM94331X29B, BCM94331X19C)

#if 0

/**
 * Apply any hardware work-arounds that must be executed exactly once, early in
 * the attach process.
 * 
 * This must be called after core enumeration and discovery of all applicable
 * quirks, but prior to probe/attach of any cores, parsing of
 * SPROM, etc.
 */
static int
bhndb_pci_wars_early_once(struct bhnd_pcihb_softc *sc)
{
	/* Determine correct polarity by observing the attach-time PCIe PHY
	 * link status. This is used later to reset/force the SerDes
	 * polarity */
	if (BHNDB_PCIE_QUIRK(sc, SDR9_POLARITY)) {
		uint32_t st;
		bool inv;


		st = bhndb_pcie_read_proto_reg(sc, BHND_PCIE_PLP_STATUSREG);
		inv = ((st & BHND_PCIE_PLP_POLARITY_INV) != 0);
		sc->sdr9_quirk_polarity.inv = inv;
	}

	return (0);
}

/**
 * Apply any hardware workarounds that are required upon attach or resume
 * of the bridge device.
 */
static int
bhndb_pci_wars_hwup(struct bhnd_pcihb_softc *sc)
{
	/* Note that the order here matters; these work-arounds
	 * should not be re-ordered without careful review of their
	 * interdependencies */

	/* Fix up any PoR defaults on SROMless devices */
	bhndb_init_sromless_pci_config(sc);

	/* Enable PCI prefetch/burst/readmulti flags */
	if (BHNDB_PCI_QUIRK(sc, SBTOPCI2_PREF_BURST) ||
	    BHNDB_PCI_QUIRK(sc, SBTOPCI2_READMULTI))
	{
		uint32_t sbp2;
		sbp2 = BHNDB_PCI_READ_4(sc, BHND_PCI_SBTOPCI2);

		if (BHNDB_PCI_QUIRK(sc, SBTOPCI2_PREF_BURST))
			sbp2 |= (BHND_PCI_SBTOPCI_PREF|BHND_PCI_SBTOPCI_BURST);
		
		if (BHNDB_PCI_QUIRK(sc, SBTOPCI2_READMULTI))
			sbp2 |= BHND_PCI_SBTOPCI_RC_READMULTI;

		BHNDB_PCI_WRITE_4(sc, BHND_PCI_SBTOPCI2, sbp2);
	}

	/* Disable PCI CLKRUN# */
	if (BHNDB_PCI_QUIRK(sc, CLKRUN_DSBL)) {
		uint32_t ctl;
	
		ctl = BHNDB_PCI_READ_4(sc, BHND_PCI_CLKRUN_CTL);
		ctl |= BHND_PCI_CLKRUN_DSBL;
		BHNDB_PCI_WRITE_4(sc, BHND_PCI_CLKRUN_CTL, ctl);
	}
	
	/* Enable TLP unmatched address handling work-around */
	if (BHNDB_PCIE_QUIRK(sc, UR_STATUS_FIX)) {
		uint32_t wrs;
		wrs = bhndb_pcie_read_proto_reg(sc, BHND_PCIE_TLP_WORKAROUNDSREG);
		wrs |= BHND_PCIE_TLP_WORKAROUND_URBIT;
		bhndb_pcie_write_proto_reg(sc, BHND_PCIE_TLP_WORKAROUNDSREG, wrs);
	}

	/* Adjust SerDes CDR tuning to ensure that CDR is stable before sending
	 * data during L0s to L0 exit transitions. */
	if (BHNDB_PCIE_QUIRK(sc, SDR9_L0s_HANG)) {
		uint16_t sdv;

		/* Set RX track/acquire timers to 2.064us/40.96us */
		sdv = BPCI_REG_INSERT(0, PCIE_SDR9_RX_TIMER1_LKTRK, (2064/16));
		sdv = BPCI_REG_INSERT(sdv, PCIE_SDR9_RX_TIMER1_LKACQ,
		    (40960/1024));
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_TIMER1, sdv);

		/* Apply CDR frequency workaround */
		sdv = BHND_PCIE_SDR9_RX_CDR_FREQ_OVR_EN;
		sdv = BPCI_REG_INSERT(sdv, PCIE_SDR9_RX_CDR_FREQ_OVR, 0x0);
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CDR, sdv);

		/* Apply CDR BW tunings */
		sdv = 0;
		sdv = BPCI_REG_INSERT(sdv, PCIE_SDR9_RX_CDRBW_INTGTRK, 0x2);
		sdv = BPCI_REG_INSERT(sdv, PCIE_SDR9_RX_CDRBW_INTGACQ, 0x4);
		sdv = BPCI_REG_INSERT(sdv, PCIE_SDR9_RX_CDRBW_PROPTRK, 0x6);
		sdv = BPCI_REG_INSERT(sdv, PCIE_SDR9_RX_CDRBW_PROPACQ, 0x6);
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CDRBW, sdv);
	}

	/* Force correct SerDes polarity */
	if (BHNDB_PCIE_QUIRK(sc, SDR9_POLARITY)) {
		uint16_t	rxctl;

		rxctl = MDIO_READREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CTRL);

		rxctl |= BHND_PCIE_SDR9_RX_CTRL_FORCE;
		if (sc->sdr9_quirk_polarity.inv)
			rxctl |= BHND_PCIE_SDR9_RX_CTRL_POLARITY_INV;
		else
			rxctl &= ~BHND_PCIE_SDR9_RX_CTRL_POLARITY_INV;

		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_TXRX,
		    BHND_PCIE_SDR9_RX_CTRL, rxctl);
	}

	/* Disable startup retry on PLL frequency detection failure */
	if (BHNDB_PCIE_QUIRK(sc, SDR9_NO_FREQRETRY)) {
		uint16_t	pctl;

		pctl = MDIO_READREG(sc->mdio, BHND_PCIE_PHY_SDR9_PLL,
		    BHND_PCIE_SDR9_PLL_CTRL);

		pctl &= ~BHND_PCIE_SDR9_PLL_CTRL_FREQDET_EN;
		MDIO_WRITEREG(sc->mdio, BHND_PCIE_PHY_SDR9_PLL,
		    BHND_PCIE_SDR9_PLL_CTRL, pctl);
	}
	
	/* Explicitly enable PCI-PM */
	if (BHNDB_PCIE_QUIRK(sc, PCIPM_REQEN)) {
		uint32_t lcreg;
		lcreg = bhndb_pcie_read_proto_reg(sc, BHND_PCIE_DLLP_LCREG);
		lcreg |= BHND_PCIE_DLLP_LCREG_PCIPM_EN;
		bhndb_pcie_write_proto_reg(sc, BHND_PCIE_DLLP_LCREG, lcreg);
	}

	/* Adjust L1 timer to fix slow L1->L0 transitions */
	if (BHNDB_PCIE_QUIRK(sc, L1_IDLE_THRESH)) {
		uint32_t pmt;
		pmt = bhndb_pcie_read_proto_reg(sc, BHND_PCIE_DLLP_PMTHRESHREG);
		pmt = BPCI_REG_INSERT(pmt, PCIE_L1THRESHOLDTIME,
		    BHND_PCIE_L1THRESHOLD_WARVAL);
		bhndb_pcie_write_proto_reg(sc, BHND_PCIE_DLLP_PMTHRESHREG, pmt);
	}

	/* Extend L1 timer for better performance.
	 * TODO: We could enable/disable this on demand for better power
	 * savings if we tie this to HT clock request handling */
	if (BHNDB_PCIE_QUIRK(sc, L1_TIMER_PERF)) {
		uint32_t pmt;
		pmt = bhndb_pcie_read_proto_reg(sc, BHND_PCIE_DLLP_PMTHRESHREG);
		pmt |= BHND_PCIE_ASPMTIMER_EXTEND;
		bhndb_pcie_write_proto_reg(sc, BHND_PCIE_DLLP_PMTHRESHREG, pmt);
	}

	/* Enable L23READY_EXIT_NOPRST if not already set in SPROM. */
	if (BHNDB_PCIE_QUIRK(sc, SPROM_L23_PCI_RESET)) {
		bus_size_t	reg;
		uint16_t	cfg;

		/* Fetch the misc cfg flags from SPROM */
		reg = BHND_PCIE_SPROM_SHADOW + BHND_PCIE_SRSH_PCIE_MISC_CONFIG;
		cfg = BHNDB_PCI_READ_2(sc, reg);

		/* Write EXIT_NOPRST flag if not already set in SPROM */
		if (!(cfg & BHND_PCIE_SRSH_L23READY_EXIT_NOPRST)) {
			cfg |= BHND_PCIE_SRSH_L23READY_EXIT_NOPRST;
			BHNDB_PCI_WRITE_2(sc, reg, cfg);
		}
	}

	return (0);
}

/**
 * Apply any hardware workarounds that are required upon resume of the
 * bridge device.
 * 
 * This must be called before any bridged bhnd(4) cores have been resumed.
 */
static int
bhndb_pci_wars_hwresume(struct bhnd_pcihb_softc *sc)
{
	int error;

	/* Nothing is possible without register access */
	if ((error = bhndb_pci_wars_register_access(sc)))
		return (error);

	/* Apply the general hwup workarounds */
	return (bhndb_pci_wars_hwup(sc));
}

/**
 * Apply any hardware workarounds that are required upon detach or suspend
 * of the bridge device.
 */
static int
bhndb_pci_wars_hwdown(struct bhnd_pcihb_softc *sc)
{
	int error;
	
	/* Reduce L1 timer for better power savings.
	 * TODO: We could enable/disable this on demand for better power
	 * savings if we tie this to HT clock request handling */
	if (BHNDB_PCIE_QUIRK(sc, L1_TIMER_PERF)) {
		uint32_t pmt;
		pmt = bhndb_pcie_read_proto_reg(sc, BHND_PCIE_DLLP_PMTHRESHREG);
		pmt &= ~BHND_PCIE_ASPMTIMER_EXTEND;
		bhndb_pcie_write_proto_reg(sc, BHND_PCIE_DLLP_PMTHRESHREG, pmt);
	}

	/* Disable clocks */
	if (BHNDB_PCI_QUIRK(sc, EXT_CLOCK_GATING)) {
		if ((error = bhndb_disable_pci_clocks(sc))) {
			device_printf(sc->dev, "failed to disable clocks\n");
			return (error);
		}
	}

	return (0);
}
#endif

/**
 * Read a 32-bit PCIe TLP/DLLP/PLP protocol register.
 * 
 * @param sc The bhndb_pci driver state.
 * @param addr The protocol register offset.
 */
static uint32_t
bhnd_pcie_read_proto_reg(struct bhnd_pcihb_softc *sc, uint32_t addr)
{
	uint32_t val;

	KASSERT(bhnd_get_class(sc->dev) == BHND_DEVCLASS_PCIE,
	    ("not a pcie device!"));

	BHND_PCI_LOCK(sc);
	bhnd_bus_write_4(sc->core, BHND_PCIE_IND_ADDR, addr);
	val = bhnd_bus_read_4(sc->core, BHND_PCIE_IND_DATA);
	BHND_PCI_UNLOCK(sc);

	return (val);
}

/**
 * Write a 32-bit PCIe TLP/DLLP/PLP protocol register value.
 * 
 * @param sc The bhndb_pci driver state.
 * @param addr The protocol register offset.
 * @param val The value to write to @p addr.
 */
static void
bhnd_pcie_write_proto_reg(struct bhnd_pcihb_softc *sc, uint32_t addr,
    uint32_t val)
{
	KASSERT(bhnd_get_class(sc->dev) == BHND_DEVCLASS_PCIE,
	    ("not a pcie device!"));

	BHND_PCI_LOCK(sc);
	bhnd_bus_write_4(sc->core, BHND_PCIE_IND_ADDR, addr);
	bhnd_bus_write_4(sc->core, BHND_PCIE_IND_DATA, val);
	BHND_PCI_UNLOCK(sc);
}

#if 0

/**
 * Return all quirks known to be applicable to the host bridge.
 * 
 * If the PCI bridge core has not yet been identified, no core-specific
 * quirk flags will be returned. This function may be called again to
 * rediscover applicable quirks after the host bridge core has been
 * identified.
 * 
 * @param sc bhndb PCI driver state.
 * @param id The host bridge core's identification table entry, or NULL
 * if the host bridge core has not yet been identified.
 * 
 * @return Returns the set of quirks applicable to the current hardware.
 */
static uint32_t 
bhndb_pci_discover_quirks(struct bhnd_pcihb_softc *sc,
    const struct bhndb_pci_id *id)
{
	struct bhnd_device_quirk	*qt;
	uint32_t			 quirks;
	uint8_t				 hwrev;

	quirks = BHNDB_PCI_QUIRK_NONE;

	/* Determine any device class-specific quirks */
	switch (sc->pci_devclass) {
	case BHND_DEVCLASS_PCI:
		/* All PCI devices require external clock gating */
		sc->quirks |= BHNDB_PCI_QUIRK_EXT_CLOCK_GATING;
		break;
	default:
		break;
	}

	// TODO: Additional quirk matching

	/* Determine any PCI core hwrev-specific device quirks */
	if (id != NULL) {
		hwrev = bhnd_get_hwrev(sc->bhndb.hostb_dev);
		for (qt = id->quirks; qt->quirks != 0; qt++) {
			if (bhnd_hwrev_matches(hwrev, &qt->hwrev))
				quirks |= qt->quirks;
		};
	}


	return (quirks);
}
#endif

static device_method_t bhnd_pci_hostb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_pci_hostb_probe),
	DEVMETHOD(device_attach,		bhnd_pci_hostb_attach),
	DEVMETHOD(device_detach,		bhnd_pci_hostb_detach),
	DEVMETHOD(device_suspend,		bhnd_pci_hostb_suspend),
	DEVMETHOD(device_resume,		bhnd_pci_hostb_resume),

	/* Bus interface */
	DEVMETHOD(bus_add_child,		bhnd_pci_add_child),
	DEVMETHOD(bus_child_deleted,		bhnd_pci_child_deleted),
	DEVMETHOD(bus_print_child,		bus_generic_print_child),
	DEVMETHOD(bus_get_resource_list,	bhnd_pci_get_resource_list),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),


	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pci_hostb, bhnd_pci_hostb_driver, bhnd_pci_hostb_methods, 
    sizeof(struct bhnd_pcihb_softc));

DRIVER_MODULE(bhnd_hostb, bhnd, bhnd_pci_hostb_driver, bhnd_hostb_devclass, 0, 0);

MODULE_VERSION(bhnd_pci_hostb, 1);
MODULE_DEPEND(bhnd_pci_hostb, pci, 1, 1, 1);
MODULE_DEPEND(bhnd_pci_hostb, bhnd_pci, 1, 1, 1);
