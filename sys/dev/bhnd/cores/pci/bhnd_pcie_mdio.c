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
 * MDIO Driver for PCIe-G1 Cores (All Revisions).
 * 
 * The MDIO interface provides access to the PCIe SerDes management registers.
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
#include "bhnd_pcie_mdioreg.h"
#include "bhnd_pcie_mdiovar.h"
#include "bhnd_pci_hostbvar.h"

devclass_t bhnd_pcie_mdio_devclass;

/* Supported device identifiers */
static const struct bhnd_pci_device {
	uint16_t	 vendor;
	uint16_t	 device;
	const char	*desc;
} mdio_devices[] = {
	{ BHND_MFGID_BCM,	BHND_COREID_PCIE,
	    "Broadcom PCIe-Gen1 MDIO bus" },

	// TODO: PCIe-Gen2 Support
	// { BHND_MFGID_BCM,	BHND_COREID_PCIE2,
	//  "Broadcom PCIe-Gen2 MDIO bus" },

	{ BHND_MFGID_INVALID, BHND_COREID_INVALID, NULL }
};

#define	BHND_MDIO_CTL_DELAY	10	/**< usec delay required between
					  *  MDIO_CTL/MDIO_DATA accesses. */
#define	BHND_MDIO_RETRY_DELAY	2000	/**< usec delay before retrying
					  *  BHND_MDIOCTL_DONE. */
#define	BHND_MDIO_RETRY_COUNT	200	/**< number of times to loop waiting
					  *  for BHND_MDIOCTL_DONE. */

#define	BHND_MDIO_READ_4(_sc, _reg)	\
	bhnd_bus_read_4((_sc)->res, (_reg))

#define	BHND_MDIO_WRITE_4(_sc, _reg, _val)		\
	bhnd_bus_write_4((_sc)->res, (_reg), (_val))

static int
bhnd_pcie_mdio_probe(device_t dev)
{
	const struct bhnd_pci_device	*id;
	device_t			 parent;

	parent = device_get_parent(dev);
	if (device_get_driver(parent) != &bhnd_pci_hostb_driver)
		return (ENXIO);

	/* We attach once, and will always the first device probed/attached */
	if (device_find_child(parent,
		devclass_get_name(bhnd_pcie_mdio_devclass), 0) != dev)
	{
		return (ENXIO);
	}

	for (id = mdio_devices; id->device != BHND_COREID_INVALID; id++)
	{
		if (bhnd_get_vendor(parent) != id->vendor)
			continue;
		
		if (bhnd_get_device(parent) != id->device)
			continue;

		device_set_desc(dev, id->desc);
		// device_quiet(dev); TODO

		return (BUS_PROBE_SPECIFIC);
	}

	/* Unsupported PCI core */
	return (ENXIO);
}

static int
bhnd_pcie_mdio_attach(device_t dev)
{
	struct bhnd_pcie_mdio_softc *sc = device_get_softc(dev);

	sc->dev = dev;
	// TODO
//	sc->c22ext = c22ext;
//	sc->c22ext = false;

	/* Allocate our MDIO register block */
	sc->rid = 0;
	sc->res = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate MDIO register block\n");
		return (ENXIO);
	}

	BHND_PCIE_MDIO_LOCK_INIT(sc);
	return (bus_generic_attach(dev));
}

static int
bhnd_pcie_mdio_detach(device_t dev)
{
	struct bhnd_pcie_mdio_softc *sc = device_get_softc(dev);
	
	BHND_PCIE_MDIO_LOCK_DESTROY(sc);
	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);

	return (0);
}

/* Spin until the MDIO device reports itself as idle, or timeout is reached. */
static int
bhnd_pcie_mdio_wait_idle(struct bhnd_pcie_mdio_softc *sc)
{
	uint32_t ctl;

	/* Spin waiting for the BUSY flag to clear */
	for (int i = 0; i < BHND_MDIO_RETRY_COUNT; i++) {
		ctl = BHND_MDIO_READ_4(sc, BHND_MDIO_CTL);
		if ((ctl & BHND_MDIOCTL_DONE))
			return (0);

		DELAY(BHND_MDIO_RETRY_DELAY);
	}

	return (ETIMEDOUT);
}


/**
 * Write an MDIO IOCTL and wait for completion.
 */
static int
bhnd_pcie_mdio_ioctl(struct bhnd_pcie_mdio_softc *sc, uint32_t cmd)
{
	BHND_PCIE_MDIO_LOCK_ASSERT(sc, MA_OWNED);

	BHND_MDIO_WRITE_4(sc, BHND_MDIO_CTL, cmd);
	DELAY(BHND_MDIO_CTL_DELAY);
	return (0);
}

/**
 * Enable MDIO device
 */
static int
bhnd_pcie_mdio_enable(struct bhnd_pcie_mdio_softc *sc)
{
	uint32_t ctl;

	/* Enable MDIO clock and preamble mode */
	ctl = BHND_MDIOCTL_PREAM_EN|BHND_MDIOCTL_DIVISOR_VAL;
	return (bhnd_pcie_mdio_ioctl(sc, ctl));
}

/**
 * Disable MDIO device.
 */
static void
bhnd_pcie_mdio_disable(struct bhnd_pcie_mdio_softc *sc)
{
	if (bhnd_pcie_mdio_ioctl(sc, 0))
		device_printf(sc->dev, "failed to disable MDIO clock\n");
}


/**
 * Issue a write command and wait for completion
 */
static int
bhnd_pcie_mdio_cmd_write(struct bhnd_pcie_mdio_softc *sc, uint32_t cmd)
{
	int error;

	BHND_PCIE_MDIO_LOCK_ASSERT(sc, MA_OWNED);

	cmd |= BHND_MDIODATA_START|BHND_MDIODATA_TA|BHND_MDIODATA_CMD_WRITE;

	BHND_MDIO_WRITE_4(sc, BHND_MDIO_DATA, cmd);
	DELAY(BHND_MDIO_CTL_DELAY);

	if ((error = bhnd_pcie_mdio_wait_idle(sc)))
		return (error);

	return (0);
}

/**
 * Issue an an MDIO read command, wait for completion, and return
 * the result in @p data_read.
 */
static int
bhnd_pcie_mdio_cmd_read(struct bhnd_pcie_mdio_softc *sc, uint32_t cmd,
    uint16_t *data_read)
{
	int error;

	BHND_PCIE_MDIO_LOCK_ASSERT(sc, MA_OWNED);

	cmd |= BHND_MDIODATA_START|BHND_MDIODATA_TA|BHND_MDIODATA_CMD_READ;
	BHND_MDIO_WRITE_4(sc, BHND_MDIO_DATA, cmd);
	DELAY(BHND_MDIO_CTL_DELAY);

	if ((error = bhnd_pcie_mdio_wait_idle(sc)))
		return (error);

	*data_read = (BHND_MDIO_READ_4(sc, BHND_MDIO_DATA) & 
	    BHND_MDIODATA_DATA_MASK);
	return (0);
}


static int
bhnd_pcie_mdio_read(device_t dev, int phy, int reg)
{
	struct bhnd_pcie_mdio_softc	*sc;
	uint32_t			 cmd;
	uint16_t			 val;
	int				 error;

	sc = device_get_softc(dev);

	/* Enable MDIO access */
	BHND_PCIE_MDIO_LOCK(sc);
	bhnd_pcie_mdio_enable(sc);

	/* Issue the read */
	cmd = BHND_MDIODATA_ADDR(phy, reg);
	error = bhnd_pcie_mdio_cmd_read(sc, cmd, &val);

	/* Disable MDIO access */
	bhnd_pcie_mdio_disable(sc);
	BHND_PCIE_MDIO_UNLOCK(sc);

	if (error)
		return (~0U);

	return (val);
}

static int
bhnd_pcie_mdio_write(device_t dev, int phy, int reg, int val)
{
	struct bhnd_pcie_mdio_softc	*sc;
	uint32_t			 cmd;
	int				 error;

	sc = device_get_softc(dev);

	/* Enable MDIO access */
	BHND_PCIE_MDIO_LOCK(sc);
	bhnd_pcie_mdio_enable(sc);

	/* Issue the write */
	cmd = BHND_MDIODATA_ADDR(phy, reg) | (val & BHND_MDIODATA_DATA_MASK);
	error = bhnd_pcie_mdio_cmd_write(sc, cmd);

	/* Disable MDIO access */
	bhnd_pcie_mdio_disable(sc);
	BHND_PCIE_MDIO_UNLOCK(sc);

	return (error);
}

static int
bhnd_pcie_mdio_read_ext(device_t dev, int phy, int devaddr, int reg)
{
	struct bhnd_pcie_mdio_softc	*sc;
	uint32_t			 cmd;
	uint16_t			 blk, val;
	uint8_t				 blk_reg;
	int				 error;

	if (devaddr == MDIO_DEVADDR_NONE)
		return (MDIO_READREG(dev, phy, reg));

	sc = device_get_softc(dev);

	/* Extended register access is only supported for the SerDes device,
	 * using the non-standard C22 extended address mechanism */
	if (!sc->c22ext)
		return (~0U);	
	if (phy != BHND_PCIE_PHYADDR_SD || devaddr != BHND_PCIE_DEVAD_SD)
		return (~0U);

	/* Enable MDIO access */
	BHND_PCIE_MDIO_LOCK(sc);
	bhnd_pcie_mdio_enable(sc);

	/* Determine the block and register values */
	blk = (reg & BHND_PCIE_SD_ADDREXT_BLK_MASK);
	blk_reg = (reg & BHND_PCIE_SD_ADDREXT_REG_MASK);

	/* Write the block address to the address extension register */
	cmd = BHND_MDIODATA_ADDR(phy, BHND_PCIE_SD_ADDREXT) |
	    (blk & BHND_MDIODATA_DATA_MASK);
	if ((error = bhnd_pcie_mdio_cmd_write(sc, cmd)))
		goto cleanup;

	/* Issue the read */
	cmd = BHND_MDIODATA_ADDR(phy, blk_reg);
	error = bhnd_pcie_mdio_cmd_read(sc, cmd, &val);

cleanup:
	bhnd_pcie_mdio_disable(sc);
	BHND_PCIE_MDIO_UNLOCK(sc);

	if (error)
		return (~0U);

	return (val);
}

static int
bhnd_pcie_mdio_write_ext(device_t dev, int phy, int devaddr, int reg,
    int val)
{	
	struct bhnd_pcie_mdio_softc	*sc;
	uint32_t			 cmd;
	uint16_t			 blk;
	uint8_t				 blk_reg;
	int				 error;

	if (devaddr == MDIO_DEVADDR_NONE)
		return (MDIO_READREG(dev, phy, reg));

	sc = device_get_softc(dev);

	/* Extended register access is only supported for the SerDes device,
	 * using the non-standard C22 extended address mechanism */
	if (!sc->c22ext)
		return (~0U);	
	if (phy != BHND_PCIE_PHYADDR_SD || devaddr != BHND_PCIE_DEVAD_SD)
		return (~0U);

	/* Enable MDIO access */
	BHND_PCIE_MDIO_LOCK(sc);
	bhnd_pcie_mdio_enable(sc);

	/* Determine the block and register values */
	blk = (reg & BHND_PCIE_SD_ADDREXT_BLK_MASK);
	blk_reg = (reg & BHND_PCIE_SD_ADDREXT_REG_MASK);

	/* Write the block address to the address extension register */
	cmd = BHND_MDIODATA_ADDR(phy, BHND_PCIE_SD_ADDREXT) |
	    (blk & BHND_MDIODATA_DATA_MASK);
	if ((error = bhnd_pcie_mdio_cmd_write(sc, cmd)))
		goto cleanup;

	/* Issue the write */
	cmd = BHND_MDIODATA_ADDR(phy, blk_reg) |
	    (val & BHND_MDIODATA_DATA_MASK);
	error = bhnd_pcie_mdio_cmd_write(sc, cmd);

cleanup:
	bhnd_pcie_mdio_disable(sc);
	BHND_PCIE_MDIO_UNLOCK(sc);

	return (error);
}

static device_method_t bhnd_pcie_mdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_pcie_mdio_probe),
	DEVMETHOD(device_attach,	bhnd_pcie_mdio_attach),
	DEVMETHOD(device_detach,	bhnd_pcie_mdio_detach),

	/* MDIO interface */
	DEVMETHOD(mdio_readreg,		bhnd_pcie_mdio_read),
	DEVMETHOD(mdio_writereg,	bhnd_pcie_mdio_write),
	DEVMETHOD(mdio_readextreg,	bhnd_pcie_mdio_read_ext),
	DEVMETHOD(mdio_writeextreg,	bhnd_pcie_mdio_write_ext),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pcie_mdio, bhnd_pcie_mdio_driver, bhnd_pcie_mdio_methods, sizeof(struct bhnd_pcie_mdio_softc));
DRIVER_MODULE(bhnd_pcie_mdio, pcib, bhnd_pcie_mdio_driver, bhnd_pcie_mdio_devclass, 0, 0);
DRIVER_MODULE(bhnd_pcie_mdio, bhnd_pci_hostb, bhnd_pcie_mdio_driver, bhnd_pcie_mdio_devclass, 0, 0);
