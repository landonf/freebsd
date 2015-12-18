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
 * Broadcom HND Generic MDIO Driver
 * 
 * Provides access to the MDIO interface shared across a number of Broadcom
 * cores.
 * 
 * Compatible with:
 * 	- BHND PCIe Gen1 Core
 * 	- BHND PCIe Gen2 Core (untested)
 * 	- Broadcom iProc SoC (untested)
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

#include "bhnd_mdioreg.h"
#include "bhnd_mdiovar.h"

#define	BHND_MDIO_RETRY_COUNT	200

#define	BHND_MDIO_READ_4(_sc, _reg)	\
	bhnd_bus_read_4((_sc)->dev, (_sc)->mem_res, (_sc)->mem_off + (_reg))

#define	BHND_MDIO_WRITE_4(_sc, _reg, _val)		\
	bhnd_bus_write_4((_sc)->dev, (_sc)->mem_res,	\
	    (_sc)->mem_off +  (_reg), (_val))

/**
 * Generic BHND MDIO attach implementation.
 * 
 * @param dev The bhnd_mdio device.
 * @param mem_res The memory resource containing the device resources.
 * @param mem_rid The @p mem_res resource ID, or -1 if this is a borrowed
 * reference the device should not assume ownership of.
 * @param offset The offset within @p mem_res at which the MMIO register
 * block is defined.
 */
int bhnd_mdio_attach(device_t dev, struct bhnd_resource *mem_res,
    int mem_rid, bus_size_t offset)
{
	struct bhnd_mdio_softc *sc = device_get_softc(dev);

	sc->dev = dev;
	sc->mem_rid = mem_rid;
	sc->mem_res = mem_res;
	sc->mem_off = offset;

	BHND_MDIO_LOCK_INIT(sc);

	return (bus_generic_attach(dev));

}

/* Default overridable attach method */
static int
bhnd_mdio_default_attach(device_t dev)
{
	struct bhnd_resource	*r;
	int			 rid;

	rid = 0;
	r = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (r == NULL) {
		device_printf(dev, "failed to allocate MDIO register block\n");
		return (ENXIO);
	}

	return (bhnd_mdio_attach(dev, r, rid, 0));
}

static int
bhnd_mdio_detach(device_t dev)
{
	struct bhnd_mdio_softc *sc = device_get_softc(dev);

	if (sc->mem_rid != -1)
		bhnd_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem_res);
	
	BHND_MDIO_LOCK_DESTROY(sc);

	return (bus_generic_detach(dev));
}

/* Spin until the MDIO device reports itself as idle, or timeout is reached. */
static int
bhnd_mdio_wait_idle(struct bhnd_mdio_softc *sc)
{
	uint32_t ctl;

	/* Spin waiting for the BUSY flag to clear */
	for (int i = 0; i < BHND_MDIO_RETRY_COUNT; i++) {
		ctl = BHND_MDIO_READ_4(sc, BHND_MDIO_CTL);
		if (!(ctl & BHND_MDIOCTL_BUSY))
			return (0);

		DELAY(1000);
	}

	return (ETIMEDOUT);
}


/**
 * Write an MDIO IOCTL and wait for completion.
 */
static int
bhnd_mdio_ioctl(struct bhnd_mdio_softc *sc, uint32_t cmd)
{
	int error;

	BHND_MDIO_LOCK_ASSERT(sc, MA_OWNED);

	if ((error = bhnd_mdio_wait_idle(sc))) {
		device_printf(sc->dev,
		    "timeout waiting to send cmd 0x%x\n", cmd);
		return (error);
	}

	BHND_MDIO_WRITE_4(sc, BHND_MDIO_CTL, cmd);

	if ((error = bhnd_mdio_wait_idle(sc))) {
		device_printf(sc->dev, "timeout waiting on 0x%x\n", cmd);
		return (error);
	}

	return (0);
}

/**
 * Enable MDIO device
 */
static int
bhnd_mdio_enable(struct bhnd_mdio_softc *sc)
{
	uint32_t ctl;

	/* Enable MDIO clock and preamble mode */
	ctl = BHND_MDIOCTL_PREAM_EN|BHND_MDIOCTL_DIVISOR_VAL;
	return (bhnd_mdio_ioctl(sc, ctl));
}

/**
 * Disable MDIO device.
 */
static void
bhnd_mdio_disable(struct bhnd_mdio_softc *sc)
{
	if (bhnd_mdio_ioctl(sc, 0))
		device_printf(sc->dev, "failed to disable MDIO clock\n");
}

/**
 * Issue a write command and wait for completion
 */
static int
bhnd_mdio_cmd_write(struct bhnd_mdio_softc *sc, uint32_t cmd)
{
	int error;

	BHND_MDIO_LOCK_ASSERT(sc, MA_OWNED);
	
	if ((error = bhnd_mdio_wait_idle(sc))) {
		device_printf(sc->dev,
		    "timeout waiting to write 0x%x\n", cmd);
		return (error);
	}

	cmd |= BHND_MDIODATA_START|BHND_MDIODATA_TA|BHND_MDIODATA_CMD_WRITE;

	BHND_MDIO_WRITE_4(sc, BHND_MDIO_DATA, cmd);

	if ((error = bhnd_mdio_wait_idle(sc))) {
		device_printf(sc->dev, "timeout on write 0x%x\n", cmd);
		return (error);
	}

	return (0);
}

/**
 * Issue an an MDIO read command, wait for completion, and return
 * the result in @p data_read.
 */
static int
bhnd_mdio_cmd_read(struct bhnd_mdio_softc *sc, uint32_t cmd,
    uint16_t *data_read)
{
	int error;

	BHND_MDIO_LOCK_ASSERT(sc, MA_OWNED);
	
	if ((error = bhnd_mdio_wait_idle(sc))) {
		device_printf(sc->dev,
		    "timeout waiting to write 0x%x\n", cmd);
		return (error);
	}

	cmd |= BHND_MDIODATA_START|BHND_MDIODATA_TA|BHND_MDIODATA_CMD_READ;
	BHND_MDIO_WRITE_4(sc, BHND_MDIO_DATA, cmd);

	if ((error = bhnd_mdio_wait_idle(sc))) {
		device_printf(sc->dev, "timeout on write 0x%x\n", cmd);
		return (error);
	}

	*data_read = BHND_MDIO_READ_4(sc, BHND_MDIO_DATA) & BHND_MDIODATA_MASK;
	return (0);
}

static int
bhnd_mdio_read(device_t dev, int phy, int reg)
{
	return (MDIO_READEXTREG(dev, phy, MDIO_DEVADDR_NONE, reg));
}

static int
bhnd_mdio_write(device_t dev, int phy, int reg, int val)
{
	return (MDIO_WRITEEXTREG(dev, phy, MDIO_DEVADDR_NONE, reg, val));
}

static int
bhnd_mdio_readext(device_t dev, int phy, int devaddr, int reg)
{
	struct bhnd_mdio_softc	*sc;
	uint32_t			 cmd;
	uint16_t			 val;
	int				 error;

	sc = device_get_softc(dev);

	/* Enable MDIO access */
	BHND_MDIO_LOCK(sc);
	bhnd_mdio_enable(sc);

	if (devaddr != MDIO_DEVADDR_NONE) {
		/* Set the target address */
		cmd = BHND_MDIODATA_ADDR(C45, phy, devaddr) |
		    (reg & BHND_MDIODATA_MASK);

		if ((error = bhnd_mdio_cmd_write(sc, cmd)))
			goto cleanup;

		/* Populate the read command address */
		cmd = BHND_MDIODATA_ADDR(C45, phy, devaddr);

	} else {
		/* Populate the read command address */
		cmd = BHND_MDIODATA_ADDR(C22, phy, reg);
	}
	
	if ((error = bhnd_mdio_cmd_read(sc, cmd, &val)))
		goto cleanup;

cleanup:
	/* Disable MDIO access */
	bhnd_mdio_disable(sc);
	BHND_MDIO_UNLOCK(sc);

	if (error)
		return (~0U);

	return (val);
}

static int
bhnd_mdio_writeext(device_t dev, int phy, int devaddr, int reg, int val)
{
	struct bhnd_mdio_softc	*sc;
	uint32_t			 cmd;
	int				 error;

	sc = device_get_softc(dev);

	/* Enable MDIO access */
	BHND_MDIO_LOCK(sc);
	bhnd_mdio_enable(sc);

	if (devaddr != MDIO_DEVADDR_NONE) {
		/* Set the target address */
		cmd = BHND_MDIODATA_ADDR(C45, phy, devaddr) |
		    (reg & BHND_MDIODATA_MASK);

		if ((error = bhnd_mdio_cmd_write(sc, cmd)))
			goto cleanup;

		/* Populate the write command address */
		cmd = BHND_MDIODATA_ADDR(C45, phy, devaddr);
	} else {
		/* Populate the write command address */
		cmd = BHND_MDIODATA_ADDR(C22, phy, reg);
	}

	/* Populate the cmd data */
	cmd |= (val & BHND_MDIODATA_MASK);

	/* Perform the write */
	if ((error = bhnd_mdio_cmd_write(sc, cmd)))
		goto cleanup;

cleanup:
	/* Disable MDIO access */
	bhnd_mdio_disable(sc);
	BHND_MDIO_UNLOCK(sc);

	return (error);
}

static device_method_t bhnd_mdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_attach,	bhnd_mdio_default_attach),
	DEVMETHOD(device_detach,	bhnd_mdio_detach),

	/* MDIO interface */
	DEVMETHOD(mdio_readreg,		bhnd_mdio_read),
	DEVMETHOD(mdio_writereg,	bhnd_mdio_write),
	DEVMETHOD(mdio_readextreg,	bhnd_mdio_readext),
	DEVMETHOD(mdio_writeextreg,	bhnd_mdio_writeext),

	DEVMETHOD_END
};

DEFINE_CLASS_1(mdio, bhnd_mdio_driver, bhnd_mdio_methods,
    sizeof(struct bhnd_mdio_softc), mdio_driver);

MODULE_VERSION(bhnd_mdio, 1);
MODULE_DEPEND(bhnd_mdio, bhnd, 1, 1, 1);
