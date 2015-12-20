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
 * Broadcom PCIe-G1 MDIO Driver
 * 
 * Provides access to the PCIe-G1 core's MDIO interface; the PCIe SerDes and
 * other devices' management interfaces are accessible via MDIO.
 * 
 * The PCIe-G1 device uses the generic BHND MDIO interface supported by
 * bhnd_mdio; this driver subclasses the common bhnd_mdio driver to add
 * support for:
 * 
 * - Using a borrowed reference to the parent's PCIe register block.
 * - Accessing extended SerDes registers from Clause 22 frames via the
 *   SerDes' address extension register block.

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
#include <dev/bhnd/bhnd_mdiovar.h>

#include "mdio_if.h"

#include "bhnd_pcireg.h"
#include "bhnd_pci_hostbvar.h"

struct bhnd_mdio_pcie_softc {
	struct bhnd_mdio_softc	mdio_sc;
	uint32_t		parent_quirks;	/**< parent PCIe quirk flags */
};

static void
bhnd_mdio_pcie_identify(driver_t *driver, device_t parent)
{
	const char *name = devclass_get_name(bhnd_mdio_pcie);

	if (bhnd_get_vendor(parent) != BHND_MFGID_BCM ||
	    bhnd_get_device(parent) != BHND_COREID_PCIE)
		return;

	if (device_find_child(parent, name, -1) == NULL)
		BUS_ADD_CHILD(parent, 0, name, -1);
}

static int
bhnd_mdio_pcie_probe(device_t dev)
{
	device_set_desc(dev, "Broadcom PCIe-G1 MDIO");
	device_quiet(dev);

	return (BUS_PROBE_DEFAULT);
}

/*
 * Fetch a borrowed reference to a parent bhnd_pci_hostb's register block
 * during attach.
 */
static int
bhnd_mdio_pcie_attach(device_t dev)
{
	struct bhnd_mdio_pcie_softc	*sc;
	struct bhnd_pci_hostb_softc	*parent_sc;

	sc = device_get_softc(dev);

	parent_sc = device_get_softc(device_get_parent(dev));
	sc->parent_quirks = parent_sc->quirks;

	return (bhnd_mdio_attach(dev, parent_sc->core, -1, BHND_PCIE_MDIO_CTL));
}

static int
bhnd_mdio_pcie_read_ext(device_t dev, int phy, int devaddr, int reg)
{
	struct bhnd_mdio_pcie_softc	*sc;
	uint16_t			 blk;
	uint8_t				 blk_reg;

	if (devaddr == MDIO_DEVADDR_NONE)
		return (MDIO_READREG(dev, phy, reg));

	sc = device_get_softc(dev);

	/* Only the SerDes' address extension mechanism is supported. */
	if (phy != BHND_PCIE_PHYADDR_SD)
		return (~0U);
	if (devaddr != BHND_PCIE_DEVAD_SD)
		return (~0U);
	if (!(sc->parent_quirks & BHND_PCIE_QUIRK_SD_C22_EXTADDR))
		return (~0U);

	/* Write the block address to the address extension register */
	blk = (reg & BHND_PCIE_SD_ADDREXT_BLK_MASK);
	blk_reg = (reg & BHND_PCIE_SD_ADDREXT_REG_MASK);
	if (MDIO_WRITEREG(dev, phy, BHND_PCIE_SD_ADDREXT, blk))
		return (~0U);

	/* Read the block register */
	return (MDIO_READREG(dev, phy, blk_reg));
}

static int
bhnd_mdio_pcie_write_ext(device_t dev, int phy, int devaddr, int reg,
    int val)
{	
	struct bhnd_mdio_pcie_softc	*sc;
	uint16_t			 blk;
	uint8_t				 blk_reg;

	if (devaddr == MDIO_DEVADDR_NONE)
		return (MDIO_WRITEREG(dev, phy, reg, val));

	sc = device_get_softc(dev);

	/* Only the SerDes' address extension mechanism is supported. */
	if (phy != BHND_PCIE_PHYADDR_SD)
		return (~0U);
	if (devaddr != BHND_PCIE_DEVAD_SD)
		return (~0U);
	if (!(sc->parent_quirks & BHND_PCIE_QUIRK_SD_C22_EXTADDR))
		return (~0U);

	/* Write the block address to the address extension register */
	blk = (reg & BHND_PCIE_SD_ADDREXT_BLK_MASK);
	blk_reg = (reg & BHND_PCIE_SD_ADDREXT_REG_MASK);
	if (MDIO_WRITEREG(dev, phy, BHND_PCIE_SD_ADDREXT, blk))
		return (~0U);

	/* Write the block register */
	return (MDIO_WRITEREG(dev, phy, blk_reg, val));
}

static device_method_t bhnd_mdio_pcie_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	bhnd_mdio_pcie_identify),
	DEVMETHOD(device_probe,		bhnd_mdio_pcie_probe),
	DEVMETHOD(device_attach,	bhnd_mdio_pcie_attach),

	/* MDIO interface */
	DEVMETHOD(mdio_readextreg,	bhnd_mdio_pcie_read_ext),
	DEVMETHOD(mdio_writeextreg,	bhnd_mdio_pcie_write_ext),

	DEVMETHOD_END
};

devclass_t bhnd_mdio_pcie;

DEFINE_CLASS_1(bhnd_mdio_pcie, bhnd_mdio_pcie_driver,
    bhnd_mdio_pcie_methods, sizeof(struct bhnd_mdio_softc), bhnd_mdio_driver);

DRIVER_MODULE(bhnd_mdio_pcie, bhnd_pci_hostb, bhnd_mdio_pcie_driver,
    bhnd_mdio_pcie, 0, 0);
