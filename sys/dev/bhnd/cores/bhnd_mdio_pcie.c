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
 * Broadcom PCIe MDIO Driver
 * 
 * Provides access to the PCIe-G1 core's MDIO interface; the PCIe SerDes and other
 * devices' management interfaces are accessible via MDIO.
 * 
 * The PCIe-G1 device uses the generic BHND MDIO interface supported by
 * bhnd_mdio; this driver subclasses the common bhnd_mdio driver to fetch
 * a borrowed reference to the parent's PCIe register block during attachment.
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

#include "bhnd_pcireg.h"
#include "bhnd_pci_hostbvar.h"

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
	device_set_desc(dev, "Broadcom MDIO");
	device_quiet(dev);

	return (BUS_PROBE_DEFAULT);
}

/*
 * Fetch a borrowed reference to a parent bhnd_pci_hostb's register block
 * during attach.
 */
static int
bhnd_mdio_pcie_hb_attach(device_t dev)
{
	struct bhnd_pci_hostb_softc	*parent_sc;
	device_t			 parent;

	parent = device_get_parent(dev);
	parent_sc = device_get_softc(parent);

	return (bhnd_mdio_attach(dev, parent_sc->core, -1, BHND_PCIE_MDIO_CTL));
}

static device_method_t bhnd_mdio_pci_hb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	bhnd_mdio_pcie_identify),
	DEVMETHOD(device_probe,		bhnd_mdio_pcie_probe),
	DEVMETHOD(device_attach,	bhnd_mdio_pcie_hb_attach),
	DEVMETHOD_END
};

devclass_t bhnd_mdio_pcie;

DEFINE_CLASS_1(bhnd_mdio_pcie, bhnd_mdio_pci_hb_driver,
    bhnd_mdio_pci_hb_methods, sizeof(struct bhnd_mdio_softc), bhnd_mdio_driver);

DRIVER_MODULE(bhnd_mdio_pcie, bhnd_pci_hostb, bhnd_mdio_pci_hb_driver,
    bhnd_mdio_pcie, 0, 0);
