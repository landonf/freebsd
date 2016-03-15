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
 * Broadcom Common PCI/PCIe Support.
 * 
 * This base driver implementation is shared by the bhnd_pcib (root complex)
 * and bhnd_pci_hostb (host bridge) drivers.
 */

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_pcireg.h"
#include "bhnd_pcivar.h"
#include "bhnd_pcie_mdiovar.h"

struct bhnd_pci_device;

static const struct bhnd_pci_device	*bhnd_pci_device_find(
					     const struct bhnd_core_info *core);

static const struct resource_spec bhnd_pci_rspec[BHND_PCI_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

static const struct bhnd_pci_device {
	uint16_t		 vendor;
	uint16_t	 	 device;
	bhnd_pci_regfmt_t	 regfmt;	/**< register format */
	const char		*pcib_desc;
	const char		*hostb_desc;
} bhnd_pci_devs[] = {
	{ BHND_MFGID_BCM,	BHND_COREID_PCI,	BHND_PCI_REGFMT_PCI,
	    "Host-PCI bridge",
	    "PCI-BHND bridge" },

	{ BHND_MFGID_BCM,	BHND_COREID_PCIE,	BHND_PCI_REGFMT_PCIE,
	    "Host-PCI bridge (PCIe-G1)",
	    "PCI-BHND bridge (PCIe-G1)" },

	{ BHND_MFGID_INVALID, BHND_COREID_INVALID, BHND_PCI_REGFMT_PCI,
	    NULL, NULL }
};

/**
 * Find the identification table entry for a core descriptor.
 */
static const struct bhnd_pci_device *
bhnd_pci_device_find(const struct bhnd_core_info *core)
{
	const struct bhnd_pci_device *id;

	for (id = bhnd_pci_devs; id->device != BHND_COREID_INVALID; id++) {
		if (core->vendor == id->vendor && core->device == id->device)
			return (id);
	}

	return (NULL);
}

int
bhnd_pci_generic_probe(device_t dev)
{
	const struct bhnd_pci_device	*id;
	struct bhnd_core_info		 cid;
	const char			*desc;
	bool				 hostb;

	cid = bhnd_get_core_info(dev);
	id = bhnd_pci_device_find(&cid);
	if (id == NULL)
		return (ENXIO);

	/* Select the appropriate description based on the bridge mode */
	hostb = bhnd_is_hostb_device(dev);
	desc = hostb ? id->hostb_desc : id->pcib_desc;
	bhnd_set_custom_core_desc(dev, desc);

	return (BUS_PROBE_DEFAULT);
}

int
bhnd_pci_generic_attach(device_t dev)
{
	struct bhnd_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Allocate bus resources */
	memcpy(sc->rspec, bhnd_pci_rspec, sizeof(sc->rspec));
	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	sc->core = sc->res[0];

	BHND_PCI_LOCK_INIT(sc);
	
	/*
	 * Attach MDIO device used to access to PCIe PHY registers.
	 * 
	 * This must always be done first, prior to probing additional
	 * children.
	 */
	if (bhnd_get_class(dev) == BHND_DEVCLASS_PCIE) {
		sc->mdio = BUS_ADD_CHILD(dev, 0,
		    devclass_get_name(bhnd_pcie_mdio_devclass), 0);
		if (sc->mdio == NULL) {
			error = ENXIO;
			goto cleanup;
		}

		error = bus_set_resource(sc->mdio, SYS_RES_MEMORY, 0,
		    rman_get_start(sc->core->res) + BHND_PCIE_MDIO_CTL,
		    sizeof(uint32_t)*2);
		if (error) {
			device_printf(dev, "failed to set MDIO resource\n");
			goto cleanup;
		}

		if ((error = device_probe_and_attach(sc->mdio))) {
			device_printf(dev, "failed to attach MDIO device\n");
			goto cleanup;
		}
	}

	/* Probe and attach children */
	if ((error = bus_generic_attach(dev)))
		goto cleanup;

	return (0);

cleanup:
	if (sc->mdio != NULL)
		device_delete_child(dev, sc->mdio);

	BHND_PCI_LOCK_DESTROY(sc);
	bhnd_release_resources(dev, sc->rspec, sc->res);

	return (error);
}

int
bhnd_pci_generic_detach(device_t dev)
{
	struct bhnd_pci_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bus_generic_detach(dev)))
		return (error);

	BHND_PCI_LOCK_DESTROY(sc);
	bhnd_release_resources(dev, sc->rspec, sc->res);
	return (0);
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

int
bhnd_pci_generic_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

int
bhnd_pci_generic_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

/**
 * Read a 32-bit PCIe TLP/DLLP/PLP protocol register.
 * 
 * @param sc The bhndb_pci driver state.
 * @param addr The protocol register offset.
 */
uint32_t
bhnd_pci_read_pcie_proto_reg(struct bhnd_pci_softc *sc, uint32_t addr)
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
void
bhnd_pci_write_pcie_proto_reg(struct bhnd_pci_softc *sc, uint32_t addr,
    uint32_t val)
{
	KASSERT(bhnd_get_class(sc->dev) == BHND_DEVCLASS_PCIE,
	    ("not a pcie device!"));

	BHND_PCI_LOCK(sc);
	bhnd_bus_write_4(sc->core, BHND_PCIE_IND_ADDR, addr);
	bhnd_bus_write_4(sc->core, BHND_PCIE_IND_DATA, val);
	BHND_PCI_UNLOCK(sc);
}

static device_method_t bhnd_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_pci_generic_probe),
	DEVMETHOD(device_attach,	bhnd_pci_generic_attach),
	DEVMETHOD(device_detach,	bhnd_pci_generic_detach),
	DEVMETHOD(device_suspend,	bhnd_pci_generic_suspend),
	DEVMETHOD(device_resume,	bhnd_pci_generic_resume),
	
	/* Bus interface */
	DEVMETHOD(bus_add_child,		bhnd_pci_add_child),
	DEVMETHOD(bus_child_deleted,		bhnd_pci_child_deleted),
	DEVMETHOD(bus_print_child,		bus_generic_print_child),
	DEVMETHOD(bus_get_resource_list,	bhnd_pci_get_resource_list),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),

	DEVMETHOD(bus_alloc_resource,		bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_activate_resource,        bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,      bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,          bus_generic_adjust_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_rl_release_resource),
	
	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pci, bhnd_pci_driver, bhnd_pci_methods, sizeof(struct bhnd_pci_softc));

MODULE_VERSION(bhnd_pci, 1);
MODULE_DEPEND(bhnd_pci, pci, 1, 1, 1);
