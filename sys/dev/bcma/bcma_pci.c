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
 * Generic PCI BCMA Device Support
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhnd_core.h>
#include <dev/bhnd/bhnd_device_ids.h>
#include <dev/bhnd/bhnd_pcireg.h>

#include "bcmavar.h"
#include "bcmareg.h"

#include "bcma_pcivar.h"

static const struct bcma_pci_device {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} bcma_pci_devices[] = {
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N,		"Broadcom BCM4331 802.11a/b/g/n Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N2G,	"Broadcom BCM4331 802.11b/g/n (2GHz) Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N5G,	"Broadcom BCM4331 802.11a/b/g/n (5GHz) Wireless" },
	{ 0, 0, NULL }
};

static struct resource_spec bcma_pci_res_spec[] = {
	{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
	{ SYS_RES_MEMORY,	PCIR_BAR(1),	RF_ACTIVE },
	{ -1,			0,		0 }
};

#define BPCI_RES_BAR0	0	/* bar0 pci_res index */
#define BPCI_RES_BAR1	1	/* bar1 pci_res index */

#define BMEM_RES_CHIPC	BPCI_RES_BAR0

#define bcma_bar_read(res, base, offset, size) \
	bus_read_ ## size (res, (base) + (offset))

#define bcma_read_chipc(sc, offset, size) \
	bcma_bar_read(sc->pci_res[BPCI_RES_BAR0], BHND_PCI_V2_CCREGS_OFFSET, offset, size)

static int
bcma_pci_probe(device_t dev)
{
	const struct bcma_pci_device *ident;
		
	for (ident = bcma_pci_devices; ident->vendor != 0; ident++) {
		if (pci_get_vendor(dev) == ident->vendor && pci_get_device(dev) == ident->device) {
			device_set_desc(dev, ident->desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
bcma_pci_attach(device_t dev)
{
	struct bcma_pci_softc	*sc = device_get_softc(dev);
	uint32_t		 erom_table;
	int			 error;
	bool			 free_mem_rman = false;
	bool			 free_pci_res = false;

	sc->bcma_dev = dev;
	pci_enable_busmaster(dev);
	
		
	/* Set up a resource manager for the device's address space. */
	sc->mem_rman.rm_start = 0;
	sc->mem_rman.rm_end = BUS_SPACE_MAXADDR_32BIT;
	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "bmca bus addresses";
	
	if (rman_init(&sc->mem_rman) ||
	    rman_manage_region(&sc->mem_rman, 0, BUS_SPACE_MAXADDR_32BIT))
	{
		device_printf(dev, "could not initialize mem_rman\n");
		return (ENXIO);
	} else {
		free_mem_rman = true;
	}


	/* Map our PCI device resources. */
	error = bus_alloc_resources(dev, bcma_pci_res_spec, sc->pci_res);
	if (error) {
		device_printf(dev, "could not allocate PCI resources\n");
		goto failed;
	} else {
		free_pci_res = true;
	}


	/* Locate and map the enumeration table into WIN1. A pointer to the
	 * table can be found within the ChipCommon register map. */
	erom_table = bcma_read_chipc(sc, BCMA_CC_EROM_ADDR, 4);
	pci_write_config(dev, BHND_PCI_BAR0_WIN1, erom_table, 4);

	/* Enumerate and register all bus devices. */
	error = bcma_scan_erom(dev, bhnd_generic_probecfg_table,
	    sc->pci_res[BMEM_RES_CHIPC], BHND_PCI_V2_BAR0_WIN1_OFFSET);
	if (error)
		goto failed;

	/* Let the generic implementation probe all added children. */
	return (bus_generic_attach(dev));

failed:
	if (free_mem_rman)
		rman_fini(&sc->mem_rman);

	if (free_pci_res)
		bus_release_resources(dev, bcma_pci_res_spec, sc->pci_res);

	return (error);
}

static int
bcma_pci_detach(device_t dev)
{
	struct bcma_pci_softc *sc = device_get_softc(dev);

	bus_release_resources(dev, bcma_pci_res_spec, sc->pci_res);

	return (0);
}

static int
bcma_pci_suspend(device_t dev)
{
	// TODO
	return (ENXIO);
}

static int
bcma_pci_resume(device_t dev)
{
	// TODO
	return (ENXIO);
}

static struct resource *
bcma_pci_alloc_resource(device_t dev, device_t child, int type,
    int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	struct rman			*rm;
	struct bcma_pci_softc		*sc;
	int				 error;

	if (device_get_parent(child) != dev) {
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), child,
		    type, rid, start, end, count, flags));
	}

	sc = device_get_softc(dev);

	switch (type) {
	case SYS_RES_MEMORY:
		rm = &sc->mem_rman;
		break;
	case SYS_RES_IRQ:
		// TODO!
	default:
		return (NULL);
	}

	/* Fetch the resource list entry */
	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl == NULL)
		return (NULL);

	rle = resource_list_find(rl, type, *rid);
	if (rle == NULL)
		return (NULL);

	/* Validate the resource addresses */
	if (start == 0ULL && end == ~0ULL && count <= rle->count) {
		start = rle->start;
		end = rle->end;
		count = rle->count;
	} else {
		if (start < rle->start || start > end)
			return NULL;
		
		if (end < start || end > rle->end)
			return NULL;
		
		if (count > end - start)
			return NULL;
	}

	/* Check for existing allocation */
	if (rle->res != NULL) {
		device_printf(dev,
		    "resource entry %#x type %d for child %s is busy\n", *rid,
			type, device_get_nameunit(child));
		return (NULL);
	}

	/* Make our reservation */
	rle->res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (rle->res == NULL)
		return (NULL);

	rman_set_rid(rle->res, *rid);

	/* Activate */
	if (flags & RF_ACTIVE) {
		error = bus_activate_resource(child, type, *rid, rle->res);
		if (error) {
			device_printf(dev,
			    "failed to activate entry %#x type %d for "
				"child %s\n",
			     *rid, type, device_get_nameunit(child));

			rman_release_resource(rle->res);
			rle->res = NULL;
			return (NULL);
		}
	}

	return (rle->res);
}

static int
bcma_pci_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *res)
{
	struct resource_list		*rl;
	struct resource_list_entry	*rle;
	struct bcma_pci_softc		*sc;
	int				 error;

	if (device_get_parent(child) != dev) {
		return (BUS_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, res));
	}

	sc = device_get_softc(dev);

	/* Fetch the resource list entry */
	rl = BUS_GET_RESOURCE_LIST(dev, child);
	if (rl == NULL)
		return (EINVAL);

	rle = resource_list_find(rl, type, rid);
	if (rle == NULL)
		panic("missing resource list entry");

	if (rle->res != res)
		panic("resource list entry does not match resource");

	/* Release the resource */
	if ((error = rman_release_resource(res)))
		return (error);

	rle->res = NULL;
	return (0);
}


static int
bcma_pci_adjust_resource(device_t dev, device_t child, int type,
    struct resource *res, u_long start, u_long end)
{
	if (device_get_parent(child) != dev) {
		return (BUS_ADJUST_RESOURCE(device_get_parent(dev), child,
		    type, res, start, end));
	}

	return (rman_adjust_resource(res, start, end));
}


static int
bcma_pci_activate_resource(device_t dev, device_t child, int type, int rid,
    struct resource *r)
{
	// TODO - window resource activations
	return (EINVAL);
}


static int
bcma_pci_deactivate_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	// TODO - window resource deactivations
	return (EINVAL);
}

static device_method_t bcma_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcma_pci_probe),
	DEVMETHOD(device_attach,		bcma_pci_attach),
	DEVMETHOD(device_detach,		bcma_pci_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bcma_pci_suspend),
	DEVMETHOD(device_resume,		bcma_pci_resume),
	
	/* Bus interface */
	DEVMETHOD(bus_print_child,		bcma_generic_print_child),
	DEVMETHOD(bus_probe_nomatch,		bcma_generic_probe_nomatch),
	DEVMETHOD(bus_read_ivar,		bcma_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,		bcma_generic_write_ivar),
	DEVMETHOD(bus_child_deleted,		bcma_generic_child_deleted),
	
	DEVMETHOD(bus_get_resource_list,	bcma_generic_get_resource_list),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_alloc_resource,		bcma_pci_alloc_resource),
	DEVMETHOD(bus_adjust_resource,		bcma_pci_adjust_resource),
	DEVMETHOD(bus_release_resource,		bcma_pci_release_resource),
	DEVMETHOD(bus_activate_resource,	bcma_pci_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bcma_pci_deactivate_resource),

	// TODO: Additional bus_* methods?

	/* BHND interface */
	DEVMETHOD(bhnd_get_port_rid,		bcma_generic_get_port_rid),

	DEVMETHOD_END
};

static devclass_t bcma_devclass;

DEFINE_CLASS_0(bcma, bcma_pci_driver, bcma_pci_methods, sizeof(struct bcma_pci_softc));
DRIVER_MODULE(bcma_pci, pci, bcma_pci_driver, bcma_devclass, 0, 0);
MODULE_DEPEND(bcma_pci, pci, 1, 1, 1);
MODULE_DEPEND(bcma_pci, bhnd, 1, 1, 1);