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

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bcma/bcma.h>

#include <dev/bhnd/bhnd_ids.h>
#include <dev/bhnd/bhndb/bhndb_pci_hwdata.h>

#include "bhndb_bus_if.h"

struct bwn_pci_devcfg;

/** bwn_pci per-instance state. */
struct bwn_pci_softc {
	device_t			 dev;		/**< device */
	device_t			 bhndb_dev;	/**< bhnd bridge device */
	const struct bwn_pci_devcfg	*devcfg;	/**< bwn device config */
};

/* PCI device descriptor */
struct bwn_pci_device {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
};

/* Supported device table */
struct bwn_pci_devcfg {
	const devclass_t		*bridge_cls;
	const struct bhndb_hwcfg	*bridge_hwcfg;
	const struct bhndb_hw		*bridge_hwtable;
	const struct bwn_pci_device	*devices;
};

/* SIBA Devices */
static const struct bwn_pci_device siba_devices[] = {
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4301,		"Broadcom BCM4301 802.11b Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4306,		"Broadcom BCM4301 802.11b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4307,		"Broadcom BCM4307 802.11b Wireless", },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4311_D11G,		"Broadcom BCM4311 802.11b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4311_D11DUAL,	"Broadcom BCM4311 802.11a/b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4328_D11G,		"Broadcom BCM4328/4312 802.11b/g Wireless" },
	
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4318_D11G,		"Broadcom BCM4318 802.11b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4318_D11DUAL,	"Broadcom BCM4318 802.11a/b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4306_D11G,		"Broadcom BCM4306 802.11b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4306_D11A,		"Broadcom BCM4306 802.11a Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4306_D11DUAL,	"Broadcom BCM4309 802.11a/b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4306_D11G_ID2,	"Broadcom BCM4306 802.11b/g Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4321_D11N,		"Broadcom BCM4321 802.11a/b/g/n Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4321_D11N2G,	"Broadcom BCM4329 802.11b/g/n" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4322_D11N,		"Broadcom BCM4322 802.11a/b/g/n Wireless" },
	{ 0, 0, NULL }
};

/** BCMA Devices */
static const struct bwn_pci_device bcma_devices[] = {
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N,		"Broadcom BCM4331 802.11a/b/g/n Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N2G,	"Broadcom BCM4331 802.11b/g/n 2.4GHz Wireless" },
	{ PCI_VENDOR_BROADCOM,	PCI_DEVID_BCM4331_D11N5G,	"Broadcom BCM4331 802.11a/b/g/n 5GHz Wireless" },
	{ 0, 0, NULL }
};

/** Device configuration table */
static const struct bwn_pci_devcfg bwn_pci_devcfgs[] = {
	/* SIBA devices */
	{
		.bridge_cls	= NULL, /* TODO &sibab_devclass */
		.bridge_hwcfg	= &bhndb_pci_siba_generic_hwcfg,
		.bridge_hwtable	= bhndb_pci_generic_hw_table,
		.devices	= siba_devices
	},
	/* BCMA devices */
	{
		.bridge_cls	= &bcmab_devclass,
		.bridge_hwcfg	= &bhndb_pci_bcma_generic_hwcfg,
		.bridge_hwtable	= bhndb_pci_generic_hw_table,
		.devices	= bcma_devices
	},
	{ NULL, NULL, NULL }
};

/** Search the device configuration table for an entry matching @p dev. */
static int
bwn_pci_find_devcfg(device_t dev, const struct bwn_pci_devcfg **cfg,
    const struct bwn_pci_device **device)
{
	const struct bwn_pci_devcfg	*dvc;
	const struct bwn_pci_device	*dv;

	for (dvc = bwn_pci_devcfgs; dvc->devices != NULL; dvc++) {
		for (dv = dvc->devices; dv->device != 0; dv++) {
			if (pci_get_vendor(dev) == dv->vendor &&
			    pci_get_device(dev) == dv->device)
			{
				if (cfg != NULL)
					*cfg = dvc;
				
				if (device != NULL)
					*device = dv;
				
				return (0);
			}
		}
	}

	return (ENOENT);
}

static int
bwn_pci_probe(device_t dev)
{
	const struct bwn_pci_device	*ident;

	if (bwn_pci_find_devcfg(dev, NULL, &ident))
		return (ENXIO);

	device_set_desc(dev, ident->desc);
	return (BUS_PROBE_DEFAULT);
}

static int
bwn_pci_attach(device_t dev)
{
	struct bwn_pci_softc		*sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Find our hardware config */
	if (bwn_pci_find_devcfg(dev, &sc->devcfg, NULL))
		return (ENXIO);

	/* Attach bridge device */
	if (bhndb_attach_bridge(dev, *sc->devcfg->bridge_cls, &sc->bhndb_dev,
	    -1))
	{
		return (ENXIO);
	}

	/* Let the generic implementation probe all added children. */
	return (bus_generic_attach(dev));
}

static int
bwn_pci_detach(device_t dev)
{
	return (bus_generic_detach(dev));
}

static void
bwn_pci_probe_nomatch(device_t dev, device_t child)
{
	const char *name;

	name = device_get_name(child);
	if (name == NULL)
		name = "unknown device";

	device_printf(dev, "<%s> (no driver attached)\n", name);
}

static const struct bhndb_hwcfg *
bwn_pci_get_generic_hwcfg(device_t dev, device_t child)
{
	struct bwn_pci_softc *sc = device_get_softc(dev);
	return (sc->devcfg->bridge_hwcfg);
}

static const struct bhndb_hw *
bwn_pci_get_bhndb_hwtable(device_t dev, device_t child)
{
	struct bwn_pci_softc *sc = device_get_softc(dev);
	return (sc->devcfg->bridge_hwtable);
}

static device_method_t bwn_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bwn_pci_probe),
	DEVMETHOD(device_attach,		bwn_pci_attach),
	DEVMETHOD(device_detach,		bwn_pci_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bus_generic_suspend),
	DEVMETHOD(device_resume,		bus_generic_resume),

	/* Bus interface */
	DEVMETHOD(bus_probe_nomatch,		bwn_pci_probe_nomatch),

	/* BHNDB_BUS Interface */
	DEVMETHOD(bhndb_bus_get_generic_hwcfg,	bwn_pci_get_generic_hwcfg),
	DEVMETHOD(bhndb_bus_get_hardware_table,	bwn_pci_get_bhndb_hwtable),

	DEVMETHOD_END
};

static devclass_t bwn_pci_devclass;

DEFINE_CLASS_0(bwn_pci, bwn_pci_driver, bwn_pci_methods, sizeof(struct bwn_pci_softc));

DRIVER_MODULE(bwn_bcmab, bwn_pci, bcmab_pci_driver, bcmab_devclass, NULL, NULL);
DRIVER_MODULE(bwn_pci, pci, bwn_pci_driver, bwn_pci_devclass, NULL, NULL);

MODULE_DEPEND(bwn_pci, bhndb_pci, 1, 1, 1);
MODULE_DEPEND(bwn_pci, bcmab_pci, 1, 1, 1);

//MODULE_DEPEND(bwn_pci, sibab_pci, 1, 1, 1);
