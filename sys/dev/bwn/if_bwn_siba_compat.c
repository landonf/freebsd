/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2017 The FreeBSD Foundation
 * All rights reserved.
 * 
 * Portions of this software were developed by Landon Fuller
 * under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_llc.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_phy.h>
#include <net80211/ieee80211_ratectl.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/siba/sibareg.h>

#include <dev/bhnd/cores/chipc/chipc.h>
#include <dev/bhnd/cores/pci/bhnd_pcireg.h>
#include <dev/bhnd/cores/pmu/bhnd_pmu.h>

#include "bhnd_nvram_map.h"

#include "if_bwn_siba_compat.h"

static int
bwn_bhnd_bus_ops_init(device_t dev)
{
	struct bwn_bhnd_ctx	*ctx;
	struct bwn_softc *sc;

	sc = device_get_softc(dev);
	ctx = NULL;

	/* Allocate our context */
	ctx = malloc(sizeof(struct bwn_bhnd_ctx), M_DEVBUF, M_WAITOK|M_ZERO);

	/* Initialize bwn_softc */
	sc->sc_bus_ctx = ctx;
	return (0);
}

static void
bwn_bhnd_bus_ops_fini(device_t dev)
{
	struct bwn_bhnd_ctx	*ctx;
	struct bwn_softc	*sc;

	sc = device_get_softc(dev);
	ctx = sc->sc_bus_ctx;

	free(ctx, M_DEVBUF);
	sc->sc_bus_ctx = NULL;
}

/**
 * Return the PCI bridge root device.
 * 
 * Will panic if a PCI bridge root device is not found.
 */
static device_t
bwn_bhnd_get_pci_dev(device_t dev)
{	device_t bridge_root;

	bridge_root = bhnd_find_bridge_root(dev, devclass_find("pci"));
	if (bridge_root == NULL)
		panic("not a PCI device");

	return (bridge_root);
}

/*
 * siba_get_pci_vendor()
 *
 * Referenced by:
 *   bwn_sprom_bugfixes()
 */
static uint16_t
bhnd_compat_get_pci_vendor(device_t dev)
{
	return (pci_get_vendor(bwn_bhnd_get_pci_dev(dev)));
}

/*
 * siba_get_pci_device()
 *
 * Referenced by:
 *   bwn_attach()
 *   bwn_attach_core()
 *   bwn_nphy_op_prepare_structs()
 *   bwn_sprom_bugfixes()
 */
static uint16_t
bhnd_compat_get_pci_device(device_t dev)
{
	return (pci_get_device(bwn_bhnd_get_pci_dev(dev)));
}

/*
 * siba_get_type()
 *
 * Referenced by:
 *   bwn_core_init()
 *   bwn_dma_attach()
 *   bwn_nphy_op_prepare_structs()
 *   bwn_sprom_bugfixes()
 */
static enum siba_type
bhnd_compat_get_type(device_t dev)
{
	device_t		bus, hostb;
	bhnd_devclass_t		hostb_devclass;

	bus = device_get_parent(dev);
	hostb = bhnd_bus_find_hostb_device(bus);

	if (hostb == NULL)
		return (SIBA_TYPE_SSB);

	hostb_devclass = bhnd_get_class(hostb);
	switch (hostb_devclass) {
	case BHND_DEVCLASS_PCCARD:
		return (SIBA_TYPE_PCMCIA);
	case BHND_DEVCLASS_PCI:
	case BHND_DEVCLASS_PCIE:
		return (SIBA_TYPE_PCI);
	default:
		panic("unsupported hostb devclass: %d\n", hostb_devclass);
	}
}

const struct bwn_bus_ops bwn_bhnd_bus_ops = {
	.init				= bwn_bhnd_bus_ops_init,
	.fini				= bwn_bhnd_bus_ops_fini,
	.get_pci_vendor			= bhnd_compat_get_pci_vendor,
	.get_pci_device			= bhnd_compat_get_pci_device,
	.get_type			= bhnd_compat_get_type,
};
