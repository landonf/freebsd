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

#include "opt_bwn.h"
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
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

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_phy.h>
#include <net80211/ieee80211_ratectl.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_ids.h>

#include "if_bwnvar.h"

static const struct bwn_device {
	uint16_t	 vendor;
	uint16_t	 device;
} bwn_devices[] = {
	{ BHND_MFGID_BCM,	BHND_COREID_D11 },
	{ BHND_MFGID_INVALID,	BHND_COREID_INVALID }
};

static int
bwn_bhnd_probe(device_t dev)
{
	const struct bwn_device	*id;

	for (id = bwn_devices; id->device != BHND_COREID_INVALID; id++)
	{
		if (bhnd_get_vendor(dev) == id->vendor &&
		    bhnd_get_device(dev) == id->device)
		{
			device_set_desc(dev, bhnd_get_device_name(dev));
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
bwn_bhnd_attach(device_t dev)
{
	// TODO
	return (bwn_attach(dev));
}

static int
bwn_bhnd_detach(device_t dev)
{
	// TODO
	return (bwn_detach(dev));
}

static device_method_t bwn_bhnd_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bwn_bhnd_probe),
	DEVMETHOD(device_attach,	bwn_bhnd_attach),
	DEVMETHOD(device_detach,	bwn_bhnd_detach),
	DEVMETHOD_END
};

static devclass_t bwn_devclass;

DEFINE_CLASS_1(bwn, bwn_bhnd_driver, bwn_bhnd_methods, sizeof(struct bwn_softc),
    bwn_driver);

DRIVER_MODULE(bwn_bhnd, bhnd, bwn_bhnd_driver, bwn_devclass, 0, 0);
MODULE_DEPEND(bwn_bhnd, bhnd, 1, 1, 1);
MODULE_VERSION(bwn_bhnd, 1);
