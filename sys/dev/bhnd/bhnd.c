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
 * Broadcom Home Networking Division (HND) Bus Driver.
 * 
 * The Broadcom HND family of devices consists of both SoCs and host-connected
 * networking chipsets containing a common family of Broadcom IP cores,
 * including an integrated MIPS and/or ARM cores.
 * 
 * HND devices expose a nearly identical interface whether accessible over a 
 * native SoC interconnect, or when connected via a host interface such as 
 * PCIe. As a result, the majority of hardware support code should be re-usable 
 * across host drivers for HND networking chipsets, as well as FreeBSD support 
 * for Broadcom MIPS/ARM HND SoCs.
 * 
 * Earlier HND models used the siba(4) on-chip interconnect, while later models
 * use bcma(4); the programming model is almost entirely independent
 * of the actual underlying interconect.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include "bhnd_device_ids.h"
#include "bhndreg.h"
#include "bhndvar.h"

MALLOC_DEFINE(M_BHND, "bhnd", "BHND-compliant bus data structures");

/**
 * Generic probe configuration.
 */
struct bhnd_probecfg bhnd_generic_probecfg_table[] = {
	/* ChipCommon devices should always be probed before any other cores. */
	{ JEDEC_MFGID_BCM,	BHND_COREID_CC,	BHND_PROBE_ORDER_FIRST,	NULL },
	BHND_PROBECFG_TABLE_END
};

/* BHND core device description table. */
static const struct bhnd_core_desc {
	uint16_t vendor;
	uint16_t device;
	const char *desc;
} bhnd_core_descs[] = {
	{ JEDEC_MFGID_BCM,	BHND_COREID_CC,		"ChipCommon I/O Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ILINE20,	"iLine20 HPNA" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SRAM,	"SRAM" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SDRAM,	"SDRAM" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCI,	"PCI Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MIPS,	"MIPS Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ENET,	"Fast Ethernet MAC" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_CODEC,	"V.90 Modem Codec" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB,	"USB 1.1 Device/Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ADSL,	"ADSL Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ILINE100,	"iLine100 HPNA" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_IPSEC,	"IPsec Accelerator" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_UTOPIA,	"UTOPIA ATM Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCMCIA,	"PCMCIA Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SOCRAM,	"Internal Memory" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MEMC,	"MEMC SDRAM Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_OFDM,	"OFDM PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_EXTIF,	"External Interface" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_D11,	"802.11 MAC" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_APHY,	"802.11a PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_BPHY,	"802.11b PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_GPHY,	"802.11g PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MIPS33,	"MIPS 3302 Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB11H,	"USB 1.1 Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB11D,	"USB 1.1 Device Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB20H,	"USB 2.0 Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB20D,	"USB 2.0 Device Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SDIOH,	"SDIO Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ROBO,	"RoboSwitch" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ATA100,	"Parallel ATA Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SATAXOR,	"SATA DMA/XOR Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_GIGETH,	"Gigabit Ethernet MAC" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCIE,	"PCIe Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_NPHY,	"802.11n 2x2 PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SRAMC,	"SRAM Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MINIMAC,	"MINI MAC/PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ARM11,	"ARM1176 Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ARM7S,	"ARM7TDMI-S Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_LPPHY,	"802.11a/b/g PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PMU,	"PMU" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SSNPHY,	"802.11n Single-Stream PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SDIOD,	"SDIO Device Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ARMCM3,	"ARM Cortex-M3 Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_HTPHY,	"802.11n 4x4 PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MIPS74K,	"MIPS74k Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_GMAC,	"Gigabit MAC core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_DMEMC,	"DDR1/2 Memory Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCIERC,	"PCIe Root Complex" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_OCP,	"OCP to OCP Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SC,		"Shared Common Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_AHB,	"OCP to AHB Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SPIH,	"SPI Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_I2S,	"I2S Digital Audio Interface" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_DMEMS,	"SDR/DDR1 Memory Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_UBUS_SHIM,	"BCM6362/UBUS WLAN SHIM" },

	{ JEDEC_MFGID_ARM,	BHND_COREID_APB_BRIDGE,	"BP135 AMBA3 AXI to APB Bridge" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_PL301,	"PL301 AMBA3 Interconnect" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_EROM,	"PL366 Device Enumeration ROM" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_OOB_ROUTER,	"PL367 OOB Interrupt Router" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_AXI_UNMAPPED,
		"Unmapped Address Ranges" },
	
	/* Derived from inspection of the BCM4331 cores that provide PrimeCell
	 * IDs. Due to lack of documentation, the surmised device name/purpose
	 * provided here may be incorrect. */
	{ JEDEC_MFGID_ARM,	BHND_PRIMEID_EROM,	"PL364 Device Enumeration ROM" },
	{ JEDEC_MFGID_ARM,	BHND_PRIMEID_SWRAP,	"PL368 Device Management Interface" },
	{ JEDEC_MFGID_ARM,	BHND_PRIMEID_MWRAP,	"PL369 Device Management Interface" },
	
	{ 0,			0,			NULL }
};

/**
 * Return the name for a given JEP106 manufacturer ID.
 * 
 * @param vendor A JEP106 Manufacturer ID, including the non-standard ARM 4-bit
 * JEP106 continuation code.
 */
const char *
bhnd_vendor_name(uint16_t vendor)
{
	switch (vendor) {
	case JEDEC_MFGID_ARM:
		return "ARM";
		break;
	case JEDEC_MFGID_BCM:
		return "Broadcom";
		break;
	case JEDEC_MFGID_MIPS:
		return "MIPS";
	default:
		return "unknown";
	}
}

/**
 * Return a human-readable name for a BHND core.
 * 
 * @param vendor The core designer's JEDEC-106 Manufacturer ID
 * @param device The core identifier.
 */
const char *
bhnd_core_name(uint16_t vendor, uint16_t device) {
	for (u_int i = 0; bhnd_core_descs[i].desc != NULL; i++) {
		if (bhnd_core_descs[i].vendor != vendor)
			continue;
		
		if (bhnd_core_descs[i].device != device)
			continue;
		
		return bhnd_core_descs[i].desc;
	}
	
	return "unknown";
}

/**
 * Return the first matching probe configuration from the given table, or NULL
 * if no match is found.
 * 
 * @param table The table to search.
 * @param vendor The device's JEP106 manufacturer identifier.
 * @param device The device identifier.
 * @param revid The device's revision identifier.
 */
struct bhnd_probecfg *
bhnd_find_probecfg(struct bhnd_probecfg table[], uint16_t vendor,
    uint16_t device)
{
	struct bhnd_probecfg *cfg;

	for (cfg = table; cfg->device != BHND_COREID_NODEV; cfg++) {
		if (vendor == cfg->vendor && device == cfg->device)
			return cfg;
	}
	
	return (NULL);
}

/**
 * Helper function for implementing BHND_ALLOC_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_ALLOC_RESOURCE().
 */
struct bhnd_resource *
bhnd_generic_alloc_bhnd_resource(device_t dev, device_t child, int type,
	int *rid, u_long start, u_long end, u_long count, u_int flags)
{
	struct bhnd_resource *r;
	
	if (device_get_parent(child) != dev)
		return (BHND_ALLOC_RESOURCE(device_get_parent(dev), child,
		    type, rid, start, end, count, flags));
	
	/* Allocate an empty wrapper for the real bus-allocated resource */
	r = malloc(sizeof(struct bhnd_resource), M_BHND, M_WAITOK);
	if (r == NULL)
		return NULL;
	
	/* Allocate the bus resource, marking it as 'direct' (not requiring
	 * any bus window remapping to perform I/O) */
	r->_direct = true;
	r->_res = bus_alloc_resource(child, type, rid, start, end,
		count, flags);

	if (r->_res == NULL) {
		free(r, M_BHND);
		return NULL;
	}

	return (r);
}

/**
 * Helper function for implementing BHND_RELEASE_RESOURCE().
 * 
 * This simple implementation delegates handling of the backing resource
 * to BUS_RELEASE_RESOURCE().
 */
int
bhnd_generic_release_bhnd_resource(device_t dev, device_t child, int type,
    int rid, struct bhnd_resource *r)
{
	int error;
	
	if (device_get_parent(child) != dev)
		return (BHND_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	error = bus_release_resource(child, type, rid, r->_res);
	if (error)
		return (error);

	free(r, M_BHND);
	return (0);
}

/**
 * Helper function for implementing BHND_ACTIVATE_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_ACTIVATE_RESOURCE().
 */
int
bhnd_generic_activate_bhnd_resource(device_t dev, device_t child, int type,
	int rid, struct bhnd_resource *r)
{
	if (device_get_parent(child) != dev)
		return (BHND_ACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	return (bus_activate_resource(child, type, rid, r->_res));
};

/**
 * Helper function for implementing BHND_DEACTIVATE_RESOURCE().
 * 
 * This simple implementation delegates allocation of the backing resource
 * to BUS_DEACTIVATE_RESOURCE().
 */
int
bhnd_generic_deactivate_bhnd_resource(device_t dev, device_t child, int type,
	int rid, struct bhnd_resource *r)
{
	if (device_get_parent(child) != dev)
		return (BHND_DEACTIVATE_RESOURCE(device_get_parent(dev), child,
		    type, rid, r));

	return (bus_deactivate_resource(child, type, rid, r->_res));
};
