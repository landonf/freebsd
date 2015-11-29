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

#include <sys/types.h> 

#include "bhndvar.h"

/* BHND core device description table. */
static const struct bhnd_core_desc {
	uint16_t	 vendor;
	uint16_t	 device;
	bhnd_devclass_t	 class;
	const char	*desc;
} bhnd_core_descs[] = {
	#define	BHND_CDESC(_mfg, _cid, _cls, _desc)		\
	    { BHND_MFGID_ ## _mfg, BHND_COREID_ ## _cid,	\
		BHND_DEVCLASS_ ## _cls, _desc }

	BHND_CDESC(BCM, CC,		CC,		"ChipCommon I/O Controller"),
	BHND_CDESC(BCM, ILINE20,	OTHER,		"iLine20 HPNA"),
	BHND_CDESC(BCM, SRAM,		MEM,		"SRAM"),
	BHND_CDESC(BCM, SDRAM,		MEM,		"SDRAM"),
	BHND_CDESC(BCM, PCI,		PCI,		"PCI Bridge"),
	BHND_CDESC(BCM, MIPS,		CPU,		"MIPS Core"),
	BHND_CDESC(BCM, ENET,		ENET_MAC,	"Fast Ethernet MAC"),
	BHND_CDESC(BCM, CODEC,		OTHER,		"V.90 Modem Codec"),
	BHND_CDESC(BCM, USB,		OTHER,		"USB 1.1 Device/Host Controller"),
	BHND_CDESC(BCM, ADSL,		OTHER,		"ADSL Core"),
	BHND_CDESC(BCM, ILINE100,	OTHER,		"iLine100 HPNA"),
	BHND_CDESC(BCM, IPSEC,		OTHER,		"IPsec Accelerator"),
	BHND_CDESC(BCM, UTOPIA,		OTHER,		"UTOPIA ATM Core"),
	BHND_CDESC(BCM, PCMCIA,		OTHER,		"PCMCIA Bridge"),
	BHND_CDESC(BCM, SOCRAM,		MEM,		"Internal Memory"),
	BHND_CDESC(BCM, MEMC,		MEMC,		"MEMC SDRAM Controller"),
	BHND_CDESC(BCM, OFDM,		OTHER,		"OFDM PHY"),
	BHND_CDESC(BCM, EXTIF,		OTHER,		"External Interface"),
	BHND_CDESC(BCM, D11,		WLAN,		"802.11 MAC/PHY/Radio"),
	BHND_CDESC(BCM, APHY,		WLAN_PHY,	"802.11a PHY"),
	BHND_CDESC(BCM, BPHY,		WLAN_PHY,	"802.11b PHY"),
	BHND_CDESC(BCM, GPHY,		WLAN_PHY,	"802.11g PHY"),
	BHND_CDESC(BCM, MIPS33,		CPU,		"MIPS 3302 Core"),
	BHND_CDESC(BCM, USB11H,		OTHER,		"USB 1.1 Host Controller"),
	BHND_CDESC(BCM, USB11D,		OTHER,		"USB 1.1 Device Core"),
	BHND_CDESC(BCM, USB20H,		OTHER,		"USB 2.0 Host Controller"),
	BHND_CDESC(BCM, USB20D,		OTHER,		"USB 2.0 Device Core"),
	BHND_CDESC(BCM, SDIOH,		OTHER,		"SDIO Host Controller"),
	BHND_CDESC(BCM, ROBO,		OTHER,		"RoboSwitch"),
	BHND_CDESC(BCM, ATA100,		OTHER,		"Parallel ATA Controller"),
	BHND_CDESC(BCM, SATAXOR,	OTHER,		"SATA DMA/XOR Controller"),
	BHND_CDESC(BCM, GIGETH,		ENET_MAC,	"Gigabit Ethernet MAC"),
	BHND_CDESC(BCM, PCIE,		PCIE,		"PCIe Bridge"),
	BHND_CDESC(BCM, NPHY,		WLAN_PHY,	"802.11n 2x2 PHY"),
	BHND_CDESC(BCM, SRAMC,		MEMC,		"SRAM Controller"),
	BHND_CDESC(BCM, MINIMAC,	OTHER,		"MINI MAC/PHY"),
	BHND_CDESC(BCM, ARM11,		CPU,		"ARM1176 Core"),
	BHND_CDESC(BCM, ARM7S,		CPU,		"ARM7TDMI-S Core"),
	BHND_CDESC(BCM, LPPHY,		WLAN_PHY,	"802.11a/b/g PHY"),
	BHND_CDESC(BCM, PMU,		OTHER,		"PMU"),
	BHND_CDESC(BCM, SSNPHY,		WLAN_PHY,	"802.11n Single-Stream PHY"),
	BHND_CDESC(BCM, SDIOD,		OTHER,		"SDIO Device Core"),
	BHND_CDESC(BCM, ARMCM3,		CPU,		"ARM Cortex-M3 Core"),
	BHND_CDESC(BCM, HTPHY,		WLAN_PHY,	"802.11n 4x4 PHY"),
	BHND_CDESC(BCM, MIPS74K,	CPU,		"MIPS74k Core"),
	BHND_CDESC(BCM, GMAC,		ENET_MAC,	"Gigabit MAC core"),
	BHND_CDESC(BCM, DMEMC,		MEMC,		"DDR1/2 Memory Controller"),
	BHND_CDESC(BCM, PCIERC,		OTHER,		"PCIe Root Complex"),
	BHND_CDESC(BCM, OCP,		SOCB,		"OCP to OCP Bridge"),
	BHND_CDESC(BCM, SC,		OTHER,		"Shared Common Core"),
	BHND_CDESC(BCM, AHB,		SOCB,		"OCP to AHB Bridge"),
	BHND_CDESC(BCM, SPIH,		OTHER,		"SPI Host Controller"),
	BHND_CDESC(BCM, I2S,		OTHER,		"I2S Digital Audio Interface"),
	BHND_CDESC(BCM, DMEMS,		MEMC,		"SDR/DDR1 Memory Controller"),
	BHND_CDESC(BCM, UBUS_SHIM,	OTHER,		"BCM6362/UBUS WLAN SHIM"),
	BHND_CDESC(BCM,	PCIE2,		PCIE,		"PCIe Bridge (Gen2)"),

	BHND_CDESC(ARM, APB_BRIDGE,	SOCB,		"BP135 AMBA3 AXI to APB Bridge"),
	BHND_CDESC(ARM, PL301,		SOCI,		"PL301 AMBA3 Interconnect"),
	BHND_CDESC(ARM, EROM,		EROM,		"PL366 Device Enumeration ROM"),
	BHND_CDESC(ARM, OOB_ROUTER,	OTHER,		"PL367 OOB Interrupt Router"),
	BHND_CDESC(ARM, AXI_UNMAPPED,	OTHER,		"Unmapped Address Ranges"),
#undef	BHND_CDESC

	/* Derived from inspection of the BCM4331 cores that provide PrimeCell
	 * IDs. Due to lack of documentation, the surmised device name/purpose
	 * provided here may be incorrect. */
	{ BHND_MFGID_ARM,	BHND_PRIMEID_EROM,	BHND_DEVCLASS_OTHER,
	    "PL364 Device Enumeration ROM" },
	{ BHND_MFGID_ARM,	BHND_PRIMEID_SWRAP,	BHND_DEVCLASS_OTHER,
	    "PL368 Device Management Interface" },
	{ BHND_MFGID_ARM,	BHND_PRIMEID_MWRAP,	BHND_DEVCLASS_OTHER,
	    "PL369 Device Management Interface" },

	{ 0, 0, 0, NULL }
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
	case BHND_MFGID_ARM:
		return "ARM";
	case BHND_MFGID_BCM:
		return "Broadcom";
	case BHND_MFGID_MIPS:
		return "MIPS";
	default:
		return "unknown";
	}
}

static const struct bhnd_core_desc *
bhnd_find_core_desc(uint16_t vendor, uint16_t device)
{
	for (u_int i = 0; bhnd_core_descs[i].desc != NULL; i++) {
		if (bhnd_core_descs[i].vendor != vendor)
			continue;
		
		if (bhnd_core_descs[i].device != device)
			continue;
		
		return (&bhnd_core_descs[i]);
	}
	
	return (NULL);
}

/**
 * Return a human-readable name for a BHND core.
 * 
 * @param vendor The core designer's JEDEC-106 Manufacturer ID
 * @param device The core identifier.
 */
const char *
bhnd_core_name(uint16_t vendor, uint16_t device)
{
	const struct bhnd_core_desc *desc;
	
	if ((desc = bhnd_find_core_desc(vendor, device)) == NULL)
		return ("unknown");

	return desc->desc;
}

/**
 * Return the device class for a BHND core.
 * 
 * @param vendor The core designer's JEDEC-106 Manufacturer ID
 * @param device The core identifier.
 */
bhnd_devclass_t
bhnd_core_class(uint16_t vendor, uint16_t device)
{
	const struct bhnd_core_desc *desc;
	
	if ((desc = bhnd_find_core_desc(vendor, device)) == NULL)
		return (BHND_DEVCLASS_OTHER);

	return desc->class;
}


/**
 * Initialize a core info record with data from from a bhnd-attached @p dev.
 * 
 * @param dev A bhnd device.
 * @param core The record to be initialized.
 */
struct bhnd_core_info
bhnd_get_core_info(device_t dev) {
	return (struct bhnd_core_info) {
		.vendor		= bhnd_get_vendor(dev),
		.device		= bhnd_get_device(dev),
		.hwrev		= bhnd_get_revid(dev),
		.core_id	= bhnd_get_core_index(dev),
		.unit		= bhnd_get_core_unit(dev)
	};
}

/**
 * Find a @p class child device with @p unit on @p dev.
 * 
 * @param parent The bhnd-compatible bus to be searched.
 * @param class The device class to match on.
 * @param unit The device unit number; specify -1 to return the first match
 * regardless of unit number.
 * 
 * @retval device_t if a matching child device is found.
 * @retval NULL if no matching child device is found.
 */
device_t
bhnd_find_child(device_t dev, bhnd_devclass_t class, int unit)
{
	struct bhnd_core_match md = {
		.vendor = BHND_MFGID_INVALID,
		.device = BHND_COREID_INVALID,
		.hwrev.start = BHND_HWREV_INVALID,
		.hwrev.end = BHND_HWREV_INVALID,
		.class = class,
		.unit = unit
	};

	return bhnd_match_child(dev, &md);
}

/**
 * Find the first child device on @p dev that matches @p desc.
 * 
 * @param parent The bhnd-compatible bus to be searched.
 * @param desc A match descriptor.
 * 
 * @retval device_t if a matching child device is found.
 * @retval NULL if no matching child device is found.
 */
device_t
bhnd_match_child(device_t dev, const struct bhnd_core_match *desc)
{
	device_t	*devlistp;
	device_t	 match;
	int		 devcnt;
	int		 error;

	error = device_get_children(dev, &devlistp, &devcnt);
	if (error != 0)
		return (NULL);

	match = NULL;
	for (int i = 0; i < devcnt; i++) {
		device_t dev = devlistp[i];
		if (bhnd_device_matches(dev, desc)) {
			match = dev;
			goto done;
		}
	}

done:
	free(devlistp, M_TEMP);
	return match;
}

/**
 * Find the first core in @p cores that matches @p desc.
 * 
 * @param cores The table to search.
 * @param num_cores The length of @p cores.
 * @param desc A match descriptor.
 * 
 * @retval bhnd_core_info if a matching core is found.
 * @retval NULL if no matching core is found.
 */
const struct bhnd_core_info *
bhnd_match_core(const struct bhnd_core_info *cores, u_int num_cores,
    const struct bhnd_core_match *desc)
{
	for (u_int i = 0; i < num_cores; i++) {
		if (bhnd_core_matches(&cores[i], desc))
			return &cores[i];
	}

	return (NULL);
}


/**
 * Find the first core in @p cores with the given @p class.
 * 
 * @param cores The table to search.
 * @param num_cores The length of @p cores.
 * @param desc A match descriptor.
 * 
 * @retval bhnd_core_info if a matching core is found.
 * @retval NULL if no matching core is found.
 */
const struct bhnd_core_info *
bhnd_find_core(const struct bhnd_core_info *cores, u_int num_cores,
    bhnd_devclass_t class)
{
	struct bhnd_core_match md = {
		.vendor = BHND_MFGID_INVALID,
		.device = BHND_COREID_INVALID,
		.hwrev.start = BHND_HWREV_INVALID,
		.hwrev.end = BHND_HWREV_INVALID,
		.class = class,
		.unit = -1
	};

	return bhnd_match_core(cores, num_cores, &md);
}

/**
 * Return true if the @p core matches @p desc.
 * 
 * @param core A bhnd core descriptor.
 * @param desc A match descriptor to compare against @p core.
 * 
 * @retval true if @p core matches @p match
 * @retval false if @p core does not match @p match.
 */
bool
bhnd_core_matches(const struct bhnd_core_info *core,
    const struct bhnd_core_match *desc)
{
	if (desc->vendor != BHND_MFGID_INVALID &&
	    desc->vendor != core->vendor)
		return false;

	if (desc->device != BHND_COREID_INVALID &&
	    desc->device != core->device)
		return false;

	if (desc->unit != -1 && desc->unit != core->unit)
		return false;

	if (desc->hwrev.start != BHND_HWREV_INVALID &&
	    desc->hwrev.start > core->hwrev)
		return false;
		
	if (desc->hwrev.end != BHND_HWREV_INVALID &&
	    desc->hwrev.end < core->hwrev)
		return false;

	if (desc->class != BHND_DEVCLASS_INVALID &&
	    desc->class != bhnd_core_class(core->vendor, core->device))
		return false;

	return true;
}

/**
 * Return true if the @p dev matches @p desc.
 * 
 * @param dev A bhnd device.
 * @param desc A match descriptor to compare against @p dev.
 * 
 * @retval true if @p dev matches @p match
 * @retval false if @p dev does not match @p match.
 */
bool
bhnd_device_matches(device_t dev, const struct bhnd_core_match *desc)
{
	struct bhnd_core_info ci = {
		.vendor = bhnd_get_vendor(dev),
		.device = bhnd_get_device(dev),
		.unit = bhnd_get_core_unit(dev),
		.hwrev = bhnd_get_revid(dev)
	};

	return bhnd_core_matches(&ci, desc);
}