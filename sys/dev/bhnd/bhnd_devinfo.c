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

#include <sys/types.h>

#include "bhnd_devinfo.h"
#include "bhndvar.h"

/**
 * Generic probe configuration.
 */
struct bhnd_probecfg bhnd_generic_probecfg_table[] = {
	/* ChipCommon devices should always be probed before any other cores. */
	{ BHND_DEVCLASS_CC,	BHND_PROBE_ORDER_FIRST,	NULL },
	BHND_PROBECFG_TABLE_END
};

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

	BHND_CDESC(BCM, CC,		CC,	"ChipCommon I/O Controller"),
	BHND_CDESC(BCM, ILINE20,	OTHER,	"iLine20 HPNA"),
	BHND_CDESC(BCM, SRAM,		MEM,	"SRAM"),
	BHND_CDESC(BCM, SDRAM,		MEM,	"SDRAM"),
	BHND_CDESC(BCM, PCI,		PCI,	"PCI Bridge"),
	BHND_CDESC(BCM, MIPS,		CPU,	"MIPS Core"),
	BHND_CDESC(BCM, ENET,		MAC,	"Fast Ethernet MAC"),
	BHND_CDESC(BCM, CODEC,		OTHER,	"V.90 Modem Codec"),
	BHND_CDESC(BCM, USB,		OTHER,	"USB 1.1 Device/Host Controller"),
	BHND_CDESC(BCM, ADSL,		OTHER,	"ADSL Core"),
	BHND_CDESC(BCM, ILINE100,	OTHER,	"iLine100 HPNA"),
	BHND_CDESC(BCM, IPSEC,		OTHER,	"IPsec Accelerator"),
	BHND_CDESC(BCM, UTOPIA,		OTHER,	"UTOPIA ATM Core"),
	BHND_CDESC(BCM, PCMCIA,		OTHER,	"PCMCIA Bridge"),
	BHND_CDESC(BCM, SOCRAM,		MEM,	"Internal Memory"),
	BHND_CDESC(BCM, MEMC,		MEMC,	"MEMC SDRAM Controller"),
	BHND_CDESC(BCM, OFDM,		OTHER,	"OFDM PHY"),
	BHND_CDESC(BCM, EXTIF,		OTHER,	"External Interface"),
	BHND_CDESC(BCM, D11,		MAC_W,	"802.11 MAC"),
	BHND_CDESC(BCM, APHY,		PHY_W,	"802.11a PHY"),
	BHND_CDESC(BCM, BPHY,		PHY_W,	"802.11b PHY"),
	BHND_CDESC(BCM, GPHY,		PHY_W,	"802.11g PHY"),
	BHND_CDESC(BCM, MIPS33,		CPU,	"MIPS 3302 Core"),
	BHND_CDESC(BCM, USB11H,		OTHER,	"USB 1.1 Host Controller"),
	BHND_CDESC(BCM, USB11D,		OTHER,	"USB 1.1 Device Core"),
	BHND_CDESC(BCM, USB20H,		OTHER,	"USB 2.0 Host Controller"),
	BHND_CDESC(BCM, USB20D,		OTHER,	"USB 2.0 Device Core"),
	BHND_CDESC(BCM, SDIOH,		OTHER,	"SDIO Host Controller"),
	BHND_CDESC(BCM, ROBO,		OTHER,	"RoboSwitch"),
	BHND_CDESC(BCM, ATA100,		OTHER,	"Parallel ATA Controller"),
	BHND_CDESC(BCM, SATAXOR,	OTHER,	"SATA DMA/XOR Controller"),
	BHND_CDESC(BCM, GIGETH,		MAC,	"Gigabit Ethernet MAC"),
	BHND_CDESC(BCM, PCIE,		PCI,	"PCIe Bridge"),
	BHND_CDESC(BCM, NPHY,		PHY_W,	"802.11n 2x2 PHY"),
	BHND_CDESC(BCM, SRAMC,		MEMC,	"SRAM Controller"),
	BHND_CDESC(BCM, MINIMAC,	MPHY_W,	"MINI MAC/PHY"),
	BHND_CDESC(BCM, ARM11,		CPU,	"ARM1176 Core"),
	BHND_CDESC(BCM, ARM7S,		CPU,	"ARM7TDMI-S Core"),
	BHND_CDESC(BCM, LPPHY,		PHY_W,	"802.11a/b/g PHY"),
	BHND_CDESC(BCM, PMU,		OTHER,	"PMU"),
	BHND_CDESC(BCM, SSNPHY,		PHY_W,	"802.11n Single-Stream PHY"),
	BHND_CDESC(BCM, SDIOD,		OTHER,	"SDIO Device Core"),
	BHND_CDESC(BCM, ARMCM3,		CPU,	"ARM Cortex-M3 Core"),
	BHND_CDESC(BCM, HTPHY,		PHY_W,	"802.11n 4x4 PHY"),
	BHND_CDESC(BCM, MIPS74K,	CPU,	"MIPS74k Core"),
	BHND_CDESC(BCM, GMAC,		MAC,	"Gigabit MAC core"),
	BHND_CDESC(BCM, DMEMC,		MEMC,	"DDR1/2 Memory Controller"),
	BHND_CDESC(BCM, PCIERC,		OTHER,	"PCIe Root Complex"),
	BHND_CDESC(BCM, OCP,		SOCB,	"OCP to OCP Bridge"),
	BHND_CDESC(BCM, SC,		OTHER,	"Shared Common Core"),
	BHND_CDESC(BCM, AHB,		SOCB,	"OCP to AHB Bridge"),
	BHND_CDESC(BCM, SPIH,		OTHER,	"SPI Host Controller"),
	BHND_CDESC(BCM, I2S,		OTHER,	"I2S Digital Audio Interface"),
	BHND_CDESC(BCM, DMEMS,		MEMC,	"SDR/DDR1 Memory Controller"),
	BHND_CDESC(BCM, UBUS_SHIM,	OTHER,	"BCM6362/UBUS WLAN SHIM"),
	BHND_CDESC(BCM,	PCIE2,		PCI,	"PCIe Bridge (Gen2)"),

	BHND_CDESC(ARM, APB_BRIDGE,	SOCB,	"BP135 AMBA3 AXI to APB Bridge"),
	BHND_CDESC(ARM, PL301,		SOCI,	"PL301 AMBA3 Interconnect"),
	BHND_CDESC(ARM, EROM,		OTHER,	"PL366 Device Enumeration ROM"),
	BHND_CDESC(ARM, OOB_ROUTER,	OTHER,	"PL367 OOB Interrupt Router"),
	BHND_CDESC(ARM, AXI_UNMAPPED,	OTHER,	"Unmapped Address Ranges"),
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
	struct bhnd_probecfg	*cfg;
	bhnd_devclass_t		 cls;
	
	cls = bhnd_core_class(vendor, device);

	for (cfg = table; cfg->devclass != BHND_DEVCLASS_INVALID; cfg++) {
		if (cls == cfg->devclass)
			return cfg;
	}
	
	return (NULL);
}


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
 * Return true if the @p dev matches @p desc.
 * 
 * @param dev A bhnd bus device.
 * @param desc A match descriptor to compare against @p dev.
 * 
 * @retval true if @p dev matches @p match
 * @retval false if @p dev does not match @p match.
 */
bool
bhnd_device_matches(device_t dev, struct bhnd_core_match *desc)
{
	if (desc->vendor != BHND_MFGID_INVALID &&
	    desc->vendor != bhnd_get_vendor(dev))
		return false;

	if (desc->device != BHND_COREID_INVALID &&
	    desc->device != bhnd_get_device(dev))
		return false;

	if (desc->hwrev.start != BHND_HWREV_INVALID &&
	    desc->hwrev.start > bhnd_get_revid(dev))
		return false;
		
	if (desc->hwrev.end != BHND_HWREV_INVALID &&
	    desc->hwrev.end < bhnd_get_revid(dev))
		return false;

	if (desc->class != BHND_DEVCLASS_INVALID &&
	    desc->class != bhnd_get_class(dev))
		return false;

	return true;
}