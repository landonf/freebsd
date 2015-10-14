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
 * networking chipsets containing a common family of Broadcom IP cores, including
 * an integrated MIPS and/or ARM processors.
 * 
 * HND devices expose a nearly identical interface whether accessible over a native
 * SoC interconnect, or when connected via a host interface such as PCIe. As a result,
 * the majority of hardware support code should be re-usable across host drivers for HND
 * networking chipsets, as well as FreeBSD support for Broadcom MIPS/ARM HND SoCs.
 * 
 * Earlier HND models used the siba(4) on-chip interconnect, while later models
 * use bcma(4); the programming model is almost entirely independent
 * of the actual underlying interconect.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include "bhnd.h"
#include "bhnd_device_ids.h"
#include "bhndreg.h"
#include "bhndvar.h"



/* BHND core device description table. */
static const struct bhnd_core_desc {
	uint16_t mfgid;
	uint16_t coreid;
	const char *desc;
} bhnd_core_descs[] = {
	{ JEDEC_MFGID_BCM,	BHND_COREID_CC,			"ChipCommon" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ILINE20,		"iLine20 HPNA" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SRAM,		"SRAM" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SDRAM,		"SDRAM" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCI,		"PCI Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MIPS,		"MIPS Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ENET,		"Fast Ethernet MAC" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_CODEC,		"V.90 Modem Codec" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB,		"USB 1.1 Device/Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ADSL,		"ADSL Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ILINE100,		"iLine100 HPNA" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_IPSEC,		"IPsec Accelerator" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_UTOPIA,		"UTOPIA ATM Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCMCIA,		"PCMCIA Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SOCRAM,		"Internal Memory" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MEMC,		"MEMC SDRAM Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_OFDM,		"OFDM PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_EXTIF,		"External Interface" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_D11,		"802.11 MAC" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_APHY,		"802.11a PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_BPHY,		"802.11b PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_GPHY,		"802.11g PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MIPS33,		"MIPS 3302 Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB11H,		"USB 1.1 Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB11D,		"USB 1.1 Device Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB20H,		"USB 2.0 Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_USB20D,		"USB 2.0 Device Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SDIOH,		"SDIO Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ROBO,		"RoboSwitch" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ATA100,		"Parallel ATA Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SATAXOR,		"SATA DMA/XOR Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_GIGETH,		"Gigabit Ethernet MAC" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCIE,		"PCIe Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_NPHY,		"802.11n 2x2 PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SRAMC,		"SRAM Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MINIMAC,		"MINI MAC/PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ARM11,		"ARM1176 Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ARM7S,		"ARM7TDMI-S Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_LPPHY,		"802.11a/b/g PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PMU,		"PMU" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SSNPHY,		"802.11n Single-Stream PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SDIOD,		"SDIO Device Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_ARMCM3,		"ARM Cortex-M3 Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_HTPHY,		"802.11n 4x4 PHY" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_MIPS74K,		"MIPS74k Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_GMAC,		"Gigabit MAC core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_DMEMC,		"DDR1/2 Memory Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_PCIERC,		"PCIe Root Complex" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_OCP,		"OCP2OCP Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SC,			"Shared Common Core" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_AHB,		"OCP2AHB Bridge" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_SPIH,		"SPI Host Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_I2S,		"I2S Digital Audio Interface" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_DMEMS,		"SDR/DDR1 Memory Controller" },
	{ JEDEC_MFGID_BCM,	BHND_COREID_DEF_SHIM_COMP,	"BCM6362/UBUS WLAN SHIM" },
	
	{ JEDEC_MFGID_ARM,	BHND_COREID_APB_BRIDGE,		"AXI-to-APB Bridge (BP135)" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_AXI,		"PL-301 GPV" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_EROM,		"Enumeration ROM" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_OOB_ROUTER,		"OOB Interrupt Router" },
	{ JEDEC_MFGID_ARM,	BHND_COREID_DEF_AI_COMP,	"Default Core (Unused Address Ranges)" },
	{ 0,			0,			NULL }
};

/**
 * Return a human-readable name for a BHND core.
 * 
 * @param mfgid The core designer's JEDEC-106 Manufacturer ID
 * @param coreid The Broadcom core identifier.
 */
const char *bhnd_core_name (uint16_t mfgid, uint16_t coreid) {
	for (u_int i = 0; bhnd_core_descs[i].desc != NULL; i++) {
		if (bhnd_core_descs[i].mfgid != mfgid)
			continue;
		
		if (bhnd_core_descs[i].coreid != coreid)
			continue;
		
		return bhnd_core_descs[i].desc;
	}
	
	return "unknown";
}

static int
bhnd_probe(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_attach(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_detach(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_suspend(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_resume(device_t dev)
{
	return (ENXIO);
}

static device_method_t bhnd_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_probe),
	DEVMETHOD(device_attach,	bhnd_attach),
	DEVMETHOD(device_detach,	bhnd_detach),
	DEVMETHOD(device_suspend,	bhnd_suspend),
	DEVMETHOD(device_resume,	bhnd_resume),
	DEVMETHOD_END
};
static driver_t bhnd_driver = {
	"bhnd",
	bhnd_methods,
	sizeof(struct bhnd_softc)
};
static devclass_t bhnd_devclass;
DRIVER_MODULE(bhnd, bhnd, bhnd_driver, bhnd_devclass, 0, 0);