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
#include "bhnd_core.h"
#include "bhndreg.h"
#include "bhndvar.h"



/* BHND core device description table. */
static const struct bhnd_core_desc {
	uint16_t designer;
	uint16_t part_num;
	const char *desc;
} bhnd_core_descs[] = {
	{ JEDEC_MFGID_BCM,	CC_CORE_ID,		"ChipCommon" },
	{ JEDEC_MFGID_BCM,	ILINE20_CORE_ID,	"iLine20 HPNA" },
	{ JEDEC_MFGID_BCM,	SRAM_CORE_ID,		"SRAM" },
	{ JEDEC_MFGID_BCM,	SDRAM_CORE_ID,		"SDRAM" },
	{ JEDEC_MFGID_BCM,	PCI_CORE_ID,		"PCI Bridge" },
	{ JEDEC_MFGID_BCM,	MIPS_CORE_ID,		"MIPS Core" },
	{ JEDEC_MFGID_BCM,	ENET_CORE_ID,		"Fast Ethernet MAC" },
	{ JEDEC_MFGID_BCM,	CODEC_CORE_ID,		"V.90 Modem Codec" },
	{ JEDEC_MFGID_BCM,	USB_CORE_ID,		"USB 1.1 Device/Host Controller" },
	{ JEDEC_MFGID_BCM,	ADSL_CORE_ID,		"ADSL Core" },
	{ JEDEC_MFGID_BCM,	ILINE100_CORE_ID,	"iLine100 HPNA" },
	{ JEDEC_MFGID_BCM,	IPSEC_CORE_ID,		"IPsec Accelerator" },
	{ JEDEC_MFGID_BCM,	UTOPIA_CORE_ID,		"UTOPIA ATM Core" },
	{ JEDEC_MFGID_BCM,	PCMCIA_CORE_ID,		"PCMCIA Bridge" },
	{ JEDEC_MFGID_BCM,	SOCRAM_CORE_ID,		"Internal Memory" },
	{ JEDEC_MFGID_BCM,	MEMC_CORE_ID,		"MEMC SDRAM Controller" },
	{ JEDEC_MFGID_BCM,	OFDM_CORE_ID,		"OFDM PHY" },
	{ JEDEC_MFGID_BCM,	EXTIF_CORE_ID,		"External Interface" },
	{ JEDEC_MFGID_BCM,	D11_CORE_ID,		"802.11 MAC" },
	{ JEDEC_MFGID_BCM,	APHY_CORE_ID,		"802.11a PHY" },
	{ JEDEC_MFGID_BCM,	BPHY_CORE_ID,		"802.11b PHY" },
	{ JEDEC_MFGID_BCM,	GPHY_CORE_ID,		"802.11g PHY" },
	{ JEDEC_MFGID_BCM,	MIPS33_CORE_ID,		"MIPS 3302 Core" },
	{ JEDEC_MFGID_BCM,	USB11H_CORE_ID,		"USB 1.1 Host Controller" },
	{ JEDEC_MFGID_BCM,	USB11D_CORE_ID,		"USB 1.1 Device Core" },
	{ JEDEC_MFGID_BCM,	USB20H_CORE_ID,		"USB 2.0 Host Controller" },
	{ JEDEC_MFGID_BCM,	USB20D_CORE_ID,		"USB 2.0 Device Core" },
	{ JEDEC_MFGID_BCM,	SDIOH_CORE_ID,		"SDIO Host Controller" },
	{ JEDEC_MFGID_BCM,	ROBO_CORE_ID,		"RoboSwitch" },
	{ JEDEC_MFGID_BCM,	ATA100_CORE_ID,		"Parallel ATA Controller" },
	{ JEDEC_MFGID_BCM,	SATAXOR_CORE_ID,	"SATA DMA/XOR Controller" },
	{ JEDEC_MFGID_BCM,	GIGETH_CORE_ID,		"Gigabit Ethernet MAC" },
	{ JEDEC_MFGID_BCM,	PCIE_CORE_ID,		"PCIe Bridge" },
	{ JEDEC_MFGID_BCM,	NPHY_CORE_ID,		"802.11n 2x2 PHY" },
	{ JEDEC_MFGID_BCM,	SRAMC_CORE_ID,		"SRAM Controller" },
	{ JEDEC_MFGID_BCM,	MINIMAC_CORE_ID,	"MINI MAC/PHY" },
	{ JEDEC_MFGID_BCM,	ARM11_CORE_ID,		"ARM1176 Core" },
	{ JEDEC_MFGID_BCM,	ARM7S_CORE_ID,		"ARM7TDMI-S Core" },
	{ JEDEC_MFGID_BCM,	LPPHY_CORE_ID,		"802.11a/b/g PHY" },
	{ JEDEC_MFGID_BCM,	PMU_CORE_ID,		"PMU" },
	{ JEDEC_MFGID_BCM,	SSNPHY_CORE_ID,		"802.11n Single-Stream PHY" },
	{ JEDEC_MFGID_BCM,	SDIOD_CORE_ID,		"SDIO Device Core" },
	{ JEDEC_MFGID_BCM,	ARMCM3_CORE_ID,		"ARM Cortex-M3 Core" },
	{ JEDEC_MFGID_BCM,	HTPHY_CORE_ID,		"802.11n 4x4 PHY" },
	{ JEDEC_MFGID_BCM,	MIPS74K_CORE_ID,	"MIPS74k Core" },
	{ JEDEC_MFGID_BCM,	GMAC_CORE_ID,		"Gigabit MAC core" },
	{ JEDEC_MFGID_BCM,	DMEMC_CORE_ID,		"DDR1/2 Memory Controller" },
	{ JEDEC_MFGID_BCM,	PCIERC_CORE_ID,		"PCIe Root Complex" },
	{ JEDEC_MFGID_BCM,	OCP_CORE_ID,		"OCP2OCP Bridge" },
	{ JEDEC_MFGID_BCM,	SC_CORE_ID,		"Shared Common Core" },
	{ JEDEC_MFGID_BCM,	AHB_CORE_ID,		"OCP2AHB Bridge" },
	{ JEDEC_MFGID_BCM,	SPIH_CORE_ID,		"SPI Host Controller" },
	{ JEDEC_MFGID_BCM,	I2S_CORE_ID,		"I2S Digital Audio Interface" },
	{ JEDEC_MFGID_BCM,	DMEMS_CORE_ID,		"SDR/DDR1 Memory Controller" },
	{ JEDEC_MFGID_BCM,	DEF_SHIM_COMP,		"BCM6362/UBUS WLAN SHIM" },
	
	{ JEDEC_MFGID_ARM,	APB_BRIDGE_CORE_ID,	"AXI-to-APB Bridge (BP135)" },
	{ JEDEC_MFGID_ARM,	AXI_CORE_ID,		"PL-301 GPV" },
	{ JEDEC_MFGID_ARM,	EROM_CORE_ID,		"Enumeration ROM" },
	{ JEDEC_MFGID_ARM,	OOB_ROUTER_CORE_ID,	"OOB Interrupt Router" },
	{ JEDEC_MFGID_ARM,	DEF_AI_COMP,		"Default Core (Unused Address Ranges)" },
	{ 0,			0,			NULL }
};

/**
 * Return a human-readable name for a BHND core.
 * 
 * @param designer The core designer's JEDEC-106 Manufacturer ID
 * @param part_num The core's part number.
 */
const char *bhnd_core_name (uint16_t designer, uint16_t part_num) {
	for (u_int i = 0; bhnd_core_descs[i].desc != NULL; i++) {
		if (bhnd_core_descs[i].designer != designer)
			continue;
		
		if (bhnd_core_descs[i].part_num != part_num)
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