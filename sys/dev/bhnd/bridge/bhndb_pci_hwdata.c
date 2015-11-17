 
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
 * Resource specifications and register maps for Broadcom PCI/PCIe cores 
 * configured as PCI-BHND bridges.
 */

#include <sys/param.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "bhndb_pcireg.h"
#include "bhndb_pcivar.h"

static const struct bhndb_hwcfg bhndb_pci_hwcfg_v0;
static const struct bhndb_hwcfg bhndb_pci_hwcfg_v1;
static const struct bhndb_hwcfg bhndb_pci_hwcfg_v2;
static const struct bhndb_hwcfg bhndb_pci_hwcfg_v3;

#define	_BHNDB_HW_REQ_ARRAY(...) (struct bhnd_core_match[]) { __VA_ARGS__ }

/**
 * Define a bhndb_hw match entry.
 * 
 * @param _name The entry name.
 * @param _vers The configuration version associated with this entry.
 */
#define	BHNDB_HW_MATCH(_name, _vers, ...) {				\
	.name		= _name,					\
	.hw_reqs	= _BHNDB_HW_REQ_ARRAY(__VA_ARGS__),		\
	.num_hw_reqs	= (sizeof(_BHNDB_HW_REQ_ARRAY(__VA_ARGS__)) /	\
	    sizeof(_BHNDB_HW_REQ_ARRAY(__VA_ARGS__)[0])),		\
	.cfg		= &bhndb_pci_hwcfg_ ## _vers			\
}

/**
 * Hardware configuration tables for Broadcom HND PCI NICs.
 */
const struct bhndb_hw bhndb_pci_hw[] = {
	/* PCI/V0 */
	BHNDB_HW_MATCH("PCI/v0 NIC", v0, {
		.vendor	= BHND_MFGID_BCM,
		.device	= BHND_COREID_PCI,
		.hwrev	= {
			.start	= 0,
			.end	= BHNDB_PCI_V0_MAX_PCI_HWREV
		},
		.class	= BHND_DEVCLASS_PCI,
		.unit	= 0
	}),
	
	/* PCI/V1 */
	BHNDB_HW_MATCH("PCI/v1 NIC", v1, {
		.vendor	= BHND_MFGID_BCM,
		.device	= BHND_COREID_PCI,
		.hwrev	= { 
			.start	= BHNDB_PCI_V1_MIN_PCI_HWREV,
			.end	= BHND_HWREV_INVALID
		},
		.class	= BHND_DEVCLASS_PCI,
		.unit	= 0
	}),

	/* PCIE/V1 */
	BHNDB_HW_MATCH("PCIe/v1 NIC", v1,
		/* PCIe Core */
		{
			.vendor	= BHND_MFGID_BCM,
			.device	= BHND_COREID_PCIE,
			.hwrev	= {
				.start	= 0,
				.end	= BHND_HWREV_INVALID
			},
			.class	= BHND_DEVCLASS_PCIE,
			.unit	= 0
		},

		/* ChipCommon (revision <= 31) */
		{
			.vendor	= BHND_MFGID_BCM,
			.device	= BHND_COREID_CC,
			.hwrev	= {
				.start	= 0,
				.end	= BHNDB_PCI_V1_MAX_CHIPC_HWREV
			},
			.class	= BHND_DEVCLASS_CC,
			.unit	= 0
		}
	),

	/* PCIE/V2 */
	BHNDB_HW_MATCH("PCIe/v2 NIC", v2,
		/* PCIe Core */
		{
			.vendor	= BHND_MFGID_BCM,
			.device	= BHND_COREID_PCIE,
			.hwrev	= { 0, BHND_HWREV_INVALID },
			.class	= BHND_DEVCLASS_PCIE,
			.unit	= 0
		},

		/* ChipCommon (revision >= 32) */
		{
			.vendor	= BHND_MFGID_BCM,
			.device	= BHND_COREID_CC,
			.hwrev	= {
				.start	= BHNDB_PCI_V2_MIN_CHIPC_HWREV,
				.end	= BHND_HWREV_INVALID
			},
			.class	= BHND_DEVCLASS_CC,
			.unit	= 0
		}
	),


	/* PCIE/V3 */
	BHNDB_HW_MATCH("PCIe-Gen2/v3 NIC", v3,
		/* PCIe Gen2 Core */
		{
			.vendor	= BHND_MFGID_BCM,
			.device	= BHND_COREID_PCIE2,
			.hwrev	= {
				.start	= 0,
				.end	= BHND_HWREV_INVALID
			},
			.class	= BHND_DEVCLASS_PCIE,
			.unit	= 0
		}
	),

	{ NULL, NULL, 0, NULL }
};



/**
 * Generic PCI-SIBA bridge configuration usable with all known siba(4)-based
 * PCI devices; this configuration is adequate for enumerating a bridged
 * siba(4) bus to determine the full hardware configuration.
 * 
 * Compatible with PCI_V0, PCI_V1, PCI_V2, and PCI_V3 devices.
 * 
 * While compatible with bcma(4) devices, this configuration does not define a
 * ChipCommon register window required for bcma(4) bus enumeration.
 * 
 * TODO: Lift out into sibab_pci.c/h
 */
const struct bhndb_hwcfg sibab_pci_hwcfg_generic = {
	.resource_specs = (const struct resource_spec[]) {
		{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
		{ -1,			0,		0 }
	},

	.register_windows = (const struct bhndb_regwin[]) {
		/* bar0+0x0000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V1_BAR0_WIN0_OFFSET,
			.win_size	= BHNDB_PCI_V1_BAR0_WIN0_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V1_BAR0_WIN0_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		BHNDB_REGWIN_TABLE_END
	},
};

/**
 * PCI_V0 hardware configuration.
 * 
 * Applies to:
 * - PCI (cid=0x804, revision <= 12)
 */
static const struct bhndb_hwcfg bhndb_pci_hwcfg_v0 = {
	.resource_specs		= (const struct resource_spec[]) {
		{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
		{ -1,			0,		0 }
	},

	.register_windows	= (const struct bhndb_regwin[]) {
		/* bar0+0x0000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V0_BAR0_WIN0_OFFSET,
			.win_size	= BHNDB_PCI_V0_BAR0_WIN0_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V0_BAR0_WIN0_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x1000: sprom shadow */ 
		{
			.win_type	= BHNDB_REGWIN_T_SPROM,
			.win_offset	= BHNDB_PCI_V0_BAR0_SPROM_OFFSET,
			.win_size	= BHNDB_PCI_V0_BAR0_SPROM_SIZE,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x1800: pci core registers */
		{
			.win_type	= BHNDB_REGWIN_T_CORE,
			.win_offset	= BHNDB_PCI_V0_BAR0_PCIREG_OFFSET,
			.win_size	= BHNDB_PCI_V0_BAR0_PCIREG_SIZE,
			.core = {
				.class	= BHND_DEVCLASS_PCI,
				.unit	= 0,
				.port	= 0,
				.region	= 0
			},
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		BHNDB_REGWIN_TABLE_END
	},
};

/**
 * PCI_V1 hardware configuration.
 * 
 * Applies to:
 * - PCI (cid=0x804, revision >= 13)
 * - PCIE (cid=0x820) with ChipCommon (revision <= 31)
 */
static const struct bhndb_hwcfg bhndb_pci_hwcfg_v1 = {
	.resource_specs		= (const struct resource_spec[]) {
		{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
		{ -1,			0,		0 }
	},

	.register_windows	= (const struct bhndb_regwin[]) {
		/* bar0+0x0000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V1_BAR0_WIN0_OFFSET,
			.win_size	= BHNDB_PCI_V1_BAR0_WIN0_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V1_BAR0_WIN0_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x1000: sprom shadow */
		{
			.win_type	= BHNDB_REGWIN_T_SPROM,
			.win_offset	= BHNDB_PCI_V1_BAR0_SPROM_OFFSET,
			.win_size	= BHNDB_PCI_V1_BAR0_SPROM_SIZE,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x2000: pci core registers */
		{
			.win_type	= BHNDB_REGWIN_T_CORE,
			.win_offset	= BHNDB_PCI_V1_BAR0_PCIREG_OFFSET,
			.win_size	= BHNDB_PCI_V1_BAR0_PCIREG_SIZE,
			.core = {
				.class	= BHND_DEVCLASS_PCI,
				.unit	= 0,
				.port	= 0,
				.region	= 0 
			},
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},

		/* bar0+0x3000: chipc core registers */
		{
			.win_type	= BHNDB_REGWIN_T_CORE,
			.win_offset	= BHNDB_PCI_V1_BAR0_CCREGS_OFFSET,
			.win_size	= BHNDB_PCI_V1_BAR0_CCREGS_SIZE,
			.core = {
				.class	= BHND_DEVCLASS_CC,
				.unit	= 0,
				.port	= 0,
				.region	= 0 
			},
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},

		BHNDB_REGWIN_TABLE_END
	},
};

/**
 * PCI_V2 hardware configuration.
 * 
 * Applies to:
 * - PCIE (cid=0x820) with ChipCommon (revision >= 32)
 */
static const struct bhndb_hwcfg bhndb_pci_hwcfg_v2 = {
	.resource_specs		= (const struct resource_spec[]) {
		{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
		{ -1,			0,		0 }
	},

	.register_windows	= (const struct bhndb_regwin[]) {
		/* bar0+0x0000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V2_BAR0_WIN0_OFFSET,
			.win_size	= BHNDB_PCI_V2_BAR0_WIN0_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V2_BAR0_WIN0_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x1000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V2_BAR0_WIN1_OFFSET,
			.win_size	= BHNDB_PCI_V2_BAR0_WIN1_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V2_BAR0_WIN1_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x2000: pcie core registers */
		{
			.win_type	= BHNDB_REGWIN_T_CORE,
			.win_offset	= BHNDB_PCI_V2_BAR0_PCIREG_OFFSET,
			.win_size	= BHNDB_PCI_V2_BAR0_PCIREG_SIZE,
			.core = {
				.class	= BHND_DEVCLASS_PCI,
				.unit	= 0,
				.port	= 0,
				.region	= 0 
			},
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},

		/* bar0+0x3000: chipc core registers */
		{
			.win_type	= BHNDB_REGWIN_T_CORE,
			.win_offset	= BHNDB_PCI_V2_BAR0_CCREGS_OFFSET,
			.win_size	= BHNDB_PCI_V2_BAR0_CCREGS_SIZE,
			.core = {
				.class	= BHND_DEVCLASS_CC,
				.unit	= 0,
				.port	= 0,
				.region	= 0 
			},
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},

		BHNDB_REGWIN_TABLE_END
	},
};

/**
 * PCI_V3 hardware configuration.
 * 
 * Applies to:
 * - PCIE2 (cid=0x83c)
 */
static const struct bhndb_hwcfg bhndb_pci_hwcfg_v3 = {
	.resource_specs		= (const struct resource_spec[]) {
		{ SYS_RES_MEMORY,	PCIR_BAR(0),	RF_ACTIVE },
		{ -1,			0,		0 }
	},

	.register_windows	= (const struct bhndb_regwin[]) {
		/* bar0+0x0000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V3_BAR0_WIN0_OFFSET,
			.win_size	= BHNDB_PCI_V3_BAR0_WIN0_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V3_BAR0_WIN0_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x1000: configurable backplane window */
		{
			.win_type	= BHNDB_REGWIN_T_DYN,
			.win_offset	= BHNDB_PCI_V3_BAR0_WIN1_OFFSET,
			.win_size	= BHNDB_PCI_V3_BAR0_WIN1_SIZE,
			.dyn.cfg_offset = BHNDB_PCI_V3_BAR0_WIN1_CONTROL,
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},
		
		/* bar0+0x2000: pcie core registers */
		{
			.win_type	= BHNDB_REGWIN_T_CORE,
			.win_offset	= BHNDB_PCI_V3_BAR0_PCIREG_OFFSET,
			.win_size	= BHNDB_PCI_V3_BAR0_PCIREG_SIZE,
			.core = {
				.class	= BHND_DEVCLASS_PCI,
				.unit	= 0,
				.port	= 0,
				.region	= 0 
			},
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},

		/* bar0+0x3000: chipc core registers */
		{
			.win_type	= BHNDB_REGWIN_T_CORE,
			.win_offset	= BHNDB_PCI_V3_BAR0_CCREGS_OFFSET,
			.win_size	= BHNDB_PCI_V3_BAR0_CCREGS_SIZE,
			.core = {
				.class	= BHND_DEVCLASS_CC,
				.unit	= 0,
				.port	= 0,
				.region	= 0 
			},
			.res		= { SYS_RES_MEMORY, PCIR_BAR(0) }
		},

		BHNDB_REGWIN_TABLE_END
	},
};