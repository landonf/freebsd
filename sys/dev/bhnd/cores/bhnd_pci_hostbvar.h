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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_CORES_PCI_HOSTBVAR_H_
#define _BHND_CORES_PCI_HOSTBVAR_H_

#include "bhnd_pcivar.h"

/* PCI hostb device identification */
struct bhnd_hostb_device {
	uint16_t			 device;
	const char			*desc;
	bhnd_pci_regs_t			 regs;
	struct bhnd_device_quirk	*quirks;
};

/** PCI host bridge driver state */
#define	BHND_PCI_HOSTB_MAX_RES		2
#define	BHND_PCI_HOSTB_MAX_RSPEC	(BHND_PCI_HOSTB_MAX_RES+1)
struct bhnd_pci_hostb_softc {
	device_t		 dev;		/**< pci device */
	struct bhnd_resource	*core;		/**< core registers. */
	bhnd_pci_regs_t		 regs;		/**< device register family */
	uint32_t		 quirks;	/**< BHND_PCI_QUIRK flags */

	struct resource_spec	 rspec[BHND_PCI_HOSTB_MAX_RSPEC];
	struct bhnd_resource	*res[BHND_PCI_HOSTB_MAX_RES];

};

/* Declare a bhnd_hostb_device entry */
#define	BHND_HOSTB_DEV(_device, _desc, ...)	{	\
	BHND_COREID_ ## _device, 				\
	"Broadcom " _desc "-BHND host bridge",		\
	BHND_PCI_REGS_ ## _device,				\
	(struct bhnd_device_quirk[]) {				\
		__VA_ARGS__					\
	}							\
}

/* BHNDB_PCI_REG_* convenience macros */ 
#define	BPCI_REG_GET			BHND_PCI_REG_GET
#define	BPCI_REG_SET			BHND_PCI_REG_SET

#define	BPCI_COMMON_REG_GET(_r, _a)	\
	BHND_PCI_COMMON_REG_GET(sc->regs, _r, _a)

#define	BPCI_COMMON_REG_SET(_r, _a, _v)	\
	BHND_PCI_COMMON_REG_SET(sc->regs, _r, _a, _v)

#define	BPCI_COMMON_REG(_name)		\
	BHND_PCI_COMMON_REG(sc->regs, _name)

#define	BPCI_COMMON_REG_OFFSET(_base, _offset)	\
	(BPCI_COMMON_REG(_base) + BPCI_COMMON_REG(_offset))

/* 
 * PCI/PCIe-Gen1 endpoint-mode device quirks
 */
enum {
	/** No quirks */
	BHND_PCI_QUIRK_NONE			= 0,

	/**
	 * SBTOPCI_PREF and SBTOPCI_BURST must be set on the
	 * SSB_PCICORE_SBTOPCI2 register.
	 */
	BHND_PCI_QUIRK_SBTOPCI2_PREF_BURST	= (1<<1),

	/**
	 * SBTOPCI_RC_READMULTI must be set on the SSB_PCICORE_SBTOPCI2
	 * register.
	 */
	BHND_PCI_QUIRK_SBTOPCI2_READMULTI	= (1<<2),

	/**
	 * Interrupt masking is handled via the interconnect configuration
	 * registers (SBINTVEC on siba), rather than the PCI_INT_MASK
	 * config register.
	 */
	BHND_PCI_QUIRK_SBINTVEC			= (1<<3),

	/**
	 * PCI CLKRUN# should be explicitly disabled (via CLKRUN_DSBL).
	 */
	BHND_PCI_QUIRK_CLKRUN_DSBL		= (1<<4),

	/**
	 * PCIe Vendor-Defined Messages should never set the 
	 * 'Unsupported Request' bit.
	 */
	BHND_PCIE_QUIRK_IGNORE_VDM		= (1<<5),

	/**
	 * PCI-PM power management must be explicitly enabled via
	 * the data link control register.
	 */
	BHND_PCIE_QUIRK_PCIPM_REQEN		= (1<<6),

	/**
	 * Fix L0s to L0 exit transition.
	 * 
	 * Increase SerDes RX timer to ensure SerDes CDR circuit is
	 * stable.
	 * 
	 * Modify CDR bandwidth (reason undocumented).
	 */
	BHND_PCIE_QUIRK_SERDES_L0s_HANG		= (1<<7),

	/**
	 * The idle time for entering L1 low-power state must be
	 * explicitly set (to 114ns) to fix slow L1->L0 transition issues.
	 */
	BHND_PCIE_QUIRK_L1_IDLE_THRESH		= (1<<8),
	
	/**
	 * The ASPM L1 entry timer should be extended for better performance,
	 * and restored for better power savings.
	 */
	BHND_PCIE_QUIRK_L1_TIMER_PERF		= (1<<9),

	/**
	 * ASPM and ECPM settings must be overridden manually.
	 * 
	 * The override behavior is controlled by the BHND_BFL2_PCIEWAR_OVR
	 * flag. If this flag is set, ASPM/CLKREQ should be overridden as
	 * enabled; otherwise, they should be overridden as disabled.
	 * 
	 * Attach/Resume:
	 *   - Set SRSH_ASPM_ENB flag in the SPROM ASPM register.
	 *   - Set ASPM L0S/L1 in the PCIER_LINK_CTL register.
	 *   - Set SRSH_CLKREQ_ENB flag in the SPROM CLKREQ_REV5 register.
	 *   - Clear ECPM in the PCIER_LINK_CTL register.
	 * 
	 * Detach/Suspend:
	 * - 
	 * - When the device enters D3 state, or system enters S3/S4 state,
	 *   clear ASPM L1 in the PCIER_LINK_CTL register.
	 */
	BHND_PCIE_QUIRK_ASPM_OVR		= (1<<10),
	
	/**
	 * The SerDes polarity must be saved at device attachment, and
	 * restored on suspend/resume.
	 */
	BHND_PCIE_QUIRK_SERDES_POLARITY		= (1<<11),

	/**
	 * The SerDes PLL override flag (CHIPCTRL_4321_PLL_DOWN) must be set on
	 * the ChipCommon core.
	 */
	BHND_PCIE_QUIRK_SERDES_NOPLLDOWN	= (1<<12),

	/**
	 * On attach and resume, consult the SPROM to determine whether
	 * the L2/L3-Ready w/o PCI RESET work-around must be applied.
	 * 
	 * If L23READY_EXIT_NOPRST is set in the SPROM, write the
	 * L23READY_EXIT_NOPRST flag to the PCI register defined by
	 * SRSH_PCIE_MISC_CONFIG.
	 */
	BHND_PCIE_QUIRK_SPROM_L23_PCI_RESET	= (1<<13),
};


#endif /* _BHND_CORES_PCI_HOSTBVAR_H_ */
