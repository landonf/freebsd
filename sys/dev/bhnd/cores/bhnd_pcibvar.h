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

#ifndef _BHND_CORES_PCIBVAR_H_
#define _BHND_CORES_PCIBVAR_H_

#define	BHND_PCIB_MAX_RES	2
#define	BHND_PCIB_MAX_RSPEC	(BHND_PCIB_MAX_RES+1)

/* Device register families. */
typedef enum {
	BHNDB_PCIB_REGS_PCI	= 0,	/* PCI register definitions */
	BHNDB_PCIB_REGS_PCIE	= 1,	/* PCIe-Gen1 register definitions */
} bhndb_pcib_regs_t;

/** PCI bridge driver-specific state */
struct bhnd_pcib_softc {
	device_t		 dev;	/**< pci device */
	struct bhnd_resource	*core;	/**< core registers. */
	bhndb_pcib_regs_t	 regs;	/**< device register family */

	struct resource_spec	 rspec[BHND_PCIB_MAX_RSPEC];
	struct bhnd_resource	*res[BHND_PCIB_MAX_RES];

};

/* broadcom pci/pcie-gen1 device quirks */
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



/**
 * Extract a register value by applying _MASK and _SHIFT defines to the common
 * PCI/PCIe register definition @p _regv
 * 
 * @param _sc The pcib device state.
 * @param _regv The register value containing the desired attribute
 * @param _attr The register attribute name to which to prepend the register
 * definition prefix and append `_MASK`/`_SHIFT` suffixes.
 */
#define BHND_PCIB_COMMON_REG_GET(_sc, _regv, _attr)	\
	_BHND_PCI_REG_GET(_regv,			\
	    BHND_PCIB_COMMON_REG(sc, _attr ## _MASK),	\
	    BHND_PCIB_COMMON_REG(sc, _attr ## _SHIFT))

/**
 * Set a register value by applying _MASK and _SHIFT defines to the common
 * PCI/PCIe register definition @p _regv
 * 
 * @param _sc The pcib device state.
 * @param _regv The register value containing the desired attribute
 * @param _attr The register attribute name to which to prepend the register
 * definition prefix and append `_MASK`/`_SHIFT` suffixes.
 * @param _val The value to bet set in @p _regv.
 */
#define BHND_PCIB_COMMON_REG_SET(_sc, _regv, _attr, _val)	\
	_BHND_PCI_REG_SET(_regv,				\
	    BHND_PCIB_COMMON_REG(sc, _attr ## _MASK),		\
	    BHND_PCIB_COMMON_REG(sc, _attr ## _SHIFT),		\
	    _val)


/**
 * Evaluates to the offset of a common PCI/PCIe register definition. 
 * 
 * This will trigger a compile-time error if the register is not defined
 * for all supported PCI/PCIe cores.
 * 
 * This should be optimized down to a constant value if the register constant
 * is the same across the register definitions.
 */
#define	BHND_PCIB_COMMON_REG(_sc, _name)	(			\
	(_sc)->regs == BHNDB_PCIB_REGS_PCI ? BHND_PCI_ ## _name :	\
	BHND_PCIE_ ## _name						\
)

#endif /* _BHND_CORES_PCIBVAR_H_ */