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

#ifndef _BHND_BHND_TYPES_H_
#define _BHND_BHND_TYPES_H_

#include <sys/types.h>

/** bhnd(4) device classes. */
typedef enum {
	BHND_DEVCLASS_CC,		/**< chipcommon i/o controller */
	BHND_DEVCLASS_CC_B,		/**< chipcommon auxiliary controller */
	BHND_DEVCLASS_PMU,		/**< pmu controller */
	BHND_DEVCLASS_PCI,		/**< pci host/device bridge */
	BHND_DEVCLASS_PCIE,		/**< pcie host/device bridge */
	BHND_DEVCLASS_PCCARD,		/**< pcmcia host/device bridge */
	BHND_DEVCLASS_RAM,		/**< internal RAM/SRAM */
	BHND_DEVCLASS_MEMC,		/**< memory controller */
	BHND_DEVCLASS_ENET,		/**< 802.3 MAC/PHY */
	BHND_DEVCLASS_ENET_MAC,		/**< 802.3 MAC */
	BHND_DEVCLASS_ENET_PHY,		/**< 802.3 PHY */
	BHND_DEVCLASS_WLAN,		/**< 802.11 MAC/PHY/Radio */
	BHND_DEVCLASS_WLAN_MAC,		/**< 802.11 MAC */
	BHND_DEVCLASS_WLAN_PHY,		/**< 802.11 PHY */
	BHND_DEVCLASS_CPU,		/**< cpu core */
	BHND_DEVCLASS_SOC_ROUTER,	/**< interconnect router */
	BHND_DEVCLASS_SOC_BRIDGE,	/**< interconnect host bridge */
	BHND_DEVCLASS_EROM,		/**< bus device enumeration ROM */
	BHND_DEVCLASS_NVRAM,		/**< nvram/flash controller */
	BHND_DEVCLASS_OTHER,		/**< other / unknown */

	BHND_DEVCLASS_INVALID		/**< no/invalid class */
} bhnd_devclass_t;


/**
 * bhnd(4) port types.
 * 
 * Only BHND_PORT_DEVICE is guaranteed to be supported by all bhnd(4) bus
 * implementations.
 */
typedef enum {
	BHND_PORT_DEVICE	= 0,	/**< device memory */
	BHND_PORT_BRIDGE	= 1,	/**< bridge memory */
	BHND_PORT_AGENT		= 2,	/**< interconnect agent/wrapper */
} bhnd_port_type;

/**
 * bhnd(4) attachment types.
 */
typedef enum {
	BHND_ATTACH_ADAPTER	= 0,	/**< A bridged card, such as a PCI WiFi chipset  */
	BHND_ATTACH_NATIVE	= 1	/**< A bus resident on the native host, such as
					  *  the primary or secondary bus of an embedded
					  *  SoC */
} bhnd_attach_type;

/**
 * bhnd(4) clock types.
 */
typedef enum {
	/**
	 * Dynamically select an appropriate clock source based on all
	 * outstanding clock requests.
	 */
	BHND_CLOCK_DYN		= 0,

	/**
	 * Idle Low-Power (ILP).
	 * 
	 * No register access is required, or long request latency is
	 * acceptable.
	 */
	BHND_CLOCK_ILP		= 1,
	
	/**
	 * Active Low-Power (ALP).
	 * 
	 * Low-latency register access and low-rate DMA.
	 */
	BHND_CLOCK_ALP		= 2,
	
	/**
	 * High Throughput (HT).
	 * 
	 * High bus throughput and lowest-latency register access.
	 */
	BHND_CLOCK_HT		= 3
} bhnd_clock;

/**
 * bhnd(4) clock sources.
 */
typedef enum {
	/**
	 * Clock is provided by the PCI bus clock
	 */
	BHND_CLKSRC_PCI		= 0,

	/** Clock is provided by a crystal. */
	BHND_CLKSRC_XTAL	= 1,

	/** Clock is provided by a low power oscillator. */
	BHND_CLKSRC_LPO		= 2,

	/** Clock source is unknown */
	BHND_CLKSRC_UNKNOWN	= 3
} bhnd_clksrc;

/** Evaluates to true if @p cls is a device class that can be configured
 *  as a host bridge device. */
#define	BHND_DEVCLASS_SUPPORTS_HOSTB(cls)					\
	((cls) == BHND_DEVCLASS_PCI || (cls) == BHND_DEVCLASS_PCIE ||	\
	 (cls) == BHND_DEVCLASS_PCCARD)

/**
 * BHND bus address.
 * 
 * @note While the interconnect may support 64-bit addressing, not
 * all bridges and SoC CPUs will.
 */
typedef uint64_t	bhnd_addr_t;
#define	BHND_ADDR_MAX	UINT64_MAX	/**< Maximum bhnd_addr_t value */

/** BHND bus size. */
typedef uint64_t	bhnd_size_t;
#define	BHND_SIZE_MAX	UINT64_MAX	/**< Maximum bhnd_size_t value */


#endif /* _BHND_BHND_TYPES_H_ */
