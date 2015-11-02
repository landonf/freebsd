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

#ifndef _BWN_IF_BWN_PCIVAR_H_
#define _BWN_IF_BWN_PCIVAR_H_

struct bcma_pci_regwin;

/*
 * Broadcom PCI-bridged AMBA backplane data structure definitions.
 */

#define	BCMA_PCI_MAX_RES	2	/**< maximum number of bmca PCI device
					  *  resources. */

/* bwn driver instance state */
struct bwn_pci_softc {
	device_t		 bcma_dev;			/**< device */
	struct resource		*pci_res[BCMA_PCI_MAX_RES];	/**< device resources */
	struct rman		 mem_rman;			/**< bus memory resource manager */
};

/**
 * Register window types.
 */
typedef enum {
	BCMA_PCI_WINTYPE_FIXED,	/**< A fixed window that maps a specific core
				 *   type */
	BCMA_PCI_WINTYPE_DYN	/**< A dynamically configurable window */
} bcma_pci_wintype_t;

/**
 * Defines a register window within a BCMA PCI BAR resource that can be
 * adjusted at runtime to map address regions from the SoC interconnect.
 */
struct bcma_pci_regwin {
	u_int			pci_res;	/**< pci_res[] index of the BAR
						     in which this register
						     window is mapped */

	bcma_pci_wintype_t	win_type;	/** register window type */
	union {
		/** BCMA_PCI_WINTYPE_DYN configuration */
		struct {
			bus_size_t	cfg_offset;	/**< window config offset within the pci
							     config space. */
		};
		
		/** BCMA_PCI_WINTYPE_FIXED configuration */
		struct {
			bhnd_devclass_t	bhnd_class;	/**< mapped cores' class */
			u_int		port_num;	/**< mapped port number */
			u_int		region_num;	/**< mapped region number */
		};
	};

	bus_size_t	win_offset;	/**< offset of the register block
					 *   within the PCI resource at */
	bus_size_t	win_size;	/**< size of the register block */
};

#endif /* _BWN_IF_BWN_PCIVAR_H_ */