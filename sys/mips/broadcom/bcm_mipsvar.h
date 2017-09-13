/*-
 * Copyright (c) 2017 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Landon Fuller under sponsorship from
 * the FreeBSD Foundation.
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

#ifndef _MIPS_BROADCOM_BCM_MIPSVAR_H_
#define _MIPS_BROADCOM_BCM_MIPSVAR_H_

#include <sys/param.h>
#include <sys/bus.h>

#include <sys/intr.h>

DECLARE_CLASS(bcm_mips_driver);

#define	BCM_MIPS_NINTR	32	/**< maximum number of addressable backplane interrupt vectors */

int	bcm_mips_attach(device_t dev);
int	bcm_mips_detach(device_t dev);

/**
 * Broadcom MIPS PIC interrupt map data.
 */
struct bcm_mips_intr_map_data {
	struct intr_map_data	mdata;
	u_int			ivec;	/**< bus interrupt vector */
};

/**
 * Broadcom MIPS PIC interrupt source definition.
 */
struct bcm_mips_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			ivec;	/**< bus interrupt vector */
};

/**
 * bcm_mips driver instance state. Must be first member of all subclass
 * softc structures.
 */
struct bcm_mips_softc {
	device_t		 dev;
	struct resource		*mem;			/**< cpu core registers */
	int			 mem_rid;
	struct resource		*irq;			/**< nested IRQ handler's MIPS interrupt */
	int			 irq_rid;
	struct bcm_mips_irqsrc	 isrcs[BCM_MIPS_NINTR];
};


#define	INTR_MAP_DATA_BCM_MIPS	INTR_MAP_DATA_PLAT_2	/**< Broadcom MIPS PIC interrupt map data type */
#define	BCM_MIPS_PIC_IRQ	2			/**< MIPS HW IRQ used by our nested interrupt handler */

#endif /* _MIPS_BROADCOM_BCM_MIPSVAR_H_ */
