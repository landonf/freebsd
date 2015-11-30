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

#ifndef _BHND_CORES_CHIPCREG_H_
#define _BHND_CORES_CHIPCREG_H_

#define	CHIPC_GET_ATTR(_entry, _attr) \
	((_entry & CHIPC_ ## _attr ## _MASK) >> CHIPC_ ## _attr ## _SHIFT)

/** device identification register */
#define	CHIPC_ID		0x0
#define	CHIPC_ID_CHIP_MASK	0x0000FFFF	/**< chip id */
#define	CHIPC_ID_CHIP_SHIFT	0
#define	CHIPC_ID_REV_MASK	0x000F0000	/**< chip revision */
#define	CHIPC_ID_REV_SHIFT	16
#define	CHIPC_ID_PKG_MASK	0x00F00000	/**< physical package ID */
#define	CHIPC_ID_PKG_SHIFT	20
#define	CHIPC_ID_NUMCORE_MASK	0x0F000000	/**< number of cores on chip (rev >= 4) */
#define	CHIPC_ID_NUMCORE_SHIFT	24
#define CHIPC_ID_BUS_MASK	0xF0000000	/**< interconnect type */
#define CHIPC_ID_BUS_SHIFT	28

/**< 32bit EROM address (bcma) */
#define	CHIPC_EROM_CORE_ADDR	0xFC

#endif /* _BHND_CORES_CHIPCREG_H_ */