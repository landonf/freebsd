/*-
 * Copyright (c) 2017 Landon Fuller <landonf@FreeBSD.org>
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


#ifndef _MIPS_BROADCOM_BCM_GEOM_CFE_H_
#define _MIPS_BROADCOM_BCM_GEOM_CFE_H_

#define	CFE_CLASS_NAME		"CFE_MAP"

/* CFE binary magic */
#define	CFE_MAGIC		0x43464531	/* 'CFE1' */
#define	CFE_MAGIC_OFFSET	0x4E0		/**< CFE magic offset */
#define	CFE_MAGIC_COUNT		2		/**< CFE magic count */
#define	CFE_MAGIC_0		0		/**< 1st CFE magic constant */
#define	CFE_MAGIC_1		1		/**< 2nd CFE magic constant */

/* Self-describing compressed CFEZ binary magic */
#define	CFE_BISZ_OFFSET		0x3E0
#define	CFE_BISZ_MAGIC		0x4249535A	/* 'BISZ' */

#define	CFE_MAX_IMG		2		/**< maximum CFE OS image count */


/**
 * GEOM CFE_MAP instance state
 */
struct g_cfe_softc {
};

/**
 * CFE/GEOM device mapping entry.
 * 
 * Provides a mapping between CFE device names, the GEOM device attribute
 * providing access to the backing device, and a list of supported ChipCommon
 * flash types (terminated by CHIPC_FLASH_NONE).
 */
struct cfe_flash_device {
	const char		*cfe_name;	/**< CFE device class name */
	const char		*geom_attr;	/**< GEOM device attribute */
	const chipc_flash	*flash_types;	/**< supported ChipCommon flash types */
};

/**
 * CFE operating system image layout types.
 */
typedef enum {
	CFE_IMAGE_FAILSAFE,	/**< CFE with FAILSAFE_UPGRADE enabled */
	CFE_IMAGE_DUAL,		/**< CFE with DUAL_IMAGE enabled */
	CFE_IMAGE_SIMPLE	/**< CFE with default config (single image) */
} cfe_bootimg_type;

/**
 * CFE operating system image info.
 */
struct cfe_bootimg_info {
	cfe_bootimg_type	type;			/**< CFE layout type */
	uint8_t			bootimage;		/**< boot image index */
	size_t			num_images;		/**< image count */
	uint64_t		offsets[CFE_MAX_IMG];	/**< image offsets */
	uint64_t		sizes[CFE_MAX_IMG];	/**< image sizes */
};

#endif /* _MIPS_BROADCOM_BCM_GEOM_CFE_H_ */
