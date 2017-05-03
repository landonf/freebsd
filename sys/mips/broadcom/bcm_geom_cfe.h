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

#include <sys/param.h>
#include <geom/geom.h>

#include <dev/bhnd/nvram/bhnd_nvram_iovar.h>

#include "bcm_cfe_disk.h"

#define	CFE_CLASS_NAME		"BCM_CFE"

/* CFE binary magic */
#define	CFE_MAGIC		0x43464531	/**< 'CFE1' */
#define	CFE_MAGIC_OFFSET	0x4E0		/**< CFE magic offset */
#define	CFE_MAGIC_COUNT		2		/**< CFE magic count */
#define	CFE_MAGIC_0		0		/**< 1st CFE magic constant */
#define	CFE_MAGIC_1		1		/**< 2nd CFE magic constant */

/* Self-describing compressed CFEZ binary magic */
#define	CFE_BISZ_OFFSET		0x3E0
#define	CFE_BISZ_MAGIC		0x4249535A	/* 'BISZ' */

/* SENTRY5 'config' partition magic (MINIX v1 filesystem, 30 char name limit) */
#define	CFE_MINIX_OFFSET	0x410
#define	CFE_MINIX_MAGIC		0x138F

#define	CFE_IMG_MAX		2		/**< maximum CFE OS image count */

/**
 * GEOM BCM_CFE instance state
 */
struct g_cfe_softc {
};

/**
 * GEOM-backed bhnd_nvram_io implementation.
 * 
 * Provides GEOM-backed I/O mapping of a probed flash partition.
 */
struct g_cfe_nvram_io {
	struct bhnd_nvram_io	 io;		/**< common I/O instance state */
	struct g_consumer	*cp;		/**< GEOM consumer */
	struct bcm_cfe_disk	*disk;		/**< disk entry */
	struct bcm_cfe_part	*part;		/**< partition entry */
};

/** GEOM CFE flash probe function */
typedef int (g_cfe_probe)(struct g_consumer *cp, struct bcm_cfe_disk *disk,
			      struct bcm_cfe_part *part);

/**
 * CFE operating system image layout types.
 */
typedef enum {
	G_CFE_IMAGE_FAILSAFE,	/**< CFE with FAILSAFE_UPGRADE enabled */
	G_CFE_IMAGE_DUAL,	/**< CFE with DUAL_IMAGE enabled */
	G_CFE_IMAGE_SIMPLE	/**< CFE with default config (single image) */
} g_cfe_bootimg_type;

/**
 * CFE operating system image info.
 */
struct cfe_bootimg_info {
	g_cfe_bootimg_type	type;			/**< CFE boot image type */
	uint8_t			bootimage;		/**< boot image index */
	size_t			num_images;		/**< image count */
	uint64_t		offsets[CFE_IMG_MAX];	/**< image offsets */
	uint64_t		sizes[CFE_IMG_MAX];	/**< image sizes */
};

#endif /* _MIPS_BROADCOM_BCM_GEOM_CFE_H_ */
