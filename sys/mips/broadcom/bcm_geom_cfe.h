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

/* Self-describing compressed CFEZ binary magic */
#define	CFE_BISZ_OFFSET		0x3E0
#define	CFE_BISZ_MAGIC		0x4249535A	/* 'BISZ' */

/* SENTRY5 'config' partition magic (MINIX v1 filesystem, 30 char name limit) */
#define	CFE_MINIX_OFFSET	0x410
#define	CFE_MINIX_MAGIC		0x138F

/* GZIP magic */
#define	G_CFE_GZIP_MAGIC0	0x1f
#define	G_CFE_GZIP_MAGIC1	0x8b
#define	G_CFE_GZIP_DEFLATE	8

/* CFE bootblock magic */
#define	G_CFE_BOOTBLK_OFFSET	472	/**< boot block offset */
#define	G_CFE_BOOTBLK_BLKSIZE	512	/**< boot block alignment */
#define	G_CFE_BOOTBLK_MAX	16	/**< maximum block number to search */
#define	G_CFE_BOOTBLK_MAGIC	((uint64_t)0x43465631424f4f54ULL)

/* TRX magic */
#define	G_CFE_TRX_MAGIC		0x30524448	/* "HDR0" */
#define	G_CFE_TRX_V1		1
#define	G_CFE_TRX_V2		2
#define	G_CFE_TRX_V1_MAX_PARTS	3
#define	G_CFE_TRX_V2_MAX_PARTS	5
#define	G_CFE_TRX_MAX_PARTS	G_CFE_TRX_V2_MAX_PARTS

#define	G_CFE_IMG_MAX		2	/**< maximum CFE OS image count */

/**
 * GEOM BCM_CFE instance state
 */
struct g_cfe_softc {
};

/**
 * CFE boot image layout types.
 */
typedef enum {
	G_CFE_IMAGE_FAILSAFE,	/**< CFE with FAILSAFE_UPGRADE enabled */
	G_CFE_IMAGE_DUAL,	/**< CFE with DUAL_IMAGE enabled */
	G_CFE_IMAGE_SIMPLE	/**< CFE with default config (single image) */
} g_cfe_bootimg_type;

/**
 * CFE boot image info.
 */
struct g_cfe_bootimg_info {
	g_cfe_bootimg_type	type;			/**< CFE boot image type */
	uint8_t			bootimage;		/**< boot image index */
	size_t			num_images;		/**< image count */
	off_t			offsets[G_CFE_IMG_MAX];	/**< image offsets */
	off_t			sizes[G_CFE_IMG_MAX];	/**< image sizes */
};

/** CFE TRX image header */
struct g_cfe_trx_header {                                                                                                                 
	uint32_t magic;
	uint32_t len;
	uint32_t crc32;
	uint32_t flag_version;
	uint32_t offsets[G_CFE_TRX_MAX_PARTS];
} __packed;

/**
 * GEOM BCM_CFE device taste context
 */
struct g_cfe_taste_io {
	struct g_consumer		*cp;		/**< GEOM consumer (borrowed reference) */
	struct bcm_cfe_disk		*disk;		/**< disk entry (borrowed reference) */
	struct g_cfe_bootimg_info	 bootimg;	/**< boot image layout */
	off_t				 palign;	/**< minimum partition alignment */
	u_char				*buf;		/**< last read sector(s), or NULL */
	off_t				 buf_off;	/**< offset of last read */
	off_t				 buf_len;	/**< length of last read */
};

/**
 * GEOM BCM_CFE partition probe function.
 * 
 * @param io	The I/O context to be used for reading.
 * @param block	The block offset to be probed.
 * 
 * @retval non-NULL	the identified partition
 * @retval NULL		no match
 */
typedef struct bcm_cfe_part *(g_cfe_part_probe)(struct g_cfe_taste_io *io,
						    off_t block);

/**
 * GEOM BCM_CFE partition probe description.
 */
struct g_cfe_part_probe_info {
	const char		*name;	/**< human-readable description */
	g_cfe_part_probe	*func;	/**< probe function */
};

SET_DECLARE(g_cfe_part_probe_set, struct g_cfe_part_probe_info);

#define	G_CFE_DEFINE_PART_PROBE(_name, _func)				\
	static const struct g_cfe_part_probe_info _func ## _info = {	\
		.name	= (_name),					\
		.func	= (_func),					\
	};								\
	DATA_SET(g_cfe_part_probe_set, _func ## _info)

/**
 * GEOM-backed bhnd_nvram_io implementation, for use when probing an NVRAM
 * partition.
 */
struct g_cfe_nvram_io {
	struct bhnd_nvram_io	 io;		/**< common I/O instance state */
	struct g_cfe_taste_io	*taste;		/**< GEOM I/O state */
	off_t			 offset;	/**< base I/O offset */
	off_t			 size;		/**< number of bytes readable at base I/O offset */
};


#endif /* _MIPS_BROADCOM_BCM_GEOM_CFE_H_ */
