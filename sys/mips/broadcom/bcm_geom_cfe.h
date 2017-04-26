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

#include <sys/queue.h>

#define	CFE_CLASS_NAME		"BCM_CFE"

/* CFE binary magic */
#define	CFE_MAGIC		0x43464531	/**< 'CFE1' */
#define	CFE_MAGIC_OFFSET	0x4E0		/**< CFE magic offset */
#define	CFE_MAGIC_COUNT		2		/**< CFE magic count */
#define	CFE_MAGIC_0		0		/**< 1st CFE magic constant */
#define	CFE_MAGIC_1		1		/**< 2nd CFE magic constant */

/* SENTRY5 'config' partition magic (MINIX v1 filesystem, 30 char name limit) */
#define	CFE_MINIX_OFFSET	0x410
#define	CFE_MINIX_MAGIC		0x138F

/* Self-describing compressed CFEZ binary magic */
#define	CFE_BISZ_OFFSET		0x3E0
#define	CFE_BISZ_MAGIC		0x4249535A	/* 'BISZ' */

#define	CFE_IMG_MAX		2		/**< maximum CFE OS image count */
#define	CFE_DUNIT_MAX		64		/**< maximum CFE device unit */
#define	CFE_DNAME_MAX		64		/**< maximum CFE device name */

struct cfe_flash_probe;

/**
 * GEOM BCM_CFE instance state
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
	uint32_t		 cfe_quirks;	/**< CFE driver quirks (see CFE_DEV_QUIRK_*) */
};

/**
 * GEOM flash partition probing bhnd_nvram_io implementation.
 * 
 * Provides GEOM-backed I/O mapping of a probed flash partition.
 */
struct g_cfe_probe_io {
	struct bhnd_nvram_io	 io;		/**< common I/O instance state */
	struct cfe_flash_probe	*probe;		/**< partition probe state (borrowed) */
	void			*last;		/**< last read sector(s), or NULL */
	off_t			 last_off;	/**< offset of last read */
	off_t			 last_len;	/**< length of last read */
};

/**
 * CFE flash device probe state.
 */
struct cfe_flash_probe {
	struct g_consumer		*cp;			/**< GEOM flash consumer */
	const struct cfe_flash_device	*dev;			/**< CFE flash device mapping descriptor */
	char				 dname[CFE_DNAME_MAX];	/**< CFE device name */
	char				 pname[CFE_DNAME_MAX];	/**< CFE partition name */
	u_int				 unit;			/**< CFE flash unit (e.g flash1) */
	int				 fd;			/**< CFE device handle, or -1 if device unavailable */
	off_t				 mediasize;		/**< media size, in bytes */
	u_int				 blksize;		/**< media block size, in bytes */

	off_t				 offset;		/**< partition offset (if known) */
	off_t				 size;			/**< partition size (if known) */
	bool				 have_offset;		/**< partition offset has been determined */
	bool				 have_size;		/**< partition size has been determined */

	TAILQ_ENTRY(cfe_flash_probe)	 fp_link;
};

/** GEOM CFE flash probe list */
TAILQ_HEAD(g_cfe_flash_probe_list, cfe_flash_probe);

/** GEOM CFE flash probe function */
typedef int (g_cfe_probe_func)(struct cfe_flash_probe *,
			           struct g_cfe_flash_probe_list *probes);

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
	uint64_t		offsets[CFE_IMG_MAX];	/**< image offsets */
	uint64_t		sizes[CFE_IMG_MAX];	/**< image sizes */
};

/**
 * CFE flash device driver quirks.
 */
enum {
	/** No quirks */
	G_CFE_QUIRK_NONE			= 0,

	/** IOCTL_FLASH_GETINFO always returns an invalid offset */
	G_CFE_QUIRK_FLASH_INV_OFF	= (1<<1),

	/** IOCTL_FLASH_GETINFO always returns an invalid size */
	G_CFE_QUIRK_FLASH_INV_SIZE	= (1<<2),

	/* IOCTL_NVRAM_GETINFO (incorrectly) returns the size of the actual
	 * partition, and may be used to determine partition size. */
	G_CFE_QUIRK_NVRAM_PART_SIZE	= (1<<3),

	/** IOCTL_NVRAM_GETINFO is not supported */
	G_CFE_QUIRK_NVRAM_UNAVAIL	= (1<<4),

	/** IOCTL_FLASH_GETINFO always returns an offset of 0x0 */
	G_CFE_QUIRK_FLASH_ZERO_OFF	= (1<<5) | G_CFE_QUIRK_FLASH_INV_OFF,

	/** IOCTL_FLASH_GETINFO always returns the total flash size (not
	  * the size of the actual partition) */
	G_CFE_QUIRK_FLASH_TOTAL_SIZE	= (1<<6) | G_CFE_QUIRK_FLASH_INV_SIZE,
};

#define	G_CFE_QUIRK(_cfe_dev, _cfe_quirk)		\
	(((_cfe_dev)->cfe_quirks & G_CFE_QUIRK_ ## _cfe_quirk) != 0)


#endif /* _MIPS_BROADCOM_BCM_GEOM_CFE_H_ */
