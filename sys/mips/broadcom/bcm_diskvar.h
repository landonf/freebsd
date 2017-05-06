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

#ifndef	_MIPS_BROADCOM_BCM_DISKVAR_H_
#define	_MIPS_BROADCOM_BCM_DISKVAR_H_

#include <sys/types.h>
#include <sys/malloc.h>

#include "bcm_disk.h"

MALLOC_DECLARE(M_BCM_CDISK);

extern const bool bcm_disk_trace;

/* forward declarations */
struct bcm_boot_label;

bool		 bcm_disk_dev_exists(struct bcm_disk *disk, const char *label);
int		 bcm_disk_dev_name(const char *drvname, u_int unit,
		     const char *label, char *buf, size_t *len);

bool		 bcm_find_boot_label(const char *label,
		     const struct bcm_boot_label **info,
		     bcm_part_type *part_type);

struct bcm_part	*bcm_disk_get_query_part(struct bcm_disk *disk);

/* TRX image constants */
#define	BCM_TRX_MAGIC		0x30524448	/* "HDR0" */
#define	BCM_TRX_V1		1
#define	BCM_TRX_V2		2
#define	BCM_TRX_V1_MAX_PARTS	3
#define	BCM_TRX_V2_MAX_PARTS	5
#define	BCM_TRX_MAX_PARTS	BCM_TRX_V2_MAX_PARTS

/** TRX image header */
struct bcm_trx_header {                                                                                                                 
	uint32_t magic;
	uint32_t len;
	uint32_t crc32;
	uint32_t flag_version;
	uint32_t offsets[BCM_TRX_MAX_PARTS];
} __packed;


/**
 * CFE OS/TRX boot partition labels.
 * 
 * Provides a mapping between TRX partition labels and corresponding OS
 * partition label.
 */
struct bcm_boot_label {
	const char *os_label;	/**< OS partition label */
	const char *trx_label;	/**< TRX partition label */
};

#define	BCM_DISK_UNIT_MAX	64		/**< maximum CFE device unit */
#define	BCM_DISK_NAME_MAX	64		/**< maximum CFE device name length */
#define	BCM_PART_ALIGN_MIN	0x1000		/**< minimum partition alignment */

/* CFE binary magic */
#define	BCM_CFE_MAGIC		0x43464531	/**< 'CFE1' */
#define	BCM_CFE_MAGIC_OFFSET	0x4E0		/**< CFE magic offset */

/* Self-describing compressed CFEZ binary magic */
#define	BCM_CFE_BISZ_OFFSET	0x3E0
#define	BCM_CFE_BISZ_MAGIC	0x4249535A	/* 'BISZ' */

/* SENTRY5 'config' partition magic (MINIX v1 filesystem, 30 char name limit) */
#define	BCM_MINIX_OFFSET	0x410
#define	BCM_MINIX_MAGIC		0x138F

/* GZIP magic */
#define	BCM_GZIP_MAGIC0		0x1f
#define	BCM_GZIP_MAGIC1		0x8b
#define	BCM_GZIP_DEFLATE	8

/* CFE bootblock magic */
#define	BCM_BOOTBLK_OFFSET	472	/**< boot block offset */
#define	BCM_BOOTBLK_BLKSIZE	512	/**< boot block alignment */
#define	BCM_BOOTBLK_MAX		16	/**< maximum number of blocks to search */
#define	BCM_BOOTBLK_MAGIC	((uint64_t)0x43465631424f4f54ULL)

/* Disk logging */
#define	BCM_DISK_LOG(_disk, msg, ...)					\
	printf("%s%u: " msg, (_disk)->drvname, (_disk)->unit, ## __VA_ARGS__)

#define	BCM_DISK_ERR(_disk, msg, ...)					\
	printf("%s(%s%u): " msg, __FUNCTION__, (_disk)->drvname,	\
	    (_disk)->unit, ## __VA_ARGS__)

#define	BCM_DISK_TRACE(_disk, msg, ...)	do {				\
	if (bcm_disk_trace)						\
		BCM_DISK_ERR((_disk), msg, ## __VA_ARGS__);		\
} while(0)

/* Partition logging */
#define	BCM_PART_LOG(_part, msg, ...)					\
	printf("%s: " msg, (_part)->devname, ## __VA_ARGS__)

#define	BCM_PART_ERR(_part, msg, ...)					\
	printf("%s(%s): " msg, __FUNCTION__, (_part)->devname,  ## __VA_ARGS__)

#define	BCM_PART_TRACE(_part, msg, ...)	do {				\
	if (bcm_disk_trace)						\
		BCM_PART_ERR((_part), msg, ## __VA_ARGS__);		\
} while(0)


/**
 * CFE flash device driver quirks.
 */
enum {
	/** No quirks */
	BCM_CFE_QUIRK_NONE		= 0,

	/** IOCTL_FLASH_GETINFO always returns an invalid offset */
	BCM_CFE_QUIRK_FLASH_INV_OFF	= (1<<1),

	/** IOCTL_FLASH_GETINFO always returns an invalid size */
	BCM_CFE_QUIRK_FLASH_INV_SIZE	= (1<<2),

	/* IOCTL_NVRAM_GETINFO (incorrectly) returns the size of the actual
	 * partition, and may be used to determine partition size. */
	BCM_CFE_QUIRK_NVRAM_PART_SIZE	= (1<<3),

	/** IOCTL_NVRAM_GETINFO is not supported */
	BCM_CFE_QUIRK_NVRAM_UNAVAIL	= (1<<4),

	/** IOCTL_FLASH_GETINFO always returns an offset of 0x0 */
	BCM_CFE_QUIRK_FLASH_ZERO_OFF	= (1<<5) | BCM_CFE_QUIRK_FLASH_INV_OFF,

	/** IOCTL_FLASH_GETINFO returns the physical flash base as the partition
	 *  offset */
	BCM_CFE_QUIRK_FLASH_PHYS_OFF	= (1<<6) | BCM_CFE_QUIRK_FLASH_INV_OFF,

	/** IOCTL_FLASH_GETINFO always returns the total flash size (not
	  * the size of the actual partition) */
	BCM_CFE_QUIRK_FLASH_TOTAL_SIZE	= (1<<7) | BCM_CFE_QUIRK_FLASH_INV_SIZE,

	/**
	 * Reading at an offset past the partition's end will trigger a driver
	 * bug.
	 */
	BCM_CFE_QUIRK_PART_EOF_CRASH	= (1<<8),

	/**
	 * Reading an offset+length range that extends past the partition's end
	 * will return an IOERR, rather than performing a read over the
	 * available bytes.
	 */
	BCM_CFE_QUIRK_PART_EOF_IOERR	= (1<<9),

	/**
	 * Reading past the partition end will succeed, returning bytes from
	 * any subsequent partition until the actual end of the device is
	 * reached.
	 */
	BCM_CFE_QUIRK_PART_EOF_OVERREAD	= (1<<10)
};

#define	BCM_DRV_QUIRK(_quirks, _cfe_quirk)		\
	(((_quirks) & BCM_CFE_QUIRK_ ## _cfe_quirk) ==	\
	    BCM_CFE_QUIRK_ ## _cfe_quirk)

#endif /* _MIPS_BROADCOM_BCM_DISKVAR_H_ */
