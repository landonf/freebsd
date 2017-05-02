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

#ifndef	_MIPS_BROADCOM_BCM_DISK_CFE_H_
#define	_MIPS_BROADCOM_BCM_DISK_CFE_H_

#include <sys/param.h>
#include <sys/queue.h>

#define	BCM_CFE_DUNIT_MAX	64		/**< maximum CFE device unit */
#define	BCM_CFE_DNAME_MAX	64		/**< maximum CFE device name length */

#define	BCM_CFE_PALIGN_MIN	0x1000		/**< minimum partition alignment */

#define	BCM_CFE_INVALID_OFF	OFF_MAX
#define	BCM_CFE_INVALID_SIZE	((off_t)0)

SLIST_HEAD(bcm_cfe_parts, bcm_cfe_part);
SLIST_HEAD(bcm_cfe_disks, bcm_cfe_disk);

int			 bcm_cfe_probe_disks(struct bcm_cfe_disks *result);
struct bcm_cfe_disk	*bcm_cfe_disk_new(const char *drvname, u_int unit);
void			 bcm_cfe_disk_free(struct bcm_cfe_disk *disk);

/**
 * CFE flash device driver quirks.
 */
enum {
	/** No quirks */
	BCM_CFE_DRV_QUIRK_NONE			= 0,

	/** IOCTL_FLASH_GETINFO always returns an invalid offset */
	BCM_CFE_DRV_QUIRK_FLASH_INV_OFF		= (1<<1),

	/** IOCTL_FLASH_GETINFO always returns an invalid size */
	BCM_CFE_DRV_QUIRK_FLASH_INV_SIZE	= (1<<2),

	/* IOCTL_NVRAM_GETINFO (incorrectly) returns the size of the actual
	 * partition, and may be used to determine partition size. */
	BCM_CFE_DRV_QUIRK_NVRAM_PART_SIZE	= (1<<3),

	/** IOCTL_NVRAM_GETINFO is not supported */
	BCM_CFE_DRV_QUIRK_NVRAM_UNAVAIL		= (1<<4),

	/** IOCTL_FLASH_GETINFO always returns an offset of 0x0 */
	BCM_CFE_DRV_QUIRK_FLASH_ZERO_OFF	= (1<<5) | BCM_CFE_DRV_QUIRK_FLASH_INV_OFF,

	/** IOCTL_FLASH_GETINFO returns the physical flash base as the partition
	 *  offset */
	BCM_CFE_DRV_QUIRK_FLASH_PHYS_OFF	= (1<<6) | BCM_CFE_DRV_QUIRK_FLASH_INV_OFF,

	/** IOCTL_FLASH_GETINFO always returns the total flash size (not
	  * the size of the actual partition) */
	BCM_CFE_DRV_QUIRK_FLASH_TOTAL_SIZE	= (1<<7) | BCM_CFE_DRV_QUIRK_FLASH_INV_SIZE,

	/**
	 * Reading at an offset past the partition's end will trigger a driver
	 * bug.
	 */
	BCM_CFE_DRV_QUIRK_READBLK_EOF_CRASH	= (1<<8),

	/**
	 * Reading an offset+length range that extends past the partition's end
	 * will return an IOERR, rather than performing a read over the
	 * available bytes.
	 */
	BCM_CFE_DRV_QUIRK_READBLK_EOF_IOERR	= (1<<9),
};

#define	BCM_CFE_DRV_QUIRK(_quirks, _cfe_quirk)			\
	(((_quirks) & BCM_CFE_DRV_QUIRK_ ## _cfe_quirk) ==	\
	    BCM_CFE_DRV_QUIRK_ ## _cfe_quirk)

/**
 * CFE-probed partition description.
 */
struct bcm_cfe_part {
	struct bcm_cfe_disk	*disk;		/**< borrowed reference to parent disk */
	char			*devname;	/**< CFE device name (e.g. 'nflash0.boot') */
	const char		*label;		/**< CFE partition label */
	int			 fd;		/**< CFE handle, or -1 if unopened */
	bool			 need_close;	/**< If the fd should be closed on free */
	off_t			 offset;	/**< partition offset, or BCM_CFE_INVALID_OFF if unknown */
	off_t			 size;		/**< partition size, or BCM_CFE_INVALID_SIZE if unknown */

	SLIST_ENTRY(bcm_cfe_part) cp_link;
};

/**
 * CFE-probed block device description.
 */
struct bcm_cfe_disk {
	const char		*drvname;	/**< CFE driver class name (e.g. 'nflash') */
	u_int			 unit;		/**< CFE device unit */
	off_t			 size;		/**< media size, or BCM_CFE_INVALID_SIZE if unknown */
	struct bcm_cfe_parts	 parts;		/**< identified partitions */

	SLIST_ENTRY(bcm_cfe_disk) cd_link;
};

#endif /* _MIPS_BROADCOM_BCM_DISK_CFE_H_ */
