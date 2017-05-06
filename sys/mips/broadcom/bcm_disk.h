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

#ifndef	_MIPS_BROADCOM_BCM_DISK_H_
#define	_MIPS_BROADCOM_BCM_DISK_H_

#include <sys/param.h>
#include <sys/queue.h>

#define	BCM_DISK_UNIT_MAX	64		/**< maximum CFE device unit */
#define	BCM_DISK_NAME_MAX	64		/**< maximum CFE device name length */

#define	BCM_PART_ALIGN_MIN	0x1000		/**< minimum partition alignment */

#define	BCM_DISK_INVALID_OFF	OFF_MAX
#define	BCM_DISK_INVALID_SIZE	((off_t)0)

SLIST_HEAD(bcm_parts, bcm_part);
SLIST_HEAD(bcm_disks, bcm_disk);

int		 bcm_probe_disks(struct bcm_disks *result);
struct bcm_disk	*bcm_disk_new(const char *drvname, u_int unit);
void		 bcm_disk_free(struct bcm_disk *disk);

void		 bcm_print_disk(struct bcm_disk *disk);
void		 bcm_print_disks(struct bcm_disks *disks);

struct bcm_disk	*bcm_find_disk(struct bcm_disks *disks, const char *drvname,
		     u_int unit);

off_t		 bcm_part_get_end(struct bcm_part *part);
off_t		 bcm_part_get_next(struct bcm_part *part, off_t align);

struct bcm_part	*bcm_parts_find(struct bcm_parts *parts, const char *label);
struct bcm_part	*bcm_parts_find_offset(struct bcm_parts *parts, off_t offset);
struct bcm_part	*bcm_parts_match(struct bcm_parts *parts, const char *label,
		     off_t offset);

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

/**
 * Partition description.
 */
struct bcm_part {
	char			*devname;	/**< CFE device name (e.g. 'nflash0.boot') */
	const char		*label;		/**< CFE partition label */
	int			 fd;		/**< CFE handle, or -1 if unopened */
	bool			 need_close;	/**< If the fd should be closed on free */
	off_t			 offset;	/**< partition offset, or BCM_DISK_INVALID_OFF if unknown */
	off_t			 size;		/**< partition size, or BCM_DISK_INVALID_SIZE if unknown */
	off_t			 fs_size;	/**< the size of the filesystem, or BCM_DISK_INVALID_SIZE if unknown.
						     may be used to determine the minimum partition size required */

	SLIST_ENTRY(bcm_part) cp_link;
};

/** Evaluates to true if @p _part has a valid offset, false otherwise */
#define	BCM_PART_HAS_OFFSET(_part)	\
    ((_part)->offset != BCM_DISK_INVALID_OFF)

/** Evaluates to true if @p _part has a valid size, false otherwise */
#define	BCM_PART_HAS_SIZE(_part)	\
    ((_part)->size != BCM_DISK_INVALID_SIZE)
    
/** Evaluates to true if @p _part has a valid filesystem size, false otherwise */
#define	BCM_PART_HAS_FS_SIZE(_part)	\
    ((_part)->fs_size != BCM_DISK_INVALID_SIZE)

/**
 * Block device description.
 */
struct bcm_disk {
	const char		*drvname;	/**< CFE driver class name (e.g. 'nflash') */
	u_int			 unit;		/**< CFE device unit */
	off_t			 size;		/**< media size, or BCM_DISK_INVALID_SIZE if unknown */
	struct bcm_parts	 parts;		/**< identified partitions */
	size_t			 num_parts;	/**< partition count */

	SLIST_ENTRY(bcm_disk) cd_link;
};

/** Evaluates to true if @p _disk has a valid size, false otherwise */
#define	BCM_DISK_HAS_SIZE(_part)	\
    ((_part)->size != BCM_DISK_INVALID_SIZE)

/**
 * Known partition types.
 */
typedef enum {
	BCM_PART_TYPE_OS,	/**< os partition */
	BCM_PART_TYPE_TRX,	/**< trx partition */
	BCM_PART_TYPE_UNKNOWN	/**< other/unknown */
} bcm_part_type;

#endif /* _MIPS_BROADCOM_BCM_DISK_H_ */
