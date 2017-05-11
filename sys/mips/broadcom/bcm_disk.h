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

#define	BCM_DISK_INVALID_OFF	OFF_MAX
#define	BCM_DISK_INVALID_SIZE	((off_t)0)

SLIST_HEAD(bcm_parts, bcm_part);
SLIST_HEAD(bcm_disks, bcm_disk);


/**
 * Known partition types.
 */
typedef enum {
	/* Common CFE partition types */
	BCM_PART_TYPE_BOOT,		/**< CFE boot partition */
	BCM_PART_TYPE_OS,		/**< OS partition (contains OS bootloader at offset 0x0,
					  *  or a CFE bootblock) */
	BCM_PART_TYPE_TRX,		/**< TRX partition */
	BCM_PART_TYPE_NVRAM,		/**< NVRAM partition */
	BCM_PART_TYPE_BRCMNAND,		/**< unused NAND space reserved for OS data */

	/* Netgear-specific partition types */
	BCM_PART_TYPE_LEAF_CONFIG,	/**< Netgear/LEAF configuration partition (MINIX partition, found on WGT634U) */
	BCM_PART_TYPE_BOARD_DATA,	/**< Netgear 'board_data' partition */
	BCM_PART_TYPE_MULTILANG,	/**< Netgear 'ML' l10n string table partition */
	BCM_PART_TYPE_POT,		/**< Netgear 'POT' partition: NTP timestamp + first associated STA MAC addr */
	BCM_PART_TYPE_TMETER,		/**< Netgear 'T_Meter' traffic meter data partition */

	/* Linksys-specific partition types */
	BCM_PART_TYPE_DEVINFO,		/**< Linksys 'devinfo' partition */

	/* Asus-specific partition types */
	BCM_PART_TYPE_ASUSFW,		/**< Asus 'asus' partition containing device firmware, etc */

	BCM_PART_TYPE_UNKNOWN		/**< other/unknown */
} bcm_part_type;

/**
 * CFE disk flags
 */
enum {
	BCM_DISK_BOOTROM	= (1<<0),	/**< provides platform bootloader, NVRAM, etc. */
	BCM_DISK_OSDEV		= (1<<1),	/**< disk contains OS loader/kernel */
	BCM_DISK_BYTESWAPPED	= (1<<2),	/**< a hint that target-endian data structures
						     are not in host byte order */
};

/**
 * CFE partition flags
 */
enum {
	BCM_PART_READONLY		= (1<<0),	/**< partition contains critical data and must be treated as read-only */
	BCM_PART_PLATFORM		= (1<<1),	/**< partition is required for device function and must be preserved (but may be writable). */
	BCM_PART_UNINITIALIZED		= (1<<2),	/**< vendor-defined partition is uninitialized */
	BCM_PART_BOOTDEV		= (1<<3),	/**< partition is marked bootable */
	BCM_PART_NVRAM			= (1<<4),	/**< partition contains NVRAM-formatted data */

	BCM_PART_PLATFORM_RO		= (BCM_PART_PLATFORM|BCM_PART_READONLY),
	BCM_PART_PLATFORM_NVRAM		= (BCM_PART_NVRAM|BCM_PART_PLATFORM),
	BCM_PART_PLATFORM_NVRAM_RO	= (BCM_PART_NVRAM|BCM_PART_PLATFORM_RO),
};


int		 bcm_probe_disks(struct bcm_disks *result);
struct bcm_disk	*bcm_disk_new(const char *drvname, u_int unit, uint32_t flags);
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
		     off_t offset, bcm_part_type type);


/**
 * Partition description.
 */
struct bcm_part {
	char			*devname;	/**< CFE device name (e.g. 'nflash0.boot') */
	const char		*label;		/**< CFE partition label */
	int			 fd;		/**< CFE handle, or -1 if unopened */
	bool			 need_close;	/**< If the fd should be closed on free */
	bcm_part_type		 type;		/**< partition type (or BCM_PART_TYPE_UNKNOWN) */
	uint32_t		 flags;		/**< partition flags (see BCM_PART_* flag enums) */
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

/** Return (in the following preferred order) either the filesystem size of
 *  @p _part, the partition size, or OFF_MAX */
#define	BCM_PART_MAX_FS_SIZE(_part)				\
	(BCM_PART_HAS_FS_SIZE(_part) ? (_part)->fs_size :	\
	 BCM_PART_HAS_SIZE(_part) ? (_part)->size :		\
	 OFF_MAX)

/** Evaluates to true if all of @p _flags are set on @p _part */
#define	BCM_PART_HAS_FLAGS(_part, _flags)	\
	(((_part)->flags & (_flags)) == (_flags))

/**
 * Block device description.
 */
struct bcm_disk {
	const char		*drvname;	/**< CFE driver class name (e.g. 'nflash') */
	u_int			 unit;		/**< CFE device unit */
	uint32_t		 flags;		/**< disk flags (see BCM_DISK_* flag enums) */
	off_t			 size;		/**< media size, or BCM_DISK_INVALID_SIZE if unknown */
	struct bcm_parts	 parts;		/**< identified partitions */
	size_t			 num_parts;	/**< partition count */

	SLIST_ENTRY(bcm_disk) cd_link;
};

/** Evaluates to true if @p _disk has a valid size, false otherwise */
#define	BCM_DISK_HAS_SIZE(_part)	\
    ((_part)->size != BCM_DISK_INVALID_SIZE)

/** Evaluates to true if all of @p _flags are set on @p _disk */
#define	BCM_DISK_HAS_FLAGS(_disk, _flags)	\
	(((_disk)->flags & (_flags)) == (_flags))

#endif /* _MIPS_BROADCOM_BCM_DISK_H_ */
