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
#include <sys/md5.h>

#include "bcm_disk.h"

MALLOC_DECLARE(M_BCM_DISK);

extern const bool bcm_disk_trace;

/* forward declarations */
struct bcm_bootinfo;
struct bcm_devunit;

int		 bcm_disk_parse_devname(const char *devname,
		     const char **drvname, size_t *drvlen, u_int *devunit,
		     const char **label, size_t *labellen);

int		 bcm_disk_devname(const char *drvname, u_int unit,
		     const char *label, char *buf, size_t *len);

bool		 bcm_disk_has_devunit(struct bcm_disk *disk,
		     struct bcm_devunit *devunit);

void		 bcm_part_free(struct bcm_part *part);

int		 bcm_part_update_sizing(struct bcm_disk *disk,
		     struct bcm_part *part, struct bcm_part_size *psz);
int		 bcm_part_validate_sizing(struct bcm_disk *disk,
		     struct bcm_part *part, struct bcm_part_size *psz);

int		 bcm_get_bootinfo(struct bcm_bootinfo *bootinfo);

struct bcm_part	*bcm_disk_get_query_part(struct bcm_disk *disk);

/**
 * CFE boot image layouts
 */
typedef enum {
	BCM_BOOTIMG_FAILSAFE,	/**< CFE with FAILSAFE_UPGRADE enabled */
	BCM_BOOTIMG_DUAL,	/**< CFE with DUAL_IMAGE enabled */
	BCM_BOOTIMG_SIMPLE,	/**< CFE with default config (single image) */
} bcm_bootimg_layout;

/** Evaluates to true if @p _layout is a dual-image layout, false otherwise */
#define	BCM_BOOTIMG_LAYOUT_DUAL(_layout)	\
	((_layout == BCM_BOOTIMG_FAILSAFE || _layout == BCM_BOOTIMG_DUAL))

/**
 * CFE boot image configuration.
 */
struct bcm_bootimg {
	off_t		 offset;	/**< image offset, or BCM_DISK_INVALID_OFF if unavailable */
	off_t		 size;		/**< image size, or BCM_DISK_INVALID_SIZE if unavailable */
};

#define	BCM_DISK_BOOTIMG_FIRST	0		/**< first boot image index */
#define	BCM_DISK_BOOTIMG_SECOND	1		/**< second boot image index */
#define	BCM_DISK_BOOTIMG_MAX	2		/**< maximum CFE boot image count */

/**
 * CFE device class/unit descriptor.
 */
struct bcm_devunit {
	const char	*drvname;	/**< CFE driver class name */
	u_int		 unit;		/**< CFE device unit */
};

/**
 * CFE boot configuration.
 */
struct bcm_bootinfo {
	struct bcm_devunit	 romdev;			/**< bootrom (+nvram) device */
	struct bcm_devunit	 osdev;				/**< OS boot device */
	bcm_bootimg_layout	 layout;			/**< OS boot image layout */
	uint8_t			 bootimg;			/**< active OS image index */
	struct bcm_bootimg	 images[BCM_DISK_BOOTIMG_MAX];	/**< OS boot images */
	size_t			 num_images;			/**< OS boot image count */
	uint32_t		 num_failures;			/**< failed boot count (if BCM_BOOTIMG_FAILSAFE) */
	uint32_t		 max_failures;			/**< maximum failed boot count (if BCM_BOOTIMG_FAILSAFE) */
};

/**
 * CFE partition sizing function.
 * 
 * Used to determine the size and/or offset of a CFE partition.
 */
typedef int (bcm_part_size_fn)(struct bcm_disk *disk, struct bcm_part *part,
				    struct bcm_part_size *size);

/**
 * CFE partition fingerprint function.
 * 
 * Produces a fingerprint (and optional sizing information) that may be used to
 * identify the partition's location on disk when a valid offset is unavailable.
 */
typedef int (bcm_part_ident_fn)(struct bcm_disk *disk, struct bcm_part *part,
				    struct bcm_part_ident *ident,
				    struct bcm_part_size *size);

/**
 * CFE partition fingerprint function registration.
 */
struct bcm_part_ident_info {
	const char		*name;	/**< human-readable description */
	bcm_part_type		 type;	/**< applicable partition type */
	bcm_part_ident_fn	*func;	/**< fingerprint function */
};

SET_DECLARE(bcm_part_ident_set, struct bcm_part_ident_info);

#define	BCM_PART_IDENT(_name, _type, _func)				\
	static const struct bcm_part_ident_info _func ## _info = {	\
		.name	= (_name),					\
		.type	= (_type),					\
		.func	= (_func),					\
	};								\
	DATA_SET(bcm_part_ident_set, _func ## _info)


#define	BCM_DRVNAME_NAND_FLASH	"nflash"	/**< NAND flash driver class */
#define	BCM_DRVNAME_NOR_FLASH	"flash"		/**< NOR flash driver class */

#define	BCM_DISK_UNIT_MAX	64		/**< maximum CFE device unit */
#define	BCM_DISK_NAME_MAX	64		/**< maximum CFE device name length */
#define	BCM_DISK_BOOTROM_UNIT	0		/**< bootrom device is always found on unit 0 */
#define	BCM_DISK_OS_UNIT	0		/**< OS boot device is always found on unit 0 */

#define	BCM_PART_ALIGN_MIN	0x1000		/**< minimum partition alignment */

#define	BCM_PART_LABEL_OS	"os"		/**< first OS boot partition label */
#define	BCM_PART_LABEL_OS2	"os2"		/**< second OS boot partition label */
#define	BCM_PART_LABEL_TRX	"trx"		/**< first TRX boot partition label */
#define	BCM_PART_LABEL_TRX2	"trx2"		/**< second TRX boot partition label */

#define	BCM_DISK_CFE_DEVTYPE_SUPPORTED(_type)	\
	((_type) == CFE_DEV_FLASH ||		\
	 (_type) == CFE_DEV_NVRAM)

/* CFE binary magic */
#define	BCM_CFE_MAGIC		0x43464531	/**< 'CFE1' */
#define	BCM_CFE_CIGAM		0x31454643
#define	BCM_CFE_MAGIC_OFFSET	0x4E0		/**< CFE magic offset */

/* Self-describing CFE binary magic */
#define	BCM_CFE_BISZ_OFFSET	0x3E0
#define	BCM_CFE_BISZ_MAGIC	0x4249535A	/* 'BISZ' */
#define	BCM_CFE_BISZ_CIGAM	0x5A534942	/* 'BISZ' */
#define	BCM_CFE_BISZ_MINSIZE	(128*1024)	/**< minimum partition size (128KB) */
#define	BCM_CFE_BISZ_NOFFS	2
#define	BCM_CFE_BISZ_ST_IDX	0
#define	BCM_CFE_BISZ_END_IDX	1


struct bcm_cfez_header {
	uint32_t	magic;		/**< magic (BCM_CFE_BISZ_MAGIC) */
	uint32_t	txt[BCM_CFE_BISZ_NOFFS];		/**< text section start/end offsets */
	uint32_t	data[BCM_CFE_BISZ_NOFFS];	/**< data section start/end offsets */
	uint32_t	bss[BCM_CFE_BISZ_NOFFS];		/**< bss section start/end offsets */
} __packed;

/* SENTRY5 'config' partition magic (MINIX v1 filesystem, 30 char name limit) */
#define	BCM_MINIX_OFFSET	0x410
#define	BCM_MINIX_MAGIC		0x138F
#define	BCM_MINIX_CIGAM		0x8F13

/* GZIP magic */
#define	BCM_GZIP_MAGIC0		0x1f
#define	BCM_GZIP_MAGIC1		0x8b
#define	BCM_GZIP_DEFLATE	8

/* BZIP2 magic */
#define	BCM_BZIP2_MAGIC		"BZ"
#define	BCM_BZIP2_MAGIC_LEN	2

/* CFE bootblock magic */
#define	BCM_BOOTBLK_MAX		16	/**< maximum number of boot block offsets to search */
#define	BCM_BOOTBLK_MAGIC	((uint64_t)0x43465631424F4F54ULL)
#define	BCM_BOOTBLK_CIGAM	((uint64_t)0x544F4F4231564643ULL)

struct bcm_cfe_bootblk {
	uint8_t		disklabel[472];	/**< OS-managed partition map */
	uint64_t	magic;		/**< magic */
	uint8_t		flags;		/**< flags (unused, zero-initialized) */
	uint16_t	reserved0;	/**< reserved for future use (zero-initialized) */
	uint8_t		version;	/**< version */
	uint64_t	ldr_offset;	/**< boot loader offset */
	uint32_t	ldr_checksum;	/**< boot loader checksum */
	uint32_t	ldr_size;	/**< boot loader size */
	uint32_t	reserved1;	/**< reserved for future use (zero-initialized) */
	uint32_t	arch;		/**< arch-specific information (unused, zero-initialized) */
} __packed;

/* TRX image header */
#define	BCM_TRX_MAGIC		0x30524448
#define	BCM_TRX_CIGAM		0x48445230
#define	BCM_TRX_V1		1
#define	BCM_TRX_V2		2
#define	BCM_TRX_V1_MAX_PARTS	3
#define	BCM_TRX_V2_MAX_PARTS	5
#define	BCM_TRX_MAX_PARTS	BCM_TRX_V2_MAX_PARTS

struct bcm_trx_header {                                                                                                                 
	uint32_t	magic;
	uint32_t	len;
	uint32_t	crc32;
	uint16_t 	flags;
	uint16_t	version;
	uint32_t	offsets[BCM_TRX_MAX_PARTS];
} __packed;

/* Netgear ML (multi-language) partition */
#define	BCM_NETGEAR_LANG_MAXLEN		0xFFF0
#define	BCM_NETGEAR_LANG_HDRSIZE	\
	offsetof(struct bcm_netgear_langhdr, bzip2)

struct bcm_netgear_langhdr {
	uint32_t	len;				/**< data length (not including header) */
	uint32_t	unknown;			/**< ??? */
	uint8_t		version[8];			/**< version (XX.XX.XX.XX_XX.XX.XX.XX) */
	uint8_t		bzip2[BCM_BZIP2_MAGIC_LEN];	/**< BZIP stream magic */
} __packed;

/* Netgear POT partition */
#define	BCM_POT_MAGIC		"POTTOP"
#define	BCM_POT_MAGIC_LEN	(sizeof(BCM_POT_MAGIC)-1)
#define	BCM_POT_MAGIC_OFFSET	0x0

/* Netgear T_Meter partition */
#define	BCM_TMETER_MAGIC	0x544D
#define	BCM_TMETER_CIGAM	0x4D54

struct bcm_tmeter_record {
	uint16_t	magic;
	uint16_t	checksum;
	uint8_t		data[20];
} __packed;

/* Netgear board_data partition */
#define	BCM_BD_RFPM_OFFSET	0x100		/**< RF parameter block offset */
#define	BCM_BD_RFPM_MAGIC	0x5246504D	/**< RF parameter block magic ('RFPM') */
#define	BCM_BD_RFPM_CIGAM	0x4D504652
#define	BCM_BD_RECORD_MAGIC	0xBD0D0BBD	/**< record magic */
#define	BCM_BD_RECORD_CIGAM	0xBD0B0DBD
#define	BCM_BD_RECORD_MAGIC0	0xBD
#define	BCM_BD_RECORD_OFFSET	0x8000		/**< initial offset of board data records */
#define	BCM_BD_RECORD_MAXLEN	\
	UINT16_MAX - offsetof(struct bcm_bd_record, tag)

struct bcm_bd_record {
	uint32_t	magic;	/**< record magic */
	uint16_t	tag;	/**< record tag */
	uint16_t	len;	/**< record length (not including magic) */
} __packed;

/**
 * CFE flash device driver quirks.
 */
enum {
	/** Uninitialized quirk value */
	BCM_CFE_QUIRK_INVALID		= 0,

	/** No quirks; used to differentiate from an uninitialized value */
	BCM_CFE_QUIRK_NONE		= (1<<0),

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

#ifdef INVARIANTS
static inline uint32_t
bcm_disk_get_quirks(struct bcm_disk *disk)
{
	KASSERT(disk->quirks != BCM_CFE_QUIRK_INVALID, ("quirks not probed"));
	return (disk->quirks);
}
#else /* !INVARIANTS */
#define	bcm_disk_get_quirks(_disk)	((_disk)->quirks)
#endif /* INVARIANTS */

#define	BCM_DISK_QUIRK(_disk, _cfe_quirk)				\
	((bcm_disk_get_quirks(_disk) & BCM_CFE_QUIRK_ ## _cfe_quirk) ==	\
	    BCM_CFE_QUIRK_ ## _cfe_quirk)

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

#endif /* _MIPS_BROADCOM_BCM_DISKVAR_H_ */
