/*-
 * Copyright (c) 2017 Landon Fuller <landonf@FreeBSD.org>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <machine/_inttypes.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>
#include <dev/cfe/cfe_ioctl.h>

#include <dev/bhnd/cores/chipc/chipcreg.h>

#include "bcm_machdep.h"

#include "bcm_disk.h"

/*
 * Probes and saves CFE-defined disk/flash partition information.
 *
 * Since enumeration may require reading from the CFE flash devices, we
 * perform the probe during early boot, and save the results for later
 * consumption by our GEOM class, et al.
 * 
 * = Design Notes =
 * 
 * In theory, we should be able to fully enumerate the flash layout using
 * cfe_enumdev() and CFE device ioctls. In practice, however, no OEMs appear
 * to ship a CFE release with CFE_CMD_DEV_ENUM support.
 *
 * Worse, the validity of the flash/nvram ioctls depends on the CFE flash
 * driver in use, requiring CFE driver-specific workarounds (see
 * 'CFE Driver Quirks' below).
 * 
 * As a result, we must:
 * 
 * - Probe a set of known CFE partition/device names.
 * - Fall back on conservative heuristics to determine partition offset/size.
 * - Require that our data consumers handle potentially missing/invalid layout
 *   data.
 * 
 * == Basic Flash Layout ==
 * 
 * On most Broadcom MIPS hardware, the flash layout will be some variation
 * of the following:
 * 
 *	[name]	[desc]			[offset+size]
 *	boot	CFE bootloader		0x0000000+0x4000000
 *	<optional device/vendor-specific partitions>
 *	trx	TRX image reservation	0x4000000+0x1fa0000
 *	<optional device/vendor-specific partitions>
 *	nvram	Broadcom NVRAM		0x1ff8000+0x8000
 *
 * - The 'boot' partition is always found at offset 0x0 -- the flash device
 *   is mapped to the MIPS boot exception vector address (0x1FC00000), and the
 *   CFE partition provides the bootcode that will begin executing in-place at
 *   0x1FC00000+0x0.
 * 
 * - The 'trx' partition will be sized by CFE to include all flash space
 *   reserved for OS use, even if the TRX image written to flash is smaller
 *   than the TRX partition size; this can be used to safely determine the
 *   usable flash range without stomping on any vendor data that may be stored
 *   in flash.
 *
 *   When booting the OS, CFE ignores the offsets specified in the TRX header,
 *   and instead assumes either a CFE boot block or a raw boot loader is
 *   located directly following the TRX header.
 * 
 * - Some earlier devices do not support TRX, and instead, a single 'os'
 *   partition will be defined; the raw boot loader (or a CFE boot block) must
 *   be located at offset 0x0 of this partition.
 *
 * - The 'nvram' partition is generally the last partition, and is interpreted
 *   (and written to) by both CFE and the operating system. While most devices
 *   use the standard Broadcom NVRAM format, some (such as the WGT634U) instead
 *   use the TLV format defined by the CFE specification.
 * 
 * === Dual / Failsafe Image Support ===
 * 
 * If CFE is built with FAILSAFE_UPGRADE or DUAL_IMAGE, the TRX flash range
 * will be split into primary (trx) and secondary (trx2) partitions; the index
 * of the boot partition is managed via the 'bootpartition' (FAILSAFE_UPGRADE)
 * or 'image_boot' (DUAL_IMAGE) NVRAM variables.
 * 
 * The offset of each image is available from NVRAM via the 'image_first_offset'
 * and 'image_second_offset' variables.
 * 
 * Additionally, FAILSAFE_UPGRADE defines two NVRAM variables that are used to
 * communicate boot failure/success between CFE and the OS: 'partialboot' and
 * 'maxpartialboot'.
 * 
 * - When a new image is written to the device, 'partialboot' is set to 1, and
 *   will be incremented by CFE before each boot.
 * - If the OS observes a 'partialboot' value >= 1 and boot was successful,
 *   'partialboot' should be set to 0.
 * - If CFE observes a 'partialboot' value >= 'maxpartialboot', it will switch
 *   to the previous (failsafe) image and reset 'partialboot' 
 * 
 * == CFE Device Naming ==
 * 
 * CFE will allocate at least one CFE flash device per partition; the device
 * name will be in the format of:
 *	<device class><unit number>.<partition name>
 *
 * On NAND devices, there may also be a single top-level non-partitioned
 * device entry covering the entire range.
 * 
 * Unfortunately, TRX support was hacked into CFE, and TRX is not understood by
 * CFE's image loaders; as a result, multiple device units will be allocated
 * for the _same_ flash device (e.g. flash0/flash1), providing two different
 * flash interpretations for use by CFE:
 * 
 *	- flash0 maps _only_ the TRX header as 'flash0.trx', while mapping the
 *	  TRX content as 'flash0.os'; the 'flash0.os' device may be passed
 *	  directly to CFE's (TRX-ignorant) boot image loader, but writing a
 *	  non-empty TRX image to the small 'flash0.trx' partition will fail.
 *	- flash1 maps the entire TRX space as 'flash1.trx', and 'flash1.os'
 *	  does not exist. A new TRX image can be written to the 'flash1.trx'
 *	  partition, but attempting to boot flash1.trx with CFE's (TRX-ignorant)
 *	  loader will fail.
 * 
 * == CFE Driver Quirks ==
 * 
 * flash (legacy) driver (CFI):
 *	IOCTL_FLASH_GETINFO
 *		Part Offset:	invalid (always the base (physical) address of
 *				the flash mapping)
 *		Part Size:	invalid (always the full size of the flash)
 *		Flash Type:	valid
 *		Flash Flags:	invalid (always 0x0)
 * 
 *	IOCTL_NVRAM_GETINFO
 *		NVRAM Offset:	invalid	(always returns the offset of the
 *				NVRAM partition)
 *		NVRAM Size:	invalid (always returns the size of the NVRAM
 *				partition).
 *		NVRAM Erase:	invalid	(always true)
 *
 *	IOCTL_FLASH_PARTITION_INFO
 *		Unsupported
 * 
 * newflash driver (CFI):
 *	IOCTL_FLASH_GETINFO
 *		Part Offset:	valid
 *		Part Size:	valid
 *		Flash Type:	valid
 *		Flash Flags:	invalid	(FLASH_FLAG_NOERASE always set)
 * 
 *	IOCTL_NVRAM_GETINFO
 *		NVRAM Offset:	invalid	(always 0x0)
 *		NVRAM Size:	valid
 *		NVRAM Erase:	valid
 *
 *	IOCTL_FLASH_PARTITION_INFO
 *		Unsupported
 *
 * sflash driver (SPI-NOR):
 *	IOCTL_FLASH_GETINFO
 *		Part Offset:	invalid	(always 0x0)
 *		Part Size:	invalid	(size of underlying flash)
 *		Flash Type:	invalid	(returns ChipCommon flash capability
 *				constants (0x100-0x700), not CFE FLASH_TYPE_*)
 *		Flash Flags:	invalid	(FLASH_FLAG_NOERASE always set)
 *
 *	IOCTL_NVRAM_GETINFO
 *		Unsupported
 *
 *	IOCTL_FLASH_PARTITION_INFO
 *		Part Offset:	valid
 *		Part Size:	valid
 *		Flash Type:	invalid	(ChipCommon flash capability constant)
 *		Flash Flags:	invalid	(FLASH_FLAG_NOERASE always set)
 *
 *		**NOTE**: Only available if CFE is built with command line
 *		support for 'flash -fill'; most CFE images are not.
 * 
 * nflash driver (NAND):
 *	IOCTL_FLASH_GETINFO
 *		Part Offset:	invalid	(always 0x0)
 *		Part Size:	invalid	(size of underlying flash) 
 *		Flash Type:	invalid	(returns the NAND manufacturer ID)
 *		Flash Flags:	correct
 *
 *	IOCTL_NVRAM_GETINFO
 *		Unsupported
 *
 *	IOCTL_FLASH_PARTITION_INFO
 *		Unsupported
 */

MALLOC_DEFINE(M_BCM_CDISK, "cfedisk", "Broadcom CFE disk metadata");

struct bcm_boot_label;

static int		 bcm_probe_disk(struct bcm_disk *disk, bool *skip_next);
static int		 bcm_probe_driver_quirks(struct bcm_disk *disk,
			     uint32_t *quirks);

static bool		 bcm_dev_exists(const char *devname);
static bool		 bcm_part_exists(const char *drvname, u_int unit,
			     const char *label);
static int		 bcm_fmt_devname(const char *drvname, u_int unit,
			     const char *partname, char *buf, size_t *len);

static struct bcm_part	*bcm_disk_get_query_part(struct bcm_disk *disk);

static bcm_part_type	 bcm_lookup_boot_label(const char *label,
			     const struct bcm_boot_label **bpinfo);

static int		 bcm_part_new(struct bcm_part **part,
			     const char *devname);
static void		 bcm_part_free(struct bcm_part *part);
static int		 bcm_probe_part(struct bcm_disk *disk,
			     struct bcm_part *part, uint32_t quirks);

static const bool bcm_disk_trace = false;

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
 * Known CFE device names.
 */
static const char * const bcm_drv_names[] = {
	"flash",	/* CFI/SPI */
	"nflash",	/* NAND */
};

/*
 * Known CFE flash partition names.
 */
static const char * const bcm_part_names[] = {
	"boot",		/* CFE image */
	"brcmnand",	/* Writable OS partition (ignored by CFE) */
	"config",	/* empty MINIX filesystem (found on Netgear WGT634U) */
	"nvram",	/* BCRM NVRAM */
	"os",		/* OS data */
	"os2",		/* OS data (dual/failsafe image) */
	"trx",		/* TRX data */
	"trx2",		/* TRX data (dual/failsafe image) */

	/* Linksys-specific partitions */
	"devinfo",	/* Factory BCRM NVRAM */

	/* Netgear/Foxcon-specific partitions */
	"board_data",	/* Device-specific HW config */
	"ML1",		/* Web interface localizations (ignored by CFE) */
	"ML2",
	"ML3",
	"ML4",
	"ML5",
	"ML6",
	"ML7",
	"POT",		/* NTP timestamp logs + first associated STA MAC addr
			 * (ignored by CFE) */
	"T_Meter1",	/* Traffic meter data (ignored by CFE) */
	"T_Meter2",
};

/**
 * OS/TRX boot partition labels.
 * 
 * Provides a mapping between TRX partition labels and corresponding OS
 * partition label.
 */
static const struct bcm_boot_label {
	const char *os_label;	/**< OS partition label */
	const char *trx_label;	/**< TRX partition label */
} bcm_boot_labels[] = {
	{ "os",		"trx" },
	{ "os2",	"trx2" }
};


/* Ascending comparison of partition offset */
static int
compare_part_offset_asc(const void *lhs, const void *rhs)
{
	struct bcm_part	*lpart, *rpart;

	lpart = (*(struct bcm_part * const *) lhs);
	rpart = (*(struct bcm_part * const *) rhs);

	/* Handle missing offset values; a missing value is always ordered
	 * after any valid value */
	if (!BCM_PART_HAS_OFFSET(lpart) || !BCM_PART_HAS_OFFSET(rpart)) {
		if (BCM_PART_HAS_OFFSET(lpart)) {
			/* LHS has an offset value, RHS does not */
			return (-1);
		} else if (BCM_PART_HAS_OFFSET(rpart)) {
			/* RHS has an offset value, LHS does not */
			return (1);
		} else {
			/* Neither has an offset value */
			return (0);
		}
	}

	/* Simple ascending order */
	if (lpart->offset < rpart->offset) {
		return (-1);
	} else if (lpart->offset > rpart->offset) {
		return (1);
	} else {
		return (0);
	}
}

/**
 * Print the @p disk partition map to the console.
 */
void
bcm_print_disk(struct bcm_disk *disk)
{
	struct bcm_part	*part, **parts;
	size_t		 part_idx;

	printf("CFE disk %s%u:\n", disk->drvname, disk->unit);

	/* Sort partitions by offset, ascending */
	part_idx = 0;
	parts = malloc(sizeof(*parts) * disk->num_parts, M_BCM_CDISK,
	    M_WAITOK);

	SLIST_FOREACH(part, &disk->parts, cp_link) {
		KASSERT(part_idx < disk->num_parts,
		    ("incorrect partition count"));

		parts[part_idx++] = part;
	}

	qsort(parts, disk->num_parts, sizeof(*parts),
	    compare_part_offset_asc);

	for (size_t i = 0; i < disk->num_parts; i++) {
		printf("    %-12s", parts[i]->label);

		if (BCM_PART_HAS_OFFSET(parts[i]))
			printf("0x%08jx", (intmax_t)parts[i]->offset);
		else
			printf("%8s", "unknown");

		if (BCM_PART_HAS_SIZE(parts[i]))
			printf("+0x%08jx", (intmax_t)parts[i]->size);
		else
			printf("+%-8s", "unknown");

		printf("\n");
	}

	free(parts, M_BCM_CDISK);
}

/**
 * Print all disks and partitions in @p disks to the console.
 */
void
bcm_print_disks(struct bcm_disks *disks)
{
	struct bcm_disk	*disk;

	SLIST_FOREACH(disk, disks, cd_link)
		bcm_print_disk(disk);
}

/**
 * Query CFE for all recognized disks and partitions, populating @p result
 * with the probed disk entries.
 * 
 * @param result	An empty list to be populated. If an error occurs,
 *			the provided list will not be unmodified.
 * 
 * @retval 0		success
 * @retval non-zero	if probing otherwise fails, a regular unix error code
 *			will be returned.
 */
int
bcm_probe_disks(struct bcm_disks *result)
{
	struct bcm_disks disks;
	struct bcm_disk	*disk, *dnext;
	int		 error;

	SLIST_INIT(&disks);

	for (size_t i = 0; i < nitems(bcm_drv_names); i++) {
		for (u_int unit = 0; unit < BCM_DISK_UNIT_MAX; unit++) {
			bool skip_next;

			/* Allocate new disk entry */
			disk = bcm_disk_new(bcm_drv_names[i], unit);
			if (disk == NULL) {
				error = ENOMEM;
				goto failed;
			}

			/* Probe partition map */
			if ((error = bcm_probe_disk(disk, &skip_next)))
				goto failed;

			/*
			 * If no partitions are found, the device either
			 * does not exist, or it vends no recognized partition
			 * names.
			 * 
			 * We discard the entry and continue probing this
			 * device class (though it's very unlikely that
			 * additional units will be discovered).
			 */
			if (SLIST_EMPTY(&disk->parts)) {
				bcm_disk_free(disk);
			} else {
				/* Add to results */
				SLIST_INSERT_HEAD(&disks, disk, cd_link);
			}

			/* If requested, skip the next disk unit */
			if (skip_next && unit < BCM_DISK_UNIT_MAX)
				unit++;
		}
	}

	/* Move all records to the result list */
	SLIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		KASSERT(disk == SLIST_FIRST(&disks), ("non-head enumeration"));
		SLIST_REMOVE_HEAD(&disks, cd_link);

		SLIST_INSERT_HEAD(result, disk, cd_link);
	}

	return (0);

failed:
	SLIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		SLIST_REMOVE_HEAD(&disks, cd_link);
		bcm_disk_free(disk);
	}

	return (error);
}

/**
 * Allocate, initialize, and return a new CFE disk entry.
 * 
 * @param drvname	CFE device class name.
 * @param unit		CFE device unit.
 * 
 * @retval non_NULL	success
 * @retval NULL		if allocation fails.
 */
struct bcm_disk *
bcm_disk_new(const char *drvname, u_int unit)
{
	struct bcm_disk *d;

	d = malloc(sizeof(*d), M_BCM_CDISK, M_WAITOK|M_ZERO);

	d->drvname = drvname;
	d->unit = unit;
	d->size = BCM_DISK_INVALID_SIZE;

	SLIST_INIT(&d->parts);
	d->num_parts = 0;

	return (d);
}

/**
 * Free all resources held by @p disk.
 */
void
bcm_disk_free(struct bcm_disk *disk)
{
	free(disk, M_BCM_CDISK);
}

/**
 * Return the first entry in @p disks matching @p drvname and @p unit, or NULL
 * if not found.
 * 
 * @param disks		List of disks to be searched.
 * @param drvname	CFE device class name
 * @param unit		CFE device unit.
 */
struct bcm_disk *
bcm_find_disk(struct bcm_disks *disks, const char *drvname, u_int unit)
{
	struct bcm_disk *disk;

	SLIST_FOREACH(disk, disks, cd_link) {
		if (strcmp(disk->drvname, drvname) != 0)
			continue;

		if (disk->unit != unit)
			continue;

		/* Match */
		return (disk);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return the first entry in @p parts matching @p label, or NULL
 * if not found.
 * 
 * @param parts	List of partitions to be searched.
 * @param label	CFE partition label.
 */
struct bcm_part *
bcm_parts_find(struct bcm_parts *parts, const char *label)
{
	struct bcm_part *part;

	SLIST_FOREACH(part, parts, cp_link) {
		if (strcmp(part->label, label) == 0)
			return (part);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return the first entry in @p parts with the given @p offset, or NULL
 * if not found.
 * 
 * @param parts		List of partitions to be searched.
 * @param offset	CFE partition offset.
 */
struct bcm_part *
bcm_parts_find_offset(struct bcm_parts *parts, off_t offset)
{
	struct bcm_part *part;

	/* An invalid offset matches nothing */
	if (offset == BCM_DISK_INVALID_OFF)
		return (NULL);

	SLIST_FOREACH(part, parts, cp_link) {
		if (part->offset == offset)
			return (part);
	}

	/* Not found */
	return (NULL);
}

/**
 * Find the first partition with @p label and either an unknown offset,
 * or an offset matching @p offset.
 * 
 * @param parts		List of partitions to be searched.
 * @param label		Required label, or NULL to match on any label.
 * @param offset	Required offset, or BCM_DISK_INVALID_OFF to match on any
 *			offset.
 */
struct bcm_part *
bcm_parts_match(struct bcm_parts *parts, const char *label,
    off_t offset)
{
	struct bcm_part *part;

	SLIST_FOREACH(part, parts, cp_link) {
		if (offset != BCM_DISK_INVALID_OFF &&
		    BCM_PART_HAS_OFFSET(part) && part->offset != offset)
		{
			continue;
		}

		if (label != NULL && strcmp(part->label, label) != 0)
			continue;

		/* Found match */
		return (part);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return a partition instance that may be used to perform disk queries.
 * 
 * Will panic if @p disk has no defined partitions.
 * 
 * @param disk A fully probed disk instance.
 */
static struct bcm_part *
bcm_disk_get_query_part(struct bcm_disk *disk)
{
	struct bcm_part *part = SLIST_FIRST(&disk->parts);

	if (part == NULL) {
		panic("query on non-existent disk device: %s%u", disk->drvname,
		    disk->unit);
	}

	return (part);
}

/**
 * Look up and return the system partition information for the given
 * @p label, if any.
 *
 * @param	label	The partition label to look up.
 * @param[out]	binfo	The boot label entry for @p label, or NULL if @p label
 *			is not a recognized boot partition label.
 *
 * @retval BCM_PART_TYPE_OS		if @p label is a recognized OS partition
 *					label.
 * @retval BCM_PART_TYPE_TRX		if @p label is a recognized TRX
 *					partition label.
 * @retval BCM_PART_TYPE_UNKNOWN	if @p label is not a recognized system
 *					partition label.
 */
static bcm_part_type
bcm_lookup_boot_label(const char *label, const struct bcm_boot_label **binfo)
{
	for (size_t i = 0; i < nitems(bcm_boot_labels); i++) {
		const struct bcm_boot_label *sp = &bcm_boot_labels[i];

		if (strcmp(label, sp->os_label) == 0) {
			*binfo = sp;
			return (BCM_PART_TYPE_OS);
		}

		if (strcmp(label, sp->trx_label) == 0) {
			*binfo = sp;
			return (BCM_PART_TYPE_TRX);
		}
	}

	/* Not found */
	*binfo = NULL;
	return (BCM_PART_TYPE_UNKNOWN);
}

/**
 * Determine the driver used by @p disk, and return the corresponding set of
 * quirk flags (see BCM_CFE_QUIRK_*).
 * 
 * If @p disk has no mapped partitions or we cannot otherwise determine the
 * driver type, a set of conservative quirk flags will be returned.
 * 
 * @param	disk	The disk to be queried.
 * @param[out]	quirks	On success, will be set to the probed quirk flags.
 * 
 * @retval 0 		success
 * @retval ENXIO	if the driver cannot be identified.
 */
static int
bcm_probe_driver_quirks(struct bcm_disk *disk, uint32_t *quirks)
{
	struct bcm_part *part;

	part = bcm_disk_get_query_part(disk);

	/* Devices backed by the nflash (NAND) driver can be identified by
	 * the unique driver class name */
	if (strcmp(disk->drvname, "nflash") == 0) {
		BCM_DISK_LOG(disk, "found CFE nflash driver\n");

		*quirks =
		    BCM_CFE_QUIRK_FLASH_ZERO_OFF |
		    BCM_CFE_QUIRK_FLASH_TOTAL_SIZE |
		    BCM_CFE_QUIRK_NVRAM_UNAVAIL |
		    BCM_CFE_QUIRK_PART_EOF_IOERR |
		    BCM_CFE_QUIRK_PART_EOF_OVERREAD;
		return (0);
	}

	/* Devices backed by the sflash (SPI), newflash (CFI), and
	 * flash (legacy CFI) drivers shared a common driver class name
	 * ("flash") and require partition access to be differentiated */
	if (strcmp(disk->drvname, "flash") == 0 && part != NULL) {
		flash_info_t		 fi;
		int			 cerr, rlen;

		/* Fetch flash info */
		cerr = cfe_ioctl(part->fd, IOCTL_FLASH_GETINFO, (u_char *)&fi,
		    sizeof(fi),  &rlen, 0);

		if (cerr != CFE_OK) {
			BCM_DISK_ERR(disk, "cfe_ioctl(%s, IOCTL_FLASH_GETINFO) "
			    "failed: %d\n", part->devname, cerr);
			goto failed;
		}

		/* The 'sflash' driver returns an invalid flash_type value in
		 * the range of 0x100-0x700 (unshifted ChipCommon flash 
		 * capability flag).
		 * 
		 * The 'flash' and 'newflash' drivers return a valid
		 * FLASH_TYPE_FLASH (3) or FLASH_TYPE_ROM (2) flash_type value.
		 * 
		 * The legacy 'flash' driver returns the physical address of
		 * the flash mapping as the flash_base; this will always be
		 * larger than the reported flash size.
		 * 
		 * Since the two flash_type ranges do not overlap, we can use
		 * this to differentiate the 'sflash' and 'newflash'/'flash'
		 * drivers.
		 * 
		 * Since the 'newflash' driver returns a valid partition offset
		 * less than the total flash size, we can use this to
		 * differentiate 'flash' from 'newflash'.
		 */
		if (fi.flash_type > FLASH_TYPE_FLASH) {
			/* sflash (SPI) driver */
			BCM_DISK_LOG(disk, "found CFE sflash driver\n");

			*quirks =
			    BCM_CFE_QUIRK_FLASH_ZERO_OFF |
			    BCM_CFE_QUIRK_FLASH_TOTAL_SIZE |
			    BCM_CFE_QUIRK_NVRAM_UNAVAIL |
			    BCM_CFE_QUIRK_PART_EOF_IOERR |
			    BCM_CFE_QUIRK_PART_EOF_OVERREAD;

			return (0);

		} else if (fi.flash_base >= fi.flash_size) {
			/* legacy flash (CFI) driver */
			BCM_DISK_LOG(disk, "found CFE flash (legacy) driver\n");

			*quirks =
			    BCM_CFE_QUIRK_FLASH_PHYS_OFF |
			    BCM_CFE_QUIRK_FLASH_TOTAL_SIZE |
			    BCM_CFE_QUIRK_PART_EOF_CRASH;

			return (0);

		} else {
			/* newflash (CFI) driver */
			BCM_DISK_LOG(disk, "found CFE newflash driver\n");

			*quirks =
			    BCM_CFE_QUIRK_NVRAM_PART_SIZE |
			    BCM_CFE_QUIRK_PART_EOF_CRASH;

			return (0);
		}
	}

failed:
	/* Cannot determine the driver type */
	BCM_DISK_LOG(disk, "unrecognized driver class\n");
	return (ENXIO);
}

/**
 * Test for the existence of a valid CFE device with @p devname.
 * 
 * @param devname The CFE device name.
 */
static bool
bcm_dev_exists(const char *devname)
{
	int dinfo, dtype;

	/* Does the device exist? */
	if ((dinfo = cfe_getdevinfo(__DECONST(char *, devname))) < 0) {
		if (dinfo != CFE_ERR_DEVNOTFOUND) {
			printf("%s: cfe_getdevinfo(%s) failed: %d\n",
			    __FUNCTION__, devname, dinfo);
		}

		return (false);
	}

	/* Verify device type */
	dtype = dinfo & CFE_DEV_MASK;
	switch (dtype) {
	case CFE_DEV_FLASH:
	case CFE_DEV_NVRAM:
		/* Valid device type */
		return (true);

	default:
		printf("%s: %s has unknown device type: %d\n", __FUNCTION__,
		    devname, dtype);
		return (false);
	}
}

/**
 * Test for the existence of a valid CFE device with the given @p drvname,
 * @p unit, and partition @p partname.
 * 
 * @param drvname	The CFE driver class name.
 * @param unit		The CFE device unit.
 * @param partname	The CFE partition label.
 * 
 * @retval 0		if the device exists and is a supported device type.
 * @retval ENODEV	if the device does not exist.
 * @retval ENXIO	if the device exists, but the device type is not
 *			supported.
 * @retval ENXIO	if testing for the CFE device otherwise fails.
 * @retval ENOMEM	if @p drvname or @p partname exceed the maximum
 *			representible length.
 */
static bool
bcm_part_exists(const char *drvname, u_int unit, const char *partname)
{
	char	dname[BCM_DISK_NAME_MAX];
	size_t	dlen;
	int	error;

	/* Format the full CFE device name */
	dlen = sizeof(dname);
	error = bcm_fmt_devname(drvname, unit, partname, dname, &dlen);
	if (error) {
		printf("%s: failed to format device name for %s%u.%s: %d\n",
		    __FUNCTION__, drvname, unit, partname, error);
		return (false);
	}

	return (bcm_dev_exists(dname));
}

/**
 * Format a CFE partition device name, writing the result to @p buf, and
 * the total length (includng trailing NUL) to @p len.
 * 
 * @param		drvname		The CFE driver class name.
 * @param		unit		The CFE device unit.
 * @param		partname	The CFE partition label.
 * @param[out]		buf		On success, the device name will be
 *					written to this buffer. This argment
 *					may be NULL if the value is not desired.
 * @param[in,out]	len		The capacity of @p buf. On success, will
 *					be set to the actual size of the
 *					requested value.
 *
 * @retval 0		success
 * @retval ENXIO	if an error occurs formatting the device name.
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 */
static int
bcm_fmt_devname(const char *drvname, u_int unit, const char *partname,
    char *buf, size_t *len)
{
	size_t	capacity;
	int	n;

	capacity = *len;

	/* Format the full CFE device name */
	n = snprintf(buf, capacity, "%s%u.%s", drvname, unit, partname);

	if (n < 0) {
		printf("%s: snprintf() failed: %d\n", __FUNCTION__, n);
		return (ENXIO);
	}

	/* Provide the actual length */
	*len = n + 1;

	if (n >= capacity && buf != NULL) {
		return (ENOMEM);
	} else {
		return (0);
	}
}

/**
 * Populate @p disk's partition map.
 * 
 * @param	disk		The disk to be probed.
 * @param[out]	skip_next	If the device unit proceeding @p disk should
 *				be skipped; this will be true in the case that
 *				the next device unit merely provides an
 *				alternative mapping over @p disk.
 * 
 * @retval 0		success
 * @retval non-zero	if probing the partition map otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_probe_disk(struct bcm_disk *disk, bool *skip_next)
{
	struct bcm_part	*part, *pnext;
	struct bcm_parts	 parts;
	const char		*drvname;
	size_t			 num_parts;
	uint32_t		 quirks;
	int			 error;

	*skip_next = false;

	SLIST_INIT(&parts);
	num_parts = 0;

	drvname = disk->drvname;

	/* Iterate over all known partition names and register new partition
	 * entries */
	for (size_t i = 0; i < nitems(bcm_part_names); i++) {
		const char			*partname;
		const struct bcm_boot_label	*binfo;
		bcm_part_type			 part_type;
		char				 dname[BCM_DISK_NAME_MAX];
		size_t				 dlen;
		u_int				 unit;

		partname = bcm_part_names[i];
		unit = disk->unit;

		/* Does the device exist? */
		if (!bcm_part_exists(drvname, unit, partname))
			continue;

		/* Is this an OS or TRX system partition? */
		part_type = bcm_lookup_boot_label(partname, &binfo);

		switch (part_type) {
		case BCM_PART_TYPE_OS:
			KASSERT(binfo != NULL, ("NULL binfo"));

			/*
			 * If a corresponding TRX partition exists, this
			 * OS partition merely provides a bootable remapping of
			 * a subset of the real TRX partition.
			 * 
			 * Such an OS partition can be safely ignored; it only
			 * exists to work around CFE's lack of native TRX boot
			 * support, is not properly page aligned, and attempting
			 * to size the partition may trigger a crash (e.g. in
			 * the nflash driver)
			 */
			if (!bcm_part_exists(drvname, unit, binfo->trx_label))
				break;

			/* The next disk unit provides an alternative mapping */
			*skip_next = true;

			/* Ignore this OS partition */
			continue;

		case BCM_PART_TYPE_TRX: {
			u_int next;
	
			KASSERT(binfo != NULL, ("NULL binfo"));

			/*
			 * If a corresponding OS partition exists, and the
			 * next device unit contains a corresponding TRX
			 * partition (but no OS partition), then this TRX
			 * partition maps only the TRX header.
			 * 
			 * We need to instead target the next device unit's TRX
			 * partition.
			 */
			if (!bcm_part_exists(drvname, unit, binfo->os_label))
				break;

			next = unit+1;
			if (!bcm_part_exists(drvname, next, binfo->trx_label))
				break;

			if (bcm_part_exists(drvname, next, binfo->os_label))
				break;

			/* Use the next device's TRX partition mapping */
			*skip_next = true;
			unit = next;
			break;
		}

		case BCM_PART_TYPE_UNKNOWN:
			break;
		}

		/* Format the full CFE device name */
		dlen = sizeof(dname);
		error = bcm_fmt_devname(drvname, unit, partname, dname,
		    &dlen);
		if (error) {
			BCM_DISK_ERR(disk, "error determining CFE device "
			    "name for %s: %d\n", partname, error);
			goto failed;
		}

	        /* Insert a new partition entry */
		if ((error = bcm_part_new(&part, dname)))
			goto failed;

		SLIST_INSERT_HEAD(&parts, part, cp_link);

		/* Update partition count */
		if (num_parts == SIZE_MAX) {
			BCM_DISK_ERR(disk, "cannot represent more than "
			    "SIZE_MAX partitions\n");
			error = ENOMEM;
			goto failed;
		} else {
			num_parts++;
		}
	}

	/* If no partitions were found, there's nothing to probe */
	if (SLIST_EMPTY(&parts))
		return (0);

	/* Move all discovered partitions to the disk entry */
	SLIST_FOREACH_SAFE(part, &parts, cp_link, pnext) {
		SLIST_REMOVE_HEAD(&parts, cp_link);
		SLIST_INSERT_HEAD(&disk->parts, part, cp_link);
	}

	/* Update the disk entry's partition count */
	disk->num_parts = num_parts;

	/* Probe for any CFE driver quirks */
	if ((error = bcm_probe_driver_quirks(disk, &quirks)))
		goto failed;

	/* Try to fetch the media size */
	if (BCM_DRV_QUIRK(quirks, FLASH_TOTAL_SIZE)) {
		flash_info_t	fi;
		int		cerr, rlen;

		part = bcm_disk_get_query_part(disk);

		/* Fetch flash info */
		cerr = cfe_ioctl(part->fd, IOCTL_FLASH_GETINFO, (u_char *)&fi,
		    sizeof(fi), &rlen, 0);
		if (cerr != CFE_OK) {
			BCM_DISK_ERR(disk, "cfe_ioctl(%s, IOCTL_FLASH_GETINFO) "
			    "failed: %d\n", part->devname, cerr);
			error = ENXIO;
			goto failed;
		}

		/* Save the media size */
#if UINT_MAX > OFF_MAX
		if (fi.flash_size > OFF_MAX) {
			BCM_DISK_ERR(disk, "CFE %s flash size %#x exceeds "
			    "maximum supported offset\n", part->devname,
			    fi.flash_size);

			quirks |= BCM_CFE_QUIRK_FLASH_INV_SIZE;
		}
#endif

		KASSERT(!BCM_DISK_HAS_SIZE(disk), ("overwrite of valid size"));
		disk->size = fi.flash_size;
	}

	/* Try to determine the size and offset of all discovered partitions */
	SLIST_FOREACH(part, &disk->parts, cp_link) {
		if ((error = bcm_probe_part(disk, part, quirks))) {
			BCM_DISK_ERR(disk, "probing %s failed: %d\n",
			    part->devname, error);
		}
	}

	return (0);

failed:
	SLIST_FOREACH_SAFE(part, &parts, cp_link, pnext) {
		SLIST_REMOVE_HEAD(&parts, cp_link);
		bcm_part_free(part);
	}

	return (error);
}

/**
 * Allocate, initialize, and return a new CFE partition entry.
 * 
 * @param[out]	part	On success, a pointer to the newly allocated and
 *			initialized partition entry.
 * @param	disk	Parent disk instance.
 * @param	devname	CFE device name.
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails
 * @retval ENXIO	if the CFE device cannot be opened.
 */
static int
bcm_part_new(struct bcm_part **part, const char *devname)
{
	struct bcm_part *p;

	p = malloc(sizeof(*p), M_BCM_CDISK, M_WAITOK|M_ZERO);

	p->devname = strdup(devname, M_BCM_CDISK);
	p->offset = BCM_DISK_INVALID_OFF;
	p->size = BCM_DISK_INVALID_SIZE;
	p->fs_size = BCM_DISK_INVALID_SIZE;
	p->fd = -1;
	p->need_close = false;

	/* Parse out the partition label ('<devname>.<label>') */
	p->label = strchr(p->devname, '.');
	if (p->label != NULL) {
		/* Advance past '.' */
		p->label++;
	} else {
		p->label = "";
	}

	/* Try to open the device handle */
	if ((p->fd = bcm_get_cfe_fd(bcm_get_platform(), p->devname)) >= 0) {
		/* Handle is shared with other platform code */
		p->need_close = false;
	} else {
		if ((p->fd = cfe_open(p->devname)) >= 0)
			p->need_close = true;
	}

	if (p->fd < 0) {
		BCM_PART_ERR(p, "cfe_open() failed: %d\n", p->fd);
		bcm_part_free(p);
		return (ENXIO);
	}

	*part = p;
	return (0);
}

/**
 * Free all resources held by @p part.
 */
static void
bcm_part_free(struct bcm_part *part)
{
	if (part->fd >= 0 && part->need_close)
		cfe_close(part->fd);

	free(part->devname, M_BCM_CDISK);
	free(part, M_BCM_CDISK);
}

/**
 * Return the offset+length of @p part, or BCM_DISK_INVALID_OFF if unavailable.
 */
off_t
bcm_part_get_end(struct bcm_part *part)
{
	if (!BCM_PART_HAS_OFFSET(part) || !BCM_PART_HAS_SIZE(part))
		return (BCM_DISK_INVALID_OFF);

	KASSERT(OFF_MAX - part->offset >= part->size, ("offset overflow"));

	return (part->offset + part->size);
}

/**
 * Return the offset of the next valid block following @p part, or
 * BCM_DISK_INVALID_OFF if unavailable.
 *
 * @param part	The partion entry to query.
 * @param align	The partition alignment to be assumed when computing the next
 *		offset.
 */
off_t
bcm_part_get_next(struct bcm_part *part, off_t align)
{
	off_t end;

	if ((end = bcm_part_get_end(part)) == BCM_DISK_INVALID_OFF)
		return (BCM_DISK_INVALID_OFF);

	return (roundup(end, align));
}

static int
bcm_probe_part_flashinfo(struct bcm_disk *disk,
    struct bcm_part *part, uint32_t quirks)
{
	flash_info_t	fi;
	int		cerr, rlen;

	/* Skip if both offset and size have already been determined */
	if (BCM_PART_HAS_OFFSET(part) && BCM_PART_HAS_SIZE(part))
		return (0);

	/* Skip if IOCTL_FLASH_GETINFO is unusable */
	if (BCM_DRV_QUIRK(quirks, FLASH_INV_OFF) &&
	    BCM_DRV_QUIRK(quirks, FLASH_INV_SIZE))
		return (0);

	/* Fetch flash info */
	cerr = cfe_ioctl(part->fd, IOCTL_FLASH_GETINFO, (u_char *)&fi,
	    sizeof(fi), &rlen, 0);
	if (cerr != CFE_OK) {
		BCM_PART_ERR(part, "IOCTL_FLASH_GETINFO failed: %d\n", cerr);

		return (ENXIO);
	}

	BCM_PART_TRACE(part, "IOCTL_FLASH_GETINFO (base=%#llx, size=%#x)\n",
	    fi.flash_base, fi.flash_size);

	/* Validate the partition offset */
	if (!BCM_DRV_QUIRK(quirks, FLASH_INV_OFF)) {
#if ULLONG_MAX > OFF_MAX
		if (fi.flash_base > OFF_MAX) {
			BCM_PART_ERR(part, "flash base %#llx exceeds maximum "
			    "supported offset\n", fi.flash_base);
			quirks |= BCM_CFE_QUIRK_FLASH_INV_OFF;
		}
#endif

		if (BCM_DRV_QUIRK(quirks, FLASH_TOTAL_SIZE) &&
		    fi.flash_base > fi.flash_size)
		{
			BCM_PART_ERR(part, "invalid offset %#llx (size=%#x)\n",
			    fi.flash_base, fi.flash_size);
			quirks |= BCM_CFE_QUIRK_FLASH_INV_OFF;
		}
	}

	/* Validate the partition size */
	if (!BCM_DRV_QUIRK(quirks, FLASH_INV_SIZE)) {
#if UINT_MAX > OFF_MAX
		if (fi.flash_size > OFF_MAX) {
			BCM_PART_ERR(part, "flash size %#x exceeds maximum "
			    "supported offset\n", fi.flash_size);
			quirks |= BCM_CFE_QUIRK_FLASH_INV_SIZE;
		}
#endif
	}

	/* Set any missing values in the partition description */
	if (!BCM_DRV_QUIRK(quirks, FLASH_INV_OFF)) {
		if (!BCM_PART_HAS_OFFSET(part))
			part->offset = fi.flash_base;
	}

	if (!BCM_DRV_QUIRK(quirks, FLASH_INV_OFF)) {
		if (!BCM_PART_HAS_SIZE(part))
			part->size = fi.flash_size;
	}

	return (0);
}

static int
bcm_probe_part_nvraminfo(struct bcm_disk *disk,
    struct bcm_part *part, uint32_t quirks)
{
	nvram_info_t	nv;
	int		cerr, rlen;

	/* Skip if size has already been determined */
	if (BCM_PART_HAS_SIZE(part))
		return (0);

	/* Skip if IOCTL_NVRAM_GETINFO does not return the partition size */
	if (!BCM_DRV_QUIRK(quirks, NVRAM_PART_SIZE))
		return (0);

	/* Fetch and validate the NVRAM info */
	cerr = cfe_ioctl(part->fd, IOCTL_NVRAM_GETINFO, (u_char *)&nv,
	    sizeof(nv), &rlen, 0);
	if (cerr != CFE_OK) {
		BCM_PART_ERR(part, "IOCTL_NVRAM_GETINFO failed: %d\n", cerr);
		return (ENXIO);
	}

	if (nv.nvram_size < 0) {
		BCM_PART_ERR(part, "IOCTL_NVRAM_GETINFO returned invalid NVRAM "
		    "size: %#x\n", nv.nvram_size);
		return (ENXIO);
	}

#if INT_MAX > OFF_MAX
	if (nv.nvram_size > OFF_MAX) {
		BCM_PART_ERR(part, "nvram size %#llx exceeds maximum supported "
		    "size\n", fi.flash_size);
		quirks |= BCM_CFE_QUIRK_FLASH_INV_SIZE;
	}
#endif

	/* Set the probed partition size */
	part->size = nv.nvram_size;
	return (0);
}

/*
 * Use cfe_readblk() to probe the partition size (or offset, if
 * the device driver has the PART_EOF_OVERREAD quirk).
 * 
 * This slow path must read the device sequentially, until EOF is hit.
 */
static int
bcm_part_readsz_slow(struct bcm_disk *disk, struct bcm_part *part,
    uint32_t quirks, off_t *result)
{
	int64_t offset;

	/*
	 * On devices where we cannot read past EOF without potentially
	 * triggering a crash, we have to read from the device sequentially.
	 * 
	 * We limit the required I/O by using a two byte read that spans a
	 * block boundary; if we read two bytes, both blocks are valid. If
	 * we read 1 byte, only the first block is valid.
	 */
	offset = 0;
	while (1) {
		u_char	buf[2];
		int	cerr;

		/* Set offset to one byte before the page boundary */
		if (INT64_MAX - offset < BCM_PART_ALIGN_MIN-1) {
			BCM_PART_ERR(part, "CFE read size %#jx exceeds maximum "
			    "supported offset\n", (intmax_t)offset);

			return (ENXIO);
		}

		offset += BCM_PART_ALIGN_MIN-1;

		/* Attempt read */
		cerr = cfe_readblk(part->fd, offset, buf, sizeof(buf));

		if (cerr == CFE_ERR_IOERR &&
		    BCM_DRV_QUIRK(quirks, PART_EOF_IOERR))
		{
			/* Some drivers fail to truncate the two byte read; try
			 * reading a single byte */
			_Static_assert(sizeof(buf) >= 1, "invalid buffer size");
			cerr = cfe_readblk(part->fd, offset, buf, 1);
		}

		if (cerr >= 1) {
			offset += 1;

			/* Have we hit the final block? */
			if (cerr == 1)
				break;
		} else {
			BCM_PART_ERR(part, "cfe_readblk(%#jx, ...) failed with "
			    "unexpected error: %d\n", (intmax_t)offset, cerr);

			return (ENXIO);
		}
	}

#if INT64_MAX > OFF_MAX
	if (offset > OFF_MAX) {
		BCM_PART_ERR(part, "CFE computed size %#jx exceeds maximum "
		    "supported offset\n",(intmax_t)offset);
		return (ENXIO);
	}
#endif

	*result = (off_t)offset;
	return (0);
}

/*
 * Use cfe_readblk() to determine the partition size (or offset, if the device
 * driver has the PART_EOF_OVERREAD quirk).
 * 
 * This fast-path avoids flash I/O by reading from the partition device
 * starting at the maximum possible offset, using a binary search to
 * working backward until we hit the last valid page.
 * 
 * This requires:
 *  - Reading past EOF must not trigger crashing bugs in the CFE flash
 *    driver.
 *  - The flash media size must be available; we need this to determine
 *    a safe maximum offset for our search.
 *  - The flash media size must be a multiple of our block size.
 *
 * If those requirements aren't met by the device/driver, we have to
 * use the slower (and I/O heavy) approach of reading each page
 * sequentially starting at offset 0x0.
 */
static int
bcm_part_readsz_fast(struct bcm_disk *disk, struct bcm_part *part,
    uint32_t quirks, off_t *result)
{
	off_t	blksize, offset;
	off_t	max, min, mid;
	bool	found;

	blksize = BCM_PART_ALIGN_MIN;

	/* Reading past EOF must not trigger a CFE driver crash */
	if (BCM_DRV_QUIRK(quirks, PART_EOF_CRASH))
		return (ENXIO);

	/* The disk must be block-aligned */
	if (disk->size < blksize || disk->size % blksize != 0)
		return (ENXIO);

	/* Perform a binary search for the last valid block */
	max = disk->size;
	min = 0;
	while (max >= min) {
		u_char	buf[1];
		int	cerr;

		mid = rounddown((min + max) / 2, blksize);
		if (mid > INT64_MAX) {
			BCM_PART_ERR(part, "next offset (%#jx) exceeds maximum "
			    "CFE-supported read offset\n", (intmax_t)offset);
			return (ENXIO);
		}

		BCM_PART_TRACE(part, "test %#jx @ %#jx-%#jx\n", (intmax_t)mid,
		    (intmax_t)min,  (intmax_t)max);

		cerr = cfe_readblk(part->fd, mid, buf, sizeof(buf));

		BCM_PART_TRACE(part, "cerr=%d\n", cerr);

		if (cerr == sizeof(buf)) {
			/* Found a valid block; try searching the upper
			 * half of the offset range */
			offset = mid;
			found = true;

			min = mid + blksize;
		} else if (cerr == CFE_ERR_IOERR) {
			/* Invalid block; try searching the lower half
			 * of the offset range */
			if (mid < blksize)
				break;

			max = mid - blksize;
		} else {
			/* Unexpected error or zero-length read */
			BCM_PART_ERR(part, "cfe_readblk(%#jx, ...) failed with "
			    "unexpected result: %d\n", (intmax_t)mid, cerr);

			return (ENXIO);
		}
	}

	/*
	 * If no valid blocks were found, the partition may be smaller than
	 * a block, or may not be block-aligned.
	 */
	if (!found) {
		BCM_PART_ERR(part, "found no valid blocks\n");
		return (ENXIO);
	}

	/* Use byte reads to find the actual terminating offset; this should
	 * generally be a full page. */
	BCM_PART_TRACE(part, "scan @ %#jx\n", offset);
	for (off_t n = blksize; n > 0; n--) {
		u_char	buf[1];
		off_t	next_off;
		int	cerr;

		KASSERT(offset % blksize == 0, ("misaligned base offset %#jx\n",
		    (intmax_t)offset));

		KASSERT(n >= sizeof(buf), ("zero offset"));
		next_off = offset + n - sizeof(buf);

		BCM_PART_TRACE(part, "cfe_readblk(%#jx, ...)\n",
		    (intmax_t)next_off);

		cerr = cfe_readblk(part->fd, next_off, buf, sizeof(buf));

		BCM_PART_TRACE(part, "cerr=%d\n", cerr);

		if (cerr > 0) {
			/* Found last valid offset */
			offset = next_off + sizeof(buf);
			break;
		} else if (cerr == CFE_ERR_IOERR) {
			/* Retry with smaller offset */
			continue;
		} else {
			/* Unexpected error or zero-length read */
			BCM_PART_ERR(part, "cfe_readblk(%#jx, ...) failed with "
			    "unexpected result: %d\n", (intmax_t)offset+n-1,
			    cerr);

			return (ENXIO);
		}
	}

	*result = offset;
	return (0);
}

/**
 * Use cfe_readblk() to determine the partition size (or offset, if the device
 * driver has the PART_EOF_OVERREAD quirk).
 */
static int
bcm_probe_part_read(struct bcm_disk *disk, struct bcm_part *part,
    uint32_t quirks)
{
	off_t	result;
	int	error;

	/* If the driver allows reading past EOF (up to the end of the device),
	 * we're determining the offset, not the size */
	if (BCM_DRV_QUIRK(quirks, PART_EOF_OVERREAD)) {
		/* If we're calculating offset via EOF_OVERREAD, the total
		 * device size must be available */
		if (!BCM_DISK_HAS_SIZE(disk))
			return (0);

		/* Skip if offset has already been determined */
		if (BCM_PART_HAS_OFFSET(part))
			return (0);
	} else {
		/* We're determining the partition size; skip if size has
		 * already been determined */
		if (BCM_PART_HAS_SIZE(part))
			return (0);
	}

	/* Try reading the partition size/offset */
	error = bcm_part_readsz_fast(disk, part, quirks, &result);
	if (error == ENXIO) {
		/* Fall back on the slow path */
		BCM_PART_TRACE(part, "using slow path\n");
		error = bcm_part_readsz_slow(disk, part, quirks, &result);
	}

	if (error) {
		BCM_PART_ERR(part, "cfe_readblk() probe failed: %d\n", error);
		return (error);
	}

	BCM_PART_TRACE(part, "read result: %#jx\n", (intmax_t)result);

	if (BCM_DRV_QUIRK(quirks, PART_EOF_OVERREAD)) {
		/* Result is the number of bytes readable at the partition
		 * offset, up to the total media size */

		KASSERT(BCM_DISK_HAS_SIZE(disk), ("missing disk size"));

		if (result > disk->size) {
			BCM_PART_ERR(part, "read %#jx bytes beyond media end",
			    (intmax_t)(result - disk->size));
			return (ENXIO);
		}

		KASSERT(!BCM_PART_HAS_OFFSET(part), ("offset overwrite"));
		part->offset = disk->size - result;
	} else {
		/* Result is the partition size */
		KASSERT(!BCM_PART_HAS_SIZE(part), ("size overwrite"));
		part->size = result;
	}

	return (0);
}

/**
 * Attempt to determine @p part's size and offset.
 * 
 * @param disk		The disk to be probed.
 * @param part		The partition to be probed.
 * @param quirks	CFE driver quirks (see BCM_CFE_QUIRK_*).
 * 
 * @retval 0		success
 * @retval non-zero	if probing the partition map otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_probe_part(struct bcm_disk *disk, struct bcm_part *part, uint32_t quirks)
{
	int error;

	/* Try to determine offset/size via IOCTL_FLASH_GETINFO */
	if ((error = bcm_probe_part_flashinfo(disk, part, quirks)))
		return (error);

	/* Fall back on IOCTL_NVRAM_GETINFO */
	if ((error = bcm_probe_part_nvraminfo(disk, part, quirks)))
		return (error);
	
	/* If all else fails, we can manually determine the size (but not the
	 * offset) via cfe_readblk() */
	if ((error = bcm_probe_part_read(disk, part, quirks)))
		return (error);

	return (0);
}
