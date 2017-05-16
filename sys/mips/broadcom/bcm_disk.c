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

/*
 * Probe CFE-defined disk/flash partition maps.
 *
 * Since enumeration may require reading from the CFE flash devices, probing
 * should be performed prior to OS driver bring-up.
 * 
 * The validity of the flash/nvram ioctls depends on the CFE flash driver in
 * use, requiring CFE driver-specific workarounds (see 'CFE Driver Quirks'
 * below) and further probing of any missing data once GEOM is available.
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

#include <sys/param.h>
#include <sys/endian.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <machine/elf.h>
#include <machine/_inttypes.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>
#include <dev/cfe/cfe_ioctl.h>

#include <dev/bhnd/cores/chipc/chipcreg.h>

#include "bcm_machdep.h"

#include "bcm_diskvar.h"

MALLOC_DEFINE(M_BCM_DISK, "bcm_disk", "Broadcom CFE disk metadata");

struct bcm_part_info;

static const struct bcm_part_info	*bcm_find_part_info(const char *label);

static int				 bcm_enum_disks(
					     struct bcm_disks *result);
static int				 bcm_enum_known_disks(
					     struct bcm_disks *result);

static int				 bcm_part_new(struct bcm_part **part,
					     const char *drvname, u_int unit,
					     const char *label,
					     bcm_part_type type,
					     uint32_t flags);
static int				 bcm_part_open(struct bcm_part *part);

static int				 bcm_probe_disk_quirks(
					     struct bcm_disk *disk);
static int				 bcm_probe_disk(struct bcm_disk *disk);
static int				 bcm_probe_part(struct bcm_disk *disk,
					     struct bcm_part *part);

static bcm_part_size_fn			 bcm_probe_part_flashinfo;
static bcm_part_size_fn			 bcm_probe_part_nvraminfo;
static bcm_part_size_fn			 bcm_probe_part_readsz;

const bool bcm_disk_trace = false;

/**
 * Known CFE driver class names.
 */
static const char * const bcm_drv_names[] = {
	BCM_DRVNAME_NOR_FLASH,	/* CFI/SPI */
	BCM_DRVNAME_NAND_FLASH,	/* NAND */
};

/*
 * Known CFE flash partition info.
 */
static const struct bcm_part_info {
	const char	**labels;	/**< partition label(s), NULL terminated */
	bcm_part_type	 type;		/**< partition type */
	uint32_t	 default_flags;	/**< default partition flags */
} bcm_part_info[] = {
#define	BCM_PART_LABELS(_label, ...)	\
	(const char *[]){ (_label), ## __VA_ARGS__ }

#define	BCM_PART(_label, _type, _flags, ...) {			\
		BCM_PART_LABELS((_label), ## __VA_ARGS__, NULL),	\
		BCM_PART_TYPE_ ## _type,				\
		(_flags),						\
	}

	BCM_PART("boot",	BOOT,		BCM_PART_PLATFORM_RO),
	BCM_PART("brcmnand",	BRCMNAND,	0x0),
	BCM_PART("nvram",	NVRAM,		BCM_PART_PLATFORM_NVRAM),
	BCM_PART("os",		OS,		BCM_PART_PLATFORM,
		 "os2"	/* dual/failsafe label */),
	BCM_PART("trx",		TRX,		BCM_PART_PLATFORM,
		 "trx2"	/* dual/failsafe label */),

	/* Netgear-specific partition types */
	BCM_PART("config",	LEAF_CONFIG,	BCM_PART_PLATFORM),
	BCM_PART("board_data",	BOARD_DATA,	BCM_PART_PLATFORM_NVRAM_RO),
	BCM_PART("ML1",		MULTILANG,	0x0,
		 "ML2",
		 "ML3",
		 "ML4",
		 "ML5",
		 "ML6",
		 "ML7"),
	BCM_PART("POT",		POT,		0x0),
	BCM_PART("T_Meter1",	TMETER,		0x0,
		 "T_Meter2"),

	/* Linksys-specific partition types */
	BCM_PART("devinfo",	DEVINFO,	BCM_PART_PLATFORM_NVRAM_RO),

	/* Asus-specific partition types */
	BCM_PART("asus",	ASUSFW,		BCM_PART_PLATFORM_RO),
};

/**
 * Return the partition info for @p label, or NULL if not found.
 */
static const struct bcm_part_info *
bcm_find_part_info(const char *label)
{
	for (size_t i = 0; i < nitems(bcm_part_info); i++) {
		const struct bcm_part_info *id = &bcm_part_info[i];

		for (size_t j = 0; id->labels[j] != NULL; j++) {
			if (strcmp(id->labels[j], label) == 0)
				return (id);
		}
	}

	/* Not found */
	return (NULL);
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
	struct bcm_disks	 disks;
	struct bcm_disk		*disk, *dnext;
	struct bcm_bootinfo	 bootinfo;
	int			 error;

	LIST_INIT(&disks);

	/* Prefer cfe_enumdev() if available; fall back on enumerating known
	 * device partitions */
	if (cfe_enumdev(0, (char[1]){}, 1) != CFE_ERR_INV_COMMAND)
		error = bcm_enum_disks(&disks);
	else
		error = bcm_enum_known_disks(&disks);

	if (error)
		return (error);

	if ((error = bcm_get_bootinfo(&bootinfo)))
		return (error);

	/*
	 * Perform initial filtering/cleanup of our discovered disks:
	 * 
	 * - Normalize OS/TRX partition mappings
	 * - Remove unused virtual disk entries
	 * - Set any bootinfo-derived flags/sizes/offsets.
	 *
	 * If a disk:
	 *  - contains both an OS and TRX partition, AND
	 *  - the next device unit contains a TRX partition
	 * Then:
	 *  - The OS partition maps the TRX partition minus the initial TRX
	 *    header, and should be dropped.
	 *  - The TRX partition maps only the TRX header; the real TRX
	 *    partition is defined on the next device unit, and should be
	 *    moved to the current device.
	 *  - The next device unit should be discarded.
	 */
	LIST_FOREACH(disk, &disks, cd_link) {
		struct bcm_disk	*virt;
		struct bcm_part	*os[BCM_DISK_BOOTIMG_MAX];
		struct bcm_part *trx[BCM_DISK_BOOTIMG_MAX];
		struct bcm_part	*part;
		size_t		 num_bootimg;
		bool		 keep_virt;

		virt = NULL;

		/* Set disk-level boot flags */
		if (bcm_disk_has_devunit(disk, &bootinfo.romdev))
			disk->flags |= BCM_DISK_BOOTROM;

		if (bcm_disk_has_devunit(disk, &bootinfo.osdev))
			disk->flags |= BCM_DISK_BOOTOS;

		/* Fetch OS and TRX partitions for all bootimgs */
		num_bootimg = 0;
		for (size_t i = 0; i < BCM_DISK_BOOTIMG_MAX; i++) {
			os[i] = bcm_disk_find_bootimg_part(disk,
			    BCM_PART_TYPE_OS, i);
			trx[i] = bcm_disk_find_bootimg_part(disk,
			    BCM_PART_TYPE_TRX, i);

			/* Boot images are not sparse; if we hit missing
			 * entries, no further images need be checked */
			if (os[i] == NULL && trx[i] == NULL)
				break;
	
			num_bootimg++;
		}

		/* Warn if our bootinfo does not match probed partitions */
		if (BCM_DISK_HAS_FLAGS(disk, BCM_DISK_BOOTOS) &&
		    num_bootimg != bootinfo.num_images)
		{
			BCM_DISK_LOG(disk, "found %zu boot images on flash "
			    "(NVRAM defines %zu)\n", num_bootimg,
			    bootinfo.num_images);
		}

		/* Nothing to do? */
		if (num_bootimg == 0)
			continue;

		/* Drop OS partitions that are simply a TRX partition
		 * remapping (i.e. both an OS and corresponding TRX partition
		 * were found) */
		for (size_t i = 0; i < num_bootimg; i++) {
			if (os[i] == NULL || trx[i] == NULL)
				continue;

			bcm_disk_remove_part(disk, os[i]->label);
			os[i] = NULL;
		}

		/* Look for virtual device mapping our full TRX partitions */
		if (disk->unit < BCM_DISK_UNIT_MAX) {
			virt = bcm_find_disk(&disks, disk->drvname,
			    disk->unit+1);
		} else {
			virt = NULL;
		}
	
		/* Replace any partial TRX partition definitions with
		 * real definitions from the virtual disk device, and if
		 * possible, drop the virtual device's disk entry */
		if (virt != NULL) {
			for (size_t i = 0; i < num_bootimg; i++) {
				struct bcm_part *real;

				if (trx[i] == NULL)
					continue;

				real = bcm_disk_get_part(virt, trx[i]->label);
				if (real == NULL)
					continue;

				trx[i] = real;
				error = bcm_disk_move_part(virt, disk,
				    trx[i]->label,   true);
				if (error) {
					BCM_DISK_LOG(disk, "error remapping "
					    "%s: %d\n", trx[i]->label, error);
					goto failed;
				}
			}

			/* If the virtual disk contains no partitions not
			 * already defined by our disk, it can safely be
			 * discarded */
			keep_virt = false;
			LIST_FOREACH(part, &virt->parts, cp_link) {
				if (bcm_disk_has_part(disk, part->label))
					continue;

				keep_virt = true;
				break;
			}

			if (keep_virt) {
				/* Should never happen */
				BCM_DISK_ERR(virt, "retaining non-empty "
				    "virtual disk device\n");
			} else {
				LIST_REMOVE(virt, cd_link);
				bcm_disk_free(virt);
			}
		}

		/* Now that all OS partitions have been fixed up, apply
		 * any partition-level boot metadata */
		if (!BCM_DISK_HAS_FLAGS(disk, BCM_DISK_BOOTOS))
			continue;

		for (size_t i = 0; i < num_bootimg; i++) {
			struct bcm_part *bp;

			if (i >= bootinfo.num_images) {
				/* Missing bootinfo. Logged above */
				break;
			}

			/* Legacy non-TRX device? */
			if (trx[i] == NULL)
				bp = os[i];
			else
				bp = trx[i];

			KASSERT(bp != NULL, ("missing partition for bootimg "
			    "%zu\n", i));

			/* Mark partition as bootable */
			if (bootinfo.bootimg == i)
				bp->flags |= BCM_PART_BOOT;

			/* Populate partition offset/size from the NVRAM
			 * boot configuration */
			if (!BCM_PART_HAS_SIZE(bp))
				bp->size = bootinfo.images[i].size;
	
			if (!BCM_PART_HAS_OFFSET(bp))
				bp->offset = bootinfo.images[i].offset;
		}
	}

	/* Now that we've cleaned up the disk/partition list, probe all
	 * partitions to determine size/offset */
	LIST_FOREACH(disk, &disks, cd_link) {
		if ((error = bcm_probe_disk(disk)))
			goto failed;
	}

	/* Move all discovered disks to the result list */
	LIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		LIST_REMOVE(disk, cd_link);
		LIST_INSERT_HEAD(result, disk, cd_link);
	}

	return (0);

failed:
	LIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		LIST_REMOVE(disk, cd_link);
		bcm_disk_free(disk);
	}

	return (error);
}


/* Use cfe_enumdev() to discover all CFE disk devices */
static int
bcm_enum_disks(struct bcm_disks *result)
{
	struct bcm_disks	 disks;
	struct bcm_disk		*disk, *dnext;
	int			 error;

	LIST_INIT(&disks);

	for (int i = 0; i < INT_MAX; i++) {
		char				 devname[BCM_DISK_NAME_MAX];
		struct bcm_part			*part;
		const struct bcm_part_info	*part_info;
		const char			*lptr, *dptr;
		char				*label, *drvname;
		bcm_part_type			 part_type;
		size_t				 llen, drvlen;
		uint32_t			 part_flags;
		u_int				 unit;
		int				 dinfo, cerr;

		/* Fetch next device name */
		cerr = cfe_enumdev(i, devname, sizeof(devname));
		if (cerr == CFE_ERR_DEVNOTFOUND) {
			/* End of list */
			break;
		} else if (cerr != CFE_OK) {
			printf("cfe_enumdev(%d) failed: %d\n", i, cerr);
			error = ENXIO;
			goto failed;
		}

		/* Verify the device type */
		if ((dinfo = cfe_getdevinfo(devname)) < 0) {
			printf("cfe_getdevinfo(%s) failed: %d\n", devname,
			    dinfo);
			error = ENXIO;
			goto failed;
		}

		if (!BCM_DISK_CFE_DEVTYPE_SUPPORTED(dinfo & CFE_DEV_MASK))
			continue;

		/* Parse the device name */
		error = bcm_disk_parse_devname(devname, &dptr, &drvlen, &unit,
		    &lptr, &llen);
		if (error) {
			printf("cfe_enumdev(%d) returned invalid device "
			    "name: %s\n", i, devname);
			goto failed;
		}

		/* Produce NUL terminated copies */
		drvname = strndup(dptr, drvlen, M_BCM_DISK);
		label = strndup(lptr, llen, M_BCM_DISK);

		/* Fetch or create a corresponding disk entry */
		if ((disk = bcm_find_disk(&disks, drvname, unit)) == NULL)
			disk = bcm_disk_new(drvname, unit, 0x0);

		free(drvname, M_BCM_DISK);
		drvname = NULL;

		/* Allocate new partition entry */
		part_info = bcm_find_part_info(label);
		if (part_info != NULL) {
			part_type = part_info->type;
			part_flags = part_info->default_flags;
		} else {
			part_type = BCM_PART_TYPE_UNKNOWN;
			part_flags = 0x0;
		}

		error = bcm_part_new(&part, disk->drvname, disk->unit, label,
		    part_type, part_flags);
		if (error) {
			BCM_DISK_ERR(disk, "failed to allocate %s partition "
			    "entry: %d\n", label, error);

			free(label, M_BCM_DISK);
			goto failed;
		} else {
			free(label, M_BCM_DISK);
			label = NULL;
		}

		/* Register partition with the disk entry */
		if ((error = bcm_disk_add_part(disk, part))) {
			BCM_DISK_ERR(disk, "error registering %s partition: "
			    "%d\n", part->label, error);

			bcm_part_free(part);
			goto failed;
		}
	}

	/* Move all discovered disks to the result list */
	LIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		LIST_REMOVE(disk, cd_link);
		LIST_INSERT_HEAD(result, disk, cd_link);
	}

	return (0);

failed:
	LIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		LIST_REMOVE(disk, cd_link);
		bcm_disk_free(disk);
	}

	return (error);
}

/*
 * Use our static list of partition names to enumerate and register
 * all known partitions on the given disk.
 */
static int
bcm_enum_known_disk_parts(struct bcm_disk *disk)
{
	int error;

	/* Iterate over all known partition names and register new partition
	 * entries */
	for (size_t i = 0; i < nitems(bcm_part_info); i++) {
		const struct bcm_part_info *info = &bcm_part_info[i];

		for (size_t j = 0; info->labels[j] != NULL; j++) {
			struct bcm_part	*part;
			const char	*label;
			char		 devname[BCM_DISK_NAME_MAX];
			size_t		 dlen;
			int		 dinfo, dtype;

			label = info->labels[j];

			/* Format the full CFE device name */
			dlen = sizeof(devname);
			error = bcm_disk_devname(disk->drvname, disk->unit,
			    label, devname, &dlen);
			if (error) {
				BCM_DISK_ERR(disk, "error formatting CFE "
				    "device name for %s: %d\n", label, error);
				return (error);
			}

			/* Verify the device existence and type */
			dinfo = cfe_getdevinfo(devname);
			if (dinfo == CFE_ERR_DEVNOTFOUND) {
				continue;
			} else if (dinfo < 0) {
				BCM_DISK_ERR(disk, "cfe_getdevinfo(%s) failed: "
				    "%d\n", devname, dinfo);
				return (ENXIO);
			}

			dtype = (dinfo & CFE_DEV_MASK);
			if (!BCM_DISK_CFE_DEVTYPE_SUPPORTED(dtype))
				continue;

		        /* Register a new partition entry */
			error = bcm_part_new(&part, disk->drvname, disk->unit,
			    label, info->type, info->default_flags);
			if (error) {
				BCM_DISK_LOG(disk, "failed to allocate %s "
				    "partition entry: %d\n", label, error);
				return (error);
			}

			if ((error = bcm_disk_add_part(disk, part))) {
				BCM_DISK_LOG(disk, "error registering %s "
				    "partition: %d\n", part->label, error);

				bcm_part_free(part);
				return (error);
			}
		}
	}

	return (0);
}

/*
 * Use a static list of driver and partition names to enumerate CFE disk
 * devices.
 *
 * Intended for use on devices without cfe_enumdev()
 */
static int
bcm_enum_known_disks(struct bcm_disks *result)
{
	struct bcm_disks	 disks;
	struct bcm_disk		*disk, *dnext;
	int			 error;

	LIST_INIT(&disks);

	for (size_t i = 0; i < nitems(bcm_drv_names); i++) {
		for (u_int unit = 0; unit < BCM_DISK_UNIT_MAX; unit++) {
			/* Allocate new disk entry */
			disk = bcm_disk_new(bcm_drv_names[i], unit, 0x0);
			if (disk == NULL) {
				error = ENOMEM;
				goto failed;
			}

			/* Register all known partition labels */
			if ((error = bcm_enum_known_disk_parts(disk))) {
				bcm_disk_free(disk);
				goto failed;
			}

			/*
			 * If no partitions are found, the device either
			 * does not exist, or it vends no recognized partition
			 * names.
			 * 
			 * We discard the entry and continue probing this
			 * device class (though it's very unlikely that
			 * additional units will be discovered).
			 */
			if (LIST_EMPTY(&disk->parts)) {
				bcm_disk_free(disk);
				continue;
			}

			/* Add to result list */
			LIST_INSERT_HEAD(&disks, disk, cd_link);
		}
	}

	/* Move all discovered disks to the result list */
	LIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		LIST_REMOVE(disk, cd_link);
		LIST_INSERT_HEAD(result, disk, cd_link);
	}

	return (0);

failed:
	LIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		LIST_REMOVE(disk, cd_link);
		bcm_disk_free(disk);
	}

	return (error);
}

/**
 * Allocate, initialize, and return a new CFE disk entry.
 * 
 * @param drvname	CFE device class name.
 * @param unit		CFE device unit.
 * @param flags		Disk flags (see BCM_PART_* flag enums)
 * 
 * @retval non_NULL	success
 * @retval NULL		if allocation fails.
 */
struct bcm_disk *
bcm_disk_new(const char *drvname, u_int unit, uint32_t flags)
{
	struct bcm_disk *d;

	d = malloc(sizeof(*d), M_BCM_DISK, M_WAITOK|M_ZERO);

	d->drvname = drvname;
	d->unit = unit;
	d->quirks = BCM_CFE_QUIRK_INVALID;
	d->size = BCM_DISK_INVALID_SIZE;
	d->flags = flags;

	LIST_INIT(&d->parts);
	d->num_parts = 0;

	return (d);
}

/**
 * Free all resources held by @p disk.
 */
void
bcm_disk_free(struct bcm_disk *disk)
{
	struct bcm_part *p, *pnext;

	LIST_FOREACH_SAFE(p, &disk->parts, cp_link, pnext) {
		LIST_REMOVE(p, cp_link);
		bcm_part_free(p);
	}

	free(disk, M_BCM_DISK);
}

/**
 * Determine the CFE driver used by @p disk, and populate the disk's driver
 * quirk flags (see BCM_CFE_QUIRK_*).
 *
 * @param	disk	The disk to be queried.
 * @param[out]	quirks	On success, will be set to the probed quirk flags.
 * 
 * @retval 0 		success
 * @retval ENXIO	if the driver cannot be identified.
 */
static int
bcm_probe_disk_quirks(struct bcm_disk *disk)
{
	struct bcm_part *part;
	
	KASSERT(disk->quirks == BCM_CFE_QUIRK_INVALID,
	    ("quirks already probed"));

	part = bcm_disk_get_query_part(disk);

	/* Devices backed by the nflash (NAND) driver can be identified by
	 * the unique driver class name */
	if (strcmp(disk->drvname, "nflash") == 0) {
		BCM_DISK_LOG(disk, "found CFE nflash driver\n");

		disk->quirks =
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

			disk->quirks =
			    BCM_CFE_QUIRK_FLASH_ZERO_OFF |
			    BCM_CFE_QUIRK_FLASH_TOTAL_SIZE |
			    BCM_CFE_QUIRK_NVRAM_UNAVAIL |
			    BCM_CFE_QUIRK_PART_EOF_IOERR |
			    BCM_CFE_QUIRK_PART_EOF_OVERREAD;

			return (0);

		} else if (fi.flash_base >= fi.flash_size) {
			/* legacy flash (CFI) driver */
			BCM_DISK_LOG(disk, "found CFE flash (legacy) driver\n");

			disk->quirks =
			    BCM_CFE_QUIRK_FLASH_PHYS_OFF |
			    BCM_CFE_QUIRK_FLASH_TOTAL_SIZE |
			    BCM_CFE_QUIRK_PART_EOF_CRASH;

			return (0);

		} else {
			/* newflash (CFI) driver */
			BCM_DISK_LOG(disk, "found CFE newflash driver\n");

			disk->quirks =
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
 * Probe and populate @p disk's quirks and partition table.
 * 
 * @param	disk		The disk to be probed.
 * 
 * @retval 0		success
 * @retval non-zero	if probing the partition map otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_probe_disk(struct bcm_disk *disk)
{
	struct bcm_part	*part;
	int		 error;

	/* Open all partitions' backing CFE device for reading */
	LIST_FOREACH(part, &disk->parts, cp_link) {
		if ((error = bcm_part_open(part)) == 0)
			continue;

		BCM_PART_ERR(part, "failed to open CFE device: %d\n", error);
		return (error);
	}

	/* Try to probe the disk driver quirks */
	if ((error = bcm_probe_disk_quirks(disk))) {
		BCM_DISK_ERR(disk, "failed to determine CFE driver quirks: "
		    "%d\n", error);
		return (error);
	}

	/* Try to fetch the media size */
	if (BCM_DISK_QUIRK(disk, FLASH_TOTAL_SIZE)) {
		flash_info_t	fi;
		int		cerr, rlen;

		part = bcm_disk_get_query_part(disk);

		/* Fetch flash info */
		cerr = cfe_ioctl(part->fd, IOCTL_FLASH_GETINFO, (u_char *)&fi,
		    sizeof(fi), &rlen, 0);
		if (cerr != CFE_OK) {
			BCM_DISK_ERR(disk, "cfe_ioctl(%s, IOCTL_FLASH_GETINFO) "
			    "failed: %d\n", part->devname, cerr);
			return (ENXIO);
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
	LIST_FOREACH(part, &disk->parts, cp_link) {
		if ((error = bcm_probe_part(disk, part))) {
			/* non-fatal; try next probe */
			BCM_DISK_ERR(disk, "probing %s failed: %d\n",
			    part->devname, error);
		}
	}

	return (0);
}

/**
 * Allocate, initialize, and return a new CFE partition entry.
 * 
 * @param[out]	part	On success, a pointer to the newly allocated and
 *			initialized partition entry.
 * @param	drvname	CFE device driver class.
 * @param	unit	CFE device unit.
 * @param	label	CFE partition label.
 * @param	type	Partition type.
 * @param	flags	Partition flags (see BCM_PART_* flag enums)
 */
static int
bcm_part_new(struct bcm_part **part, const char *drvname, u_int unit,
    const char *label, bcm_part_type type, uint32_t flags)
{
	struct bcm_part	*p;
	char		*devname;
	size_t		 nlen;
	int		 error;

	/* Determine formatted CFE device name length */
	if ((error = bcm_disk_devname(drvname, unit, label, NULL, &nlen)))
		return (error);

	/* Format CFE device name */
	devname = malloc(nlen, M_BCM_DISK, M_WAITOK);
	if ((error = bcm_disk_devname(drvname, unit, label, devname, &nlen))) {
		free(devname, M_BCM_DISK);
		return (error);
	}

	p = malloc(sizeof(*p), M_BCM_DISK, M_WAITOK|M_ZERO);
	p->devname = devname;
	p->label = strdup(label, M_BCM_DISK);
	p->type = type;
	p->flags = flags;
	p->offset = BCM_DISK_INVALID_OFF;
	p->size = BCM_DISK_INVALID_SIZE;
	p->fs_size = BCM_DISK_INVALID_SIZE;

	/* On some devices, cfe_close() may trash CFE's heap; thus,
	 * we avoid opening the backing CFE partition until we're sure
	 * we actually need the partition in question (and we avoid closing
	 * the device) */
	p->fd = -1;
	p->need_close = false;

	*part = p;
	return (0);
}

/**
 * Open the partition's backing CFE device for reading.
 * 
 * @param part	The partition to be opened.
 * 
 * @retval 0		success
 * @retval EBUSY	if @p part is already open.
 * @retval ENXIO	if opening the partition map otherwise fails.
 */
static int
bcm_part_open(struct bcm_part *part)
{
	int fd;

	if (part->fd >= 0)
		return (EBUSY);

	if ((fd = bcm_get_cfe_fd(bcm_get_platform(), part->devname)) >= 0) {
		/* Handle is shared with other platform code */
		part->fd = fd;
		part->need_close = false;

		return (0);
	} else if ((fd = cfe_open(part->devname)) >= 0) {
		part->fd = fd;
		part->need_close = true;

		return (0);
	}

	BCM_PART_ERR(part, "cfe_open() failed: %d\n", fd);
	return (ENXIO);
}

/**
 * Free all resources held by @p part.
 */
void
bcm_part_free(struct bcm_part *part)
{
	if (part->fd >= 0 && part->need_close)
		cfe_close(part->fd);

	free(part->devname, M_BCM_DISK);
	free(part, M_BCM_DISK);
}

static int
bcm_probe_part_flashinfo(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_size *result)
{
	flash_info_t	fi;
	int		cerr, rlen;

	/* Skip if IOCTL_FLASH_GETINFO is unusable */
	if (BCM_DISK_QUIRK(disk, FLASH_INV_OFF) &&
	    BCM_DISK_QUIRK(disk, FLASH_INV_SIZE))
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

	/* Provide the partition offset */
	if (!BCM_DISK_QUIRK(disk, FLASH_INV_OFF)) {
#if ULLONG_MAX > OFF_MAX
		if (fi.flash_base > OFF_MAX) {
			BCM_PART_ERR(part, "flash base %#llx exceeds maximum "
			    "supported offset\n", fi.flash_base);
			return (ENXIO);
		}
#endif

		result->offset = fi.flash_base;
	}

	/* Provide the partition size */
	if (!BCM_DISK_QUIRK(disk, FLASH_INV_SIZE)) {
#if UINT_MAX > OFF_MAX
		if (fi.flash_size > OFF_MAX) {
			BCM_PART_ERR(part, "flash size %#x exceeds maximum "
			    "supported size\n", fi.flash_size);
			return (ENXIO);
		}
#endif

		result->size = fi.flash_size;
	}

	return (0);
}

static int
bcm_probe_part_nvraminfo(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_size *result)
{
	nvram_info_t	nv;
	int		cerr, rlen;

	/* Skip if IOCTL_NVRAM_GETINFO does not return the partition size */
	if (!BCM_DISK_QUIRK(disk, NVRAM_PART_SIZE))
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
		return (ENXIO);
	}
#endif

	/* Set the probed partition size */
	result->size = nv.nvram_size;
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
    off_t *result)
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
		    BCM_DISK_QUIRK(disk, PART_EOF_IOERR))
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
    off_t *result)
{
	off_t	blksize, offset;
	off_t	max, min, mid;
	bool	found;

	blksize = BCM_PART_ALIGN_MIN;

	/* Reading past EOF must not trigger a CFE driver crash */
	if (BCM_DISK_QUIRK(disk, PART_EOF_CRASH))
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
bcm_probe_part_readsz(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_size *result)
{
	off_t	rd_result;
	int	error;

	/* If the driver allows reading past EOF (up to the end of the device),
	 * we're determining the offset, not the size */
	if (BCM_DISK_QUIRK(disk, PART_EOF_OVERREAD)) {
		/* If we're calculating offset via EOF_OVERREAD, the total
		 * device size must be available */
		if (!BCM_DISK_HAS_SIZE(disk))
			return (0);
	}

	/* Try reading the partition size/offset */
	error = bcm_part_readsz_fast(disk, part, &rd_result);
	if (error == ENXIO) {
		/* Fall back on the slow path */
		BCM_PART_TRACE(part, "using slow path\n");
		error = bcm_part_readsz_slow(disk, part, &rd_result);
	}

	if (error) {
		BCM_PART_ERR(part, "cfe_readblk() probe failed: %d\n", error);
		return (error);
	}

	BCM_PART_TRACE(part, "read result: %#jx\n", (intmax_t)rd_result);

	if (BCM_DISK_QUIRK(disk, PART_EOF_OVERREAD)) {
		/* Result is the number of bytes readable at the partition
		 * offset, up to the total media size */

		KASSERT(BCM_DISK_HAS_SIZE(disk), ("missing disk size"));

		if (rd_result > disk->size) {
			BCM_PART_ERR(part, "read %#jx bytes beyond media end",
			    (intmax_t)(rd_result - disk->size));
			return (ENXIO);
		}

		result->offset = disk->size - rd_result;
	} else {
		/* Result is the partition size */
		result->size = rd_result;
	}

	return (0);
}

/**
 * Execute the given partition sizing @p fn, and if the result is valid,
 * update @p part with the probed partition sizing.
 * 
 * @param disk	The disk containing @p part.
 * @param part	The partition to be probed.
 * @param fn	The probe function.
 * 
 * @retval 0		success
 * @retval ENXIO	if the probe returns invalid partition sizing metadata.
 * @retval ENXIO	if the probe returns partition sizing metadata that
 *			conflicts with the existing @p part metadata.
 */
static int
bcm_try_size_part(struct bcm_disk *disk, struct bcm_part *part,
    bcm_part_size_fn *fn)
{
	struct bcm_part_size	psz;
	int			error;

	psz = BCM_PARTSZ_INITIALIZER;

	/* Probe and update partition sizing */
	if ((error = fn(disk, part, &psz)))
		return (error);

	if ((error = bcm_part_update_sizing(disk, part, &psz)))
		return (error);

	return (0);
}

/**
 * Perform a CFE-based read of @p len bytes from @p part at @p offset.
 * 
 * @param part		The partition to read from.
 * @param offset	The read offset.
 * @param outp		Output buffer.
 * @param len		Number of bytes to be read.
 * 
 * @retval 0		success
 * @retval ENODEV	if the backing CFE device has not been opened.
 * @retval ENXIO	if the read fails, or fewer than @p len bytes were
 *			read.
 */
static int
bcm_part_read(struct bcm_part *part, off_t offset, void *outp, size_t len)
{
	int cerr;

	if (part->fd < 0)
		return (ENODEV);

	if (offset > INT64_MAX)
		return (ENXIO);

	if (len > INT_MAX)
		return (ENXIO);

	cerr = cfe_readblk(part->fd, (int64_t)offset, outp, (int)len);
	if (cerr < 0) {
		BCM_PART_ERR(part, "cfe_readblk() failed: %d\n", cerr);
		return (ENXIO);
	}

	if (cerr != len) {
		BCM_PART_ERR(part, "cfe_readblk() short read: %d\n", cerr);
		return (ENXIO);
	}

	return (0);
}

/**
 * Initialize @p ident with the given offset, size, and bytes to be used
 * for fingerprinting.
 * 
 * @param ident		Identification instance to be initialized.
 * @param bytes		Bytes to be used when generating the partition
 *			fingerprint.
 * @param offset	Offset at which @p bytes were read, relative to the
 *			partition start.
 * @param len		The total size of @p bytes.
 * @param byteswap	If target-endian data structures are not in host byte
 *			order.
 * 
 * @retval 0		success
 * @retval non-zero	if initializing @p ident otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_part_ident_init(struct bcm_part_ident *ident, void *bytes, off_t offset,
    off_t len, bool byteswap)
{
	MD5_CTX ctx;

	KASSERT(len > 0 && len != BCM_DISK_INVALID_SIZE, ("invalid length"));
	KASSERT(offset != BCM_DISK_INVALID_OFF, ("invalid offset"));
	KASSERT(OFF_MAX - offset >= len, ("offset+len overflow"));

	ident->fp_offset = offset;
	ident->fp_size = len;

	MD5Init(&ctx);
	MD5Update(&ctx, bytes, len);
	MD5Final(ident->fp_md5, &ctx);

	return (0);
}

/**
 * Attempt to determine @p part's size and offset.
 * 
 * @param disk		The disk to be probed.
 * @param part		The partition to be probed.
 * 
 * @retval 0		success
 * @retval non-zero	if probing the partition map otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_probe_part(struct bcm_disk *disk, struct bcm_part *part)
{
	struct bcm_part_ident_info	*idfn, **idfnp;
	int				 error;

	/* Try to determine offset/size via IOCTL_FLASH_GETINFO */
	error = bcm_try_size_part(disk, part, bcm_probe_part_flashinfo);
	if (error) {
		BCM_PART_ERR(part, "bcm_probe_part_flashinfo() failed: %d\n",
		    error);
		return (error);
	}

	/* ... IOCTL_NVRAM_GETINFO */
	error = bcm_try_size_part(disk, part, bcm_probe_part_nvraminfo);
	if (error) {
		BCM_PART_ERR(part, "bcm_probe_part_nvraminfo() failed: %d\n",
		    error);
		return (error);
	}

	/* If all else fails, we can manually determine the size (or offset,
	 * if the device has the PART_EOF_OVERREAD quirk) via cfe_readblk() */
	error = bcm_try_size_part(disk, part, bcm_probe_part_readsz);
	if (error) {
		BCM_PART_ERR(part, "bcm_probe_part_readsz() failed: %d\n",
		    error);
		return (error);
	}

	/* Finally, perform partition identification/fingerprinting */
	SET_FOREACH(idfnp, bcm_part_ident_set) {
		struct bcm_part_size	psz;
		struct bcm_part_ident	ident;

		idfn = *idfnp;
		psz = BCM_PARTSZ_INITIALIZER;

		/* Already identified? */
		if (BCM_PART_HAS_FLAGS(part, BCM_PART_IDENTIFIED))
			break;

		/* Skip inapplicable ident functions */
		if (idfn->type != part->type)
			continue;

		/* Try the given identification function */
		error = idfn->func(disk, part, &ident, &psz);

		BCM_PART_TRACE(part, "identified %s partition: %d\n",
		    idfn->name, error);
		if (error)
			continue;

		/* Identification succeeded -- try applying any updated sizing
		 * (which may fail) first */
		if ((error = bcm_part_update_sizing(disk, part, &psz)))
			return (error);

		/* Update identification data */
		part->ident = ident;
		part->flags |= BCM_PART_IDENTIFIED;
	}

	return (0);
}

/* CFE partition identification */
static int
bcm_part_ident_cfe(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_ident *ident, struct bcm_part_size *size)
{
	uint32_t	magic[2];
	off_t		offset;
	bool		bswap;
	int		error;

	offset = BCM_CFE_MAGIC_OFFSET;
	if ((error = bcm_part_read(part, offset, magic, sizeof(magic))))
		return (error);

	switch (magic[0]) {
	case BCM_CFE_MAGIC:
		if (magic[1] != BCM_CFE_MAGIC)
			return (ENXIO);

		/* Native endian */
		bswap = false;
		break;

	case BCM_CFE_CIGAM:
		if (magic[1] != BCM_CFE_CIGAM)
			return (ENXIO);

		/* Byte-swapped */
		bswap = true;
		break;

	default:
		/* Unrecognized */
		return (ENXIO);
	}

	error = bcm_part_ident_init(ident, magic, offset, sizeof(magic), bswap);
	return (error);
}

BCM_PART_IDENT("CFE", BCM_PART_TYPE_BOOT, bcm_part_ident_cfe);


/* CFE self-describing binary identification */
static int
bcm_part_ident_cfez(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_ident *ident, struct bcm_part_size *size)
{
	struct bcm_cfez_header	hdr;
	off_t			offset;
	bool			bswap;
	uint32_t		start, end;
	off_t			fs_size, part_size;
	int			error;

	offset = BCM_CFE_BISZ_OFFSET;
	if ((error = bcm_part_read(part, offset, &hdr, sizeof(hdr))))
		return (error);

	switch (hdr.magic) {
	case BCM_CFE_BISZ_MAGIC:
		/* Native endian */
		bswap = false;
		break;

	case BCM_CFE_BISZ_CIGAM:
		/* Byte-swapped */
		bswap = true;
		break;

	default:
		/* Unrecognized */
		return (ENXIO);
	}

	/* Populate our ident data before performing byte swapping */
	error = bcm_part_ident_init(ident, &hdr, offset, sizeof(hdr), bswap);
	if (error)
		return (error);

	/* Swap all offsets */
	if (bswap) {
		KASSERT(
		    nitems(hdr.txt) == BCM_CFE_BISZ_NOFFS &&
		    nitems(hdr.data) == BCM_CFE_BISZ_NOFFS &&
		    nitems(hdr.bss) == BCM_CFE_BISZ_NOFFS,
		    ("invalid offset array size"));

		for (size_t i = 0; i < BCM_CFE_BISZ_NOFFS; i++) {
			hdr.txt[i] = bswap32(hdr.txt[i]);
			hdr.data[i] = bswap32(hdr.data[i]);
			hdr.bss[i] = bswap32(hdr.bss[i]);
		}
	}

	/* Determine the binary size */
	start = hdr.txt[BCM_CFE_BISZ_ST_IDX];
	end = hdr.data[BCM_CFE_BISZ_END_IDX];

#if UINT32_MAX > OFF_MAX
	if (end <= start || start - end > OFF_MAX) {
#else
	if (end <= start) {
#endif
		BCM_PART_ERR(part, "invalid start offset %#" PRIx32 " relative "
		    "to end offset %#" PRIx32 "\n", start, end);
		return (ENXIO);
	}

	fs_size = (off_t)(end - start);

	/*
	 * Determine the enclosing partition size.
	 * 
	 * CFE rounds the binary size up to the nearest power of two to
	 * determine the total partition size (with a minimum size of 128KB).
	 */
	if (fs_size <= BCM_CFE_BISZ_MINSIZE) {
		part_size = BCM_CFE_BISZ_MINSIZE;
	} else {
		/* Round up to the next power of two; from the public domain
		 * "Bit Twiddling Hacks" */
		part_size = fs_size;

		KASSERT(part_size > 0, ("zero-length size"));
		part_size--;
		part_size |= part_size >> 1;
		part_size |= part_size >> 2;
		part_size |= part_size >> 4;
		part_size |= part_size >> 8;
		part_size |= part_size >> 16;
		part_size++;
	}

	BCM_PART_TRACE(part, "size=%#jx fs_size=%#jx\n", (intmax_t)part_size,
	    (intmax_t)fs_size);

	size->fs_size = fs_size;
	size->size = part_size;

	return (0);
}

BCM_PART_IDENT("CFEZ", BCM_PART_TYPE_BOOT, bcm_part_ident_cfez);


/* OS partition identification */
static int
bcm_part_ident_os(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_ident *ident, struct bcm_part_size *size)
{
	off_t	offset;
	int	error;
	union {
		Elf_Ehdr	 elf;
		uint8_t		 gzip[3];
	} hdr;

	offset = 0x0;
	if ((error = bcm_part_read(part, offset, &hdr, sizeof(hdr))))
		return (error);

	/* Look for an ELF bootloader */
	if (IS_ELF(hdr.elf)) {
		BCM_PART_TRACE(part, "ELF bootloader at %#jx\n",
		    (intmax_t)offset);

		return (bcm_part_ident_init(ident, &hdr.elf, offset,
		    sizeof(hdr.elf), false));
	}

	/* Look for a GZIP-compressed bootloader */
	if (hdr.gzip[0] == BCM_GZIP_MAGIC0 && hdr.gzip[1] == BCM_GZIP_MAGIC1 &&
	    hdr.gzip[2] == BCM_GZIP_DEFLATE)
	{
		BCM_PART_TRACE(part, "GZIP-compressed loader at %#jx\n",
		    (intmax_t)offset);

		return (bcm_part_ident_init(ident, &hdr.gzip, offset,
		    sizeof(hdr.gzip), false));
	}

	/* Scan for a boot block */
	for (size_t i = 0; i < BCM_BOOTBLK_MAX; i++ ) {
		struct bcm_cfe_bootblk	bootblk;
		bool			bswap;

		offset = sizeof(bootblk) * i;
		error = bcm_part_read(part, offset, &bootblk, sizeof(bootblk));
		if (error);
			return (error);

		switch (bootblk.magic) {
		case BCM_BOOTBLK_CIGAM:
			/* Found (byte swapped) */
			bswap = true;
			break;
		case BCM_BOOTBLK_MAGIC:
			/* Found (native endian) */
			bswap = false;
			break;
		default:
			/* Check next boot block offset */
			continue;
		}

		/* Found boot block */
		BCM_PART_TRACE(part, "boot block at %#jx\n", (intmax_t)offset);

		return (bcm_part_ident_init(ident, &bootblk, offset,
		    sizeof(bootblk), bswap));
	}

	/* No boot block found */
	return (ENXIO);
}

BCM_PART_IDENT("OS", BCM_PART_TYPE_OS, bcm_part_ident_os);

