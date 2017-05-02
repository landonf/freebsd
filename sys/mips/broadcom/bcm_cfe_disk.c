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

#include "bcm_cfe_disk.h"

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

static int			 bcm_cfe_probe_disk(struct bcm_cfe_disk *disk);
static uint32_t			 bcm_cfe_probe_driver_quirks(
				     struct bcm_cfe_disk *disk);

static struct bcm_cfe_part	*bcm_cfe_disk_get_query_part(
				     struct bcm_cfe_disk *disk);

static int			 bcm_cfe_part_new(struct bcm_cfe_part **part,
				     struct bcm_cfe_disk *disk,
				     const char *devname);
static void			 bcm_cfe_part_free(struct bcm_cfe_part *part);
static int			 bcm_cfe_probe_part(struct bcm_cfe_part *part,
				     uint32_t quirks);

static bool			 bcm_cfe_part_is_os_label(const char *label);

static const bool bcm_disk_trace = false;

#define	BCM_DISK_LOG(msg, ...)	printf(msg, ## __VA_ARGS__)

#define	BCM_DISK_ERR(msg, ...)	\
	BCM_DISK_LOG("%s: " msg, __FUNCTION__, ## __VA_ARGS__)

#define	BCM_DISK_DBG(msg, ...)	do {			\
	if (bootverbose)				\
		BCM_DISK_ERR(msg, ## __VA_ARGS__);	\
} while(0)

#define	BCM_DISK_TRACE(msg, ...)	do {			\
	if (bcm_disk_trace)				\
		BCM_DISK_ERR(msg, ## __VA_ARGS__);	\
} while(0)

/**
 * Known CFE device names.
 */
static const char * const cfe_drv_names[] = {
	"flash",	/* CFI/SPI */
	"nflash",	/* NAND */
};

/*
 * Known CFE flash partition names.
 */
static const char * const cfe_part_names[] = {
	"boot",		/* CFE image */
	"brcmnand",	/* OS partition (ignored by CFE, NAND-only) */
	"config",	/* MINIX filesystem used by Netgear WGT634U */
	"devinfo",	/* Factory BCRM NVRAM on Linksys devices (EA6700) */
	"nvram",	/* BCRM NVRAM */
	"os",		/* OS data */
	"os2",		/* OS data (dual/failsafe image) */
	"trx",		/* TRX data */
	"trx2",		/* TRX data (dual/failsafe image) */
};

/**
 * OS/TRX partition labels.
 */
static const struct cfe_trx_part {
	const char *os_label;	/**< OS partition label */
	const char *trx_label;	/**< TRX partition label */
} cfe_trx_parts[] = {
	{ "os",		"trx" },
	{ "os2",	"trx2" }
};

static void
bcm_cfe_print_disks(struct bcm_cfe_disks *disks)
{
	struct bcm_cfe_disk *disk;
	struct bcm_cfe_part *part;

	SLIST_FOREACH(disk, disks, cd_link) {
		printf("CFE disk %s%u:\n", disk->drvname, disk->unit);

		SLIST_FOREACH(part, &disk->parts, cp_link) {
			printf("    %-12s", part->label);

			if (part->offset != BCM_CFE_INVALID_OFF)
				printf("0x%08jx", (intmax_t)part->offset);
			else
				printf("%8s", "unknown");

			if (part->size != BCM_CFE_INVALID_SIZE)
				printf("+0x%08jx", (intmax_t)part->size);
			else
				printf("+%-8s", "unknown");

			printf("\n");
		}
	}
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
bcm_cfe_probe_disks(struct bcm_cfe_disks *result)
{
	struct bcm_cfe_disks	 disks;
	struct bcm_cfe_disk	*disk, *dnext;
	int			 error;

	SLIST_INIT(&disks);

	for (size_t i = 0; i < nitems(cfe_drv_names); i++) {
		for (u_int unit = 0; unit < BCM_CFE_DUNIT_MAX; unit++) {
			/* Allocate new disk entry */
			disk = bcm_cfe_disk_new(cfe_drv_names[i], unit);
			if (disk == NULL) {
				error = ENOMEM;
				goto failed;
			}

			/* Probe partition map */
			if ((error = bcm_cfe_probe_disk(disk)))
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
				bcm_cfe_disk_free(disk);
				continue;
			}

			/* Add to results */
			SLIST_INSERT_HEAD(&disks, disk, cd_link);
		}
	}

	if (bootverbose)
		bcm_cfe_print_disks(&disks);

	/* Move all records to the result list */
	SLIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		KASSERT(disk == SLIST_FIRST(&disks), ("non-head enumeration"));
		SLIST_REMOVE_HEAD(&disks, cd_link);

		SLIST_INSERT_HEAD(result, disk, cd_link);
	}

	panic("yeeeehaw!");

	return (0);

failed:
	SLIST_FOREACH_SAFE(disk, &disks, cd_link, dnext) {
		SLIST_REMOVE_HEAD(&disks, cd_link);
		bcm_cfe_disk_free(disk);
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
struct bcm_cfe_disk *
bcm_cfe_disk_new(const char *drvname, u_int unit)
{
	struct bcm_cfe_disk *d;

	d = malloc(sizeof(*d), M_BCM_CDISK, M_WAITOK|M_ZERO);

	d->drvname = drvname;
	d->unit = unit;
	d->size = BCM_CFE_INVALID_SIZE;

	SLIST_INIT(&d->parts);

	return (d);
}

/**
 * Free all resources held by @p disk.
 */
void
bcm_cfe_disk_free(struct bcm_cfe_disk *disk)
{
	free(disk, M_BCM_CDISK);
}

/**
 * Return a partition instance that may be used to perform disk queries.
 * 
 * Will panic if @p disk has no defined partitions.
 * 
 * @param disk A fully probed disk instance.
 */
static struct bcm_cfe_part *
bcm_cfe_disk_get_query_part(struct bcm_cfe_disk *disk)
{
	struct bcm_cfe_part *part = SLIST_FIRST(&disk->parts);

	if (part == NULL) {
		panic("query on non-existent disk device: %s%u", disk->drvname,
		    disk->unit);
	}

	return (part);
}

/**
 * Determine the driver used by @p disk, and return the corresponding set of
 * quirk flags (see BCM_CFE_DRV_QUIRK_*).
 * 
 * If @p disk has no mapped partitions or we cannot otherwise determine the
 * driver type, a set of conservative quirk flags will be returned.
 * 
 * @param disk The disk to be queried.
 */
static uint32_t
bcm_cfe_probe_driver_quirks(struct bcm_cfe_disk *disk)
{
	struct bcm_cfe_part *part;

	part = bcm_cfe_disk_get_query_part(disk);

	/* Devices backed by the nflash (NAND) driver can be identified by
	 * the unique driver class name */
	if (strcmp(disk->drvname, "nflash") == 0) {
		BCM_DISK_LOG("%s%u: found CFE nflash driver\n", disk->drvname,
		    disk->unit);

		return (BCM_CFE_DRV_QUIRK_FLASH_ZERO_OFF |
			BCM_CFE_DRV_QUIRK_FLASH_TOTAL_SIZE |
			BCM_CFE_DRV_QUIRK_NVRAM_UNAVAIL |
			BCM_CFE_DRV_QUIRK_READBLK_EOF_IOERR);
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
			BCM_DISK_ERR("cfe_ioctl(%s, IOCTL_FLASH_GETINFO) "
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
			BCM_DISK_LOG("%s%u: found CFE sflash driver\n",
			    disk->drvname, disk->unit);

			return (BCM_CFE_DRV_QUIRK_FLASH_ZERO_OFF |
				BCM_CFE_DRV_QUIRK_FLASH_TOTAL_SIZE |
				BCM_CFE_DRV_QUIRK_NVRAM_UNAVAIL |
				BCM_CFE_DRV_QUIRK_READBLK_EOF_IOERR);
		} else if (fi.flash_base >= fi.flash_size) {
			/* legacy flash (CFI) driver */
			BCM_DISK_LOG("%s%u: found CFE flash (legacy) driver\n",
			    disk->drvname, disk->unit);

			return (BCM_CFE_DRV_QUIRK_FLASH_PHYS_OFF |
				BCM_CFE_DRV_QUIRK_FLASH_TOTAL_SIZE |
				BCM_CFE_DRV_QUIRK_READBLK_EOF_CRASH);
		} else {
			/* newflash (CFI) driver */
			BCM_DISK_LOG("%s%u: found CFE newflash driver\n",
			    disk->drvname, disk->unit);

			return (BCM_CFE_DRV_QUIRK_NVRAM_PART_SIZE |
				BCM_CFE_DRV_QUIRK_READBLK_EOF_CRASH);
		}
	}

	BCM_DISK_LOG("%s%u: unrecognized driver class\n", disk->drvname,
	    disk->unit);

failed:
	/* Cannot determine the driver type; assume all ioctls return invalid
	 * data. */
	return (BCM_CFE_DRV_QUIRK_FLASH_INV_OFF |
		BCM_CFE_DRV_QUIRK_FLASH_INV_SIZE |
		BCM_CFE_DRV_QUIRK_NVRAM_UNAVAIL);
}

/**
 * Populate @p disk's partition map.
 * 
 * @param disk	The disk to be probed.
 * 
 * @retval 0		success
 * @retval non-zero	if probing the partition map otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_cfe_probe_disk(struct bcm_cfe_disk *disk)
{
	struct bcm_cfe_part	*part, *pnext;
	struct bcm_cfe_parts	 parts;
	uint32_t		 quirks;
	int			 error;

	SLIST_INIT(&parts);

	/* Iterate over all known partition names and register new partition
	 * entries */
	for (size_t i = 0; i < nitems(cfe_part_names); i++) {
		const char	*partname;
		char		 dname[BCM_CFE_DNAME_MAX];
		int		 dinfo, dtype, n;

		/* Format the full CFE device name */
		partname = cfe_part_names[i];
		n = snprintf(dname, sizeof(dname), "%s%u.%s", disk->drvname,
		    disk->unit, partname);

		if (n >= sizeof(dname)) {
			BCM_DISK_ERR("invalid partition name: %s\n", partname);
			error = ENOMEM;
			goto failed;
		}

		/* Does the partition exist? */
		if ((dinfo = cfe_getdevinfo(dname)) < 0) {
			if (dinfo != CFE_ERR_DEVNOTFOUND) {
				BCM_DISK_ERR("cfe_getdevinfo(%s) failed: %d\n",                                                                                                                                        
				    dname, dinfo);
			}

			continue;
		}

		/* Verify device type */
		dtype = dinfo & CFE_DEV_MASK;
		switch (dtype) {
		case CFE_DEV_FLASH:
		case CFE_DEV_NVRAM:
			/* Valid device type */
			break;
		default:
			BCM_DISK_ERR("%s has unknown device type: %d\n", dname,
			    dtype);
			continue;
		}

	        /* Insert a new partition entry */
		if ((error = bcm_cfe_part_new(&part, disk, dname)))
			goto failed;

		SLIST_INSERT_HEAD(&parts, part, cp_link);
	}

	/* If no partitions were found, there's nothing to probe */
	if (SLIST_EMPTY(&parts))
		return (0);

	/* Move all discovered partitions to the disk entry */
	SLIST_FOREACH_SAFE(part, &parts, cp_link, pnext) {
		SLIST_REMOVE_HEAD(&parts, cp_link);
		SLIST_INSERT_HEAD(&disk->parts, part, cp_link);
	}

	/* Probe for any CFE driver quirks */
	quirks = bcm_cfe_probe_driver_quirks(disk);

	/* Try to fetch the media size */
	if (BCM_CFE_DRV_QUIRK(quirks, FLASH_TOTAL_SIZE)) {
		flash_info_t	fi;
		int		cerr, rlen;

		part = bcm_cfe_disk_get_query_part(disk);

		/* Fetch flash info */
		cerr = cfe_ioctl(part->fd, IOCTL_FLASH_GETINFO, (u_char *)&fi,
		    sizeof(fi), &rlen, 0);
		if (cerr != CFE_OK) {
			BCM_DISK_ERR("cfe_ioctl(%s, IOCTL_FLASH_GETINFO) "
			    "failed: %d\n", part->devname, cerr);
			error = ENXIO;
			goto failed;
		}

		/* Save the media size */
#if UINT_MAX > OFF_MAX
		if (fi.flash_size > OFF_MAX) {
			BCM_DISK_ERR("CFE %s flash size %#x exceeds maximum "
			    "supported offset\n", part->devname, fi.flash_size);
			quirks |= BCM_CFE_DRV_QUIRK_FLASH_INV_SIZE;
		}
#endif

		KASSERT(disk->size == BCM_CFE_INVALID_SIZE,
		    ("overwrite of valid size"));

		disk->size = fi.flash_size;
	}

	/* Try to determine the size and offset of all discovered partitions */
	SLIST_FOREACH(part, &disk->parts, cp_link) {
		if ((error = bcm_cfe_probe_part(part, quirks))) {
			BCM_DISK_LOG("probing %s failed: %d\n", part->devname,
			    error);
		}
	}

	return (0);

failed:
	SLIST_FOREACH_SAFE(part, &parts, cp_link, pnext) {
		SLIST_REMOVE_HEAD(&parts, cp_link);
		bcm_cfe_part_free(part);
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
bcm_cfe_part_new(struct bcm_cfe_part **part, struct bcm_cfe_disk *disk,
    const char *devname)
{
	struct bcm_cfe_part *p;

	p = malloc(sizeof(*p), M_BCM_CDISK, M_WAITOK|M_ZERO);

	p->disk = disk;
	p->devname = strdup(devname, M_BCM_CDISK);
	p->offset = BCM_CFE_INVALID_OFF;
	p->size = BCM_CFE_INVALID_SIZE;
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
		BCM_DISK_ERR("cfe_open(%s) failed: %d\n", p->devname, p->fd);
		bcm_cfe_part_free(p);
		return (ENXIO);
	}

	*part = p;
	return (0);
}

/**
 * Free all resources held by @p part.
 */
static void
bcm_cfe_part_free(struct bcm_cfe_part *part)
{
	if (part->fd >= 0 && part->need_close)
		cfe_close(part->fd);

	free(part->devname, M_BCM_CDISK);
	free(part, M_BCM_CDISK);
}

static int
bcm_cfe_probe_part_flashinfo(struct bcm_cfe_part *part, uint32_t quirks)
{
	flash_info_t	fi;
	int		cerr, rlen;

	/* Skip if both offset and size have already been determined */
	if (part->offset != BCM_CFE_INVALID_OFF &&
	    part->size != BCM_CFE_INVALID_SIZE)
		return (0);

	/* Skip if IOCTL_FLASH_GETINFO is unusable */
	if (BCM_CFE_DRV_QUIRK(quirks, FLASH_INV_OFF) &&
	    BCM_CFE_DRV_QUIRK(quirks, FLASH_INV_SIZE))
		return (0);

	/* Fetch flash info */
	cerr = cfe_ioctl(part->fd, IOCTL_FLASH_GETINFO, (u_char *)&fi,
	    sizeof(fi), &rlen, 0);
	if (cerr != CFE_OK) {
		BCM_DISK_ERR("cfe_ioctl(%s, IOCTL_FLASH_GETINFO) "
		    "failed: %d\n", part->devname, cerr);

		return (ENXIO);
	}

	/* Validate the partition offset */
	if (!BCM_CFE_DRV_QUIRK(quirks, FLASH_INV_OFF)) {
#if ULLONG_MAX > OFF_MAX
		if (fi.flash_base > OFF_MAX) {
			BCM_DISK_ERR("CFE %s flash base %#llx exceeds maximum "
			    "supported offset\n", part->devname, fi.flash_base);
			quirks |= BCM_CFE_DRV_QUIRK_FLASH_INV_OFF;
		}
#endif

		if (BCM_CFE_DRV_QUIRK(quirks, FLASH_TOTAL_SIZE) &&
		    fi.flash_base > fi.flash_size)
		{
			BCM_DISK_ERR("CFE %s: invalid offset %#llx "
			    "(size=%#x)\n", part->devname, fi.flash_base,
			    fi.flash_size);
			quirks |= BCM_CFE_DRV_QUIRK_FLASH_INV_OFF;
		}
	}

	/* Validate the partition size */
	if (!BCM_CFE_DRV_QUIRK(quirks, FLASH_INV_SIZE)) {
#if UINT_MAX > OFF_MAX
		if (fi.flash_size > OFF_MAX) {
			BCM_DISK_ERR("CFE %s flash size %#x exceeds maximum "
			    "supported offset\n", part->devname, fi.flash_size);
			quirks |= BCM_CFE_DRV_QUIRK_FLASH_INV_SIZE;
		}
#endif
	}

	/* Set any missing values in the partition description */
	if (!BCM_CFE_DRV_QUIRK(quirks, FLASH_INV_OFF)) {
		if (part->offset == BCM_CFE_INVALID_OFF)
			part->offset = fi.flash_base;
	}

	if (!BCM_CFE_DRV_QUIRK(quirks, FLASH_INV_OFF)) {
		if (part->size == BCM_CFE_INVALID_SIZE)
			part->size = fi.flash_size;
	}

	return (0);
}

static int
bcm_cfe_probe_part_nvraminfo(struct bcm_cfe_part *part, uint32_t quirks)
{
	nvram_info_t	nv;
	int		cerr, rlen;

	/* Skip if size has already been determined */
	if (part->size != BCM_CFE_INVALID_SIZE)
		return (0);

	/* Skip if IOCTL_NVRAM_GETINFO does not return the partition size */
	if (!BCM_CFE_DRV_QUIRK(quirks, NVRAM_PART_SIZE))
		return (0);

	/* Fetch and validate the NVRAM info */
	cerr = cfe_ioctl(part->fd, IOCTL_NVRAM_GETINFO, (u_char *)&nv,
	    sizeof(nv), &rlen, 0);
	if (cerr != CFE_OK) {
		BCM_DISK_ERR("cfe_ioctl(%s, IOCTL_NVRAM_GETINFO) "
		    "failed: %d\n", part->devname, cerr);

		return (ENXIO);
	}

	if (nv.nvram_size < 0) {
		BCM_DISK_ERR("CFE %s returned invalid NVRAM size: %#x\n",
		    part->devname, nv.nvram_size);
		return (ENXIO);
	}

#if INT_MAX > OFF_MAX
	if (nv.nvram_size > OFF_MAX) {
		BCM_DISK_ERR("CFE %s flash size %#llx exceeds maximum "
		    "supported offset\n", part->devname, fi.flash_size);
		quirks |= BCM_CFE_DRV_QUIRK_FLASH_INV_SIZE;
	}
#endif

	/* Set the probed partition size */
	part->size = nv.nvram_size;
	return (0);
}


static int
bcm_cfe_probe_part_readsize_slow(struct bcm_cfe_part *part, uint32_t quirks)
{
	off_t	size;

	/* Skip if size has already been determined */
	if (part->size != BCM_CFE_INVALID_SIZE)
		return (0);

	/*
	 * Ideally, we could determine the total size of the partition without
	 * requiring I/O by reading 1 byte at the end of each block, working
	 * backwards from the media size until a 0-length read succeeded.
	 * 
	 * In practice, reading at an offset beyond the partition's end triggers
	 * arithemetic bugs in the legacy 'flash' driver (and possibly others)
	 * leading to the CFE code smashing our read buffer and/or reading from
	 * a negative offset.
	 * 
	 * To avoid this, we have to read from the device sequentially, using
	 * a two byte read that spans a known valid block, and a potentially
	 * invalid block.
	 */
	size = 0;
	while (1) {
		u_char		buf[2];
		uint64_t	offset;
		int		cerr;

		/* The next offset must not overflow the int64_t representation,
		 * and must be representible as an off_t */
		offset = size;
		if (INT64_MAX - offset < BCM_CFE_PALIGN_MIN-1 ||
		    offset > OFF_MAX ||
		    OFF_MAX - offset < BCM_CFE_PALIGN_MIN-1)
		{
			BCM_DISK_ERR("CFE %s computed size %#jx exceeds "
			    "maximum supported offset\n", part->devname,
			    (intmax_t)offset);

			return (ENXIO);
		}

		offset += BCM_CFE_PALIGN_MIN-1;

		/* Attempt read */
		KASSERT(sizeof(buf) == 2, ("invalid buffer size"));
		cerr = cfe_readblk(part->fd, offset, buf, 2);

		if (cerr == CFE_ERR_IOERR &&
		    BCM_CFE_DRV_QUIRK(quirks, READBLK_EOF_IOERR))
		{
			/* Some drivers fail to truncate the two byte read; try
			 * reading a single byte */
			cerr = cfe_readblk(part->fd, offset, buf, 1);
		}

		if (cerr >= 1) {
			size += BCM_CFE_PALIGN_MIN;

			/* Have we hit the final block? */
			if (cerr == 1)
				break;
		} else {
			BCM_DISK_ERR("cfe_readblk(%s, %#jx, ...) failed with "
			    "unexpected error: %d\n", part->devname,
			    (intmax_t)offset, cerr);

			return (ENXIO);
		}
	}

#if UINT_MAX > OFF_MAX
	/* This is ensured by the read loop overflow check above */
	KASSERT(offset <= OFF_MAX, ("invalid offset"));
#endif

	part->size = size;
	return (0);
}

/**
 * Returns true if @p label is a CFE OS partition label, false otherwise.
 */
static bool
bcm_cfe_part_is_os_label(const char *label)
{
	for (size_t i = 0; i < nitems(cfe_trx_parts); i++) {
		if (strcmp(label, cfe_trx_parts[i].os_label) == 0)
			return (true);
	}

	return (false);
}

/*
 * Introspect the partition and try to determine ... TODO
 */
static int
bcm_cfe_probe_part_readsize(struct bcm_cfe_part *part, uint32_t quirks)
{
	off_t	blksize, offset;
	bool	found;

	if (/*BCM_CFE_DRV_QUIRK(quirks, ALIGN_LAST_BLOCK) && */
	    bcm_cfe_part_is_os_label(part->label))
	{
		// TODO
		return (0);
	}

	// XXX TODO; nflash driver locks up on misaligned reads required by the
	// os partion.
	// if (strcmp(part->label, "os") == 0)
	// 	return (0);

	/*
	 * Our fast-path avoids flash I/O by reading from the partition device
	 * starting at the maximum possible offset, and then working backwards
	 * until we hit a valid page.
	 * 
	 * This requires:
	 *  - Reading past EOF must not trigger crashing bugs in the CFE flash
	 *    driver.
	 *  - The flash media size must be available; we need this to determine
	 *    a safe offset at which to begin our searching for EOF.
	 *  - The flash media size must be a multiple of our block size.
	 *
	 * If those requirements aren't met by the device/driver, we have to
	 * use the slower (and I/O heavy) approach of reading each page
	 * sequentially starting at offset 0x0.
	 */
	blksize = BCM_CFE_PALIGN_MIN;

	if (part->disk->size == BCM_CFE_INVALID_SIZE ||
	    part->disk->size < blksize || part->disk->size % blksize != 0 ||
	    BCM_CFE_DRV_QUIRK(quirks, READBLK_EOF_CRASH))
	{
		BCM_DISK_TRACE("Using cfe_readblk(%s) slow path\n",
		    part->devname);
		return (bcm_cfe_probe_part_readsize_slow(part, quirks));
	} else {
		BCM_DISK_TRACE("Using cfe_readblk(%s) fast path\n",
		    part->devname);
	}

	/* Scan backwards for the first valid block */
	found = false;
	for (offset = rounddown(part->disk->size - 1, blksize);
	     offset >= blksize; offset -= blksize)
	{
		u_char	buf[1];
		int	cerr;

		/* Check offset+blksize for overflow */
		if (OFF_MAX - offset < blksize)
			break;

		KASSERT(offset+blksize < INT64_MAX, ("unsupported offset"));
		KASSERT(part->fd >= 0, ("device not open"));

		BCM_DISK_TRACE("cfe_readblk(%s, %#jx, ...)\n", part->devname,
		    (intmax_t)offset - sizeof(buf));

		cerr = cfe_readblk(part->fd, offset, buf, sizeof(buf));

		BCM_DISK_TRACE("cerr=%d\n", cerr);

		if (cerr == sizeof(buf)) {
			/* Found a valid block */
			found = true;
			break;
		} else if (cerr == CFE_ERR_IOERR) {
			/* Invalid block; keep searching */
		} else {
			/* Unexpected error or zero-length read */
			BCM_DISK_ERR("cfe_readblk(%s, %#jx, ...) failed with "
			    "unexpected result: %d\n", part->devname,
			    (intmax_t)offset, cerr);
			return (ENXIO);
		}
	}

	/*
	 * If no valid blocks were found, the partition may be smaller than
	 * a block, or may not be block-aligned.
	 * 
	 * We can still determine the real partition size by starting our
	 * forward search at offset 0x0.
	 * 
	 * This is the case for CFE's hacked-in 'os' and 'trx' partition
	 * mappings, where 'flash0.os' and 'flash0.trx' each map only a subset
	 * of the actual TRX partition; the real TRX partition is defined by
	 * the 'flash1.trx' device.
	 */
	if (!found) {
		BCM_DISK_DBG("cfe_readblk(%s) found no valid blocks\n",
		    part->devname);
		offset = 0x0;
	}

	/* Find the actual terminating offset */
	// TODO
	part->size = offset;
	return (0);
}

/**
 * Attempt to determine @p part's size and offset.
 * 
 * @param part		The partition to be probed.
 * @param quirks	CFE driver quirks (see BCM_CFE_DRV_QUIRK_*).
 * 
 * @retval 0		success
 * @retval non-zero	if probing the partition map otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_cfe_probe_part(struct bcm_cfe_part *part, uint32_t quirks)
{
	int error;

	/* Try to determine offset/size via IOCTL_FLASH_GETINFO */
	if ((error = bcm_cfe_probe_part_flashinfo(part, quirks)))
		return (error);

	/* Fall back on IOCTL_NVRAM_GETINFO */
	if ((error = bcm_cfe_probe_part_nvraminfo(part, quirks)))
		return (error);
	
	/* If all else fails, we can manually determine the size (but not the
	 * offset) via cfe_readblk() */
	if ((error = bcm_cfe_probe_part_readsize(part, quirks)))
		return (error);

	return (0);
}
