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

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>

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
 * newflash driver (CFI):
 * 
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
 * 
 *	IOCTL_FLASH_GETINFO
 *		Part Offset:	invalid	(always 0x0)
 *		Part Size:	invalid	(size of underlying flash)
 *		Flash Type:	invalid	(returns ChipCommon flash capability
 *					 bits, not CFE FLASH_TYPE_* constant)
 *		Flash Flags:	invalid	(FLASH_FLAG_NOERASE always set)
 *
 *	IOCTL_NVRAM_GETINFO
 *		NVRAM Offset:	invalid	(always 0x0)
 *		NVRAM Size:	valid
 *		NVRAM Erase:	valid
 *
 *	IOCTL_FLASH_PARTITION_INFO
 *		Part Offset:	valid
 *		Part Size:	valid
 *		Flash Type:	invalid	(ChipCommon flash capability bits)
 *		Flash Flags:	invalid	(FLASH_FLAG_NOERASE always set)
 *
 *		**NOTE**: Only available if CFE is built with command line
 *		support for 'flash -fill'; most CFE images are not.
 * 
 * nflash driver (NAND):
 * 
 *	IOCTL_FLASH_GETINFO
 *		Part Offset:	invalid	(always 0x0)
 *		Part Size:	invalid	(size of underlying flash) 
 *		Flash Type:	invalid	(ChipCommon flash capability bits)
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

static struct bcm_cfe_part	*bcm_cfe_part_new(const char *devname);
static void			 bcm_cfe_part_free(struct bcm_cfe_part *part);
static int			 bcm_cfe_probe_part(struct bcm_cfe_part *part);

#define	BCM_DISK_LOG(msg, ...)	printf("%s: " msg, __FUNCTION__, ## __VA_ARGS__)

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

static void
bcm_cfe_print_disks(struct bcm_cfe_disks *disks)
{
	struct bcm_cfe_disk *disk;
	struct bcm_cfe_part *part;

	SLIST_FOREACH(disk, disks, cd_link) {
		printf("CFE disk %s%u:\n", disk->drvname, disk->unit);

		SLIST_FOREACH(part, &disk->parts, cp_link) {
			printf("%s:\t%#08jx+%#08jx\n", part->label,
			    (intmax_t)part->offset, (intmax_t)part->size);
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
			BCM_DISK_LOG("invalid partition name: %s\n", partname);
			error = ENOMEM;
			goto failed;
		}

		/* Does the partition exist? */
		if ((dinfo = cfe_getdevinfo(dname)) < 0) {
			if (dinfo != CFE_ERR_DEVNOTFOUND) {
				BCM_DISK_LOG("cfe_getdevinfo(%s) failed: %d\n",                                                                                                                                        
				    dname, dinfo);
			}

			continue;
		}

		/* Verify device type */
		dtype = dinfo & CFE_DEV_MASK;
		switch (dtype) {
		case CFE_DEV_FLASH:
		case CFE_DEV_NVRAM:
		case CFE_DEV_DISK:
			/* Valid device type */
			break;
		default:
			BCM_DISK_LOG("%s has unknown device type: %d\n", dname,
			    dtype);
			continue;
		}

	        /* Insert a new partition entry */
		if ((part = bcm_cfe_part_new(dname)) == NULL) {
			error = ENOMEM;
			goto failed;
		}

		SLIST_INSERT_HEAD(&parts, part, cp_link);
	}

	/* Try to determine the size and offset of all discovered
	 * partitions */
	SLIST_FOREACH(part, &parts, cp_link) {
		if ((error = bcm_cfe_probe_part(part)))
			goto failed;
	}

	/* Move all partition entries to the disk entry */
	SLIST_FOREACH_SAFE(part, &parts, cp_link, pnext) {
		SLIST_REMOVE_HEAD(&parts, cp_link);
		SLIST_INSERT_HEAD(&disk->parts, part, cp_link);
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
 * @param devname CFE device name.
 * 
 * @retval non_NULL	success
 * @retval NULL		if allocation fails.
 */
static struct bcm_cfe_part *
bcm_cfe_part_new(const char *devname)
{
	struct bcm_cfe_part *p;

	p = malloc(sizeof(*p), M_BCM_CDISK, M_WAITOK|M_ZERO);

	p->devname = strdup(devname, M_BCM_CDISK);
	p->offset = OFF_MAX;
	p->size = 0x0;

	/* Parse out the partition label ('<devname>.<label>') */
	p->label = strchr(p->devname, '.');
	if (p->label != NULL) {
		/* Advance past '.' */
		p->label++;
	} else {
		p->label = "";
	}

	return (p);
}

/**
 * Free all resources held by @p part.
 */
static void
bcm_cfe_part_free(struct bcm_cfe_part *part)
{
	free(part->devname, M_BCM_CDISK);
	free(part, M_BCM_CDISK);
}


/**
 * Attempt to determine @p part's size and offset.
 * 
 * @param part	The partition to be probed.
 * 
 * @retval 0		success
 * @retval non-zero	if probing the partition map otherwise fails, a regular
 *			unix error code will be returned.
 */
static int
bcm_cfe_probe_part(struct bcm_cfe_part *part)
{
	// TODO
	return (0);
}
