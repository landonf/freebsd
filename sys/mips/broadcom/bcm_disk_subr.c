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

#include <machine/_inttypes.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>
#include <dev/cfe/cfe_ioctl.h>

#include "bhnd_nvram_map.h"

#include "bcm_machdep.h"

#include "bcm_diskvar.h"

static int	compare_part_offset_asc(const void *lhs, const void *rhs);


/* Known CFE bootimg layouts */
static const struct bcm_bootimg_var {
	bcm_bootimg_layout	 layout;	/**< CFE layout */
	const char		*name;		/**< NVRAM variable containing
						     active image index */
	size_t			 num_images;	/**< image count for this layout*/
} bcm_bootimg_vars[] = {
	{ BCM_BOOTIMG_FAILSAFE,	BHND_NVAR_BOOTPARTITION,	2 },
	{ BCM_BOOTIMG_DUAL,	BHND_NVAR_IMAGE_BOOT,		2 },
};

/* Image offset variables (by partition index) */
static const char *bcm_bootimg_offset_vars[] = {
	BHND_NVAR_IMAGE_FIRST_OFFSET,
	BHND_NVAR_IMAGE_SECOND_OFFSET
};

/* Known CFE OS/TRX boot partition labels */
static const struct bcm_boot_label bcm_boot_labels[] = {
	{ "os",		"trx" },
	{ "os2",	"trx2" }	/* If CFE built with
					 * FAILSAFE_UPGRADE/DUAL_IMAGE */
};



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

/**
 * Test for the existence of a usable CFE device for a partition named @p label
 * on @p disk.
 * 
 * @param disk	Disk to query.
 * @param label	Partition label.
 * 
 * @retval true		if the device exists and is a supported device type.
 * @retval false	if the device does not exist.
 * @retval false	if the device exists, but the device type is not
 *			supported.
 * @retval false	if testing for the CFE device otherwise fails.
 */
bool
bcm_disk_dev_exists(struct bcm_disk *disk, const char *label)
{
	char	dname[BCM_DISK_NAME_MAX];
	size_t	dlen;
	int	dinfo, dtype;
	int	error;

	/* Format the full CFE device name */
	dlen = sizeof(dname);
	error = bcm_disk_dev_name(disk->drvname, disk->unit, label, dname, &dlen);
	if (error) {
		BCM_DISK_ERR(disk, "failed to format device name for '%s': "
		    "%d\n", label, error);
		return (false);
	}

	/* Does the device exist? */
	if ((dinfo = cfe_getdevinfo(__DECONST(char *, dname))) < 0) {
		if (dinfo != CFE_ERR_DEVNOTFOUND) {
			BCM_DISK_ERR(disk, "cfe_getdevinfo(%s) failed: %d\n",
			    dname, dinfo);
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
		BCM_DISK_ERR(disk, "%s has unknown device type: %d\n", label,
		    dtype);
		return (false);
	}
}

/**
 * Return the CFE device name for the given driver class, unit, and partition
 * label.
 * 
 * @param		drvname	The CFE driver class name.
 * @param		unit	The CFE device unit.
 * @param		label	The CFE partition label.
 * @param[out]		buf	On success, the device name will be written to
 *				this buffer. This argment may be NULL if the
 *				value is not desired.
 * @param[in,out]	len	The capacity of @p buf. On success, will be set
 *				to the actual size of the requested value.
 *
 * @retval 0		success
 * @retval ENXIO	if an error occurs formatting the device name.
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 */
int
bcm_disk_dev_name(const char *drvname, u_int unit, const char *label,
    char *buf, size_t *len)
{
	size_t	capacity;
	int	n;

	capacity = *len;

	/* Format the full CFE device name */
	n = snprintf(buf, capacity, "%s%u.%s", drvname, unit, label);

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
 * Return a partition instance that may be used to perform CFE ioctls.
 * 
 * Will panic if @p disk has no defined partitions.
 * 
 * @param disk A fully probed disk instance.
 */
struct bcm_part *
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
 * Find the boot label entry for @p label, if any.
 *
 * @param	label		The partition label to look up.
 * @param[out]	info		On success, the boot label entry for @p label.
 * @param[out]	part_type	On success, the partition type of @p label.
 * 
 * @retval true		if @p label is a known boot partition name.
 * @retval false	if @p label is unrecognized.
 */
bool
bcm_find_boot_label(const char *label, const struct bcm_boot_label **info,
    bcm_part_type *part_type)
{
	for (size_t i = 0; i < nitems(bcm_boot_labels); i++) {
		const struct bcm_boot_label *l = &bcm_boot_labels[i];

		if (strcmp(label, l->os_label) == 0) {
			*info = l;
			*part_type = BCM_PART_TYPE_OS;

			return (true);
		}

		if (strcmp(label, l->trx_label) == 0) {
			*info = l;
			*part_type = BCM_PART_TYPE_TRX;

			return (true);
		}
	}

	/* Not found */
	return (false);
}


/**
 * Fetch the CFE boot configuration.
 */
int
bcm_get_bootinfo(struct bcm_bootinfo *info)
{
	struct bcm_platform		*bp;
	const struct bcm_bootimg_var	*imgvar;
	uint32_t			 bootflags;
	uint64_t			 imgsize;
	uint8_t				 bootimg;
	size_t				 len;
	int				 error;

	bp = bcm_get_platform();
	bootflags = bcm_get_bootflags(bp);

	/* Determine the boot device */
	if (bootflags & BHND_BOOTFLAG_KERNEL_NFLASH) {
		/* kernel (and OS) on NAND */
		info->drvname = "nflash";
		info->devunit = 0;
	} else {
		/* kernel (and OS) on NOR */
		info->drvname = "flash";
		info->devunit = 0;
	}

	/* Determine boot image layout type and the active image index */
	imgvar = NULL;
	for (size_t i = 0; i < nitems(bcm_bootimg_vars); i++) {
		/* Try to fetch the active image index from NVRAM */
		len = sizeof(bootimg);
		error = bcm_get_nvram(bp, bcm_bootimg_vars[i].name, &bootimg,
		    &len, BHND_NVRAM_TYPE_UINT8);

		if (!error) {
			/* Matched */
			imgvar = &bcm_bootimg_vars[i];
			break;
		} else if (error != ENOENT) {
			/* Return an error if the NVRAM read fails for any
			 * reason other than variable not found */
			printf("%s: error fetching '%s' boot image index "
			    "variable: %d\n", __func__,
			    bcm_bootimg_vars[i].name, error);

			return (error);
		}
	}

	/* No dual/failsafe image configuration found? */
	if (imgvar == NULL) {
		info->layout		= BCM_BOOTIMG_SIMPLE;
		info->num_images	= 1;
		info->bootimage		= 0;
		info->offsets[0]	= BCM_DISK_INVALID_OFF;
		info->sizes[0]		= BCM_DISK_INVALID_SIZE;

		return (0);
	}

	/* Found dual/failsafe image configuration */
	info->layout		= imgvar->layout;
	info->num_images	= imgvar->num_images;
	info->bootimage		= bootimg;

	/* Fetch our image offsets */
	for (size_t i = 0; i < info->num_images; i++) {
		uint64_t offset;

		KASSERT(i < nitems(info->offsets), ("bad image count: %zu", i));

		if (i >= nitems(bcm_bootimg_offset_vars)) {
			printf("%s: missing offset variable for image %zu\n",
			    __func__, i);
			return (ENXIO);
		}

		len = sizeof(info->offsets[i]);
		error = bcm_get_nvram(bp, bcm_bootimg_offset_vars[i], &offset,
		    &len, BHND_NVRAM_TYPE_UINT64);
		if (error) {
			printf("%s: error fetching offset[%zu]: %d\n", __func__,
			    i, error);
			return (error);
		}

		if (offset > OFF_MAX) {
			printf("%s: invalid offset: 0x%" PRIx64"\n", __func__,
			    offset);
			return (ERANGE);
		}

		info->offsets[i] = (off_t)offset;
	}

	/* Fetch our image size (all images have the same size) */
	len = sizeof(imgsize);
	error = bcm_get_nvram(bp, BHND_NVAR_IMAGE_SIZE, &imgsize, &len,
	    BHND_NVRAM_TYPE_UINT64);
	if (error) {
		printf("%s: error fetching image size: %d\n", __func__, error);
		return (error);
	}

	if (imgsize > OFF_MAX) {
		printf("%s: invalid image size: 0x%" PRIx64"\n", __func__,
		    imgsize);

		return (ERANGE);
	}

	for (size_t i = 0; i < info->num_images; i++)
		info->sizes[i] = (off_t)imgsize;

	return (0);
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
