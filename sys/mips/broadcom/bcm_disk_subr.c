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
#include <sys/ctype.h>

#include <machine/_inttypes.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>
#include <dev/cfe/cfe_ioctl.h>

#include "bhnd_nvram_map.h"

#include "bcm_machdep.h"

#include "bcm_diskvar.h"

static int	bcm_bootimg_init(struct bcm_bootimg *bootimg,
		    const char *offset_var, const char *size_var);

static int	compare_part_offset_asc(const void *lhs, const void *rhs);

/* Known CFE bootimg layouts and corresponding active image NVRAM variable */
static const struct bcm_bootimg_var {
        bcm_bootimg_layout       layout;
        const char              *varname;
	size_t			 num_images;
} bcm_bootimg_vars[] = {
        { BCM_BOOTIMG_FAILSAFE, BHND_NVAR_BOOTPARTITION, 2 },
        { BCM_BOOTIMG_DUAL,     BHND_NVAR_IMAGE_BOOT, 2 }
};

/* Image offset variables and assocation partition name (ordered by image
 * index) */
static const struct bcm_bootimg_offset_var {
	const char	*offset_var;
	const char	*part_label;
} bcm_bootimg_offset_vars[] = {
	{ BHND_NVAR_IMAGE_FIRST_OFFSET,		BCM_PART_LABEL_TRX },
	{ BHND_NVAR_IMAGE_SECOND_OFFSET,	BCM_PART_LABEL_TRX2 }
};

/**
 * Add @p part to the @p disk partition list.
 * 
 * @param disk	Disk to be modified.
 * @param part	Partition to add to @p disk.
 * 
 * @retval 0		success
 * @retval ENOMEM	if the maximum @p disk partition count has been
 *			reached.
 * @retval EEXIST	if a partition with the given label already exists.
 */
int
bcm_disk_add_part(struct bcm_disk *disk, struct bcm_part *part)
{
	struct bcm_part *p;

	/* Check for duplicate label */
	LIST_FOREACH(p, &disk->parts, cp_link) {
		if (strcmp(p->label, part->label) == 0)
			return (EEXIST);
	}

	if (disk->num_parts == SIZE_MAX)
		return (ENOMEM);

	disk->num_parts++;
	LIST_INSERT_HEAD(&disk->parts, part, cp_link);

	return (0);
}

/**
 * Move a partition with @p label in @p from to @p to.
 * 
 * @param from		The disk from which the partition will be moved.
 * @param to		The disk to which the partition will be moved.
 * @param label		The label of the partition to be moved.
 * @param overwrite	Replace any existing partition with @p label in @p to.
 * 
 * @retval 0		success
 * @retval EEXIST	if @p overwrite is false and @p to contains a partition
 *			with @p label.
 * @retval ENOENT	if a partition with @p label is not found in @p from.
 * @retval ENOMEM	if the maximum partition count of @p to has been
 *			reached.
 */
int
bcm_disk_move_part(struct bcm_disk *from, struct bcm_disk *to,
    const char *label, bool overwrite)
{
	struct bcm_part	*part, *exists;

	/* Fetch from source */
	if ((part = bcm_disk_get_part(from, label)) == NULL)
		return (ENOENT);

	/* Validate target disk */
	exists = bcm_disk_get_part(to, label);
	if (exists != NULL) {
		if (!overwrite)
			return (EEXIST);

		/* Remove existing partition */
		KASSERT(to->num_parts > 0, ("partition count underflow"));
		LIST_REMOVE(exists, cp_link);
		to->num_parts--;
	}

	if (to->num_parts == SIZE_MAX)
		return (ENOMEM);

	/* Remove from source disk */
	KASSERT(from->num_parts > 0, ("partition count underflow"));
	LIST_REMOVE(part, cp_link);
	from->num_parts--;

	/* Add to target disk */
	to->num_parts++;
	LIST_INSERT_HEAD(&to->parts, part, cp_link);

	return (0);
}

/**
 * Return the partition with @p label in @p disk, or NULL if not found.
 * 
 * @param disk	Disk to be searched.
 * @param label	Requested partition's label.
 */
struct bcm_part *
bcm_disk_get_part(struct bcm_disk *disk, const char *label)
{
	struct bcm_part *part;

	LIST_FOREACH(part, &disk->parts, cp_link) {
		if (strcmp(part->label, label) == 0)
			return (part);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return true if @p disk contains a partition with @p label, false otherwise.
 * 
 * @param disk	Disk to be searched.
 * @param label	Partition label to search for.
 */
bool
bcm_disk_has_part(struct bcm_disk *disk, const char *label)
{
	if (bcm_disk_get_part(disk, label) != NULL)
		return (true);

	return (false);
}

/**
 * Find the TRX or OS partition for the given bootimg index.
 * 
 * @param disk		Disk to be searched.
 * @param type		One of BCM_PART_TYPE_OS or BCM_PART_TYPE_TRX.
 * @param bootimg	The bootimg index to be searched.
 */
struct bcm_part *
bcm_disk_find_bootimg_part(struct bcm_disk *disk, bcm_part_type type,
    u_int bootimg)
{
	const char	*label;
	char		 buf[BCM_DISK_NAME_MAX];

	switch (type) {
	case BCM_PART_TYPE_OS:
		label = BCM_PART_LABEL_OS;
		break;
	case BCM_PART_TYPE_TRX:
		label = BCM_PART_LABEL_TRX;
		break;
	default:
		BCM_DISK_ERR(disk, "requested unknown bootimg partition of "
		    "type: %d\n", type);
		return (NULL);
	}

	/* The first image has no suffix; later images have an integer
	 * suffix starting at '2' */
	if (bootimg > 0) {
		int len;

		if (bootimg >= BCM_DISK_BOOTIMG_MAX || bootimg == UINT_MAX)
			return (NULL);

		len = snprintf(buf, sizeof(buf), "%s%u", label, bootimg+1);
		if (len < 0 || len >= sizeof(buf)) {
			/* Should never occur */
			BCM_DISK_ERR(disk, "snprintf() failed: %d", len);
			return (NULL);
		}

		label = buf;
	}

	return (bcm_disk_get_part(disk, label));
}

/**
 * Remove the partition with @p label from @p disk.
 * 
 * @param disk	Disk to be searched.
 * @param label	Label of the partition to be removed.
 */
void
bcm_disk_remove_part(struct bcm_disk *disk, const char *label)
{
	struct bcm_part *part;

	if ((part = bcm_disk_get_part(disk, label)) == NULL)
		return;

	KASSERT(disk->num_parts > 0, ("partition count underflow"));
	LIST_REMOVE(part, cp_link);
	disk->num_parts--;

	bcm_part_free(part);
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

	LIST_FOREACH(disk, disks, cd_link) {
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

	LIST_FOREACH(part, parts, cp_link) {
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

	LIST_FOREACH(part, parts, cp_link) {
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
 * @param type		Required type, or BCM_PART_TYPE_UNKNOWN to match on any
 *			partition type.
 */
struct bcm_part *
bcm_parts_match(struct bcm_parts *parts, const char *label,
    off_t offset, bcm_part_type type)
{
	struct bcm_part *part;

	LIST_FOREACH(part, parts, cp_link) {
		if (offset != BCM_DISK_INVALID_OFF) {
		    if (BCM_PART_HAS_OFFSET(part) && part->offset != offset)
			continue;
		}

		if (type != BCM_PART_TYPE_UNKNOWN && part->type != type)
			continue;

		if (label != NULL && strcmp(part->label, label) != 0)
			continue;

		/* Found match */
		return (part);
	}

	/* Not found */
	return (NULL);
}

/**
 * Update @p part with the given sizing metadata. The given sizing metadata will
 * first be validated as per bcm_part_validate_sizing().
 * 
 * @param disk	The disk containing @p part.
 * @param part	The partition to be probed.
 * @param fn	The probe function.
 * 
 * @retval 0		success
 * @retval ENXIO	if the probe returns invalid partition sizing metadata.
 * @retval ENXIO	if the probe returns partition sizing metadata that
 *			conflicts with the existing @p part or @p disk metadata.
 */
int
bcm_part_update_sizing(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_size *psz)
{
	int error;

	/* Validate */
	if ((error = bcm_part_validate_sizing(disk, part, psz)))
		return (error);

	/* Populate any new values */
	if (!BCM_PART_HAS_OFFSET(part) && BCM_PARTSZ_HAS_OFFSET(psz))
		part->offset = psz->offset;

	if (!BCM_PART_HAS_SIZE(part) && BCM_PARTSZ_HAS_SIZE(psz))
		part->size = psz->size;

	if (!BCM_PART_HAS_FS_SIZE(part) && BCM_PARTSZ_HAS_FS_SIZE(psz))
		part->fs_size = psz->fs_size;

	return (0);
}

/**
 * Validate @p size against @p part and @p disk.
 * 
 * Verifies that:
 *
 * - Adding the size/fs_size to the offset will not overflow.
 * - The partition range fits within the media size of @p disk (if known).
 * - Any new sizing values in @p size match (or are compatible with) existing
 *   @p part values.
 * 
 * @param disk	The disk containing @p part.
 * @param part	The partition to be probed.
 * @param fn	The probe function.
 * 
 * @retval 0		success
 * @retval ENXIO	if the probe returns invalid partition sizing metadata.
 * @retval ENXIO	if the probe returns partition sizing metadata that
 *			conflicts with the existing @p part or @p disk metadata.
 */
int
bcm_part_validate_sizing(struct bcm_disk *disk, struct bcm_part *part,
    struct bcm_part_size *psz)
{
	off_t maxsize;

	/* Do the sizing values conflict with existing partition data? */
	if (BCM_PART_HAS_SIZE(part) && BCM_PARTSZ_HAS_SIZE(psz)) {
		if (part->size != psz->size) {
			BCM_PART_ERR(part, "probe returned new size %#jx "
			    "(previous %#jx)\n", (intmax_t)psz->size,
			    (intmax_t)part->size);

			return (ENXIO);
		}
	}

	if (BCM_PART_HAS_FS_SIZE(part) && BCM_PARTSZ_HAS_FS_SIZE(psz)) {
		if (part->fs_size != psz->fs_size) {
			BCM_PART_ERR(part, "probe returned new fs_size %#jx "
			    "(previous %#jx)\n", (intmax_t)psz->fs_size,
			    (intmax_t)part->fs_size);

			return (ENXIO);
		}
	}

	if (BCM_PART_HAS_OFFSET(part) && BCM_PARTSZ_HAS_OFFSET(psz)) {
		if (part->offset != psz->offset) {
			BCM_PART_ERR(part, "probe returned new offset %#jx "
			    "(previous %#jx)\n", (intmax_t)psz->offset,
			    (intmax_t)part->offset);

			return (ENXIO);
		}
	}

	/* Does the base (offset|size|fs_size) fit within the mediasize? */
	if (BCM_DISK_HAS_SIZE(disk))
		maxsize = disk->size;
	else
		maxsize = OFF_MAX;

	if (BCM_PARTSZ_HAS_SIZE(psz) && psz->size > maxsize) {
		BCM_PART_ERR(part, "probed size %#jx exceeds media "
		    "size %#jx\n", (intmax_t)psz->size, (intmax_t)maxsize);

		return (ENXIO);
	}

	if (BCM_PARTSZ_HAS_FS_SIZE(psz) && psz->fs_size > maxsize) {
		BCM_PART_ERR(part, "probed size %#jx exceeds media "
		    "size %#jx\n", (intmax_t)psz->fs_size, (intmax_t)maxsize);

		return (ENXIO);
	}

	if (BCM_PARTSZ_HAS_OFFSET(psz) && psz->offset > maxsize) {
		BCM_PART_ERR(part, "probed offset %#jx exceeds media "
		    "size %#jx\n", (intmax_t)psz->offset, (intmax_t)maxsize);

		return (ENXIO);
	}

	/* offset+(size|fs_size) must not overflow, and must fit within the
	 * media size */
	if (BCM_PARTSZ_HAS_SIZE(psz) && BCM_PARTSZ_HAS_OFFSET(psz)) {
		/* Check for overflow */
		if (OFF_MAX - psz->offset < psz->size) {
			BCM_PART_ERR(part, "probed range %#jx+%#jx invalid\n",
			    (intmax_t)psz->offset, (intmax_t)psz->size);

			return (ENXIO);
		}

		/* Verify range */
		if (psz->offset + psz->size > maxsize) {
			BCM_PART_ERR(part, "probed range %#jx+%#jx exceeds "
			    "media size %#jx\n", (intmax_t)psz->offset,
			    (intmax_t)psz->size, (intmax_t)maxsize);

			return (ENXIO);
		}
	}

	if (BCM_PARTSZ_HAS_FS_SIZE(psz) && BCM_PARTSZ_HAS_OFFSET(psz)) {
		/* Check for overflow */
		if (OFF_MAX - psz->offset < psz->fs_size) {
			BCM_PART_ERR(part, "probed fs_range %#jx+%#jx "
			    "invalid\n", (intmax_t)psz->offset,
			    (intmax_t)psz->fs_size);

			return (ENXIO);
		}

		/* Verify range */
		if (psz->offset + psz->fs_size > maxsize) {
			BCM_PART_ERR(part, "probed fs_range %#jx+%#jx exceeds "
			    "media size %#jx\n", (intmax_t)psz->offset,
			    (intmax_t)psz->fs_size, (intmax_t)maxsize);

			return (ENXIO);
		}
	}

	return (0);
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
 * Parse a CFE disk/partition device name.
 * 
 * @param		devname		Name to parse.
 * @param[in,out]	drvname		On success, a pointer to the driver
 *					class name within @p devname.
 * @param		drvlen		On success, the length of @p drvname.
 * @param[out]		devunit		On success, the device unit.
 * @param[out]		label		On success, a pointer to the partition
 *					label string within @p devname.
 * @param		labellen	On success, the length of @p label.
 * 
 * @retval 0		success
 * @retval EFTYPE	if @p devname is not a valid CFE device name.
 */
int
bcm_disk_parse_devname(const char *devname, const char **drvname,
    size_t *drvlen, u_int *devunit, const char **label, size_t *labellen)
{
	const char	*p;
	char		*end;
	u_long		 lunit;

	/* Parse the driver class name ('<drvclass><unit>.label') */
	for (p = devname; *p != '\0' && !isdigit(*p); p++) {
		if (!isalpha(*p))
			return (EFTYPE);

		continue;
	}

	*drvname = devname;
	*drvlen = (size_t)(p - devname);

	/* Parse the device unit; it must be followed by either '.' delimiter,
	 * or end of string */
	lunit = strtoul(p, &end, 10);

	if (end == p || (*end != '\0' && *end != '.'))
		return (EFTYPE);

	if (lunit > INT_MAX || lunit > BCM_DISK_UNIT_MAX)
		return (EFTYPE);

	*devunit = lunit;
	p = end;

	/* Parse the partition label ('<devname>.<label>') */
	for (size_t i = 0; p[i] != '\0'; i++) {
		if (!isalpha(p[i]) && !isdigit(p[i]))
			return (EFTYPE);
	}

	*label = p;
	*labellen = strlen(*label);

	return (0);
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
bcm_disk_devname(const char *drvname, u_int unit, const char *label,
    char *buf, size_t *len)
{
	size_t	capacity;
	int	n;

	if (buf == NULL)
		capacity = 0x0;
	else
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
 * Return true if @p disk has a driver class and device unit matching
 * @p devunit, false otherwise.
 */
bool
bcm_disk_has_devunit(struct bcm_disk *disk, struct bcm_devunit *devunit)
{
	if (strcmp(devunit->drvname, disk->drvname) != 0)
		return (false);

	if (devunit->unit != disk->unit)
		return (false);

	return (true);
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
	struct bcm_part *part = LIST_FIRST(&disk->parts);

	if (part == NULL) {
		panic("query on non-existent disk device: %s%u", disk->drvname,
		    disk->unit);
	}

	return (part);
}

/* Populate the bootimg instance with the given NVRAM-fetched offset and size */
static int
bcm_bootimg_init(struct bcm_bootimg *bootimg, const char *offset_var,
    const char *size_var)
{
	struct bcm_platform	*bp;
	uint64_t		 offset, size;
	size_t			 len;
	int			 error;

	/* Fetch offset */
	len = sizeof(offset);
	error = bcm_get_nvram(bp, offset_var, &offset, &len,
	    BHND_NVRAM_TYPE_UINT64);
	if (error) {
		printf("%s: error reading NVRAM variable %s: %d\n", __func__,
		    offset_var, error);
		return (error);
	}

	if (offset > OFF_MAX) {
		printf("%s: %s has invalid offset: 0x%" PRIx64"\n", __func__,
		    offset_var, offset);
		return (ERANGE);
	}

	/* Fetch size */
	len = sizeof(size);
	error = bcm_get_nvram(bp, size_var, &size, &len,
	    BHND_NVRAM_TYPE_UINT64);
	if (error) {
		printf("%s: error reading NVRAM variable %s: %d\n", __func__,
		    size_var, error);
		return (error);
	}

	if (size > OFF_MAX) {
		printf("%s: %s has invalid size: 0x%" PRIx64"\n", __func__,
		    size_var, offset);
		return (ERANGE);
	}

	bootimg->offset = (off_t)offset;
	bootimg->size = (off_t)size;

	return (0);
}

/**
 * Fetch the CFE boot configuration.
 */
int
bcm_get_bootinfo(struct bcm_bootinfo *bootinfo)
{
	struct bcm_platform	*bp;
	uint32_t		 bootflags;
	size_t			 len;
	int			 error;

	bp = bcm_get_platform();
	bootflags = bcm_get_bootflags(bp);

	/* Determine the bootrom device */
	bootinfo->romdev.unit = BCM_DISK_BOOTROM_UNIT;
	if (bootflags & BHND_BOOTFLAG_BOOTROM_NFLASH)
		bootinfo->romdev.drvname = BCM_DRVNAME_NAND_FLASH;
	else
		bootinfo->romdev.drvname = BCM_DRVNAME_NOR_FLASH;

	/* Determine the OS boot device */
	bootinfo->osdev.unit = BCM_DISK_OS_UNIT;
	if (bootflags & BHND_BOOTFLAG_KERNEL_NFLASH)
		bootinfo->osdev.drvname = BCM_DRVNAME_NAND_FLASH;
	else
		bootinfo->osdev.drvname = BCM_DRVNAME_NOR_FLASH;

	/* Default to simple layout */
	_Static_assert(nitems(bootinfo->images) >= 1,
	    ("BCM_BOOTIMG_MAX invalid"));

	bootinfo->layout = BCM_BOOTIMG_SIMPLE;
	bootinfo->num_images = 1;
	bootinfo->bootimg = 0;
	bootinfo->images[0].offset = BCM_DISK_INVALID_OFF;
	bootinfo->images[0].size = BCM_DISK_INVALID_SIZE;
	bootinfo->num_failures = 0;
	bootinfo->max_failures = 0;

	/* Determine the boot image layout by requesting the layout-specific
	 * active image NVRAM variable */
	for (size_t i = 0; i < nitems(bcm_bootimg_vars); i++) {
		const struct bcm_bootimg_var	*bootvar;
		uint8_t				 bootimg;
		
		bootvar = &bcm_bootimg_vars[i];

		/* Try to fetch the active image index from NVRAM */
		len = sizeof(bootimg);
		error = bcm_get_nvram(bp, bootvar->varname, &bootimg, &len,
		    BHND_NVRAM_TYPE_UINT8);

		/* Found? */
		if (!error) {
			if (bootimg >= bootvar->num_images) {
				printf("%s: invalid boot image: %d (max %d)\n",
				    __func__, bootimg, bootvar->num_images);

				return (ENXIO);
			}

			bootinfo->layout = bootvar->layout;
			bootinfo->bootimg = bootimg;
			bootinfo->num_images = bootvar->num_images;
			break;
		}

		if (error != ENOENT) {
			/* Return an error if the NVRAM read fails for any
			 * reason other than variable not found */
			printf("%s: error fetching '%s': %d\n", __func__,
			    bootvar->varname, error);
			return (error);
		}
	}

	/* Simple layout? */
	if (bootinfo->layout == BCM_BOOTIMG_SIMPLE)
		return (0);

	/* Populate per-image info */
	for (size_t i = 0; i < bootinfo->num_images; i++) {
		const struct bcm_bootimg_offset_var *ov;

		if (i > nitems(bcm_bootimg_offset_vars)) {
			printf("%s: unsupported image index: %" PRIu8 "\n",
			    __func__, i);
			return (ENXIO);
		}
	
		ov = &bcm_bootimg_offset_vars[i];

		error = bcm_bootimg_init(&bootinfo->images[i], ov->offset_var,
		    BHND_NVAR_IMAGE_SIZE);
		if (error)
			return (error);
	}

	/* If using failsafe layout, fetch partialboot info */
	if (bootinfo->layout == BCM_BOOTIMG_FAILSAFE) {
		len = sizeof(bootinfo->num_failures);
		error = bcm_get_nvram(bp, BHND_NVAR_PARTIALBOOTS,
		    &bootinfo->num_failures, &len, BHND_NVRAM_TYPE_UINT32);
		if (error) {
			printf("%s: error fetching '%s': %d\n", __func__,
			    BHND_NVAR_PARTIALBOOTS, error);

			return (error);
		}

		len = sizeof(bootinfo->max_failures);
		error = bcm_get_nvram(bp, BHND_NVAR_MAXPARTIALBOOTS,
		    &bootinfo->max_failures, &len, BHND_NVRAM_TYPE_UINT32);
		if (error) {
			printf("%s: error fetching '%s': %d\n", __func__,
			    BHND_NVAR_MAXPARTIALBOOTS, error);

			return (error);
		}
	}

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
	bool		 comma;

	comma = false;

#define	BCM_PRINT(_expr, _fmt, ...) do {				\
	if (_expr) {							\
		printf("%s" _fmt, comma ? ", " : "", ## __VA_ARGS__);	\
		comma = true;						\
	}								\
} while (0)

#define	BCM_DISK_PRINT_FLAG(_flag, _fmt, ...)				\
	BCM_PRINT(BCM_DISK_HAS_FLAGS(disk, BCM_DISK_ ## _flag), _fmt,	\
	    ## __VA_ARGS__)

#define	BCM_PART_PRINT_FLAG(_flag, _fmt, ...)				\
	BCM_PRINT(BCM_PART_HAS_FLAGS(part, BCM_PART_ ## _flag), _fmt,	\
	    ## __VA_ARGS__)

	comma = false;
	printf("CFE disk %s%u (", disk->drvname, disk->unit);

	BCM_DISK_PRINT_FLAG(BOOTROM,		"romdev");
	BCM_DISK_PRINT_FLAG(BOOTOS,		"osdev");
	BCM_DISK_PRINT_FLAG(BYTESWAPPED,	"byteswapped");

	printf("):\n");

	/* Sort partitions by offset, ascending */
	part_idx = 0;
	parts = malloc(sizeof(*parts) * disk->num_parts, M_BCM_DISK, M_WAITOK);

	LIST_FOREACH(part, &disk->parts, cp_link) {
		KASSERT(part_idx < disk->num_parts,
		    ("incorrect partition count (%zu >= %zu)", part_idx,
		     disk->num_parts));

		parts[part_idx++] = part;
	}

	qsort(parts, disk->num_parts, sizeof(*parts),
	    compare_part_offset_asc);

	for (size_t i = 0; i < disk->num_parts; i++) {

		part = parts[i];

		printf("    %-12s", part->label);

		if (BCM_PART_HAS_OFFSET(part))
			printf("0x%08jx", (intmax_t)part->offset);
		else
			printf("%10s", "unknown");

		if (BCM_PART_HAS_SIZE(part))
			printf("+0x%08jx", (intmax_t)part->size);
		else
			printf("+%-10s", "unknown");

		/* Print partition flags */
		comma = false;
		printf(" (");

		BCM_PART_PRINT_FLAG(BOOT,		"boot");
		BCM_PART_PRINT_FLAG(NVRAM,		"nvram");
		BCM_PART_PRINT_FLAG(PLATFORM,		"required");

		if (!BCM_PART_HAS_FLAGS(part, BCM_PART_PLATFORM))
			BCM_PRINT(true, "optional");

		BCM_PART_PRINT_FLAG(UNINITIALIZED,	"uninitialized");
		BCM_PART_PRINT_FLAG(READONLY,		"readonly");

		BCM_PRINT(BCM_PART_HAS_FS_SIZE(part), "used=0x%08jx",
		    (intmax_t)part->fs_size);

		printf(")\n");
	}

	free(parts, M_BCM_DISK);

#undef	BCM_DISK_PRINT
#undef	BCM_DISK_PRINT_FLAG
#undef	BCM_PART_PRINT_FLAG
}

/**
 * Print all disks and partitions in @p disks to the console.
 */
void
bcm_print_disks(struct bcm_disks *disks)
{
	struct bcm_disk	*disk;

	LIST_FOREACH(disk, disks, cd_link)
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
