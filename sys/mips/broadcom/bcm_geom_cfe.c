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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/endian.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/fcntl.h>
#include <sys/malloc.h>
#include <sys/bio.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sbuf.h>

#include <geom/geom.h>
#include <geom/geom_slice.h>

#include <dev/bhnd/cores/chipc/chipc.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>

#include "bcm_machdep.h"
#include "bhnd_nvram_map.h"

#define	CFE_CLASS_NAME		"CFE_MAP"

#define	G_CFE_LOG(msg, ...)	printf("%s: " msg, __FUNCTION__, ## __VA_ARGS__)

/*
 * Performs slicing of a ChipCommon-attached flash device based on CFE's
 * hardcoded flash partition map and conservative heuristics. 
 * 
 * In theory, we should be able to fully enumerate the flash layout using
 * cfe_enumdev() and CFE device ioctls.
 * 
 * In practice, no OEMs appear to ship a remotely modern CFE release with
 * CFE_CMD_DEV_ENUM support, requiring that we instead iterate over a set of
 * known flash device/partition names to build our partition map.
 *
 * Worse, the validity of the flash/nvram ioctls depends on the CFE flash
 * driver in use, requiring CFE driver-specific workarounds (see
 * 'CFE Driver Quirks' below).
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

/*
 * Supported CFE flash devices.
 */
static const struct cfe_flash_device {
	const char		*cfe_name;	/**< CFE device class name */
	const char		*geom_attr;	/**< GEOM device attribute */
	const chipc_flash	*flash_types;	/**< supported ChipCommon flash types */
} cfe_flash_devices[] = {
	{ "nflash",	"NAND::device",	(chipc_flash[]){
	    CHIPC_NFLASH, CHIPC_NFLASH_4706, CHIPC_FLASH_NONE } },
	{ "flash",	"CFI::device",	(chipc_flash[]){
	    CHIPC_PFLASH_CFI, CHIPC_FLASH_NONE } },
	{ "flash",	"SPI::device",	(chipc_flash[]){
	    CHIPC_SFLASH_AT, CHIPC_SFLASH_ST, CHIPC_FLASH_NONE } }
};

#if 0

/*
 * Known CFE flash partition names.
 */
static char *cfe_flash_parts[] = {
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

#endif

/**
 * CFE operating system image layout types.
 */
typedef enum {
	CFE_IMAGE_FAILSAFE,	/**< CFE with FAILSAFE_UPGRADE enabled */
	CFE_IMAGE_DUAL,		/**< CFE with DUAL_IMAGE enabled */
	CFE_IMAGE_SIMPLE	/**< CFE with default config (single image) */
} cfe_image_layout;

/* NVRAM variables defining the currently selected CFE OS boot image */
static const struct cfe_image_defn {
	const char		*nvar;		/**< NVRAM variable name */
	cfe_image_layout	 layout;	/**< layout type */
	size_t			 num_images;	/**< image count */
} cfe_image_defns[] = {
	{ BHND_NVAR_BOOTPARTITION,	CFE_IMAGE_FAILSAFE,	2},
	{ BHND_NVAR_IMAGE_BOOT,		CFE_IMAGE_DUAL,		2},
};

/* Image offset variables (by partition index) */
static const char *cfe_image_offset_vars[] = {
	BHND_NVAR_IMAGE_FIRST_OFFSET,
	BHND_NVAR_IMAGE_SECOND_OFFSET
};

#define	CFE_MAX_IMG	2	/**< maximum CFE image count */

/**
 * CFE operating system image info.
 */
struct cfe_image_info {
	cfe_image_layout	layout;			/**< CFE layout type */
	uint8_t			bootimage;		/**< boot image index */
	size_t			num_images;		/**< image count */
	uint64_t		offsets[CFE_MAX_IMG];	/**< image offsets */
	uint64_t		sizes[CFE_MAX_IMG];	/**< image sizes */
};

/* Standard CFE image */
#define	CFE_MAGIC_OFFSET	0x4E0
#define	CFE_MAGIC_COUNT		2
#define	CFE_MAGIC_0		0
#define	CFE_MAGIC_1		1
#define	CFE_MAGIC		0x43464531	/* 'CFE1' */

/* Self-decompressing CFE image */
#define	CFE_BISZ_OFFSET		0x3E0
#define	CFE_BISZ_MAGIC		0x4249535A	/* 'BISZ' */

struct g_cfe_softc {
};

static int
g_cfe_access(struct g_provider *pp, int dread, int dwrite, int dexcl)
{
	struct g_geom *gp;
	struct g_consumer *cp;

	gp = pp->geom;
	cp = LIST_FIRST(&gp->consumer);

	// TODO: block writes to boot loader

	return (g_access(cp, dread, dwrite, dexcl));
}

#if 0
static int
g_cfe_start(struct bio *bp)
{
	switch (bp->bio_cmd) {
	case BIO_GETATTR:
		// TODO
		// return (1);
		return (0);
	default:
		return (0);
	}
}
#endif

static void
g_cfe_dumpconf(struct sbuf *sb, const char *indent, struct g_geom *gp,
    struct g_consumer *cp, struct g_provider *pp)
{
	// TODO: plaintext output?
	// TODO: CFE-specific attributes?
	g_slice_dumpconf(sb, indent, gp, cp, pp);
}


static void
g_cfe_orphan_taste(struct g_consumer *cp)
{
	struct g_geom		*gp;
	struct g_provider	*pp;

	g_trace(G_T_TOPOLOGY, "cfe_orphan_taste(%p/%s)", cp,
	    cp->provider->name);
	g_topology_assert();

	gp = cp->geom;
	gp->flags |= G_GEOM_WITHER;
	LIST_FOREACH(pp, &gp->provider, provider) {
		g_wither_provider(pp, ENXIO);
	}
}

static void
g_cfe_start_taste(struct bio *bp)
{
	g_io_deliver(bp, EOPNOTSUPP);
}

static int
g_cfe_access_taste(struct g_provider *pp, int dread, int dwrite, int dexcl)
{
	return (EOPNOTSUPP);
}

/**
 * Return the ChipCommon device to which @p dev is attached, or NULL if
 * not attached via ChipCommon.
 */
static device_t
g_cfe_find_chipc_parent(device_t dev)
{
	device_t	parent;
	devclass_t	chipc_class, bhnd_class;

	mtx_assert(&Giant, MA_OWNED);

	if ((chipc_class = devclass_find("bhnd_chipc")) == NULL) {
		G_CFE_LOG("missing chipcommon device class\n");
		return (NULL);
	}

	if ((bhnd_class = devclass_find("bhnd")) == NULL) {
		G_CFE_LOG("missing bhnd device class\n");
		return (NULL);
	}

	parent = dev;
	while ((parent = device_get_parent(parent)) != NULL) {
		devclass_t cls = device_get_devclass(parent);

		/* If we hit the bhnd(4) bus, we've gone too far */
		if (cls == bhnd_class)
			return (NULL);

		/* Found our ChipCommon parent */
		if (cls == chipc_class)
			return (parent);
	}

	/* Not attached via ChipCommon */
	return (NULL);
}

/**
 * Return true if @p id declares support for flash @p type, false otherwise.
 */
static bool
g_cfe_flash_type_matches(chipc_flash type, const struct cfe_flash_device *id)
{
	const chipc_flash *req;

	for (req = id->flash_types; *req != CHIPC_FLASH_NONE; req++) {
		if (*req == type)
			return (true);
	}

	return (false);
}

/**
 * Return the first matching device table entry for @p cp, or NULL
 * if no matching device is found.
 * 
 * @param cp The GEOM consumer to match against the supported device table.
 */
static const struct cfe_flash_device *
g_cfe_device_lookup(struct g_consumer *cp)
{
	int error;

	mtx_lock(&Giant);	/* for newbus */

	/* Try to fetch the backing device */
	for (size_t i = 0; i < nitems(cfe_flash_devices); i++) {
		const struct cfe_flash_device	*id;
		const struct chipc_caps		*ccaps;
		device_t			 chipc, dev;
		size_t				 len;

		id = &cfe_flash_devices[i];
		len = sizeof(dev);

		/* Try to fetch the backing device */
		error = g_io_getattr(id->geom_attr, cp, &len, &dev);
		if (error)
			continue;

		/* Must be attached via ChipCommon */
		if ((chipc = g_cfe_find_chipc_parent(dev)) == NULL)
			continue;

		/* ChipCommon's advertised flash type must match the device
		 * table entry */
		ccaps = BHND_CHIPC_GET_CAPS(chipc);

		if (!g_cfe_flash_type_matches(ccaps->flash_type, id)) {
			G_CFE_LOG("unknown chipc flash type: %d\n",
			    ccaps->flash_type);
			continue;
		}

		/* Match! */
		mtx_unlock(&Giant);
		return (id);
	}

	/* No match */
	mtx_unlock(&Giant);
	return (NULL);
}

/**
 * Read and validate the CFE image magic.
 * 
 * @retval 0		success
 * @retval non-zero	if probing fails, a regular unix error code will be
 *			returned.
 */
static int
g_cfe_check_magic(struct g_consumer *cp)
{
	u_char		*buf;
	off_t		 sectorsize;
	uint32_t	*magic;
	int		 error;

	g_topology_assert();
	sectorsize = cp->provider->sectorsize;

	/* Standard CFE image? */
	g_topology_unlock();
	buf = g_read_data(cp, rounddown(CFE_MAGIC_OFFSET, sectorsize),
	    roundup(sizeof(*magic) * CFE_MAGIC_COUNT, sectorsize), &error);
	g_topology_lock();

	if (buf == NULL)
		return (error);

	magic = (uint32_t *)(buf + (CFE_MAGIC_OFFSET % sectorsize));
	if (magic[CFE_MAGIC_0] == CFE_MAGIC &&
	    magic[CFE_MAGIC_1] == CFE_MAGIC)
	{
		g_free(buf);
		return (0);
	}

	/* Self-decompressing CFE image? */
	g_free(buf);
	g_topology_unlock();
	buf = g_read_data(cp, rounddown(CFE_BISZ_OFFSET, sectorsize),
	    roundup(sizeof(*magic), sectorsize), &error);
	g_topology_lock();

	if (buf == NULL)
		return (error);

	magic = (uint32_t *)(buf + (CFE_BISZ_OFFSET % sectorsize));
	if (*magic == CFE_BISZ_MAGIC) {
		g_free(buf);
		return (0);
	}

	/* Unrecognized */
	return (ENXIO);
}

/**
 * Parse all partitions.
 */
static int
g_cfe_read_parts(struct g_consumer *cp)
{
	int error;

	g_topology_assert();

	if ((error = g_cfe_check_magic(cp))) {
		G_CFE_LOG("unknown CFE image format\n");
		return (error);
	}

	return (0);
}

static int
g_cfe_get_bootflags(struct bcm_platform *bp, uint32_t *bootflags)
{
	size_t	len;
	int	error;

	len = sizeof(*bootflags);
	error = bcm_get_nvram(bp, BHND_NVAR_BOOTFLAGS, bootflags, &len,
	    BHND_NVRAM_TYPE_UINT32);
	if (error == ENOENT) {
		/* Not found; default to no flags set */
		*bootflags = 0x0;
		error = 0;
	}

	return (error);
}

/**
 * Populate @p info with the CFE image layout.
 */
static int
g_cfe_get_image_info(struct cfe_image_info *info)
{
	struct bcm_platform		*bp;
	const struct cfe_image_defn	*defn;
	uint32_t			 bootflags;
	uint8_t				 bootimage;
	uint64_t			 imagesize;
	size_t				 len;
	int				 error;

	bp = bcm_get_platform();

	/* Determine layout type and boot image index */
	defn = NULL;
	for (size_t i = 0; i < nitems(cfe_image_defns); i++) {
		/* Try to fetch the boot image index from NVRAM */
		len = sizeof(bootimage);
		error = bcm_get_nvram(bp, cfe_image_defns[i].nvar, &bootimage,
		    &len, BHND_NVRAM_TYPE_UINT8);

		if (!error) {
			/* Matched */
			defn = &cfe_image_defns[i];
			break;
		} else if (error != ENOENT) {
			/* Return an error if the NVRAM read fails for any
			 * reason other than variable not found */
			return (error);
		}
	}

	if (defn == NULL) {
		/* No image support */
		info->layout = CFE_IMAGE_SIMPLE;
		info->num_images = 0;
		info->bootimage = 0;
		return (0);
	} else {
		/* Images supported by CFE */
		info->layout = defn->layout;
		info->num_images = defn->num_images;
		info->bootimage = bootimage;
	}

	/* Fetch the boot flags */
	if ((error = g_cfe_get_bootflags(bp, &bootflags))) {
		G_CFE_LOG("error reading bootflags: %d\n", error);
		return (error);
	}

	/* Probe image offsets */
	for (size_t i = 0; i < info->num_images; i++) {
		KASSERT(i < nitems(info->offsets), ("bad image count: %zu", i));

		if (i >= nitems(cfe_image_offset_vars))
			return (ENXIO);

		len = sizeof(info->offsets[i]);
		error = bcm_get_nvram(bp, cfe_image_offset_vars[i],
		    &info->offsets[i], &len, BHND_NVRAM_TYPE_UINT64);
		if (error) {
			G_CFE_LOG("error fetching offset[%zu]: %d\n", i, error);
			return (error);
		}
	}

	/* Determine image size */
	len = sizeof(imagesize);
	error = bcm_get_nvram(bp, BHND_NVAR_IMAGE_SIZE, &imagesize, &len,
	    BHND_NVRAM_TYPE_UINT64);
	if (error == ENOENT) {
		
	} else if (error) {
		G_CFE_LOG("error fetching image size: %d\n", error);
		return (error);
	}

	// TODO: image size

	return (0);
}

static struct g_geom *
g_cfe_taste(struct g_class *mp, struct g_provider *pp, int insist)
{
	struct g_consumer	*cp;
	struct g_geom		*gp;
	struct cfe_image_info	 info;
	int			 error;

	g_trace(G_T_TOPOLOGY, "cfe_taste(%s,%s)", mp->name, pp->name);
	g_topology_assert();

	/* Don't recurse */
	if (strcmp(pp->geom->class->name, CFE_CLASS_NAME) == 0)
		return (NULL);

	/*
	 * Add a consumer to the GEOM topology, acquiring read access to
	 * the provider.
	 */
	gp = g_new_geomf(mp, pp->name);
	gp->orphan = g_cfe_orphan_taste;
	gp->start = g_cfe_start_taste;
	gp->access = g_cfe_access_taste;
	cp = g_new_consumer(gp);
	cp->flags |= G_CF_DIRECT_SEND | G_CF_DIRECT_RECEIVE;
	error = g_attach(cp, pp);
	if (error == 0)
		error = g_access(cp, 1, 0, 0);

	if (error) {
		g_wither_geom(gp, ENXIO);
		return (NULL);
	}

	/* Match against our device table */
	if (g_cfe_device_lookup(cp) == NULL)
		goto failed;

	/* Fetch CFE image layout */
	if ((error = g_cfe_get_image_info(&info))) {
		G_CFE_LOG("error fetching CFE image info: %d\n", error);
		goto failed;
	}

	/* Read our partition map */
	if ((error = g_cfe_read_parts(cp))) {
		G_CFE_LOG("error parsing CFE image: %d\n", error);
		goto failed;
	}

	// TODO
	printf("MATCH\n");

failed:	
	/* No match */
	g_access(cp, -1, 0, 0);
	g_detach(cp);
	g_destroy_consumer(cp);
	g_destroy_geom(gp);
	return (NULL);
}

static void
g_cfe_config(struct gctl_req *req, struct g_class *mp, const char *verb)
{

}

static struct g_class g_cfe_class = {
	.name		= CFE_CLASS_NAME,
	.version	= G_VERSION,
	.taste		= g_cfe_taste,
	.access		= g_cfe_access,
	.dumpconf	= g_cfe_dumpconf,
	.ctlreq		= g_cfe_config,
};
DECLARE_GEOM_CLASS(g_cfe_class, g_map);
