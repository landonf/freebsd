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
#include <sys/sysctl.h>

#include <machine/_inttypes.h>

#include <geom/geom.h>
#include <geom/geom_slice.h>

#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/cores/chipc/chipc.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>
#include <dev/cfe/cfe_ioctl.h>

#include "bcm_machdep.h"
#include "bhnd_nvram_map.h"

#include "bcm_geom_cfe.h"

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
 * As a result, we probe a set of known CFE partitions to determine the
 * space useable by the OS.
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

static device_t				 g_cfe_find_chipc_parent(device_t dev);
static bool				 g_cfe_flash_type_matches(
					     const struct cfe_flash_device *id,
					     chipc_flash type);

static int				 g_cfe_new_probe(
					     struct cfe_flash_probe **probe,
					     struct g_consumer *cp,
					     const struct cfe_flash_device *dev,
					     u_int cfe_unit,
					     const char *cfe_part);
static void				 g_cfe_free_probe(
					     struct cfe_flash_probe *probe);

static const struct cfe_flash_device	*g_cfe_device_lookup(
					     struct g_consumer *cp);
static int				 g_cfe_check_magic(
					     struct g_consumer *cp,
					     off_t cfe_offset);
static int				 g_cfe_read_parts(
					     struct g_consumer *cp);
static int				 g_cfe_get_bootimg_info(
					     struct cfe_bootimg_info *info);

/* CFE flash probe functions */
static g_cfe_probe_func			 g_cfe_probe_flash_info;
static g_cfe_probe_func			 g_cfe_probe_nvram_info;
static g_cfe_probe_func			 g_cfe_fallback_size_probe;
static g_cfe_probe_func			 g_cfe_probe_part_boot;


#define G_CFE_PROBE_FUNC(_name, _pass)	\
	{ __STRING(_name), _name, _pass }
/**
 * Table of all CFE flash probe functions, ordered by application priorty.
 */
static const struct g_cfe_probe_func_info {
	const char		*desc;
	g_cfe_probe_func	*fn;
	u_int			 pass;
} g_cfe_probe_funcs[] = {
	G_CFE_PROBE_FUNC(g_cfe_probe_flash_info,	0),
	G_CFE_PROBE_FUNC(g_cfe_probe_nvram_info,	0),
	G_CFE_PROBE_FUNC(g_cfe_fallback_size_probe,	1),
	G_CFE_PROBE_FUNC(g_cfe_probe_part_boot,		2),
};

/*
 * Supported CFE/GEOM flash devices.
 */
static const struct cfe_flash_device cfe_flash_devices[] = {
	{ "nflash",	"NAND::device", (chipc_flash[]) {
	    CHIPC_NFLASH,	CHIPC_NFLASH_4706,	CHIPC_FLASH_NONE },
	    (G_CFE_QUIRK_FLASH_ZERO_OFF | G_CFE_QUIRK_FLASH_TOTAL_SIZE |
	     G_CFE_QUIRK_NVRAM_UNAVAIL)
	},

	{ "flash",	"CFI::device",	(chipc_flash[]){
	    CHIPC_PFLASH_CFI,	CHIPC_FLASH_NONE },
	    G_CFE_QUIRK_NVRAM_PART_SIZE
	},

	{ "flash",	"SPI::device",	(chipc_flash[]){
	    CHIPC_SFLASH_AT,	CHIPC_SFLASH_ST,	CHIPC_FLASH_NONE },
	    (G_CFE_QUIRK_FLASH_ZERO_OFF | G_CFE_QUIRK_FLASH_TOTAL_SIZE |
	     G_CFE_QUIRK_NVRAM_PART_SIZE)
	}
};

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

/**
 * Map of CFE image layout types to their associated OS boot image index
 * NVRAM variable.
 */
static const struct cfe_bootimg_var {
	cfe_bootimg_type	 type;		/**< layout type */
	const char		*nvar;		/**< NVRAM variable name */
	size_t			 num_images;	/**< image count */
} cfe_bootimg_vars[] = {
	{ CFE_IMAGE_FAILSAFE,	BHND_NVAR_BOOTPARTITION,	2 },
	{ CFE_IMAGE_DUAL,	BHND_NVAR_IMAGE_BOOT,		2 },
};

/* Image offset variables (by partition index) */
static const char *cfe_bootimg_offset_vars[] = {
	BHND_NVAR_IMAGE_FIRST_OFFSET,
	BHND_NVAR_IMAGE_SECOND_OFFSET
};

/* Debugging flags */
enum {
	G_CFE_DEBUG_PROBE = 1 << 0,
};

static u_long g_cfe_debug = 0 /* XXX */ | G_CFE_DEBUG_PROBE;

#define	G_CFE_DEBUG_EN(_type)	((G_CFE_DEBUG_ ## _type & g_cfe_debug) != 0)

SYSCTL_DECL(_kern_geom);
SYSCTL_NODE(_kern_geom, OID_AUTO, bcmcfe, CTLFLAG_RW, 0, "BCM_CFE");
SYSCTL_UINT(_kern_geom_bcmcfe, OID_AUTO, debug, CTLFLAG_RWTUN,
    &g_cfe_debug, 0, "Debug flags");

#define	G_CFE_LOG(msg, ...)	printf("%s: " msg, __FUNCTION__, ## __VA_ARGS__)
#define	G_CFE_DEBUG(flg, msg, ...)	do {		\
	if (G_CFE_DEBUG_EN(flg))			\
		G_CFE_LOG(msg, ## __VA_ARGS__);		\
} while (0)

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

static struct g_geom *
g_cfe_taste(struct g_class *mp, struct g_provider *pp, int insist)
{
	struct g_consumer		*cp;
	struct g_geom			*gp;
	const struct cfe_flash_device	*cfe_dev;
	struct cfe_flash_probe		*probe, *pnext;
	struct cfe_bootimg_info		 info;
	int				 error;
	TAILQ_HEAD(,cfe_flash_probe)	 probes;

	g_trace(G_T_TOPOLOGY, "cfe_taste(%s,%s)", mp->name, pp->name);
	g_topology_assert();

	/* Don't recurse */
	if (strcmp(pp->geom->class->name, CFE_CLASS_NAME) == 0)
		return (NULL);

	TAILQ_INIT(&probes);

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
	if ((cfe_dev = g_cfe_device_lookup(cp)) == NULL)
		goto failed;

	/* Look for known partitions on unit 0 */
	for (size_t i = 0; i < nitems(cfe_flash_parts); i++) {
		error = g_cfe_new_probe(&probe, cp, cfe_dev, 0,
		    cfe_flash_parts[i]);
		if (!error) {
			TAILQ_INSERT_TAIL(&probes, probe, fp_link);
		} else {
			G_CFE_DEBUG(PROBE, "g_cfe_new_probe(%s): %d\n",
			    cfe_dev->cfe_name, error);
		}
	}

	/* Fetch CFE boot image info */
	if ((error = g_cfe_get_bootimg_info(&info))) {
		G_CFE_LOG("error fetching CFE image info: %d\n", error);
		goto failed;
	}

	/* Read our partition map */
	if ((error = g_cfe_read_parts(cp))) {
		G_CFE_LOG("error parsing CFE image: %d\n", error);
		goto failed;
	}

	// TODO
	TAILQ_FOREACH(probe, &probes, fp_link) {
		if (probe->have_offset && probe->have_size) {
			G_CFE_LOG("%s (base=%#jx, size=%#jx)\n", probe->dname,
			    (intmax_t)probe->offset, (intmax_t)probe->size);
		} else if (probe->have_offset) {
			G_CFE_LOG("%s (base=%#jx, size=unk)\n", probe->dname,
			    (intmax_t)probe->offset);
		} else if (probe->have_size) {
			G_CFE_LOG("%s (base=unk, size=%#jx)\n", probe->dname,
			    (intmax_t)probe->size);
		} else {
			G_CFE_LOG("%s (base=unk, size=unk)\n", probe->dname);
		}
	}

	printf("MATCH\n");

failed:
	TAILQ_FOREACH_SAFE(probe, &probes, fp_link, pnext) {
		TAILQ_REMOVE(&probes, probe, fp_link);
		g_cfe_free_probe(probe);
	}

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

/**
 * Attempt to populate @p probe with IOCTL_FLASH_GETINFO.
 */
static int
g_cfe_probe_flash_info(struct cfe_flash_probe *probe)
{
	flash_info_t	fi;
	int		cerr, rlen;

	/* Skip? */
	if (G_CFE_QUIRK(probe->dev, FLASH_INV_OFF) &&
	    G_CFE_QUIRK(probe->dev, FLASH_INV_SIZE))
		return (ENXIO);

	/* Fetch and populate the flash data */
	cerr = cfe_ioctl(probe->fd, IOCTL_FLASH_GETINFO, (u_char *)&fi,
	    sizeof(fi), &rlen, 0);
	if (cerr != CFE_OK) {
		if (cerr != CFE_ERR_INV_COMMAND)
			G_CFE_LOG("%s IOCTL_FLASH_GETINFO failed %d\n",
			    probe->dname, cerr);

		return (ENXIO);
	}

	if (!G_CFE_QUIRK(probe->dev, FLASH_INV_OFF)) {
#if ULLONG_MAX > OFF_MAX
		if (fi.flash_base > OFF_MAX) {
			G_CFE_LOG("CFE %s flash base %#llx exceeds maximum "
			    "supported offset\n", probe->dname, fi.flash_base);
			return (ENXIO);
		}
#endif

		probe->offset = (off_t)fi.flash_base;
		probe->have_offset = true;
	}

	if (!G_CFE_QUIRK(probe->dev, FLASH_INV_SIZE)) {
#if UINT_MAX > OFF_MAX
		if (fi.flash_size > OFF_MAX) {
			G_CFE_LOG("CFE %s flash size %#llx exceeds maximum "
			    "supported size\n", probe->dname, fi.flash_base);
			return (ENXIO);
		}
#endif

		probe->size = (off_t)fi.flash_size;
		probe->have_size = true;
	}

	return (0);
}

/**
 * Attempt to populate @p probe with IOCTL_NVRAM_GETINFO.
 */
static int
g_cfe_probe_nvram_info(struct cfe_flash_probe *probe)
{
	nvram_info_t	ni;
	int		cerr, rlen;

	/* Skip if IOCTL_NVRAM_GETINFO does not provide the partition size */
	if (!G_CFE_QUIRK(probe->dev, NVRAM_PART_SIZE))
		return (ENXIO);

	/* Fetch and populate the partition's size info */
	cerr = cfe_ioctl(probe->fd, IOCTL_NVRAM_GETINFO, (u_char *)&ni,
	    sizeof(ni), &rlen, 0);
	if (cerr != CFE_OK) {
		if (cerr != CFE_ERR_INV_COMMAND)
			G_CFE_LOG("%s IOCTL_NVRAM_GETINFO failed %d\n",
			    probe->dname, cerr);

		return (ENXIO);
	}

	if (ni.nvram_size < 0) {
		G_CFE_LOG("CFE returned invalid nvram size (%d) for %s\n",
		    ni.nvram_size, probe->dname);
		return (ENXIO);
	}

#if INT_MAX > OFF_MAX
	if (ni.nvram_size > OFF_MAX) {
		G_CFE_LOG("CFE returned invalid nvram size (%d) for %s\n",
		    ni.nvram_size, probe->dname);
		return (ENXIO);
	}
#endif

	probe->size = (off_t)ni.nvram_size;
	probe->have_size = true;

	return (0);
}

/**
 * Attempt to populate @p probe's partition size by performing sector-aligned
 * zero length reads.
 */
static int
g_cfe_fallback_size_probe(struct cfe_flash_probe *probe)
{
	int64_t	cfe_offset;
	u_char	buf[1];
	bool	log_readblk_err;
	int	cerr;

	log_readblk_err = G_CFE_DEBUG_EN(PROBE);
	cfe_offset = (int64_t)probe->mediasize;

#if OFF_MAX > INT64_MAX
	if (probe->mediasize > INT64_MAX)
		cfe_offset = INT64_MAX;
#endif

	while (cfe_offset > probe->blksize && probe->blksize > 0) {
		cerr = cfe_readblk(probe->fd, cfe_offset, buf, sizeof(buf));

		if (cerr == CFE_ERR_IOERR) {
			/* Continue search at next block */
			cfe_offset -= probe->blksize;

			continue;

		} else if (cerr == CFE_ERR_EOF || cerr == 0) {
			/* Found partition EOF */
			probe->size = (off_t)cfe_offset;
			probe->have_size = true;

			return (0);

		} else if (cerr < 0) {
			/* Read failed with an unknown error; this is expected
			 * on some earlier SENTRY5 CFE releases, where reading
			 * past EOF may trigger an overflow */
			if (log_readblk_err) {
				G_CFE_LOG("cfe_readblk(%s, %#jx, ...) failed "
				    "with unexpected error: %d\n", probe->dname,
				    cfe_offset, cerr);

				/* Only log the first failure */
				log_readblk_err = false;
			}

			cfe_offset -= probe->blksize;

			continue;

		} else if (cerr > 0) {
			/* Read succeeded. If we previously read an invalid
			 * block, this is the last valid block in the
			 * partition */
			if (cfe_offset < probe->mediasize) {
				probe->size = cfe_offset + probe->blksize;
				probe->have_size = true;
				return (0);
			}

			/* If this is our first read, our media size must be
			 * incorrect; we calculated an invalid maximum read
			 * offset */
			G_CFE_LOG("%s partition extends beyond maximum "
			    "expected CFE read offset %#" PRIx64 "\n",
			    probe->dname, cfe_offset);

			return (ENXIO);
		}
	}

	/* Failed to locate final valid block */
	return (ENXIO);
}

/**
 * If @p probe's is a CFE bootloader partition, provide the expected zero
 * offset.
 */
static int
g_cfe_probe_part_boot(struct cfe_flash_probe *probe)
{
	/* Must be a boot partition */
	if (strcmp(probe->pname, "boot") != 0)
		return (ENXIO);

	/* Skip if an offset was already probed */
	if (probe->have_offset) {
		if (probe->offset != 0) {
			G_CFE_LOG("%s has unexpected non-zero offset: %#jx\n",
			    probe->dname, (intmax_t)probe->offset);
		}

		return (0);
	}

	probe->have_offset = true;
	probe->offset = 0x0;

	return (0);
}

/**
 * Apply the given probe function to @p probe and validate the result. If
 * the probe function fails, or returns an invalid result, @p probe will be
 * left unmodified.
 * 
 * @param[in,out]	probe	Probe state.
 * @param		pfi	Probe function info.
 * 
 * @retval 0		success
 * @retval ENODEV	if the specified device/partition does not exist.
 * @retval ENXIO	if the probe otherwise fails.
 */
static int
g_cfe_try_probe(struct cfe_flash_probe *probe,
    const struct g_cfe_probe_func_info *pfi)
{
	struct cfe_flash_probe	cp;
	int			error;

	/* Execute with a local copy of the probe data */
	cp = *probe;
	if ((error = pfi->fn(&cp))) {
		G_CFE_DEBUG(PROBE, "%s(%s): %d\n", pfi->desc, probe->dname,
		    error);
		return (error);
	}

	/* Validate probed offset */
	if (cp.have_offset) {
		/* Must fit within total media size */
		if (cp.mediasize <= cp.offset) {
			G_CFE_LOG("%s: %s returned invalid offset %#jx "
			    "(mediasize=%#jx)\n", cp.dname, pfi->desc,
			    cp.offset, cp.mediasize);

			cp.have_offset = false;
		}
	}

	/* Validate probed size (+offset, if available) */
	if (cp.have_size) {
		off_t offset;

		/* Must fit within total media size */
		offset = 0x0;
		if (cp.have_offset)
			offset = cp.offset;

		if (offset >= cp.mediasize) {
			G_CFE_LOG("%s: %s invalid offset %#jx "
			    "(mediasize=%#jx)\n", cp.dname, pfi->desc,
			    cp.offset, cp.mediasize);

			cp.have_size = false;
		}

		if (cp.mediasize - offset < cp.size) {
			G_CFE_LOG("%s: %s %#jx+%#jx exceeds media size"
			    " %#jx\n", cp.dname, pfi->desc, cp.offset, cp.size,
			    cp.mediasize);

			cp.have_size = false;
		}
	}

	/* Save validated results */
	if (cp.have_offset) {
		probe->have_offset = true;
		probe->offset = cp.offset;
	}

	if (cp.have_size) {
		probe->have_size = true;
		probe->size = cp.size;
	}

	return (0);
}

/**
 * Allocate, initialize, and return a new CFE flash prove instance
 * for the specified CFE device.
 * 
 * @param[out]	probe		On success, a newly allocated probe instance.
 * @param	cp		Flash GEOM consumer.
 * @param	dev		CFE device description.
 * @param	unit		CFE device unit.
 * @param	part		CFE partition name.
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval ENODEV	if the specified device/partition does not exist.
 * @retval ENXIO	if probing for the specified device otherwise fails.
 */
static int
g_cfe_new_probe(struct cfe_flash_probe **probe,
    struct g_consumer *cp, const struct cfe_flash_device *cfe_dev,
    u_int cfe_unit, const char *cfe_part)
{
	struct cfe_flash_probe	*p;
	int			 error, n;

	p = malloc(sizeof(*p), M_BHND, M_WAITOK|M_ZERO);
	p->dev = cfe_dev;
	p->unit = cfe_unit;
	p->fd = -1;
	p->cp = cp;

	g_topology_assert();
	p->mediasize = cp->provider->mediasize;
	p->blksize = cp->provider->sectorsize;

	/* Format the full CFE device name */
	n = snprintf(p->dname, sizeof(p->dname), "%s%u.%s",
	    cfe_dev->cfe_name, cfe_unit, cfe_part);

	if (n >= sizeof(p->dname)) {
		g_cfe_free_probe(p);
		return (ENOMEM);
	}

	/* Populate the CFE partition name */
	if (strlen(cfe_part) >= sizeof(p->pname)) {
		g_cfe_free_probe(p);
		return (ENOMEM);
	}

	strlcpy(p->pname, cfe_part, sizeof(p->pname));

	/* Try to open the device */
	if ((p->fd = cfe_open(p->dname)) < 0) {
		if (p->fd == CFE_ERR_DEVNOTFOUND) {
			error = ENODEV;
		} else {
			G_CFE_LOG("error opening CFE device %s: %d\n", p->dname,
			    p->fd);
			error = ENXIO;
		}
 
		g_cfe_free_probe(p);
		return (ENXIO);
	}

	/* Try to populate partition info */
	for (u_int pass = 0, final_pass = false; !final_pass; pass++) {
		final_pass = true;

		for (size_t i = 0; i < nitems(g_cfe_probe_funcs); i++) {
			const struct g_cfe_probe_func_info *pfi;

			pfi = &g_cfe_probe_funcs[i];

			/* Applies to current pass? */
			if (pfi->pass != pass) {
				if (pfi->pass > pass)
					final_pass = false;

				continue;
			}

			error = g_cfe_try_probe(p, pfi);
			G_CFE_DEBUG(PROBE, "%s(%s): %d\n", pfi->desc, p->dname, error);

			if (error)
				continue;

			if (p->have_offset)
				G_CFE_DEBUG(PROBE, "\toffset:\t%#jx\n",
				    (intmax_t)p->offset);

			if (p->have_size)
				G_CFE_DEBUG(PROBE, "\tsize:\t%#jx\n",
				    (intmax_t)p->size);
		}
	}

	*probe = p;
	return (0);
}

static void
g_cfe_free_probe(struct cfe_flash_probe *probe)
{
	if (probe->fd >= 0)
		cfe_close(probe->fd);

	free(probe, M_BHND);
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
g_cfe_flash_type_matches(const struct cfe_flash_device *id, chipc_flash type)
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

		if (!g_cfe_flash_type_matches(id, ccaps->flash_type)) {
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
 * @param cp		GEOM consumer containing CFE image.
 * @param cfe_offset	Offset to CFE image within @p cp.
 * 
 * @retval 0		success
 * @retval non-zero	if the CFE image cannot be identified, a regular unix
 *			error code will be returned.
 */
static int
g_cfe_check_magic(struct g_consumer *cp, off_t cfe_offset)
{
	u_char		*buf;
	off_t		 nbytes, sector, sectorsize;
	uint32_t	*magic;
	int		 error;

	g_topology_assert();
	sectorsize = cp->provider->sectorsize;

	/* Standard CFE image */
	if (OFF_MAX - cfe_offset >= CFE_MAGIC_OFFSET) {
		sector = rounddown(cfe_offset + CFE_MAGIC_OFFSET, sectorsize);
		nbytes = roundup(sizeof(*magic) * CFE_MAGIC_COUNT, sectorsize);

		g_topology_unlock();
		buf = g_read_data(cp, sector, nbytes, &error);
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

		/* No match */
		g_free(buf);
		buf = NULL;
	}


	/* Self-decompressing CFEZ image */
	if (OFF_MAX - cfe_offset >= CFE_BISZ_OFFSET) {
		sector = rounddown(cfe_offset + CFE_BISZ_OFFSET, sectorsize);
		nbytes = roundup(sizeof(*magic), sectorsize);

		g_topology_unlock();
		buf = g_read_data(cp, sector, nbytes, &error);
		g_topology_lock();

		if (buf == NULL)
			return (error);

		magic = (uint32_t *)(buf + (CFE_BISZ_OFFSET % sectorsize));
		if (*magic == CFE_BISZ_MAGIC) {
			g_free(buf);
			return (0);
		}

		/* No match */
		g_free(buf);
		buf = NULL;
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

	if ((error = g_cfe_check_magic(cp, 0x0))) {
		G_CFE_LOG("unknown CFE image format\n");
		return (error);
	}

	return (0);
}

/**
 * Populate @p info with the CFE boot image layout.
 */
static int
g_cfe_get_bootimg_info(struct cfe_bootimg_info *info)
{
	struct bcm_platform		*bp;
	const struct cfe_bootimg_var	*imgvar;
	uint8_t				 bootimg;
	uint64_t			 imgsize;
	size_t				 len;
	int				 error;

	bp = bcm_get_platform();

	/* Determine layout type and boot image index */
	imgvar = NULL;
	for (size_t i = 0; i < nitems(cfe_bootimg_vars); i++) {
		/* Try to fetch the boot image index from NVRAM */
		len = sizeof(bootimg);
		error = bcm_get_nvram(bp, cfe_bootimg_vars[i].nvar, &bootimg,
		    &len, BHND_NVRAM_TYPE_UINT8);

		if (!error) {
			/* Matched */
			imgvar = &cfe_bootimg_vars[i];
			break;
		} else if (error != ENOENT) {
			/* Return an error if the NVRAM read fails for any
			 * reason other than variable not found */
			return (error);
		}
	}

	if (imgvar == NULL) {
		/* No dual/failsafe image support */
		info->type = CFE_IMAGE_SIMPLE;
		info->num_images	= 1;
		info->bootimage		= 0;
		info->offsets[0]	= 0;
		info->sizes[0]		= 0;

		// TODO: probe TRX/OS image size?

		return (0);
	}

	/* Dual/failsafe image */
	info->type = imgvar->type;
	info->num_images = imgvar->num_images;
	info->bootimage = bootimg;

	/* Probe image offsets */
	for (size_t i = 0; i < info->num_images; i++) {
		KASSERT(i < nitems(info->offsets), ("bad image count: %zu", i));

		if (i >= nitems(cfe_bootimg_offset_vars))
			return (ENXIO);

		len = sizeof(info->offsets[i]);
		error = bcm_get_nvram(bp, cfe_bootimg_offset_vars[i],
		    &info->offsets[i], &len, BHND_NVRAM_TYPE_UINT64);
		if (error) {
			G_CFE_LOG("error fetching offset[%zu]: %d\n", i, error);
			return (error);
		}
	}

	/* Probe image sizes */
	len = sizeof(imgsize);
	error = bcm_get_nvram(bp, BHND_NVAR_IMAGE_SIZE, &imgsize, &len,
	    BHND_NVRAM_TYPE_UINT64);
	if (error) {
		G_CFE_LOG("error fetching image size: %d\n", error);
		return (error);
	}

	for (size_t i = 0; i < info->num_images; i++)
		info->sizes[i] = imgsize;

	return (0);
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
