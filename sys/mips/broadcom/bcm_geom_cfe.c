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
#include <sys/ctype.h>
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

#include <machine/elf.h>
#include <machine/_inttypes.h>

#include <geom/geom.h>
#include <geom/geom_slice.h>

#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/cores/chipc/chipc.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>
#include <dev/cfe/cfe_ioctl.h>

#include "bcm_machdep.h"

#include "bcm_geom_cfe.h"

/*
 * Performs slicing of a bhnd(4) flash device based on CFE's flash partition
 * map and conservative heuristics.
 */

BHND_NVRAM_IOPS_DEFN(g_cfeio);

struct g_cfe_device;

static device_t				 g_cfe_find_chipc_parent(device_t dev);
static bool				 g_cfe_flash_type_matches(
					     const struct g_cfe_device *id,
					     chipc_flash type);

static const struct g_cfe_device	*g_cfe_device_lookup(
					     struct g_consumer *cp);

static void				 g_cfe_probe_bootloader(void *ident);
static struct bcm_disk			*g_cfe_claim_disk(
					     const struct g_cfe_device *id);
static void				 g_cfe_unclaim_disk(
					     struct bcm_disk *disk);

static bool				 g_cfe_is_boot_disk(
					     struct bcm_disk *disk,
					     struct bcm_bootinfo *bootinfo);

static int				 g_cfe_taste_init(
					     struct g_cfe_taste_io *io,
					     struct g_consumer *cp,
					     struct bcm_disk *disk);
static void				 g_cfe_taste_fini(
					     struct g_cfe_taste_io *io);

static int                               g_cfe_taste_read(
					     struct bcm_part *part,
					     struct g_cfe_taste_io *io,
					     off_t block, off_t offset,
					     void *buf, off_t len);

static int				 g_cfe_taste_read_direct(
					     struct g_cfe_taste_io *io,
					     off_t base, off_t offset,
					     void *buf, off_t len);

static int				 g_cfe_parse_parts(
					     struct g_cfe_taste_io *io);

static int				 g_cfe_nvram_io_init(
					     struct g_cfe_nvram_io *io,
					     struct g_cfe_taste_io *taste,
					     off_t offset, off_t size);

/*
 * Supported CFE/GEOM flash devices.
 * 
 * Provides a mapping between CFE device names, GEOM attribute names,
 * and a list of supported ChipCommon flash types (terminated by
 * CHIPC_FLASH_NONE).
 */
static const struct g_cfe_device {
	const char		*cfe_name;	/**< CFE device class name */
	const char		*geom_attr;	/**< GEOM device_t attribute name */
	const chipc_flash	*flash_types;	/**< supported ChipCommon flash types */
} g_cfe_devices[] = {
	{ "nflash",	"NAND::device", (chipc_flash[]) {
	    CHIPC_NFLASH,
	    CHIPC_FLASH_NONE,
	}},

	{ "flash",	"CFI::device",	(chipc_flash[]) {
	    CHIPC_PFLASH_CFI,
	    CHIPC_FLASH_NONE
	}},

	{ "flash",	"SPI::device",	(chipc_flash[]) {
	    CHIPC_SFLASH_AT,
	    CHIPC_SFLASH_ST,
	    CHIPC_FLASH_NONE
	}}
};

/* Debugging flags */
enum {
	G_CFE_DEBUG_PROBE = 1 << 0,
};

static u_long g_cfe_debug = 0;

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

/** CFE disk entries */
static struct bcm_disks g_cfe_disks;

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
	const struct g_cfe_device	*id;
	struct g_consumer		*cp;
	struct g_geom			*gp;
	struct g_cfe_taste_io		 io;
	struct bcm_disk			*disk;
	int				 error;
	bool				 have_io = false;

	disk = NULL;

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
	if ((id = g_cfe_device_lookup(cp)) == NULL)
		goto failed;

	/* Claim the corresponding CFE disk entry */
	if ((disk = g_cfe_claim_disk(id)) == NULL) {
		G_CFE_LOG("missing CFE disk entry for %s\n", id->cfe_name);
		goto failed;
	}

	/* Initialize our I/O state */
	if ((error = g_cfe_taste_init(&io, cp, disk))) {
		G_CFE_LOG("error initializing I/O context: %d\n", error);
		goto failed;
	} else {
		have_io = true;
	}

	/* Read our partition map */
	if ((error = g_cfe_parse_parts(&io))) {
		G_CFE_LOG("error parsing CFE image: %d\n", error);
		goto failed;
	}

	// TODO
	bcm_print_disk(disk);
	printf("MATCH\n");
	
	// TODO
	// bcm_disk_free(disk);
	// g_cfe_taste_fini(&io);
	// return (???);

failed:
	/* No match */
	g_access(cp, -1, 0, 0);
	g_detach(cp);
	g_destroy_consumer(cp);
	g_destroy_geom(gp);

	if (disk != NULL) {
		g_cfe_unclaim_disk(disk);
		disk = NULL;
	}

	if (have_io)
		g_cfe_taste_fini(&io);

	return (NULL);
}

/*
 * Probe CFE for flash layout information once the kernel memory subsystem
 * if available.
 * 
 * This must be done prior to driver/GEOM attachment, as calling into CFE
 * will conflict with native flash driver access.
 */
static void
g_cfe_probe_bootloader(void *ident)
{
	int error;

	SLIST_INIT(&g_cfe_disks);

	if ((error = bcm_probe_disks(&g_cfe_disks))) {
		BCM_ERR("bcm_probe_disks() failed: %d\n", error);
		return;
	}

	if (bootverbose)
		bcm_print_disks(&g_cfe_disks);
}

SYSINIT(g_cfe_probe_bootloader, SI_SUB_KMEM, SI_ORDER_ANY,
    g_cfe_probe_bootloader, NULL);

/**
 * Return the first matching device table entry for @p cp, or NULL
 * if no matching device is found.
 * 
 * @param cp The GEOM consumer to match against the supported device table.
 */
static const struct g_cfe_device *
g_cfe_device_lookup(struct g_consumer *cp)
{
	int error;

	mtx_lock(&Giant);	/* for newbus */

	/* Try to fetch the backing device */
	for (size_t i = 0; i < nitems(g_cfe_devices); i++) {
		const struct g_cfe_device	*id;
		const struct chipc_caps		*ccaps;
		device_t			 chipc, dev;
		size_t				 len;

		id = &g_cfe_devices[i];
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
 * Claim and return the CFE disk entry corresponding to @p id, or NULL if
 * not found.
 * 
 * The caller is responsible for either freeing the claimed entry via
 * bcm_disk_free(), or releasing the claim on the entry via
 * g_cfe_unclaim_disk().
 * 
 * @param id The device description for which a CFE disk entry should be
 * returned.
 */
static struct bcm_disk *
g_cfe_claim_disk(const struct g_cfe_device *id)
{
	struct bcm_disk *disk;

	/* Exclusive access to our shared CFE disk entries is guaranteed by
	 * the topology lock */
	g_topology_assert();

	/*
	 * Assume a CFE device unit of 0; no currently known hardware supports
	 * multiple CFE flash devices with the same driver class; there will
	 * either be:
	 *
	 * - 1 SPI/CFI device ('flash0')
	 * - 1 SPI/CFI device ('flash0') AND 1 NAND device ('nflash0')
	 */
	if ((disk = bcm_find_disk(&g_cfe_disks, id->cfe_name, 0)) == NULL)
		return (NULL);

	/* Remove from list of unclaimed disks */
	SLIST_REMOVE(&g_cfe_disks, disk, bcm_disk, cd_link);

	return (disk);
}

/**
 * Release a claim on a CFE disk entry previously returned by
 * g_cfe_claim_disk().
 * 
 * @param disk A disk previously claimed via g_cfe_claim_disk().
 */
static void
g_cfe_unclaim_disk(struct bcm_disk *disk)
{
	g_topology_assert(); /* for g_cfe_disks */

#ifdef INVARIANTS
	struct bcm_disk *next;

	SLIST_FOREACH(next, &g_cfe_disks, cd_link) {
		KASSERT(next != disk, ("disk not owned by caller"));
	}
#endif

	/* Reinsert into list of unclaimed disks */
	SLIST_INSERT_HEAD(&g_cfe_disks, disk, cd_link);
}

static void
g_cfe_config(struct gctl_req *req, struct g_class *mp, const char *verb)
{

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
g_cfe_flash_type_matches(const struct g_cfe_device *id, chipc_flash type)
{
	const chipc_flash *req;

	for (req = id->flash_types; *req != CHIPC_FLASH_NONE; req++) {
		if (*req == type)
			return (true);
	}

	return (false);
}

/**
 * Return true of @p bootinfo matches @p disk, false otherwise.
 */
static bool
g_cfe_is_boot_disk(struct bcm_disk *disk, struct bcm_bootinfo *bootinfo)
{
	if (strcmp(bootinfo->drvname, disk->drvname) != 0)
		return (false);

	if (bootinfo->devunit != disk->unit)
		return (false);

	return (true);
}

/**
 * Initialize a new GEOM-backed I/O context.
 *
 * The caller is responsible for releasing all resources held by the returned
 * I/O context via g_cfe_taste_fini().
 * 
 * @param[out]	io	On success, will be initialized as an I/O context
 *			mapping cp.
 * @param	cp	The GEOM consumer to be used for reading.
 * @param	disk	The CFE disk entry corresponding to @p cp.
 *
 * @retval 0		success.
 * @retval non-zero	if initializing @p io otherwise fails, a standard unix
 *			error will be returned.
 */
static int
g_cfe_taste_init(struct g_cfe_taste_io *io, struct g_consumer *cp,
    struct bcm_disk *disk)
{
	struct bcm_bootinfo	bootinfo;
	off_t			mediasize, stripesize;
	int			error;

	io->cp = cp;
	io->disk = disk;
	io->buf = NULL; 
	io->buf_off = 0x0;
	io->buf_len = 0x0;
	io->palign = BCM_PART_ALIGN_MIN;

	/* Sanity check our alignment */
	g_topology_assert();
	mediasize = cp->provider->mediasize;
	stripesize = cp->provider->stripesize;

	if ((stripesize % io->palign) != 0) {
		G_CFE_LOG("misaligned flash block size %#jx\n",
		    (intmax_t)io->palign);

		return (ENXIO);
	}

	if ((mediasize % io->palign) != 0) {
		G_CFE_LOG("misaligned media size %#jx/%#jx\n",
		    (intmax_t)mediasize, (intmax_t)io->palign);
		return (ENXIO);
	}

	/* Fetch our boot info, if any */
	if ((error = bcm_get_bootinfo(&bootinfo))) {
		G_CFE_LOG("error fetching CFE boot info: %d\n", error);
		return (error);
	}

	if (g_cfe_is_boot_disk(disk, &bootinfo)) {
		io->bootinfo = bootinfo;
		io->have_bootinfo = true;
	}

	return (0);
}

/**
 * Release any resources held by an @p io instance previously initialized via
 * g_cfe_taste_io_init().
 *
 * @param       erom    An erom parser instance previously initialized via
 *                      bhnd_erom_init_static().
 */
static void
g_cfe_taste_fini(struct g_cfe_taste_io *io)
{
	if (io->buf != NULL)
		g_free(io->buf);
}

/**
 * Validate the read request against the range defined by @p part (if any),
 * reading @p len bytes at @p base + @p offset from @p io.
 * 
 * @param	part	The partition definition defining the valid I/O range.
 * @param	io	The GEOM taste I/O context to be used for reading.
 * @param	block	The base read offset within @p io.
 * @param	offset	The read offset relative to @p base.
 * @param[out]	buf	On success, @p len bytes will be written to this buffer.
 * @param	len	The number of bytes to be read.
 *
 * @retval 0		success
 * @retval ENXIO	if @p part is not NULL, and the partition's offset or
 *			size are unknown.
 * @retval ENXIO	if the request is beyond the disk or partition's total
 *			size.
 * @retval non-zero	if the read otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
g_cfe_taste_read(struct bcm_part *part, struct g_cfe_taste_io *io, off_t block,
    off_t offset, void *buf, off_t len)
{
	/* The partition offset (if known) must match the provided block
	 * offset */
	if (BCM_PART_HAS_OFFSET(part) && part->offset != block)
		return (ENXIO);

	/* The request range must fall within the partition size (if known) */
	if (BCM_PART_HAS_SIZE(part)) {
		if (offset > part->size)
			return (ENXIO);

		if (part->size - offset < len)
			return (ENXIO);
	}

	return (g_cfe_taste_read_direct(io, block, offset, buf, len));
}

/**
 * Read @p len bytes at @p base + @p offset from @p io.
 * 
 * @param	io	The GEOM taste I/O context to be used for reading.
 * @param	base	The base read offset within @p io.
 * @param	offset	The read offset relative to @p base.
 * @param[out]	buf	On success, @p len bytes will be written to this buffer.
 * @param	len	The number of bytes to be read.
 * 
 * @retval 0		success
 * @retval ENXIO	if @p part is not NULL, and the partition's offset or
 *			size are unknown.
 * @retval ENXIO	if the request is beyond the disk or partition's total
 *			size.
 * @retval non-zero	if the read otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
g_cfe_taste_read_direct(struct g_cfe_taste_io *io, off_t base, off_t offset,
    void *buf, off_t len)
{
	off_t	mediasize, nbytes, read_offset, sectorsize, sector;
	int	error;

	g_topology_assert();
	mediasize = io->cp->provider->mediasize;
	sectorsize = io->cp->provider->sectorsize;

	/* Calculate read offset */
	if (OFF_MAX - base < offset)
		return (ENXIO); /* base+offset would overflow */

	read_offset = base + offset;

	/* Verify that the request falls within the mediasize */
	if (mediasize < read_offset || (mediasize - read_offset) < len)
		return (ENXIO);

	/* Perform the sector-aligned read */
	sector = rounddown(read_offset, sectorsize);
	nbytes = roundup(len, sectorsize);

	if (io->buf == NULL || io->buf_off != sector || io->buf_len < nbytes) {
		io->buf_off = sector;
		io->buf_len = nbytes;

		g_topology_unlock();
		io->buf = g_read_data(io->cp, sector, nbytes, &error);
		g_topology_lock();

		if (io->buf == NULL)
			return (error);
		
	}

	/* Copy out the result */
	memcpy(buf, io->buf + (read_offset % sectorsize), len);
	return (0);
}

/**
 * Find the first partition matching @p label and @p offset.
 * 
 * @param io		I/O context.
 * @param label		Required label, or NULL to match on any label.
 * @param offset	Required offset, or BCM_DISK_INVALID_OFF to match on any
 *			offset.
 */
static struct bcm_part *
g_cfe_find_matching_part(struct g_cfe_taste_io *io, const char *label,
    off_t offset)
{
	return (bcm_parts_match(&io->disk->parts, label, offset));
}

/* Probe CFE 'boot' partition */
static struct bcm_part *
g_cfe_probe_cfe(struct g_cfe_taste_io *io, off_t block)
{
	struct bcm_part	*boot;
	uint32_t	 magic[2];
	int		 error;

	/* If a CFE partition exists, it will always be the 'boot' partition */
	if ((boot = g_cfe_find_matching_part(io, "boot", block)) == NULL)
		return (NULL);

	/* Standard CFE image? */
	error = g_cfe_taste_read(boot, io, block, BCM_CFE_MAGIC_OFFSET, magic,
	    sizeof(magic));
	if (error) {
		G_CFE_LOG("error reading CFE magic: %d", error);
		return (NULL);
	}

	if (magic[0] == BCM_CFE_MAGIC && magic[1] == BCM_CFE_MAGIC)
		return (boot);


	/* Self-decompressing CFEZ image? */
	error = g_cfe_taste_read(boot, io, block, BCM_CFE_BISZ_OFFSET, magic,
	    sizeof(*magic));
	if (error) {
		G_CFE_LOG("error reading CFE BISZ magic: %d", error);
		return (NULL);
	}

	if (magic[0] == BCM_CFE_BISZ_MAGIC)
		return (boot);

	/* Not recognized */
	G_CFE_DEBUG(PROBE, "unrecognized CFE image at offset %#jx\n",
	    (intmax_t)block);

	return (NULL);
}

G_CFE_DEFINE_PART_PROBE("CFE", g_cfe_probe_cfe);

/* Probe MINIX 'config' partition (as found on WGT634U) */
static struct bcm_part *
g_cfe_probe_minix_config(struct g_cfe_taste_io *io, off_t block)
{
	struct bcm_part	*config, *boot;
	uint16_t	 magic;
	int		 error;

	/* Must be a 'config' partition */
	if ((config = g_cfe_find_matching_part(io, "config", block)) == NULL)
		return (NULL);

	/* If a MINIX config partition exists, it should always directly follow
	 * the boot partition (which itself must be at offset 0x0) */
	if ((boot = g_cfe_find_matching_part(io, "boot", 0x0)) == NULL)
		return (NULL);

	if (bcm_part_get_next(boot, io->palign) != block)
		return (NULL);

	/* Look for the MINIX superblock */
	error = g_cfe_taste_read(config, io, block, BCM_MINIX_OFFSET, &magic,
	    sizeof(magic));
	if (error) {
		G_CFE_LOG("error reading MINIX magic: %d", error);
		return (NULL);
	}

	if (magic == BCM_MINIX_MAGIC || bswap16(magic) == BCM_MINIX_MAGIC)
		return (config);

	G_CFE_DEBUG(PROBE, "unrecognized config partition magic 0x%04" PRIx16
	    " at offset %#jx\n", magic, (intmax_t)block);

	return (NULL);
}

G_CFE_DEFINE_PART_PROBE("MINIX_CONFIG", g_cfe_probe_minix_config);


/* Probe an NVRAM partition */
static struct bcm_part *
g_cfe_probe_nvram(struct g_cfe_taste_io *io, off_t block)
{
	struct g_cfe_nvram_io	 nvram_io;
	struct bcm_part		*nvram;
	off_t			 mediasize;
	int			 error, result;

	g_topology_assert();
	mediasize = io->cp->provider->mediasize;

	/* Must be an 'nvram' partition */
	if ((nvram = g_cfe_find_matching_part(io, "nvram", block)) == NULL)
		return (NULL);

	/* Initialize our NVRAM I/O context */
	if ((error = g_cfe_nvram_io_init(&nvram_io, io, block, nvram->size))) {
		G_CFE_LOG("error initializing NVRAM I/O context: %d\n", error);
		return (NULL);
	}

	/* Probe for BCM NVRAM */
	result = bhnd_nvram_data_probe(&bhnd_nvram_bcm_class, &nvram_io.io);
	if (result <= BHND_NVRAM_DATA_PROBE_SPECIFIC)
		goto finished;

	/*
	 * Probe for TLV NVRAM.
	 * 
	 * A TLV NVRAM partition is always found at the end of the flash
	 * device
	 */
	if (!BCM_PART_HAS_SIZE(nvram) || nvram->size > mediasize) {
		result = ENXIO;
		goto finished;
	}

	/* Probe for TLV NVRAM */
	result = bhnd_nvram_data_probe(&bhnd_nvram_tlv_class, &nvram_io.io);
	if (result <= BHND_NVRAM_DATA_PROBE_SPECIFIC)
		goto finished;

	/* No match */
	result = ENXIO;

finished:
	bhnd_nvram_io_free(&nvram_io.io);

	if (result > BHND_NVRAM_DATA_PROBE_SPECIFIC) {
		G_CFE_DEBUG(PROBE, "no NVRAM partition at offset %#jx\n",
		    (intmax_t)block);

		return (NULL);
	}

	return (nvram);
}

G_CFE_DEFINE_PART_PROBE("NVRAM", g_cfe_probe_nvram);


/* Probe an 'os' partition as used by early devices without TRX support */
static struct bcm_part *
g_cfe_probe_os(struct g_cfe_taste_io *io, off_t block)
{
	struct bcm_part	*os;
	Elf_Ehdr	 ehdr;
	uint8_t		 gz_hdr[3];
	int		 error;

	/* Must be an 'os' partition */
	if ((os = g_cfe_find_matching_part(io, "os", block)) == NULL)
		return (NULL);

	/* Look for an ELF bootloader */
	error = g_cfe_taste_read(os, io, block, 0, &ehdr, sizeof(ehdr));
	if (error) {
		G_CFE_LOG("error reading ELF header: %d", error);
		return (NULL);
	}

	if (IS_ELF(ehdr))
		return (os);

	/* Look for a GZIP-compressed bootloader */
	error = g_cfe_taste_read(os, io, block, 0, &gz_hdr, sizeof(gz_hdr));
	if (error) {
		G_CFE_LOG("error reading GZIP magic: %d", error);
		return (NULL);
	}

	if (gz_hdr[0] == BCM_GZIP_MAGIC0 && gz_hdr[1] == BCM_GZIP_MAGIC1 &&
	    gz_hdr[2] == BCM_GZIP_DEFLATE)
	{
		return (os);
	}

	/* Scan for a boot block */
	for (size_t i = 0; i < BCM_BOOTBLK_MAX; i++ ) {
		uint64_t	magic;
		off_t		boff;

		boff = (BCM_BOOTBLK_BLKSIZE * i) + BCM_BOOTBLK_OFFSET;
		error = g_cfe_taste_read(os, io, block, boff, &magic,
		    sizeof(magic));
		if (error) {
			G_CFE_LOG("error reading boot block %zu: %d", i, error);
			return (NULL);
		}

		if (magic == BCM_BOOTBLK_MAGIC)
			return (os);
	}

	G_CFE_DEBUG(PROBE, "unrecognized OS partition at offset %#jx\n",
	    (intmax_t)block);

	return (NULL);
}

G_CFE_DEFINE_PART_PROBE("OS", g_cfe_probe_os);

/**
 * Return the best TRX partition entry for @p offset, if any.
 */
static struct bcm_part *
g_cfe_find_trx_part(struct g_cfe_taste_io *io, off_t offset)
{
	struct bcm_bootinfo	*bootinfo;
	struct bcm_part		*part;

	/* Find the bootimg that corresponds to the current offset, if any */
	if (io->have_bootinfo) {
		for (size_t i = 0; i < bootinfo->num_images; i++) {
			struct bcm_bootimg *img = &bootinfo->images[i];

			if (img->offset == BCM_DISK_INVALID_OFF)
				continue;

			if (img->offset != offset)
				continue;

			/* Look for matching partition entry */
			part = g_cfe_find_matching_part(io, img->label, offset);
			if (part != NULL)
				return (part);
		}
	}

	/* No match found in bootinfo; search for the next available/matching
	 * TRX partition entry */
	for (size_t i = 0; i < io->disk->num_parts; i++) {
		const char	*label;
		char		 buf[BCM_DISK_NAME_MAX];

		/* Format the partition label (trx, trx2, trx3...) */
		if (i == 0) {
			label = BCM_PART_LABEL_TRX;
		} else {
			int ret;

			ret = snprintf(buf, sizeof(buf), "trx%zu", i+1);
			if (ret >= sizeof(buf)) {
				G_CFE_LOG("%zu exceeds maximum CFE "
				    "device name length\n", i);
				return (NULL);
			}

			label = buf;
		}

		/* Look for matching partition */
		part = g_cfe_find_matching_part(io, label, offset);
		if (part != NULL)
			return (part);
	}

	/* No matching TRX partition found */
	return (NULL);
}

/* Probe a TRX partition */
static struct bcm_part *
g_cfe_probe_trx(struct g_cfe_taste_io *io, off_t block)
{
	struct bcm_part			*trx;
	struct bcm_trx_header		 hdr;
	int				 error;

	/* Find corresponding TRX partition entry */
	if ((trx = g_cfe_find_trx_part(io, block)) == NULL)
		return (NULL);

	/* Read our TRX header */
	error = g_cfe_taste_read(trx, io, block, 0, &hdr, sizeof(hdr));
	if (error) {
		G_CFE_LOG("error reading TRX header: %d", error);
		return (NULL);
	}

	if (le32toh(hdr.magic) != BCM_TRX_MAGIC) {
		G_CFE_DEBUG(PROBE, "invalid TRX magic 0x%" PRIx32 " at offset "
		    "%#jx\n", hdr.magic, (intmax_t)block);

		if (hdr.magic == 0x0) {
			/* Uninitialized TRX partition? */
			// TODO: add a partition flag for unused partitions?
			// TODO: if we do this via cfe_readblk(), we can be
			// certain that we're reading at the right offset and
			// have found an empty TRX partition
		}

		return (NULL);
	}

	/* Update our partition's fs_size */
	if (!BCM_PART_HAS_FS_SIZE(trx))
		trx->fs_size = le32toh(hdr.len);

	/* Validate any existing fs_size */
	if (trx->fs_size != le32toh(hdr.len)) {
		G_CFE_LOG("%s partition has incorrect fs_size %#jx, expected "
		    "%#" PRIx32 "\n", trx->label, (intmax_t)trx->fs_size,
		    le32toh(hdr.len));

		return (NULL);
	}

	return (trx);
}

G_CFE_DEFINE_PART_PROBE("TRX", g_cfe_probe_trx);

/**
 * Identify all partitions.
 *
 * @param io	The I/O context to be used for reading.
 */
static int
g_cfe_parse_parts(struct g_cfe_taste_io *io)
{
	off_t mediasize, palign;

	g_topology_assert();
	mediasize = io->cp->provider->mediasize;
	palign = io->palign;

	KASSERT((mediasize % palign) == 0, ("invalid partition alignment"));

	for (size_t offset = 0; offset < mediasize;) {
		struct g_cfe_part_probe_info	**probep, *probe;
		struct bcm_part			*part;
		off_t				 size;

		SET_FOREACH(probep, g_cfe_part_probe_set) {
			probe = *probep;

			/* Try probing the given offset */
			part = probe->func(io, offset);
			if (part == NULL)
				continue;

			/* We now know the actual offset */
			if (part->offset == BCM_DISK_INVALID_OFF)
				part->offset = offset;

			G_CFE_DEBUG(PROBE, "found %-11s +0x%08jx (%s)\n",
			    part->label, (intmax_t)part->offset, probe->name);

			/* Partition found; terminate probing of this offset */
			break;
		}

		/* Skip any blocks claimed by the partition entry, or advance
		 * to the next block. */
		size = palign;
		if (part != NULL) {
			if (BCM_PART_HAS_SIZE(part))
				size = part->size;
			else if (BCM_PART_HAS_FS_SIZE(part))
				size = part->fs_size;
		}

		if (part != NULL && BCM_PART_HAS_OFFSET(part))
			offset = part->offset;

		/* Verify that the next offset fits within the media size */
		if (offset > mediasize || mediasize - offset < size) {
			G_CFE_LOG("invalid size/offset %#jx+%#jx\n",
			    (intmax_t)size, (intmax_t)offset);

			return (ENXIO);
		}

		/* Advance to the next offset */
		if (size != palign) {
			G_CFE_DEBUG(PROBE, "seeking to 0x%jx\n",
			    (intmax_t)offset+size);
		}

		offset = roundup(offset+size, palign);
	}

	return (0);
}

/**
 * Initialize a new GEOM-backed NVRAM partition I/O context.
 *
 * The caller is responsible for releasing all resources held by the returned
 * I/O context via bhnd_nvram_io_free().
 * 
 * @param[out]	io	On success, will be initialized as an I/O context
 *			mapping @p part from @p cp.
 * @param	taste	The GEOM taste context to be used for reading.
 * @param	offset	The base offset for all I/O operations.
 * @param	size	The number of readable bytes at @p offset, or
 *			BCM_DISK_INVALID_OFF to allow any read up to the
 *			mediasize.
 *
 * @retval 0		success.
 * @retval EINVAL	if @p offset or @p size is invalid.
 * @retval non-zero	if initializing @p io otherwise fails, a standard unix
 *			error will be returned.
 */
static int
g_cfe_nvram_io_init(struct g_cfe_nvram_io *io, struct g_cfe_taste_io *taste,
    off_t offset, off_t size)
{
	off_t mediasize;

	g_topology_assert();
	mediasize = taste->cp->provider->mediasize;

	io->io.iops = &bhnd_nvram_g_cfeio_ops;
	io->taste = taste;

	/* Offset must fall within the total media size */
	if (offset > mediasize)
		return (EINVAL);

	/* Provide a default size? */
	if (size == BCM_DISK_INVALID_SIZE)
		size = mediasize - offset;

	/* offset+size must not overflow, and must fall within the total
	 * media size */
	if (OFF_MAX - offset < size || offset+size > mediasize)
		return (EINVAL);

	io->offset = offset;
	io->size = size;

	return (0);
}

static void
bhnd_nvram_g_cfeio_free(struct bhnd_nvram_io *io)
{
	/* All resources are managed externally */
}

static size_t
bhnd_nvram_g_cfeio_getsize(struct bhnd_nvram_io *io)
{
	struct g_cfe_nvram_io *nio = (struct g_cfe_nvram_io *)io;
	return (nio->size);
}

static int
bhnd_nvram_g_cfeio_setsize(struct bhnd_nvram_io *io, size_t size)
{
	/* unsupported */
	return (ENODEV);
}

static int
bhnd_nvram_g_cfeio_read_ptr(struct bhnd_nvram_io *io, size_t offset,
    const void **ptr, size_t nbytes, size_t *navail)
{
	/* unsupported */
	return (ENODEV);
}

static int
bhnd_nvram_g_cfeio_write_ptr(struct bhnd_nvram_io *io, size_t offset,
    void **ptr, size_t nbytes, size_t *navail)
{
	/* unsupported */
	return (ENODEV);
}

static int
bhnd_nvram_g_cfeio_write(struct bhnd_nvram_io *io, size_t offset,
    void *buffer, size_t nbytes)
{
	/* unsupported */
	return (ENODEV);
}

static int
bhnd_nvram_g_cfeio_read(struct bhnd_nvram_io *io, size_t offset, void *buffer,
    size_t nbytes)
{
	struct g_cfe_nvram_io *nio = (struct g_cfe_nvram_io *)io;

	/* Verify that the request falls within our mapped range */
	if (offset > nio->size || nio->size - offset < nbytes)
		return (ENXIO);

	return (g_cfe_taste_read_direct(nio->taste, nio->offset, offset, buffer,
	    nbytes));
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
