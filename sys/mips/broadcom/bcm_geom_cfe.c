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

#include "bcm_cfe_disk.h"
#include "bcm_machdep.h"

#include "bhnd_nvram_map.h"

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
static struct bcm_cfe_disk		*g_cfe_claim_disk(
					     const struct g_cfe_device *id);
static void				 g_cfe_unclaim_disk(
					     struct bcm_cfe_disk *disk);

static int				 g_cfe_taste_init(
					     struct g_cfe_taste_io *io,
					     struct g_consumer *cp,
					     struct bcm_cfe_disk *disk);
static void				 g_cfe_taste_fini(
					     struct g_cfe_taste_io *io);

static int				 g_cfe_taste_read(
					     struct g_cfe_taste_io *io,
					     off_t base, off_t offset,
					     void *buf, off_t len);
static int				 g_cfe_taste_read_part(
					     struct g_cfe_taste_io *io,
					     struct bcm_cfe_part *part,
					     off_t offset, void *buf,
					     off_t len);

static int				 g_cfe_parse_parts(
					     struct g_cfe_taste_io *io);

static int				 g_cfe_get_bootimg_info(
					     struct cfe_bootimg_info *info);

static int				 g_cfe_nvram_io_init(
					     struct g_cfe_nvram_io *io,
					     struct g_cfe_taste_io *taste,
					     struct bcm_cfe_part *part);

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

/**
 * Map of CFE image layout types to their associated OS boot image index
 * NVRAM variable.
 */
static const struct cfe_bootimg_var {
	g_cfe_bootimg_type	 type;		/**< layout type */
	const char		*nvar;		/**< NVRAM variable name */
	size_t			 num_images;	/**< image count */
} cfe_bootimg_vars[] = {
	{ G_CFE_IMAGE_FAILSAFE,	BHND_NVAR_BOOTPARTITION,	2 },
	{ G_CFE_IMAGE_DUAL,	BHND_NVAR_IMAGE_BOOT,		2 },
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
static struct bcm_cfe_disks g_cfe_disks;

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
	struct bcm_cfe_disk		*disk;
	struct cfe_bootimg_info		 info;
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

	/* Fetch CFE boot image info */
	if ((error = g_cfe_get_bootimg_info(&info))) {
		G_CFE_LOG("error fetching CFE image info: %d\n", error);
		goto failed;
	}

	/* Read our partition map */
	if ((error = g_cfe_parse_parts(&io))) {
		G_CFE_LOG("error parsing CFE image: %d\n", error);
		goto failed;
	}

	// TODO
	(void)g_cfe_nvram_io_init;
	bcm_cfe_print_disk(disk);

	printf("MATCH\n");
	
	// TODO
	// bcm_cfe_disk_free(disk);
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

	if ((error = bcm_cfe_probe_disks(&g_cfe_disks))) {
		BCM_ERR("bcm_cfe_probe_disks() failed: %d\n", error);
		return;
	}

	if (bootverbose)
		bcm_cfe_print_disks(&g_cfe_disks);
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
 * bcm_cfe_disk_free(), or releasing the claim on the entry via
 * g_cfe_unclaim_disk().
 * 
 * @param id The device description for which a CFE disk entry should be
 * returned.
 */
static struct bcm_cfe_disk *
g_cfe_claim_disk(const struct g_cfe_device *id)
{
	struct bcm_cfe_disk *disk;

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
	if ((disk = bcm_cfe_find_disk(&g_cfe_disks, id->cfe_name, 0)) == NULL)
		return (NULL);

	/* Remove from list of unclaimed disks */
	SLIST_REMOVE(&g_cfe_disks, disk, bcm_cfe_disk, cd_link);

	return (disk);
}

/**
 * Release a claim on a CFE disk entry previously returned by
 * g_cfe_claim_disk().
 * 
 * @param disk A disk previously claimed via g_cfe_claim_disk().
 */
static void
g_cfe_unclaim_disk(struct bcm_cfe_disk *disk)
{
	g_topology_assert(); /* for g_cfe_disks */

#ifdef INVARIANTS
	struct bcm_cfe_disk *next;

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

// TODO
#if 0
/**
 * If @p probe is a CFE bootloader partition, provide the expected zero
 * offset.
 */
static int
g_cfe_probe_part_boot(struct g_consumer *cp, struct bcm_cfe_disk *disk,
    struct bcm_cfe_part *part)
{
	uint32_t	magic[CFE_MAGIC_COUNT];
	int		error;

	/* Must be a boot partition */
	if (strcmp(probe->pname, "boot") != 0)
		return (ENXIO);

	/* Skip if a valid offset was already probed */
	if (!probe->have_offset) {
		probe->have_offset = true;
		probe->offset = 0x0;
	}

	/* Standard CFE image? */
	error = g_cfe_read_probe(probe, magic, CFE_MAGIC_OFFSET, sizeof(magic));
	if (!error) {
		if (magic[CFE_MAGIC_0] == CFE_MAGIC &&
		    magic[CFE_MAGIC_1] == CFE_MAGIC)
		{
			return (0);
		}
	}

	/* Self-decompressing CFEZ image? */
	error = g_cfe_read_probe(probe, magic, CFE_BISZ_OFFSET, sizeof(*magic));
	if (!error) {
		if (magic[CFE_MAGIC_0] == CFE_BISZ_MAGIC)
			return (0);
	}

	/* Unrecognized */
	G_CFE_LOG("%s: unrecognized CFE image at offset %#jx\n", probe->dname,
	    (intmax_t)probe->offset);
	return (ENXIO);
}

/**
 * Determine offset of the NVRAM partition.
 */
static int
g_cfe_probe_part_nvram(struct g_consumer *cp, struct bcm_cfe_disk *disk,
    struct bcm_cfe_part *part)
{
	struct g_cfe_probe_io	pio;
	int			error, result;

	/* Recognized NVRAM formats, in probe order. */
	bhnd_nvram_data_class * const nvram_classes[] = {
		&bhnd_nvram_bcm_class,
		&bhnd_nvram_tlv_class
	};

	/* Must be an NVRAM partition */
	if (strcmp(probe->pname, "nvram") != 0)
		return (ENXIO);	

	/* Skip if a valid offset was already probed */
	if (!probe->have_offset) {
		/* Invalid size? */
		if (probe->size > probe->mediasize) {
			G_CFE_LOG("cannot determine %s offset: invalid size "
			    "%#jx\n", probe->dname, (intmax_t)probe->size);
			return (ENXIO);
		}

		/* The NVRAM partition is always positioned at the end of the
		 * flash image; we need the size to determine the offset */
		if (!probe->have_size)
			return (ENXIO);

		probe->have_offset = true;
		probe->offset = rounddown(probe->mediasize - probe->size,
		    probe->blksize);
	}

	/* Initialize our mapping NVRAM I/O context */
	if ((error = g_cfe_probe_io_init(&pio, probe)))
		return (error);

	/* Check for NVRAM magic */
	for (size_t i = 0; i < nitems(nvram_classes); i++) {
		result = bhnd_nvram_data_probe(nvram_classes[i], &pio.io);
		if (result <= 0)
			break;
	}

	/* Clean up our I/O context */
	bhnd_nvram_io_free(&pio.io);

	if (result > 0) {
		G_CFE_LOG("%s: unrecognized NVRAM image at offset %#jx\n",
		    probe->dname, (intmax_t)probe->offset);

		return (ENXIO);
	}

	return (0);
}


/**
 * Determine size/offset of the TRX partition.
 */
static int
g_cfe_probe_part_trx(struct g_consumer *cp, struct bcm_cfe_disk *disk,
    struct bcm_cfe_part *part)
{
	// TODO
#if 0
	struct cfe_flash_probe	*os;
	const char		*trx_names[] = { "trx", "trx1", "trx2" };
	const char		*os_names[] = { "os", "os1", "os2" };

	/* Must be a trx partition */
	for (size_t i = 0; i < nitems(trx_names); i++) {
		// TODO
		if (strcmp(probe->pname, "config") != 0)
			return (ENXIO);
	}
#endif

	// TODO
	return (ENXIO);
}

/**
 * Determine offset of the OS partition.
 */
static int
g_cfe_probe_part_os(struct g_consumer *cp, struct bcm_cfe_disk *disk,
    struct bcm_cfe_part *part)
{
	// TODO
	return (ENXIO);
}

#endif

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
    struct bcm_cfe_disk *disk)
{
	off_t mediasize, stripesize;

	io->cp = cp;
	io->disk = disk;
	io->buf = NULL; 
	io->buf_off = 0x0;
	io->buf_len = 0x0;

	/* Partitions are aligned to either the flash block size, or
	 * BCM_CFE_PALIGN_MIN, whichever is greater */
	g_topology_assert();
	mediasize = cp->provider->mediasize;
	stripesize = cp->provider->stripesize;

	if (stripesize >= BCM_CFE_PALIGN_MIN)
		io->palign = stripesize;
	else {
		io->palign = BCM_CFE_PALIGN_MIN;
	}

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
g_cfe_taste_read(struct g_cfe_taste_io *io, off_t base, off_t offset, void *buf,
    off_t len)
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
 * Read @p len bytes from @p io at an offset relative to @p part.
 * 
 * @param	io	The GEOM taste I/O context to be used for reading.
 * @param	part	The partition to be read.
 * @param	offset	The read offset within @p part.
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
g_cfe_taste_read_part(struct g_cfe_taste_io *io, struct bcm_cfe_part *part,
    off_t offset, void *buf, off_t len)
{
	off_t mediasize;

	g_topology_assert();
	mediasize = io->cp->provider->mediasize;

	/* Validate our partition's size and offset */
	if (part->offset == BCM_CFE_INVALID_OFF)
		return (ENXIO);

	if (part->size == BCM_CFE_INVALID_SIZE)
		return (ENXIO);

	if (part->size > mediasize || mediasize - part->size < part->offset)
		return (ENXIO);

	/* Verify that the request falls within the partition range */
	if (len > part->size || part->size - len < offset)
		return (ENXIO); /* offset+len falls outside the partition */

	return (g_cfe_taste_read(io, part->offset, offset, buf, len));
}

/**
 * Find the first partition matching @p label and @p offset.
 * 
 * @param io		I/O context.
 * @param label		Required label, or NULL to match on any label.
 * @param offset	Required offset, or BCM_CFE_INVALID_OFF to match on any
 *			offset.
 */
static struct bcm_cfe_part *
g_cfe_find_matching_part(struct g_cfe_taste_io *io, const char *label,
    off_t offset)
{
	return (bcm_cfe_parts_match(&io->disk->parts, label, offset));
}

/* Probe CFE 'boot' partition */
static struct bcm_cfe_part *
g_cfe_probe_cfe(struct g_cfe_taste_io *io, off_t block)
{
	struct bcm_cfe_part	*boot;
	uint32_t		 magic[2];
	int			 error;

	/* If a CFE partition exists, it will always be the 'boot' partition */
	if ((boot = g_cfe_find_matching_part(io, "boot", block)) == NULL)
		return (NULL);

	/* Standard CFE image? */
	error = g_cfe_taste_read(io, block, CFE_MAGIC_OFFSET, magic,
	    sizeof(magic));
	if (error) {
		G_CFE_LOG("error reading CFE magic: %d", error);
		return (NULL);
	}

	if (magic[0] == CFE_MAGIC && magic[1] == CFE_MAGIC)
		return (boot);


	/* Self-decompressing CFEZ image? */
	error = g_cfe_taste_read(io, block, CFE_BISZ_OFFSET, magic,
	    sizeof(*magic));
	if (error) {
		G_CFE_LOG("error reading CFE BISZ magic: %d", error);
		return (NULL);
	}

	if (magic[0] == CFE_BISZ_MAGIC)
		return (boot);

	/* Not recognized */
	G_CFE_LOG("unrecognized CFE image at offset %#jx\n", (intmax_t)block);
	return (NULL);
}

G_CFE_DEFINE_PART_PROBE("CFE", g_cfe_probe_cfe);

/* Probe MINIX 'config' partition (as found on WGT634U) */
static struct bcm_cfe_part *
g_cfe_probe_minix_config(struct g_cfe_taste_io *io, off_t block)
{
	struct bcm_cfe_part	*config, *boot;
	uint16_t		 magic;
	int			 error;

	/* This is only applicable to a 'config' partition */
	if ((config = g_cfe_find_matching_part(io, "config", block)) == NULL)
		return (NULL);

	/* If a MINIX config partition exists, it should always directly follow
	 * the boot partition (which itself must be at offset 0x0) */
	if ((boot = g_cfe_find_matching_part(io, "boot", 0x0)) == NULL)
		return (NULL);

	if (bcm_cfe_part_get_next(boot, io->palign) != block)
		return (NULL);

	/* Look for the MINIX superblock */
	error = g_cfe_taste_read(io, block, CFE_MINIX_OFFSET, &magic,
	    sizeof(magic));
	if (error) {
		G_CFE_LOG("error reading MINIX magic: %d", error);
		return (NULL);
	}

	if (magic != CFE_MINIX_MAGIC && bswap16(magic) != CFE_MINIX_MAGIC) {
		G_CFE_LOG("unrecognized config partition magic 0x%04" PRIx16
		    " at offset %#jx\n", magic, (intmax_t)block);

		return (NULL);
	}

	return (config);
}

G_CFE_DEFINE_PART_PROBE("MINIX_CONFIG", g_cfe_probe_minix_config);

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
		struct bcm_cfe_part		*part;

		SET_FOREACH(probep, g_cfe_part_probe_set) {

			probe = *probep;

			/* Try probing the given offset */
			part = probe->func(io, offset);
			if (part == NULL)
				continue;

			/* We now know the actual offset */
			if (part->offset == BCM_CFE_INVALID_OFF)
				part->offset = offset;

			/* Terminate probing of this offset */
			break;
		}

		/* Advance to next block? */
		if (part == NULL || part->size == BCM_CFE_INVALID_SIZE) {
			offset += palign;
			continue;
		}

		/* Advance past known partition size */
		KASSERT(part->offset != BCM_CFE_INVALID_OFF,
		    ("missing offset"));

		if (OFF_MAX - part->size < part->offset ||
		    part->size + part->offset > mediasize)
		{
			G_CFE_LOG("%s: invalid size/offset %#jx+%#jx\n",
			    part->label, (intmax_t)part->size,
			    (intmax_t)part->offset);

			return (ENXIO);
		}

		offset = roundup(part->offset+part->size, palign);
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
			G_CFE_LOG("error fetching '%s' boot image index "
			    "variable: %d\n", cfe_bootimg_vars[i].nvar, error);
			return (error);
		}
	}

	if (imgvar == NULL) {
		/* No dual/failsafe image support */
		info->type = G_CFE_IMAGE_SIMPLE;
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

		if (i >= nitems(cfe_bootimg_offset_vars)) {
			G_CFE_LOG("missing offset variable for image %zu\n", i);
			return (ENXIO);
		}

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

/**
 * Initialize a new GEOM-backed NVRAM partition I/O context.
 *
 * The caller is responsible for releasing all resources held by the returned
 * I/O context via bhnd_nvram_io_free().
 * 
 * @param[out]	io	On success, will be initialized as an I/O context
 *			mapping @p part from @p cp.
 * @param	taste	The GEOM taste context to be used for reading.
 * @param	part	The CFE partition entry.
 *
 * @retval 0		success.
 * @retval EINVAL	if @p part has not been populated with a valid offset
 *			and size.
 * @retval non-zero	if opening @p part otherwise fails, a standard unix
 *			error will be returned.
 */
static int
g_cfe_nvram_io_init(struct g_cfe_nvram_io *io, struct g_cfe_taste_io *taste,
    struct bcm_cfe_part *part)
{
	/* Size and offset are required */
	if (part->size == BCM_CFE_INVALID_SIZE)
		return (EINVAL);

	if (part->offset == BCM_CFE_INVALID_OFF)
		return (EINVAL);

	io->io.iops = &bhnd_nvram_g_cfeio_ops;
	io->taste = taste;
	io->part = part;

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
	return (nio->part->size);
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

	return (g_cfe_taste_read_part(nio->taste, nio->part, offset, buffer,
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
