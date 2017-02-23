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

#define	CFE_CLASS_NAME	"CFE_MAP"

#define	G_CFE_LOG(msg, ...)	printf("%s: " msg, __FUNCTION__, ## __VA_ARGS__)

/* Device attribute names for our supported GEOM providers (NAND, CFI, SPI) */
static const struct cfe_flash_device {
	const char		*geom_attr;	/**< GEOM device attribute */
	const chipc_flash	*flash_types;	/**< supported ChipCommon flash types */
} cfe_flash_devices[] = {
	{ "NAND::device", (chipc_flash[]){
	    CHIPC_NFLASH, CHIPC_NFLASH_4706, CHIPC_FLASH_NONE } },
	{ "CFI::device", (chipc_flash[]){
	    CHIPC_PFLASH_CFI, CHIPC_FLASH_NONE } },
	{ "SPI::device", (chipc_flash[]){
	    CHIPC_SFLASH_AT, CHIPC_SFLASH_ST, CHIPC_FLASH_NONE } }
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

static struct g_geom *
g_cfe_taste(struct g_class *mp, struct g_provider *pp, int insist)
{
	struct g_consumer	*cp;
	struct g_geom		*gp;
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

	/* Read our partition map */
	if ((error = g_cfe_read_parts(cp))) {
		printf("%s: error parsing CFE image: %d\n", __FUNCTION__,
		    error);
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
