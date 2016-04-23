/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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
#include <sys/endian.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhndvar.h>

#include "nvramvar.h"

#include "bhnd_spromreg.h"
#include "bhnd_spromvar.h"

/*
 * BHND SPROM Parsing
 * 
 * Provides identification and parsing of BHND SPROM data.
 */

static int	sprom_direct_read(struct bhnd_sprom *sc, size_t offset,
		    void *buf, size_t nbytes, uint8_t *crc);
static int	sprom_extend_shadow(struct bhnd_sprom *sc, size_t image_size,
		    uint8_t *crc);
static int	sprom_populate_shadow(struct bhnd_sprom *sc);

static int	sprom_var_defn(struct bhnd_sprom *sc, const char *name,
		    const struct bhnd_nvram_var **var,
		    const struct bhnd_sprom_var **sprom);

/* SPROM revision is always located at the second-to-last byte */
#define	SPROM_REV(_sc)	SPROM_READ_1((_sc), (_sc)->sp_size - 2)

/* SPROM shadow I/O (with byte-order translation) */
#define	SPROM_READ_1(_sc, _off)	((_sc)->sp_shadow[_off])
#define	SPROM_READ_2(_sc, _off)	le16toh(*(uint16_t *)((_sc)->sp_shadow + _off))
#define	SPROM_READ_4(_sc, _off)	le32toh(*(uint32_t *)((_sc)->sp_shadow + _off))

#define	SPROM_WRITE_1(_sc, _off, _v)	\
	(_sc[_off]) = (_v)
#define	SPROM_WRITE_2(_sc, _off, _v)	\
	*((uint16_t *)((_sc)->sp_shadow + _off)) = htole16(_v)
#define	SPROM_WRITE_4(_sc, _off, _v)	\
	*((uint32_t *)((_sc)->sp_shadow + _off)) = htole32(_v)
	

/*
 * Supported SPROM image formats ordered by image size, from smallest to
 * largest.
 */
#define	SPROM_FMT(_sz, _revmin, _revmax, _sig)	\
	{ SPROM_SZ_ ## _sz, _revmin, _revmax,	\
	    SPROM_SIG_ ## _sig ## _OFF,		\
	    SPROM_SIG_ ## _sig }

static const struct sprom_fmt {
	size_t		size;
	uint8_t		rev_min;
	uint8_t		rev_max;
	size_t		sig_offset;
	uint16_t	sig_req;
} sprom_fmts[] = {
	SPROM_FMT(R1_3,		1, 3,	NONE),
	SPROM_FMT(R4_8_9,	4, 4,	R4),
	SPROM_FMT(R4_8_9,	8, 9,	R8_9),
	SPROM_FMT(R10,		10, 10,	R10),
	SPROM_FMT(R11,		11, 11,	R11)
};

/**
 * Identify the SPROM format at @p offset within @p r, verify the CRC,
 * and allocate a local shadow copy of the SPROM data.
 * 
 * After successful initialization, @p r will not be accessed; any pin
 * configuration required for SPROM access may be reset.
 * 
 * @param[out] sprom On success, will be initialized with shadow of the SPROM
 * data. 
 * @param r An active resource mapping the SPROM data.
 * @param offset Offset of the SPROM data within @p resource.
 */
int
bhnd_sprom_init(struct bhnd_sprom *sprom, struct bhnd_resource *r,
    bus_size_t offset)
{
	bus_size_t	 res_size;
	int		 error;

	sprom->dev = rman_get_device(r->res);
	sprom->sp_res = r;
	sprom->sp_res_off = offset;

	/* Determine maximum possible SPROM image size */
	res_size = rman_get_size(r->res);
	if (offset >= res_size)
		return (EINVAL);

	sprom->sp_size_max = MIN(res_size - offset, SPROM_SZ_MAX); 

	/* Allocate and populate SPROM shadow */
	sprom->sp_size = 0;
	sprom->sp_capacity = sprom->sp_size_max;
	sprom->sp_shadow = malloc(sprom->sp_capacity, M_BHND, M_NOWAIT);
	if (sprom->sp_shadow == NULL)
		return (ENOMEM);

	/* Read and identify SPROM image */
	if ((error = sprom_populate_shadow(sprom)))
		return (error);

	// TODO
	device_printf(sprom->dev, "spromrev %hhu\n", sprom->sp_rev);
	const struct bhnd_nvram_var *v;
	const struct bhnd_sprom_var *sv;

	if ((error = sprom_var_defn(sprom, "macaddr", &v, &sv)))
		return (error);

	device_printf(sprom->dev, "macaddr has %zu offsets\n", sv->num_offsets);

	return (0);
}

/**
 * Release all resources held by @p sprom.
 * 
 * @param sprom A SPROM instance previously initialized via bhnd_sprom_init().
 */
void
bhnd_sprom_fini(struct bhnd_sprom *sprom)
{
	free(sprom->sp_shadow, M_BHND);
}

/**
 * Read a SPROM variable, performing conversion to host byte order.
 *
 * @param		sc	The SPROM parser state.
 * @param		name	The SPROM variable name.
 * @param[out]		buf	On success, the requested value will be written
 *				to this buffer. This argment may be NULL if
 *				the value is not desired.
 * @param[in,out]	size	The capacity of @p buf. On success, will be set
 *				to the actual size of the requested value.
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable was not found.
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p size is too
 *			small to hold the requested value.
 * @retval non-zero	If reading @p name otherwise fails, a regular unix
 *			error code will be returned.
 */
int
bhnd_sprom_getvar(struct bhnd_sprom *sc, const char *name, void *buf,
    size_t *size)
{
	const struct bhnd_nvram_var	*nv;
	const struct bhnd_sprom_var	*sv;
	int				 error;

	if ((error = sprom_var_defn(sc, name, &nv, &sv)))
		return (error);

	// TODO
	
	return (ENOENT);
}

/* Read and identify the SPROM image by incrementally performing
 * read + CRC of all supported image formats */
static int
sprom_populate_shadow(struct bhnd_sprom *sc)
{
	const struct sprom_fmt	*fmt;
	int			 error;
	uint16_t		 sig;
	uint8_t			 srom_rev;
	uint8_t			 crc;

	crc = BHND_NVRAM_CRC8_INITIAL;

	/* Identify the SPROM revision (and populate the SPROM shadow) */
	for (size_t i = 0; i < nitems(sprom_fmts); i++) {
		fmt = &sprom_fmts[i];

		/* Read image data and check CRC */
		if ((error = sprom_extend_shadow(sc, fmt->size, &crc)))
			return (error);

		/* Skip on invalid CRC */
		if (crc != BHND_NVRAM_CRC8_VALID)
			continue;

		/* Fetch SROM revision */
		srom_rev = SPROM_REV(sc);

		/* Early sromrev 1 devices (specifically some BCM440x enet
		 * cards) are reported to have been incorrectly programmed
		 * with a revision of 0x10. */
		if (fmt->size == SPROM_SZ_R1_3 && srom_rev == 0x10)
			srom_rev = 0x1;

		/* Verify revision range */
		if (srom_rev < fmt->rev_min || srom_rev > fmt->rev_max)
			continue;

		/* Verify signature (if any) */
		sig = SPROM_SIG_NONE;
		if (fmt->sig_offset != SPROM_SIG_NONE_OFF)
			sig = SPROM_READ_2(sc, fmt->sig_offset);
		
		if (sig != fmt->sig_req) {
			device_printf(sc->dev,
			    "invalid sprom %hhu signature: 0x%hx "
			    "(expected 0x%hx)\n",
			    srom_rev, sig, fmt->sig_req);
			return (EINVAL);
		}

		/* Identified */
		sc->sp_rev = srom_rev;
		return (0);
	}

	/* identification failed */
	device_printf(sc->dev, "unrecognized sprom format\n");
	return (EINVAL);
}

/*
 * Extend the shadowed SPROM buffer to image_size, reading any required
 * data from the backing SPROM resource and updating the CRC.
 */
static int
sprom_extend_shadow(struct bhnd_sprom *sc, size_t image_size,
    uint8_t *crc)
{
	int	error;

	KASSERT(image_size >= sc->sp_size, (("shadow truncation unsupported")));

	/* Verify the request fits within our shadow buffer */
	if (image_size > sc->sp_capacity)
		return (ENOSPC);

	/* Skip no-op requests */
	if (sc->sp_size == image_size)
		return (0);

	/* Populate the extended range */
	error = sprom_direct_read(sc, sc->sp_size, sc->sp_shadow + sc->sp_size,
	     image_size - sc->sp_size, crc);
	if (error)
		return (error);

	sc->sp_size = image_size;
	return (0);
}

/**
 * Read nbytes at the given offset from the backing SPROM resource, and
 * update the CRC.
 */
static int
sprom_direct_read(struct bhnd_sprom *sc, size_t offset, void *buf,
    size_t nbytes, uint8_t *crc)
{
	bus_size_t	 res_offset;
	size_t		 nread;
	uint16_t	*p;

	KASSERT(nbytes % sizeof(uint16_t) == 0, ("unaligned sprom size"));
	KASSERT(offset % sizeof(uint16_t) == 0, ("unaligned sprom offset"));

	/* Check for read overrun */
	if (offset >= sc->sp_size_max || sc->sp_size_max - offset < nbytes) {
		device_printf(sc->dev, "requested SPROM read would overrun\n");
		return (EINVAL);
	}

	p = (uint16_t *)buf;
	res_offset = sc->sp_res_off + offset;

	/* Perform read */
	for (nread = 0; nread < nbytes; nread += 2) {
		*p = bhnd_bus_read_stream_2(sc->sp_res, res_offset+nread);
		*crc = bhnd_nvram_crc8(p, sizeof(*p), *crc);
		p++;
	};

	return (0);
}


/**
 * Locate the variable and SPROM revision-specific definitions
 * for variable with @p name.
 */
static int
sprom_var_defn(struct bhnd_sprom *sc, const char *name,
    const struct bhnd_nvram_var **var,
    const struct bhnd_sprom_var **sprom)
{
	/* Find variable definition */
	*var = bhnd_nvram_var_defn(name);
	if (*var == NULL)
		return (ENOENT);

	/* Find revision-specific SPROM definition */
	for (size_t i = 0; i < (*var)->num_sp_descs; i++) {
		const struct bhnd_sprom_var *sp = &(*var)->sprom_descs[i];

		if (sc->sp_rev < sp->compat.first)
			continue;
		
		if (sc->sp_rev > sp->compat.last)
			continue;

		/* Found */
		*sprom = sp;
		return (0);
	}

	/* Not supported by this SPROM revision */
	return (ENOENT);
}