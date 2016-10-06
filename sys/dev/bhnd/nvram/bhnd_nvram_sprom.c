/*-
 * Copyright (c) 2015-2016 Landon Fuller <landonf@FreeBSD.org>
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
#include <sys/ctype.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_data.h"
#include "bhnd_nvram_datavar.h"

#include "bhnd_nvram_map.h"

#include "bhnd_nvram_spromreg.h"

/*
 * BHND SPROM NVRAM data class
 *
 * The SPROM data format is a fixed-layout, non-self-descriptive binary format,
 * used on Broadcom wireless and wired adapters, that provides a subset of the
 * variables defined by Broadcom SoC NVRAM formats.
 */

struct bhnd_nvram_sprom {
	struct bhnd_nvram_data	 nv;	/**< common instance state */
	struct bhnd_nvram_io	*data;	/**< backing SPROM image */
	uint8_t			 srev;	/**< parsed SPROM revision */
};

BHND_NVRAM_DATA_CLASS_DEFN(sprom, "Broadcom SPROM")

/**
 * SPROM value storage.
 *
 * Sufficient for representing the native encoding of any defined SPROM
 * variable.
 */
union bhnd_nvram_sprom_storage {
	uint8_t		u8[SPROM_ARRAY_MAXLEN];
	uint16_t	u16[SPROM_ARRAY_MAXLEN];
	uint32_t	u32[SPROM_ARRAY_MAXLEN];
	int8_t		i8[SPROM_ARRAY_MAXLEN];
	int16_t		i16[SPROM_ARRAY_MAXLEN];
	int32_t		i32[SPROM_ARRAY_MAXLEN];
	char		ch[SPROM_ARRAY_MAXLEN];
};

/**
 * SPROM common integer value representation.
 */
union bhnd_nvram_sprom_intv {
	uint32_t	u32;
	int32_t		s32;
};

static size_t	bhnd_nvram_spromdef_nelems(
		    const struct bhnd_sprom_vardefn *sprom_def);

static bool	bhnd_nvram_sprom_matches(struct bhnd_nvram_sprom *sprom,
		    const struct bhnd_nvram_vardefn *var,
		    const struct bhnd_sprom_vardefn **sprom_def);

/*
 * Table of supported SPROM image formats, sorted by image size, ascending.
 */
#define	SPROM_FMT(_sz, _revmin, _revmax, _sig)	\
	{ SPROM_SZ_ ## _sz, _revmin, _revmax,	\
	    SPROM_SIG_ ## _sig ## _OFF,		\
	    SPROM_SIG_ ## _sig }

/* A SPROM image format descriptor */
static const struct bhnd_nvram_sprom_fmt {
	size_t		size;		/**< SPROM image size, in bytes */
	uint8_t		rev_min;	/**< Mimimum SPROM format revision */
	uint8_t		rev_max;	/**< Maximum SPROM format revision */
	size_t		sig_offset;	/**< SPROM signature offset, or SPROM_SIG_NONE_OFF */
	uint16_t	sig_req;	/**< SPROM signature value, or SPROM_SIG_NONE */
} bhnd_nvram_sprom_fmts[] = {
	SPROM_FMT(R1_3,		1, 3,	NONE),
	SPROM_FMT(R4_8_9,	4, 4,	R4),
	SPROM_FMT(R4_8_9,	8, 9,	R8_9),
	SPROM_FMT(R10,		10, 10,	R10),
	SPROM_FMT(R11,		11, 11,	R11)
};

#define	SPROM_NVLOG(_fmt, ...)	\
	printf("%s: " _fmt, __FUNCTION__, ##__VA_ARGS__)

#define	SPROM_NVRAM_TO_COOKIE(_var)	\
	((void *)(uintptr_t)(_var))

#define	SPROM_COOKIE_TO_NVRAM(_cookie)	\
	((const struct bhnd_nvram_vardefn *)(uintptr_t)(_cookie))

/**
 * Attempt to identify and parse the SPROM data mapped by @p io.
 *
 * The SPROM data format does not provide any identifying information at a
 * known offset, instead requiring that we iterate over the known SPROM image
 * sizes until we are able to compute a valid checksum (and, for later
 * revisions, validate a signature at a format-specific offset).
 *
 * @param	io	An I/O context mapping the SPROM data to be identified.
 * @param[out]	srev	On success, the identified SPROM revision.
 * @param[out]	shadow	On success, a correctly sized iobuf instance mapping
 *			a copy of the parsed SPROM image. The caller is
 *			responsible for deallocating this instance via
 *			bhnd_nvram_io_free()
 *
 * @retval 0		success
 * @retval non-zero	If identifying @p io otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvram_sprom_parse(struct bhnd_nvram_io *io, uint8_t *srev,
    struct bhnd_nvram_io **shadow)
{
	const struct bhnd_nvram_sprom_fmt	*fmt;
	struct bhnd_nvram_io			*buf;
	uint8_t					 crc;
	int					 error;

	crc = BHND_NVRAM_CRC8_INITIAL;
	buf = bhnd_nvram_iobuf_empty(0, SPROM_SZ_MAX);

	/* We iterate the image formats smallest to largest, allowing us to
	 * perform incremental checksum calculation */
	for (size_t i = 0; i < nitems(bhnd_nvram_sprom_fmts); i++) {
		void		*ptr;
		size_t		 nbytes, nr;
		uint16_t	 sig;

		fmt = &bhnd_nvram_sprom_fmts[i];
		nbytes = bhnd_nvram_io_getsize(buf);

		if (nbytes > fmt->size)
			panic("SPROM format is defined out-of-order");

		nr = fmt->size - nbytes;

		/* Adjust the buffer size and fetch a write pointer */
		if ((error = bhnd_nvram_io_setsize(buf, fmt->size)))
			goto failed;

		error = bhnd_nvram_io_write_ptr(buf, nbytes, &ptr, nr, NULL);
		if (error)
			goto failed;

		/* Read image data and update CRC */
		if ((error = bhnd_nvram_io_read(io, nbytes, ptr, nr)))
			goto failed;

		crc = bhnd_nvram_crc8(ptr, nr, crc);

		/* Skip on invalid CRC */
		if (crc != BHND_NVRAM_CRC8_VALID)
			continue;

		/* Fetch SPROM revision */
		*srev = *((uint8_t *)ptr + SPROM_REV_OFF(nr));
		
		/* Early sromrev 1 devices (specifically some BCM440x enet
		 * cards) are reported to have been incorrectly programmed
		 * with a revision of 0x10. */
		if (fmt->size == SPROM_SZ_R1_3 && *srev == 0x10)
			*srev = 0x1;
		
		/* Verify revision range */
		if (*srev < fmt->rev_min || *srev > fmt->rev_max)
			continue;

		/* Verify signature (if any) */
		sig = SPROM_SIG_NONE;
		if (fmt->sig_offset != SPROM_SIG_NONE_OFF) {
			error = bhnd_nvram_io_read(buf, fmt->sig_offset, &sig,
			    sizeof(sig));
			if (error)
				goto failed;

			sig = le16toh(sig);
		}
		
		if (sig != fmt->sig_req) {
			SPROM_NVLOG("invalid sprom %hhu signature: 0x%hx "
			    "(expected 0x%hx)\n", *srev, sig, fmt->sig_req);
			continue;
		}

		/* Identified */
		*shadow = buf;
		return (0);
	}

	/* No match -- set error and fallthrough */
	error = ENXIO;

failed:
	bhnd_nvram_io_free(buf);
	return (error);
}

static int
bhnd_nvram_sprom_probe(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_io	*shadow;
	uint8_t			 srev;
	int			 error;

	/* Try to parse the input */
	if ((error = bhnd_nvram_sprom_parse(io, &srev, &shadow)))
		return (error);

	/* Clean up the shadow iobuf */
	bhnd_nvram_io_free(shadow);

	return (BUS_PROBE_DEFAULT);
}

static int
bhnd_nvram_sprom_new(struct bhnd_nvram_data **nv, struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_sprom	*sp;
	int			 error;
	
	/* Allocate and initialize the SPROM data instance */
	sp = malloc(sizeof(*sp), M_BHND_NVRAM, M_NOWAIT|M_ZERO);
	if (sp == NULL)
		return (ENOMEM);

	sp->nv.cls = &bhnd_nvram_sprom_class;
	sp->data = NULL;
	
	/* Parse the SPROM input data */
	if ((error = bhnd_nvram_sprom_parse(io, &sp->srev, &sp->data)))
		goto failed;

	*nv = &sp->nv;
	return (0);
	
failed:
	if (sp->data != NULL)
		bhnd_nvram_io_free(sp->data);

	free(sp, M_BHND_NVRAM);

	return (error);
}

static void
bhnd_nvram_sprom_free(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_sprom *sp = (struct bhnd_nvram_sprom *)nv;
	
	bhnd_nvram_io_free(sp->data);
	free(sp, M_BHND_NVRAM);
}

static int
bhnd_nvram_sprom_size(struct bhnd_nvram_data *nv, size_t *size)
{
	struct bhnd_nvram_sprom *sprom = (struct bhnd_nvram_sprom *)nv;

	/* The serialized form will be identical in length
	 * to our backing buffer representation */
	*size = bhnd_nvram_io_getsize(sprom->data);
	return (0);
}

static int
bhnd_nvram_sprom_serialize(struct bhnd_nvram_data *nv, void *buf, size_t *len)
{
	struct bhnd_nvram_sprom	*sprom;
	size_t			 limit, req_len;
	int			 error;

	sprom = (struct bhnd_nvram_sprom *)nv;
	limit = *len;

	/* Provide the required size */
	if ((error = bhnd_nvram_sprom_size(nv, &req_len)))
		return (error);

	*len = req_len;

	if (buf == NULL) {
		return (0);
	} else if (*len > limit) {
		return (ENOMEM);
	}

	/* Write to the output buffer */
	return (bhnd_nvram_io_read(sprom->data, 0x0, buf, *len));
}

static uint32_t
bhnd_nvram_sprom_getcaps(struct bhnd_nvram_data *nv)
{
	return (BHND_NVRAM_DATA_CAP_INDEXED);
}

static const char *
bhnd_nvram_sprom_next(struct bhnd_nvram_data *nv, void **cookiep)
{
	struct bhnd_nvram_sprom		*sp;
	const struct bhnd_nvram_vardefn	*var, *table;
	size_t				 elem, nelem;

	sp = (struct bhnd_nvram_sprom *)nv;

	table = bhnd_nvram_vardefn_table();
	nelem = bhnd_nvram_vardefn_count();
	
	if (*cookiep == NULL) {
		/* Start iteration at the first NVRAM table entry */
		elem = 0;
	} else {
		/* Start iteration at the next NVRAM table entry */
		var = SPROM_COOKIE_TO_NVRAM(*cookiep);
		elem = var - table;

		KASSERT(elem < nelem, ("invalid cookie pointer"));
		elem++;
	}

	/* Find the next matching entry */
	while (elem < nelem) {
		var = &table[elem];
		*cookiep = SPROM_NVRAM_TO_COOKIE(var);

		if (bhnd_nvram_sprom_matches(sp, var, NULL))
			return (var->name);

		elem++;
	}

	/* Not found */
	return (NULL);
}

static void *
bhnd_nvram_sprom_find(struct bhnd_nvram_data *nv, const char *name)
{
	struct bhnd_nvram_sprom		*sp;
	const struct bhnd_nvram_vardefn	*var;

	sp = (struct bhnd_nvram_sprom *)nv;

	/* Look up the variable entry */
	if ((var = bhnd_nvram_find_vardefn(name)) == NULL)
		return (NULL);

	/* Verify that it matches our SPROM revision */
	if (!bhnd_nvram_sprom_matches(sp, var, NULL))
		return (NULL);

	/* Valid */
	return (SPROM_NVRAM_TO_COOKIE(var));
}

static int
bhnd_nvram_sprom_getvar(struct bhnd_nvram_data *nv, void *cookiep, void *buf,
    size_t *len, bhnd_nvram_type otype)
{
	struct bhnd_nvram_sprom		*sp;
	const struct bhnd_nvram_vardefn	*var;
	const struct bhnd_sprom_vardefn	*sp_def;
	struct bhnd_nvram_fmt_hint	 hint;
	union bhnd_nvram_sprom_storage	 storage;
	union bhnd_nvram_sprom_storage	*inp;
	union bhnd_nvram_sprom_intv	 intv;
	size_t				 ilen;
	size_t				 ipos;
	size_t				 nelem, nelem_all1;
	size_t				 iwidth, owidth;
	int				 error;

	sp = (struct bhnd_nvram_sprom *)nv;

	KASSERT(cookiep != NULL, ("NULL variable cookiep"));

	/* Fetch NVRAM and SPROM definitions */
	var = SPROM_COOKIE_TO_NVRAM(cookiep);
	if (!bhnd_nvram_sprom_matches(sp, var, &sp_def)) {
		/* Invalid cookie? */
		panic("missing sprom definition for '%s'", var->name);
	}

	nelem = bhnd_nvram_spromdef_nelems(sp_def);
	iwidth = bhnd_nvram_type_width(var->type);
	ilen = nelem * iwidth;

	/* SPROM does not use (and we do not support) decoding of
	 * variable-width data types */
	if (iwidth == 0) {
		SPROM_NVLOG("invalid SPROM data type: %d", var->type);
		return (EFTYPE);
	}

	/* Fetch caller's destination type info */
	owidth = bhnd_nvram_type_width(otype);

	/* If operating entirely on fixed type widths, we can
	 * handle size requests without performing any parsing. */
	if (buf == NULL && iwidth > 0 && owidth > 0) {
		*len = nelem * owidth;
		return (0);
	}

	/* If the caller has requested the native variable representation,
	 * we can decode directly into a supplied buffer.
	 *
	 * Otherwise, we need to decode into our own local storage, and then
	 * perform value coercion. */
	if (buf != NULL && otype == var->type) {
		inp = buf;
		if (ilen > *len)
			return (ENOMEM);
	} else {
		inp = &storage;
		if (ilen > sizeof(storage)) {
			SPROM_NVLOG("error decoding '%s', SPROM_ARRAY_MAXLEN "
			    "incorrect\n", var->name);
			return (EFTYPE);
		}
	}

	/* Decode the SPROM data */
	nelem_all1 = 0;
	ipos = 0;
	intv.u32 = 0x0;
	for (size_t i = 0; i < sp_def->num_offsets; i++) {
		const struct bhnd_sprom_offset	*off;
		bhnd_nvram_type			 intv_type;
		size_t				 nbyte;
		void				*ptr;
		union {
			uint8_t		u8;
			uint16_t	u16;
			uint32_t	u32;
			int8_t		s8;
			int16_t		s16;
			int32_t		s32;
			char		ch;
		} offv;

		off = &sp_def->offsets[i];

		KASSERT(i > 0 || !off->cont, ("cont marked on first offset"));
		KASSERT(ipos < nelem, ("output positioned past last element"));

#define	NV_READ_INT(_src, _dst, _swap)	do {				\
	error = bhnd_nvram_io_read(sp->data, off->offset, &offv. _src,	\
	    sizeof(offv. _src));					\
	if (error) {							\
		SPROM_NVLOG("error reading SPROM offset %#hx: %d\n",	\
		    off->offset, error);				\
		return (EFTYPE);					\
	}								\
									\
	offv. _src = _swap(offv. _src);					\
	if (off->shift > 0) {						\
		offv. _src >>= off->shift;				\
	} else if (off->shift < 0) {					\
		offv. _src <<= -off->shift;				\
	}								\
	offv. _src &= off->mask;					\
	intv. _dst |= offv. _src;					\
} while(0)
	
		/* Read the value, widening to a common 32bit representation */
		switch (off->type) {
		case BHND_NVRAM_TYPE_UINT8:
			NV_READ_INT(u8, u32,);
			break;
		case BHND_NVRAM_TYPE_UINT16:
			NV_READ_INT(u16, u32, le16toh);
			break;
		case BHND_NVRAM_TYPE_UINT32:
			NV_READ_INT(u32, u32, le32toh);
			break;
		case BHND_NVRAM_TYPE_INT8:
			NV_READ_INT(s8, s32,);
			break;
		case BHND_NVRAM_TYPE_INT16:
			NV_READ_INT(s16, s32, le16toh);
			break;
		case BHND_NVRAM_TYPE_INT32:
			NV_READ_INT(s32, s32, le32toh);
			break;
		case BHND_NVRAM_TYPE_CHAR:
			NV_READ_INT(ch, s32,);
			break;

		case BHND_NVRAM_TYPE_CSTR:
			/* fallthrough (unused by SPROM) */
		default:
			SPROM_NVLOG("unhandled offset type: %d\n", off->type);
			return (EFTYPE);
		}
		
		/* If IGNALL1, record whether value has all bits set. */
		if (var->flags & BHND_NVRAM_VF_IGNALL1) {
			uint32_t all1;

			all1 = off->mask;
			if (off->shift > 0)
				all1 >>= off->shift;
			else if (off->shift < 0)
				all1 <<= -off->shift;

			if ((intv.u32 & all1) == all1)
				nelem_all1++;
		}

		/* Skip writing to inp if additional continuations remain */
		if (i+1 < sp_def->num_offsets && sp_def->offsets[i].cont)
			continue;

		/* Use standard coercion support to perform overflow-checked
		 * coercion from parsed uint32/int32 offset value, to the
		 * actual variable type */
		if (BHND_NVRAM_SIGNED_TYPE(var->type))
			intv_type = BHND_NVRAM_TYPE_INT32;
		else
			intv_type = BHND_NVRAM_TYPE_UINT32;

		/* Calculate address of the current element output position */
		ptr = (uint8_t *)inp + (iwidth * ipos);

		/* Perform coercion of the array element */
		nbyte = iwidth;
		error = bhnd_nvram_coerce_value(ptr, &nbyte, var->type,
		    &intv, sizeof(intv), intv_type, NULL);
		if (error)
			return (error);

		/* Clear temporary state and advance our position */
		intv.u32 = 0x0;
		ipos++;
	}

	/* If we were decoding directly to the caller's output buffer, nothing
	 * left to do but provide the decoded length */
	if (inp == buf) {
		*len = ilen;
		return (0);
	}

	/* Otherwise, perform value coercion from our local storage */
	hint = (struct bhnd_nvram_fmt_hint) {
		.sfmt = var->sfmt,
		.flags = var->flags
	};

	return (bhnd_nvram_coerce_value(buf, len, otype, inp, ilen, var->type,
	    &hint));
}

static const void *
bhnd_nvram_sprom_getvar_ptr(struct bhnd_nvram_data *nv, void *cookiep,
			  size_t *len, bhnd_nvram_type *type)
{
	/* Unsupported */
	return (NULL);
}

static const char *
bhnd_nvram_sprom_getvar_name(struct bhnd_nvram_data *nv, void *cookiep)
{
	const struct bhnd_nvram_vardefn	*var;

	KASSERT(cookiep != NULL, ("NULL variable cookiep"));

	var = SPROM_COOKIE_TO_NVRAM(cookiep);
	return (var->name);
}

/**
 * Return true if @p var is applicable to our SPROM revision, false otherwise.
 *
 * @param sprom SPROM data instance.
 * @param var The variable definition to check for applicability to @p sprom.
 * @param[out] sprom_def On success, the matching SPROM definition for @p var.
 * May be NULL.
 */
static bool
bhnd_nvram_sprom_matches(struct bhnd_nvram_sprom *sprom,
    const struct bhnd_nvram_vardefn *var,
    const struct bhnd_sprom_vardefn **sprom_def)
{
	/* Look for a SPROM definition that matches our SPROM
	 * revision */
	for (size_t i = 0; i < var->num_sp_defs; i++) {
		const struct bhnd_sprom_vardefn *sp = &var->sp_defs[i];
		
		/* Check the range */
		if (sprom->srev < sp->compat.first)
			continue;
		
		if (sprom->srev > sp->compat.last)
			continue;
		
		/* Match */
		if (sprom_def != NULL)
			*sprom_def = sp;
		return (true);
	}
	
	/* No match */
	return (false);
}

/**
 * Return the total number of array elements defined by @p sprom_def.
 */
static size_t
bhnd_nvram_spromdef_nelems(const struct bhnd_sprom_vardefn *sprom_def)
{
	size_t nelem;

	nelem = 0;
	for (size_t j = 0; j < sprom_def->num_offsets; j++) {
		/* Skip continuations */
		if (!sprom_def->offsets[j].cont)
			nelem += 1;
	}

	return (nelem);
}
