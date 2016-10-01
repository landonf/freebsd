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
#include <sys/ctype.h>
#include <sys/endian.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "bhnd_nvram_codec.h"
#include "bhnd_nvram_codecvar.h"

#include "bhnd_nvram_parserreg.h"
#include "bhnd_nvram_parservar.h"

/*
 * BHND NVRAM Parser
 * 
 * Provides identification, decoding, and encoding of BHND NVRAM data.
 */

static int	 bhnd_nvram_sort_idx(void *ctx, const void *lhs,
		     const void *rhs);
static int	 bhnd_nvram_generate_index(struct bhnd_nvram *sc);
static void	*bhnd_nvram_index_lookup(struct bhnd_nvram *sc,
		     const char *name);

#define	NVRAM_LOG(sc, fmt, ...)	do {			\
	if (sc->dev != NULL)					\
		device_printf(sc->dev, fmt, ##__VA_ARGS__);	\
	else							\
		printf("bhnd_nvram: " fmt, ##__VA_ARGS__);	\
} while (0)

/* Limit a size_t value to a suitable range for use as a printf string field
 * width */
#define	NVRAM_PRINT_WIDTH(_len)	\
	((_len) > BHND_NVRAM_VAL_MAXLEN ? BHND_NVRAM_VAL_MAXLEN : (int)(_len))


static bhnd_nvram_codec_class_t *
bhnd_nvram_get_parser_class(bhnd_nvram_format fmt)
{
	switch (fmt) {
	case BHND_NVRAM_FMT_BCM:
		// TODO
		// return (&bhnd_nvram_bcm_class);
		return (NULL);

	case BHND_NVRAM_FMT_TLV:
		return (&bhnd_nvram_tlv_class);
		
	case BHND_NVRAM_FMT_BTXT:
		return (&bhnd_nvram_btxt_class);
		
	default:
		printf("%s: unknown format: %d\n", __FUNCTION__, fmt);
		return (NULL);
	}
}

/**
 * Identify the NVRAM format of @p io.
 * 
 * @param io I/O context mapping the NVRAM data to be identified.
 * @param expected Expected format against which @p ident will be tested.
 * @param[out] size_hint If not NULL, will be set to the maximum possible size
 * of the NVRAM data (which may be less than the input @p io size), if @p io
 * is sucessfully identified.
 *
 * @retval 0 If @p ident has the @p expected format.
 * @retval ENODEV If @p ident does not match @p expected.
 * @retval non-zero If reading from @p io otherwise fails, a standard unix
 * error code will be returned.
 */
int
bhnd_nvram_parser_identify(struct bhnd_nvram_io *io,
    bhnd_nvram_format expected, size_t *size_hint)
{
	bhnd_nvram_codec_class_t *cls;

	if ((cls = bhnd_nvram_get_parser_class(expected)) == NULL)
		return (ENODEV);
	
	if (size_hint != NULL)
		*size_hint = bhnd_nvram_io_get_size(io);

	if (bhnd_nvram_codec_probe(cls, io) <= 0)
		return (0);

	return (ENODEV);
}

/**
 * Identify the NVRAM format at @p offset within @p r, verify the
 * CRC (if applicable), and allocate a local shadow copy of the NVRAM data.
 * 
 * After initialization, no reference to @p io will be held by the
 * NVRAM parser, and @p io may be safely deallocated.
 * 
 * @param[out] sc The NVRAM parser state to be initialized.
 * @param dev The parser's parent device, or NULL if none.
 * @param io An I/O context mapping the NVRAM data to be parsed.
 * @param fmt Required format of @p io.
 * 
 * @retval 0 success
 * @retval ENOMEM If internal allocation of NVRAM state fails.
 * @retval EINVAL If @p io parsing fails.
 */
int
bhnd_nvram_parser_init(struct bhnd_nvram *sc, device_t dev,
    struct bhnd_nvram_io *io, bhnd_nvram_format fmt)
{
	bhnd_nvram_codec_class_t	*cls;
	int				 error;

	/* Initialize NVRAM state */
	memset(sc, 0, sizeof(*sc));
	sc->dev = dev;
	LIST_INIT(&sc->devpaths);

	/* Verify data format and fetch parser class */
	if ((error = bhnd_nvram_parser_identify(io, fmt, NULL)))
		return (error);

	cls = bhnd_nvram_get_parser_class(fmt);

	/* Allocate backing parser */
	if ((error = bhnd_nvram_codec_new(cls, &sc->nv, io)))
		return (error);

	/* Allocate uncommitted change list */
	sc->pending = nvlist_create(NV_FLAG_IGNORE_CASE);
	if (sc->pending == NULL)
		goto cleanup;

	/* Generate all indices */
	if ((error = bhnd_nvram_generate_index(sc)))
		goto cleanup;

	return (0);

cleanup:
	bhnd_nvram_parser_fini(sc);
	return (error);
}


/**
 * Release all resources held by @p sc.
 * 
 * @param sc A NVRAM instance previously initialized via
 * bhnd_nvram_parser_init().
 */
void
bhnd_nvram_parser_fini(struct bhnd_nvram *sc)
{
	struct bhnd_nvram_devpath	*dpath, *dnext;

        LIST_FOREACH_SAFE(dpath, &sc->devpaths, dp_link, dnext) {
		free(dpath->path, M_BHND_NVRAM);
                free(dpath, M_BHND_NVRAM);
        }

	if (sc->pending != NULL)
		nvlist_destroy(sc->pending);

	if (sc->idx != NULL)
		free(sc->idx, M_BHND_NVRAM);

	if (sc->nv != NULL)
		bhnd_nvram_codec_free(sc->nv);
}

/**
 * Read an NVRAM variable.
 *
 * @param              sc      The NVRAM parser state.
 * @param              name    The NVRAM variable name.
 * @param[out]         buf     On success, the requested value will be written
 *                             to this buffer. This argment may be NULL if
 *                             the value is not desired.
 * @param[in,out]      len     The capacity of @p buf. On success, will be set
 *                             to the actual size of the requested value.
 * @param              type    The requested data type to be written to @p buf.
 *
 * @retval 0           success
 * @retval ENOENT      The requested variable was not found.
 * @retval ENOMEM      If @p buf is non-NULL and a buffer of @p len is too
 *                     small to hold the requested value.
 * @retval non-zero    If reading @p name otherwise fails, a regular unix
 *                     error code will be returned.
  */
int
bhnd_nvram_parser_getvar(struct bhnd_nvram *sc, const char *name, void *buf,
    size_t *len, bhnd_nvram_type type)
{
	const char	*next;
	void		*cookiep;
	const void	*inp;
	size_t		 ilen;
	bhnd_nvram_type	 itype;

	/*
	 * Search order:
	 *
	 * - uncommitted changes
	 * - index lookup OR buffer scan
	 */

	/* Is variable marked for deletion? */
	if (nvlist_exists_null(sc->pending, name)) {
		return (ENOENT);
	}

	/* Does an uncommitted value exist? */
	if (nvlist_exists_string(sc->pending, name)) {
		/* Uncommited value exists, is not a deletion */
		inp = nvlist_get_string(sc->pending, name);
		ilen = strlen(inp) + 1;
		itype = BHND_NVRAM_TYPE_CSTR;

		return (bhnd_nvram_coerce_value(buf, len, type, inp, ilen,
		    itype, NULL));
	} else if (nvlist_exists(sc->pending, name)) {
		panic("invalid value type for pending change %s", name);
	}

	/* Fetch variable from backing parser. We use the index if available;
	 * otherwise, perform a full scan */
	if (sc->idx != NULL) {
		if ((cookiep = bhnd_nvram_index_lookup(sc, name)) == NULL)
			return (ENOENT);
	} else {
		cookiep = NULL;
		while ((next = bhnd_nvram_codec_next(sc->nv, &cookiep))) {
			if (strcasecmp(name, next) == 0)
				break;
		}

		/* Hit end without a match */
		if (name == NULL)
			return (ENOENT);
	}

	/* Let the parser itself perform value coercion */
	return (bhnd_nvram_codec_getvar(sc->nv, cookiep, buf, len, type));
}

/**
 * Set an NVRAM variable.
 * 
 * @param		sc	The NVRAM parser state.
 * @param		name	The NVRAM variable name.
 * @param[out]		buf	The new value.
 * @param[in,out]	len	The size of @p buf.
 * @param		type	The data type of @p buf.
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable was not found.
 * @retval EINVAL	If @p len does not match the expected variable size.
 */
int
bhnd_nvram_parser_setvar(struct bhnd_nvram *sc, const char *name,
    const void *buf, size_t len, bhnd_nvram_type type)
{
	const char	*inp;
	char		 vbuf[BHND_NVRAM_VAL_MAXLEN];

	/* Verify name validity */
	if (!bhnd_nvram_validate_name(name, strlen(name)))
		return (EINVAL);

	/* Verify buffer size alignment for the given type. If this is a
	 * variable width type, a width of 0 will always pass this check */
	if (len % bhnd_nvram_type_width(type) != 0)
		return (EINVAL);

	/* Determine string format (or directly add variable, if a C string) */
	switch (type) {
	case BHND_NVRAM_TYPE_UINT8:
	case BHND_NVRAM_TYPE_UINT16:
	case BHND_NVRAM_TYPE_UINT32:
	case BHND_NVRAM_TYPE_INT8:
	case BHND_NVRAM_TYPE_INT16:
	case BHND_NVRAM_TYPE_INT32:
		// TODO: primitive type value support
		return (EOPNOTSUPP);

	case BHND_NVRAM_TYPE_CHAR:
	case BHND_NVRAM_TYPE_CSTR:
		inp = buf;

		/* Must not exceed NVRAM_VAL_MAX */
		if (len > sizeof(vbuf))
			return (EINVAL);

		/* Must have room for a trailing NUL */
		if (len == sizeof(vbuf) && inp[len-1] != '\0')
			return (EINVAL);

		/* Copy out the string value and append trailing NUL */
		strlcpy(vbuf, buf, len);
	
		/* Add to pending change list */
		nvlist_add_string(sc->pending, name, vbuf);
	}

	return (0);
}

/* sort function for bhnd_nvram_idx_entry values */
static int
bhnd_nvram_sort_idx(void *ctx, const void *lhs, const void *rhs)
{
	struct bhnd_nvram			*sc;
	const struct bhnd_nvram_idx_entry	*l_idx, *r_idx;
	const char				*l_str, *r_str;

	sc = ctx;
	l_idx = lhs;
	r_idx = rhs;

	/* Fetch string pointers */
	l_str = bhnd_nvram_codec_getvar_name(sc->nv, l_idx->cookiep);
	r_str = bhnd_nvram_codec_getvar_name(sc->nv, r_idx->cookiep);

	/* Perform comparison */
	return (strcasecmp(l_str, r_str));
}


/**
 * Generate all indices for the NVRAM data backing @p nvram.
 * 
 * @param sc		The NVRAM parser state.
 *
 * @retval 0		success
 * @retval non-zero	If indexing @p nvram fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvram_generate_index(struct bhnd_nvram *sc)
{
	const char	*name;
	void		*cookiep;
	size_t		 idx_bytes;
	size_t		 num_records;
	int		 error;

	num_records = 0;

	/* Parse and register all device path aliases */
	cookiep = NULL;
	while ((name = bhnd_nvram_codec_next(sc->nv, &cookiep))) {
		struct bhnd_nvram_devpath	*devpath;
		bhnd_nvram_type			 path_type;
		const char			*path, *suffix;
		char				*eptr;
		char				 pathbuf[BHND_NVRAM_VAL_MAXLEN];
		size_t				 path_len;
		u_long				 index;

		num_records++;

		/* Check for devpath prefix */
		if (strncmp(name, NVRAM_DEVPATH_STR, NVRAM_DEVPATH_LEN) != 0)
			continue;

		/* Parse index value that should follow a 'devpath' prefix */
		suffix = name + NVRAM_DEVPATH_LEN;
		index = strtoul(suffix, &eptr, 10);
		if (eptr == suffix || *eptr != '\0') {
			NVRAM_LOG(sc, "invalid devpath variable '%s'\n", name);
			continue;
		}

		/* Try to read the target path */
		path = bhnd_nvram_codec_getvar_ptr(sc->nv, cookiep, &path_len,
		    &path_type);
		if (path == NULL || path_type != BHND_NVRAM_TYPE_CSTR) {
			path_len = sizeof(pathbuf);
			error = bhnd_nvram_codec_getvar(sc->nv, cookiep,
			    &pathbuf, &path_len, BHND_NVRAM_TYPE_CSTR);
			if (error)
				return (error);
		}

		/* Register path alias */
		devpath = malloc(sizeof(*devpath), M_BHND_NVRAM, M_NOWAIT);
		if (devpath == NULL)
			return (ENOMEM);

		devpath->index = index;
		devpath->path = strndup(path, path_len, M_BHND_NVRAM);
		LIST_INSERT_HEAD(&sc->devpaths, devpath, dp_link);
	}

	/* Skip generating variable index if threshold is not met */
	if (num_records < NVRAM_IDX_VAR_THRESH)
		return (0);

	/* Allocate and populate variable index */
	idx_bytes = sizeof(struct bhnd_nvram_idx) +
	    (sizeof(struct bhnd_nvram_idx_entry) * num_records);
	sc->idx = malloc(idx_bytes, M_BHND_NVRAM, M_NOWAIT);
	if (sc->idx == NULL) {
		NVRAM_LOG(sc, "error allocating %zu byte index\n", idx_bytes);
		goto bad_index;
	}

	sc->idx->num_entries = num_records;

	if (bootverbose) {
		NVRAM_LOG(sc, "allocated %zu byte index for %zu variables\n",
		    idx_bytes, num_records);
	}

	cookiep = NULL;
	for (size_t i = 0; i < sc->idx->num_entries; i++) {
		struct bhnd_nvram_idx_entry *idx;

		/* Fetch next entry */
		name = bhnd_nvram_codec_next(sc->nv, &cookiep);

		/* Early EOF */
		if (name == NULL) {
			NVRAM_LOG(sc, "indexing failed, expected %zu records "
			    "(got %zu)\n", sc->idx->num_entries, i+1);
			goto bad_index;
		}

		/* Save the variable reference */
		idx = &sc->idx->entries[i];
		idx->cookiep = cookiep;
	}

	/* Sort the index table */
	qsort_r(sc->idx->entries, sc->idx->num_entries,
	    sizeof(sc->idx->entries[0]), sc, bhnd_nvram_sort_idx);

	return (0);

bad_index:
	/* Fall back on non-indexed access */
	NVRAM_LOG(sc, "reverting to non-indexed variable lookup\n");
	if (sc->idx != NULL) {
		free(sc->idx, M_BHND_NVRAM);
		sc->idx = NULL;
	}

	return (0);
}


/**
 * Perform an index lookup of @p name, returning the associated cookie
 * value, or NULL if the variable does not exist.
 *
 * @param	sc		The NVRAM parser state.
 * @param	name		The variable to search for.
 */
static void *
bhnd_nvram_index_lookup(struct bhnd_nvram *sc, const char *name)
{
	struct bhnd_nvram_idx_entry	*idxe;
	const char			*idxe_key;
	size_t				 min, mid, max;
	int				 order;

	if (sc->idx == NULL || sc->idx->num_entries == 0)
		return (NULL);

	/*
	 * Locate the requested variable using a binary search.
	 */
	min = 0;
	max = sc->idx->num_entries - 1;

	while (max >= min) {
		/* Select midpoint */
		mid = (min + max) / 2;
		idxe = &sc->idx->entries[mid];

		/* Determine which side of the partition to search */
		idxe_key = bhnd_nvram_codec_getvar_name(sc->nv, idxe->cookiep);
		order = strcasecmp(idxe_key, name);

		if (order < 0) {
			/* Search upper partition */
			min = mid + 1;
		} else if (order > 0) {
			/* Search lower partition */
			max = mid - 1;
		} else if (order == 0) {
			/* Match found */
			return (idxe->cookiep);
		}
	}

	/* Not found */
	return (NULL);
}

// legacy
#if 0

/* FMT_BCM NVRAM data size calculation */
static int
bhnd_nvram_bcm_getsize(const void *data, size_t *size)
{
	const struct bhnd_nvram_header *hdr;

	if (*size < sizeof(*hdr))
		return (EINVAL);

	hdr = (const struct bhnd_nvram_header *) data;
	*size = le32toh(hdr->size);
	return (0);
}

/* FMT_BCM-specific parser initialization */
static int
bhnd_nvram_bcm_init(struct bhnd_nvram *sc)
{
	const uint8_t	*p;
	uint32_t	 cfg0;
	uint8_t		 crc, valid;

	/* Validate CRC */
	if (sc->buf_size < NVRAM_CRC_SKIP)
		return (EINVAL);

	if (sc->buf_size < sizeof(struct bhnd_nvram_header))
		return (EINVAL);

	cfg0 = ((struct bhnd_nvram_header *)sc->buf)->cfg0;
	valid = (cfg0 & NVRAM_CFG0_CRC_MASK) >> NVRAM_CFG0_CRC_SHIFT;

	p = sc->buf;
	crc = bhnd_nvram_crc8(p + NVRAM_CRC_SKIP, sc->buf_size-NVRAM_CRC_SKIP,
	    BHND_NVRAM_CRC8_INITIAL);

	if (crc != valid) {
		NVRAM_LOG(sc, "warning: NVRAM CRC error (crc=%#hhx, "
		    "expected=%hhx)\n", crc, valid);
	}

	return (0);
}

/* Populate FMT_BCM-specific default values */
static int
bhnd_nvram_bcm_init_defaults(struct bhnd_nvram *sc)
{
	struct bhnd_nvram_header	*header;
	char				 vbuf[BHND_NVRAM_VAL_MAXLEN];
	uint32_t			 value;
	int				 nwrite;

	/* Verify that our header is readable */
	header = (struct bhnd_nvram_header *) sc->buf;
	if (!bhnd_nvram_bufptr_valid(sc, header, sizeof(*header), true))
		return (EINVAL);

	/* Extract a value from the NVRAM header, format it, and
	 * register a new default variable tuple */
#define	NVRAM_BCM_HEADER_DEFAULT(_field, _name)	do {			\
	value = NVRAM_GET_BITS(le32toh(header->_field), _name);		\
	nwrite = snprintf(vbuf, sizeof(vbuf), _name ##_FMT, value);	\
	if (nwrite < 0 || nwrite >= sizeof(vbuf)) {			\
		NVRAM_LOG(sc, "%s: formatting '%s' failed: %d\n",	\
		    __FUNCTION__, _name ## _VAR, nwrite);		\
		return (ENXIO);						\
	}								\
	nvlist_add_string(sc->defaults, _name ##_VAR, vbuf);		\
} while(0)

	NVRAM_BCM_HEADER_DEFAULT(cfg0,		NVRAM_CFG0_SDRAM_INIT);
	NVRAM_BCM_HEADER_DEFAULT(cfg1,		NVRAM_CFG1_SDRAM_CFG);
	NVRAM_BCM_HEADER_DEFAULT(cfg1,		NVRAM_CFG1_SDRAM_REFRESH);
	NVRAM_BCM_HEADER_DEFAULT(sdram_ncdl,	NVRAM_SDRAM_NCDL);

#undef	NVRAM_BCM_HEADER_DEFAULT

	return (0);
}

/* FMT_BCM record length */
static int
bhnd_nvram_bcm_env_len(struct bhnd_nvram *sc, const char *env, size_t *env_len)
{
	if (!bhnd_nvram_bufptr_valid(sc, env, 1, true))
		return (EINVAL);

	*env_len = strnlen(env, sc->buf_size - ((const uint8_t *)env - sc->buf));
	return (0);
}

/* FMT_BCM record parsing */
static int
bhnd_nvram_bcm_enum_buf(struct bhnd_nvram *sc, const char **env, size_t *len,
    const uint8_t *p, uint8_t const **next)
{
	int error;

	/* First record is found following the NVRAM header */
	if (p == NULL)
		p = sc->buf + sizeof(struct bhnd_nvram_header);

	if (!bhnd_nvram_bufptr_valid(sc, p, 1, true))
		return (EINVAL);

	/* EOF */
	if (*p == '\0') {
		*env = NULL;
		*len = 0;
		*next = p;
		return (0);
	}

	/* Provide pointer to env data and determine the record length */
	*env = p;
	if ((error = bhnd_nvram_bcm_env_len(sc, *env, len)))
		return (error);

	/* Advance to next entry and skip terminating NUL */
	p += *len;
	if (bhnd_nvram_bufptr_valid(sc, p, 1, false)) {
		p++;
	} else {
		NVRAM_LOG(sc, "warning: missing NVRAM termination record");
	}

	*next = p;
	return (0);
}

/* FMT_TLV NVRAM data size calculation */
static int
bhnd_nvram_tlv_getsize(const void *data, size_t *size)
{
	const uint8_t	*const start = data;
	size_t		 offset;
	uint16_t	 rlen;

	offset = 0;
	while (offset < *size) {
		uint8_t type;

		/* Fetch type */
		type = *(start+offset);

		/* EOF */
		if (type == NVRAM_TLV_TYPE_END) {
			*size = offset + 1;
			return (0);
		}

		if ((offset++) == *size)
			return (EINVAL);

		/* Determine record length */
		if (type & NVRAM_TLV_TF_U8_LEN) {
			rlen = *(start+offset);
		} else {
			rlen = *(start+offset) << 8;
			if ((offset++) == *size)
				return (EINVAL);
			rlen |= *(start+offset);
		}

		if ((offset++) >= *size)
			return (EINVAL);

		/* Advance to next entry */
		if (rlen > *size || *size - rlen < offset)
			return (EINVAL);

		offset += rlen;
	}

	/* EOF not found */
	return (EINVAL);
}

/* FMT_TLV-specific parser initialization */
static int
bhnd_nvram_tlv_init(struct bhnd_nvram *sc)
{
	return (0);
}

/* Populate FMT_TLV-specific default values */
static int
bhnd_nvram_tlv_init_defaults(struct bhnd_nvram *sc)
{
	return (0);
}

/* FMT_TLV record length */
static int
bhnd_nvram_tlv_env_len(struct bhnd_nvram *sc, const char *env, size_t *env_len)
{
	if (!bhnd_nvram_bufptr_valid(sc, env, 1, true))
		return (EINVAL);

	// XXX TODO: we really need an offset to the actual
	// entry, not the env string
	return (ENXIO);
}

/* FMT_TLV record parsing */
static int
bhnd_nvram_tlv_enum_buf(struct bhnd_nvram *sc, const char **env, size_t *len,
    const uint8_t *p, uint8_t const **next)
{
	size_t		 rlen;
	uint8_t		 type;

	if (p == NULL)
		p = sc->buf;

	/* Fetch type */
	if (!bhnd_nvram_bufptr_valid(sc, p, 1, true))
		return (EINVAL);

	type = *p;

	/* EOF */
	if (type == NVRAM_TLV_TYPE_END) {
		*env = NULL;
		*len = 0;
		*next = p;
		return (0);
	}

	/* Determine record length */
	p++;
	if (type & NVRAM_TLV_TF_U8_LEN) {
		if (!bhnd_nvram_bufptr_valid(sc, p, 1, true))
			return (EINVAL);
	
		rlen = *p;
		p += 1;
	} else {
		if (!bhnd_nvram_bufptr_valid(sc, p, 2, true))
			return (EINVAL);
		rlen = (p[0] << 8) | (p[1]);
		p += 2;
	}

	/* Verify record readability */
	if (!bhnd_nvram_bufptr_valid(sc, p, rlen, true))
		return (EINVAL);

	/* Error on non-env records */
	if (type != NVRAM_TLV_TYPE_ENV) {
		NVRAM_LOG(sc, "unsupported NVRAM TLV tag: %#hhx\n", type);
		return (EINVAL);
	}

	/* Skip flag field */
	if (rlen < 1)
		return (EINVAL);
	p++;
	rlen--;

	/* Provide pointer to env data */
	*env = p;
	*len = strnlen(*env, rlen);

	/* Advance to next entry */
	*next = p + rlen;

	return (0);
}

/* FMT_BTXT NVRAM data size calculation */
static int
bhnd_nvram_txt_getsize(const void *data, size_t *size)
{
	*size = (strnlen(data, *size));
	return (0);
}

/* FMT_BTXT-specific parser initialization */
static int
bhnd_nvram_txt_init(struct bhnd_nvram *sc)
{
	return (0);
}

/* Populate FMT_BTXT-specific default values */
static int
bhnd_nvram_txt_init_defaults(struct bhnd_nvram *sc)
{
	return (0);
}

/* Seek past the next line ending (\r, \r\n, or \n) */
static const uint8_t *
bhnd_nvram_txt_seek_eol(struct bhnd_nvram *sc, const uint8_t *p)
{
	while (p < sc->buf + sc->buf_size) {
		switch (*p) {
		case '\r':
			/* \r\n */
			if (bhnd_nvram_bufptr_valid(sc, p, 1, false)) {
				if (*(p+1) == '\n')
					p++;
			}

			return (p+1);
		case '\n':
			return (p+1);
		default:
			p++;
			break;
		}
	}

	return (p);
}

/* Seek to the next valid key=value entry (or EOF) */
static const uint8_t *
bhnd_nvram_txt_seek_nextline(struct bhnd_nvram *sc, const uint8_t *p)
{
	/* Skip leading whitespace and comments */
	while (p < sc->buf + sc->buf_size) {
		if (isspace(*p)) {
			p++;
			continue;
		}
		
		if (*p == '#') {
			p = bhnd_nvram_txt_seek_eol(sc, p);
			continue;
		}
		
		break;
	}

	return (p);
}

/* FMT_BTXT record length */
static int
bhnd_nvram_txt_env_len(struct bhnd_nvram *sc, const char *env, size_t *env_len)
{
	const uint8_t	*p, *startp;
	size_t		 line_len;

	if ((const uint8_t *)env < sc->buf ||
	    (const uint8_t *)env > (sc->buf + sc->buf_size))
		return (EINVAL);

	/* Find record termination (EOL, or '#') */
	p = env;
	startp = env;
	while (p < sc->buf + sc->buf_size) {
		if (*p == '#' || *p == '\n' || *p == '\r')
			break;

		p++;
	}

	/* Got line length, now trim trailing whitespace to determine
	 * actual env length */
	line_len = p - startp;
	*env_len = line_len;

	for (size_t i = 0; i < line_len && line_len > 0; i++) {
		char c = startp[line_len - i - 1];
		if (!isspace(c))
			break;

		*env_len -= 1;
	}

	return (0);
}

/* FMT_BTXT record parsing */
static int
bhnd_nvram_txt_enum_buf(struct bhnd_nvram *sc, const char **env, size_t *len,
    const uint8_t *p, uint8_t const **next)
{
	int error;

	if (p == NULL)
		p = sc->buf;

	/* Skip any leading whitespace and comments */
	p = bhnd_nvram_txt_seek_nextline(sc, p);

	/* EOF? */
	if (!bhnd_nvram_bufptr_valid(sc, p, 1, false)) {
		*env = NULL;
		*len = 0;
		*next = p;
		return (0);
	}

	/* Determine the entry length */
	if ((error = bhnd_nvram_tlv_env_len(sc, p, len)))
		return (error);

	/* Advance to next entry */
	p += *len;
	p = bhnd_nvram_txt_seek_nextline(sc, p);
	
	*next = p;
	return (0);
}

#endif