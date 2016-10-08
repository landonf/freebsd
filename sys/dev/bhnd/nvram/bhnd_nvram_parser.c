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

#include <sys/queue.h>

#ifdef _KERNEL

#include <sys/param.h>
#include <sys/systm.h>

#else /* !_KERNEL */

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#endif /* _KERNEL */

#include "bhnd_nvramvar.h"

#include "bhnd_nvram_data.h"
#include "bhnd_nvram_datavar.h"

#include "bhnd_nvram_parserreg.h"
#include "bhnd_nvram_parservar.h"

/*
 * BHND NVRAM Parser
 * 
 * Provides identification, decoding, and encoding of BHND NVRAM data.
 */

static int	bhnd_nvram_parser_init_common(struct bhnd_nvram *sc,
		    struct bhnd_nvram_io *io, bhnd_nvram_data_class_t *cls);
static int	 bhnd_nvram_sort_idx(void *ctx, const void *lhs,
		     const void *rhs);
static int	 bhnd_nvram_generate_index(struct bhnd_nvram *sc);
static void	*bhnd_nvram_index_lookup(struct bhnd_nvram *sc,
		     const char *name);

#define	NVRAM_LOG(sc, fmt, ...)	\
	BHND_NV_DEVLOG(sc->owner, fmt, ##__VA_ARGS__)


#ifdef _KERNEL

/**
 * Identify the NVRAM format at @p offset within @p io, verify the
 * CRC (if applicable), and allocate a local shadow copy of the NVRAM data.
 * 
 * After initialization, no reference to @p io will be held by the
 * NVRAM parser, and @p io may be safely deallocated.
 * 
 * @param[out] sc The NVRAM parser state to be initialized.
 * @param owner The parser's parent device, or NULL if none.
 * @param io An I/O context mapping the NVRAM data to be parsed.
 * @param cls An NVRAM data class capable of parsing @p io.
 * 
 * @retval 0 success
 * @retval ENOMEM If internal allocation of NVRAM state fails.
 * @retval EINVAL If @p io parsing fails.
 */
int
bhnd_nvram_parser_init(struct bhnd_nvram *sc, device_t owner,
    struct bhnd_nvram_io *io, bhnd_nvram_data_class_t *cls)
{
	sc->owner = owner;
	return (bhnd_nvram_parser_init_common(sc, io, cls));
}

#else /* !_KERNEL */

/**
 * Identify the NVRAM format at @p offset within @p io, verify the
 * CRC (if applicable), and allocate a local shadow copy of the NVRAM data.
 * 
 * After initialization, no reference to @p io will be held by the
 * NVRAM parser, and @p io may be safely deallocated.
 * 
 * @param[out] sc The NVRAM parser state to be initialized.
 * @param io An I/O context mapping the NVRAM data to be parsed.
 * @param cls An NVRAM data class capable of parsing @p io.
 * 
 * @retval 0 success
 * @retval ENOMEM If internal allocation of NVRAM state fails.
 * @retval EINVAL If @p io parsing fails.
 */
int
bhnd_nvram_parser_init(struct bhnd_nvram *sc, struct bhnd_nvram_io *io,
    bhnd_nvram_data_class_t *cls)
{
	return (bhnd_nvram_parser_init_common(sc, io, cls));
}

#endif /* _KERNEL */

	
/*
 * Initialization common to kernel and non-kernel bhnd_nvram_parser_init()
 * implementations
 */
static int
bhnd_nvram_parser_init_common(struct bhnd_nvram *sc, struct bhnd_nvram_io *io,
    bhnd_nvram_data_class_t *cls)
{
	int error;

	/* Initialize NVRAM state */
	sc->idx = NULL;
	sc->nv = NULL;
	sc->pending = NULL;
	LIST_INIT(&sc->devpaths);

	/* Parse the input data */
	if ((error = bhnd_nvram_data_new(cls, &sc->nv, io)))
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
		bhnd_nv_free(dpath->path);
                bhnd_nv_free(dpath);
        }

	if (sc->pending != NULL)
		nvlist_destroy(sc->pending);

	if (sc->idx != NULL)
		bhnd_nv_free(sc->idx);

	if (sc->nv != NULL)
		bhnd_nvram_data_free(sc->nv);
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
		BHND_NV_PANIC("invalid value type for pending change %s", name);
	}

	/* Fetch variable from parsed NVRAM data. */
	if ((cookiep = bhnd_nvram_index_lookup(sc, name)) == NULL)
		return (ENOENT);

	/* Let the parser itself perform value coercion */
	return (bhnd_nvram_data_getvar(sc->nv, cookiep, buf, len, type));
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
	l_str = bhnd_nvram_data_getvar_name(sc->nv, l_idx->cookiep);
	r_str = bhnd_nvram_data_getvar_name(sc->nv, r_idx->cookiep);

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
	while ((name = bhnd_nvram_data_next(sc->nv, &cookiep))) {
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
		path = bhnd_nvram_data_getvar_ptr(sc->nv, cookiep, &path_len,
		    &path_type);
		if (path == NULL || path_type != BHND_NVRAM_TYPE_CSTR) {
			path_len = sizeof(pathbuf);
			error = bhnd_nvram_data_getvar(sc->nv, cookiep,
			    &pathbuf, &path_len, BHND_NVRAM_TYPE_CSTR);
			if (error)
				return (error);
		}

		/* Register path alias */
		devpath = bhnd_nv_malloc(sizeof(*devpath));
		if (devpath == NULL)
			return (ENOMEM);

		devpath->index = index;
		devpath->path = bhnd_nv_strndup(path, path_len);
		LIST_INSERT_HEAD(&sc->devpaths, devpath, dp_link);
	}

	/* Skip generating a variable index if threshold is not met ... */
	if (num_records < NVRAM_IDX_VAR_THRESH)
		return (0);

	/* ... or if the backing data instance implements indexed lookup
	 * internally */
	if (bhnd_nvram_data_getcaps(sc->nv) & BHND_NVRAM_DATA_CAP_INDEXED)
		return (0);

	/* Allocate and populate variable index */
	idx_bytes = sizeof(struct bhnd_nvram_idx) +
	    (sizeof(struct bhnd_nvram_idx_entry) * num_records);
	sc->idx = bhnd_nv_malloc(idx_bytes);
	if (sc->idx == NULL) {
		NVRAM_LOG(sc, "error allocating %zu byte index\n", idx_bytes);
		goto bad_index;
	}

	sc->idx->num_entries = num_records;

#ifdef _KERNEL
	if (bootverbose) {
		NVRAM_LOG(sc, "allocated %zu byte index for %zu variables\n",
		    idx_bytes, num_records);
	}
#endif /* _KERNEL */

	cookiep = NULL;
	for (size_t i = 0; i < sc->idx->num_entries; i++) {
		struct bhnd_nvram_idx_entry *idx;

		/* Fetch next entry */
		name = bhnd_nvram_data_next(sc->nv, &cookiep);

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
		bhnd_nv_free(sc->idx);
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
		return (bhnd_nvram_data_find(sc->nv, name));

	/*
	 * Locate the requested variable using a binary search.
	 */
	BHND_NV_ASSERT(sc->idx->num_entries > 0,
	    ("empty array causes underflow"));
	min = 0;
	max = sc->idx->num_entries - 1;

	while (max >= min) {
		/* Select midpoint */
		mid = (min + max) / 2;
		idxe = &sc->idx->entries[mid];

		/* Determine which side of the partition to search */
		idxe_key = bhnd_nvram_data_getvar_name(sc->nv, idxe->cookiep);
		order = strcasecmp(idxe_key, name);

		if (order < 0) {
			/* Search upper partition */
			min = mid + 1;
		} else if (order > 0) {
			/* Search (non-empty) lower partition */
			if (mid == 0)
				break;
			max = mid - 1;
		} else if (order == 0) {
			/* Match found */
			return (idxe->cookiep);
		}
	}

	/* Not found */
	return (NULL);
}
