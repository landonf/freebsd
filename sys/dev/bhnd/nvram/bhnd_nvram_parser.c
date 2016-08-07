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
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "bhnd_nvram_parservar.h"

/*
 * BHND NVRAM Parser
 * 
 * Provides identification, decoding, and encoding of BHND NVRAM data.
 */

static const struct bhnd_nvram_ops *bhnd_nvram_find_ops(bhnd_nvram_format fmt);

static int	bhnd_nvram_contains_var(struct bhnd_nvram *nvram,
		    const char *name);
static int	bhnd_nvram_find_var(struct bhnd_nvram *nvram, const char *name,
		    const char **value, size_t *value_len);

static struct bhnd_nvram_tuple	*bhnd_nvram_find_tuple(
				    struct bhnd_nvram_tuples *tuples,
				    const char *name, size_t name_len);

static int	bhnd_nvram_add_tuple(struct bhnd_nvram_tuples *tuples,
		    const char *name, const char *value);
static void	bhnd_nvram_tuple_free(struct bhnd_nvram_tuple *tuple);

static int	bhnd_nvram_keycmp(const char *lhs, size_t lhs_len,
		    const char *rhs, size_t rhs_len);
static int	bhnd_nvram_sort_idx(void *ctx, const void *lhs,
		    const void *rhs);
static int	bhnd_nvram_generate_index(struct bhnd_nvram *nvram);

static int	bhnd_nvram_index_lookup(struct bhnd_nvram *nvram,
		    struct bhnd_nvram_idx *idx, const char *name,
		    const char **env, size_t *len, const char **value,
		    size_t *value_len);
static int	bhnd_nvram_buffer_lookup(struct bhnd_nvram *nvram,
		    const char *name, const char **env, size_t *env_len,
		    const char **value, size_t *value_len);

static bool	bhnd_nvram_bufptr_valid(struct bhnd_nvram *nvram,
		    const void *ptr, size_t nbytes, bool log_error);

static int	bhnd_nvram_parse_env(struct bhnd_nvram *nvram, const char *env,
		    size_t len, const char **key, size_t *key_len,
		    const char **val, size_t *val_len);

/**
 * Calculate the size of the NVRAM data in @p data.
 * 
 * @param		data	Pointer to NVRAM data to be parsed.
 * @param[in,out]	size	On input, the total size of @p data. On
 *				successful parsing of @p data, will be set to
 *				the parsed size (which may be larger).
 */
typedef int	(*bhnd_nvram_op_getsize)(const void *data, size_t *size);

/** Perform format-specific initialization. */
typedef int	(*bhnd_nvram_op_init)(struct bhnd_nvram *nvram);

/** Initialize any format-specific default values. */
typedef int	(*bhnd_nvram_op_init_defaults)(struct bhnd_nvram *nvram);
typedef int	(*bhnd_nvram_op_enum_buf)(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

/* FMT_BCM ops */
static int	bhnd_nvram_bcm_getsize(const void *data, size_t *size);
static int	bhnd_nvram_bcm_init(struct bhnd_nvram *nvram);
static int	bhnd_nvram_bcm_init_defaults(struct bhnd_nvram *nvram);
static int	bhnd_nvram_bcm_enum_buf(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

/* FMT_TLV ops */
static int	bhnd_nvram_tlv_getsize(const void *data, size_t *size);
static int	bhnd_nvram_tlv_init(struct bhnd_nvram *nvram);
static int	bhnd_nvram_tlv_enum_buf(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

/* FMT_TXT ops */
static int	bhnd_nvram_txt_getsize(const void *data, size_t *size);
static int	bhnd_nvram_txt_init(struct bhnd_nvram *nvram);
static int	bhnd_nvram_txt_enum_buf(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

/**
 * Format-specific operations.
 */
struct bhnd_nvram_ops {
	bhnd_nvram_format		 fmt;		/**< nvram format */
	bhnd_nvram_op_getsize		 getsize;	/**< determine actual NVRAM size */
	bhnd_nvram_op_init		 init;		/**< format-specific initialization */
	bhnd_nvram_op_enum_buf		 enum_buf;	/**< enumerate backing buffer */
	bhnd_nvram_op_init_defaults	 init_defaults;	/**< populate any default values */
};

static const struct bhnd_nvram_ops bhnd_nvram_ops_table[] = {
	{ 
		BHND_NVRAM_FMT_BCM,
		bhnd_nvram_bcm_getsize,
		bhnd_nvram_bcm_init, 
		bhnd_nvram_bcm_enum_buf,
		bhnd_nvram_bcm_init_defaults
	},
	{
		BHND_NVRAM_FMT_TLV,
		bhnd_nvram_tlv_getsize,
		bhnd_nvram_tlv_init,
		bhnd_nvram_tlv_enum_buf,
		NULL
	},
	{
		BHND_NVRAM_FMT_BTXT,
		bhnd_nvram_txt_getsize,
		bhnd_nvram_txt_init,
		bhnd_nvram_txt_enum_buf,
		NULL
	},
};

#define	NVRAM_LOG(nvram, fmt, ...)	do {			\
	if (nvram->dev != NULL)					\
		device_printf(nvram->dev, fmt, ##__VA_ARGS__);	\
	else							\
		printf("bhnd_nvram: " fmt, ##__VA_ARGS__);	\
} while (0)

/* Limit a size_t value to a suitable range for use as a printf string field
 * width */
#define	NVRAM_PRINT_WIDTH(_len)	\
	((_len) > NVRAM_VAL_MAX ? NVRAM_VAL_MAX : (int)(_len))

/**
 * Identify @p ident.
 * 
 * @param ident Initial header data to be used for identification.
 * @param expected Expected format against which @p ident will be tested.
 * 
 * @retval 0 If @p ident has the @p expected format.
 * @retval ENODEV If @p ident does not match @p expected.
 */
int
bhnd_nvram_identify(const union bhnd_nvram_ident *ident,
    bhnd_nvram_format expected)
{
	uint32_t bcm_magic = le32toh(ident->bcm.magic);

	switch (expected) {
	case BHND_NVRAM_FMT_BCM:
		if (bcm_magic == NVRAM_MAGIC)
			return (0);

		return (ENODEV);
	case BHND_NVRAM_FMT_TLV:
		if (bcm_magic == NVRAM_MAGIC)
			return (ENODEV);

		if (ident->tlv.tag != NVRAM_TLV_TYPE_ENV)
			return (ENODEV);

		return (0);
	case BHND_NVRAM_FMT_BTXT:
		for (size_t i = 0; i < nitems(ident->btxt); i++) {
			char c = ident->btxt[i];
			if (!isprint(c) && !isspace(c))
				return (ENODEV);
		}
		return (0);
		break;
	default:
		printf("%s: unknown format: %d\n", __FUNCTION__, expected);
		return (ENODEV);
	}
}

/** Return the set of operations for @p fmt, if any */
static const struct bhnd_nvram_ops *
bhnd_nvram_find_ops(bhnd_nvram_format fmt)
{
	const struct bhnd_nvram_ops *ops;

	/* Fetch format-specific operation callbacks */
	for (size_t i = 0; i < nitems(bhnd_nvram_ops_table); i++) {
		ops = &bhnd_nvram_ops_table[i];

		if (ops->fmt != fmt)
			continue;

		/* found */
		return (ops);
	}

	return (NULL);
}

/**
 * Identify the NVRAM format at @p offset within @p r, verify the
 * CRC (if applicable), and allocate a local shadow copy of the NVRAM data.
 * 
 * After initialization, no reference to @p input will be held by the
 * NVRAM parser, and @p input may be safely deallocated.
 * 
 * @param[out] nvram On success, will be initialized with shadow of the NVRAM
 * data.
 * @param dev The parser's parent device, or NULL if none.
 * @param data NVRAM data to be parsed.
 * @param size Size of @p data.
 * @param fmt Required format of @p input.
 * 
 * @retval 0 success
 * @retval ENOMEM If internal allocation of NVRAM state fails.
 * @retval EINVAL If @p input parsing fails.
 */
int
bhnd_nvram_init(struct bhnd_nvram *nvram, device_t dev, const void *data,
    size_t size, bhnd_nvram_format fmt)
{
	int error;

	/* Initialize NVRAM state */
	memset(nvram, 0, sizeof(*nvram));

	nvram->dev = dev;
	STAILQ_INIT(&nvram->devpaths);
	STAILQ_INIT(&nvram->defaults);
	STAILQ_INIT(&nvram->pending);

	/* Verify data format and init operation callbacks */
	if (size < sizeof(union bhnd_nvram_ident))
		return (EINVAL);

	error = bhnd_nvram_identify(
	    (const union bhnd_nvram_ident *)data, fmt);
	if (error)
		return (error);

	if ((nvram->ops = bhnd_nvram_find_ops(fmt)) == NULL) {
		NVRAM_LOG(nvram, "unsupported format: %d\n", fmt);
		return (error);
	}

	/* Determine appropiate size for backing buffer */
	nvram->buf_size = size;
	if ((error = nvram->ops->getsize(data, &nvram->buf_size)))
		return (error);

	if (nvram->buf_size > size) {
		NVRAM_LOG(nvram, "cannot parse %zu NVRAM bytes, would overrun "
		    "%zu byte input buffer\n", nvram->buf_size, size);
		return (EINVAL);
	}

	/* Allocate and populate backing buffer */
	nvram->buf = malloc(nvram->buf_size, M_BHND_NVRAM, M_NOWAIT);
	if (nvram->buf == NULL)
		return (ENOMEM);
	memcpy(nvram->buf, data, nvram->buf_size);

	/* Perform format-specific initialization */
	if ((error = nvram->ops->init(nvram)))
		goto cleanup;

	/* Generate all indices */
	if ((error = bhnd_nvram_generate_index(nvram)))
		goto cleanup;

	/* Add any format-specific default values */
	if (nvram->ops->init_defaults != NULL) {
		if ((error = nvram->ops->init_defaults(nvram)))
			goto cleanup;
	}

	// TODO
	const char *val;
	size_t val_len;
	if ((error = bhnd_nvram_find_var(nvram, "boardtype", &val, &val_len)))
		goto cleanup;
	NVRAM_LOG(nvram, "boardtype='%.*s'\n",
	    NVRAM_PRINT_WIDTH(val_len), val);

	struct bhnd_nvram_devpath *dp;
	STAILQ_FOREACH(dp, &nvram->devpaths, dp_link) {
		NVRAM_LOG(nvram, "alias %lu to '%s'\n", dp->index, dp->path);
	}
	
	struct bhnd_nvram_tuple *t;
	STAILQ_FOREACH(t, &nvram->defaults, t_link) {
		NVRAM_LOG(nvram, "default %s='%s'\n", t->name, t->value);
	}

	return (0);

cleanup:
	bhnd_nvram_fini(nvram);
	return (error);
}


/**
 * Release all resources held by @p nvram.
 * 
 * @param nvram A NVRAM instance previously initialized via bhnd_nvram_init().
 */
void
bhnd_nvram_fini(struct bhnd_nvram *nvram)
{
	struct bhnd_nvram_devpath	*dpath, *dnext;
	struct bhnd_nvram_tuple		*tuple, *tnext;

        STAILQ_FOREACH_SAFE(dpath, &nvram->devpaths, dp_link, dnext) {
		free(dpath->path, M_BHND_NVRAM);
                free(dpath, M_BHND_NVRAM);
        }

	STAILQ_FOREACH_SAFE(tuple, &nvram->defaults, t_link, tnext)
		bhnd_nvram_tuple_free(tuple);

	STAILQ_FOREACH_SAFE(tuple, &nvram->pending, t_link, tnext)
		bhnd_nvram_tuple_free(tuple);

	if (nvram->idx != NULL)
		free(nvram->idx, M_BHND_NVRAM);

	if (nvram->buf != NULL)
		free(nvram->buf, M_BHND_NVRAM);

}

/**
 * Return true if @p ptr + nbytes falls within our backing buffer, false
 * otherwise.
 */
static bool
bhnd_nvram_bufptr_valid(struct bhnd_nvram *nvram, const void *ptr,
    size_t nbytes, bool log_error)
{
	const uint8_t *p = ptr;

	if (p < nvram->buf)
		goto failed;

	if (nbytes > nvram->buf_size)
		goto failed;

	if (p - nvram->buf > nvram->buf_size - nbytes)
		goto failed;

	return (true);
	
failed:
	if (log_error)
		NVRAM_LOG(nvram, "NVRAM record not readable at %p+%#zx "
		    "(base=%p, len=%zu)\n", p, nbytes, nvram->buf,
		    nvram->buf_size);
	return (false);
}

/**
 * Parse a 'key=value' env string.
 */
static int
bhnd_nvram_parse_env(struct bhnd_nvram *nvram, const char *env, size_t len,
    const char **key, size_t *key_len, const char **val, size_t *val_len)
{
	const char	*p;

	/* Key */
	if ((p = memchr(env, '=', len)) == NULL) {
		NVRAM_LOG(nvram, "missing delim in '%.*s'\n",
		    NVRAM_PRINT_WIDTH(len), env);
		return (EINVAL);
	}

	*key = env;
	*key_len = p - env;

	/* Skip '=' */
	p++;

	/* Vaue */
	*val = p;
	*val_len = len - (p - env);

	return (0);
}

/**
 * Test if @p name exists.
 * 
 * @param	nvram		The NVRAM parser state.
 * @param	name		The NVRAM variable name to search for.
 * 
 * @retval 0		success
 * @retval ENOENT	The requested variable was not found.
 * @retval non-zero	If reading @p name otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvram_contains_var(struct bhnd_nvram *nvram, const char *name)
{
	const char	*val;
	size_t		 val_len;

	return (bhnd_nvram_find_var(nvram, name, &val, &val_len));
}

/**
 * Fetch a string pointer to @p name's value, if any.
 * 
 * @param	nvram		The NVRAM parser state.
 * @param	name		The NVRAM variable name.
 * @param[out]	value		On success, a pointer to the variable's value
 *				string. The string may not be NUL terminated.
 * @param[out]	value_len	On success, the length of @p value, not
 *				including a terminating NUL (if any exists).
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable was not found.
 * @retval non-zero	If reading @p name otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvram_find_var(struct bhnd_nvram *nvram, const char *name,
    const char **value, size_t *value_len)
{
	struct bhnd_nvram_tuple	*t;
	bhnd_nvram_op_enum_buf	 enum_fn;
	const char		*env;
	size_t			 env_len;
	size_t			 name_len;
	int			 error;

	enum_fn = nvram->ops->enum_buf;
	name_len = strlen(name);

	/*
	 * Search path:
	 * 
	 * - uncommitted changes
	 * - index lookup OR buffer scan
	 * - registered defaults
	 */

	/* Search uncommitted changes */
	t = bhnd_nvram_find_tuple(&nvram->pending, name, name_len);
	if (t != NULL) {
		if (t->value != NULL) {
			/* Uncommited value exists, is not a deletion */
			*value = t->value;
			*value_len = t->value_len;
			return (0);
		} else {
			/* Value is marked for deletion. */
			error = ENOENT;
			goto failed;
		}
	}

	/* Search backing buffer. We the index if available; otherwise,
	 * perform a buffer scan */
	if (nvram->idx != NULL) {
		error = bhnd_nvram_index_lookup(nvram, nvram->idx, name, &env,
		    &env_len, value, value_len);
	} else {
		error = bhnd_nvram_buffer_lookup(nvram, name, &env, &env_len,
		    value, value_len);
	}

failed:
	/* If a parse error occured, we don't want to hide the issue by
	 * returning a default NVRAM value. Otherwise, look for a matching
	 * default. */
	if (error != ENOENT)
		return (error);

	t = bhnd_nvram_find_tuple(&nvram->defaults, name, name_len);
	if (t != NULL) {
		*value = t->value;
		*value_len = t->value_len;
		return (0);
	}

	/* Not found, and no default value available */
	return (ENOENT);
}

/*
 * An strcmp()-compatible  lexical comparison implementation that
 * handles non-NUL-terminated strings.
 */
static int
bhnd_nvram_keycmp(const char *lhs, size_t lhs_len, const char *rhs,
    size_t rhs_len)
{
	int order;

	order = strncmp(lhs, rhs, ulmin(lhs_len, rhs_len));
	if (order == 0) {
		if (lhs_len < rhs_len)
			order = -1;
		else if (lhs_len > rhs_len)
			order = 1;
	}

	return (order);
}

/* sort function for bhnd_nvram_idx_entry values */
static int
bhnd_nvram_sort_idx(void *ctx, const void *lhs, const void *rhs)
{
	struct bhnd_nvram			*nvram;
	const struct bhnd_nvram_idx_entry	*l_idx, *r_idx;
	const char				*l_str, *r_str;

	nvram = ctx;
	l_idx = lhs;
	r_idx = rhs;

	/* Fetch string pointers */
	l_str = (char *)(nvram->buf + l_idx->env_offset);
	r_str = (char *)(nvram->buf + r_idx->env_offset);

	/* Perform comparison */
	return (bhnd_nvram_keycmp(l_str, l_idx->key_len, r_str,
	    r_idx->key_len));
}


/**
 * Generate all indices for the NVRAM data backing @p nvram.
 * 
 * @param nvram		The NVRAM parser state.
 *
 * @retval 0		success
 * @retval non-zero	If indexing @p nvram fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvram_generate_index(struct bhnd_nvram *nvram)
{
	bhnd_nvram_op_enum_buf	 enum_fn;
	const char		*key, *val;
	const char		*env;
	const uint8_t		*p;
	size_t			 env_len;
	size_t			 idx_bytes;
	size_t			 key_len, val_len;
	size_t			 num_records;
	int			 error;

	enum_fn = nvram->ops->enum_buf;
	num_records = 0;

	/* Parse and register all device path aliases */
	p = NULL;
	while ((error = enum_fn(nvram, &env, &env_len, p, &p)) == 0) {
		struct bhnd_nvram_devpath	*devpath;
		char				*eptr;
		char				 suffix[NVRAM_KEY_MAX+1];
		size_t				 suffix_len;
		u_long				 index;

		/* Hit EOF */
		if (env == NULL)
			break;

		num_records++;

		/* Skip string comparison if env_len < strlen(devpath) */
		if (env_len < NVRAM_DEVPATH_LEN)
			continue;

		/* Check for devpath prefix */
		if (strncmp(env, NVRAM_DEVPATH_STR, NVRAM_DEVPATH_LEN) != 0)
			continue;

		/* Split key and value */
		error = bhnd_nvram_parse_env(nvram, env, env_len, &key,
		    &key_len, &val, &val_len);
		if (error)
			return (error);

		/* NUL terminate the devpath's suffix */
		if (key_len >= sizeof(suffix)) {
			NVRAM_LOG(nvram, "variable '%.*s' exceeds "
			    "NVRAM_KEY_MAX, skipping devpath parsing\n",
			    NVRAM_PRINT_WIDTH(key_len), key);
			continue;
		} else {
			suffix_len = key_len - NVRAM_DEVPATH_LEN;
			if (suffix_len == 0)
				continue;

			strcpy(suffix, key+NVRAM_DEVPATH_LEN);
			suffix[suffix_len] = '\0';
		}

		/* Parse the index value */
		index = strtoul(suffix, &eptr, 10);
		if (eptr == suffix || *eptr != '\0') {
			NVRAM_LOG(nvram, "invalid devpath variable '%.*s'\n",
			    NVRAM_PRINT_WIDTH(key_len), key);
			continue;
		}

		/* Register path alias */
		devpath = malloc(sizeof(*devpath), M_BHND_NVRAM, M_NOWAIT);
		if (devpath == NULL)
			return (ENOMEM);

		devpath->index = index;
		devpath->path = strndup(val, val_len, M_BHND_NVRAM);
		STAILQ_INSERT_TAIL(&nvram->devpaths, devpath, dp_link);
	}

	if (error)
		return (error);

	/* Save record count */
	nvram->num_buf_vars = num_records;

	/* Skip generating variable index if threshold is not met */
	if (nvram->num_buf_vars < NVRAM_IDX_VAR_THRESH)
		return (0);

	/* Allocate and populate variable index */
	idx_bytes = sizeof(struct bhnd_nvram_idx) +
	    (sizeof(struct bhnd_nvram_idx_entry) * nvram->num_buf_vars);
	nvram->idx = malloc(idx_bytes, M_BHND_NVRAM, M_NOWAIT);
	if (nvram->idx == NULL) {
		NVRAM_LOG(nvram, "error allocating %zu byte index\n",
		    idx_bytes);
		goto bad_index;
	}

	nvram->idx->num_entries = nvram->num_buf_vars;

	if (bootverbose) {
		NVRAM_LOG(nvram, "allocated %zu byte index for %zu variables "
		    "in %zu bytes\n", idx_bytes, nvram->num_buf_vars,
		    nvram->buf_size);
	}

	p = NULL;
	for (size_t i = 0; i < nvram->idx->num_entries; i++) {
		struct bhnd_nvram_idx_entry	*idx;
		size_t				 env_offset;
		size_t				 key_len, val_len;

		/* Fetch next record */
		if ((error = enum_fn(nvram, &env, &env_len, p, &p)))
			return (error);

		/* Early EOF */
		if (env == NULL) {
			NVRAM_LOG(nvram, "indexing failed, expected %zu records"
			" (got %zu)\n", nvram->idx->num_entries, i+1);
			goto bad_index;
		}
	
		/* Calculate env offset */
		env_offset = (const uint8_t *)env - (const uint8_t *)nvram->buf;
		if (env_offset > NVRAM_IDX_OFFSET_MAX) {
			NVRAM_LOG(nvram, "'%.*s' offset %#zx exceeds maximum "
			    "indexable value\n", NVRAM_PRINT_WIDTH(env_len),
			    env, env_offset);
			goto bad_index;
		}

		/* Split key and value */
		error = bhnd_nvram_parse_env(nvram, env, env_len, &key,
		    &key_len, &val, &val_len);
		if (error)
			return (error);

		if (key_len > NVRAM_IDX_LEN_MAX) {
			NVRAM_LOG(nvram, "key length %#zx at %#zx exceeds "
			"maximum indexable value\n", key_len, env_offset);
			goto bad_index;
		}

		if (val_len > NVRAM_IDX_LEN_MAX) {
			NVRAM_LOG(nvram, "value length %#zx for key '%.*s' "
			    "exceeds maximum indexable value\n", val_len,
			    NVRAM_PRINT_WIDTH(key_len), key);
			goto bad_index;
		}

		idx = &nvram->idx->entries[i];
		idx->env_offset = env_offset;
		idx->key_len = key_len;
		idx->val_len = val_len;
	}

	/* Sort the index table */
	qsort_r(nvram->idx->entries, nvram->idx->num_entries,
	    sizeof(nvram->idx->entries[0]), nvram, bhnd_nvram_sort_idx);

	return (0);

bad_index:
	/* Fall back on non-indexed access */
	NVRAM_LOG(nvram, "reverting to non-indexed variable lookup\n");
	if (nvram->idx != NULL) {
		free(nvram->idx, M_BHND_NVRAM);
		nvram->idx = NULL;
	}

	return (0);
}


/**
 * Perform an index lookup of @p name.
 *
 * @param	nvram		The NVRAM parser state.
 * @param	idx		The index to search.
 * @param	name		The variable to search for.
 * @param[out]	env		On success, the pointer to @p name within the
 *				backing buffer.
 * @param[out]	env_len		On success, the length of @p env.
 * @param[out]	value		On success, the pointer to @p name's value
 *				within the backing buffer.
 * @param[out]	value_len	On success, the length of @p value.
 * 
 * @retval 0 If @p name was found in the index.
 * @retval ENOENT If @p name was not found in the index.
 * @retval ENODEV If no index has been generated.
 */
static int
bhnd_nvram_index_lookup(struct bhnd_nvram *nvram, struct bhnd_nvram_idx *idx,
    const char *name, const char **env, size_t *env_len, const char **value,
    size_t *value_len)
{
	struct bhnd_nvram_idx_entry	*idxe;
	const char			*idxe_key;
	size_t				 min, mid, max;
	size_t				 name_len;
	int				 order;

	if (idx->num_entries == 0)
		return (ENOENT);

	/*
	 * Locate the requested variable using a binary search.
	 */
	min = 0;
	mid = 0;
	max = idx->num_entries - 1;
	name_len = strlen(name);

	while (max >= min) {
		/* Select midpoint */
		mid = (min + max) / 2;
		idxe = &idx->entries[mid];

		/* Determine which side of the partition to search */
		idxe_key = (const char *) (nvram->buf + idxe->env_offset);
		order = bhnd_nvram_keycmp(idxe_key, idxe->key_len, name,
		    name_len);

		if (order < 0) {
			/* Search upper partition */
			min = mid + 1;
		} else if (order > 0) {
			/* Search lower partition */
			max = mid - 1;
		} else if (order == 0) {
			/* Match found */
			*env = nvram->buf + idxe->env_offset;
			*env_len = idxe->key_len + idxe->val_len + 1 /* '=' */;

			*value = *env + idxe->key_len + 1 /* '=' */;
			*value_len = idxe->val_len;

			return (0);
		}
	}

	/* Not found */
	return (ENOENT);
}


/**
 * Perform a unindexed search for an entry matching @p name in the backing
 * NVRAM data buffer.
 *
 * @param	nvram		The NVRAM parser state.
 * @param	name		The variable to search for.
 * @param[out]	env		On success, the pointer to @p name within the
 *				backing buffer.
 * @param[out]	env_len		On success, the length of @p env.
 * @param[out]	value		On success, the pointer to @p name's value
 *				within the backing buffer.
 * @param[out]	value_len	On success, the length of @p value.
 * 
 * @retval 0 If @p name was found in the index.
 * @retval ENOENT If @p name was not found in the index.
 * @retval ENODEV If no index has been generated.
 */
static int
bhnd_nvram_buffer_lookup(struct bhnd_nvram *nvram, const char *name,
    const char **env, size_t *env_len, const char **value, size_t *value_len)
{
	bhnd_nvram_op_enum_buf	 enum_fn;
	const uint8_t		*p;
	size_t			 name_len;
	int			 error;

	enum_fn = nvram->ops->enum_buf;
	name_len = strlen(name);

	/* Iterate over all records in the backing buffer */
	p = NULL;
	while ((error = enum_fn(nvram, env, env_len, p, &p)) == 0) {
		/* Hit EOF, not found */
		if (*env == NULL)
			return (ENOENT);

		/* Skip string comparison if env_len < strlen('key=') */
		if (*env_len < name_len + 1)
			continue;

		/* Skip string comparison if delimiter isn't found at
		* expected position */
		if (*(*env + name_len) != '=')
			continue;

		/* Check for match */
		if (strncmp(*env, name, name_len) == 0) {
			/* Found */
			*value = *env + name_len + 1;
			*value_len = *env_len - name_len - 1;
			return (0);
		};
	}

	return (error);
}

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
bhnd_nvram_bcm_init(struct bhnd_nvram *nvram)
{
	const uint8_t	*p;
	uint32_t	 cfg0;
	uint8_t		 crc, valid;

	/* Validate CRC */
	if (nvram->buf_size < NVRAM_CRC_SKIP)
		return (EINVAL);

	if (nvram->buf_size < sizeof(struct bhnd_nvram_header))
		return (EINVAL);

	cfg0 = ((struct bhnd_nvram_header *)nvram->buf)->cfg0;
	valid = (cfg0 & NVRAM_CFG0_CRC_MASK) >> NVRAM_CFG0_CRC_SHIFT;

	p = nvram->buf;
	crc = bhnd_nvram_crc8(p + NVRAM_CRC_SKIP, nvram->buf_size-NVRAM_CRC_SKIP,
	    BHND_NVRAM_CRC8_INITIAL);

	if (crc != valid) {
		NVRAM_LOG(nvram, "warning: NVRAM CRC error (crc=%#hhx, "
		    "expected=%hhx)\n", crc, valid);
	}

	return (0);
}

/**
 * Release all resources held by @p tuple.
 */
static void
bhnd_nvram_tuple_free(struct bhnd_nvram_tuple *tuple)
{
	free(tuple->name, M_BHND_NVRAM);
	free(tuple->value, M_BHND_NVRAM);
	free(tuple, M_BHND_NVRAM);
}

/**
 * Find a tuple with @p name in @p tuples, if any.
 */
static struct bhnd_nvram_tuple *
bhnd_nvram_find_tuple(struct bhnd_nvram_tuples *tuples, const char *name,
    size_t name_len)
{
	struct bhnd_nvram_tuple *t;

	STAILQ_FOREACH(t, tuples, t_link) {
		if (t->name_len != name_len)
			continue;

		if (strncmp(t->name, name, name_len) == 0)
			return (t);
	}

	/* Not found */
	return (NULL);
}

/**
 * Add (or replace) a tuple with @p name and @p value in @p tuples.
 */
static int
bhnd_nvram_add_tuple(struct bhnd_nvram_tuples *tuples, const char *name,
    const char *value)
{
	struct bhnd_nvram_tuple	*t;
	size_t			 name_len;

	/* Fetch an existing tuple, or allocate a new one. */
	name_len = strlen(name);
	if ((t = bhnd_nvram_find_tuple(tuples, name, name_len)) != NULL) {
		STAILQ_REMOVE(tuples, t, bhnd_nvram_tuple, t_link);

		/* Drop value data */
		if (t->value != NULL)
			free(t->value, M_BHND_NVRAM);
		t->value_len = 0;
	} else {
		t = malloc(sizeof(*t), M_BHND_NVRAM, M_NOWAIT);
		if (t == NULL)
			return (ENOMEM);
		t->name = strdup(name, M_BHND_NVRAM);
		t->name_len = name_len;
	}

	if (value != NULL) {
		t->value = strdup(value, M_BHND_NVRAM);
		t->value_len = strlen(value);
	} else {
		t->value = NULL;
		t->value_len = 0;
	}

	/* Append to list */
	STAILQ_INSERT_TAIL(tuples, t, t_link);
	return (0);
}

/* Populate FMT_BCM-specific default values */
static int
bhnd_nvram_bcm_init_defaults(struct bhnd_nvram *nvram)
{
	struct bhnd_nvram_header	*header;
	char				 vbuf[NVRAM_VAL_MAX];
	int				 error;

	/* Verify that our header is readable */
	header = (struct bhnd_nvram_header *) nvram->buf;
	if (!bhnd_nvram_bufptr_valid(nvram, header, sizeof(*header), true))
		return (EINVAL);

	/* If the given header-shadowed variable does not in the NVRAM
	 * buffer, extract its value from the header, format it, and register
	 * a new default variable tuple */
#define	NVRAM_BCM_HEADER_DEFAULT(_field, _name)	do {	\
	error = bhnd_nvram_contains_var(nvram, _name ##_VAR);	\
	if (error == ENOENT) {					\
		uint32_t value;					\
		value = le32toh(header->_field);		\
		value = NVRAM_GET_BITS(value, _name);		\
								\
		snprintf(vbuf, sizeof(vbuf), _name ##_FMT,	\
		    value);					\
		error = bhnd_nvram_add_tuple(&nvram->defaults,	\
		    _name ##_VAR, vbuf);			\
	}							\
								\
	if (error) {						\
		return (error);					\
	}							\
} while(0)

	NVRAM_BCM_HEADER_DEFAULT(cfg0,		NVRAM_CFG0_SDRAM_INIT);
	NVRAM_BCM_HEADER_DEFAULT(cfg1,		NVRAM_CFG1_SDRAM_CFG);
	NVRAM_BCM_HEADER_DEFAULT(cfg1,		NVRAM_CFG1_SDRAM_REFRESH);
	NVRAM_BCM_HEADER_DEFAULT(sdram_ncdl,	NVRAM_SDRAM_NCDL);

#undef	NVRAM_BCM_HEADER_DEFAULT

	return (0);
}


/* FMT_BCM record parsing */
static int
bhnd_nvram_bcm_enum_buf(struct bhnd_nvram *nvram, const char **env,
    size_t *len, const uint8_t *p, uint8_t const **next)
{
	/* First record is found following the NVRAM header */
	if (p == NULL)
		p = nvram->buf + sizeof(struct bhnd_nvram_header);

	if (!bhnd_nvram_bufptr_valid(nvram, p, 1, true))
		return (EINVAL);

	/* EOF */
	if (*p == '\0') {
		*env = NULL;
		*len = 0;
		*next = p;
		return (0);
	}

	/* Provide pointer to env data */
	*env = p;
	*len = strnlen(p, nvram->buf_size - (p - nvram->buf));

	/* Advance to next entry and skip terminating NUL */
	p += *len;
	if (bhnd_nvram_bufptr_valid(nvram, p, 1, false)) {
		p++;
	} else {
		NVRAM_LOG(nvram, "warning: missing NVRAM termination record");
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
bhnd_nvram_tlv_init(struct bhnd_nvram *nvram)
{
	return (0);
}

/* FMT_TLV record parsing */
static int
bhnd_nvram_tlv_enum_buf(struct bhnd_nvram *nvram, const char **env,
    size_t *len, const uint8_t *p, uint8_t const **next)
{
	size_t		 rlen;
	uint8_t		 type;

	if (p == NULL)
		p = nvram->buf;

	/* Fetch type */
	if (!bhnd_nvram_bufptr_valid(nvram, p, 1, true))
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
		if (!bhnd_nvram_bufptr_valid(nvram, p, 1, true))
			return (EINVAL);
	
		rlen = *p;
		p += 1;
	} else {
		if (!bhnd_nvram_bufptr_valid(nvram, p, 2, true))
			return (EINVAL);
		rlen = (p[0] << 8) | (p[1]);
		p += 2;
	}

	/* Verify record readability */
	if (!bhnd_nvram_bufptr_valid(nvram, p, rlen, true))
		return (EINVAL);

	/* Error on non-env records */
	if (type != NVRAM_TLV_TYPE_ENV) {
		NVRAM_LOG(nvram, "unsupported NVRAM TLV tag: %#hhx\n",
		    type);
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
bhnd_nvram_txt_init(struct bhnd_nvram *nvram)
{
	return (0);
}

/* Seek past the next line ending (\r, \r\n, or \n) */
static const uint8_t *
bhnd_nvram_txt_seek_eol(struct bhnd_nvram *nvram, const uint8_t *p)
{
	while (p < nvram->buf + nvram->buf_size) {
		switch (*p) {
		case '\r':
			/* \r\n */
			if (bhnd_nvram_bufptr_valid(nvram, p, 1, false)) {
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
bhnd_nvram_txt_seek_nextline(struct bhnd_nvram *nvram, const uint8_t *p)
{
	/* Skip leading whitespace and comments */
	while (p < nvram->buf + nvram->buf_size) {
		if (isspace(*p)) {
			p++;
			continue;
		}
		
		if (*p == '#') {
			p = bhnd_nvram_txt_seek_eol(nvram, p);
			continue;
		}
		
		break;
	}

	return (p);
}

/* FMT_BTXT record parsing */
static int
bhnd_nvram_txt_enum_buf(struct bhnd_nvram *nvram, const char **env,
    size_t *len, const uint8_t *p, uint8_t const **next)
{
	const uint8_t	*startp;
	size_t		 line_len;

	if (p == NULL)
		p = nvram->buf;

	/* Skip any leading whitespace and comments */
	p = bhnd_nvram_txt_seek_nextline(nvram, p);

	/* EOF? */
	if (!bhnd_nvram_bufptr_valid(nvram, p, 1, false)) {
		*env = NULL;
		*len = 0;
		*next = p;
		return (0);
	}

	/* Find record termination (EOL, or '#') */
	startp = p;
	while (p < nvram->buf + nvram->buf_size) {
		if (*p == '#' || *p == '\n' || *p == '\r')
			break;

		p++;
	}

	/* Calculate line length, check for EOF */
	line_len = p - startp;
	if (!bhnd_nvram_bufptr_valid(nvram, p, 1, false)) {
		*env = NULL;
		*len = 0;
		*next = p;
		return (0);
	}

	/* Got env data; trim any tailing whitespace */
	*env = startp;
	*len = line_len;

	for (size_t i = 0; i < line_len && line_len > 0; i++) {
		char c = startp[line_len - i - 1];
		if (!isspace(c))
			break;

		*len -= 1;
	}

	/* Advance to next entry */
	p = bhnd_nvram_txt_seek_nextline(nvram, p);
	
	*next = p;
	return (0);
}
