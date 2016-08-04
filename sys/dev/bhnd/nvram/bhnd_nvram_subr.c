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

#include "bhnd_nvram_private.h"

#include "bhnd_nvramvar.h"
#include "bhnd_nvramreg.h"

/*
 * BHND NVRAM Parser
 * 
 * Provides identification, decoding, and encoding of BHND NVRAM data.
 */

typedef struct bhnd_nvram_ctx bhnd_nvram_ctx;

static int	bhnd_nvram_find_var(struct bhnd_nvram *nvram, const char *name,
		    const char **value, size_t *value_len);

static int	bhnd_nvram_keycmp(const char *lhs, size_t lhs_len,
		    const char *rhs, size_t rhs_len);
static int	bhnd_nvram_sort_idx(void *ctx, const void *lhs,
		    const void *rhs);
static int	bhnd_nvram_generate_index(struct bhnd_nvram *nvram);
static int	bhnd_nvram_index_lookup(struct bhnd_nvram *nvram,
		    const char *name, const char **env, size_t *len,
		    const char **value, size_t *value_len);

static bool	bhnd_nvram_bufptr_valid(struct bhnd_nvram *nvram,
		    const void *ptr, size_t nbytes, bool log_error);

static int	bhnd_nvram_parse_env(struct bhnd_nvram *nvram, const char *env,
		    size_t len, const char **key, size_t *key_len,
		    const char **val, size_t *val_len);

/* NVRAM format-specific operations */
typedef int	(*bhnd_nvram_op_init)(struct bhnd_nvram *nvram);
typedef int	(*bhnd_nvram_enum_buf)(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

/* FMT_BCM ops */
static int	bhnd_nvram_init_bcm(struct bhnd_nvram *nvram);
static int	bhnd_nvram_enum_buf_bcm(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

/* FMT_TLV ops */
static int	bhnd_nvram_init_tlv(struct bhnd_nvram *nvram);
static int	bhnd_nvram_enum_buf_tlv(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

/* FMT_TXT ops */
static int	bhnd_nvram_init_txt(struct bhnd_nvram *nvram);
static int	bhnd_nvram_enum_buf_txt(struct bhnd_nvram *nvram,
		    const char **env, size_t *len, const uint8_t *p,
		    uint8_t const **next);

struct bhnd_nvram_ops {
	bhnd_nvram_format	fmt;		/**< nvram format */
	bhnd_nvram_op_init	init;
	bhnd_nvram_enum_buf	enum_buf;
};

static const struct bhnd_nvram_ops bhnd_nvram_ops_table[] = {
	{ BHND_NVRAM_FMT_BCM, bhnd_nvram_init_bcm, bhnd_nvram_enum_buf_bcm },
	{ BHND_NVRAM_FMT_TLV, bhnd_nvram_init_tlv, bhnd_nvram_enum_buf_tlv },
	{ BHND_NVRAM_FMT_BTXT, bhnd_nvram_init_txt, bhnd_nvram_enum_buf_txt },
};


/* standard callback context */
struct bhnd_nvram_ctx {
	const uint8_t	*iobuf;
	size_t		 iolen;
	void		*iomisc;
	int		 ioerr;
};

#define	NVRAM_LOG(nvram, fmt, ...)	do {			\
	if (nvram->dev != NULL)					\
		device_printf(nvram->dev, fmt, ##__VA_ARGS__);	\
	else							\
		printf("bhnd_nvram: " fmt, ##__VA_ARGS__);	\
} while (0)

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
	bhnd_nvram_enum_buf	 enum_fn;
	const char		*env;
	const uint8_t		*p;
	size_t			 env_len;
	size_t			 name_len;
	int			 error;

	enum_fn = nvram->ops->enum_buf;
	name_len = strlen(name);

	// TODO: search dirty records

	/* Prefer the index */
	if (nvram->idx != NULL)
		return (bhnd_nvram_index_lookup(nvram, name, &env, &env_len,
		    value, value_len));

	/* Iterate over all records */
	p = NULL;
	while ((error = enum_fn(nvram, &env, &env_len, p, &p)) == 0) {
		/* Hit EOF, not found */
		if (env == NULL)
			break;

		/* Skip string comparison if env_len < strlen(key + '=') */
		if (env_len < name_len + 1)
			continue;

		/* Skip string comparison if delimiter isn't found at
		 * expected position */
		if (*(env + name_len) != '=')
			continue;

		/* Check for match */
		if (strncmp(env, name, name_len) == 0) {
			/* Found */
			*value = env + name_len + 1;
			*value_len = env_len - name_len - 1;
			return (0);
		};
	}

	/* Parse error */
	if (error)
		return (error);

	/* Not found */
	if (env == NULL)
		return (ENOENT);

	// TODO: fallback to defaults
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

/* bhnd_nvram_idx qsort_r sort function */
static int
bhnd_nvram_sort_idx(void *ctx, const void *lhs, const void *rhs)
{
	struct bhnd_nvram		*nvram;
	const struct bhnd_nvram_idx	*l_idx, *r_idx;
	const char			*l_str, *r_str;

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
	bhnd_nvram_enum_buf	 enum_fn;
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
		char	*eptr;
		char	 suffix[NVRAM_KEY_MAX+1];
		size_t	 suffix_len;
		u_long	 index;

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
			    key_len, key);
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
			    key_len, key);
			continue;
		}

		// TODO: register alias
		NVRAM_LOG(nvram, "got devpath %lu = '%.*s'\n", index, val_len,
		    val);
	}

	if (error)
		return (error);

	/* Save record count */
	nvram->num_buf_vars = num_records;

	/* Skip generating variable index if threshold is not met */
	if (nvram->num_buf_vars < BHND_NVRAM_IDX_VAR_THRESH)
		return (0);

	/* Allocate and populate variable index */
	idx_bytes = sizeof(nvram->idx[0]) * nvram->num_buf_vars;
	nvram->idx = malloc(idx_bytes, M_BHND_NVRAM, M_NOWAIT);
	if (nvram->idx == NULL) {
		NVRAM_LOG(nvram, "error allocating %zu byte index\n",
		    idx_bytes);
		goto bad_index;
	}

	if (bootverbose) {
		NVRAM_LOG(nvram, "allocated %zu byte index for %zu variables "
		    "in %zu bytes\n", idx_bytes, nvram->num_buf_vars,
		    nvram->buf_size);
	}

	p = NULL;
	for (size_t i = 0; i < num_records; i++) {
		struct bhnd_nvram_idx	*idx;
		size_t			 env_offset;
		size_t			 key_len, val_len;

		/* Fetch next record */
		if ((error = enum_fn(nvram, &env, &env_len, p, &p)))
			return (error);

		/* Early EOF */
		if (env == NULL) {
			NVRAM_LOG(nvram, "indexing failed, expected "
			    "%zu records (got %zu)\n", num_records, i+1);
			goto bad_index;
		}
	
		/* Calculate env offset */
		env_offset = (const uint8_t *)env - (const uint8_t *)nvram->buf;
		if (env_offset > BHND_NVRAM_IDX_OFFSET_MAX) {
			NVRAM_LOG(nvram, "'%.*s' offset %#zx exceeds maximum "
			    "indexable value\n", env_len, env, env_offset);
			goto bad_index;
		}

		/* Split key and value */
		error = bhnd_nvram_parse_env(nvram, env, env_len, &key,
		    &key_len, &val, &val_len);
		if (error)
			return (error);

		if (key_len > BHND_NVRAM_IDX_LEN_MAX) {
			NVRAM_LOG(nvram, "key length %#zx at %#zx exceeds "
			"maximum indexable value\n", key_len, env_offset);
			goto bad_index;
		}

		if (val_len > BHND_NVRAM_IDX_LEN_MAX) {
			NVRAM_LOG(nvram, "value length %#zx for key '%.*s' "
			    "exceeds maximum indexable value\n", val_len,
			    key_len, key);
			goto bad_index;
		}

		idx = &nvram->idx[i];
		idx->env_offset = env_offset;
		idx->key_len = key_len;
		idx->val_len = val_len;
	}

	/* Sort the index table */
	qsort_r(nvram->idx, nvram->num_buf_vars, sizeof(nvram->idx[0]), nvram,
	    bhnd_nvram_sort_idx);

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
 * Identify the NVRAM format at @p offset within @p r, verify the CRC (if applicable),
 * and allocate a local shadow copy of the NVRAM data.
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

	/* Check for specified data format */
	if (size < sizeof(union bhnd_nvram_ident))
		return (EINVAL);

	error = bhnd_nvram_identify(
	    (const union bhnd_nvram_ident *)data, fmt);
	if (error)
		return (error);

	/* Allocate backing buffer */
	nvram->buf_size = size;
	nvram->buf = malloc(nvram->buf_size, M_BHND_NVRAM, M_NOWAIT);
	if (nvram->buf == NULL)
		return (ENOMEM);
	memcpy(nvram->buf, data, nvram->buf_size);

	/* Fetch format-specific operation callbacks */
	for (size_t i = 0; i < nitems(bhnd_nvram_ops_table); i++) {
		const struct bhnd_nvram_ops *ops = &bhnd_nvram_ops_table[i];

		if (ops->fmt != fmt)
			continue;

		/* found */
		nvram->ops = ops;
		break;
	}

	if (nvram->ops == NULL)
		goto cleanup;

	/* Perform format-specific initialization */
	if ((error = nvram->ops->init(nvram)))
		goto cleanup;

	/* Generate all indices */
	if ((error = bhnd_nvram_generate_index(nvram)))
		goto cleanup;

	// TODO
	const char *val;
	size_t val_len;
	if ((error = bhnd_nvram_find_var(nvram, "boardtype", &val, &val_len)))
		return (error);
	NVRAM_LOG(nvram, "boardtype='%.*s'\n", val_len, val);

	return (0);

cleanup:
	free(nvram->buf, M_BHND_NVRAM);
	return (error);
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

	if (p >= nvram->buf + (nvram->buf_size - nbytes))
		goto failed;

	return (true);
	
failed:
	if (log_error)
		NVRAM_LOG(nvram, "NVRAM record not readable at %p+%#zx "
		    "(base=%p, len=%zu)\n", p, nbytes, nvram->buf,
		    nvram->buf_size);
	return (false);
}


static int
bhnd_nvram_init_bcm(struct bhnd_nvram *nvram)
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
 * Parse a 'key=value' env string.
 */
static int
bhnd_nvram_parse_env(struct bhnd_nvram *nvram, const char *env, size_t len,
    const char **key, size_t *key_len, const char **val, size_t *val_len)
{
	const char	*p;

	/* Key */
	if ((p = memchr(env, '=', len)) == NULL) {
		NVRAM_LOG(nvram, "missing delim in '%.*s'\n", len, env);
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
 * Perform an index lookup of @p name.
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
bhnd_nvram_index_lookup(struct bhnd_nvram *nvram, const char *name,
    const char **env, size_t *env_len, const char **value, size_t *value_len)
{
	struct bhnd_nvram_idx	*idx;
	const char		*idx_key;
	size_t			 min, mid, max;
	size_t			 name_len;
	int			 order;

	if (nvram->idx == NULL)
		return (ENODEV);

	if (nvram->num_buf_vars == 0)
		return (ENOENT);

	/*
	 * Locate the requested variable using a binary search.
	 */
	min = 0;
	mid = 0;
	max = nvram->num_buf_vars - 1;
	name_len = strlen(name);

	while (max >= min) {
		/* Select midpoint */
		mid = (min + max) / 2;
		idx = &nvram->idx[mid];

		/* Determine which side of the partition to search */
		idx_key = (const char *) (nvram->buf + idx->env_offset);
		order = bhnd_nvram_keycmp(idx_key, idx->key_len, name,
		    name_len);

		if (order < 0) {
			/* Search upper partition */
			min = mid + 1;
		} else if (order > 0) {
			/* Search lower partition */
			max = mid - 1;
		} else if (order == 0) {
			/* Match found */
			*env = nvram->buf + idx->env_offset;
			*env_len = idx->key_len + idx->val_len + 1 /* '=' */;

			*value = *env + idx->key_len + 1 /* '=' */;
			*value_len = idx->val_len;

			return (0);
		}
	}

	/* Not found */
	return (ENOENT);
}

static int
bhnd_nvram_enum_buf_bcm(struct bhnd_nvram *nvram, const char **env,
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

static int
bhnd_nvram_init_tlv(struct bhnd_nvram *nvram)
{
	// TODO
	return (0);
}

static int
bhnd_nvram_enum_buf_tlv(struct bhnd_nvram *nvram, const char **env,
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

static int
bhnd_nvram_init_txt(struct bhnd_nvram *nvram)
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

static int
bhnd_nvram_enum_buf_txt(struct bhnd_nvram *nvram, const char **env,
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

/**
 * Release all resources held by @p nvram.
 * 
 * @param nvram A NVRAM instance previously initialized via bhnd_nvram_init().
 */
void
bhnd_nvram_fini(struct bhnd_nvram *nvram)
{
	if (nvram->idx != NULL)
		free(nvram->idx, M_BHND_NVRAM);
	free(nvram->buf, M_BHND_NVRAM);
}
