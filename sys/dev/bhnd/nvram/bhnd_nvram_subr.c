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

static bool	bhnd_nvram_bufptr_valid(struct bhnd_nvram *nvram,
		    const void *ptr, size_t nbytes);

#if 0
static int	bhnd_nvram_parse_env(struct bhnd_nvram *nvram, const char *env,
		    size_t len, const char **key, size_t *key_len,
		    const char **val, size_t *val_len);
#endif

/* NVRAM format-specific operations */
typedef int	(*bhnd_nvram_op_init)(struct bhnd_nvram *nvram);
typedef int	(*bhnd_nvram_enum_buf_cb)(struct bhnd_nvram *nvram,
		    const char *env, size_t len, bool *stop,
		    bhnd_nvram_ctx *ctx);
typedef int	(*bhnd_nvram_enum_buf)(struct bhnd_nvram *nvram,
		    bhnd_nvram_enum_buf_cb cb, bhnd_nvram_ctx *ctx);

/* FMT_BCM ops */
static int	bhnd_nvram_init_bcm(struct bhnd_nvram *nvram);
static int	bhnd_nvram_enum_buf_bcm(struct bhnd_nvram *nvram,
		    bhnd_nvram_enum_buf_cb cb, bhnd_nvram_ctx *ctx);

/* FMT_TLV ops */
static int	bhnd_nvram_init_tlv(struct bhnd_nvram *nvram);
static int	bhnd_nvram_enum_buf_tlv(struct bhnd_nvram *nvram,
		    bhnd_nvram_enum_buf_cb cb, bhnd_nvram_ctx *ctx);

struct bhnd_nvram_ops {
	bhnd_nvram_format	fmt;		/**< nvram format */
	bhnd_nvram_op_init	init;
	bhnd_nvram_enum_buf	enum_buf;
};

static const struct bhnd_nvram_ops bhnd_nvram_ops_table[] = {
	{ BHND_NVRAM_FMT_BCM, bhnd_nvram_init_bcm, bhnd_nvram_enum_buf_bcm },
	{ BHND_NVRAM_FMT_TLV, bhnd_nvram_init_tlv, bhnd_nvram_enum_buf_tlv },
};


/* standard callback context */
struct bhnd_nvram_ctx {
	const uint8_t	*iobuf;
	size_t		 iolen;
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
	switch (expected) {
	case BHND_NVRAM_FMT_BCM:
		if (le32toh(ident->bcm.magic) == NVRAM_MAGIC)
			return (0);

		return (ENODEV);
	case BHND_NVRAM_FMT_TLV:
		if (ident->bcm.magic == NVRAM_MAGIC)
			return (ENODEV);

		if (ident->tlv.tag != NVRAM_TLV_TYPE_ENV)
			return (ENODEV);

		return (0);
	default:
		printf("%s: unknown format: %d\n", __FUNCTION__, expected);
		return (ENODEV);
	}
}

static int
bhnd_nvram_find_bufvar_cb(struct bhnd_nvram *nvram, const char *env, size_t len,
    bool *stop, bhnd_nvram_ctx *ctx)
{
	const char	*key;
	size_t		 key_len;

	key = ctx->iobuf;
	key_len = ctx->iolen;

	// TODO: devpath aliases

	/* Skip string comparison if len < strlen(key + '=') */
	if (len < key_len + 1)
		return (0);

	/* Skip string comparison if delimiter isn't found at
	 * expected position */
	if (*(env + key_len) != '=')
		return (0);

	/* Check for match */
	if (strncmp(env, key, key_len) == 0) {
		/* Return matching value */
		ctx->ioerr = 0;
		ctx->iobuf = env + key_len + 1;
		ctx->iolen = len - key_len - 1;

		*stop = true;
		return (0);
	};

	/* No match; continue enumeration */
	return (0);
}

static int
bhnd_nvram_find_var(struct bhnd_nvram *nvram, const char *name,
    const char **value, size_t *len)
{
	bhnd_nvram_ctx	ctx;
	int		error;

	ctx.iobuf = name;
	ctx.iolen = strlen(name);
	ctx.ioerr = ENOENT;

	error = nvram->ops->enum_buf(nvram, bhnd_nvram_find_bufvar_cb, &ctx);
	if (error)
		return (error);

	*value = ctx.iobuf;
	*len = ctx.iolen;
	return (ctx.ioerr);
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

	if (nvram->ops == NULL) {
		free(nvram->buf, M_BHND_NVRAM);
		return (EINVAL);
	}

	/* Perform format-specific initialization */
	if ((error = nvram->ops->init(nvram))) {
		free(nvram->buf, M_BHND_NVRAM);
		return (error);
	}

	// TODO
	const char *val;
	size_t val_len;

	if ((error = bhnd_nvram_find_var(nvram, "boardflags", &val, &val_len))) {
		NVRAM_LOG(nvram, "boardflags not found: %d\n", error);
	} else {
		NVRAM_LOG(nvram, "got boardflags: %.*s\n", val_len, val);
	}

	if ((error = bhnd_nvram_find_var(nvram, "boardtype", &val, &val_len))) {
		NVRAM_LOG(nvram, "boardtype not found: %d\n", error);
	} else {
		NVRAM_LOG(nvram, "got boardtype: %.*s\n", val_len, val);
	}

	return (0);
}

/**
 * Return true if @p ptr + nbytes falls within our backing buffer, false
 * otherwise.
 */
static bool
bhnd_nvram_bufptr_valid(struct bhnd_nvram *nvram, const void *ptr,
    size_t nbytes)
{
	const uint8_t *p = ptr;

	if (p < nvram->buf)
		goto failed;

	if (nbytes > nvram->buf_size)
		goto failed;

	if (p > nvram->buf + (nvram->buf_size - nbytes))
		goto failed;

	return (true);
	
failed:
	NVRAM_LOG(nvram, "NVRAM record not readable at %p+%#zx (base=%p, "
	    "len=%zu)\n", p, nbytes, nvram->buf, nvram->buf_size);
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

#if 0
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
#endif

static int
bhnd_nvram_enum_buf_bcm(struct bhnd_nvram *nvram, bhnd_nvram_enum_buf_cb cb,
    bhnd_nvram_ctx *ctx)
{
	const char	*p;
	size_t		 pos, len;
	bool		 stop;
	int		 error;

	stop = false;
	len = 0;
	pos = sizeof(struct bhnd_nvram_header);

	/* Iterate over all variables in the backing buffer */
	while (pos < nvram->buf_size) {
		p = nvram->buf + pos;

		/* EOF */
		if (*p == '\0')
			return (0);

		/* Issue callback */
		len = strnlen(p, nvram->buf_size - pos);
		if ((error = cb(nvram, p, len, &stop, ctx)))
			return (error);

		if (stop)
			break;

		/* Advance past current variable */
		pos += len;

		/* Skip trailing NUL */
		if (pos >= nvram->buf_size) {
			NVRAM_LOG(nvram, "warning: missing NVRAM termination "
			    "record");
			return (0);
		}
		pos++;
	}

	return (0);
}

static int
bhnd_nvram_init_tlv(struct bhnd_nvram *nvram)
{
	// TODO
	return (0);
}

static int
bhnd_nvram_enum_buf_tlv(struct bhnd_nvram *nvram, bhnd_nvram_enum_buf_cb cb,
    bhnd_nvram_ctx *ctx)
{
	const uint8_t	*p, *env;
	size_t		 env_len, rlen;
	bool		 stop;
	int		 error;

	stop = false;

	/* Iterate over all TLV records in the backing buffer */
	p = nvram->buf;
	for (p = nvram->buf; (p - nvram->buf) < nvram->buf_size; p += rlen)
	{
		uint8_t type;
		
		type = *p;
		rlen = 1;

		/* EOF */
		if (type == NVRAM_TLV_TYPE_END)
			return (0);

		/* Determine record length */
		p++;
		if (type & NVRAM_TLV_TF_U8_LEN) {
			if (!bhnd_nvram_bufptr_valid(nvram, p, 1))
				return (EINVAL);
	
			rlen = *p;
			p += 1;
		} else {
			if (!bhnd_nvram_bufptr_valid(nvram, p, 2))
				return (EINVAL);
			rlen = (p[0] << 8) | (p[1]);
			p += 2;
		}

		/* Verify record readability */
		if (!bhnd_nvram_bufptr_valid(nvram, p, rlen))
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

		/* Determine env string length */
		env = p;
		env_len = strnlen(env, rlen);

		/* Issue callback */
		if ((error = cb(nvram, env, env_len, &stop, ctx)))
			return (error);

		if (stop)
			break;
	}

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
	free(nvram->buf, M_BHND_NVRAM);
}
