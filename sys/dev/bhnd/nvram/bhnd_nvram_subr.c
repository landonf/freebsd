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

static int	bhnd_nvram_init_bcm(struct bhnd_nvram *nvram);
static int	bhnd_nvram_init_tlv(struct bhnd_nvram *nvram);

typedef int	(*bhnd_nvram_op_init)(struct bhnd_nvram *nvram);

typedef int	(*bhnd_nvram_cb_enum_var)(struct bhnd_nvram *, const char *ptr,
		    size_t len, bool *stop, void *ctx);
typedef int	(*bhnd_nvram_op_enum_vars)(struct bhnd_nvram *nvram,
		    bhnd_nvram_cb_enum_var *fn, void *ctx);

/* Format-specific operations */
struct bhnd_nvram_ops {
	bhnd_nvram_format	fmt;		/**< nvram format */
	bhnd_nvram_op_init	init;
	bhnd_nvram_op_enum_vars	enum_vars;
};

static const struct bhnd_nvram_ops bhnd_nvram_ops_table[] = {
	{ BHND_NVRAM_FMT_BCM, bhnd_nvram_init_bcm, NULL },
	{ BHND_NVRAM_FMT_TLV, bhnd_nvram_init_tlv, NULL },
};

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

/**
 * Identify the NVRAM format at @p offset within @p r, verify the CRC (if applicable),
 * and allocate a local shadow copy of the NVRAM data.
 * 
 * After initialization, no reference to @p input will be held by the
 * NVRAM parser, and @p input may be safely deallocated.
 * 
 * @param[out] nvram On success, will be initialized with shadow of the NVRAM
 * data. 
 * @param input NVRAM data to be parsed.
 * @param fmt Required format of @p input.
 * 
 * @retval 0 success
 * @retval ENOMEM If internal allocation of NVRAM state fails.
 * @retval EINVAL If @p input parsing fails.
 */
int
bhnd_nvram_init(struct bhnd_nvram *nvram, struct bhnd_nvram_input *input,
    bhnd_nvram_format fmt)
{
	int error;

	/* Verify expected format */
	if (input->size < sizeof(union bhnd_nvram_ident))
		return (EINVAL);

	error = bhnd_nvram_identify(
	    (const union bhnd_nvram_ident *)input->buffer, fmt);
	if (error)
		return (error);

	/* Allocate backing buffer */
	nvram->buf_len = input->size;
	nvram->buf = malloc(input->size, M_BHND_NVRAM, M_NOWAIT);
	if (nvram->buf == NULL)
		return (ENOMEM);
	memcpy(nvram->buf, input->buffer, input->size);

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

	return (0);
}

static int
bhnd_nvram_init_bcm(struct bhnd_nvram *nvram)
{
	const uint8_t	*p;
	uint32_t	 cfg0;
	uint8_t		 crc, valid;

	/* Validate CRC */
	if (nvram->buf_len < NVRAM_CRC_SKIP)
		return (EINVAL);

	if (nvram->buf_len < sizeof(struct bhnd_nvram_header))
		return (EINVAL);

	cfg0 = ((struct bhnd_nvram_header *)nvram->buf)->cfg0;
	valid = (cfg0 & NVRAM_CFG0_CRC_MASK) >> NVRAM_CFG0_CRC_SHIFT;

	p = nvram->buf;
	crc = bhnd_nvram_crc8(p + NVRAM_CRC_SKIP, nvram->buf_len-NVRAM_CRC_SKIP,
	    BHND_NVRAM_CRC8_INITIAL);

	if (crc != valid) {
		printf("warning: NVRAM CRC error (crc=%#hhx, expected=%hhx)\n",
		    crc, valid);
	}

	// TODO
	p = nvram->buf;
	p += sizeof(struct bhnd_nvram_header);
	printf("first key=%s\n", p);
	return (0);
}


static int
bhnd_nvram_init_tlv(struct bhnd_nvram *nvram)
{
	// TODO
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
