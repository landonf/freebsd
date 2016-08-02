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
 * BHND NVRAM support.
 */

static int	bhnd_nvram_init_bcm(struct bhnd_nvram *sc,
		    struct bhnd_nvram_input *input);
static int	bhnd_nvram_init_tlv(struct bhnd_nvram *sc,
		    struct bhnd_nvram_input *input);
static int	bhnd_nvram_read(struct bhnd_nvram_input *input, size_t offset,
		    void *output, size_t nbytes);

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
bhnd_nvram_identify(union bhnd_nvram_ident *ident, bhnd_nvram_format expected)
{
	switch (expected) {
	case BHND_NVRAM_FMT_BCM:
		if (ident->bcm.magic == NVRAM_MAGIC)
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
	union bhnd_nvram_ident	ident;
	int			error;


	/* Read NVRAM ident data */
	if ((error = bhnd_nvram_read(input, 0, &ident, sizeof(ident))))
		return (error);

	/* Verify expected format */
	if ((error = bhnd_nvram_identify(&ident, fmt)))
		return (error);

	switch (fmt) {
	case BHND_NVRAM_FMT_BCM:
		nvram->header = ident.bcm;
		nvram->fmt = fmt;
		return (bhnd_nvram_init_bcm(nvram, input));
	case BHND_NVRAM_FMT_TLV:
		nvram->fmt = fmt;
		return (bhnd_nvram_init_tlv(nvram, input));
	default:
		return (EINVAL);
	}
}

static int
bhnd_nvram_init_bcm(struct bhnd_nvram *sc, struct bhnd_nvram_input *input)
{
	// TODO
	return (0);
}


static int
bhnd_nvram_init_tlv(struct bhnd_nvram *sc, struct bhnd_nvram_input *input)
{
	// TODO
	return (0);
}


static int
bhnd_nvram_read(struct bhnd_nvram_input *input, size_t offset, void *output,
    size_t nbytes)
{
	if (offset > input->size || input->size - offset < nbytes)
		return (ENXIO);

	memcpy(output, ((const uint8_t *)input->buffer) + offset, nbytes);
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
	// TODO
}
