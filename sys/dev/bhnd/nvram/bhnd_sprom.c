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
#include <sys/systm.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhndvar.h>

#include "bhnd_spromreg.h"
#include "bhnd_spromvar.h"

#include "nvramvar.h"

/*
 * BHND SPROM Parsing
 * 
 * Provides identification and parsing of BHND SPROM data.
 */

static int	sprom_extend_shadow(struct bhnd_sprom *sc, size_t image_size,
		    uint8_t *crc);
static int	sprom_read(struct bhnd_sprom *sc, off_t offset, void *buf,
		    size_t nbytes, uint8_t *crc);

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
	uint8_t		 crc;

	sprom->dev = rman_get_device(r->res);
	sprom->sp_res = r;
	sprom->sp_res_off = offset;

	/* Determine maximum possible SPROM image size */
	res_size = rman_get_size(r->res);
	if (offset >= res_size)
		return (EINVAL);

	sprom->sp_size_max = MIN(res_size - offset, BHND_SPROMSZ_MAX); 

	/* Allocate SPROM shadow */
	sprom->sp_size = 0;
	sprom->sp_capacity = sprom->sp_size_max;
	sprom->sp_shadow = malloc(sprom->sp_capacity, M_BHND, M_NOWAIT);
	if (sprom->sp_shadow == NULL)
		return (ENOMEM);

	/* Read and identify the SPROM by incrementally performing read + CRC
	 * validation on the possible SPROM image sizes, from smallest to
	 * largest */
	crc = BHND_NVRAM_CRC8_INITIAL;

	/* sromrev 1-3 */
	if ((error = sprom_extend_shadow(sprom, BHND_SPROMSZ_R1_3, &crc))) {
		goto failed;
	} else if (crc == BHND_NVRAM_CRC8_VALID) {
		device_printf(sprom->dev, "sromrev r1-3\n");
		return (0);
	}

	/* sromrev 4, 8-9 */
	if ((error = sprom_extend_shadow(sprom, BHND_SPROMSZ_R4_8_9, &crc))) {
		goto failed;
	} else if (crc == BHND_NVRAM_CRC8_VALID) {
		device_printf(sprom->dev, "sromrev r4, r8-9\n");
		return (0);
	}

	/* sromrev 10 */
	if ((error = sprom_extend_shadow(sprom, BHND_SPROMSZ_R10, &crc))) {
		goto failed;
	} else if (crc == BHND_NVRAM_CRC8_VALID) {
		device_printf(sprom->dev, "sromrev r10\n");
		return (0);
	}

	/* sromrev 11 */
	if ((error = sprom_extend_shadow(sprom, BHND_SPROMSZ_R11, &crc))) {
		goto failed;
	} else if (crc == BHND_NVRAM_CRC8_VALID) {
		device_printf(sprom->dev, "sromrev r11\n");
		return (0);
	}

	// TODO - fallback to <= r4?
	return (0);

failed:
	device_printf(sprom->dev, "unrecognized sprom format");
	return (error);
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
 * Extend the shadowed SPROM buffer to @p image_size, reading any required
 * data from the backing SPROM resource and updating @p crc.
 * 
 * @param sc parser state.
 * @param image_size required image size.
 * @param[in,out] crc CRC state.
 * 
 * @retval 0 success
 * @retval ENOSPC @p image_size exceeds the capacity of the backing buffer.
 * @retval EINVAL @p image_size is larger than the available sprom data.
 */
static int
sprom_extend_shadow(struct bhnd_sprom *sc, size_t image_size,
    uint8_t *crc)
{
	int	error;

	KASSERT(image_size > sc->sp_size, (("shadow truncation unsupported")));

	/* Verify the request fits within our shadow buffer */
	if (image_size > sc->sp_capacity)
		return (ENOSPC);

	/* Populate the extended range */
	error = sprom_read(sc, sc->sp_size, sc->sp_shadow + sc->sp_size,
	     image_size - sc->sp_size, crc);
	if (error)
		return (error);

	sc->sp_size = image_size;
	return (0);
}

/**
 * Read @p nbytes from SPROM resource at @p offset, updating @p crc.
 * 
 * @param sc parser state.
 * @param offset offset within SPROM at which to perform the read.
 * @param buf output buffer.
 * @param nbytes number of bytes to be read.
 * @param[in,out] crc CRC state
 */
static int
sprom_read(struct bhnd_sprom *sc, off_t offset, void *buf, size_t nbytes,
    uint8_t *crc)
{
	bus_size_t	 res_offset;
	size_t		 nread;
	uint16_t	*p;

	KASSERT(nbytes % 2 == 0, ("unaligned sprom read size"));
	KASSERT(offset % 2 == 0, ("unaligned sprom read offset"));

	/* Check for read overrun */
	if (offset >= sc->sp_size_max || sc->sp_size_max - offset < nbytes) {
		device_printf(sc->dev, "requested SPROM read would overrun\n");
		return (EINVAL);
	}

	p = (uint16_t *)buf;
	res_offset = sc->sp_res_off + offset;

	/* Perform read and update CRC */
	for (nread = 0; nread < nbytes; nread += 2) {
		uint16_t	val;
		uint8_t		cval;

		val = bhnd_bus_read_stream_2(sc->sp_res, res_offset+nread);

		/* Update CRC */
		cval = (val & 0xFF);
		*crc = bhnd_nvram_crc8(&cval, sizeof(cval), *crc);
		cval = (val & 0xFF00) >> 8;
		*crc = bhnd_nvram_crc8(&cval, sizeof(cval), *crc);

		*p++ = val;
	};

	return (0);
}
