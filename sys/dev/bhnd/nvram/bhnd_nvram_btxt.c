/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
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
#include <sys/rman.h>

#include <machine/bus.h>

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_bcmreg.h"

#include "bhnd_nvram_parser.h"
#include "bhnd_nvram_parservar.h"

/**
 * Broadcom board text file parser.
 *
 * This format is used to provide external NVRAM data for some
 * fullmac WiFi devices, and as an input format when programming
 * NVRAM/SPROM/OTP.
 */

struct bhnd_nvram_btxt {
	struct bhnd_nvram_parser	 nv;	/**< common instance state */
	struct bhnd_nvram_io		*data;	/**< memory-backed board text data */
};

BHND_NVRAM_PARSER_DEFN(btxt)

/** Minimal identification header */
union bhnd_nvram_btxt_ident {
	uint32_t	bcm_magic;
	char		btxt[8];
};

static int
bhnd_nvram_btxt_probe(struct bhnd_nvram_io *io)
{
	union bhnd_nvram_btxt_ident	ident;
	size_t				nbytes;
	char				c;
	int				error;

	/* Look at the initial header for something that looks like 
	 * an ASCII board text file */
	nbytes = sizeof(ident);
	if ((error = bhnd_nvram_io_read(io, 0x0, &ident, &nbytes)))
		return (error);

	if (nbytes < sizeof(ident))
		return (ENXIO);

	/* The BCM NVRAM format uses a 'FLSH' little endian magic value, which
	 * shouldn't be interpreted as BTXT */
	if (le32toh(ident.bcm_magic) == BCM_NVRAM_MAGIC)
		return (ENXIO);

	/* Don't match on non-ASCII data */
	for (size_t i = 0; i < nitems(ident.btxt); i++) {
		c = ident.btxt[i];
		if (!isprint(c) && !isspace(c))
			return (ENXIO);
	}

	/* The first character should either be a valid key char (alpha),
	 * whitespace, or the start of a comment ('#') */
	c = ident.btxt[0];
	if (!isspace(c) && !isalpha(c) && c != '#')
		return (ENXIO);

	/* We assert a low priority, given that we've only scanned an
	 * initial few bytes of the file. In practice, BTXT will generally
	 * be referenced explicitly when parsing files of a known format. */
	return (BUS_PROBE_LOW_PRIORITY);
}

static int
bhnd_nvram_btxt_new(struct bhnd_nvram_parser **nv, struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_btxt	*btxt;
	int			 error;

	/* Allocate and initialize the BTXT parser instance */
	btxt = malloc(sizeof(*btxt), M_BHND_NVRAM, M_NOWAIT|M_ZERO);
	if (btxt == NULL)
		return (ENOMEM);

	/* Copy the BTXT input data */
	if ((btxt->data = bhnd_nvram_iobuf_copy(io)) == NULL) {
		error = ENOMEM;
		goto failed;
	}

	*nv = &btxt->nv;
	return (0);

failed:
	if (btxt->data != NULL)
		bhnd_nvram_io_free(btxt->data);

	free(btxt, M_BHND_NVRAM);

	return (error);
}

static void
bhnd_nvram_btxt_free(struct bhnd_nvram_parser *nv)
{
	struct bhnd_nvram_btxt	*btxt = (struct bhnd_nvram_btxt *)nv;

	bhnd_nvram_io_free(btxt->data);
	free(nv, M_BHND_NVRAM);
}


