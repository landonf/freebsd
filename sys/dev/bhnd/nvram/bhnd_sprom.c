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

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhndvar.h>

#include "bhnd_spromvar.h"

/*
 * BHND SPROM Parsing
 * 
 * Provides identification and parsing of BHND SPROM data.
 */

/**
 * Identify the SPROM format at @p offset within @p r, verify the CRC,
 * and allocate a local shadow copy of the SPROM data.
 * 
 * After successful initialization, @p r will not be accessed; any pin/gpio
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
	// TODO
	return (ENXIO);
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
