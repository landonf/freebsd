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

#include "bhnd_nvram_io.h"

#include "bhnd_nvram_parservar.h"
#include "bhnd_nvram_parser.h"

/**
 * Probe to see if this NVRAM parser class supports the data mapped by the
 * given I/O context, returning a standard newbus device probe result
 * (see BUS_PROBE_*).
 *
 * @param cls The NVRAM class.
 * @param io An I/O context mapping the NVRAM data.
 *
 * @retval 0 if this is the only possible NVRAM parser parser for @p io.
 * @retval negative if the probe succeeds, a negative value should be returned;
 * the parser returning the highest negative value will be selected to handle
 * NVRAM parsing.
 * @retval ENXIO If the NVRAM format is not handled by this parser.
 * @retval positive if an error occurs during probing, a regular unix error
 * code should be returned.
 */
int
bhnd_nvram_parser_probe(bhnd_nvram_parser_class_t *cls,
    struct bhnd_nvram_io *io)
{
	return (cls->op_probe(io));
}

/**
 * Allocate and initialize a new instance of parser class @p cls, copying and
 * parsing NVRAM data from @p io.
  *
 * The caller is responsible for deallocating the parser instance
 * via bhnd_nvram_parser_free().
 * 
 * @param cls The parser class to be allocated.
 * @param nv On success, a pointer to the newly allocated NVRAM instance.
 * @param io An I/O context mapping the NVRAM data to be copied and parsed.
 * 
 * @retval 0 success
 * @retval non-zero if an error occurs during allocation or initialization, a
 * regular unix error code will be returned.
 */
int
bhnd_nvram_parser_new(bhnd_nvram_parser_class_t *cls,
    struct bhnd_nvram_parser **nv, struct bhnd_nvram_io *io)
{
	return (cls->op_new(nv, io));
}

/**
 * Free a previously allocated parser instance, releasing all associated
 * resources.
 * 
 * @param nv The NVRAM parser to be deallocated.
 */
void
bhnd_nvram_parser_free(struct bhnd_nvram_parser *nv)
{
	return (nv->cls->op_free(nv));
}

/**
 * Iterate over @p nv, returning the names of subsequent entries.
 * 
 * @param nv The NVRAM parser to be iterated.
 * @param cookiep A pointer to a cookiep value previously returned by
 * bhnd_nvram_parser_next(), or a NULL value to begin iteration.
 * 
 * @return Returns the next variable name, or NULL if there are no more
 * variables defined in @p nv.
 */
const char *
bhnd_nvram_parser_next(struct bhnd_nvram_parser *nv, void **cookiep)
{
	return (nv->cls->op_next(nv, cookiep));
}
