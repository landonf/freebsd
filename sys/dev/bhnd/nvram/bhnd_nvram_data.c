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
#include <sys/systm.h>

#include "bhnd_nvram_io.h"

#include "bhnd_nvram_datavar.h"
#include "bhnd_nvram_data.h"

/**
 * Return a human-readable description for the given NVRAM data class.
 * 
 * @param cls The NVRAM class.
 */
const char *
bhnd_nvram_data_class_desc(bhnd_nvram_data_class_t *cls)
{
	return (cls->desc);
}

/**
 * Probe to see if this NVRAM data class class supports the data mapped by the
 * given I/O context, returning a standard newbus device probe result
 * (see BUS_PROBE_*).
 *
 * @param cls The NVRAM class.
 * @param io An I/O context mapping the NVRAM data.
 *
 * @retval 0 if this is the only possible NVRAM data class for @p io.
 * @retval negative if the probe succeeds, a negative value should be returned;
 * the class returning the highest negative value should be selected to handle
 * NVRAM parsing.
 * @retval ENXIO If the NVRAM format is not handled by @p cls.
 * @retval positive if an error occurs during probing, a regular unix error
 * code should be returned.
 */
int
bhnd_nvram_data_probe(bhnd_nvram_data_class_t *cls, struct bhnd_nvram_io *io)
{
	return (cls->op_probe(io));
}

/**
 * Allocate and initialize a new instance of data class @p cls, copying and
 * parsing NVRAM data from @p io.
  *
 * The caller is responsible for deallocating the parser instance
 * via bhnd_nvram_data_free().
 * 
 * @param cls The data class to be allocated.
 * @param nv On success, a pointer to the newly allocated NVRAM data instance.
 * @param io An I/O context mapping the NVRAM data to be copied and parsed.
 * 
 * @retval 0 success
 * @retval non-zero if an error occurs during allocation or initialization, a
 * regular unix error code will be returned.
 */
int
bhnd_nvram_data_new(bhnd_nvram_data_class_t *cls,
    struct bhnd_nvram_data **nv, struct bhnd_nvram_io *io)
{
	return (cls->op_new(nv, io));
}

/**
 * Free a previously allocated data instance, releasing all associated
 * resources.
 * 
 * @param nv The NVRAM data to be deallocated.
 */
void
bhnd_nvram_data_free(struct bhnd_nvram_data *nv)
{
	return (nv->cls->op_free(nv));
}

/**
 * Return the capability flags (@see BHND_NVRAM_DATA_CAP_*) for @p nv.
 */
uint32_t
bhnd_nvram_data_getcaps(struct bhnd_nvram_data *nv)
{
	return (nv->cls->op_getcaps(nv));
}

/**
 * Iterate over @p nv, returning the names of subsequent variables.
 * 
 * @param nv The NVRAM data to be iterated.
 * @param[in,out] cookiep A pointer to a cookiep value previously returned by
 * bhnd_nvram_data_next(), or a NULL value to begin iteration.
 * 
 * @return Returns the next variable name, or NULL if there are no more
 * variables defined in @p nv.
 */
const char *
bhnd_nvram_data_next(struct bhnd_nvram_data *nv, void **cookiep)
{
	return (nv->cls->op_next(nv, cookiep));
}


/**
 * Search @p nv for a named variable, returning the variable's opaque reference
 * if found, or NULL if unavailable.
 *
 * The BHND_NVRAM_DATA_CAP_INDEXED capability flag will be returned by
 * bhnd_nvram_data_getcaps() if @p nv supports effecient name-based
 * lookups.
 *
 * @param nv The NVRAM data to search.
 * @param name The name to search for.
 *
 * @retval non-NULL	If @p name is found, the opaque cookie value will be
 *			returned.
 * @retval NULL		If @p name is not found.
 */
void *
bhnd_nvram_data_find(struct bhnd_nvram_data *nv, const char *name)
{
	return (nv->cls->op_find(nv, name));
}

/**
 * A generic implementation of bhnd_nvram_data_find().
 *
 * This implementation will use bhnd_nvram_data_next() to perform a
 * simple O(n) case-insensitve search for @p name.
 */
void *
bhnd_nvram_data_generic_find(struct bhnd_nvram_data *nv, const char *name)
{
	const char	*next;
	void		*cookiep;

	cookiep = NULL;
	while ((next = bhnd_nvram_data_next(nv, &cookiep))) {
		if (strcasecmp(name, next) == 0)
			return (cookiep);
	}

	/* Not found */
	return (NULL);
}


/**
 * Read a variable and decode as @p type.
 *
 * @param		nv	The NVRAM data.
 * @param		cookiep	An NVRAM variable cookie previously returned
 *				via bhnd_nvram_data_next() or
 *				bhnd_nvram_data_find().
 * @param[out]		buf	On success, the requested value will be written
 *				to this buffer. This argment may be NULL if
 *				the value is not desired.
 * @param[in,out]	len	The capacity of @p buf. On success, will be set
 *				to the actual size of the requested value.
 * @param		type	The data type to be written to @p buf.
 *
 * @retval 0		success
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 * @retval EFTYPE	If the variable data cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 */
int
bhnd_nvram_data_getvar(struct bhnd_nvram_data *nv, void *cookiep, void *buf,
    size_t *len, bhnd_nvram_type type)
{
	return (nv->cls->op_getvar(nv, cookiep, buf, len, type));
}

/**
 * If available and supported by the NVRAM data instance, return a reference
 * to the internal buffer containing an entry's variable data,
 * 
 * Note that string values may not be NUL terminated.
 *
 * @param		nv	The NVRAM data.
 * @param		cookiep	An NVRAM variable cookie previously returned
 *				via bhnd_nvram_data_next() or
 *				bhnd_nvram_data_find().
 * @param[out]		len	On success, will be set to the actual size of
 *				the requested value.
 * @param[out]		type	The data type of the entry data.
 *
 * @retval non-NULL	success
 * @retval NULL		if direct data access is unsupported by @p nv, or
 *			unavailable for @p cookiep.
 */
const void *
bhnd_nvram_data_getvar_ptr(struct bhnd_nvram_data *nv, void *cookiep,
    size_t *len, bhnd_nvram_type *type)
{
	return (nv->cls->op_getvar_ptr(nv, cookiep, len, type));
}


/**
 * Return the variable name associated with a given @p cookiep.
 * @param		nv	The NVRAM data to be iterated.
 * @param[in,out]	cookiep	A pointer to a cookiep value previously returned
 *				via bhnd_nvram_data_next() or
 *				bhnd_nvram_data_find().
 *
 * @return Returns the variable's name.
 */
const char *
bhnd_nvram_data_getvar_name(struct bhnd_nvram_data *nv, void *cookiep)
{
	return (nv->cls->op_getvar_name(nv, cookiep));
}
