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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_EROM_BHND_EROM_H_
#define _BHND_EROM_BHND_EROM_H_

#include <sys/param.h>
#include <sys/kobj.h>

#include <dev/bhnd/bhnd.h>

typedef kobj_class_t		 bhnd_erom_class_t;	/**< bhnd erom parser class */
typedef struct bhnd_erom	*bhnd_erom_t;		/**< bhnd erom parser instance */

#include "bhnd_erom_if.h"

bhnd_erom_t			 bhnd_erom_alloc(bhnd_erom_class_t cls,
				     device_t parent, int rid,
				     bus_addr_t enum_addr);
void				 bhnd_erom_free(bhnd_erom_t erom);

/**
 * Abstract bhnd_erom instance state. Must be first member of all subclass
 * instances.
 */
struct bhnd_erom {
	KOBJ_FIELDS;
};

/**
 * Parse all cores descriptors in @p erom, returning the array in @p cores and
 * the count in @p num_cores.
 * 
 * The memory allocated for the table should be freed using
 * `free(*cores, M_BHND)`. @p cores and @p num_cores are not changed
 * when an error is returned.
 * 
 * @param	erom		The erom parser to be queried.
 * @param[out]	cores		The table of parsed core descriptors.
 * @param[out]	num_cores	The number of core records in @p cores.
 * 
 * @retval 0		success
 * @retval non-zero	if an error occurs, a regular unix error code will
 *			be returned.
 */
static inline int
bhnd_erom_get_core_table(bhnd_erom_t erom, struct bhnd_core_info **cores,
    u_int *num_cores)
{
	return (BHND_EROM_GET_CORE_TABLE(erom, cores, num_cores));
}

/**
 * Locate the first core table entry in @p erom that matches @p desc.
 *
 * @param	erom	The erom parser to be queried.
 * @param	desc	A core match descriptor.
 * @param[out]	core	On success, the matching core info record.
 * 
 * @retval 0		success
 * @retval ENOENT	No core matching @p desc was found.
 * @retval non-zero	Reading or parsing failed.
 */
static inline int
bhnd_erom_lookup_core(bhnd_erom_t erom, const struct bhnd_core_match *desc,
    struct bhnd_core_info *core)
{
	return (BHND_EROM_LOOKUP_CORE(erom, desc, core));
}

/**
 * Locate the first core table entry in @p erom that matches @p desc,
 * and return the specified port region's base address and size.
 *
 * If a core matching @p desc is not found, or the requested port region
 * is not mapped to the matching core, ENOENT is returned.
 *
 * @param	erom	The erom parser to be queried.
 * @param	desc	A core match descriptor.
 * @param	type	The port type to search for.
 * @param	port	The port to search for.
 * @param	region	The port region to search for.
 * @param[out]	addr	On success, the base address of the port region.
 * @param[out]	size	On success, the total size of the port region.
 *
 * @retval 0		success
 * @retval ENOENT	No core matching @p desc was found.
 * @retval ENOENT	No port region matching @p type, @p port, and @p region
 *			was found.
 * @retval non-zero	Reading or parsing failed.
 */
static inline int
bhnd_erom_lookup_core_addr(bhnd_erom_t erom, const struct bhnd_core_match *desc,
    bhnd_port_type type, u_int port, u_int region, bhnd_addr_t *addr,
    bhnd_size_t *size)
{
	return (BHND_EROM_LOOKUP_CORE_ADDR(erom, desc, type, port, region,
	    addr, size));
};

#endif /* _BHND_EROM_BHND_EROM_H_ */
