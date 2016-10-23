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

#ifdef _KERNEL

#include <sys/ctype.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <machine/_inttypes.h>

#else /* !_KERNEL */

#include <ctype.h>
#include <string.h>

#endif /* _KERNEL */

#include "bhnd_nvramvar.h"

#include "bhnd_nvram_private.h"

#include "bhnd_nvram_map_data.h"

/*
 * Common NVRAM/SPROM support, including NVRAM variable map
 * lookup.
 */

#ifdef _KERNEL
MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "bhnd nvram data");
#endif

/**
 * Return the size of type @p type, or 0 if @p type has a variable width
 * (e.g. a C string).
 * 
 * @param type NVRAM data type.
 * @result the byte width of @p type.
 */
size_t
bhnd_nvram_type_width(bhnd_nvram_type type)
{
	switch (type) {
	case BHND_NVRAM_TYPE_INT8:
	case BHND_NVRAM_TYPE_UINT8:
	case BHND_NVRAM_TYPE_CHAR:
		return (sizeof(uint8_t));

	case BHND_NVRAM_TYPE_INT16:
	case BHND_NVRAM_TYPE_UINT16:
		return (sizeof(uint16_t));

	case BHND_NVRAM_TYPE_INT32:
	case BHND_NVRAM_TYPE_UINT32:
		return (sizeof(uint32_t));

	case BHND_NVRAM_TYPE_CSTR:
		return (0);
	}

	/* Quiesce gcc4.2 */
	BHND_NV_PANIC("bhnd nvram type %u unknown", type);
}

/* used by bhnd_nvram_find_vardefn() */
static int
bhnd_nvram_find_vardefn_compare(const void *key, const void *rhs)
{
	const struct bhnd_nvram_vardefn *r = rhs;

	return (strcmp((const char *)key, r->name));
}

/**
 * Find and return the variable definition for @p varname, if any.
 * 
 * @param varname variable name
 * 
 * @retval bhnd_nvram_vardefn If a valid definition for @p varname is found.
 * @retval NULL If no definition for @p varname is found. 
 */
const struct bhnd_nvram_vardefn *
bhnd_nvram_find_vardefn(const char *varname)
{
	return (bsearch(varname, bhnd_nvram_vardefns, bhnd_nvram_num_vardefns,
	    sizeof(bhnd_nvram_vardefns[0]), bhnd_nvram_find_vardefn_compare));
}

/**
 * Return the variable ID for a variable definition.
 * 
 * @param defn Variable definition previously returned by
 * bhnd_nvram_find_vardefn() or bhnd_nvram_get_vardefn().
 */
size_t
bhnd_nvram_get_vardefn_id(const struct bhnd_nvram_vardefn *defn)
{
	BHND_NV_ASSERT(
	    defn >= bhnd_nvram_vardefns &&
	    defn <= &bhnd_nvram_vardefns[bhnd_nvram_num_vardefns-1],
	    ("invalid variable definition pointer %p", defn));

	return (defn - bhnd_nvram_vardefns);
}

/**
 * Return the variable definition with the given @p id, or NULL
 * if no such variable ID is defined.
 * 
 * @param id variable ID.
 *
 * @retval bhnd_nvram_vardefn If a valid definition for @p id is found.
 * @retval NULL If no definition for @p id is found. 
 */
const struct bhnd_nvram_vardefn *
bhnd_nvram_get_vardefn(size_t id)
{
	if (id >= bhnd_nvram_num_vardefns)
		return (NULL);

	return (&bhnd_nvram_vardefns[id]);
}

/**
 * Validate an NVRAM variable name.
 * 
 * Scans for special characters (path delimiters, value delimiters, path
 * alias prefixes), returning false if the given name cannot be used
 * as a relative NVRAM key.
 * 
 * @param name A relative NVRAM variable name to validate.
 * @param name_len The length of @p name, in bytes.
 * 
 * @retval true If @p name is a valid relative NVRAM key.
 * @retval false If @p name should not be used as a relative NVRAM key.
 */
bool
bhnd_nvram_validate_name(const char *name, size_t name_len)
{
	size_t limit;

	limit = strnlen(name, name_len);
	if (limit == 0)
		return (false);

	/* Disallow path alias prefixes ([0-9]+:.*) */
	if (limit >= 2 && isdigit(*name)) {
		for (const char *p = name; (size_t)(p - name) < limit; p++) {
			if (isdigit(*p))
				continue;
			else if (*p == ':')
				return (false);
			else
				break;
		}
	}

	/* Scan for special characters */
	for (const char *p = name; (size_t)(p - name) < limit; p++) {
		switch (*p) {
		case '/':	/* path delimiter */
		case '=':	/* key=value delimiter */
			return (false);

		default:
			if (isspace(*p) || !isascii(*p))
				return (false);
		}
	}

	return (true);
}
