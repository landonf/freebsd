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

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_map_data.h"

/*
 * Common NVRAM/SPROM support, including NVRAM variable map
 * lookup.
 */

#ifdef _KERNEL
MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "bhnd nvram data");
#endif

/*
 * CRC-8 lookup table used to checksum SPROM and NVRAM data via
 * bhnd_nvram_crc8().
 * 
 * Generated with following parameters:
 * 	polynomial:	CRC-8 (x^8 + x^7 + x^6 + x^4 + x^2 + 1)
 * 	reflected bits:	false
 * 	reversed:	true
 */
const uint8_t bhnd_nvram_crc8_tab[] = {
	0x00, 0xf7, 0xb9, 0x4e, 0x25, 0xd2, 0x9c, 0x6b, 0x4a, 0xbd, 0xf3,
	0x04, 0x6f, 0x98, 0xd6, 0x21, 0x94, 0x63, 0x2d, 0xda, 0xb1, 0x46,
	0x08, 0xff, 0xde, 0x29, 0x67, 0x90, 0xfb, 0x0c, 0x42, 0xb5, 0x7f,
	0x88, 0xc6, 0x31, 0x5a, 0xad, 0xe3, 0x14, 0x35, 0xc2, 0x8c, 0x7b,
	0x10, 0xe7, 0xa9, 0x5e, 0xeb, 0x1c, 0x52, 0xa5, 0xce, 0x39, 0x77,
	0x80, 0xa1, 0x56, 0x18, 0xef, 0x84, 0x73, 0x3d, 0xca, 0xfe, 0x09,
	0x47, 0xb0, 0xdb, 0x2c, 0x62, 0x95, 0xb4, 0x43, 0x0d, 0xfa, 0x91,
	0x66, 0x28, 0xdf, 0x6a, 0x9d, 0xd3, 0x24, 0x4f, 0xb8, 0xf6, 0x01,
	0x20, 0xd7, 0x99, 0x6e, 0x05, 0xf2, 0xbc, 0x4b, 0x81, 0x76, 0x38,
	0xcf, 0xa4, 0x53, 0x1d, 0xea, 0xcb, 0x3c, 0x72, 0x85, 0xee, 0x19,
	0x57, 0xa0, 0x15, 0xe2, 0xac, 0x5b, 0x30, 0xc7, 0x89, 0x7e, 0x5f,
	0xa8, 0xe6, 0x11, 0x7a, 0x8d, 0xc3, 0x34, 0xab, 0x5c, 0x12, 0xe5,
	0x8e, 0x79, 0x37, 0xc0, 0xe1, 0x16, 0x58, 0xaf, 0xc4, 0x33, 0x7d,
	0x8a, 0x3f, 0xc8, 0x86, 0x71, 0x1a, 0xed, 0xa3, 0x54, 0x75, 0x82,
	0xcc, 0x3b, 0x50, 0xa7, 0xe9, 0x1e, 0xd4, 0x23, 0x6d, 0x9a, 0xf1,
	0x06, 0x48, 0xbf, 0x9e, 0x69, 0x27, 0xd0, 0xbb, 0x4c, 0x02, 0xf5,
	0x40, 0xb7, 0xf9, 0x0e, 0x65, 0x92, 0xdc, 0x2b, 0x0a, 0xfd, 0xb3,
	0x44, 0x2f, 0xd8, 0x96, 0x61, 0x55, 0xa2, 0xec, 0x1b, 0x70, 0x87,
	0xc9, 0x3e, 0x1f, 0xe8, 0xa6, 0x51, 0x3a, 0xcd, 0x83, 0x74, 0xc1,
	0x36, 0x78, 0x8f, 0xe4, 0x13, 0x5d, 0xaa, 0x8b, 0x7c, 0x32, 0xc5,
	0xae, 0x59, 0x17, 0xe0, 0x2a, 0xdd, 0x93, 0x64, 0x0f, 0xf8, 0xb6,
	0x41, 0x60, 0x97, 0xd9, 0x2e, 0x45, 0xb2, 0xfc, 0x0b, 0xbe, 0x49,
	0x07, 0xf0, 0x9b, 0x6c, 0x22, 0xd5, 0xf4, 0x03, 0x4d, 0xba, 0xd1,
	0x26, 0x68, 0x9f
};

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
