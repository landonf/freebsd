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

#ifdef _KERNEL

#include <sys/ctype.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/systm.h>

#include <machine/_inttypes.h>

#else /* !_KERNEL */

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#endif /* _KERNEL */

#include "bhnd_nvram_io.h"
#include "bhnd_nvram_private.h"

#include "bhnd_nvram_map_data.h"

/*
 * Common NVRAM/SPROM support, including NVRAM variable map
 * lookup.
 */

#ifdef _KERNEL
MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "bhnd nvram data");
#endif

/** signed/unsigned 32-bit integer value storage */
union bhnd_nvram_int_storage {
	uint32_t	u32;
	int32_t		s32;
};

/* Limit a size_t value to a suitable range for use as a printf string field
 * width */
#define	NVRAM_PRINT_WIDTH(_len)			\
	((_len) > (BHND_NVRAM_VAL_MAXLEN*2) ?	\
	    (BHND_NVRAM_VAL_MAXLEN*2) :		\
	    (int)(_len))

#define	NVRAM_LOG(_fmt, ...)	\
	printf("%s: " _fmt, __FUNCTION__, ##__VA_ARGS__)

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

/**
 * Identify the integer format of @p inp.
 *
 * @param inp Input string to be parsed.
 * @param ilen Length of the integer string readable via @p inp.
 * @param[out] base Integer base.
 * @param[out] negated True if integer is prefixed with negation sign.
 * 
 * @retval true if parsed successfully
 * @retval false if the format of @p inp cannot be determined.
 */
static bool
bhnd_nvram_ident_integer_fmt(const char *inp, size_t ilen, int *base,
    bool *negated)
{
	const char *p;

	/* Hex? */
	p = inp;
	if (ilen > 2 && p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
		bool valid;

		/* Check all input characters */
		valid = true;
		for (p = inp + 2; (size_t)(p - inp) < ilen; p++) {
			if (isxdigit(*p))
				continue;

			valid = false;
			break;
		}

		if (valid) {
			*base = 16;
			*negated = false;
			return (true);
		}
	}

	/* Decimal? */
	p = inp;
	if (ilen >= 1 && (*p == '-' || isdigit(*p))) {
		bool		 valid;

		valid = true;
		*negated = false;
		for (p = inp; (size_t)(p - inp) < ilen; p++) {
			if (p == inp && *p == '-') {
				*negated = true;
				continue;
			}

			if (isdigit(*p))
				continue;

			valid = false;
			break;
		}

		if (valid) {
			*base = 10;
			return (true);
		}
	}

	/* No match */
	*base = 0;
	*negated = false;
	return (false);
}

/**
 * Parse a field value, returning the actual pointer to the field string,
 * and the total size of the field string.
 * 
 * @param[in,out] inp The field string to parse. Will be updated to point
 * at the first non-whitespace character found.
 * @param ilen The length of @p inp, in bytes.
 * @param delim The field delimiter to search for.
 *
 * @return Returns the actual size of the field data.
 */
static size_t
bhnd_nvram_parse_field(const char **inp, size_t ilen, char delim)
{
	const char	*p, *sp;

	/* Skip any leading whitespace */
	for (sp = *inp; (size_t)(sp - *inp) < ilen && isspace(*sp); sp++)
		continue;

	*inp = sp;

	/* Find the last field character */
	for (p = *inp; (size_t)(p - *inp) < ilen; p++) {
		if (*p == delim || *p == '\0')
			break;
	}

	return (p - *inp);
}


/**
 * Determine whether @p inp is in octet string format, consisting of a
 * fields of two hex characters, separated with ':' or '-' delimiters.
 * 
 * This may be used to identify MAC address octet strings
 * (BHND_NVRAM_SFMT_MACADDR).
 *
 * @param		inp	The string to be parsed.
 * @param		ilen	The length of @p inp, in bytes.
 * @param[out]		delim	On success, the delimiter used by this octet
 * 				string.
 * 
 * @retval true		if @p inp is a valid octet string
 * @retval false	if @p inp is not a valid octet string.
 */
static bool
bhnd_nvram_ident_octet_string(const char *inp, size_t ilen, char *delim)
{
	size_t	elem_count;
	size_t	max_elem_count, min_elem_count;
	char	idelim;

	/* Require exactly two digits. If we relax this, there is room
	 * for ambiguity with signed integers and the '-' delimiter */
	min_elem_count = 2;
	max_elem_count = 2;

	/* Identify the delimiter used. The standard delimiter for MAC
	 * addresses is ':', but some earlier NVRAM formats may use '-' */
	for (const char *d = ":-";; d++) {
		const char *loc;

		/* No delimiter found, not an octet string */
		if (*d == '\0')
			return (false);

		/* Look for the delimiter */
		if ((loc = memchr(inp, *d, ilen)) == NULL)
			continue;

		/* Delimiter found */
		idelim = *loc;
		break;
	}

	/* To disambiguate from signed integers, if the delimiter is "-",
	 * the octets must be exactly 2 chars each */
	if (idelim == '-')
		min_elem_count = 2;

	/* String must be composed of individual octets (zero or more hex
	 * digits) separated by our delimiter. */
	elem_count = 0;
	for (const char *p = inp; (size_t)(p - inp) < ilen && *p != '\0'; p++) {
		switch (*p) {
		case ':':
		case '-':
			/* Hit a delim character; all delims must match
			 * the first delimiter used */
			if (*p != idelim)
				return (false);

			/* Must have parsed at least min_elem_count digits */
			if (elem_count < min_elem_count)
				return (false);

			/* Reset element count */
			elem_count = 0;
			break;
		default:
			/* More than maximum number of hex digits? */
			if (elem_count >= max_elem_count)
				return (false);

			/* Octet values must be hex digits */
			if (!isxdigit(*p))
				return (false);

			elem_count++;
			break;
		}
	}

	*delim = idelim;
	return (true);
}

/**
 * Coerce string @p inp to @p otype, writing the result to @p outp.
 *
 * @param[out]		outp	On success, the value will be written to this 
 *				buffer. This argment may be NULL if the value
 *				is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual size of the requested value.
 * @param		otype	The data type to be written to @p outp.
 * @param		odelim	The default string field delimiter to be
 *				emitted when an @p otype of BHND_NVRAM_TYPE_CSTR
 *				is provided. Ignored if @p hint defines a field
 *				delimiter format.
 * @param		inp	The string value to be coerced.
 * @param		ilen	The size of @p inp, in bytes.
 * @param		idelim	The string field delimiter to be used when
 *				parsing the input string. Ignored if @p hint
 *				defines a field delimiter format.
 * @param		hint	Variable formatting hint, or NULL.
 *
 * @retval 0		success
 * @retval ENOMEM	If @p outp is non-NULL and a buffer of @p olen is too
 *			small to hold the requested value.
 * @retval EFTYPE	If the variable data cannot be coerced to @p otype.
 * @retval ERANGE	If value coercion would overflow @p otype.
 */
static int
bhnd_nvram_coerce_string(void *outp, size_t *olen, bhnd_nvram_type otype,
    char odelim, const char *inp, size_t ilen, char idelim,
    struct bhnd_nvram_fmt_hint *hint)
{
	const char	*cstr;
	char		*cstr_buf, cstr_stack[BHND_NVRAM_VAL_MAXLEN];
	size_t		 cstr_size, cstr_len;
	size_t		 limit, nbytes;
	bool		 is_octet_str, free_cstr_buf;
	int		 error;

	nbytes = 0;
	cstr_buf = NULL;
	free_cstr_buf = false;

	if (outp != NULL)
		limit = *olen;
	else
		limit = 0;

	/*
	 * Determine whether this is an octet string, update the
	 * input delimiter as appropriate, and verify against the provided
	 * formatting hint (if any).
	 */
	is_octet_str = bhnd_nvram_ident_octet_string(inp, ilen, &idelim);
	if (hint != NULL &&
	    hint->sfmt == BHND_NVRAM_SFMT_MACADDR &&
	    !is_octet_str)
	{
		NVRAM_LOG("cannot format '%.*s' as MAC address\n",
		    NVRAM_PRINT_WIDTH(ilen), inp);
		return (EFTYPE);
	}

	/* If an octet string, we also must override the output delimiter */
	if (is_octet_str)
		odelim = idelim;

	/*
	 * We need a NUL-terminated instance of the string value
	 * for parsing.
	 */
	cstr_len = strnlen(inp, ilen);
	cstr_size = cstr_len + 1;

	if (cstr_size <= ilen) {
		/* String is already NUL terminated */
		cstr = inp;
	} else if (cstr_size <= sizeof(cstr_stack)) {
		/* Use stack allocated buffer to copy and NUL terminate
		 * the input string */
		cstr_buf = cstr_stack;
		cstr = cstr_buf;
	} else {
		/* Use heap allocated buffer to copy and NUL terminate
		 * the input string */
		if ((cstr_buf = bhnd_nv_malloc(cstr_size)) == NULL)
			return (ENOMEM);

		cstr = cstr_buf;
		free_cstr_buf = true;
	}

	/* Copy and NUL terminate */
	if (cstr_buf != NULL) {
		strncpy(cstr_buf, inp, cstr_len);
		cstr_buf[cstr_len] = '\0';

		cstr = cstr_buf;
	}

	/*
	 * Parse the NUL-terminated string.
	 */
	for (const char *p = cstr; *p != '\0';) {
		char				*endp;
		size_t				 field_len;
		int				 base;
		bool				 is_int, is_negated;
		union {
			unsigned long	u32;
			long		s32;
		} intv;

		/* Determine the field value's position and length,
		 * skipping any leading whitespace */
		field_len = cstr_len - (p - cstr);
		field_len = bhnd_nvram_parse_field(&p, field_len, idelim);

		/* Empty field values cannot be parsed as a fixed
		 * data type */
		if (field_len == 0) {
			NVRAM_LOG("error: cannot parse empty string "
			    "in '%s' at offset %zu\n", cstr, (size_t)(p-cstr));
			error = EFTYPE;
			goto finished;
		}

		/* Identify integer format */
		if (is_octet_str) {
			is_int = true;
			is_negated = false;
			base = 16;
		} else {
			is_int = bhnd_nvram_ident_integer_fmt(p, field_len,
			    &base, &is_negated);
		}

		/* Extract the field data */
#define	NV_READ_INT(_ctype, _max, _min, _dest, _strto)	do {		\
	if (!is_int) {							\
		error = EFTYPE;						\
		goto finished;						\
	}								\
									\
	if (is_negated && _min == 0) {					\
		error = ERANGE;						\
		goto finished;						\
	}								\
									\
	_dest = (_ctype) _strto(p, &endp, base);			\
	if (endp == p || !(*endp == '\0' || *endp == idelim)) {		\
		error = ERANGE;						\
		goto finished;						\
	}								\
									\
	if (_dest > _max || _dest < _min) {				\
		error = ERANGE;						\
		goto finished;						\
	}								\
									\
	if (limit > nbytes && limit - nbytes >= sizeof(_ctype))		\
		*((_ctype *)((uint8_t *)outp + nbytes)) = (_ctype)_dest;\
									\
	nbytes += sizeof(_ctype);					\
} while(0)

		switch (otype) {
		case BHND_NVRAM_TYPE_CHAR:
		case BHND_NVRAM_TYPE_CSTR:
			/* Copy out the characters directly */
			for (size_t i = 0; i < field_len; i++) {
				if (limit > nbytes)
					*((char *)outp + nbytes) = p[i];
				nbytes++;
			}
			break;

		case BHND_NVRAM_TYPE_UINT8:
			NV_READ_INT(uint8_t, UINT8_MAX, 0, intv.u32, strtoul);
			break;

		case BHND_NVRAM_TYPE_UINT16:
			NV_READ_INT(uint16_t, UINT16_MAX, 0, intv.u32, strtoul);
			break;

		case BHND_NVRAM_TYPE_UINT32:
			NV_READ_INT(uint32_t, UINT32_MAX, 0, intv.u32, strtoul);
			break;

		case BHND_NVRAM_TYPE_INT8:
			NV_READ_INT(int8_t, INT8_MAX, INT8_MIN, intv.s32,
			    strtol);
			break;

		case BHND_NVRAM_TYPE_INT16:
			NV_READ_INT(int16_t, INT16_MAX, INT16_MIN, intv.s32,
			    strtol);
			break;

		case BHND_NVRAM_TYPE_INT32:
			NV_READ_INT(int32_t, INT32_MAX, INT32_MIN, intv.s32,
			    strtol);
			break;

		default:
			NVRAM_LOG("unhandled NVRAM output type: %d\n", otype);
			error = EFTYPE;
			goto finished;
		}

		/* Advance to next field, skipping trailing input delimiter,
		 * and appending an output delimiter, if appropriate */
		p += field_len;
		if (*p == idelim) {
			p++;

			if (otype == BHND_NVRAM_TYPE_CSTR) {
				if (limit > nbytes)
					*((char *)outp + nbytes) = odelim;

				nbytes++;
			}
		}
	}

	/* If emitting a C string, append terminating NUL */
	if (otype == BHND_NVRAM_TYPE_CSTR) {
		if (limit > nbytes)
			*((char *)outp + nbytes) = '\0';

		nbytes++;
	}

	/* Provide the actual written length */
	*olen = nbytes;
	if (limit < *olen && outp != NULL) {
		error = ENOMEM;
	} else {
		error = 0;
	}

finished:
	if (free_cstr_buf)
		bhnd_nv_free(cstr_buf);

	return (error);
}

/**
 * Perform string formatting of integer value @p inv, writing the
 * result to @p outp.
 * 
 * Either NUL or a field delimiter character will be appended to the output if
 * required by both @p ofmt and the element's @p elem position.
 * 
 * The total number of bytes that would have been written if @p olen was
 * unlimited will be returned via @p olen. This includes any terminating
 * delimiter or NUL character.
 * 
 * @param		outp	The buffer to which the string value should be
 *				written.
 * @param[in,out]	olen	The total capacity of @p outp. On return, will
 *				contain the actual number of bytes required.
 * @param		ofmt	The output string format to be used when
 *				formatting @p inv.
 * @param		odelim	The default string field delimiter to be
 *				emitted. Ignored if @p ofmt defines a field
 *				delimiter format.
 * @param		inv	The integer value to be written to @p outp.
 * @param		itype	The integer type from which @p inv was parsed.
 * @param		elem	The element index being printed. If this is the
 *				first value in an array of elements, the index
 *				would be 0, the next would be 1, and so on.
 * @param		nelem	The total number of elements being printed.
 * 
 * @retval 0		success
 * @retval EFTYPE	If the variable data cannot be coerced to a string
 *			representation.
 * @retval ERANGE	If value coercion would overflow the string's integer
 *			representation.
 */
static int
bhnd_nvram_coerce_int_string(char *outp, size_t *olen, bhnd_nvram_sfmt ofmt,
    char odelim, const union bhnd_nvram_int_storage *inv, bhnd_nvram_type itype,
    size_t elem, size_t nelem)
{
	size_t		 iwidth;
	size_t		 limit;
	int		 nwrite;
	bool		 last_elem, has_delim;

	if (outp != NULL)
		limit = *olen;
	else
		limit = 0;

	/* Default to emitting a field element delimiter */
	has_delim = true;

	/* Is this the last element? */
	last_elem = false;
	if (elem + 1 == nelem)
		last_elem = true;

	/* Sanity check the input type */
	if (!BHND_NVRAM_INT_TYPE(itype)) {
		NVRAM_LOG("invalid type: %d\n", itype);
		return (EFTYPE);
	}

	iwidth = bhnd_nvram_type_width(itype);
	switch (iwidth) {
	case 1:
	case 2:
	case 4:
		break;
	default:
		NVRAM_LOG("invalid type width for %d: %zu\n", itype, iwidth);
		return (EFTYPE);
	}


	/* Format the string value */
	switch (ofmt) {
	case BHND_NVRAM_SFMT_MACADDR:
		/* Canonical MACADDR format uses a ':' delimiter */
		odelim = ':';

		nwrite = snprintf(outp, limit, "%02" PRIx32, inv->u32);
		break;
	case BHND_NVRAM_SFMT_LEDDC:
		/* Do not delimit LEDDC values; they're simply appended */
		has_delim = false;

		/* Only include the '0x' prefix on the first element */
		if (elem == 0) {
			nwrite = snprintf(outp, limit, "0x%0*" PRIx32,
			    (int)iwidth * 2 /* byte-width padding */, inv->u32);
		} else {
			nwrite = snprintf(outp, limit, "%0*" PRIx32,
			    (int)iwidth * 2 /* byte-width padding */, inv->u32);
		}
		break;
	case BHND_NVRAM_SFMT_HEX:
		nwrite = snprintf(outp, limit, "0x%0*" PRIx32,
		    (int)iwidth * 2 /* byte-width padding */,  inv->u32);
		break;

	case BHND_NVRAM_SFMT_DEC:
		if (BHND_NVRAM_SIGNED_TYPE(itype))
			nwrite = snprintf(outp, limit, "%" PRId32, inv->s32);
		else
			nwrite = snprintf(outp, limit, "%" PRIu32, inv->u32);

		break;
	case BHND_NVRAM_SFMT_CCODE: {
		unsigned char c;

		/* Do not delimit CCODE values; they're simply appended */
		has_delim = false;

		/* Must be representable as an ascii char */
		if (BHND_NVRAM_SIGNED_TYPE(itype)) {
			if (inv->s32 < 0 ||
			    inv->s32 > CHAR_MAX ||
			    !isascii(inv->s32))
			{
				NVRAM_LOG("cannot encode %" PRId32 "as ccode "
				    "element\n", inv->s32);
				return (ERANGE);
			}

			c = (char) inv->s32;
		} else {
			if (inv->u32 > CHAR_MAX || !isascii(inv->u32)) {
				NVRAM_LOG("cannot encode %" PRIu32 "as ccode "
				    "element\n", inv->s32);
				return (ERANGE);
			}

			c = inv->u32;
		}

		nwrite = snprintf(outp, limit, "%c", c);
		break;
	}
	default:
		NVRAM_LOG("unsupported output string format %d\n", ofmt);
		return (EFTYPE);
	}

	/* Handle snprintf failure */
	if (nwrite < 0) {
		NVRAM_LOG("snprintf() failed: %d\n", nwrite);
		return (EFTYPE);
	}

	/* Provide the number of bytes written (or required if we could write
	 * them), plus the cost of a trailing NUL or delimiter */
	*olen = nwrite;
	if (last_elem || has_delim)
		*olen += 1;

	/* If we exceeded our buffer capacity, nothing left to do */
	if ((size_t)nwrite >= limit)
		return (0);

	/* Do we need to replace NUL with a delimiter? */
	if (!last_elem && has_delim && odelim != '\0')
		outp[nwrite] = odelim;

	return (0);
}

/**
 * Coerce integer value @p inp to @p otype, writing the result to @p outp.
 *
 * @param[out]		outp	On success, the value will be written to this 
 *				buffer. This argment may be NULL if the value
 *				is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual size of the requested value.
 * @param		otype	The data type to be written to @p outp.
 * @param		odelim	The default string field delimiter to be
 *				emitted when an @p otype of BHND_NVRAM_TYPE_CSTR
 *				is provided. Ignored if @p hint defines a field
 *				delimiter format.
 * @param		inp	The string value to be coerced.
 * @param		ilen	The size of @p inp, in bytes.
 * @param		itype	The base data type of @p inp.
 * @param		hint	Variable formatting hint, or NULL.
 *
 * @retval 0		success
 * @retval ENOMEM	If @p outp is non-NULL and a buffer of @p olen is too
 *			small to hold the requested value.
 * @retval EFTYPE	If the variable data cannot be coerced to @p otype.
 * @retval ERANGE	If value coercion would overflow @p otype.
 * @retval EFAULT	If @p ilen is not correctly aligned for elements of
 *			type @p itype.
 */
static int
bhnd_nvram_coerce_int(void *outp, size_t *olen, bhnd_nvram_type otype,
    char odelim, const char *inp, size_t ilen, bhnd_nvram_type itype,
    struct bhnd_nvram_fmt_hint *hint)
{
	bhnd_nvram_sfmt	pfmt;
	size_t		limit, nbytes;
	size_t		iwidth, owidth;
	size_t		nelem;
	int		error;

	nbytes = 0;
	if (outp != NULL)
		limit = *olen;
	else
		limit = 0;

	/* Verify the input type */
	if (!BHND_NVRAM_INT_TYPE(itype)) {
		NVRAM_LOG("non-integer input type %d", itype);
		return (EFTYPE);
	}

	/*
	 * Fetch the string format to be used with an output type of
	 * BHND_NVRAM_TYPE_CSTR.
	 *
	 * We prefer the hinted format, but otherwise fall back back on a
	 * sane default.
	 */
	if (hint != NULL) {
		pfmt = hint->sfmt;
	} else {
		if (BHND_NVRAM_SIGNED_TYPE(itype))
			pfmt = BHND_NVRAM_SFMT_DEC;
		else
			pfmt = BHND_NVRAM_SFMT_HEX;
	}

	/* Determine the input integer type width */
	iwidth = bhnd_nvram_type_width(itype);
	if (iwidth == 0) {
		/* Shouldn't be possible (unless we add support for
		 * something like LEB128 encoding) */
		NVRAM_LOG("variable width integer input type %d", itype);
		return (EFTYPE);
	}

	/* Verify input buffer size and alignment for the given type. */                                                                                                                                                                      
	if (ilen % bhnd_nvram_type_width(itype) != 0)
		return (EFAULT);

	/* Reject empty input values */
	if (ilen == 0)
		return (EFAULT);

	/* Determine the number of input elements */
	nelem = ilen / iwidth;

	/* Determine the output type width (will be 0 if variable-width) */
	owidth = bhnd_nvram_type_width(otype);

	/* Iterate over the input elements, coercing to the output type
	 * and writing to the output buffer */
	for (size_t i = 0; i < nelem; i++) {
		union bhnd_nvram_int_storage	intv;
		size_t				remain;

		/* Read the input element */
		switch (itype) {
		case BHND_NVRAM_TYPE_UINT8:
			intv.u32 = *((const uint8_t *)inp + i);
			break;
		case BHND_NVRAM_TYPE_UINT16:
			intv.u32 = *((const uint16_t *)inp + i);
			break;
		case BHND_NVRAM_TYPE_UINT32:
			intv.u32 = *((const uint32_t *)inp + i);
			break;
		case BHND_NVRAM_TYPE_INT8:
			intv.s32 = *((const int8_t *)inp + i);
			break;
		case BHND_NVRAM_TYPE_INT16:
			intv.s32 = *((const int16_t *)inp + i);
			break;
		case BHND_NVRAM_TYPE_INT32:
			intv.s32 = *((const int32_t *)inp + i);
			break;
		case BHND_NVRAM_TYPE_CHAR:
			intv.s32 = *((const char *)inp + i);
			break;
			
		case BHND_NVRAM_TYPE_CSTR:
			/* unreachable */
			return (EFTYPE);
		}

		/* Handle signed/unsigned coercions */
		if (BHND_NVRAM_SIGNED_TYPE(itype) &&
		    BHND_NVRAM_UNSIGNED_TYPE(otype))
		{
			if (intv.s32 < 0) {
				/* Can't represent negative value */
				return (ERANGE);
			}

			/* Convert to unsigned representation */
			intv.u32 = intv.s32;
		} else if (BHND_NVRAM_UNSIGNED_TYPE(itype) &&
		    BHND_NVRAM_SIGNED_TYPE(otype))
		{
			/* Handle unsigned -> signed coercions */
			if (intv.u32 > INT32_MAX) {
				/* Can't represent positive value */
				return (ERANGE);
			}

			/* Convert to signed representation */
			intv.s32 = intv.u32;
		}
		
		/* Determine remaining space in output buffer */
		if (limit <= nbytes) {
			remain = 0;
		} else {
			BHND_NV_ASSERT(outp != NULL, ("NULL output buffer"));
			remain = limit - nbytes;
		}

		/* Write output */
		switch (otype) {
		case BHND_NVRAM_TYPE_UINT8:
			if (intv.u32 > UINT8_MAX)
				return (ERANGE);

			if (remain >= sizeof(uint8_t))
				*((uint8_t *)outp + i) = intv.u32;
			break;

		case BHND_NVRAM_TYPE_UINT16:
			if (intv.u32 > UINT16_MAX)
				return (ERANGE);

			if (remain >= sizeof(uint16_t))
				*((uint16_t *)outp + i) = intv.u32;
			break;

		case BHND_NVRAM_TYPE_UINT32:
			if (remain >= sizeof(uint32_t))
				*((uint32_t *)outp + i) = intv.u32;
			break;

		case BHND_NVRAM_TYPE_INT8:
			if (intv.s32 < INT8_MIN || intv.s32 > INT8_MAX)
				return (ERANGE);

			if (remain >= sizeof(int8_t))
				*((int8_t *)outp + i) = intv.s32;
			break;

		case BHND_NVRAM_TYPE_INT16:
			if (intv.s32 < INT16_MIN || intv.s32 > INT16_MAX)
				return (ERANGE);

			if (remain >= sizeof(uint16_t))
				*((int16_t *)outp + i) = intv.s32;
			break;

		case BHND_NVRAM_TYPE_INT32:
			if (remain >= sizeof(uint32_t))
				*((int32_t *)outp + i) = intv.s32;
			break;

		case BHND_NVRAM_TYPE_CHAR:
			if (intv.s32 < CHAR_MIN || intv.s32 > CHAR_MAX)
				return (ERANGE);

			if (remain >= sizeof(char))
				*((char *)outp + i) = intv.s32;
			break;

		case BHND_NVRAM_TYPE_CSTR: {
			char		*p;
			size_t		 p_len;

			/* Determine our output pointer */
			p_len = remain;
			if (p_len == 0) {
				/* We're just formatting to determine the
				 * required size */
				p = NULL;
			} else {
				p = (char *)outp + nbytes;
			}

			/* Attempt to write the entry + delimiter/NUL,
			 * which gives us the actual number of bytes required
			 * even if the buffer is too small. */
			error = bhnd_nvram_coerce_int_string(p, &p_len, pfmt,
			    odelim, &intv, itype, i, nelem);
			if (error)
				return (error);

			/* Add to total length */
			if (SIZE_MAX - p_len < nbytes)
				return (EFTYPE); /* string too long */

			nbytes += p_len;
			break;
		}
		default:
			NVRAM_LOG("unknown type %d\n", otype);
			return (EFTYPE);
		}

		nbytes += owidth;
	}

	/* Provide the actual length */
	*olen = nbytes;

	/* If no output was requested, nothing left to do */
	if (outp == NULL)
		return (0);

	/* Otherwise, report a memory error if the output buffer was too
	 * small */
	if (limit < nbytes)
		return (ENOMEM);

	return (0);
}

/**
 * Coerce value @p inp of type @p itype to @p otype, writing the
 * result to @p outp.
 *
 * @param[out]		outp	On success, the value will be written to this 
 *				buffer. This argment may be NULL if the value
 *				is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual size of the requested value.
 * @param		otype	The data type to be written to @p outp.
 * @param		odelim	The default string field delimiter to be
 *				emitted when an @p otype of BHND_NVRAM_TYPE_CSTR
 *				is provided. Ignored if @p hint defines a field
 *				delimiter format.
 * @param		inp	The value to be coerced.
 * @param		ilen	The size of @p inp, in bytes.
 * @param		itype	The base data type of @p inp.
 * @param		idelim	The string field delimiter to be used when
 *				parsing an @p itype of BHND_NVRAM_TYPE_CSTR.
 *				Ignored if @p hint defines a field delimiter
 *				format.
 * @param		hint	Variable formatting hint, or NULL.
 *
 * @retval 0		success
 * @retval ENOMEM	If @p outp is non-NULL and a buffer of @p olen is too
 *			small to hold the requested value.
 * @retval EFTYPE	If the variable data cannot be coerced to @p otype.
 * @retval ERANGE	If value coercion would overflow @p otype.
 * @retval EFAULT	If @p ilen is not correctly aligned for elements of
 *			type @p itype.
 */
int
bhnd_nvram_coerce_value(void *outp, size_t *olen, bhnd_nvram_type otype,
    char odelim, const void *inp, size_t ilen, bhnd_nvram_type itype,
    char idelim, struct bhnd_nvram_fmt_hint *hint)
{
	switch (itype) {
		case BHND_NVRAM_TYPE_CHAR:
		case BHND_NVRAM_TYPE_CSTR:
			return (bhnd_nvram_coerce_string(outp, olen, otype,
			    odelim, inp, ilen, idelim, hint));

		case BHND_NVRAM_TYPE_UINT8:
		case BHND_NVRAM_TYPE_UINT16:
		case BHND_NVRAM_TYPE_UINT32:
		case BHND_NVRAM_TYPE_INT8:
		case BHND_NVRAM_TYPE_INT16:
		case BHND_NVRAM_TYPE_INT32:
			return (bhnd_nvram_coerce_int(outp, olen, otype, odelim,
			    inp, ilen, itype, hint));

		default:
			NVRAM_LOG("unhandled NVRAM input type: %d\n", itype);
			return (EFTYPE);
	}
}

/**
 * Parse a 'name=value' string.
 * 
 * @param env The string to be parsed.
 * @param env_len The length of @p envp.
 * @param delim The delimiter used in @p envp. This will generally be '='.
 * @param[out] name If not NULL, a pointer to the name string. This argument
 * may be NULL.
 * @param[out] name_len On success, the length of the name substring. This
 * argument may be NULL.
 * @param[out] value On success, a pointer to the value substring. This argument
 * may be NULL.
 * @param[out] value_len On success, the length of the value substring. This
 * argument may be NULL.
 * 
 * @retval 0 success
 * @retval EINVAL if parsing @p envp fails.
 */
int
bhnd_nvram_parse_env(const char *env, size_t env_len, char delim,
    const char **name, size_t *name_len, const char **value, size_t *value_len)
{
	const char *p;

	/* Name */
	if ((p = memchr(env, delim, env_len)) == NULL) {
		NVRAM_LOG("delimiter '%c' not found in '%.*s'\n", delim,
		    NVRAM_PRINT_WIDTH(env_len), env);
		return (EINVAL);
	}

	/* Name */
	if (name != NULL)
		*name = env;
	if (name_len != NULL)
		*name_len = p - env;

	/* Skip delim */
	p++;

	/* Value */
	if (value != NULL)
		*value = p;
	if (value_len != NULL)
		*value_len = env_len - (p - env);

	return (0);
}
