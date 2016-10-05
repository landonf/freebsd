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
#include <sys/ctype.h>
#include <sys/limits.h>
#include <sys/systm.h>

#include <machine/_inttypes.h>

#include "bhnd_nvram_io.h"

#include "bhnd_nvram_datavar.h"
#include "bhnd_nvram_data.h"

#include "bhnd_nvram_common.h"

/* Limit a size_t value to a suitable range for use as a printf string field
 * width */
#define	NVRAM_PRINT_WIDTH(_len)			\
	((_len) > (BHND_NVRAM_VAL_MAXLEN*2) ?	\
	    (BHND_NVRAM_VAL_MAXLEN*2) :		\
	    (int)(_len))

#define	NVRAM_LOG(_fmt, ...)	\
	printf("%s: " _fmt, __FUNCTION__, ##__VA_ARGS__)

/** signed/unsigned 32-bit integer value storage */
union bhnd_nvram_int_storage {
	uint32_t	u32;
	int32_t		s32;
};

/**
 * Coerce a string value to a NUL terminated C string.
 * 
 * @param[out]		outp	On success, the C string will be written to this
 *				buffer. This argment may be NULL if the value
 *				is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual length of the C string, including
 *				terminating NUL.
 * @param		inp	The string value to be coerced.
 * @param		ilen	The length of @p inp, in bytes.
 * 
 * @retval 0		success
 * @retval ENOMEM	If @p outp is non-NULL and a buffer of @p olen is too
 *			small to hold the requested value.
 */
static int
bhnd_nvram_coerce_string_cstr(char *outp, size_t *olen, const char *inp,
    size_t ilen)
{
	size_t str_len;
	size_t req_size;
	size_t max_size;

	if (outp != NULL)
		max_size = *olen;
	else
		max_size = 0;

	/* Determine input and output sizes, including whether additional space
	 * is required for a trailing NUL */
	str_len = strnlen(inp, ilen);
	if (str_len == ilen)
		req_size = str_len + 1;
	else
		req_size = ilen;

	/* Provide actual size to caller */
	*olen = req_size;
	if (max_size < req_size) {
		if (outp != NULL)
			return (ENOMEM);
		else
			return (0);
	}

	/* Copy and NUL terminate output */
	if (outp != NULL) {
		memcpy(outp, inp, str_len);
		outp[str_len] = '\0';
	}

	return (0);
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
		for (p = inp + 2; p - inp < ilen; p++) {
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
		for (p = inp; p - inp < ilen; p++) {
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
	for (sp = *inp; sp - *inp < ilen && isspace(*sp); sp++)
		continue;

	*inp = sp;

	/* Find the last field character */
	for (p = *inp; p - *inp < ilen; p++) {
		if (*p == delim || *p == '\0')
			break;
	}

	return (p - *inp);
}


/**
 * Determine whether @p inp is in "octet string" format (i.e.
 * BHND_NVRAM_SFMT_MACADDR), consisting of hex octets separated with ':' or
 * '-'.
 *
 * @param		inp	The string to be parsed.
 * @param		ilen	The length of @p inp, in bytes.
 * @param		delim	On success, the delimiter used by this octet
 * 				string.
 * 
 * @retval true		if @p inp is a valid octet string
 * @retval false	if @p inp is not a valid octet string.
 */
static bool
bhnd_nvram_ident_octet_string(const char *inp, size_t ilen, char *delim)
{
	size_t slen;

	slen = strnlen(inp, ilen);

	/* String length (not including NUL) must be aligned on an octet
	 * boundary ('AA:BB', not 'AA:B', etc), and must be large enough
	 * to contain at least two octet entries. */
	if (slen % 3 != 2 || slen < sizeof("AA:BB") - 1)
		return (false);

	/* Identify the delimiter used. The standard delimiter for
	 * MAC addresses is ':', but some earlier NVRAM formats may use
	 * '-' */
	switch ((*delim = inp[2])) {
	case ':':
	case '-':
		break;
	default:
		return (false);
	}

	/* Verify the octet string contains only hex digits */ 
	for (const char *p = inp; p - inp < ilen; p++) {
		size_t pos;

		pos = (p - inp);

		/* Skip delimiter after each octet */
		if (pos % 3 == 2) {
			if (*p == *delim)
				continue;

			if (*p == '\0')
				return (0);

			/* No delimiter? */
			return (false);
		}

		if (!isxdigit(*p))
			return (false);
	}

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
 * @param		inp	The string value to be coerced.
 * @param		ilen	The size of @p inp, in bytes.
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
    const char *inp, size_t ilen, struct bhnd_nvram_fmt_hint *hint)
{
	const char	*cstr;
	char		*cstr_buf, cstr_stack[BHND_NVRAM_VAL_MAXLEN];
	size_t		 cstr_size, cstr_len;
	size_t		 limit, nbytes;
	bool		 is_octet_str, free_cstr_buf;
	char		 delim;
	int		 error;

	nbytes = 0;
	cstr_buf = NULL;
	free_cstr_buf = false;

	if (outp != NULL)
		limit = *olen;
	else
		limit = 0;

	/*
	 * Determine whether this is an octet string, and validate
	 * the result against the provided formatting hint (if any)
	 */
	is_octet_str = bhnd_nvram_ident_octet_string(inp, ilen, &delim);
	if (!is_octet_str) {
		/* Use standard field delimiter */
		delim = ',';
	}

	if (hint != NULL && hint->sfmt == BHND_NVRAM_SFMT_MACADDR) {
		if (!is_octet_str) {
			NVRAM_LOG("cannot format '%.*s' as MAC address\n",
			    NVRAM_PRINT_WIDTH(ilen), inp);
			return (EFTYPE);
		}
	}

	/*
	 * If the output type is also a C string, populate it directly from
	 * the input string
	 */
	if (otype == BHND_NVRAM_TYPE_CSTR)
		return (bhnd_nvram_coerce_string_cstr(outp, olen, inp, ilen));

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
		cstr_buf = malloc(cstr_size, M_BHND_NVRAM, M_NOWAIT|M_WAITOK);
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
		field_len = bhnd_nvram_parse_field(&p, field_len, delim);

		/* Empty field values cannot be parsed as a fixed
		 * data type */
		if (field_len == 0) {
			NVRAM_LOG("error: cannot parse empty string "
			    "in '%s'\n", cstr);
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
	_dest = _strto(p, &endp, base);					\
	if (endp == p || !(*endp == '\0' || *endp == delim)) {		\
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

		case BHND_NVRAM_TYPE_CSTR:	/* Must be handled above */
			/* fallthrough */
		default:
			NVRAM_LOG("unhandled NVRAM output type: %d\n", otype);
			error = EFTYPE;
			goto finished;
		}

		/* Advance to next field, skip any trailing delimiter */
		p += field_len;
		if (*p == delim)
			p++;
	}

	error = 0;

finished:
	if (free_cstr_buf)
		free(cstr_buf, M_BHND_NVRAM);

	return (error);
}

/**
 * Perform string formatting of integer value @p inv, writing the
 * result to @p outp.
 * 
 * Either NUL or an array delimiter character will be appended to the output if
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
    const union bhnd_nvram_int_storage *inv, bhnd_nvram_type itype, size_t elem,
    size_t nelem)
{
	size_t		 iwidth;
	size_t		 limit;
	char		 delim;
	int		 nwrite;
	bool		 last_elem, has_delim;

	if (outp != NULL)
		limit = *olen;
	else
		limit = 0;

	/* Default array element delimiter */
	delim = ',';
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
		delim = ':';

		nwrite = snprintf(outp, limit, "%02" PRIX32, inv->u32);
		break;
	case BHND_NVRAM_SFMT_LEDDC:
		/* Do not delimit LEDDC values; they're simply appended */
		has_delim = false;

		/* Only include the '0x' prefix on the first element */
		if (elem == 0) {
			nwrite = snprintf(outp, limit, "0x%0*" PRIX32,
			    (int)iwidth * 2 /* byte-width padding */, inv->u32);
		} else {
			nwrite = snprintf(outp, limit, "%0*" PRIX32,
			    (int)iwidth * 2 /* byte-width padding */, inv->u32);
		}
		break;
	case BHND_NVRAM_SFMT_HEX:
		nwrite = snprintf(outp, limit, "0x%0*" PRIX32,
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
	if (nwrite >= limit)
		return (0);

	/* Do we need to replace NUL with a delimiter? */
	if (!last_elem && has_delim)
		outp[nwrite] = delim;

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
    const char *inp, size_t ilen, bhnd_nvram_type itype,
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
			KASSERT(outp != NULL, ("NULL output buffer"));
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
			    &intv, itype, i, nelem);
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
 * @param		inp	The value to be coerced.
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
int
bhnd_nvram_coerce_value(void *outp, size_t *olen, bhnd_nvram_type otype,
    const void *inp, size_t ilen, bhnd_nvram_type itype,
    struct bhnd_nvram_fmt_hint *hint)
{
	switch (itype) {
		case BHND_NVRAM_TYPE_CHAR:
		case BHND_NVRAM_TYPE_CSTR:
			return (bhnd_nvram_coerce_string(outp, olen, otype, inp,
			    ilen, hint));

		case BHND_NVRAM_TYPE_UINT8:
		case BHND_NVRAM_TYPE_UINT16:
		case BHND_NVRAM_TYPE_UINT32:
		case BHND_NVRAM_TYPE_INT8:
		case BHND_NVRAM_TYPE_INT16:
		case BHND_NVRAM_TYPE_INT32:
			return (bhnd_nvram_coerce_int(outp, olen, otype, inp,
			    ilen, itype, hint));

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
