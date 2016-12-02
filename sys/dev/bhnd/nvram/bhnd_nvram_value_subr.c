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

#include <sys/systm.h>

#else /* !_KERNEL */

#include <errno.h>
#include <string.h>

#endif /* _KERNEL */

#include "bhnd_nvram_private.h"
#include "bhnd_nvram_valuevar.h"

/**
 * Calculate the number of elements represented by a value of @p len bytes
 * with @p type.
 *
 * @param	type	The value type.
 * @param	data	The actual data to be queried, or NULL if unknown.
 * @param	len	The length in bytes of @p data, or if @p data is NULL,
 *			the expected length in bytes.
 * @param[out]	nelem	On success, the number of elements. If @p type is not
 *			a fixed width type (e.g. BHND_NVRAM_TYPE_STRING_ARRAY),
 *			and @p data is NULL, an @p nelem value of 0 will be
 *			returned.
 *
 * @retval 0		success
 * @retval EFTYPE	if @p type is not an array type, and @p len is not
 *			equal to the size of a single element of @p type.
 * @retval EFAULT	if @p len is not correctly aligned for elements of
 *			@p type.
 */
int
bhnd_nvram_value_nelem(bhnd_nvram_type type, const void *data, size_t len,
    size_t *nelem)
{
	bhnd_nvram_type	base_type;
	size_t		base_size;

	/* Length must be aligned to the element size */
	base_type = bhnd_nvram_base_type(type);
	base_size = bhnd_nvram_value_size(base_type, NULL, 0, 1);
	if (base_size != 0 && len % base_size != 0)
		return (EFAULT);

	switch (type) {
	case BHND_NVRAM_TYPE_BYTES:
		/* Always exactly one element */
		*nelem = 1;
		return (0);

	case BHND_NVRAM_TYPE_NULL:
		/* Must be zero length */
		if (len != 0)
			return (EFAULT);

		/* Always exactly one element */
		*nelem = 1;
		return (0);

	case BHND_NVRAM_TYPE_STRING:
	case BHND_NVRAM_TYPE_STRING_ARRAY: {
		const char	*p;
		size_t		 nleft;

		/* Cannot determine the element count without parsing
		 * the actual data */
		if (data == NULL) {
			*nelem = 0;
			return (0);
		}

		/* Iterate over the NUL-terminated strings to calculate
		 * total element count */
		p = data;
		nleft = len;
		*nelem = 0;
		while (nleft > 0) {
			size_t slen;

			/* Increment element count */
			(*nelem)++;

			/* If not a string array, data must not contain more
			 * than one entry. */
			if (!bhnd_nvram_is_array_type(type) && *nelem > 1)
				return (EFTYPE);

			/* Determine string length */
			slen = strnlen(p, nleft);
			nleft -= slen;
	
			/* Advance input */
			p += slen;

			/* Account for trailing NUL, if we haven't hit the end
			 * of the input */
			if (nleft > 0) {
				nleft--;
				p++;
			}
		}

		return (0);
	}

	case BHND_NVRAM_TYPE_INT8:
	case BHND_NVRAM_TYPE_UINT8:
	case BHND_NVRAM_TYPE_CHAR:
	case BHND_NVRAM_TYPE_INT16:
	case BHND_NVRAM_TYPE_UINT16:
	case BHND_NVRAM_TYPE_INT32:
	case BHND_NVRAM_TYPE_UINT32:
	case BHND_NVRAM_TYPE_INT64:
	case BHND_NVRAM_TYPE_UINT64:
	case BHND_NVRAM_TYPE_BOOL:
		/* Length must be equal to the size of exactly one
		 * element (arrays can represent zero elements -- non-array
		 * types cannot) */
		if (len != base_size)
			return (EFTYPE);
		*nelem = 1;
		return (0);

	case BHND_NVRAM_TYPE_UINT8_ARRAY:
	case BHND_NVRAM_TYPE_UINT16_ARRAY:
	case BHND_NVRAM_TYPE_UINT32_ARRAY:
	case BHND_NVRAM_TYPE_UINT64_ARRAY:
	case BHND_NVRAM_TYPE_INT8_ARRAY:
	case BHND_NVRAM_TYPE_INT16_ARRAY:
	case BHND_NVRAM_TYPE_INT32_ARRAY:
	case BHND_NVRAM_TYPE_INT64_ARRAY:
	case BHND_NVRAM_TYPE_CHAR_ARRAY:
	case BHND_NVRAM_TYPE_BOOL_ARRAY:
		BHND_NV_ASSERT(base_size != 0, ("invalid base size"));
		*nelem = len / base_size;
		return (0);
	}

	/* Quiesce gcc4.2 */
	BHND_NV_PANIC("bhnd nvram type %u unknown", type);
}

/**
 * Return the size, in bytes, of a value of @p type with @p nelem elements.
 * 
 * @param	type	The value type.
 * @param	data	The actual data to be queried, or NULL if unknown. If
 *			NULL and the base type is not a fixed width type
 *			(e.g. BHND_NVRAM_TYPE_STRING), 0 will be returned.
 * @param	nbytes	The size of @p data, in bytes, or 0 if @p data is NULL.
 * @param	nelem	The number of elements. If @p type is not an array type,
 *			this value must be 1.
 * 
 * @retval 0		If @p type has a variable width, and @p data is NULL.
 * @retval 0		If a @p nelem value greater than 1 is provided for a
 *			non-array @p type.
 * @retval 0		If a @p nelem value of 0 is provided.
 * @retval 0		If the result would exceed the maximum value
 *			representable by size_t.
 * @retval 0		If @p type is BHND_NVRAM_TYPE_NULL.
 * @retval non-zero	The size, in bytes, of @p type with @p nelem elements.
 */
size_t
bhnd_nvram_value_size(bhnd_nvram_type type, const void *data, size_t nbytes,
    size_t nelem)
{
	/* If nelem 0, nothing to do */
	if (nelem == 0)
		return (0);

	/* Non-array types must have an nelem value of 1 */
	if (!bhnd_nvram_is_array_type(type) && nelem != 1)
		return (0);

	switch (type) {
	case BHND_NVRAM_TYPE_UINT8_ARRAY:
	case BHND_NVRAM_TYPE_UINT16_ARRAY:
	case BHND_NVRAM_TYPE_UINT32_ARRAY:
	case BHND_NVRAM_TYPE_UINT64_ARRAY:
	case BHND_NVRAM_TYPE_INT8_ARRAY:
	case BHND_NVRAM_TYPE_INT16_ARRAY:
	case BHND_NVRAM_TYPE_INT32_ARRAY:
	case BHND_NVRAM_TYPE_INT64_ARRAY:
	case BHND_NVRAM_TYPE_CHAR_ARRAY:
	case BHND_NVRAM_TYPE_BOOL_ARRAY:{
		bhnd_nvram_type	base_type;
		size_t		base_size;

		base_type = bhnd_nvram_base_type(type);
		base_size = bhnd_nvram_value_size(base_type, NULL, 0, 1);

		/* Would nelem * base_size overflow? */
		if (SIZE_MAX / nelem < base_size) {
			BHND_NV_LOG("cannot represent size %s * %zu\n",
			    bhnd_nvram_type_name(base_type), nelem);
			return (0);
		}

		return (nelem * base_size);
	}

	case BHND_NVRAM_TYPE_STRING_ARRAY: {
		const char	*p;
		size_t		 total_size;

		if (data == NULL)
			return (0);

		/* Iterate over the NUL-terminated strings to calculate
		 * total byte length */
		p = data;
		total_size = 0;
		for (size_t i = 0; i < nelem; i++) {
			size_t	elem_size;

			elem_size = strnlen(p, nbytes - total_size);
			p += elem_size;

			/* Check for (and skip) terminating NUL */
			if (total_size < nbytes && *p == '\0') {
				elem_size++;
				p++;
			}

			/* Would total_size + elem_size overflow?
			 * 
			 * A memory range larger than SIZE_MAX shouldn't be,
			 * possible, but include the check for completeness */
			if (SIZE_MAX - total_size < elem_size)
				return (0);

			total_size += elem_size;
		}

		return (total_size);
	}

	case BHND_NVRAM_TYPE_STRING: {
		size_t size;

		if (data == NULL)
			return (0);

		/* Find length */
		size = strnlen(data, nbytes);

		/* Is there a terminating NUL, or did we just hit the
		 * end of the string input */
		if (size < nbytes)
			size++;

		return (size);
	}

	case BHND_NVRAM_TYPE_NULL:
		return (0);

	case BHND_NVRAM_TYPE_BYTES:
		if (data == NULL)
			return (0);

		return (nbytes);

	case BHND_NVRAM_TYPE_BOOL:
		return (sizeof(bhnd_nvram_bool_t));

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

	case BHND_NVRAM_TYPE_UINT64:
	case BHND_NVRAM_TYPE_INT64:
		return (sizeof(uint64_t));
	}

	/* Quiesce gcc4.2 */
	BHND_NV_PANIC("bhnd nvram type %u unknown", type);
}


/**
 * Format a string representation of @p inp using @p fmt, with, writing the
 * result to @p outp.
 *
 * Refer to bhnd_nvram_val_vprintf() for full format string documentation.
 *
 * @param		fmt	The format string.
 * @param		inp	The value to be formatted.
 * @param		ilen	The size of @p inp, in bytes.
 * @param		itype	The type of @p inp.
 * @param[out]		outp	On success, the string value will be written to
 *				this buffer. This argment may be NULL if the
 *				value is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual size of the formatted string.
 *
 * @retval 0		success
 * @retval EINVAL	If @p fmt contains unrecognized format string
 *			specifiers.
 * @retval ENOMEM	If the @p outp is non-NULL, and the provided @p olen
 *			is too small to hold the encoded value.
 * @retval EFTYPE	If value coercion from @p inp to a string value via
 *			@p fmt is unsupported.
 * @retval ERANGE	If value coercion of @p value would overflow (or
 *			underflow) the representation defined by @p fmt.
 */
int
bhnd_nvram_value_printf(const char *fmt, const void *inp, size_t ilen,
    bhnd_nvram_type itype, char *outp, size_t *olen, ...)
{
	va_list	ap;
	int	error;

	va_start(ap, olen);
	error = bhnd_nvram_value_vprintf(fmt, inp, ilen, itype, outp, olen, ap);
	va_end(ap);

	return (error);
}

/**
 * Format a string representation of @p inp using @p fmt, with, writing the
 * result to @p outp.
 *
 * Refer to bhnd_nvram_val_vprintf() for full format string documentation.
 *
 * @param		fmt	The format string.
 * @param		inp	The value to be formatted.
 * @param		ilen	The size of @p inp, in bytes.
 * @param		itype	The type of @p inp.
 * @param[out]		outp	On success, the string value will be written to
 *				this buffer. This argment may be NULL if the
 *				value is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual size of the formatted string.
 * @param		ap	Argument list.
 *
 * @retval 0		success
 * @retval EINVAL	If @p fmt contains unrecognized format string
 *			specifiers.
 * @retval ENOMEM	If the @p outp is non-NULL, and the provided @p olen
 *			is too small to hold the encoded value.
 * @retval EFTYPE	If value coercion from @p inp to a string value via
 *			@p fmt is unsupported.
 * @retval ERANGE	If value coercion of @p value would overflow (or
 *			underflow) the representation defined by @p fmt.
 */
int
bhnd_nvram_value_vprintf(const char *fmt, const void *inp, size_t ilen,
    bhnd_nvram_type itype, char *outp, size_t *olen, va_list ap)
{
	bhnd_nvram_val	val;
	int		error;

	/* Map input buffer as a value instance */
	error = bhnd_nvram_val_init(&val, NULL, inp, ilen, itype,
	    BHND_NVRAM_VAL_BORROW_DATA);
	if (error)
		return (error);

	/* Attempt to format the value */
	error = bhnd_nvram_val_vprintf(&val, fmt, outp, olen, ap);

	/* Clean up */
	bhnd_nvram_val_release(&val);
	return (error);
}

/**
 * Iterate over all elements in @p inp.
 *
 * @param		inp	The value to be iterated.
 * @param		ilen	The size, in bytes, of @p inp.
 * @param		itype	The data type of @p inp.
 * @param		prev	The value previously returned by
 *				bhnd_nvram_value_array_next(), or NULL to begin
 *				iteration.
 * @param[in,out]	olen	If @p prev is non-NULL, @p olen must be a
 *				pointer to the length previously returned by
 *				bhnd_nvram_value_array_next(). On success, will
 *				be set to the next element's length, in bytes.
 *
 * @retval non-NULL	A borrowed reference to the next element of @p inp.
 * @retval NULL		If the end of the array is reached.
 */
const void *
bhnd_nvram_value_array_next(const void *inp, size_t ilen, bhnd_nvram_type itype,
    const void *prev, size_t *olen)
{
	const u_char	*next;
	size_t		 offset;

	/* Handle first element */
	if (prev == NULL) {
		/* Zero-length array? */
		if (ilen == 0)
			return (NULL);

		*olen = bhnd_nvram_value_size(itype, inp, ilen, 1);
		return (inp);
	}

	/* Advance to next element */
	BHND_NV_ASSERT(prev >= (const void *)inp, ("invalid cookiep"));
	next = (const u_char *)prev + *olen;
	offset = (size_t)(next - (const u_char *)inp);

	if (offset >= ilen) {
		/* Hit end of the array */
		return (NULL);
	}

	/* Determine element size */
	*olen = bhnd_nvram_value_size(itype, next, ilen - offset, 1);
	if (ilen - offset < *olen) {
		BHND_NV_LOG("short element of type %s -- misaligned "
		    "representation", bhnd_nvram_type_name(itype));
		return (NULL);
	}

	return (next);
}

/**
 * Coerce value @p inp of type @p itype to @p otype, writing the
 * result to @p outp.
 *
 * @param		inp	The value to be coerced.
 * @param		ilen	The size of @p inp, in bytes.
 * @param		itype	The base data type of @p inp.
 * @param[out]		outp	On success, the value will be written to this 
 *				buffer. This argment may be NULL if the value
 *				is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual size of the requested value.
 * @param		otype	The data type to be written to @p outp.
 *
 * @retval 0		success
 * @retval ENOMEM	If @p outp is non-NULL and a buffer of @p olen is too
 *			small to hold the requested value.
 * @retval EFTYPE	If the variable data cannot be coerced to @p otype.
 * @retval ERANGE	If value coercion would overflow @p otype.
 */
int
bhnd_nvram_value_coerce(const void *inp, size_t ilen, bhnd_nvram_type itype,
    void *outp, size_t *olen, bhnd_nvram_type otype)
{
	bhnd_nvram_val	val;
	int		error;

	/* Wrap input buffer in a value instance */
	error = bhnd_nvram_val_init(&val, NULL, inp, ilen,
	    itype, BHND_NVRAM_VAL_BORROW_DATA|BHND_NVRAM_VAL_FIXED);
	if (error)
		return (error);

	/* Try to encode as requested type */
	error = bhnd_nvram_val_encode(&val, outp, olen, otype);

	/* Clean up and return error */
	bhnd_nvram_val_release(&val);
	return (error);
}
