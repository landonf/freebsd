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

#include <sys/endian.h>

#ifdef _KERNEL
#include <sys/param.h>
#include <sys/ctype.h>
#include <sys/malloc.h>
#include <sys/systm.h>

#include <machine/_inttypes.h>
#else /* !_KERNEL */
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif /* _KERNEL */

#include "bhnd_nvram_map.h"

#include "bhnd_nvram_private.h"
#include "bhnd_nvram_datavar.h"

#include "bhnd_nvram_data_spromvar.h"

/*
 * BHND SPROM NVRAM data class
 *
 * The SPROM data format is a fixed-layout, non-self-descriptive binary format,
 * used on Broadcom wireless and wired adapters, that provides a subset of the
 * variables defined by Broadcom SoC NVRAM formats.
 */
BHND_NVRAM_DATA_CLASS_DEFN(sprom, "Broadcom SPROM",
    BHND_NVRAM_DATA_CAP_DEVPATHS, sizeof(struct bhnd_nvram_sprom))

#define	SPROM_COOKIE_TO_VID(_cookie)	\
	(((struct bhnd_sprom_opcode_idx_entry *)(_cookie))->vid)

#define	SPROM_COOKIE_TO_NVRAM_VAR(_cookie)	\
	bhnd_nvram_get_vardefn(SPROM_COOKIE_TO_VID(_cookie))

/**
 * Read the magic value from @p io, and verify that it matches
 * the @p layout's expected magic value.
 * 
 * If @p layout does not defined a magic value, @p magic is set to 0x0
 * and success is returned.
 * 
 * @param	io	An I/O context mapping the SPROM data to be identified.
 * @param	layout	The SPROM layout against which @p io should be verified.
 * @param[out]	magic	On success, the SPROM magic value.
 * 
 * @retval 0		success
 * @retval non-zero	If checking @p io otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvram_sprom_check_magic(struct bhnd_nvram_io *io,
    const bhnd_sprom_layout *layout, uint16_t *magic)
{
	int error;

	/* Skip if layout does not define a magic value */
	if (layout->flags & SPROM_LAYOUT_MAGIC_NONE)
		return (0);

	/* Read the magic value */
	error = bhnd_nvram_io_read(io, layout->magic_offset, magic,
	    sizeof(*magic));
	if (error)
		return (error);

	*magic = le16toh(*magic);

	/* If the signature does not match, skip to next layout */
	if (*magic != layout->magic_value)
		return (ENXIO);

	return (0);
}

/**
 * Attempt to identify the format of the SPROM data mapped by @p io.
 *
 * The SPROM data format does not provide any identifying information at a
 * known offset, instead requiring that we iterate over the known SPROM image
 * sizes until we are able to compute a valid checksum (and, for later
 * revisions, validate a signature at a revision-specific offset).
 *
 * @param	io	An I/O context mapping the SPROM data to be identified.
 * @param[out]	ident	On success, the identified SPROM layout.
 * @param[out]	shadow	On success, a correctly sized iobuf instance mapping
 *			a copy of the identified SPROM image. The caller is
 *			responsible for deallocating this instance via
 *			bhnd_nvram_io_free()
 *
 * @retval 0		success
 * @retval non-zero	If identifying @p io otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvram_sprom_ident(struct bhnd_nvram_io *io,
    const bhnd_sprom_layout **ident, struct bhnd_nvram_io **shadow)
{
	struct bhnd_nvram_io	*buf;
	uint8_t			 crc;
	size_t			 crc_errors;
	size_t			 sprom_sz_max;
	int			 error;

	/* Find the largest SPROM layout size */
	sprom_sz_max = 0;
	for (size_t i = 0; i < bhnd_sprom_num_layouts; i++) {
		sprom_sz_max = bhnd_nv_ummax(sprom_sz_max,
		    bhnd_sprom_layouts[i].size);
	}

	/* Allocate backing buffer and initialize CRC state */
	buf = bhnd_nvram_iobuf_empty(0, sprom_sz_max);
	crc = BHND_NVRAM_CRC8_INITIAL;
	crc_errors = 0;

	/* We iterate the SPROM layouts smallest to largest, allowing us to
	 * perform incremental checksum calculation */
	for (size_t i = 0; i < bhnd_sprom_num_layouts; i++) {
		const bhnd_sprom_layout	*layout;
		void			*ptr;
		size_t			 nbytes, nr;
		uint16_t		 magic;
		uint8_t			 srev;
		bool			 crc_valid;
		bool			 have_magic;

		layout = &bhnd_sprom_layouts[i];
		nbytes = bhnd_nvram_io_getsize(buf);

		if ((layout->flags & SPROM_LAYOUT_MAGIC_NONE)) {
			have_magic = false;
		} else {
			have_magic = true;
		}

		/* Layout instances must be ordered from smallest to largest by
		 * the nvram_map compiler */
		if (nbytes > layout->size)
			BHND_NV_PANIC("SPROM layout is defined out-of-order");

		/* Calculate number of additional bytes to be read */
		nr = layout->size - nbytes;

		/* Adjust the buffer size and fetch a write pointer */
		if ((error = bhnd_nvram_io_setsize(buf, layout->size)))
			goto failed;

		error = bhnd_nvram_io_write_ptr(buf, nbytes, &ptr, nr, NULL);
		if (error)
			goto failed;

		/* Read image data and update CRC (errors are reported
		 * after the signature check) */
		if ((error = bhnd_nvram_io_read(io, nbytes, ptr, nr)))
			goto failed;

		crc = bhnd_nvram_crc8(ptr, nr, crc);
		crc_valid = (crc == BHND_NVRAM_CRC8_VALID);
		if (!crc_valid)
			crc_errors++;

		/* Fetch SPROM revision */
		error = bhnd_nvram_io_read(buf, layout->srev_offset, &srev,
		    sizeof(srev));
		if (error)
			goto failed;

		/* Early sromrev 1 devices (specifically some BCM440x enet
		 * cards) are reported to have been incorrectly programmed
		 * with a revision of 0x10. */
		if (layout->rev == 1 && srev == 0x10)
			srev = 0x1;
		
		/* Check revision against the layout definition */
		if (srev != layout->rev)
			continue;

		/* Check the magic value, skipping to the next layout on
		 * failure. */
		error = bhnd_nvram_sprom_check_magic(buf, layout, &magic);
		if (error) {
			/* If the CRC is was valid, log the mismatch */
			if (crc_valid || BHND_NV_VERBOSE) {
				BHND_NV_LOG("invalid sprom %hhu signature: "
					    "0x%hx (expected 0x%hx)\n", srev,
					    magic, layout->magic_value);

					error = ENXIO;
					goto failed;
			}
	
			continue;
		}

		/* Check for an earlier CRC error */
		if (!crc_valid) {
			/* If the magic check succeeded, then we may just have
			 * data corruption -- log the CRC error */
			if (have_magic || BHND_NV_VERBOSE) {
				BHND_NV_LOG("sprom %hhu CRC error (crc=%#hhx, "
					    "expected=%#x)\n", srev, crc,
					    BHND_NVRAM_CRC8_VALID);
			}

			continue;
		}

		/* Identified */
		*shadow = buf;
		*ident = layout;
		return (0);
	}

	/* No match -- set error and fallthrough */
	error = ENXIO;
	if (crc_errors > 0 && BHND_NV_VERBOSE) {
		BHND_NV_LOG("sprom parsing failed with %zu CRC errors\n",
		    crc_errors);
	}

failed:
	bhnd_nvram_io_free(buf);
	return (error);
}

static int
bhnd_nvram_sprom_probe(struct bhnd_nvram_io *io)
{
	const bhnd_sprom_layout	*layout;
	struct bhnd_nvram_io	*shadow;
	int			 error;

	/* Try to parse the input */
	if ((error = bhnd_nvram_sprom_ident(io, &layout, &shadow)))
		return (error);

	/* Clean up the shadow iobuf */
	bhnd_nvram_io_free(shadow);

	return (BHND_NVRAM_DATA_PROBE_DEFAULT);
}

static int
bhnd_nvram_sprom_class_serialize(bhnd_nvram_data_class *cls,
    bhnd_nvram_plist *props, void *outp, size_t *olen)
{
	bhnd_nvram_prop		*prop;
	const bhnd_sprom_layout	*layout;
	uint8_t			 sromrev;
	size_t			 limit, proplen;
	int			 error;

	limit = *olen;
	layout = NULL;

	/* Fetch sromrev property */
	if ((prop = bhnd_nvram_plist_get(props, BHND_NVAR_SROMREV)) == NULL) {
		BHND_NV_LOG("missing required sromrev property\n");
		return (ENOENT);
	}

	proplen = sizeof(sromrev);
	error = bhnd_nvram_prop_encode(prop, &sromrev, &proplen,
	    BHND_NVRAM_TYPE_UINT8);
	if (error) {
		BHND_NV_LOG("error reading sromrev property: %d\n", error);
		return (EFTYPE);
	}

	/* Find matching SPROM layout definition */
	for (size_t i = 0; i < bhnd_sprom_num_layouts; i++) {
		if (bhnd_sprom_layouts[i].rev == sromrev) {
			layout = &bhnd_sprom_layouts[i];
			break;
		}
	}

	if (layout == NULL) {
		BHND_NV_LOG("unsupported sromrev: %hhu\n", sromrev);
		return (EFTYPE);
	}

	/* Provide required size to caller */
	*olen = layout->size;
	if (outp == NULL)
		return (0);
	else if (limit < *olen)
		return (ENOMEM);

	// XXX TODO
	return (ENXIO);
}

static int
bhnd_nvram_sprom_new(struct bhnd_nvram_data *nv, struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_sprom	*sp;
	int			 error;

	sp = (struct bhnd_nvram_sprom *)nv;

	/* Identify the SPROM input data */
	if ((error = bhnd_nvram_sprom_ident(io, &sp->layout, &sp->data)))
		goto failed;

	/* Initialize SPROM binding eval state */
	if ((error = bhnd_sprom_opcode_init(&sp->state, sp->layout)))
		goto failed;

	return (0);

failed:
	if (sp->data != NULL)
		bhnd_nvram_io_free(sp->data);

	return (error);
}

static void
bhnd_nvram_sprom_free(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_sprom *sp = (struct bhnd_nvram_sprom *)nv;

	bhnd_sprom_opcode_fini(&sp->state);
	bhnd_nvram_io_free(sp->data);
}

size_t
bhnd_nvram_sprom_count(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_sprom *sprom = (struct bhnd_nvram_sprom *)nv;
	return (sprom->layout->num_vars);
}

static int
bhnd_nvram_sprom_size(struct bhnd_nvram_data *nv, size_t *size)
{
	struct bhnd_nvram_sprom *sprom = (struct bhnd_nvram_sprom *)nv;

	/* The serialized form will be identical in length
	 * to our backing buffer representation */
	*size = bhnd_nvram_io_getsize(sprom->data);
	return (0);
}

static int
bhnd_nvram_sprom_serialize(struct bhnd_nvram_data *nv, void *buf, size_t *len)
{
	struct bhnd_nvram_sprom	*sprom;
	size_t			 limit, req_len;
	int			 error;

	sprom = (struct bhnd_nvram_sprom *)nv;
	limit = *len;

	/* Provide the required size */
	if ((error = bhnd_nvram_sprom_size(nv, &req_len)))
		return (error);

	*len = req_len;

	if (buf == NULL) {
		return (0);
	} else if (*len > limit) {
		return (ENOMEM);
	}

	/* Write to the output buffer */
	return (bhnd_nvram_io_read(sprom->data, 0x0, buf, *len));
}

static uint32_t
bhnd_nvram_sprom_caps(struct bhnd_nvram_data *nv)
{
	return (BHND_NVRAM_DATA_CAP_INDEXED);
}

static const char *
bhnd_nvram_sprom_next(struct bhnd_nvram_data *nv, void **cookiep)
{
	struct bhnd_nvram_sprom		*sp;
	bhnd_sprom_opcode_idx_entry	*entry;
	const struct bhnd_nvram_vardefn	*var;

	sp = (struct bhnd_nvram_sprom *)nv;

	/* Find next index entry that is not disabled by virtue of IGNALL1 */
	entry = *cookiep;
	while ((entry = bhnd_sprom_opcode_index_next(&sp->state, entry))) {
		/* Update cookiep and fetch variable definition */
		*cookiep = entry;
		var = SPROM_COOKIE_TO_NVRAM_VAR(*cookiep);

		/* We might need to parse the variable's value to determine
		 * whether it should be treated as unset */
		if (var->flags & BHND_NVRAM_VF_IGNALL1) {
			int     error;
			size_t  len;

			error = bhnd_nvram_sprom_getvar(nv, *cookiep, NULL,
			    &len, var->type);
			if (error) {
				BHND_NV_ASSERT(error == ENOENT, ("unexpected "
				    "error parsing variable: %d", error));

				continue;
			}
		}

		/* Found! */
		return (var->name);
	}

	/* Reached end of index entries */
	return (NULL);
}



static void *
bhnd_nvram_sprom_find(struct bhnd_nvram_data *nv, const char *name)
{
	struct bhnd_nvram_sprom		*sp;
	bhnd_sprom_opcode_idx_entry	*entry;

	sp = (struct bhnd_nvram_sprom *)nv;

	entry = bhnd_sprom_opcode_index_find(&sp->state, name);
	return (entry);
}

/**
 * Read the value of @p type from the SPROM data at @p offset, apply @p mask
 * and @p shift, and OR with the existing @p value.
 * 
 * @param sp The SPROM data instance.
 * @param var The NVRAM variable definition
 * @param type The type to read at @p offset
 * @param offset The data offset to be read.
 * @param mask The mask to be applied to the value read at @p offset.
 * @param shift The shift to be applied after masking; if positive, a right
 * shift will be applied, if negative, a left shift.
 * @param value The read destination; the parsed value will be OR'd with the
 * current contents of @p value.
 */
static int
bhnd_nvram_sprom_read_offset(struct bhnd_nvram_sprom *sp,
    const struct bhnd_nvram_vardefn *var, bhnd_nvram_type type,
    size_t offset, uint32_t mask, int8_t shift,
    union bhnd_nvram_sprom_intv *value)
{
	size_t	sp_width;
	int	error;
	union {
		uint8_t		u8;
		uint16_t	u16;
		uint32_t	u32;
		int8_t		s8;
		int16_t		s16;
		int32_t		s32;
	} sp_value;

	/* Determine type width */
	sp_width = bhnd_nvram_value_size(type, NULL, 0, 1);
	if (sp_width == 0) {
		/* Variable-width types are unsupported */
		BHND_NV_LOG("invalid %s SPROM offset type %d\n", var->name,
		    type);
		return (EFTYPE);
	}

	/* Perform read */
	error = bhnd_nvram_io_read(sp->data, offset, &sp_value,
	    sp_width);
	if (error) {
		BHND_NV_LOG("error reading %s SPROM offset %#zx: %d\n",
		    var->name, offset, error);
		return (EFTYPE);
	}

#define	NV_PARSE_INT(_type, _src, _dest, _swap)	do {			\
	/* Swap to host byte order */					\
	sp_value. _src = (_type) _swap(sp_value. _src);			\
									\
	/* Mask and shift the value */					\
	sp_value. _src &= mask;				\
	if (shift > 0) {					\
		sp_value. _src >>= shift;			\
	} else if (shift < 0) {				\
		sp_value. _src <<= -shift;			\
	}								\
									\
	/* Emit output, widening to 32-bit representation  */		\
	value-> _dest |= sp_value. _src;				\
} while(0)

	/* Apply mask/shift and widen to a common 32bit representation */
	switch (type) {
	case BHND_NVRAM_TYPE_UINT8:
		NV_PARSE_INT(uint8_t,	u8,	u32,	);
		break;
	case BHND_NVRAM_TYPE_UINT16:
		NV_PARSE_INT(uint16_t,	u16,	u32,	le16toh);
		break;
	case BHND_NVRAM_TYPE_UINT32:
		NV_PARSE_INT(uint32_t,	u32,	u32,	le32toh);
		break;
	case BHND_NVRAM_TYPE_INT8:
		NV_PARSE_INT(int8_t,	s8,	s32,	);
		break;
	case BHND_NVRAM_TYPE_INT16:
		NV_PARSE_INT(int16_t,	s16,	s32,	le16toh);
		break;
	case BHND_NVRAM_TYPE_INT32:
		NV_PARSE_INT(int32_t,	s32,	s32,	le32toh);
		break;
	case BHND_NVRAM_TYPE_CHAR:
		NV_PARSE_INT(uint8_t,	u8,	u32,	);
		break;

	case BHND_NVRAM_TYPE_DATA:
	case BHND_NVRAM_TYPE_NULL:
	case BHND_NVRAM_TYPE_UINT64:
	case BHND_NVRAM_TYPE_INT64:
	case BHND_NVRAM_TYPE_STRING:
		/* fallthrough (unused by SPROM) */
	default:
		BHND_NV_LOG("unhandled %s offset type: %d\n", var->name, type);
		return (EFTYPE);
	}

	return (0);
}

/**
 * Common variable decoding; fetches and decodes variable to @p val,
 * using @p storage for actual data storage.
 * 
 * The returned @p val instance will hold a borrowed reference to @p storage,
 * and must be copied via bhnd_nvram_val_copy() if it will be referenced beyond
 * the lifetime of @p storage.
 *
 * The caller is responsible for releasing any allocated value state
 * via bhnd_nvram_val_free().
 */
static int
bhnd_nvram_sprom_getvar_common(struct bhnd_nvram_data *nv, void *cookiep,
    union bhnd_nvram_sprom_storage *storage, bhnd_nvram_val *val)
{
	struct bhnd_nvram_sprom		*sp;
	bhnd_sprom_opcode_idx_entry	*entry;
	const struct bhnd_nvram_vardefn	*var;
	union bhnd_nvram_sprom_storage	*inp;
	union bhnd_nvram_sprom_intv	 intv;
	bhnd_nvram_type			 var_btype;
	size_t				 ilen, ipos, iwidth;
	size_t				 nelem;
	bool				 all_bits_set;
	int				 error;

	sp = (struct bhnd_nvram_sprom *)nv;
	entry = cookiep;

	BHND_NV_ASSERT(cookiep != NULL, ("NULL variable cookiep"));

	/* Fetch canonical variable definition */
	var = SPROM_COOKIE_TO_NVRAM_VAR(cookiep);
	BHND_NV_ASSERT(var != NULL, ("invalid cookiep %p", cookiep));

	/*
	 * Fetch the array length from the SPROM variable definition.
	 *
	 * This generally be identical to the array length provided by the
	 * canonical NVRAM variable definition, but some SPROM layouts may
	 * define a smaller element count.
	 */
	if ((error = bhnd_sprom_opcode_parse_var(&sp->state, entry))) {
		BHND_NV_LOG("variable evaluation failed: %d\n", error);
		return (error);
	}

	nelem = sp->state.var.nelem;
	if (nelem > var->nelem) {
		BHND_NV_LOG("SPROM array element count %zu cannot be "
		    "represented by '%s' element count of %hhu\n", nelem,
		    var->name, var->nelem);
		return (EFTYPE);
	}

	/* Fetch the var's base element type */
	var_btype = bhnd_nvram_base_type(var->type);

	/* Calculate total byte length of the native encoding */
	if ((iwidth = bhnd_nvram_value_size(var_btype, NULL, 0, 1)) == 0) {
		/* SPROM does not use (and we do not support) decoding of
		 * variable-width data types */
		BHND_NV_LOG("invalid SPROM data type: %d", var->type);
		return (EFTYPE);
	}
	ilen = nelem * iwidth;

	/* Decode into our caller's local storage */
	inp = storage;
	if (ilen > sizeof(*storage)) {
		BHND_NV_LOG("error decoding '%s', SPROM_ARRAY_MAXLEN "
		    "incorrect\n", var->name);
		return (EFTYPE);
	}

	/* Zero-initialize our decode buffer; any output elements skipped
	 * during decode should default to zero. */
	memset(inp, 0, ilen);

	/*
	 * Decode the SPROM data, iteratively decoding up to nelem values.
	 */
	if ((error = bhnd_sprom_opcode_seek(&sp->state, entry))) {
		BHND_NV_LOG("variable seek failed: %d\n", error);
		return (error);
	}

	ipos = 0;
	intv.u32 = 0x0;
	if (var->flags & BHND_NVRAM_VF_IGNALL1)
		all_bits_set = true;
	else
		all_bits_set = false;
	while ((error = bhnd_sprom_opcode_next_binding(&sp->state)) == 0) {
		bhnd_sprom_opcode_bind	*binding;
		bhnd_sprom_opcode_var	*binding_var;
		bhnd_nvram_type		 intv_type;
		size_t			 offset;
		size_t			 nbyte;
		uint32_t		 skip_in_bytes;
		void			*ptr;

		BHND_NV_ASSERT(
		    sp->state.var_state >= SPROM_OPCODE_VAR_STATE_OPEN,
		    ("invalid var state"));
		BHND_NV_ASSERT(sp->state.var.have_bind, ("invalid bind state"));

		binding_var = &sp->state.var;
		binding = &sp->state.var.bind;

		if (ipos >= nelem) {
			BHND_NV_LOG("output skip %u positioned "
			    "%zu beyond nelem %zu\n",
			    binding->skip_out, ipos, nelem);
			return (EINVAL);
		}

		/* Calculate input skip bytes for this binding */
		skip_in_bytes = binding->skip_in;
		error = bhnd_sprom_opcode_apply_scale(&sp->state,
		    &skip_in_bytes);
		if (error)
			return (error);

		/* Bind */
		offset = sp->state.offset;
		for (size_t i = 0; i < binding->count; i++) {
			/* Read the offset value, OR'ing with the current
			 * value of intv */
			error = bhnd_nvram_sprom_read_offset(sp, var,
			    binding_var->base_type,
			    offset,
			    binding_var->mask,
			    binding_var->shift,
			    &intv);
			if (error)
				return (error);

			/* If IGNALL1, record whether value does not have
			 * all bits set. */
			if (var->flags & BHND_NVRAM_VF_IGNALL1 &&
			    all_bits_set)
			{
				uint32_t all1;

				all1 = binding_var->mask;
				if (binding_var->shift > 0)
					all1 >>= binding_var->shift;
				else if (binding_var->shift < 0)
					all1 <<= -binding_var->shift;

				if ((intv.u32 & all1) != all1)
					all_bits_set = false;
			}

			/* Adjust input position; this was already verified to
			 * not overflow/underflow during SPROM opcode
			 * evaluation */
			if (binding->skip_in_negative) {
				offset -= skip_in_bytes;
			} else {
				offset += skip_in_bytes;
			}

			/* Skip writing to inp if additional bindings are
			 * required to fully populate intv */
			if (binding->skip_out == 0)
				continue;

			/* We use bhnd_nvram_value_coerce() to perform
			 * overflow-checked coercion from the widened
			 * uint32/int32 intv value to the requested output
			 * type */
			if (bhnd_nvram_is_signed_type(var_btype))
				intv_type = BHND_NVRAM_TYPE_INT32;
			else
				intv_type = BHND_NVRAM_TYPE_UINT32;

			/* Calculate address of the current element output
			 * position */
			ptr = (uint8_t *)inp + (iwidth * ipos);

			/* Perform coercion of the array element */
			nbyte = iwidth;
			error = bhnd_nvram_value_coerce(&intv, sizeof(intv),
			    intv_type, ptr, &nbyte, var_btype);
			if (error)
				return (error);

			/* Clear temporary state */
			intv.u32 = 0x0;

			/* Advance output position */
			if (SIZE_MAX - binding->skip_out < ipos) {
				BHND_NV_LOG("output skip %u would overflow "
				    "%zu\n", binding->skip_out, ipos);
				return (EINVAL);
			}

			ipos += binding->skip_out;
		}
	}

	/* Did we iterate all bindings until hitting end of the variable
	 * definition? */
	BHND_NV_ASSERT(error != 0, ("loop terminated early"));
	if (error != ENOENT) {
		return (error);
	}

	/* If marked IGNALL1 and all bits are set, treat variable as
	 * unavailable */
	if ((var->flags & BHND_NVRAM_VF_IGNALL1) && all_bits_set)
		return (ENOENT);

	/* Provide value wrapper */
	return (bhnd_nvram_val_init(val, var->fmt, inp, ilen, var->type,
	    BHND_NVRAM_VAL_BORROW_DATA));
		return (error);
}

static int
bhnd_nvram_sprom_getvar(struct bhnd_nvram_data *nv, void *cookiep, void *buf,
    size_t *len, bhnd_nvram_type otype)
{
	bhnd_nvram_val			val;
	union bhnd_nvram_sprom_storage	storage;
	int				error;

	/* Decode variable to a new value instance */
	error = bhnd_nvram_sprom_getvar_common(nv, cookiep, &storage, &val);
	if (error)
		return (error);

	/* Perform value coercion */
	error = bhnd_nvram_val_encode(&val, buf, len, otype);

	/* Clean up */
	bhnd_nvram_val_release(&val);
	return (error);
}

static int
bhnd_nvram_sprom_getval(struct bhnd_nvram_data *nv, void *cookiep,
    bhnd_nvram_val **value)
{
	bhnd_nvram_val			val;
	union bhnd_nvram_sprom_storage	storage;
	int				error;

	/* Decode variable to a new value instance */
	error = bhnd_nvram_sprom_getvar_common(nv, cookiep, &storage, &val);
	if (error)
		return (error);

	/* Attempt to copy to heap */
	*value = bhnd_nvram_val_copy(&val);
	bhnd_nvram_val_release(&val);

	if (*value == NULL)
		return (ENOMEM);

	return (0);
}

static const void *
bhnd_nvram_sprom_getvar_ptr(struct bhnd_nvram_data *nv, void *cookiep,
    size_t *len, bhnd_nvram_type *type)
{
	/* Unsupported */
	return (NULL);
}

static const char *
bhnd_nvram_sprom_getvar_name(struct bhnd_nvram_data *nv, void *cookiep)
{
	const struct bhnd_nvram_vardefn	*var;

	BHND_NV_ASSERT(cookiep != NULL, ("NULL variable cookiep"));

	var = SPROM_COOKIE_TO_NVRAM_VAR(cookiep);
	BHND_NV_ASSERT(var != NULL, ("invalid cookiep %p", cookiep));

	return (var->name);
}
