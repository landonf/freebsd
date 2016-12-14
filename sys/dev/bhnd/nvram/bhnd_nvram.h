/*-
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
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

#ifndef _BHND_NVRAM_BHND_NVRAM_H_
#define _BHND_NVRAM_BHND_NVRAM_H_

#ifdef _KERNEL
#include <sys/types.h>
#else /* !_KERNEL */
#include <stdbool.h>
#include <stdint.h>
#endif /* _KERNEL */

/**
 * BHND NVRAM boolean type; guaranteed to be exactly 8-bits, representing
 * true as integer constant 1, and false as integer constant 0.
 * 
 * Compatible with stdbool constants (true, false).
 */
typedef uint8_t	bhnd_nvram_bool_t;

/**
 * NVRAM data sources supported by bhnd(4) devices.
 */
typedef enum {
	BHND_NVRAM_SRC_OTP,	/**< On-chip one-time-programmable
				  *  memory. */

	BHND_NVRAM_SRC_FLASH,	/**< External flash */
	BHND_NVRAM_SRC_SPROM,	/**< External serial EEPROM. */
	
	BHND_NVRAM_SRC_UNKNOWN	/**< No NVRAM source is directly
				  *  attached.
				  *
				  *  This will be returned by ChipCommon
				  *  revisions (rev <= 31) used in early
				  *  chipsets that vend SPROM/OTP via the
				  *  native host bridge interface.
				  *
				  *  For example, PCMCIA cards may vend
				  *  Broadcom NVRAM data via their standard CIS
				  *  table, and earlier PCI(e) devices map
				  *  SPROM statically into PCI BARs, and the
				  *  control registers into PCI config space.
				  
				  *  This will also be returned on later
				  *  devices that are attached via PCI(e) to
				  *  BHND SoCs, but do not include an attached
				  *  SPROM, or programmed OTP. On such SoCs,
				  *  NVRAM configuration for individual devices
				  *  is provided by a common platform NVRAM
				  *  device.
				  */
} bhnd_nvram_src;

/**
 * NVRAM data types.
 * 
 * @internal
 * 
 * All primitive (non-array) constants should be representable as a 4-bit
 * integer (e.g. 0-15) to support SPROM_OPCODE_TYPE_IMM encoding as used by
 * nvram_map_gen.awk.
 */
typedef enum {
	BHND_NVRAM_TYPE_UINT8		= 0,	/**< unsigned 8-bit integer */
	BHND_NVRAM_TYPE_UINT16		= 1,	/**< unsigned 16-bit integer */
	BHND_NVRAM_TYPE_UINT32		= 2,	/**< unsigned 32-bit integer */
	BHND_NVRAM_TYPE_UINT64		= 3,	/**< signed 64-bit integer */
	BHND_NVRAM_TYPE_INT8		= 4,	/**< signed 8-bit integer */
	BHND_NVRAM_TYPE_INT16		= 5,	/**< signed 16-bit integer */
	BHND_NVRAM_TYPE_INT32		= 6,	/**< signed 32-bit integer */
	BHND_NVRAM_TYPE_INT64		= 7,	/**< signed 64-bit integer */
	BHND_NVRAM_TYPE_CHAR		= 8,	/**< ASCII/UTF-8 character */
	BHND_NVRAM_TYPE_STRING		= 9,	/**< ASCII/UTF-8 NUL-terminated
						     string */
	BHND_NVRAM_TYPE_BOOL		= 10,	/**< uint8 boolean value. see
						     bhnd_nvram_bool_t. */
	BHND_NVRAM_TYPE_NULL		= 11,	/**< NULL (empty) value */
	BHND_NVRAM_TYPE_DATA		= 12,	/**< opaque octet string */

	/* 10-15 reserved for primitive (non-array) types */

	BHND_NVRAM_TYPE_UINT8_ARRAY	= 16,	/**< array of uint8 integers */
	BHND_NVRAM_TYPE_UINT16_ARRAY	= 17,	/**< array of uint16 integers */
	BHND_NVRAM_TYPE_UINT32_ARRAY	= 18,	/**< array of uint32 integers */
	BHND_NVRAM_TYPE_UINT64_ARRAY	= 19,	/**< array of uint64 integers */
	BHND_NVRAM_TYPE_INT8_ARRAY	= 20,	/**< array of int8 integers */
	BHND_NVRAM_TYPE_INT16_ARRAY	= 21,	/**< array of int16 integers */
	BHND_NVRAM_TYPE_INT32_ARRAY	= 22,	/**< array of int32 integers */
	BHND_NVRAM_TYPE_INT64_ARRAY	= 23,	/**< array of int64 integers */
	BHND_NVRAM_TYPE_CHAR_ARRAY	= 24,	/**< array of ASCII/UTF-8
						     characters */
	BHND_NVRAM_TYPE_STRING_ARRAY	= 25,	/**< array of ASCII/UTF-8
						     NUL-terminated strings */
	BHND_NVRAM_TYPE_BOOL_ARRAY	= 26,	/**< array of uint8 boolean
						     values */
} bhnd_nvram_type;


bool			 bhnd_nvram_is_signed_type(bhnd_nvram_type type);
bool			 bhnd_nvram_is_unsigned_type(bhnd_nvram_type type);
bool			 bhnd_nvram_is_int_type(bhnd_nvram_type type);
bool			 bhnd_nvram_is_array_type(bhnd_nvram_type type);
bhnd_nvram_type		 bhnd_nvram_base_type(bhnd_nvram_type type);
bhnd_nvram_type		 bhnd_nvram_raw_type(bhnd_nvram_type type);
const char		*bhnd_nvram_type_name(bhnd_nvram_type type);
size_t			 bhnd_nvram_type_width(bhnd_nvram_type type);
size_t			 bhnd_nvram_type_host_align(bhnd_nvram_type type);

const char		*bhnd_nvram_string_array_next(const char *inp,
			     size_t ilen, const char *prev, size_t *olen); 

#ifdef _KERNEL

/* forward declarations */
struct bhnd_nvram_plane;
struct bhnd_nvram_prov;
struct bhnd_nvram_plist;

typedef struct bhnd_nvram_phandle bhnd_nvram_phandle;


struct bhnd_nvram_plane	*bhnd_nvram_plane_new(struct bhnd_nvram_plane *parent);
struct bhnd_nvram_plane	*bhnd_nvram_plane_retain(
			     struct bhnd_nvram_plane *plane);
void			 bhnd_nvram_plane_release(
			     struct bhnd_nvram_plane *plane);
int			 bhnd_nvram_plane_register(
			     struct bhnd_nvram_plane *plane,
			     struct bhnd_nvram_prov *prov);
int			 bhnd_nvram_plane_deregister(
			     struct bhnd_nvram_plane *plane,
			     struct bhnd_nvram_prov *prov);
int			 bhnd_nvram_plane_register_path(
			     struct bhnd_nvram_plane *plane,
			     struct bhnd_nvram_prov *prov, const char *path);
int			 bhnd_nvram_plane_deregister_path(
			     struct bhnd_nvram_plane *plane,
			     struct bhnd_nvram_prov *prov, const char *path);

bhnd_nvram_phandle	*bhnd_nvram_plane_open_root(
			     struct bhnd_nvram_plane *plane);
bhnd_nvram_phandle	*bhnd_nvram_plane_open_path(
			     struct bhnd_nvram_plane *plane, const char *path);
bhnd_nvram_phandle	*bhnd_nvram_plane_find_path(
			     struct bhnd_nvram_plane *plane, const char *path);

bhnd_nvram_phandle	*bhnd_nvram_plane_open_parent(
			     bhnd_nvram_phandle *phandle);
bhnd_nvram_phandle	*bhnd_nvram_plane_retain_path(
			     bhnd_nvram_phandle *phandle);
void			 bhnd_nvram_plane_close_path(
			     bhnd_nvram_phandle *phandle);
bhnd_nvram_phandle	*bhnd_nvram_plane_findprop_path(
			     bhnd_nvram_phandle *phandle, const char *propname);

int			 bhnd_nvram_plane_setprop(bhnd_nvram_phandle *phandle,
			     const char *propname, const void *buf, size_t len,
			     bhnd_nvram_type type);
int			 bhnd_nvram_plane_getprop(bhnd_nvram_phandle *phandle,
			     const char *propname, void *buf, size_t *len,
			     bhnd_nvram_type type);

int			 bhnd_nvram_plane_getprop_alloc(
			     bhnd_nvram_phandle *phandle, const char *propname,
			     void **buf, size_t *len, bhnd_nvram_type type,
			     int flags);
void			 bhnd_nvram_plane_getprop_free(void *buf);

struct bhnd_nvram_plist	*bhnd_nvram_plane_getprops_copy(
			     bhnd_nvram_phandle *phandle);

#endif /* _KERNEL */


#endif /* _BHND_NVRAM_BHND_NVRAM_H_ */
