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

#include "bhnd_nvram_types.h"
#include "bhnd_nvram_value.h"

typedef struct bhnd_nvram_prop		bhnd_nvram_prop;
typedef struct bhnd_nvram_plist		bhnd_nvram_plist;

bhnd_nvram_plist	*bhnd_nvram_plist_new(void);
bhnd_nvram_plist	*bhnd_nvram_plist_retain(bhnd_nvram_plist *plist);
void			 bhnd_nvram_plist_release(bhnd_nvram_plist *plist);

bhnd_nvram_plist	*bhnd_nvram_plist_copy(bhnd_nvram_plist *plist);

size_t			 bhnd_nvram_plist_count(bhnd_nvram_plist *plist);

int			 bhnd_nvram_plist_append_list(bhnd_nvram_plist *plist,
			     bhnd_nvram_plist *tail);

int			 bhnd_nvram_plist_append(bhnd_nvram_plist *plist,
			     bhnd_nvram_prop *prop);
int			 bhnd_nvram_plist_append_val(bhnd_nvram_plist *plist,
			     const char *name, bhnd_nvram_val *val);
int			 bhnd_nvram_plist_append_bytes(bhnd_nvram_plist *plist,
			     const char *name, const void *inp, size_t ilen,
			     bhnd_nvram_type itype);
int			 bhnd_nvram_plist_append_string(bhnd_nvram_plist *plist,
			     const char *name, const char *val);

int			 bhnd_nvram_plist_replace(bhnd_nvram_plist *plist,
			     bhnd_nvram_prop *prop);
int			 bhnd_nvram_plist_replace_val(bhnd_nvram_plist *plist,
			     const char *name, bhnd_nvram_val *val);
int			 bhnd_nvram_plist_replace_bytes(bhnd_nvram_plist *plist,
			     const char *name, const void *inp, size_t ilen,
			     bhnd_nvram_type itype);
int			 bhnd_nvram_plist_replace_string(bhnd_nvram_plist *plist,
			     const char *name, const char *val);

int			 bhnd_nvram_plist_remove(bhnd_nvram_plist *plist,
			     const char *name);

bool			 bhnd_nvram_plist_contains(bhnd_nvram_plist *plist,
			     const char *name);
bhnd_nvram_prop		*bhnd_nvram_plist_next(bhnd_nvram_plist *plist,
			     bhnd_nvram_prop *prop);

bhnd_nvram_prop		*bhnd_nvram_plist_get_prop(bhnd_nvram_plist *plist,
			     const char *name);
int			 bhnd_nvram_plist_get_encoded(bhnd_nvram_plist *plist,
			     const char *name, void *outp, size_t olen,
			     bhnd_nvram_type otype);

int			 bhnd_nvram_plist_get_char(bhnd_nvram_plist *plist,
			     const char *name, u_char *val);
int			 bhnd_nvram_plist_get_uint8(bhnd_nvram_plist *plist,
			     const char *name, uint8_t *val);
int			 bhnd_nvram_plist_get_uint16(bhnd_nvram_plist *plist,
			     const char *name, uint16_t *val);
int			 bhnd_nvram_plist_get_uint32(bhnd_nvram_plist *plist,
			     const char *name, uint32_t *val);
int			 bhnd_nvram_plist_get_uint64(bhnd_nvram_plist *plist,
			     const char *name, uint64_t *val);
int			 bhnd_nvram_plist_get_string(bhnd_nvram_plist *plist,
			     const char *name, const char **val);
int			 bhnd_nvram_plist_get_bool(bhnd_nvram_plist *plist,
			     const char *name, bool *val);

bhnd_nvram_prop		*bhnd_nvram_prop_new(const char *name,
			     bhnd_nvram_val *val);
bhnd_nvram_prop		*bhnd_nvram_prop_bytes_new(const char *name,
			     const void *inp, size_t ilen,
			     bhnd_nvram_type itype);

bhnd_nvram_prop		*bhnd_nvram_prop_retain(bhnd_nvram_prop *prop);
void			 bhnd_nvram_prop_release(bhnd_nvram_prop *prop);

const char		*bhnd_nvram_prop_name(bhnd_nvram_prop *prop);
bhnd_nvram_val		*bhnd_nvram_prop_val(bhnd_nvram_prop *prop);
bhnd_nvram_type		 bhnd_nvram_prop_type(bhnd_nvram_prop *prop);

const void		*bhnd_nvram_prop_bytes(bhnd_nvram_prop *prop,
			     size_t *olen, bhnd_nvram_type *otype);
int			 bhnd_nvram_prop_encode(bhnd_nvram_prop *prop,
			     void *outp, size_t *olen, bhnd_nvram_type otype);

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

const char	*bhnd_nvram_string_array_next(const char *inp, size_t ilen,
		     const char *prev, size_t *olen); 

#endif /* _BHND_NVRAM_BHND_NVRAM_H_ */
