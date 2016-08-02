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

#ifndef _BHND_NVRAM_BHND_NVRAMVAR_H_
#define _BHND_NVRAM_BHND_NVRAMVAR_H_

#include <sys/types.h>

#define	BHND_NVRAM_KEY_MAX	64	/** maximum key length (not incl. NUL) */
#define	BHND_NVRAM_VAL_MAX	255	/** maximum value length (not incl. NUL) */

/**
 * NVRAM header
 */
struct bhnd_nvram_header {
	uint32_t magic;
	uint32_t size;
	uint32_t cfg0;		/**< crc:8, version:8, sdram_init:16 */
	uint32_t cfg1;		/**< sdram_config:16, sdram_refresh:16 */
	uint32_t memc_ncdl;	/**< sdram_ncdl */
};

/**
 * NVRAM magic or TLV identification
 */
union bhnd_nvram_ident {
	struct bhnd_nvram_header	bcm;
	struct bhnd_tlv_ident {
		uint8_t		tag;
		uint8_t		size[2];
		uint8_t		flags;
	} tlv;
};

/**
 * Supported NVRAM encodings.
 */
typedef enum {
	BHND_NVRAM_FMT_BCM	= 0,	/**< Broadcom NUL-delimited key=value pairs */
	BHND_NVRAM_FMT_TLV	= 1,	/**< CFE TLV encoding, as used on WGT634U */
} bhnd_nvram_format;

/**
 * bhnd nvram parser instance state.
 */
struct bhnd_nvram {
	device_t			dev;	/**< nvram device */
	bhnd_nvram_format		fmt;	/**< nvram format */
	struct bhnd_nvram_header	header;	/**< header (if BHND_NVRAM_FMT_BCM) */
};

int		bhnd_nvram_identify(union bhnd_nvram_ident *ident,
		    bhnd_nvram_format expected);

int		bhnd_nvram_init(struct bhnd_nvram *nv, const void *input,
		    size_t len);
void		bhnd_nvram_fini(struct bhnd_nvram *nv);

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
