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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kobj.h>

DECLARE_CLASS(bhnd_nvram_driver);

union bhnd_nvram_ident;
struct bhnd_nvram_input;
struct bhnd_nvram_ops;
struct bhnd_nvram;

/** Supported NVRAM formats. */
typedef enum {
	BHND_NVRAM_FMT_BCM	= 0,	/**< Broadcom NUL-delimited key=value pairs */
	BHND_NVRAM_FMT_TLV	= 1,	/**< CFE TLV encoding, as used on WGT634U */
	BHND_NVRAM_FMT_BTXT	= 2,	/**< Broadcom board text file. This is used
					     to provide external NVRAM data for some
					     fullmac WiFi devices. */

} bhnd_nvram_format;

int	bhnd_nvram_probe(device_t dev);
int	bhnd_nvram_attach(device_t dev);
int	bhnd_nvram_resume(device_t dev);
int	bhnd_nvram_suspend(device_t dev);
int	bhnd_nvram_detach(device_t dev);

int	bhnd_nvram_identify(const union bhnd_nvram_ident *ident,
	    bhnd_nvram_format expected);
int	bhnd_nvram_init(struct bhnd_nvram *nvram, device_t owner,
	    const void *data, size_t len, bhnd_nvram_format fmt);
void	bhnd_nvram_fini(struct bhnd_nvram *nvam);

/** BCM NVRAM header */
struct bhnd_nvram_header {
	uint32_t magic;
	uint32_t size;
	uint32_t cfg0;		/**< crc:8, version:8, sdram_init:16 */
	uint32_t cfg1;		/**< sdram_config:16, sdram_refresh:16 */
	uint32_t memc_ncdl;	/**< sdram_ncdl */
};

/** 
 * BCM/TLV/BTXT NVRAM format identification.
 * 
 * To perform identification of the NVRAM format using bhnd_nvram_identify(),
 * read `sizeof(bhnd_nvram_indent)` bytes from the head of the NVRAM data.
 */
union bhnd_nvram_ident {
	struct bhnd_nvram_header	bcm;
	char				btxt[4];
	struct bhnd_tlv_ident {
		uint8_t		tag;
		uint8_t		size[2];
		uint8_t		flags;
	} tlv;
};

/**
 * NVRAM index record.
 * 
 * Provides a compact index into the backing NVRAM buffer.
 */
struct bhnd_nvram_idx {
	uint16_t	env_offset;	/**< offset to env string */
	uint16_t	env_len;	/**< env length */
};

/** bhnd nvram parser instance state */
struct bhnd_nvram {
	device_t			 dev;		/**< parent device, or NULL */
	const struct bhnd_nvram_ops	*ops;
	uint8_t				*buf;		/**< nvram data */
	size_t				 buf_size;

	struct bhnd_nvram_idx		*idx;		/**< sorted key index into nvram buf */
	size_t				 num_idx;	/**< number of index records */

	char				**devpaths;	/**< device path aliases */
	size_t				 num_devpaths;	/**< number of device path alias records */
};


/**
 * bhnd_nvram_cfe driver instance state. Must be first member of all subclass
 * softc structures.
 */
struct bhnd_nvram_softc {
	device_t		 	dev;
	struct mtx		 	mtx;	/**< nvram mutex */
	struct bhnd_nvram		nvram;	/**< nvram shadow */
};


#define	BHND_NVRAM_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev), \
	    "bhnd_nvram lock", MTX_DEF)
#define	BHND_NVRAM_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	BHND_NVRAM_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	BHND_NVRAM_LOCK_ASSERT(sc, what)	mtx_assert(&(sc)->mtx, what)
#define	BHND_NVRAM_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->mtx)

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
