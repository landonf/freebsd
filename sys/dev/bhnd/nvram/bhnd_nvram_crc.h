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

#ifndef _BHND_NVRAM_BHND_NVRAM_CRC_H_
#define _BHND_NVRAM_BHND_NVRAM_CRC_H_

#ifdef _KERNEL
#include <sys/param.h>
#else /* !_KERNEL */
#include <stddef.h>
#include <stdint.h>
#endif /* _KERNEL */

extern const uint8_t bhnd_nvram_crc8_tab[];

/**
 * Calculate CRC-8 over @p buf using the Broadcom SPROM/NVRAM CRC-8
 * polynomial.
 * 
 * @param buf input buffer
 * @param size buffer size
 * @param crc last computed crc, or BHND_NVRAM_CRC8_INITIAL
 */
static inline uint8_t
bhnd_nvram_crc8(const void *buf, size_t size, uint8_t crc)
{
	const uint8_t *p = (const uint8_t *)buf;
	while (size--)
		crc = bhnd_nvram_crc8_tab[(crc ^ *p++)];

	return (crc);
}

#define	BHND_NVRAM_CRC8_INITIAL	0xFF	/**< Initial bhnd_nvram_crc8 value */
#define	BHND_NVRAM_CRC8_VALID	0x9F	/**< Valid CRC-8 checksum */

#endif /* _BHND_NVRAM_BHND_NVRAM_CRC_H_ */
