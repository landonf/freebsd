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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_NVRAM_BHND_NVRAMREG_H_
#define _BHND_NVRAM_BHND_NVRAMREG_H_

#define	NVRAM_KEY_MAX	64	/** maximum key length (not incl. NUL) */
#define	NVRAM_VAL_MAX	255	/** maximum value length (not incl. NUL) */

#define	NVRAM_DEVPATH_STR	"devpath"	/**< name prefix of device path
						  *  aliases */
#define	NVRAM_DEVPATH_LEN	(sizeof(NVRAM_DEVPATH_STR) - 1)

/* NVRAM header fields */
#define	NVRAM_MAGIC			0x48534C46	/* 'FLSH' */
#define	NVRAM_VERSION			1

#define	NVRAM_CRC_SKIP			9	/* skip magic, size, and crc8 */

#define	NVRAM_CFG0_CRC_MASK		0x000000FF
#define	NVRAM_CFG0_CRC_SHIFT		0
#define	NVRAM_CFG0_VER_MASK		0x0000FF00
#define	NVRAM_CFG0_VER_SHIFT		8
#define	NVRAM_CFG0_SDRAM_INIT_MASK	0xFFFF0000
#define	NVRAM_CFG0_SDRAM_INIT_SHIFT	16
#define	NVRAM_CFG0_SDRAM_INIT_VAR	"sdram_init"

#define	NVRAM_CFG1_SDRAM_CFG_MASK	0x0000FFFF
#define	NVRAM_CFG0_SDRAM_CFG_SHIFT	0
#define	NVRAM_CFG0_SDRAM_CFG_VAR	"sdram_config"
#define	NVRAM_CFG1_SDRAM_REFRESH_MASK	0xFFFF0000
#define	NVRAM_CFG1_SDRAM_REFRESH_SHIFT	16
#define	NVRAM_CFG1_SDRAM_REFRESH_VAR	"sdram_refresh"

#define	NVRAM_SDRAM_NCDL_VAR		"sdram_ncdl"

/* WGT634U-specific TLV encoding */
#define	NVRAM_TLV_TF_U8_LEN		0x01	/**< type has 8-bit length */
#define	NVRAM_TLV_TYPE_END		0x00	/**< end of table */
#define	NVRAM_TLV_TYPE_ENV		0x01	/**< variable record */

#endif /* _BHND_NVRAM_BHND_NVRAMREG_H_ */
