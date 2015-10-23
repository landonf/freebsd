/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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

#ifndef _BHND_BHNDVAR_H_
#define _BHND_BHNDVAR_H_

/*
 * Broadcom Home Networking Division (HND) Bus
 * Data Structures and Constants
 */

struct bhnd_softc {
};

enum bhnd_device_vars {
	/** Core designer's JEP-106 manufacturer ID. */ 
	BHND_IVAR_DESIGNER,
	
	/** Core identifier. */
	BHND_IVAR_CORE_ID,
	
	/** Core revision. */
	BHND_IVAR_CORE_REVISION,
	
	/** Core name. */
	BHND_IVAR_CORE_NAME
};

/*
 * Simplified accessors for bhnd device ivars
 */
#define	BHND_ACCESSOR(var, ivar, type) \
	__BUS_ACCESSOR(bhnd, var, BHND, ivar, type)

BHND_ACCESSOR(designer,		DESIGNER,	uint16_t);
BHND_ACCESSOR(core_id,		CORE_ID,	uint16_t);
BHND_ACCESSOR(core_revision,	CORE_REVISION,	uint8_t);
BHND_ACCESSOR(core_name,	CORE_NAME,	const char *);

#undef	BHND_ACCESSOR

#endif /* _BHND_BHNDVAR_H_ */