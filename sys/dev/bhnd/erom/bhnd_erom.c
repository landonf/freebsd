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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kobj.h>

#include <dev/bhnd/bhndvar.h>

#include "bhnd_erom.h"

/**
 * Allocate and return a new device enumeration table parser.
 * 
 * @param cls		The parser class for which an instance will be
 *			allocated.
 * @param parent	The parent device from which EROM resources should
 *			be allocated.
 * @param chipc_addr	The base address of the ChipCommon core.
 *
 * @retval non-NULL	success
 * @retval NULL		if an error occured allocating or initializing the
 *			EROM parser.
 */
bhnd_erom_t
bhnd_erom_alloc(bhnd_erom_class_t cls, device_t parent, bus_addr_t chipc_addr)
{
	bhnd_erom_t	erom;
	int		error;

	erom = (bhnd_erom_t)kobj_create((kobj_class_t)cls, M_BHND,
	    M_WAITOK|M_ZERO);

	if ((error = BHND_EROM_INIT(erom, parent, chipc_addr))) {
		printf("EROM init error: %d\n", error);

		kobj_delete((kobj_t)erom, M_BHND);
		return (NULL);
	}

	return (erom);
}

/**
 * Release all resources held by a @p erom parser previously
 * allocated via bhnd_erom_alloc().
 * 
 * @param	erom	An erom parser instance previously allocated via
 *			bhnd_erom_alloc().
 */
void
bhnd_erom_free(bhnd_erom_t erom)
{
	BHND_EROM_FINI(erom);
	kobj_delete((kobj_t)erom, M_BHND);
}
