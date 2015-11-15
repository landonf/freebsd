 
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/bus.h>

#include "bhndbvar.h"

devclass_t bhndb_devclass;

/**
 * Attach a PCI-BHND bridge device to @p parent.
 * 
 * @param parent A parent PCI device.
 * @param devclass The devclass of the bridge device to be added.
 * @param[out] bhndb On success, the attached bhndb bridge device.
 * @param unit The device unit number, or -1 to select the next available unit
 * number.
 * 
 * @retval 0 success
 * @retval non-zero Failed to attach the bhndb device.
 */
int
bhndb_attach(device_t parent, devclass_t devclass, device_t *bhndb, int unit)
{
	*bhndb = device_add_child(parent, devclass_get_name(devclass), unit);
	if (*bhndb == NULL)
		return (ENXIO);

	return (0);
}

/**
 * Search @p windows for the first window with the given @p type.
 * 
 * @param table The table to search.
 * @param type The required window type.
 * 
 * @retval bhndb_regwin The first matching window.
 * @retval NULL If no window of the requested type could be found. 
 */
const struct bhndb_regwin *
bhndb_regwin_find_type(const struct bhndb_regwin *table,
    bhndb_regwin_type_t type)
{
	const struct bhndb_regwin *rw;

	for (rw = table; rw->win_type != BHNDB_REGWIN_T_INVALID; rw++)
	{
		if (rw->win_type == BHNDB_REGWIN_T_DYN)
			return (rw);
	}

	return (NULL);
}

/**
 * Search @p windows for the first matching core window.
 * 
 * @param table The table to search.
 * @param class The required core class.
 * @param unit The required core unit, or -1.
 * @param port The required core unit, or -1.
 * @param region The required core unit, or -1.
 *
 * @retval bhndb_regwin The first matching window.
 * @retval NULL If no matching window was found. 
 */
const struct bhndb_regwin *
bhndb_regwin_find_core(const struct bhndb_regwin *table, bhnd_devclass_t class,
    int unit, int port, int region)
{
	const struct bhndb_regwin *rw;
	
	for (rw = table; rw->win_type != BHNDB_REGWIN_T_INVALID; rw++)
	{
		if (rw->win_type != BHNDB_REGWIN_T_CORE)
			continue;
		
		if (unit != -1 && rw->core.unit != unit)
			continue;
		
		if (port != -1 && rw->core.port != port)
			continue;
		
		if (region != -1 && rw->core.region != region)
			continue;

		return (rw);
	}

	return (NULL);
}

MODULE_VERSION(bhndb, 1);
MODULE_DEPEND(bhndb, bhnd, 1, 1, 1);