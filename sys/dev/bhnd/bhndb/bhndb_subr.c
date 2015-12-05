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
#include <sys/kernel.h>

#include "bhndb_private.h"
#include "bhndbvar.h"

/**
 * Attach a BHND bridge device to @p parent.
 * 
 * @param parent A parent PCI device.
 * @param[out] bhndb On success, the probed and attached bhndb bridge device.
 * @param unit The device unit number, or -1 to select the next available unit
 * number.
 * 
 * @retval 0 success
 * @retval non-zero Failed to attach the bhndb device.
 */
int
bhndb_attach_bridge(device_t parent, device_t *bhndb, int unit)
{
	int error;

	*bhndb = device_add_child(parent, devclass_get_name(bhndb_devclass),
	    unit);
	if (*bhndb == NULL)
		return (ENXIO);

	if (!(error = device_probe_and_attach(*bhndb)))
		return (0);

	if ((device_delete_child(parent, *bhndb)))
		device_printf(parent, "failed to detach bhndb child\n");

	return (error);
}


/**
 * Return the default bridge resource allocation priority for the given device
 * class.
 * 
 * Refer to BHNDB_RES_PRIO_CRITICAL, BHNDB_RES_PRIO_DEFAULT,
 * BHNDB_RES_PRIO_LOW, and BHNDB_RES_PRIO_NONE.
 * 
 * @param cls A BHND device class.
 */
int
bhndb_class_resource_prio(bhnd_devclass_t cls)
{
	switch (cls) {
	case BHND_DEVCLASS_SOCI:
	case BHND_DEVCLASS_SOCB:
	case BHND_DEVCLASS_OTHER:
	case BHND_DEVCLASS_INVALID:
		return (BHNDB_RES_PRIO_NONE);

	case BHND_DEVCLASS_CC:
	case BHND_DEVCLASS_CPU:
	case BHND_DEVCLASS_EROM:
		return (BHNDB_RES_PRIO_LOW);

	case BHND_DEVCLASS_PCCARD:
	case BHND_DEVCLASS_PCI:
	case BHND_DEVCLASS_PCIE:
	case BHND_DEVCLASS_MEM:
	case BHND_DEVCLASS_MEMC:
	case BHND_DEVCLASS_ENET:
	case BHND_DEVCLASS_ENET_MAC:
	case BHND_DEVCLASS_ENET_PHY:
	case BHND_DEVCLASS_WLAN:
	case BHND_DEVCLASS_WLAN_MAC:
	case BHND_DEVCLASS_WLAN_PHY:
		return (BHNDB_RES_PRIO_CRITICAL);
	}
}

/**
 * Return the count of @p type register windows in @p table that are
 * at least @p min_size large.
 * 
 * @param table The table to search.
 * @param type The required window type.
 * @param min_size The minimum window size.
 */
size_t
bhndb_regwin_count(const struct bhndb_regwin *table, bhndb_regwin_type_t type,
    bus_size_t min_size)
{
	const struct bhndb_regwin	*rw;
	size_t				 count;

	count = 0;
	for (rw = table; rw->win_type != BHNDB_REGWIN_T_INVALID; rw++) {
		if (rw->win_type == type && rw->win_size >= min_size)
			count++;
	}

	return (count);
}

/**
 * Search @p table for the first window with the given @p type.
 * 
 * @param table The table to search.
 * @param type The required window type.
 * @param min_size The minimum window size.
 * 
 * @retval bhndb_regwin The first matching window.
 * @retval NULL If no window of the requested type could be found. 
 */
const struct bhndb_regwin *
bhndb_regwin_find_type(const struct bhndb_regwin *table,
    bhndb_regwin_type_t type, bus_size_t min_size)
{
	const struct bhndb_regwin *rw;

	for (rw = table; rw->win_type != BHNDB_REGWIN_T_INVALID; rw++)
	{
		if (rw->win_type == type && rw->win_size >= min_size)
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

		if (rw->core.class != class)
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

/**
 * Search @p windows for the first matching core window. If none is found,
 * search instead for a dynamic window of at least @p min_size.
 * 
 * @param table The table to search.
 * @param class The required core class.
 * @param unit The required core unit, or -1.
 * @param port The required core unit, or -1.
 * @param region The required core unit, or -1.
 * @param min_size The minimum window size.
 *
 * @retval bhndb_regwin The first matching window.
 * @retval NULL If no matching window was found. 
 */
const struct bhndb_regwin *
bhndb_regwin_find_core_or_dyn(const struct bhndb_regwin *table,
    bhnd_devclass_t class, int unit, int port, int region,
    bus_size_t min_size)
{
	const struct bhndb_regwin *rw;

	/* Prefer a fixed core mapping */
	rw = bhndb_regwin_find_core(table, class, unit, port, region);
	if (rw != NULL)
		return (rw);

	/* Fall back on a generic dynamic window */
	return (bhndb_regwin_find_type(table, BHNDB_REGWIN_T_DYN, min_size));
}
