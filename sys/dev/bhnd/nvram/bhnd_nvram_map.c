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
#include <sys/types.h>
#include <sys/systm.h>

#include "bhnd_nvram_map_data.h"

/*
 * NVRAM map subroutines.
 */

/**
 * Return the size of type @p dt.
 * 
 * @param dt NVRAM data type.
 * @result the byte width of @p dt.
 */
size_t
bhnd_nvram_type_width(bhnd_nvram_dt dt)
{
	switch (dt) {
	case BHND_NVRAM_DT_INT8:
	case BHND_NVRAM_DT_UINT8:
	case BHND_NVRAM_DT_CHAR:
		return (sizeof(uint8_t));

	case BHND_NVRAM_DT_INT16:
	case BHND_NVRAM_DT_UINT16:
		return (sizeof(uint16_t));

	case BHND_NVRAM_DT_INT32:
	case BHND_NVRAM_DT_UINT32:
		return (sizeof(uint32_t));
	}

	/* Quiesce gcc4.2 */
	panic("bhnd nvram data type %u unknown", dt);
}


/**
 * Return the variable definition for @p varname, if any.
 * 
 * @param varname variable name
 * 
 * @retval bhnd_nvram_var If a valid definition for @p varname is found.
 * @retval NULL If no definition for @p varname is found. 
 */
const struct bhnd_nvram_var *
bhnd_nvram_var_defn(const char *varname)
{
	size_t	min, mid, max;
	int	order;

	/*
	 * Locate the requested variable using a binary search.
	 * 
	 * The variable table is guaranteed to be sorted in lexicographical
	 * order (using the 'C' locale for collation rules)
	 */
	min = 0;
	mid = 0;
	max = nitems(bhnd_nvram_vars) - 1;

	while (max >= min) {
		/* Select midpoint */
		mid = (min + max) / 2;

		/* Determine which side of the partition to search */
		order = strcmp(bhnd_nvram_vars[mid].name, varname);
		if (order < 0) {
			/* Search upper partition */
			min = mid + 1;
		} else if (order > 0) {
			/* Search lower partition */
			max = mid - 1;
		} else if (order == 0) {
			/* Match found */
			return (&bhnd_nvram_vars[mid]);
		}
	}

	/* Not found */
	return (NULL);
}
