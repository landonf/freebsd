#-
# Copyright (c) 2016 Landon Fuller <landon@landonf.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# $FreeBSD$

#include <dev/bhnd/erom/bhnd_erom.h>

INTERFACE bhnd_erom;

#
# bhnd(4) device enumeration.
#
# Provides a common parser interface to the incompatible device enumeration
# tables used by bhnd(4) buses.
#

/**
 * Initialize a device enumeration table parser.
 * 
 * @param erom		The erom parser to initialize.
 * @param parent	The parent device from which EROM resources should
 *			be allocated.
 * @param chipc_addr	The base address of the ChipCommon core.
 *
 * @retval 0		success
 * @retval non-zero	if an error occurs initializing the EROM parser,
 *			a regular unix error code will be returned.
 */
METHOD int init {
	bhnd_erom_t	erom;
	device_t	parent;
	bus_addr_t	chipc_addr;
};

/**
 * Initialize an device enumeration table parser using the provided bus space
 * tag and handle.
 * 
 * @param erom		The erom parser to initialize.
 * @param bst		Bus space tag.
 * @param bsh		Bus space handle mapping the device enumeration
 *			space.
 *
 * @retval 0		success
 * @retval non-zero	if an error occurs initializing the EROM parser,
 *			a regular unix error code will be returned.
 */
METHOD int init_static {
	bhnd_erom_t		erom;
	bus_space_tag_t 	bst;
	bus_space_handle_t	bsh;
};

/**
 * Release all resources held by @p erom.
 * 
 * @param	erom	An erom parser instance previously initialized via
 *			BHND_EROM_INIT() or BHND_EROM_INIT_STATIC().
 */
METHOD void fini {
	bhnd_erom_t	erom;
};

/**
 * Parse all cores descriptors, returning the array in @p cores and the count
 * in @p num_cores.
 * 
 * The memory allocated for the table should be freed using
 * `free(*cores, M_BHND)`. @p cores and @p num_cores are not changed
 * when an error is returned.
 * 
 * @param	erom		The erom parser to be queried.
 * @param[out]	cores		The table of parsed core descriptors.
 * @param[out]	num_cores	The number of core records in @p cores.
 * 
 * @retval 0		success
 * @retval non-zero	if an error occurs, a regular unix error code will
 *			be returned.
 */
METHOD int get_core_table {
	bhnd_erom_t		 erom;
	struct bhnd_core_info	*cores;
	u_int			 num_cores;
};


/**
 * Locate the first core table entry in @p erom that matches @p desc.
 *
 * @param	erom	The erom parser to be queried.
 * @param	desc	A core match descriptor.
 * @param[out]	core	On success, the matching core info record.
 * 
 * @retval 0		success
 * @retval ENOENT	No core matching @p desc was found.
 * @retval non-zero	Reading or parsing failed.
 */
METHOD int lookup_core {
	bhnd_erom_t			 erom;
	const struct bhnd_core_match	*desc;
	struct bhnd_core_info		*core;
};

/**
 * Locate the first core table entry in @p erom that matches @p desc,
 * and return the specified port region's base address and size.
 *
 * If a core matching @p desc is not found, or the requested port region
 * is not mapped to the matching core, ENOENT is returned.
 *
 * @param	erom	The erom parser to be queried.
 * @param	desc	A core match descriptor.
 * @param	type	The port type to search for.
 * @param	port	The port to search for.
 * @param	region	The port region to search for.
 * @param[out]	addr	On success, the base address of the port region.
 * @param[out]	size	On success, the total size of the port region.
 *
 * @retval 0		success
 * @retval ENOENT	No core matching @p desc was found.
 * @retval ENOENT	No port region matching @p type, @p port, and @p region
 *			was found.
 * @retval non-zero	Reading or parsing failed.
 */
METHOD int lookup_core_addr {
	bhnd_erom_t			 erom;
	const struct bhnd_core_match	*desc;
	bhnd_port_type			 type;
	u_int				 port;
	u_int				 region;
	bhnd_addr_t			*addr;
	bhnd_size_t			*size;
};
