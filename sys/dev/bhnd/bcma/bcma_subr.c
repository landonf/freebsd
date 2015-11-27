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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhndvar.h>

#include "bcmavar.h"


/**
 * Return the name of a slave port type.
 */
const char *
bcma_port_type_name (bcma_sport_type port_type)
{
	switch (port_type) {
	case BCMA_SPORT_TYPE_DEVICE:
		return "device";
	case BCMA_SPORT_TYPE_BRIDGE:
		return "bridge";
	case BCMA_SPORT_TYPE_SWRAP:
		return "swrap";
	case BCMA_SPORT_TYPE_MWRAP:
		return "mwrap";
	}
}

/**
 * Search @p cfg for mapped address region for the given @p type, @p port_id,
 * and @p map_id.
 * 
 * @param cfg The core configuration to search.
 * @param type The port type to search for.
 * @param port_id The port identifier to search for.
 * @param map_id The map identifier to search for.
 * 
 * @retval bcma_map if the requested map is found.
 * @retval NULL not found
 */
struct bcma_map *
bcma_corecfg_find_region_map(struct bcma_corecfg *cfg, bcma_sport_type type,
    bcma_pid_t port_id, bcma_rmid_t map_id)
{
	struct bcma_sport_list	*ports;
	struct bcma_sport	*port;
	struct bcma_map		*map;

	switch (type) {
	case BCMA_SPORT_TYPE_BRIDGE:
		ports = &cfg->bridge_ports;
		break;
	case BCMA_SPORT_TYPE_DEVICE:
		ports = &cfg->dev_ports;
		break;
	case BCMA_SPORT_TYPE_MWRAP:
	case BCMA_SPORT_TYPE_SWRAP:
		ports = &cfg->wrapper_ports;
		break;
	}

	STAILQ_FOREACH(port, ports, sp_link) {
		if (port->sp_num != port_id)
			continue;

		STAILQ_FOREACH(map, &port->sp_maps, m_link) {
			if (map->m_region_num == map_id)
				return (map);
		}
	}

	return (NULL);
}


 /**
 * Allocate and initialize new core config structure.
 * 
 * @param core_index Core index on the bus.
 * @param core_unit Core unit number.
 * @param vendor Core designer.
 * @param device Core identifier (e.g. part number).
 * @param revid Core revision identifier.
 */
struct bcma_corecfg *
bcma_alloc_corecfg(u_int core_index, int core_unit, uint16_t vendor,
    uint16_t device, uint8_t revid)
{
	struct bcma_corecfg *cfg;

	cfg = malloc(sizeof(*cfg), M_BHND, M_WAITOK);
	if (cfg == NULL)
		return NULL;

	cfg->vendor = vendor;
	cfg->device = device;
	cfg->revid = revid;
	cfg->core_index = core_index;
	cfg->core_unit = core_unit;
	
	STAILQ_INIT(&cfg->master_ports);
	cfg->num_master_ports = 0;

	STAILQ_INIT(&cfg->dev_ports);
	cfg->num_dev_ports = 0;

	STAILQ_INIT(&cfg->bridge_ports);
	cfg->num_bridge_ports = 0;

	STAILQ_INIT(&cfg->wrapper_ports);
	cfg->num_wrapper_ports = 0;

	return (cfg);
}

/**
 * Deallocate the given core config and any associated resources.
 * 
 * @param corecfg Core info to be deallocated.
 */
void
bcma_free_corecfg(struct bcma_corecfg *corecfg)
{
	struct bcma_mport *mport, *mnext;
	struct bcma_sport *sport, *snext;

	STAILQ_FOREACH_SAFE(mport, &corecfg->master_ports, mp_link, mnext) {
		free(mport, M_BHND);
	}
	
	STAILQ_FOREACH_SAFE(sport, &corecfg->dev_ports, sp_link, snext) {
		bcma_free_sport(sport);
	}

	STAILQ_FOREACH_SAFE(sport, &corecfg->bridge_ports, sp_link, snext) {
		bcma_free_sport(sport);
	}
	
	STAILQ_FOREACH_SAFE(sport, &corecfg->wrapper_ports, sp_link, snext) {
		bcma_free_sport(sport);
	}

	free(corecfg, M_BHND);
}

/**
 * Populate the resource list and bcma_map RIDs using the maps defined on
 * @p ports.
 * 
 * @param bus The requesting bus device.
 * @param dinfo The device info instance to be initialized.
 * @param ports The set of ports to be enumerated
 */
static void
bcma_dinfo_init_resource_info(device_t bus, struct bcma_devinfo *dinfo,
    struct bcma_sport_list *ports)
{
	struct bcma_map		*map;
	struct bcma_sport	*port;
	bcma_addr_t		 end;

	STAILQ_FOREACH(port, ports, sp_link) {
		STAILQ_FOREACH(map, &port->sp_maps, m_link) {
			/*
			* Create the corresponding device resource list entry.
			* 
			* We necessarily skip registration of the region in the 
			* per-device resource_list if the memory range is not
			* representable using rman/resource API's u_long address
			* type.
			*/
			end = map->m_base + map->m_size;
			if (map->m_base <= ULONG_MAX && end <= ULONG_MAX) {
				map->m_rid = resource_list_add_next(
				    &dinfo->resources, SYS_RES_MEMORY,
				    map->m_base, end, map->m_size);
			} else if (bootverbose) {
				device_printf(bus,
				    "core%u %s%u.%u: region %llx-%llx extends "
				        "beyond supported addressable range\n",
				    dinfo->corecfg->core_index,
				    bcma_port_type_name(port->sp_type),
				    port->sp_num, map->m_region_num,
				    (unsigned long long) map->m_base,
				    (unsigned long long) end);
			}
		}
	}
}

/**
 * Allocate and initialize new device info structure, assuming ownership
 * of the provided core configuration.
 * 
 * @param dev The requesting bus device.
 * @param corecfg Device core configuration.
 */
struct bcma_devinfo *
bcma_alloc_dinfo(device_t bus, struct bcma_corecfg *corecfg)
{
	struct bcma_devinfo *dinfo;
	
	dinfo = malloc(sizeof(struct bcma_devinfo), M_BHND, M_WAITOK);
	if (dinfo == NULL)
		return NULL;

	dinfo->corecfg = corecfg;

	resource_list_init(&dinfo->resources);

	bcma_dinfo_init_resource_info(bus, dinfo, &corecfg->dev_ports);
	bcma_dinfo_init_resource_info(bus, dinfo, &corecfg->bridge_ports);
	bcma_dinfo_init_resource_info(bus, dinfo, &corecfg->wrapper_ports);

	return dinfo;
}

/**
 * Deallocate the given device info structure and any associated resources.
 * 
 * @param dinfo Device info to be deallocated.
 */
void
bcma_free_dinfo(struct bcma_devinfo *dinfo)
{
	bcma_free_corecfg(dinfo->corecfg);
	resource_list_free(&dinfo->resources);

	free(dinfo, M_BHND);
}


/**
 * Allocate and initialize new slave port descriptor.
 * 
 * @param port_num Per-core port number.
 * @param port_type Port type.
 */
struct bcma_sport *
bcma_alloc_sport(bcma_pid_t port_num, bcma_sport_type port_type)
{
	struct bcma_sport *sport;
	
	sport = malloc(sizeof(struct bcma_sport), M_BHND, M_WAITOK);
	if (sport == NULL)
		return NULL;
	
	sport->sp_num = port_num;
	sport->sp_type = port_type;
	sport->sp_num_maps = 0;
	STAILQ_INIT(&sport->sp_maps);

	return sport;
}

/**
 * Deallocate all resources associated with the given port descriptor.
 * 
 * @param sport Port descriptor to be deallocated.
 */
void
bcma_free_sport(struct bcma_sport *sport) {
	struct bcma_map *map, *mapnext;

	STAILQ_FOREACH_SAFE(map, &sport->sp_maps, m_link, mapnext) {
		free(map, M_BHND);
	}

	free(sport, M_BHND);
}

