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

#include <dev/bhnd/bhnd.h>
#include "bcmavar.h"

#include "bcmavar.h"
#include "bcma_dmp.h"
#include "bcma_private.h"


/*
 * BCMA Enumeration ROM (EROM) Table
 * 
 * Provides auto-discovery of BCMA cores on Broadcom's HND SoC.
 * 
 * The EROM table address can be found at BCMA_CC_EROM_ADDR within the
 * ChipCommon registers. The table itself is comprised of 32-bit
 * type-tagged entries, organized into an array of variable-length
 * core descriptor records.
 * 
 * The final core descriptor is followed by a 32-bit BCMA_EROM_TABLE_EOF (0xF)
 * marker.
 */

static const char	*erom_entry_type_name(uint8_t entry);

static int		 erom_read32(device_t bus, struct resource *erom_res,
			     bus_size_t erom_end, bus_size_t offset,
			     uint32_t *entry);

static int		 erom_read_port_region(device_t bus,
			     struct resource *erom_res, bus_size_t erom_end,
			     bcma_pid_t req_port, uint8_t req_type, 
			     bcma_addr_t *address, bcma_size_t *size,
			     bus_size_t * const offset);

static int		 erom_scan_port_regions(device_t bus,
			     struct bcma_devinfo *dinfo, bcma_pid_t port_num,
			     bcma_sport_type port_type, 
			     struct resource *erom_res,
			     bus_size_t erom_end, bus_size_t * const offset);

static int		 erom_cmp_core_idx(const void *a, const void *b);
static int		 erom_next_core_unit(device_t dev, uint16_t vendor,
			     uint16_t device, int *unit);

static int		 erom_scan_core(device_t bus, u_int core_index,
			     struct resource *erom_res, bus_size_t erom_start,
			     bus_size_t erom_end, bus_size_t * const offset,
			     struct bcma_devinfo **result);

/* Extract entry attribute by applying _MASK and _SHIFT defines. */
#define	EROM_GET_ATTR(_entry, _attr)			\
	((_entry & BCMA_EROM_ ## _attr ## _MASK)	\
	>> BCMA_EROM_ ## _attr ## _SHIFT)

/* Test an entry's type */
#define	EROM_ENTRY_IS(_entry, _type)					\
	(EROM_GET_ATTR(_entry, ENTRY_ISVALID) &&			\
	 EROM_GET_ATTR(_entry, ENTRY_TYPE) == BCMA_EROM_ENTRY_TYPE_ ## _type)

/** Return the type name for an EROM entry */
static const char *
erom_entry_type_name (uint8_t entry)
{
	switch (EROM_GET_ATTR(entry, ENTRY_TYPE)) {
	case BCMA_EROM_ENTRY_TYPE_CORE:
		return "core";
	case BCMA_EROM_ENTRY_TYPE_MPORT:
		return "mport";
	case BCMA_EROM_ENTRY_TYPE_REGION:
		return "region";
	default:
		return "unknown";
	}
}

/**
 * Read a 32-bit entry value from the EROM table.
 * 
 * @param bus The BCMA bus.
 * @param erom_res The EROM resource.
 * @param erom_end The maximum permitted EROM offset.
 * @param offset The offset at which to perform the read.
 * @param entry Will contain the read result on success.
 * @return If successful, returns 0. If unsuccessful, returns non-zero error.
 */
static int
erom_read32(device_t bus, struct resource *erom_res, bus_size_t 
    erom_end, bus_size_t offset, uint32_t *entry)
{
	if (offset >= erom_end) {
		device_printf(bus, "EROM table missing terminating EOF\n");
		return (EINVAL);
	}
	
	*entry = bus_read_4(erom_res, offset);
	return (0);
}

/**
 * Read a port region descriptor from the EROM table.
 * 
 * @param bus The BCMA bus.
 * @param erom_res The EROM resource.
 * @param erom_end The maximum permitted EROM offset.
 * @param req_port The required port number.
 * @param req_type The required region type.
 * @param[out] address On success, the parsed region base address.
 * @param[out] size On success, the parsed region size.
 * @param[out] offset The offset at which to perform parsing. On success, this
 * will be updated to point to the next EROM table entry.
 * 
 * @return If successful, returns 0. If the end of the region table for the given
 * port and type is reached, ENOENT will be returned. On error, returns a
 * non-zero error value.
 */
static int
erom_read_port_region(
    device_t bus,
    struct resource *erom_res, 
    bus_size_t erom_end,
    bcma_pid_t req_port,
    uint8_t req_type,
    bcma_addr_t *address,
    bcma_size_t *size,
    bus_size_t * const offset)
{
	uint32_t 	 addr_high, addr_low;
	uint32_t	 entry;
	uint32_t	 size_high, size_low;
	uint8_t		 type;
	uint8_t		 port;
	uint8_t		 size_type;
	int		 error;

	/* Fetch the next table entry */
	if ((error = erom_read32(bus, erom_res, erom_end, *offset, &entry)))
		return (error);

	/* A non-region entry signifies the end of the region table. */
	if (!EROM_ENTRY_IS(entry, REGION))
		return (ENOENT);
		
	/* Parse the region attributes */
	addr_low = EROM_GET_ATTR(entry, REGION_BASE);
	addr_high = 0;
	size_type = EROM_GET_ATTR(entry, REGION_SIZE);
	type = EROM_GET_ATTR(entry, REGION_TYPE);
	port = EROM_GET_ATTR(entry, REGION_PORT);

	/* A region or type mismatch also signifies end of table. */
	if (port != req_port || type != req_type)
		return (ENOENT);

	/* Fetch the high bits of a 64-bit region address */
	if (EROM_GET_ATTR(entry, REGION_64BIT)) {
		*offset += 4;
		if ((error = erom_read32(bus, erom_res, erom_end, *offset, &addr_high)))
			return (error);
	}
	
	/* Parse the region size; it's either encoded as the binary logarithm
	 * of the number of 4K pages (i.e. log2 n), or its encoded as a
	 * 32-bit/64-bit literal value directly following the current entry. */
	size_high = 0;
	switch (size_type) {
	case BCMA_EROM_REGION_SIZE_OTHER:
		*offset += 4;
		if ((error = erom_read32(bus, erom_res, erom_end, *offset, &entry)))
			return (error);

		size_low = EROM_GET_ATTR(entry, RSIZE_VAL);
		if (EROM_GET_ATTR(entry, RSIZE_64BIT)) {
			*offset += 4;
			if ((erom_read32(bus, erom_res, erom_end, *offset, &size_high)))
				return (error);
		}

		break;
	default:
		size_low = BCMA_EROM_REGION_SIZE_BASE << size_type;
		break;
	}


	/* Return the parsed region data */
	*address = addr_low | ((uint64_t)addr_high << 32);
	*size = size_low | ((uint64_t)size_high << 32);

	/* Verify that start+count does not overflow. */
	if (count != 0 && BCMA_ADDR_MAX - (count - 1) < start) {
		device_printf(bus,
			"core%u %s%u.%u: invalid address map %llx:%llx\n",
			dinfo->cfg.core_index,
			bcma_port_type_name(port_type),
			port_num, region_num,
			(unsigned long long) start,
			(unsigned long long) count);

		error = EINVAL;
		goto cleanup;
	}

	*offset += 4;
	return (0);
}

/**
 * Register all MMIO region descriptors for the given slave port.
 * 
 * @param bus The BCMA bus.
 * @param dinfo Device info to be populated with the scanned port regions.
 * @param port_num Port index for which regions will be parsed.
 * @param port_type The port region type to be parsed.
 * @param erom_res The EROM resource.
 * @param erom_end The maximum permitted EROM offset.
 * @param[out] offset The offset at which to perform parsing. On success, this
 * will be updated to point to the next EROM table entry.
 */
static int 
erom_scan_port_regions(device_t bus, struct bcma_devinfo *dinfo,
    bcma_pid_t port_num, bcma_sport_type port_type, struct resource *erom_res,
    bus_size_t erom_end, bus_size_t * const offset)
{
	struct bcma_sport	*sport;
	struct bcma_sport_list	*sports;
	int			 error;
	uint8_t			 req_region_type;
	bool			 skip_registration;

	error = 0;
	skip_registration = false;

	/* Allocate a new port descriptor */
	sport = bcma_alloc_sport(port_num, port_type);
	if (sport == NULL)
		return (ENOMEM);
	
	/* Determine the required region type for our region descriptors */
	switch (port_type) {
	case BCMA_SPORT_TYPE_DEVICE:
		req_region_type = BCMA_EROM_REGION_TYPE_DEVICE;
		sports = &dinfo->cfg.dports;
		break;
	case BCMA_SPORT_TYPE_BRIDGE:
		req_region_type = BCMA_EROM_REGION_TYPE_BRIDGE;
		sports = &dinfo->cfg.dports;
		break;
	case BCMA_SPORT_TYPE_SWRAP:
		req_region_type = BCMA_EROM_REGION_TYPE_SWRAP;
		sports = &dinfo->cfg.wports;
		break;
	case BCMA_SPORT_TYPE_MWRAP:
		req_region_type = BCMA_EROM_REGION_TYPE_MWRAP;
		sports = &dinfo->cfg.wports;
		break;
	}

	/* Read all address regions defined for this port */
	for (bcma_rmid_t region_num = 0;; region_num++) {
		struct bcma_map	*map;
		bcma_addr_t	 start;
		bcma_addr_t	 end;
		bcma_size_t	 count;

		/* No valid port definition should come anywhere near
		 * BCMA_RMID_MAX. */
		if (region_num == BCMA_RMID_MAX) {
			device_printf(bus,
			    "core%u %s%u: region count reached upper limit of %u\n",
			    dinfo->cfg.core_index,
			    bcma_port_type_name(port_type),
			    port_num, BCMA_RMID_MAX);

			error = EINVAL;
			goto cleanup;
		}

		/* Parse the next region entry. */
		error = erom_read_port_region(bus, erom_res, erom_end, port_num, 
		    req_region_type, &start, &count, offset);
		if (error == ENOENT) {
			/* No further entries */
			error = 0;
			break;
		} else if (error) {
			goto cleanup;
		}

		/*
		 * Create the map entry associated resource. 
		 * 		
		 * We necessarily skip registration of the region in the 
		 * per-device resource_list if the memory range is not
		 * representable using rman/resource API's u_long address
		 * type.
		 */
		map = malloc(sizeof(struct bcma_map), M_BCMA, M_WAITOK);
		if (map == NULL) {
			error = ENOMEM;
			goto cleanup;
		}

		map->m_region_num = region_num;
		map->m_base = start;
		map->m_size = count;
		map->m_rid = -1;

		if (map->m_base > ULONG_MAX ||
		    map->m_base + map->m_size > ULONG_MAX)
		{
			if (bootverbose) {
				device_printf(bus,
				    "core%u %s%u.%u: region %llx-%llx extends "
				        "beyond supported addressable range\n",
				    dinfo->cfg.core_index,
				    bcma_port_type_name(port_type),
				    port_num, region_num,
				    (unsigned long long) start,
				    (unsigned long long) end);
			}
		} else {
			map->m_rid = resource_list_add_next(&dinfo->resources,
			    SYS_RES_MEMORY, start, end, count);
		}

		/* Add the region map to the port */
		STAILQ_INSERT_TAIL(&sport->sp_maps, map, m_link);
		sport->sp_num_maps++;
	}

cleanup:
	/* Append the new port descriptor on success, or deallocate the
	 * partially parsed descriptor on failure. */
	if (error == 0) {
		STAILQ_INSERT_TAIL(sports, sport, sp_link);
	} else if (sport != NULL) {
		bcma_free_sport(sport);
	}

	return error;
}


/*
 * Quick sort callout for comparing bcma devices by core index.
 */
static int
erom_cmp_core_idx(const void *a, const void *b)
{
	struct bcma_devinfo	*dinfoA, *dinfoB;

	dinfoA = device_get_ivars(*(device_t *) a);
	dinfoB = device_get_ivars(*(device_t *) b);

	if (dinfoA->cfg.core_index < dinfoB->cfg.core_index)
		return (-1);
	else if (dinfoA->cfg.core_index > dinfoB->cfg.core_index)
		return (1);
	else
		return (0);
}


/**
 * Determine the next available unit number for the given vendor/device pair.
 * 
 * @param dev The bcma bus device.
 * @param vendor The device vendor
 * @param evice The device identifier
 * @param[out] unit On success, the next available unit.
 * 
 * @retval 0 success
 * @retval non-zero an error occured fetching the device list
 */
static int
erom_next_core_unit(device_t dev, uint16_t vendor, uint16_t device, int *unit)
{
	struct bcma_devinfo	*dinfo;
	device_t		*devlist;
	int			 devcount;
	int			 error;

	/* Fetch all previously added children */
	error = device_get_children(dev, &devlist, &devcount);
	if (error)
		return (error);
	
	/* Sort by core index */
	if (devcount)
		qsort(devlist, devcount, sizeof(*devlist), erom_cmp_core_idx);

	/* Determine the next unit number */
	*unit = 0;
	for (int i = 0; i < devcount; i++) {
		dinfo = device_get_ivars(devlist[i]);
		if (dinfo->cfg.vendor == vendor && dinfo->cfg.device == device)
			*unit += 1;
	}

	free(devlist, M_TEMP);
	return (0);
}

/**
 * Parse a core entry from the EROM table.
 * 
 * @param bus The BCMA bus.
 * @param core_index The index of the core being parsed.
 * @param erom_res The EROM resource.
 * @param erom_start The initial offset of the EROM table.
 * @param erom_end The maximum permitted EROM offset.
 * @param[out] offset The offset at which to perform parsing. On success, this
 * will be updated to point to the next EROM table entry.
 * @param[out] result On success, the core's device info. The caller inherits
 * ownership of this allocation.
 * 
 * @return If successful, returns 0. If the end of the EROM table is hit,
 * ENOENT will be returned. On error, returns a non-zero error value.
 */
static int
erom_scan_core(device_t bus, u_int core_index, struct resource *erom_res,
    bus_size_t erom_start, bus_size_t erom_end, bus_size_t * const offset,
    struct bcma_devinfo **result
)
{
	struct bcma_devinfo	*dinfo;
	bcma_sport_type		 first_port_type;
	uint32_t		 entry;
	uint16_t		 core_designer;
	uint16_t		 core_part;
	uint8_t			 core_revid;
	u_long			 num_mport, num_dport, num_mwrap, num_swrap;
	int			 error;

	dinfo = NULL;

	/* Fetch the next entry */
	if (erom_read32(bus, erom_res, erom_end, *offset, &entry))
		return (EINVAL);

	/* Signal EOF */
	if (entry == BCMA_EROM_TABLE_EOF)
		return (ENOENT);
			
	/* Must be the start of the next core description */
	if (!EROM_ENTRY_IS(entry, CORE)) {
		device_printf(bus,
		    "Unexpected EROM %s entry 0x%x at offset 0x%llx\n",
		    erom_entry_type_name(entry), entry,
		    (unsigned long long) *offset-erom_start);
		return (EINVAL);
	}
	
	
	/* Parse CoreDescA */
	core_designer = EROM_GET_ATTR(entry, COREA_DESIGNER);
	core_part = EROM_GET_ATTR(entry, COREA_ID);
	

	/* Parse CoreDescB */
	*offset += 4;
	if (erom_read32(bus, erom_res, erom_end, *offset, &entry))
		return (EINVAL);

	if (!EROM_ENTRY_IS(entry, CORE)) {
		device_printf(bus,
		    "Invalid EROM CoreDescB value 0x%x at offset 0x%llx\n", 
		    entry, (unsigned long long) (*offset-erom_start));

		return (EINVAL);
	}
	
	core_revid = EROM_GET_ATTR(entry, COREB_REV);
	num_mport = EROM_GET_ATTR(entry, COREB_NUM_MP);
	num_dport = EROM_GET_ATTR(entry, COREB_NUM_DP);
	num_mwrap = EROM_GET_ATTR(entry, COREB_NUM_WMP);
	num_swrap = EROM_GET_ATTR(entry, COREB_NUM_WSP);
	
	/* These are 5-bit values in the current implementation, and should
	 * never be able to overflow BCMA_PID_MAX. */
	KASSERT(num_mport <= BCMA_PID_MAX, ("unsupported mport count"));
	KASSERT(num_dport <= BCMA_PID_MAX, ("unsupported dport count"));
	KASSERT(num_mwrap + num_swrap <= BCMA_PID_MAX,
	    ("unsupported wport count"));

	/* Allocate our device info */
	dinfo = bcma_alloc_dinfo(core_index, core_designer, core_part, core_revid);
	if (dinfo == NULL)
		return (ENOMEM);

	dinfo->cfg.num_dports = num_dport;
	dinfo->cfg.num_mports = num_mport;
	dinfo->cfg.num_wports = num_mwrap + num_swrap;

	/* Assign a core unit number */
	error = erom_next_core_unit(bus, dinfo->cfg.vendor, dinfo->cfg.device,
	    &dinfo->cfg.core_unit);
	if (error)
		goto failed;

	if (bootverbose) {
		device_printf(bus, 
			    "core%u: %s %s (cid=%hx, rev=%hhu, unit=%d)\n",
			    core_index,
			    bhnd_vendor_name(dinfo->cfg.vendor),
			    bhnd_core_name(dinfo->cfg.vendor, dinfo->cfg.device), 
			    dinfo->cfg.device, dinfo->cfg.revid,
			    dinfo->cfg.core_unit);
	}
	*offset += 4;

	/* Parse Master Port Descriptors */
	for (uint8_t i = 0; i < num_mport; i++) {
		struct bcma_mport	*mport;
		uint8_t			 mp_num;
		uint8_t			 mp_vid;
		
		/* Parse the master port descriptor */
		error = erom_read32(bus, erom_res, erom_end, *offset, &entry);
		if (error)
			goto failed;

		if (!EROM_ENTRY_IS(entry, MPORT)) {
			device_printf(bus,
			    "Invalid EROM MPD value 0x%x at ROM "
			        "offset 0x%llx\n", 
			    entry, (unsigned long long) (*offset-erom_start));
			error = EINVAL;
			goto failed;
		}
		
		mp_vid = EROM_GET_ATTR(entry, MPORT_ID);
		mp_num = EROM_GET_ATTR(entry, MPORT_NUM);


		/* Initialize a new bus mport structure */
		mport = malloc(sizeof(struct bcma_mport), M_BCMA, M_WAITOK);
		if (mport == NULL) {
			error = ENOMEM;
			goto failed;
		}
		
		mport->mp_vid = mp_vid;
		mport->mp_num = mp_num;

		/* Update dinfo */
		STAILQ_INSERT_TAIL(&dinfo->cfg.mports, mport, mp_link);

		*offset += 4;
	}
	

	/*
	 * Determine whether this is a bridge device; if so, we can
	 * expect the first sequence of address region descriptors to
	 * be of EROM_REGION_TYPE_BRIDGE instead of
	 * BCMA_EROM_REGION_TYPE_DEVICE.
	 * 
	 * It's unclear whether this is the correct mechanism by which we
	 * should detect/handle bridge devices, but this approach matches
	 * that of (some of) Broadcom's published drivers.
	 */
	if (erom_read32(bus, erom_res, erom_end, *offset, &entry)) {
		error = EINVAL;
		goto failed;
	}
	
	if (EROM_ENTRY_IS(entry, REGION) && 
	    EROM_GET_ATTR(entry, REGION_TYPE) == BCMA_EROM_REGION_TYPE_BRIDGE)
	{
		first_port_type = BCMA_SPORT_TYPE_BRIDGE;
	} else {
		first_port_type = BCMA_SPORT_TYPE_DEVICE;
	}
	
	/* Device/bridge port descriptors */
	for (uint8_t sp_num = 0; sp_num < num_dport; sp_num++) {
		error = erom_scan_port_regions(bus, dinfo, sp_num,
		    first_port_type, erom_res, erom_end, offset);

		if (error)
			goto failed;
	}

	/* Wrapper (aka device management) descriptors (for master ports). */
	for (uint8_t sp_num = 0; sp_num < num_mwrap; sp_num++) {
		error = erom_scan_port_regions(bus, dinfo, sp_num,
		    BCMA_SPORT_TYPE_MWRAP, erom_res, erom_end, offset);

		if (error)
			goto failed;
	}

	
	/* Wrapper (aka device management) descriptors (for slave ports). */	
	for (uint8_t i = 0; i < num_swrap; i++) {
		/* Slave wrapper ports are not numbered distinctly from master
		 * wrapper ports. */
		uint8_t sp_num = num_mwrap + i;
		error = erom_scan_port_regions(bus, dinfo, sp_num,
		    BCMA_SPORT_TYPE_SWRAP, erom_res, erom_end, offset);

		if (error)
			goto failed;
	}

	*result = dinfo;
	return (0);
	
failed:
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);

	return error;
}

/**
 * Scan a device enumeration ROM table, adding all discovered cores to the bus.
 * 
 * @param dev The bus to enumerate.
 * @param erom_res The enumeration ROM resource.
 * @param erom_offset Base offset of the EROM register mapping.
 */
int
bcma_scan_erom(device_t dev, struct resource *erom_res, bus_size_t erom_offset)
{
	struct bcma_devinfo	*dinfo;
	struct bcma_corecfg	*cfg;
	device_t		 child;
	bus_size_t		 erom_end;
	bus_size_t		 erom_start;
	bus_size_t		 offset;
	int			 error;
	
	offset = erom_offset + BCMA_EROM_TABLE;
	erom_end = erom_offset + BCMA_EROM_TABLE_END;
	erom_start = offset;
	dinfo = NULL;
	
	for (u_int core_index = 0; offset < erom_end && core_index < UINT_MAX;
	    core_index++) 
	{
		error = erom_scan_core(dev, core_index, erom_res, erom_start,
		    erom_end, &offset, &dinfo);
		
		/* Handle EOF or error */
		if (error == ENOENT)
			break;
		else if (error)
			return (error);

		cfg = &dinfo->cfg;

		/* Add the child device */
		child = device_add_child_ordered(dev, BHND_PROBE_ORDER_DEFAULT, NULL, -1);
		if (child == NULL) {
			error = ENXIO;
			goto failed;
		}

		/* The child device now owns the dinfo pointer */
		device_set_ivars(child, dinfo);
		dinfo = NULL;
	}

	return (0);
	
failed:
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);

	return (error);
}
