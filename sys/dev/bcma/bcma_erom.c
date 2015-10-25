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
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndreg.h>
#include <dev/bhnd/bhnd_device_ids.h>

#include "bcmavar.h"

#include "bcma_dmp.h"
#include "bcmavar.h"

// XXX for temporary PrimeCell PeripherialID code
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/bhnd/bhnd_pcireg.h>

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

static const char *erom_designer_name(uint16_t mfgid);
static const char *erom_entry_type_name (uint8_t entry);

static int erom_read32(device_t bus, struct resource *erom_res,
    bus_size_t erom_end, bus_size_t offset, uint32_t *entry);

static int erom_scan_port_regions(device_t bus, struct bcma_sport_list *ports,
    uint8_t port_num, bcma_sport_type port_type, struct resource *erom_res,
    bus_size_t erom_end, bus_size_t * const offset);

static int erom_scan_core(device_t bus, u_int core_idx,
    struct resource *erom_res, bus_size_t erom_start, bus_size_t erom_end,
    bus_size_t * const offset, struct bcma_devinfo **result);

/* Extract entry attribute by applying _MASK and _SHIFT defines. */
#define	EROM_GET_ATTR(_entry, _attr)			\
	((_entry & BCMA_EROM_ ## _attr ## _MASK)	\
	>> BCMA_EROM_ ## _attr ## _SHIFT)

/* Test an entry's type */
#define	EROM_ENTRY_IS(_entry, _type)					\
	(EROM_GET_ATTR(_entry, ENTRY_ISVALID) &&			\
	 EROM_GET_ATTR(_entry, ENTRY_TYPE) == BCMA_EROM_ENTRY_TYPE_ ## _type)

/** Return the name associated with a given JEP-106 manufacturer ID. */
static const char *
erom_designer_name (uint16_t mfgid)
{
	switch (mfgid) {
	case JEDEC_MFGID_ARM:
		return "ARM";
		break;
	case JEDEC_MFGID_BCM:
		return "Broadcom";
		break;
	case JEDEC_MFGID_MIPS:
		return "MIPS";
	default:
		return "unknown";
	}
}

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
 * Register all MMIO region descriptors for the given slave port.
 * 
 * @param bus The BCMA bus.
 * @param ports The port registration list to be updated.
 * @param port_num Port index for which regions will be parsed.
 * @param port_type The port region type to be parsed.
 * @param erom_res The EROM resource.
 * @param erom_end The maximum permitted EROM offset.
 * @param offset The offset at which to perform parsing. This will be updated
 * to point past the last valid parsed region on exit.
 */
static int 
erom_scan_port_regions(device_t bus, struct bcma_sport_list *ports,
    uint8_t port_num, bcma_sport_type port_type, struct resource *erom_res,
    bus_size_t erom_end, bus_size_t * const offset)
{
	struct bcma_sport	*sport;
	int			 error;
	uint8_t			 req_region_type;
	
	error = 0;

	/* Allocate a new port descriptor */
	sport = bcma_alloc_sport(port_num, port_type);
	if (sport == NULL)
		return ENOMEM;
	
	/* Determine the required region type for our region descriptors */
	switch (port_type) {
	case BCMA_SPORT_TYPE_DEVICE:
		req_region_type = BCMA_EROM_REGION_TYPE_DEVICE;
		break;
	case BCMA_SPORT_TYPE_BRIDGE:
		req_region_type = BCMA_EROM_REGION_TYPE_BRIDGE;
		break;
	case BCMA_SPORT_TYPE_SWRAP:
		req_region_type = BCMA_EROM_REGION_TYPE_SWRAP;
		break;
	case BCMA_SPORT_TYPE_MWRAP:
		req_region_type = BCMA_EROM_REGION_TYPE_MWRAP;
		break;
	}

	/* Read all address regions defined for this port */
	for (int i = 0; ; i++) {
		struct bcma_map	*map;
		uint32_t 	 addr_high, addr_low;
		uint32_t	 entry;
		uint32_t	 size_high, size_low;
		uint8_t		 region_type;
		uint8_t		 region_port;
		uint8_t		 size_type;

		/* Fetch the next entry */
		error = erom_read32(bus, erom_res, erom_end, *offset, &entry);
		if (error)
			goto done;
		
		/* A non-region entry signifies the end of the region table. */
		if (!EROM_ENTRY_IS(entry, REGION))
			goto done;
		
		/* Parse the region attributes */
		addr_low = EROM_GET_ATTR(entry, REGION_BASE);
		size_type = EROM_GET_ATTR(entry, REGION_SIZE);
		region_type = EROM_GET_ATTR(entry, REGION_TYPE);
		region_port = EROM_GET_ATTR(entry, REGION_PORT);


		/* Terminate when we hit the next port's region descriptor. */
		if (region_port != port_num || region_type != req_region_type)
			goto done;

		/* Parse the high bits of the base address */
		if (EROM_GET_ATTR(entry, REGION_64BIT)) {
			*offset += 4;
			error = erom_read32(bus, erom_res, erom_end, *offset, &addr_high);
			if (error)
				goto done;
		} else {
			addr_high = 0;
		}


		/* Parse the region size */
		if (size_type == BCMA_EROM_REGION_SIZE_OTHER) {
			*offset += 4;
			error = erom_read32(bus, erom_res, erom_end, *offset, &entry);
			if (error)
				goto done;

			size_low = EROM_GET_ATTR(entry, RSIZE_VAL);
			if (EROM_GET_ATTR(entry, RSIZE_64BIT)) {
				*offset += 4;
				error = erom_read32(bus, erom_res, erom_end, *offset, &size_high);
				if (error)
					goto done;
			} else {
				size_high = 0;
			}
		} else {
			size_low = BCMA_EROM_REGION_SIZE_BASE << size_type;
			size_high = 0;
		}
		
		/* Register the map entry for this region */
		map = malloc(sizeof(struct bcma_map), M_BCMA, M_WAITOK);
		if (map == NULL) {
			error = ENOMEM;
			goto done;
		}
		map->m_base = addr_low | ((uint64_t)addr_high << 32);
		map->m_size = size_low | ((uint64_t)size_high << 32);

		STAILQ_INSERT_TAIL(&sport->sp_maps, map, m_link);
		sport->sp_num_maps++;

		*offset += 4;
	}
		
done:
	/* Append the new port descriptor on success, or deallocate the
	 * partially parsed descriptor on failure. */
	if (error == 0) {
		STAILQ_INSERT_TAIL(ports, sport, sp_link);
	} else if (sport != NULL) {
		bcma_free_sport(sport);
	}

	return error;
}


/**
 * Parse a core entry from the EROM table.
 * 
 * @param bus The BCMA bus.
 * @param core_idx The index of the core being parsed.
 * @param erom_res The EROM resource.
 * @param erom_end The maximum permitted EROM offset.
 * @param offset The offset at which to perform parsing. This will be updated
 * to point past the last valid parsed region on exit.
 * @param result On success, the core's device info. The caller inherits ownership
 * of this allocation.
 * 
 * @return If successful, returns 0. If the end of the EROM table is hit,
 * ENOENT will be returned. On error, returns a non-zero error value.
 */
static int
erom_scan_core(device_t bus, u_int core_idx, struct resource *erom_res,
    bus_size_t erom_start, bus_size_t erom_end, bus_size_t * const offset,
    struct bcma_devinfo **result
)
{
	struct bcma_devinfo	*dinfo;
	bcma_sport_type		 first_port_type;
	uint32_t		 entry;
	uint16_t		 core_designer;
	uint16_t		 core_id;
	uint8_t			 core_revision;
	uint8_t			 num_mport, num_dport, num_mwrap, num_swrap;
	uint8_t			 swrap_port_base;
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
		device_printf(bus, "Unexpected EROM %s entry 0x%x at ROM offset 0x%lx\n", erom_entry_type_name(entry), entry, *offset-erom_start);
		return (EINVAL);
	}
	
	
	/* Parse CoreDescA */
	core_designer = EROM_GET_ATTR(entry, COREA_DESIGNER);
	core_id = EROM_GET_ATTR(entry, COREA_ID);
	

	/* Parse CoreDescB */
	*offset += 4;
	if (erom_read32(bus, erom_res, erom_end, *offset, &entry))
		return (EINVAL);

	if (!EROM_ENTRY_IS(entry, CORE)) {
		device_printf(bus, "Invalid EROM CoreDescB value 0x%x at ROM offset 0x%lx\n", entry, (*offset-erom_start));
		return (EINVAL);
	}
	
	core_revision = EROM_GET_ATTR(entry, COREB_REV);
	num_mport = EROM_GET_ATTR(entry, COREB_NUM_MP);
	num_dport = EROM_GET_ATTR(entry, COREB_NUM_DP);
	num_mwrap = EROM_GET_ATTR(entry, COREB_NUM_WMP);
	num_swrap = EROM_GET_ATTR(entry, COREB_NUM_WSP);

	/* Allocate our device info */
	dinfo = bcma_alloc_dinfo(core_designer, core_id, core_revision);
	if (dinfo == NULL)
		return (ENOMEM);

	dinfo->cfg.num_dports = num_dport;
	dinfo->cfg.num_mports = num_mport;
	dinfo->cfg.num_wports = num_mwrap + num_swrap;

	if (bootverbose) {
		device_printf(bus, 
			    "core%u: %s %s (cid=%hx, rev=%hhu)\n",
			    core_idx,
			    erom_designer_name(dinfo->cfg.designer),
			    bhnd_core_name(dinfo->cfg.designer, dinfo->cfg.core_id), 
			    dinfo->cfg.core_id, dinfo->cfg.revision);
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
			goto cleanup;

		if (!EROM_ENTRY_IS(entry, MPORT)) {
			device_printf(bus, "Invalid EROM MPD value 0x%x at ROM offset 0x%lx\n", entry, (*offset-erom_start));
			error = EINVAL;
			goto cleanup;
		}
		
		mp_vid = EROM_GET_ATTR(entry, MPORT_ID);
		mp_num = EROM_GET_ATTR(entry, MPORT_NUM);


		/* Initialize a new bus mport structure */
		mport = malloc(sizeof(struct bcma_mport), M_BCMA, M_WAITOK);
		if (mport == NULL) {
			error = ENOMEM;
			goto cleanup;
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
		goto cleanup;
	}
	
	if (EROM_ENTRY_IS(entry, REGION) && 
	    EROM_GET_ATTR(entry, REGION_TYPE) == BCMA_EROM_REGION_TYPE_BRIDGE)
	{
		first_port_type = BCMA_SPORT_TYPE_BRIDGE;
	} else {
		first_port_type = BCMA_SPORT_TYPE_DEVICE;
	}

	/* Device/bridge region descriptors */
	for (uint8_t sp_num = 0; sp_num < num_dport; sp_num++) {
		error = erom_scan_port_regions(bus, &dinfo->cfg.dports, sp_num,
		    first_port_type, erom_res, erom_end, offset);

		if (error)
			goto cleanup;
	}
	
	/* Agent (aka wrapper) region descriptors (for master ports) */
	for (uint8_t sp_num = 0; sp_num < num_mwrap; sp_num++) {
		error = erom_scan_port_regions(bus, &dinfo->cfg.wports, sp_num,
		    BCMA_SPORT_TYPE_MWRAP, erom_res, erom_end, offset);

		if (error)
			goto cleanup;
	}
	
	/* Agent (aka wrapper) region descriptors (for slave ports) */
	swrap_port_base = 1; /* hardware bug? */
	if (num_dport == 1)
		swrap_port_base = 0;
	
	for (uint8_t i = 0; i < num_swrap; i++) {
		uint8_t sp_num = i + swrap_port_base;
		error = erom_scan_port_regions(bus, &dinfo->cfg.wports, sp_num,
		    BCMA_SPORT_TYPE_SWRAP, erom_res, erom_end, offset);

		if (error)
			goto cleanup;
	}

	*result = dinfo;
	return (0);
	
cleanup:
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);

	return error;
}

// XXX Debugging code to read PrimeCell/Peripherial IDs
static void
read_primecell_id(device_t bus, struct resource *res, struct bcma_map *map) {
	uint32_t pcell_id, pid0, pid1;
	bus_addr_t offset;
	const char *designer_name;
	const char *part_name;

	/* Find the last 4KB block */
	if (map->m_size < 0x1000)
		return;
	offset = (map->m_size-1) & ~(0x1000-1);
				
	// XXX: Assumes PCI
	pci_write_config(bus, BHND_PCI_BAR0_WIN0, map->m_base + offset, 4);
				
	pcell_id = 0;
	for (int i = 0; i < 4; i++)
		pcell_id |= (bus_read_4(res, BHND_PCI_V2_BAR0_WIN0_OFFSET + 0xFF0 + (4 * i)) & 0xFF) << (i * 8);
				
	if (pcell_id != 0xB105F00D) {
		device_printf(bus, "Skipping non-PrimeCell device\n");
		return;
	}

	/* Read Peripheral ID0-ID3 */
	pid0 = 0;
	for (int i = 0; i < 4; i++)
		pid0 |= (bus_read_4(res, BHND_PCI_V2_BAR0_WIN0_OFFSET + 0xFE0 + (4 * i)) & 0xFF) << (i * 8);
				
	uint16_t part = pid0 & 0xFFF;
	uint8_t use_jedec = (pid0 & 0x80000) >> 19;
	uint16_t designer = (pid0 & 0x7F000) >> 12;
	uint8_t rev = (pid0 & 0xF00000) >> 20;
	uint8_t cust_mod = (pid0 & 0xF000000) >> 24;
	uint8_t rev_and = (pid0 & 0xF0000000) >> 28;
				
	/* Read Peripheral ID4-ID7 */
	pid1 = 0;
	for (int i = 0; i < 4; i++)
		pid1 |= (bus_read_4(res, BHND_PCI_V2_BAR0_WIN0_OFFSET + 0xFD0 + (4 * i)) & 0xFF) << (i * 8);
				
	uint8_t jedec_c_cont = pid1 & 0xF;
	uint32_t region_size = 0x1000 << ((pid1 & 0xF0) >> 4);

	if (use_jedec) {
		designer |= (jedec_c_cont << 8);
		designer_name = erom_designer_name(designer);
		part_name = bhnd_core_name(designer, part);
	} else {
		switch (designer) {
		case 0x3b:	/* Some devices use the JEDEC ID without specifying the
				 * 4-bit continuation code */
		case 0x41:
			designer_name = erom_designer_name(JEDEC_MFGID_ARM);
			part_name = bhnd_core_name(JEDEC_MFGID_ARM, part);
			break;
		default:
			designer_name = "unknown";
			part_name = "unknown";
			break;
		}
	}

	device_printf(bus, "  %s %s (designer=0x%hx, jedec=%s, part=0x%hx, rev=%hhu+%hhu, cust_mod=%hhu, size=%u, PID0=0x%08x, PID1=0x%08x)\n",
	    designer_name, part_name,
	    designer,
	    use_jedec ? "yes" : "no",
	    part, rev, rev_and, cust_mod, region_size, pid0, pid1);
}

/**
 * Scan a device enumeration ROM table, adding all discovered cores to the bus.
 * 
 * @param bus The bus to enumerate.
 * @param erom_res The enumeration ROM resource.
 * @param erom_base Base address of the EROM register mapping.
 */
int
bcma_scan_erom(device_t bus, struct resource *erom_res, bus_size_t erom_base)
{
	struct bcma_devinfo	*dinfo;
	device_t		 child;
	bus_size_t		 erom_end;
	bus_size_t		 erom_start;
	bus_size_t		 offset;
	int			 error;
	
	offset = erom_base + BCMA_EROM_TABLE;
	erom_end = erom_base + BCMA_EROM_TABLE_END;
	erom_start = offset;
	
	for (u_int core_idx = 0; offset < erom_end; core_idx++) {
		error = erom_scan_core(bus, core_idx, erom_res, erom_start,
		    erom_end, &offset, &dinfo);
		
		/* Handle EOF or error */
		if (error == ENOENT)
			break;
		else if (error)
			return (error);
		
		// XXX Debugging code to read PrimeCell/Peripherial IDs
		struct bcma_sport *sp;
		if (!STAILQ_EMPTY(&dinfo->cfg.wports)) {
			STAILQ_FOREACH(sp, &dinfo->cfg.wports, sp_link) {
				struct bcma_map *map;
				STAILQ_FOREACH(map, &sp->sp_maps, m_link) {
					read_primecell_id(bus, erom_res, map);
				}
			}
		} else if (dinfo->cfg.designer == JEDEC_MFGID_ARM) {
			STAILQ_FOREACH(sp, &dinfo->cfg.dports, sp_link) {
				struct bcma_map *map;
				STAILQ_FOREACH(map, &sp->sp_maps, m_link) {
					read_primecell_id(bus, erom_res, map);
				}
			}
		}
		
		/* Add the child device */
		// TODO: Ordering and other configuration
		child = device_add_child(bus, NULL, -1);
		if (child == NULL)
			return ENXIO;

		device_set_ivars(child, dinfo);
	}
	
	return (0);
}
