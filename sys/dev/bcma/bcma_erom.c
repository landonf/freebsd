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

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndreg.h>
#include <dev/bhnd/bhnd_device_ids.h>

#include "bcma.h"
#include "bcma_dmp.h"
#include "bcma_erom.h"

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
static const char *erom_region_type_name(uint8_t region_type);
static const char *erom_entry_type_name (uint8_t entry);

static int erom_read32(device_t bus, struct resource *erom_res,
		       bus_size_t erom_end, bus_size_t offset, uint32_t *entry);

static int erom_scan_port_regions(device_t bus, uint8_t port,
				  uint8_t port_type, struct resource *erom_res,
				  bus_size_t erom_end, bus_size_t *offset);

/* Extract entry attribute by applying _MASK and _SHIFT defines. */
#define EROM_GET_ATTR(_entry, _attr)	\
	((_entry & BCMA_EROM_ ## _attr ## _MASK) \
	 >> BCMA_EROM_ ## _attr ## _SHIFT)

/* Test an entry's type */
#define EROM_ENTRY_IS(_entry, _type)	\
	(EROM_GET_ATTR(_entry, ENTRY_ISVALID) && \
	 EROM_GET_ATTR(_entry, ENTRY_TYPE) == BCMA_EROM_ENTRY_TYPE_ ## _type)

/** Return the name associated with a given JEP-106 manufacturer ID. */
static const char *erom_designer_name (uint16_t mfgid) {
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
static const char *erom_entry_type_name (uint8_t entry) {
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

/** Return the name for an EROM region type */
static const char *erom_region_type_name (uint8_t region_type) {
	switch (region_type) {
		case BCMA_EROM_REGION_TYPE_SLAVE:
			return "slave";
		case BCMA_EROM_REGION_TYPE_BRIDGE:
			return "bridge";
		case BCMA_EROM_REGION_TYPE_MWRAP:
			return "mwrap";
		case BCMA_EROM_REGION_TYPE_SWRAP:
			return "swrap";
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
 * Scan all Region Descriptors for the given interconnect port.
 * 
 * @param bus The BCMA bus.
 * @param port A port index for which regions will be parsed.
 * @param port_type The port region type to be parsed.
 * @param erom_res The EROM resource.
 * @param erom_end The maximum permitted EROM offset.
 * @param offset The offset at which to perform parsing. This will be updated
 * to point past the last valid parsed region on exit.
 */
static int 
erom_scan_port_regions(device_t bus, uint8_t port, uint8_t port_type,
    struct resource *erom_res, bus_size_t erom_end, bus_size_t *offset)
{
	/* Read all address regions defined for this port */
	for (int i = 0; ; i++) {	
		uint32_t 	addr_high, addr_low;
		uint32_t	entry;
		uint32_t	size_high, size_low;
		uint8_t		region_type;
		uint8_t		region_port;
		uint8_t		size_type;

		/* Fetch the next entry */
		if (erom_read32(bus, erom_res, erom_end, *offset, &entry))
			return (EINVAL);
		
		if (!EROM_ENTRY_IS(entry, REGION))
			return (0);
		
		/* Parse the region attributes */
		addr_low = EROM_GET_ATTR(entry, REGION_BASE);
		size_type = EROM_GET_ATTR(entry, REGION_SIZE);
		region_type = EROM_GET_ATTR(entry, REGION_TYPE);
		region_port = EROM_GET_ATTR(entry, REGION_PORT);


		/* Terminate when we hit the next port's region descriptor. */
		if (region_port != port || region_type != port_type) {
			return (0);
		}


		/* Parse the high bits of the base address */
		if (EROM_GET_ATTR(entry, REGION_64BIT)) {
			*offset += 4;
			if (erom_read32(bus, erom_res, erom_end, *offset, &addr_high))
				return (EINVAL);
		} else {
			addr_high = 0;
		}


		/* Parse the region size */
		if (size_type == BCMA_EROM_REGION_SIZE_OTHER) {
			*offset += 4;
			if (erom_read32(bus, erom_res, erom_end, *offset, &entry))
				return (EINVAL);
			
			size_low = EROM_GET_ATTR(entry, RSIZE_VAL);
			if (EROM_GET_ATTR(entry, RSIZE_64BIT)) {
				*offset += 4;
				if (erom_read32(bus, erom_res, erom_end, *offset, &size_high))
					return (EINVAL);
			} else {
				size_high = 0;
			}
		} else {
			size_low = BCMA_EROM_REGION_SIZE_BASE << size_type;
			size_high = 0;
		}
		
		// TODO
		uint64_t addr = addr_low | ((uint64_t)addr_high << 32);
		uint64_t size = size_low | ((uint64_t)size_high << 32);
		device_printf(bus, "%s%hhu.%d at 0x%08lx-0x%08lx\n", erom_region_type_name(region_type), port, i, addr, addr+size-1);
		
		*offset += 4;
	}
	
	return (0);
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
	bus_size_t erom_end;
	bus_size_t erom_start;
	bus_size_t offset;
	
	offset = erom_base + BCMA_EROM_TABLE;
	erom_end = erom_base + BCMA_EROM_TABLE_END;
	erom_start = offset;
	
	while (offset != erom_end) {
		uint32_t	entry;
		uint16_t	core_designer;
		uint16_t	core_partnum;
		uint8_t		core_revision;
		uint8_t		first_port_type;
		uint8_t		num_mport, num_sport, num_mwrap, num_swrap;
		uint8_t		swrap_port_base;

		/* Fetch the next entry */
		if (erom_read32(bus, erom_res, erom_end, offset, &entry))
			return (EINVAL);

		if (entry == BCMA_EROM_TABLE_EOF)
			break;
				
		/* Must be the start of the next core description */
		if (!EROM_ENTRY_IS(entry, CORE)) {
			device_printf(bus, "Unexpected EROM %s entry 0x%x at ROM offset 0x%lx\n", erom_entry_type_name(entry), entry, offset-erom_start);
			return (EINVAL);
		}
		
		
		/* Parse CoreDescA */
		core_designer = EROM_GET_ATTR(entry, COREA_DESIGNER);
		core_partnum = EROM_GET_ATTR(entry, COREA_ID);
		

		/* Parse CoreDescB */
		offset += 4;
		if (erom_read32(bus, erom_res, erom_end, offset, &entry))
			return (EINVAL);

		if (!EROM_ENTRY_IS(entry, CORE)) {
			device_printf(bus, "Invalid EROM CoreDescB value 0x%x at ROM offset 0x%lx\n", entry, (offset-erom_start));
			return (EINVAL);
		}
		
		core_revision = EROM_GET_ATTR(entry, COREB_REV);
		num_mport = EROM_GET_ATTR(entry, COREB_NUM_MP);
		num_sport = EROM_GET_ATTR(entry, COREB_NUM_SP);
		num_mwrap = EROM_GET_ATTR(entry, COREB_NUM_WMP);
		num_swrap = EROM_GET_ATTR(entry, COREB_NUM_WSP);

			
		device_printf(bus, "erom[%lx] = 0x%x (designer=%s, core=\"%s\" cid=%hx, rev=%hhx)\n", (offset-erom_start)/4, entry, erom_designer_name(core_designer), bhnd_core_name(core_designer, core_partnum), core_partnum, core_revision);
		device_printf(bus, "erom[%lx] nmp=%hhx\tnsp=%hhx\tnwmp=%hhx\tnwsp=%hhx\n", (offset-erom_start)/4, num_mport, num_sport, num_mwrap, num_swrap);
		offset += 4;

		/* Parse Master Port Descriptors */
		for (uint8_t i = 0; i < num_mport; i++) {
			uint8_t master_id;
			uint8_t port_num;
			
			if (erom_read32(bus, erom_res, erom_end, offset, &entry))
				return (EINVAL);
			
			if (!EROM_ENTRY_IS(entry, MPORT)) {
				device_printf(bus, "Invalid EROM MPD value 0x%x at ROM offset 0x%lx\n", entry, (offset-erom_start));
				return (EINVAL);
			}
			
			master_id = EROM_GET_ATTR(entry, MPORT_ID);
			port_num = EROM_GET_ATTR(entry, MPORT_NUM);
			
			device_printf(bus, "mport%hhu-%hhu\n", port_num, master_id);

			offset += 4;
		}
		
		/*
		 * Determine whether this is a bridge device; if so, we can
		 * expect the first sequence of address region descriptors to
		 * be of EROM_REGION_TYPE_BRIDGE instead of EROM_REGION_TYPE_SLAVE.
		 * 
		 * It's unclear whether this is the correct mechanism by which we
		 * should detect/handle bridge devices, but this approach matches
		 * that of (some of) Broadcom's published drivers.
		 */
		if (erom_read32(bus, erom_res, erom_end, offset, &entry))
			return (EINVAL);
		
		if (EROM_ENTRY_IS(entry, REGION) && 
			EROM_GET_ATTR(entry, REGION_TYPE) == BCMA_EROM_REGION_TYPE_BRIDGE)
		{
			first_port_type = BCMA_EROM_REGION_TYPE_BRIDGE;
		} else {
			first_port_type = BCMA_EROM_REGION_TYPE_SLAVE;
		}

		/* Slave/Bridge Region Descriptors */
		for (uint8_t port = 0; port < num_sport; port++) {
			if (erom_scan_port_regions(bus, port, first_port_type, erom_res, erom_end, &offset))
				return (EINVAL);
		}
		
		/* Master (Wrapper) Region Descriptors */
		for (uint8_t port = 0; port < num_mwrap; port++) {
			if (erom_scan_port_regions(bus, port, BCMA_EROM_REGION_TYPE_MWRAP, erom_res, erom_end, &offset))
				return (EINVAL);
		}
		
		/* Slave (Wrapper) Region Descriptors. */
		swrap_port_base = 1; /* hardware bug? */
		if (num_sport == 1)
			swrap_port_base = 0;
		
		for (uint8_t i = 0; i < num_swrap; i++) {
			uint8_t port = i + swrap_port_base;
			if (erom_scan_port_regions(bus, port, BCMA_EROM_REGION_TYPE_SWRAP, erom_res, erom_end, &offset))
				return (EINVAL);
		}
	}
	
	return (0);
}