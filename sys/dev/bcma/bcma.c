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


/*
 * BCMA Enumeration ROM (EROM) Table
 * 
 * Provides auto-discovery of BCMA cores on Broadcom's HND SoC.
 * 
 * The EROM table is comprised of 32-bit type-tagged entries, organized
 * into an array of variable-length core descriptor records.
 * 
 * The final core descriptor is followed by a 32-bit EROM_ENTRY_EOF (0xF)
 * marker.
 * 
 * ==Background==
 * 
 * This source of this enumeration ROM data format has been described
 * by Broadcom engineers (and in published header files) as being
 * ARM's PL-368 "Device Management Plugin" system IP, included with
 * the CoreLink AMBA Designer tooling.
 * 
 * Documentation for the PL-368 is not publicly available, however,
 * and the only public reference by ARM to its existence appears to be
 * in the proprietary "NIC-301 Interconnect Device Management (PL368)"
 * errata publication, available to licensees as part of ARM's
 * CoreLink Controllers and Peripherals Engineering Errata.
 * 
 * ==Locating the EROM==
 * 
 * The EROM table address can be found at BCMA_CC_EROM_ADDR within the
 * ChipCommon registers.
 */

static const char *erom_designer_name(uint16_t mfgid);
static const char *erom_region_type_name(uint8_t region_type);
static const char *erom_entry_type_name (uint8_t entry);

static int bcma_scan_erom_port_regions(device_t bus,
	uint8_t port,
	uint8_t port_type,
	struct resource *erom_res,
	bus_size_t erom_base, 
	bus_size_t *offset);

#define EROM_ENTRY_ISVALID_MASK		0x1	/* is entry valid? */
#define EROM_ENTRY_ISVALID_SHIFT	0
#define EROM_ENTRY_EOF			0xF	/* end of EROM table */

/* Entry type flags */
#define EROM_ENTRY_TYPE_MASK		0x6	/* entry type mask */
#define EROM_ENTRY_TYPE_SHIFT		0
#  define EROM_ENTRY_CORE		0x0	/* core descriptor */
#  define EROM_ENTRY_MPORT		0x2	/* master port descriptor */
#  define EROM_ENTRY_REGION		0x4	/* address region descriptor */

/* Core DescriptorA (31:0) */
#define EROM_COREA_DESIGNER_MASK	0xFFF00000	/* core designer (JEP-106 mfg id) */
#define EROM_COREA_DESIGNER_SHIFT	20
#define EROM_COREA_ID_MASK		0x000FFF00	/* broadcom-assigned core id */
#define EROM_COREA_ID_SHIFT		8
#define EROM_COREA_CLASS_MASK		0x000000F0	/* core class */
#define EROM_COREA_CLASS_SHIFT		4

/* Core DescriptorB (63:32) */
#define EROM_COREB_NUM_MP_MASK		0x0000001F	/* master port count */
#define EROM_COREB_NUM_MP_SHIFT		4
#define EROM_COREB_NUM_SP_MASK		0x00003E00	/* slave port count */
#define EROM_COREB_NUM_SP_SHIFT		9
#define EROM_COREB_NUM_WMP_MASK		0x0007C000	/* master wrapper port count */
#define EROM_COREB_NUM_WMP_SHIFT	14
#define EROM_COREB_NUM_WSP_MASK		0x00F80000	/* slave wrapper port count */
#define EROM_COREB_NUM_WSP_SHIFT	19
#define EROM_COREB_REV_MASK		0xFF000000	/* broadcom-assigned core revision */
#define EROM_COREB_REV_SHIFT		24

/* Master Port Descriptor */
#define EROM_MPORT_NUM_MASK		0x0000FF00	/* port number */
#define EROM_MPORT_NUM_SHIFT		8

/* Region Descriptor */
#define EROM_REGION_BASE_MASK		0xFFFFF000	/* region base address */
#define EROM_REGION_BASE_SHIFT		0
#define EROM_REGION_64BIT_MASK		0x00000008	/* base address spans two 32-bit entries */
#define EROM_REGION_64BIT_SHIFT		0
#define EROM_REGION_PORT_MASK		0x00000F00	/* region's associated port */
#define EROM_REGION_PORT_SHIFT		8
#define EROM_REGION_TYPE_MASK		0x000000C0	/* region type */
#define EROM_REGION_TYPE_SHIFT		6
#  define EROM_REGION_TYPE_SLAVE	0		/* slave */
#  define EROM_REGION_TYPE_BRIDGE	1		/* bridge */
#  define EROM_REGION_TYPE_SWRAP	2		/* slave wrapper */
#  define EROM_REGION_TYPE_MWRAP	3		/* master wrapper */

#define EROM_REGION_SIZE_MASK		0x00000030	/* region size encoding */
#define EROM_REGION_SIZE_SHIFT		4
#  define EROM_REGION_SIZE_4K		0		/* 4K region */
#  define EROM_REGION_SIZE_8K		1		/* 8K region */
#  define EROM_REGION_SIZE_16K		2		/* 16K region */
#  define EROM_REGION_SIZE_OTHER	3		/* defined by an additional size descriptor entry. */
#define EROM_REGION_SIZE_BASE		0x1000

/* Region Size Descriptor */
#define EROM_RSIZE_VAL_MASK		0xFFFFF000	/* region size */
#define EROM_RSIZE_VAL_SHIFT		0
#define EROM_RSIZE_64BIT_MASK		0x00000008	/* size spans two 32-bit entries */
#define EROM_RSIZE_64BIT_SHIFT		0

/* Read arbitrary entry attribute by applying the attribute's _MASK and _SHIFT defines. */
#define EROM_GET_EATTR(_entry, _attr)	((_entry & EROM_ ## _attr ## _MASK) >> EROM_ ## _attr ## _SHIFT)

/* Test entry type */
#define EROM_ENTRY_IS(_entry, _type)	(EROM_GET_EATTR(_entry, ENTRY_ISVALID) && \
					 EROM_GET_EATTR(_entry, ENTRY_TYPE) == _type)

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

static const char *erom_entry_type_name (uint8_t entry) {
	switch (entry & EROM_ENTRY_TYPE_MASK) {
		case EROM_ENTRY_CORE:
			return "core";
		case EROM_ENTRY_MPORT:
			return "mport";
		case EROM_ENTRY_REGION:
			return "region";
		default:
			return "unknown";
	}
}

static const char *erom_region_type_name (uint8_t region_type) {
	switch (region_type) {
		case EROM_REGION_TYPE_SLAVE:
			return "slave";
		case EROM_REGION_TYPE_BRIDGE:
			return "bridge";
		case EROM_REGION_TYPE_MWRAP:
			return "mwrap";
		case EROM_REGION_TYPE_SWRAP:
			return "swrap";
		default:
			return "unknown";
	}
}

/**
 * Scan all Region Descriptors for the given interconnect port.
 * 
 * @param bus The BCMA bus.
 * @param port A port index for which regions will be parsed.
 * @param port_type The port region type to be parsed.
 * @param erom_res The EROM resource.
 * @param offset The offset at which to perform parsing. This will be updated to point
 * past the last valid parsed region on exit.
 */
static int bcma_scan_erom_port_regions(
	device_t bus,
	uint8_t port,
	uint8_t port_type,
	struct resource *erom_res,
	bus_size_t erom_base, 
	bus_size_t *offset)
{
	/* Read all address regions defined for this port */
	for (int i = 0; ; i++) {	
		uint32_t addr_high, addr_low;
		uint32_t entry;
		uint8_t	 region_type;
		uint8_t region_port;
		uint32_t size_high, size_low;
		uint8_t	 size_type;

		entry = bus_read_4(erom_res, *offset);
		
		if (!EROM_ENTRY_IS(entry, EROM_ENTRY_REGION)) {
			return (0);
		}
		
		/* Parse the region attributes */
		addr_low = EROM_GET_EATTR(entry, REGION_BASE);
		size_type = EROM_GET_EATTR(entry, REGION_SIZE);
		region_type = EROM_GET_EATTR(entry, REGION_TYPE);
		region_port = EROM_GET_EATTR(entry, REGION_PORT);


		/* Terminate if we've read past the end of our region definitions. */
		if (region_port != port || region_type != port_type) {
			return (0);
		}


		/* Parse the high bits of the base address */
		if (EROM_GET_EATTR(entry, REGION_64BIT)) {
			*offset += 4;
			addr_high = bus_read_4(erom_res, *offset);
		} else {
			addr_high = 0;
		}


		/* Parse the region size */
		if (size_type == EROM_REGION_SIZE_OTHER) {
			*offset += 4;
			entry = bus_read_4(erom_res, *offset);
			size_low = EROM_GET_EATTR(entry, RSIZE_VAL);
			if (EROM_GET_EATTR(entry, RSIZE_64BIT)) {
				*offset += 4;
				size_high = bus_read_4(erom_res, *offset);
			} else {
				size_high = 0;
			}
		} else {
			size_low = EROM_REGION_SIZE_BASE << size_type;
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
 * @param offset The offset to the enumeration table.
 * @param max_size The maximum size of the table.
 */
int bcma_scan_erom(device_t bus, struct resource *erom_res, bus_size_t offset, bus_size_t max_size)
{
	bus_size_t erom_end;
	bus_size_t erom_base;
	
	erom_end = offset + max_size; // TODO - Refer to ER_REMAPCONTROL offset
	erom_base = offset;
	
	while (offset != erom_end) {
		uint16_t	core_designer;
		uint8_t		core_revision;
		uint16_t	core_partnum;
		uint32_t	entry;
		bus_size_t	entry_start;
		uint8_t		first_port_type;
		uint8_t		num_mport, num_sport, num_mwrap, num_swrap;
		uint8_t		swrap_port_base;

		/* Fetch the next entry */
		entry = bus_read_4(erom_res, offset);
		entry_start = offset;

		if (entry == EROM_ENTRY_EOF)
			break;
		
		
		/* Must be the start of the next core description */
		if (!EROM_ENTRY_IS(entry, EROM_ENTRY_CORE)) {
			device_printf(bus, "Unexpected EROM %s entry 0x%x at ROM offset 0x%lx\n", erom_entry_type_name(entry), entry, offset-erom_base);
			return (EINVAL);
		}
		
		
		/* Parse CoreDescA */
		core_designer = EROM_GET_EATTR(entry, COREA_DESIGNER);
		core_partnum = EROM_GET_EATTR(entry, COREA_ID);
		

		/* Parse CoreDescB */
		offset += 4;
		entry = bus_read_4(erom_res, offset);
		if (!EROM_ENTRY_IS(entry, EROM_ENTRY_CORE)) {
			device_printf(bus, "Invalid EROM CoreDescB value 0x%x at ROM offset 0x%lx\n", entry, (offset-erom_base));
			return (EINVAL);
		}
		
		core_revision = EROM_GET_EATTR(entry, COREB_REV);
		num_mport = EROM_GET_EATTR(entry, COREB_NUM_MP);
		num_sport = EROM_GET_EATTR(entry, COREB_NUM_SP);
		num_mwrap = EROM_GET_EATTR(entry, COREB_NUM_WMP);
		num_swrap = EROM_GET_EATTR(entry, COREB_NUM_WSP);

			
		device_printf(bus, "erom[%lx] = 0x%x (designer=%s, core=\"%s\" cid=%hx, rev=%hhx)\n", (offset-erom_base)/4, entry, erom_designer_name(core_designer), bhnd_core_name(core_designer, core_partnum), core_partnum, core_revision);
		device_printf(bus, "erom[%lx] nmp=%hhx\tnsp=%hhx\tnwmp=%hhx\tnwsp=%hhx\n", (offset-erom_base)/4, num_mport, num_sport, num_mwrap, num_swrap);
		offset += 4;

		/* Parse Master Port Descriptors */
		for (uint8_t i = 0; i < num_mport; i++) {
			uint8_t port_num;
			
			entry = bus_read_4(erom_res, offset);
			
			if (!EROM_ENTRY_IS(entry, EROM_ENTRY_MPORT)) {
				device_printf(bus, "Invalid EROM MPD value 0x%x at ROM offset 0x%lx\n", entry, (offset-erom_base));
				return (EINVAL);
			}
			
			port_num = EROM_GET_EATTR(entry, MPORT_NUM);
			
			device_printf(bus, "mport%hhu\n", port_num);

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
		entry = bus_read_4(erom_res, offset);
		if (EROM_ENTRY_IS(entry, EROM_ENTRY_REGION) && 
			EROM_GET_EATTR(entry, REGION_TYPE) == EROM_REGION_TYPE_BRIDGE)
		{
			first_port_type = EROM_REGION_TYPE_BRIDGE;
		} else {
			first_port_type = EROM_REGION_TYPE_SLAVE;
		}

		/* Slave/Bridge Region Descriptors */
		for (uint8_t i = 0; i < num_sport; i++) {
			if (bcma_scan_erom_port_regions(bus, i, first_port_type, erom_res, erom_base, &offset))
				return (EINVAL);
		}
		
		/* Master (Wrapper) Region Descriptors */
		for (uint8_t i = 0; i < num_mwrap; i++) {
			if (bcma_scan_erom_port_regions(bus, i, EROM_REGION_TYPE_MWRAP, erom_res, erom_base, &offset))
				return (EINVAL);
		}
		
		/* Slave (Wrapper) Region Descriptors. */
		swrap_port_base = 1; /* hardware bug? */
		if (num_sport == 1)
			swrap_port_base = 0;
		
		for (uint8_t i = 0; i < num_swrap; i++) {
			if (bcma_scan_erom_port_regions(bus, i + swrap_port_base, EROM_REGION_TYPE_SWRAP, erom_res, erom_base, &offset))
				return (EINVAL);
		}
	}
	
	return (0);
}