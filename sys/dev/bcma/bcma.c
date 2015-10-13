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
#include <dev/bhnd/bhnd_core.h>

#include "bcma.h"

// TODO
#include <sys/types.h>
#include <sys/systm.h>


/*
 * PL-368 "Device Management Plugin"
 * 
 * This enumeration table is not publicly documented; Broadcom
 * refers to the table as part of an undocumented ARM PL-368
 * "Device Management Plugin" IP component
 */

#define EROM_ENTRY_ISVALID		0x1	/* is entry valid? */
#define EROM_ENTRY_EOF			0xF	/* end of EROM table */

#define EROM_ENTRY_TYPE_MASK		0x6	/* opcode tag mask */
#  define EROM_ENTRY_CORE		0x0	/* core definition (64-bit) */
#  define EROM_ENTRY_PORT		0x2	/* port definition */
#  define EROM_ENTRY_ADDR		0x4	/* address (32 or 64-bit) definition */


#define EROM_CORE0_DESIGNER_MASK	0xFFF00000	/* core designer (JEP-106 mfg id) */
#define EROM_CORE0_DESIGNER_SHIFT	20
#define EROM_CORE0_ID_MASK		0x000FFF00	/* broadcom-assigned core id */
#define EROM_CORE0_ID_SHIFT		8

#define EROM_CORE1_MP_MASK		0x0000001F	/* ??? */
#define EROM_CORE1_MP_SHIFT		4;
#define EROM_CORE1_SP_MASK		0x00003E00	/* ??? */
#define EROM_CORE1_SP_SHIFT		9;
#define EROM_CORE1_WMP_MASK		0x0007C000	/* ??? */
#define EROM_CORE1_WMP_SHIFT		14
#define EROM_CORE1_WSP_MASK		0x00F80000	/* ??? */
#define EROM_CORE1_WSP_SHIFT		19
#define EROM_CORE1_REV_MASK		0xFF000000	/* broadcom-assigned core revision */
#define EROM_CORE1_REV_SHIFT		24

#define EROM_ADDR_64BIT			0x8	/* address spans two 32-bit opcodes */

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
	bus_size_t erom_end = offset + max_size;
	
	
	bus_size_t orig_loc = offset;
	bus_size_t comp_loc = orig_loc;

	while (offset != erom_end) {
		uint32_t entry = bus_read_4(erom_res, offset);
		
		if (entry == EROM_ENTRY_EOF)
			break;
		
		if ((entry & EROM_ENTRY_ISVALID) == 0) {
			device_printf(bus, "erom[0%lx] = 0x%x (marked invalid)\n", (offset-orig_loc)/4, entry);
			offset += 4;
			continue;
		}
		
		uint8_t type = entry & EROM_ENTRY_TYPE_MASK;
		const char *desc;
		switch (type) {
			case EROM_ENTRY_CORE:
				comp_loc = offset+4;
				desc = "component";
				break;
			case EROM_ENTRY_PORT:
				desc = "port";
				break;
			case EROM_ENTRY_ADDR:
				if (entry & EROM_ADDR_64BIT) {
					desc = "addr64";
				} else {
					desc = "addr";
				}

				break;
			default:
				desc = "UNKNOWN";
				break;
		}
		
		uint32_t remainder = entry & ~(EROM_ENTRY_ISVALID | EROM_ENTRY_TYPE_MASK);
		uint32_t sec_rem = 0;
		if (type == 0) {
			uint16_t designer = (entry & EROM_CORE0_DESIGNER_MASK) >> EROM_CORE0_DESIGNER_SHIFT;
			remainder &= ~(EROM_CORE0_DESIGNER_MASK);
			
			uint16_t part_num = (entry & EROM_CORE0_ID_MASK) >> EROM_CORE0_ID_SHIFT;
			remainder &= ~(EROM_CORE0_ID_MASK);

			const char *des;
			switch (designer) {
				case JEDEC_MFGID_ARM:
					des = "ARM";
					break;
				case JEDEC_MFGID_BCM:
					des = "BROADCOM";
					break;
				case JEDEC_MFGID_MIPS:
					des = "MIPS";
					break;
				default:
					des = "unknown";
					break;
			}
			
			offset += 4;
			uint32_t core1 = bus_read_4(erom_res, offset);
			if ((core1 & EROM_ENTRY_TYPE_MASK) != EROM_ENTRY_CORE)
				panic("Invalid next entry!");
			
			if ((core1 & EROM_ENTRY_ISVALID) == 0)
				panic("Missing entry IS_VALID flag!");
			
			sec_rem = core1 & ~(EROM_ENTRY_ISVALID | EROM_ENTRY_TYPE_MASK);
			uint8_t mp = (core1 & EROM_CORE1_MP_MASK) >> EROM_CORE1_MP_SHIFT;
			sec_rem &= ~EROM_CORE1_MP_MASK;
			
			uint8_t sp = (core1 & EROM_CORE1_SP_MASK) >> EROM_CORE1_SP_SHIFT;
			sec_rem &= ~EROM_CORE1_SP_MASK;

			uint8_t wmp = (core1 & EROM_CORE1_WMP_MASK) >> EROM_CORE1_WMP_SHIFT;
			sec_rem &= ~EROM_CORE1_WMP_MASK;

			uint8_t wsp = (core1 & EROM_CORE1_WSP_MASK) >> EROM_CORE1_WSP_SHIFT;
			sec_rem &= ~EROM_CORE1_WSP_MASK;

			uint8_t rev = (core1 & EROM_CORE1_REV_MASK) >> EROM_CORE1_REV_SHIFT;
			sec_rem &= ~EROM_CORE1_REV_MASK;
			
			device_printf(bus, "erom[%lx] = 0x%x (designer=%s, core=\"%s\" cid=%hx, rev=%hhx)\n", (offset-orig_loc)/4, entry, des, bhnd_core_name(designer, part_num), part_num, rev);
			device_printf(bus, "erom[%lx] m=%hhx\ts=%hhx\twm=%hhx\tws=%hhx\n", (offset-orig_loc)/4, mp, sp, wmp, wsp);

		} else if (type == EROM_ENTRY_ADDR && entry & EROM_ADDR_64BIT) {
			remainder &= ~(EROM_ENTRY_ADDR | EROM_ADDR_64BIT);
			entry >>= 4;
			
			device_printf(bus, "    [%lx] = 0x%x (%s)\n", (offset-comp_loc)/4, entry, desc);
			offset += 4;
			uint32_t entry = bus_read_4(erom_res, offset);
			device_printf(bus, "    [%lx] = 0x%x (add64-cont)\n", (offset-comp_loc)/4, entry);
		} else {
			remainder &= ~EROM_ENTRY_ADDR;
			entry >>= 4;
			
			device_printf(bus, "    [%lx] = 0x%x (%s)\n", (offset-comp_loc)/4, entry, desc);
		}
		
		if (remainder)
			device_printf(bus, "FLAGS_UNACCOUNTED_FOR=0x%x\n", remainder);
		
		if (sec_rem)
			device_printf(bus, "FLAGS2_UNACCOUNTED_FOR=0x%x\n", sec_rem);

		offset += 4;
	}
	
	return (0);
}