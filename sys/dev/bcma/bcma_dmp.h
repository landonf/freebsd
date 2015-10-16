/*
 * Broadcom HND "Device Management Plugin" Register Constants
 * 
 * Portions of this file were derived from the aidmp.h header
 * provided with Broadcom's initial brcm80211 Linux driver release, as
 * contributed to the Linux staging repository. 
 * 
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef	_BCMA_BCMA_DMP_H_
#define	_BCMA_BCMA_DMP_H_

/*
 * PL-368 Device Management Plugin (DMP) Registers & Constants
 * 
 * The "DMP" core used in Broadcom HND devices has been described
 * by Broadcom engineers (and in published header files) as being
 * ARM's PL-368 "Device Management Plugin" system IP, included with
 * the CoreLink AMBA Designer tooling.
 * 
 * Documentation for the PL-368 is not publicly available, however,
 * and the only public reference by ARM to its existence appears to be
 * in the proprietary "NIC-301 Interconnect Device Management (PL368)"
 * errata publication, available to licensees as part of ARM's
 * CoreLink Controllers and Peripherals Engineering Errata.
 */

/* Enumeration ROM registers */
#define	BCMA_EROM_TABLE			0x000	/* device enumeration table offset */
#define	BCMA_EROM_REMAPCONTROL		0xe00
#define	BCMA_EROM_REMAPSELECT		0xe04
#define	BCMA_EROM_MASTERSELECT		0xe10
#define	BCMA_EROM_ITCR			0xf00
#define	BCMA_EROM_ITIP			0xf04
#define	BCMA_EROM_TABLE_END		BCMA_EROM_REMAPCONTROL
#define	BCMA_EROM_TABLE_SIZE		BMCA_EROM_TABLE_END - BCMA_EROM_TABLE

/*
 * Enumeration ROM Constants
 */
#define	BCMA_EROM_TABLE_EOF		0xF		/* end of EROM table */

#define	BCMA_EROM_ENTRY_ISVALID_MASK	0x1		/* is entry valid? */
#define	BCMA_EROM_ENTRY_ISVALID_SHIFT	0

/* EROM Entry Types */
#define	BCMA_EROM_ENTRY_TYPE_MASK	0x6		/* entry type mask */
#define	BCMA_EROM_ENTRY_TYPE_SHIFT	0
#  define BCMA_EROM_ENTRY_TYPE_CORE	0x0		/* core descriptor */
#  define BCMA_EROM_ENTRY_TYPE_MPORT	0x2		/* master port descriptor */
#  define BCMA_EROM_ENTRY_TYPE_REGION	0x4		/* address region descriptor */

/* EROM Core DescriptorA (31:0) */
#define	BCMA_EROM_COREA_DESIGNER_MASK	0xFFF00000	/* core designer (JEP-106 mfg id) */
#define	BCMA_EROM_COREA_DESIGNER_SHIFT	20
#define	BCMA_EROM_COREA_ID_MASK		0x000FFF00	/* broadcom-assigned core id */
#define	BCMA_EROM_COREA_ID_SHIFT	8
#define	BCMA_EROM_COREA_CLASS_MASK	0x000000F0	/* core class */
#define	BCMA_EROM_COREA_CLASS_SHIFT	4

/* EROM Core DescriptorB (63:32) */
#define	BCMA_EROM_COREB_NUM_MP_MASK	0x0000001F	/* master port count */
#define	BCMA_EROM_COREB_NUM_MP_SHIFT	4
#define	BCMA_EROM_COREB_NUM_SP_MASK	0x00003E00	/* slave port count */
#define	BCMA_EROM_COREB_NUM_SP_SHIFT	9
#define	BCMA_EROM_COREB_NUM_WMP_MASK	0x0007C000	/* master wrapper port count */
#define	BCMA_EROM_COREB_NUM_WMP_SHIFT	14
#define	BCMA_EROM_COREB_NUM_WSP_MASK	0x00F80000	/* slave wrapper port count */
#define	BCMA_EROM_COREB_NUM_WSP_SHIFT	19
#define	BCMA_EROM_COREB_REV_MASK	0xFF000000	/* broadcom-assigned core revision */
#define	BCMA_EROM_COREB_REV_SHIFT	24

/* EROM Master Port Descriptor */
#define	BCMA_EROM_MPORT_NUM_MASK	0x0000FF00	/* port number */
#define	BCMA_EROM_MPORT_NUM_SHIFT	8

/* EROM Region Descriptor */
#define	BCMA_EROM_REGION_BASE_MASK	0xFFFFF000	/* region base address */
#define	BCMA_EROM_REGION_BASE_SHIFT	0
#define	BCMA_EROM_REGION_64BIT_MASK	0x00000008	/* base address spans two 32-bit entries */
#define	BCMA_EROM_REGION_64BIT_SHIFT	0
#define	BCMA_EROM_REGION_PORT_MASK	0x00000F00	/* region's associated port */
#define	BCMA_EROM_REGION_PORT_SHIFT	8
#define	BCMA_EROM_REGION_TYPE_MASK	0x000000C0	/* region type */
#define	BCMA_EROM_REGION_TYPE_SHIFT	6
#define	  BCMA_EROM_REGION_TYPE_SLAVE	0		/* slave */
#define	  BCMA_EROM_REGION_TYPE_BRIDGE	1		/* bridge */
#define	  BCMA_EROM_REGION_TYPE_SWRAP	2		/* slave wrapper */
#define	  BCMA_EROM_REGION_TYPE_MWRAP	3		/* master wrapper */

#define	BCMA_EROM_REGION_SIZE_MASK	0x00000030	/* region size encoding */
#define	BCMA_EROM_REGION_SIZE_SHIFT	4
#define	  BCMA_EROM_REGION_SIZE_4K	0		/* 4K region */
#define	  BCMA_EROM_REGION_SIZE_8K	1		/* 8K region */
#define	  BCMA_EROM_REGION_SIZE_16K	2		/* 16K region */
#define	  BCMA_EROM_REGION_SIZE_OTHER	3		/* defined by an additional size descriptor entry. */
#define	BCMA_EROM_REGION_SIZE_BASE	0x1000

/* Region Size Descriptor */
#define	BCMA_EROM_RSIZE_VAL_MASK	0xFFFFF000	/* region size */
#define	BCMA_EROM_RSIZE_VAL_SHIFT	0
#define	BCMA_EROM_RSIZE_64BIT_MASK	0x00000008	/* size spans two 32-bit entries */
#define	BCMA_EROM_RSIZE_64BIT_SHIFT	0

/* Out-of-band Router registers */
#define	BCMA_OOB_BUSCONFIG	0x020
#define	BCMA_OOB_STATUSA	0x100
#define	BCMA_OOB_STATUSB	0x104
#define	BCMA_OOB_STATUSC	0x108
#define	BCMA_OOB_STATUSD	0x10c
#define	BCMA_OOB_ENABLEA0	0x200
#define	BCMA_OOB_ENABLEA1	0x204
#define	BCMA_OOB_ENABLEA2	0x208
#define	BCMA_OOB_ENABLEA3	0x20c
#define	BCMA_OOB_ENABLEB0	0x280
#define	BCMA_OOB_ENABLEB1	0x284
#define	BCMA_OOB_ENABLEB2	0x288
#define	BCMA_OOB_ENABLEB3	0x28c
#define	BCMA_OOB_ENABLEC0	0x300
#define	BCMA_OOB_ENABLEC1	0x304
#define	BCMA_OOB_ENABLEC2	0x308
#define	BCMA_OOB_ENABLEC3	0x30c
#define	BCMA_OOB_ENABLED0	0x380
#define	BCMA_OOB_ENABLED1	0x384
#define	BCMA_OOB_ENABLED2	0x388
#define	BCMA_OOB_ENABLED3	0x38c
#define	BCMA_OOB_ITCR		0xf00
#define	BCMA_OOB_ITIPOOBA	0xf10
#define	BCMA_OOB_ITIPOOBB	0xf14
#define	BCMA_OOB_ITIPOOBC	0xf18
#define	BCMA_OOB_ITIPOOBD	0xf1c
#define	BCMA_OOB_ITOPOOBA	0xf30
#define	BCMA_OOB_ITOPOOBB	0xf34
#define	BCMA_OOB_ITOPOOBC	0xf38
#define	BCMA_OOB_ITOPOOBD	0xf3c

/* DMP wrapper registers */
#define	BCMA_DMP_OOBSELINA30	0x000
#define	BCMA_DMP_OOBSELINA74	0x004
#define	BCMA_DMP_OOBSELINB30	0x020
#define	BCMA_DMP_OOBSELINB74	0x024
#define	BCMA_DMP_OOBSELINC30	0x040
#define	BCMA_DMP_OOBSELINC74	0x044
#define	BCMA_DMP_OOBSELIND30	0x060
#define	BCMA_DMP_OOBSELIND74	0x064
#define	BCMA_DMP_OOBSELOUTA30	0x100
#define	BCMA_DMP_OOBSELOUTA74	0x104
#define	BCMA_DMP_OOBSELOUTB30	0x120
#define	BCMA_DMP_OOBSELOUTB74	0x124
#define	BCMA_DMP_OOBSELOUTC30	0x140
#define	BCMA_DMP_OOBSELOUTC74	0x144
#define	BCMA_DMP_OOBSELOUTD30	0x160
#define	BCMA_DMP_OOBSELOUTD74	0x164
#define	BCMA_DMP_OOBSYNCA	0x200
#define	BCMA_DMP_OOBSELOUTAEN	0x204
#define	BCMA_DMP_OOBSYNCB	0x220
#define	BCMA_DMP_OOBSELOUTBEN	0x224
#define	BCMA_DMP_OOBSYNCC	0x240
#define	BCMA_DMP_OOBSELOUTCEN	0x244
#define	BCMA_DMP_OOBSYNCD	0x260
#define	BCMA_DMP_OOBSELOUTDEN	0x264
#define	BCMA_DMP_OOBAEXTWIDTH	0x300
#define	BCMA_DMP_OOBAINWIDTH	0x304
#define	BCMA_DMP_OOBAOUTWIDTH	0x308
#define	BCMA_DMP_OOBBEXTWIDTH	0x320
#define	BCMA_DMP_OOBBINWIDTH	0x324
#define	BCMA_DMP_OOBBOUTWIDTH	0x328
#define	BCMA_DMP_OOBCEXTWIDTH	0x340
#define	BCMA_DMP_OOBCINWIDTH	0x344
#define	BCMA_DMP_OOBCOUTWIDTH	0x348
#define	BCMA_DMP_OOBDEXTWIDTH	0x360
#define	BCMA_DMP_OOBDINWIDTH	0x364
#define	BCMA_DMP_OOBDOUTWIDTH	0x368

// This was inherited from Broadcom's aidmp.h header
// Is it required for any of our use-cases?
#if 0 /* defined(IL_BIGENDIAN) && defined(BCMHND74K) */
/* Selective swapped defines for those registers we need in
 * big-endian code.
 */
#define	BCMA_DMP_IOCTRLSET	0x404
#define	BCMA_DMP_IOCTRLCLEAR	0x400
#define	BCMA_DMP_IOCTRL		0x40c
#define	BCMA_DMP_IOSTATUS	0x504
#define	BCMA_DMP_RESETCTRL	0x804
#define	BCMA_DMP_RESETSTATUS	0x800

#else				/* !IL_BIGENDIAN || !BCMHND74K */

#define	BCMA_DMP_IOCTRLSET	0x400
#define	BCMA_DMP_IOCTRLCLEAR	0x404
#define	BCMA_DMP_IOCTRL		0x408
#define	BCMA_DMP_IOSTATUS	0x500
#define	BCMA_DMP_RESETCTRL	0x800
#define	BCMA_DMP_RESETSTATUS	0x804

#endif				/* IL_BIGENDIAN && BCMHND74K */

#define	BCMA_DMP_IOCTRLWIDTH	0x700
#define	BCMA_DMP_IOSTATUSWIDTH	0x704

#define	BCMA_DMP_RESETREADID	0x808
#define	BCMA_DMP_RESETWRITEID	0x80c
#define	BCMA_DMP_ERRLOGCTRL	0xa00
#define	BCMA_DMP_ERRLOGDONE	0xa04
#define	BCMA_DMP_ERRLOGSTATUS	0xa08
#define	BCMA_DMP_ERRLOGADDRLO	0xa0c
#define	BCMA_DMP_ERRLOGADDRHI	0xa10
#define	BCMA_DMP_ERRLOGID	0xa14
#define	BCMA_DMP_ERRLOGUSER	0xa18
#define	BCMA_DMP_ERRLOGFLAGS	0xa1c
#define	BCMA_DMP_INTSTATUS	0xa00
#define	BCMA_DMP_CONFIG		0xe00
#define	BCMA_DMP_ITCR		0xf00
#define	BCMA_DMP_ITIPOOBA	0xf10
#define	BCMA_DMP_ITIPOOBB	0xf14
#define	BCMA_DMP_ITIPOOBC	0xf18
#define	BCMA_DMP_ITIPOOBD	0xf1c
#define	BCMA_DMP_ITIPOOBAOUT	0xf30
#define	BCMA_DMP_ITIPOOBBOUT	0xf34
#define	BCMA_DMP_ITIPOOBCOUT	0xf38
#define	BCMA_DMP_ITIPOOBDOUT	0xf3c
#define	BCMA_DMP_ITOPOOBA	0xf50
#define	BCMA_DMP_ITOPOOBB	0xf54
#define	BCMA_DMP_ITOPOOBC	0xf58
#define	BCMA_DMP_ITOPOOBD	0xf5c
#define	BCMA_DMP_ITOPOOBAIN	0xf70
#define	BCMA_DMP_ITOPOOBBIN	0xf74
#define	BCMA_DMP_ITOPOOBCIN	0xf78
#define	BCMA_DMP_ITOPOOBDIN	0xf7c
#define	BCMA_DMP_ITOPRESET	0xf90
#define	BCMA_DMP_PERIPHERIALID4	0xfd0
#define	BCMA_DMP_PERIPHERIALID5	0xfd4
#define	BCMA_DMP_PERIPHERIALID6	0xfd8
#define	BCMA_DMP_PERIPHERIALID7	0xfdc
#define	BCMA_DMP_PERIPHERIALID0	0xfe0
#define	BCMA_DMP_PERIPHERIALID1	0xfe4
#define	BCMA_DMP_PERIPHERIALID2	0xfe8
#define	BCMA_DMP_PERIPHERIALID3	0xfec
#define	BCMA_DMP_COMPONENTID0	0xff0
#define	BCMA_DMP_COMPONENTID1	0xff4
#define	BCMA_DMP_COMPONENTID2	0xff8
#define	BCMA_DMP_COMPONENTID3	0xffc

/* resetctrl */
#define	BMCA_DMP_RC_RESET	1

/* config */
#define	BCMA_DMP_CFG_OOB	0x00000020
#define	BCMA_DMP_CFG_IOS	0x00000010
#define	BCMA_DMP_CFGIOC		0x00000008
#define	BCMA_DMP_CFGTO		0x00000004
#define	BCMA_DMP_CFGERRL	0x00000002
#define	BCMA_DMP_CFGRST		0x00000001

#endif /* _BCMA_BCMA_DMP_H_ */
