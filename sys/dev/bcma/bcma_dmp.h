/*
 * Broadcom HND "Device Management Plugin" Register Constants
 * 
 * This file was derived from the aidmp.h header provided with
 * Broadcom's initial brcm80211 Linux driver release, as
 * contributed to the Linux staging repository. 
 * 
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

/* Enumeration ROM registers */
#define	ER_EROMENTRY		0x000
#define	ER_REMAPCONTROL		0xe00
#define	ER_REMAPSELECT		0xe04
#define	ER_MASTERSELECT		0xe10
#define	ER_ITCR			0xf00
#define	ER_ITIP			0xf04

#ifndef _LANGUAGE_ASSEMBLY

typedef volatile struct _aidmp {
	u32 oobselina30;	/* 0x000 */
	u32 oobselina74;	/* 0x004 */
	u32 PAD[6];
	u32 oobselinb30;	/* 0x020 */
	u32 oobselinb74;	/* 0x024 */
	u32 PAD[6];
	u32 oobselinc30;	/* 0x040 */
	u32 oobselinc74;	/* 0x044 */
	u32 PAD[6];
	u32 oobselind30;	/* 0x060 */
	u32 oobselind74;	/* 0x064 */
	u32 PAD[38];
	u32 oobselouta30;	/* 0x100 */
	u32 oobselouta74;	/* 0x104 */
	u32 PAD[6];
	u32 oobseloutb30;	/* 0x120 */
	u32 oobseloutb74;	/* 0x124 */
	u32 PAD[6];
	u32 oobseloutc30;	/* 0x140 */
	u32 oobseloutc74;	/* 0x144 */
	u32 PAD[6];
	u32 oobseloutd30;	/* 0x160 */
	u32 oobseloutd74;	/* 0x164 */
	u32 PAD[38];
	u32 oobsynca;	/* 0x200 */
	u32 oobseloutaen;	/* 0x204 */
	u32 PAD[6];
	u32 oobsyncb;	/* 0x220 */
	u32 oobseloutben;	/* 0x224 */
	u32 PAD[6];
	u32 oobsyncc;	/* 0x240 */
	u32 oobseloutcen;	/* 0x244 */
	u32 PAD[6];
	u32 oobsyncd;	/* 0x260 */
	u32 oobseloutden;	/* 0x264 */
	u32 PAD[38];
	u32 oobaextwidth;	/* 0x300 */
	u32 oobainwidth;	/* 0x304 */
	u32 oobaoutwidth;	/* 0x308 */
	u32 PAD[5];
	u32 oobbextwidth;	/* 0x320 */
	u32 oobbinwidth;	/* 0x324 */
	u32 oobboutwidth;	/* 0x328 */
	u32 PAD[5];
	u32 oobcextwidth;	/* 0x340 */
	u32 oobcinwidth;	/* 0x344 */
	u32 oobcoutwidth;	/* 0x348 */
	u32 PAD[5];
	u32 oobdextwidth;	/* 0x360 */
	u32 oobdinwidth;	/* 0x364 */
	u32 oobdoutwidth;	/* 0x368 */
	u32 PAD[37];
	u32 ioctrlset;	/* 0x400 */
	u32 ioctrlclear;	/* 0x404 */
	u32 ioctrl;		/* 0x408 */
	u32 PAD[61];
	u32 iostatus;	/* 0x500 */
	u32 PAD[127];
	u32 ioctrlwidth;	/* 0x700 */
	u32 iostatuswidth;	/* 0x704 */
	u32 PAD[62];
	u32 resetctrl;	/* 0x800 */
	u32 resetstatus;	/* 0x804 */
	u32 resetreadid;	/* 0x808 */
	u32 resetwriteid;	/* 0x80c */
	u32 PAD[60];
	u32 errlogctrl;	/* 0x900 */
	u32 errlogdone;	/* 0x904 */
	u32 errlogstatus;	/* 0x908 */
	u32 errlogaddrlo;	/* 0x90c */
	u32 errlogaddrhi;	/* 0x910 */
	u32 errlogid;	/* 0x914 */
	u32 errloguser;	/* 0x918 */
	u32 errlogflags;	/* 0x91c */
	u32 PAD[56];
	u32 intstatus;	/* 0xa00 */
	u32 PAD[127];
	u32 config;		/* 0xe00 */
	u32 PAD[63];
	u32 itcr;		/* 0xf00 */
	u32 PAD[3];
	u32 itipooba;	/* 0xf10 */
	u32 itipoobb;	/* 0xf14 */
	u32 itipoobc;	/* 0xf18 */
	u32 itipoobd;	/* 0xf1c */
	u32 PAD[4];
	u32 itipoobaout;	/* 0xf30 */
	u32 itipoobbout;	/* 0xf34 */
	u32 itipoobcout;	/* 0xf38 */
	u32 itipoobdout;	/* 0xf3c */
	u32 PAD[4];
	u32 itopooba;	/* 0xf50 */
	u32 itopoobb;	/* 0xf54 */
	u32 itopoobc;	/* 0xf58 */
	u32 itopoobd;	/* 0xf5c */
	u32 PAD[4];
	u32 itopoobain;	/* 0xf70 */
	u32 itopoobbin;	/* 0xf74 */
	u32 itopoobcin;	/* 0xf78 */
	u32 itopoobdin;	/* 0xf7c */
	u32 PAD[4];
	u32 itopreset;	/* 0xf90 */
	u32 PAD[15];
	u32 peripherialid4;	/* 0xfd0 */
	u32 peripherialid5;	/* 0xfd4 */
	u32 peripherialid6;	/* 0xfd8 */
	u32 peripherialid7;	/* 0xfdc */
	u32 peripherialid0;	/* 0xfe0 */
	u32 peripherialid1;	/* 0xfe4 */
	u32 peripherialid2;	/* 0xfe8 */
	u32 peripherialid3;	/* 0xfec */
	u32 componentid0;	/* 0xff0 */
	u32 componentid1;	/* 0xff4 */
	u32 componentid2;	/* 0xff8 */
	u32 componentid3;	/* 0xffc */
} aidmp_t;

#endif				/* _LANGUAGE_ASSEMBLY */

/* Out-of-band Router registers */
#define	OOB_BUSCONFIG		0x020
#define	OOB_STATUSA		0x100
#define	OOB_STATUSB		0x104
#define	OOB_STATUSC		0x108
#define	OOB_STATUSD		0x10c
#define	OOB_ENABLEA0		0x200
#define	OOB_ENABLEA1		0x204
#define	OOB_ENABLEA2		0x208
#define	OOB_ENABLEA3		0x20c
#define	OOB_ENABLEB0		0x280
#define	OOB_ENABLEB1		0x284
#define	OOB_ENABLEB2		0x288
#define	OOB_ENABLEB3		0x28c
#define	OOB_ENABLEC0		0x300
#define	OOB_ENABLEC1		0x304
#define	OOB_ENABLEC2		0x308
#define	OOB_ENABLEC3		0x30c
#define	OOB_ENABLED0		0x380
#define	OOB_ENABLED1		0x384
#define	OOB_ENABLED2		0x388
#define	OOB_ENABLED3		0x38c
#define	OOB_ITCR		0xf00
#define	OOB_ITIPOOBA		0xf10
#define	OOB_ITIPOOBB		0xf14
#define	OOB_ITIPOOBC		0xf18
#define	OOB_ITIPOOBD		0xf1c
#define	OOB_ITOPOOBA		0xf30
#define	OOB_ITOPOOBB		0xf34
#define	OOB_ITOPOOBC		0xf38
#define	OOB_ITOPOOBD		0xf3c

/* DMP wrapper registers */
#define	AI_OOBSELINA30		0x000
#define	AI_OOBSELINA74		0x004
#define	AI_OOBSELINB30		0x020
#define	AI_OOBSELINB74		0x024
#define	AI_OOBSELINC30		0x040
#define	AI_OOBSELINC74		0x044
#define	AI_OOBSELIND30		0x060
#define	AI_OOBSELIND74		0x064
#define	AI_OOBSELOUTA30		0x100
#define	AI_OOBSELOUTA74		0x104
#define	AI_OOBSELOUTB30		0x120
#define	AI_OOBSELOUTB74		0x124
#define	AI_OOBSELOUTC30		0x140
#define	AI_OOBSELOUTC74		0x144
#define	AI_OOBSELOUTD30		0x160
#define	AI_OOBSELOUTD74		0x164
#define	AI_OOBSYNCA		0x200
#define	AI_OOBSELOUTAEN		0x204
#define	AI_OOBSYNCB		0x220
#define	AI_OOBSELOUTBEN		0x224
#define	AI_OOBSYNCC		0x240
#define	AI_OOBSELOUTCEN		0x244
#define	AI_OOBSYNCD		0x260
#define	AI_OOBSELOUTDEN		0x264
#define	AI_OOBAEXTWIDTH		0x300
#define	AI_OOBAINWIDTH		0x304
#define	AI_OOBAOUTWIDTH		0x308
#define	AI_OOBBEXTWIDTH		0x320
#define	AI_OOBBINWIDTH		0x324
#define	AI_OOBBOUTWIDTH		0x328
#define	AI_OOBCEXTWIDTH		0x340
#define	AI_OOBCINWIDTH		0x344
#define	AI_OOBCOUTWIDTH		0x348
#define	AI_OOBDEXTWIDTH		0x360
#define	AI_OOBDINWIDTH		0x364
#define	AI_OOBDOUTWIDTH		0x368

#if	defined(IL_BIGENDIAN) && defined(BCMHND74K)
/* Selective swapped defines for those registers we need in
 * big-endian code.
 */
#define	AI_IOCTRLSET		0x404
#define	AI_IOCTRLCLEAR		0x400
#define	AI_IOCTRL		0x40c
#define	AI_IOSTATUS		0x504
#define	AI_RESETCTRL		0x804
#define	AI_RESETSTATUS		0x800

#else				/* !IL_BIGENDIAN || !BCMHND74K */

#define	AI_IOCTRLSET		0x400
#define	AI_IOCTRLCLEAR		0x404
#define	AI_IOCTRL		0x408
#define	AI_IOSTATUS		0x500
#define	AI_RESETCTRL		0x800
#define	AI_RESETSTATUS		0x804

#endif				/* IL_BIGENDIAN && BCMHND74K */

#define	AI_IOCTRLWIDTH		0x700
#define	AI_IOSTATUSWIDTH	0x704

#define	AI_RESETREADID		0x808
#define	AI_RESETWRITEID		0x80c
#define	AI_ERRLOGCTRL		0xa00
#define	AI_ERRLOGDONE		0xa04
#define	AI_ERRLOGSTATUS		0xa08
#define	AI_ERRLOGADDRLO		0xa0c
#define	AI_ERRLOGADDRHI		0xa10
#define	AI_ERRLOGID		0xa14
#define	AI_ERRLOGUSER		0xa18
#define	AI_ERRLOGFLAGS		0xa1c
#define	AI_INTSTATUS		0xa00
#define	AI_CONFIG		0xe00
#define	AI_ITCR			0xf00
#define	AI_ITIPOOBA		0xf10
#define	AI_ITIPOOBB		0xf14
#define	AI_ITIPOOBC		0xf18
#define	AI_ITIPOOBD		0xf1c
#define	AI_ITIPOOBAOUT		0xf30
#define	AI_ITIPOOBBOUT		0xf34
#define	AI_ITIPOOBCOUT		0xf38
#define	AI_ITIPOOBDOUT		0xf3c
#define	AI_ITOPOOBA		0xf50
#define	AI_ITOPOOBB		0xf54
#define	AI_ITOPOOBC		0xf58
#define	AI_ITOPOOBD		0xf5c
#define	AI_ITOPOOBAIN		0xf70
#define	AI_ITOPOOBBIN		0xf74
#define	AI_ITOPOOBCIN		0xf78
#define	AI_ITOPOOBDIN		0xf7c
#define	AI_ITOPRESET		0xf90
#define	AI_PERIPHERIALID4	0xfd0
#define	AI_PERIPHERIALID5	0xfd4
#define	AI_PERIPHERIALID6	0xfd8
#define	AI_PERIPHERIALID7	0xfdc
#define	AI_PERIPHERIALID0	0xfe0
#define	AI_PERIPHERIALID1	0xfe4
#define	AI_PERIPHERIALID2	0xfe8
#define	AI_PERIPHERIALID3	0xfec
#define	AI_COMPONENTID0		0xff0
#define	AI_COMPONENTID1		0xff4
#define	AI_COMPONENTID2		0xff8
#define	AI_COMPONENTID3		0xffc

/* resetctrl */
#define	AIRC_RESET		1

/* config */
#define	AICFG_OOB		0x00000020
#define	AICFG_IOS		0x00000010
#define	AICFG_IOC		0x00000008
#define	AICFG_TO		0x00000004
#define	AICFG_ERRL		0x00000002
#define	AICFG_RST		0x00000001

#endif /* _BCMA_BCMA_DMP_H_ */
