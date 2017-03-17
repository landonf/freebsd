/*-
 * Copyright (c) 2015-2017 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2010 Broadcom Corporation
 * All rights reserved.
 *
 * This file is derived from the sbchipc.h header contributed by Broadcom 
 * to to the Linux staging repository, as well as later revisions of sbchipc.h
 * distributed with the Asus RT-N16 firmware source code release.
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
 *
 * $FreeBSD$
 */

#ifndef _BHND_CORES_PMU_BHND_PMU0REG_H_
#define	_BHND_CORES_PMU_BHND_PMU0REG_H_

/*
 * PMU0 chip-specific PMU resources
 */

/* 5354 resources */
#define	BHND_PMU0_RES5354_EXT_SWITCHER_PWM	0	/* 0x00001 */
#define	BHND_PMU0_RES5354_BB_SWITCHER_PWM	1	/* 0x00002 */
#define	BHND_PMU0_RES5354_BB_SWITCHER_BURST	2	/* 0x00004 */
#define	BHND_PMU0_RES5354_BB_EXT_SWITCHER_BURST	3	/* 0x00008 */
#define	BHND_PMU0_RES5354_ILP_REQUEST		4	/* 0x00010 */
#define	BHND_PMU0_RES5354_RADIO_SWITCHER_PWM	5	/* 0x00020 */
#define	BHND_PMU0_RES5354_RADIO_SWITCHER_BURST	6	/* 0x00040 */
#define	BHND_PMU0_RES5354_ROM_SWITCH		7	/* 0x00080 */
#define	BHND_PMU0_RES5354_PA_REF_LDO		8	/* 0x00100 */
#define	BHND_PMU0_RES5354_RADIO_LDO		9	/* 0x00200 */
#define	BHND_PMU0_RES5354_AFE_LDO		10	/* 0x00400 */
#define	BHND_PMU0_RES5354_PLL_LDO		11	/* 0x00800 */
#define	BHND_PMU0_RES5354_BG_FILTBYP		12	/* 0x01000 */
#define	BHND_PMU0_RES5354_TX_FILTBYP		13	/* 0x02000 */
#define	BHND_PMU0_RES5354_RX_FILTBYP		14	/* 0x04000 */
#define	BHND_PMU0_RES5354_XTAL_PU		15	/* 0x08000 */
#define	BHND_PMU0_RES5354_XTAL_EN		16	/* 0x10000 */
#define	BHND_PMU0_RES5354_BB_PLL_FILTBYP	17	/* 0x20000 */
#define	BHND_PMU0_RES5354_RF_PLL_FILTBYP	18	/* 0x40000 */
#define	BHND_PMU0_RES5354_BB_PLL_PU		19	/* 0x80000 */

/* 4328 resources */
#define	BHND_PMU0_RES4328_EXT_SWITCHER_PWM	0	/* 0x00001 */
#define	BHND_PMU0_RES4328_BB_SWITCHER_PWM	1	/* 0x00002 */
#define	BHND_PMU0_RES4328_BB_SWITCHER_BURST	2	/* 0x00004 */
#define	BHND_PMU0_RES4328_BB_EXT_SWITCHER_BURST	3	/* 0x00008 */
#define	BHND_PMU0_RES4328_ILP_REQUEST		4	/* 0x00010 */
#define	BHND_PMU0_RES4328_RADIO_SWITCHER_PWM	5	/* 0x00020 */
#define	BHND_PMU0_RES4328_RADIO_SWITCHER_BURST	6	/* 0x00040 */
#define	BHND_PMU0_RES4328_ROM_SWITCH		7	/* 0x00080 */
#define	BHND_PMU0_RES4328_PA_REF_LDO		8	/* 0x00100 */
#define	BHND_PMU0_RES4328_RADIO_LDO		9	/* 0x00200 */
#define	BHND_PMU0_RES4328_AFE_LDO		10	/* 0x00400 */
#define	BHND_PMU0_RES4328_PLL_LDO		11	/* 0x00800 */
#define	BHND_PMU0_RES4328_BG_FILTBYP		12	/* 0x01000 */
#define	BHND_PMU0_RES4328_TX_FILTBYP		13	/* 0x02000 */
#define	BHND_PMU0_RES4328_RX_FILTBYP		14	/* 0x04000 */
#define	BHND_PMU0_RES4328_XTAL_PU		15	/* 0x08000 */
#define	BHND_PMU0_RES4328_XTAL_EN		16	/* 0x10000 */
#define	BHND_PMU0_RES4328_BB_PLL_FILTBYP	17	/* 0x20000 */
#define	BHND_PMU0_RES4328_RF_PLL_FILTBYP	18	/* 0x40000 */
#define	BHND_PMU0_RES4328_BB_PLL_PU		19	/* 0x80000 */

/*
 * PMU0 chip specific PLL controls.
 * 
 * PMU<rev>_PLL<num>_XX where <rev> is PMU corerev and <num> is an arbitrary
 * number to differentiate different PLLs controlled by the same PMU rev.
 */

/* pllcontrol registers */
/* PDIV, div_phy, div_arm, div_adc, dith_sel, ioff, kpd_scale, lsb_sel,
 * mash_sel, lf_c & lf_r */
#define	BHND_PMU0_PLL0_PLLCTL0			0
#define	BHND_PMU0_PLL0_PC0_PDIV_MASK		1
#define	BHND_PMU0_PLL0_PC0_PDIV_FREQ		25000
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_MASK		0x00000038
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_SHIFT	3
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_BASE		8

/* PC0_DIV_ARM for PLLOUT_ARM */
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_110MHZ	0
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_97_7MHZ	1
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_88MHZ	2
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_80MHZ	3	/* Default */
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_73_3MHZ	4
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_67_7MHZ	5
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_62_9MHZ	6
#define	BHND_PMU0_PLL0_PC0_DIV_ARM_58_6MHZ	7

/* Wildcard base, stop_mod, en_lf_tp, en_cal & lf_r2 */
#define	BHND_PMU0_PLL0_PLLCTL1			1
#define	BHND_PMU0_PLL0_PC1_WILD_INT_MASK	0xf0000000
#define	BHND_PMU0_PLL0_PC1_WILD_INT_SHIFT	28
#define	BHND_PMU0_PLL0_PC1_WILD_FRAC_MASK	0x0fffff00
#define	BHND_PMU0_PLL0_PC1_WILD_FRAC_SHIFT	8
#define	BHND_PMU0_PLL0_PC1_STOP_MOD		0x00000040

/* Wildcard base, vco_calvar, vco_swc, vco_var_selref,
 * vso_ical & vco_sel_avdd */
#define	BHND_PMU0_PLL0_PLLCTL2			2
#define	BHND_PMU0_PLL0_PC2_WILD_INT_MASK	0xf
#define	BHND_PMU0_PLL0_PC2_WILD_INT_SHIFT	4

#endif /* _BHND_CORES_PMU_BHND_PMU0REG_H_ */
