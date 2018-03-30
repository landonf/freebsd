/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2018 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2015 Broadcom Corporation. All Rights Reserved.
 * All rights reserved.
 *
 * This file was derived from the sbhnddma.h header distributed with the Asus
 * RT-N18 firmware source code release.
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
 * $Id: sbhnddma.h 419467 2013-08-21 09:19:48Z $
 *
 * $FreeBSD$
 */

#ifndef _BHND_DMA_BHND_DMAREG_H_
#define _BHND_DMA_BHND_DMAREG_H_

/*
 * Common 32-bit/64-bit register definitions
 */

/**
 * Evaluates to the register offset of the DMA channel @p _num with direction
 * @p _dir.
 * 
 * @param _regf The DMA register format (@see bhnd_dma_regfmt).
 * @param _dir The DMA channel direction (@see bnnd_dma_direction).
 * @param _num The DMA channel number.
 */
#define	BHND_DMA_CHAN_OFFSET(_regf, _dir, _num)	(			\
	(_regf) == BHND_DMA_REGFMT32 ?					\
	     (BHND_D32_CHAN_OFFSET((_dir), (_num))) :			\
	     (BHND_D64_CHAN_OFFSET((_dir), (_num)))			\
)

/**
 * Evaluates to the value of a common DMA register definition. 
 * 
 * This will trigger a compile-time error if the register is not defined
 * for all supported DMA register layouts.
 * 
 * This should be optimized down to a constant value if the register constant
 * is the same across the register definitions.
 * 
 * @param _regf	The DMA register format (@see bhnd_dma_regfmt), or a
 *		bhnd_dma_chan instance.
 * @param _name The DMA register name's common suffix.
 */
#define	BHND_DMA_REGF(_regf, _name)	(				\
	(_regf) == BHND_DMA_REGFMT32 ? (BHND_D32_ ## _name) :		\
	(BHND_D64_ ## _name)						\
)

/**
 * Evaluates to the value of a common DMA register definition. 
 * 
 * This will trigger a compile-time error if the register is not defined
 * for all supported DMA register layouts.
 * 
 * This should be optimized down to a constant value if the register constant
 * is the same across the register definitions.
 * 
 * @param _chan	A DMA channel instance
 * @param _name The DMA register name's common suffix.
 */
#define	BHND_DMA_REG(_chan, _name)	(				\
	(_chan)->dma->regfmt == BHND_DMA_REGFMT32 ?			\
	    (BHND_D32_ ## _name) : (BHND_D64_ ## _name)			\
)

#define BHND_DMA_GET_FLAG(_chan, _value, _flag)				\
	(((_value) & BHND_DMA_REG((_chan), _flag) != 0)

#define	BHND_DMA_GET_BITS(_chan, _value, _name)				\
	((_value & ((_chan)->dma->regfmt == BHND_DMA_REGFMT32 ?		\
	    (BHND_D32_ ## _name ## _MASK) :				\
	    (BHND_D64_ ## _name ## _MASK))) >>				\
	    ((_chan)->dma->regfmt == BHND_DMA_REGFMT32 ?		\
		(BHND_D32_ ## _name ## _SHIFT) :			\
		(BHND_D64_ ## _name ## _SHIFT)))

/*
 * Per-channel register access macros.
 */ 
#define	_BHND_DMA_WRITE_N(_size, _chan, _reg, _val)		\
	bus_space_write_ ## _size ((_chan)->dma->regs_bst,	\
	    (_chan)->bsh, (_reg), (_val))
        
#define	_BHND_DMA_READ_N(_size, _chan, _reg)			\
	bus_space_read_ ## _size ((_chan)->dma->regs_bst, (_chan)->bsh, (_reg))

#define	BHND_DMA_WRITE_4(_chan, _reg, _val)			\
	_BHND_DMA_WRITE_N(4, (_chan), (_reg), (_val))
#define	BHND_DMA_READ_4(_chan, _reg)				\
	_BHND_DMA_READ_N(4, (_chan), (_reg))

#define	BHND_DMA_WRITE_2(_chan, _reg, _val)			\
	_BHND_DMA_WRITE_N(2, (_chan), (_reg), (_val))
#define	BHND_DMA_READ_2(_chan, _reg)				\
	_BHND_DMA_READ_N(2, (_chan), (_reg))

#define	BHND_DMA_WRITE_1(_chan, _reg, _val)			\
	_BHND_DMA_WRITE_N(1, (_chan), (_reg), (_val))
#define	BHND_DMA_READ_1(_chan, _reg)				\
	_BHND_DMA_READ_N(1, (_chan), (_reg))

#define	BHND_DMA_BARRIER(_chan, _offset, _size, _flags)		\
	bus_space_barrier((_chan)->dma->regs_bst, (_chan)->bsh,	\
	    (_offset), (_size), (_flags))

/* Multiple outstanding reads */
#define	BHND_DMA_MR_1		0
#define	BHND_DMA_MR_2		1
/* 2, 3: reserved */

/* DMA Burst Length in bytes */
#define	BHND_DMA_BL_16		0
#define	BHND_DMA_BL_32		1
#define	BHND_DMA_BL_64		2
#define	BHND_DMA_BL_128		3
#define	BHND_DMA_BL_256		4
#define	BHND_DMA_BL_512		5
#define	BHND_DMA_BL_1024	6

/* Prefetch control */
#define	BHND_DMA_PC_0		0
#define	BHND_DMA_PC_4		1
#define	BHND_DMA_PC_8		2
#define	BHND_DMA_PC_16		3
/* others: reserved */

/* Prefetch threshold */
#define	BHND_DMA_PT_1		0
#define	BHND_DMA_PT_2		1
#define	BHND_DMA_PT_4		2
#define	BHND_DMA_PT_8		3

#endif /* _BHND_DMA_BHND_DMAREG_H_ */
