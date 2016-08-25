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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_BHND_INTR_H_
#define _BHND_BHND_INTR_H_

#include <sys/_bitset.h>
#include <sys/bitset.h>

/**
 * bhnd(4) interrupt line ports.
 */
typedef enum {
	BHND_INTR_INPUT		= 0,	/**< input port */
	BHND_INTR_OUTPUT	= 1,	/**< output port */
} bhnd_intr_port;

/** Maximum backplane interrupt vector value */
#define	BHND_MAX_INTRVEC	32

/** bhnd(4) interrupt vector set size */
#define	BHND_IVECSET_SIZE	BHND_MAX_INTRVEC

/** bhnd(4) interrupt vector set. */
typedef BITSET_DEFINE(bhnd_intrvec_set, BHND_IVECSET_SIZE) bhnd_intrvec_set_t;

#define	BHND_IVECSET_INITIALIZER	BITSET_T_INITIALIZER

#define	BHND_IVECSET_CLR(n, p)		BIT_CLR(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_COPY(f, t)		BIT_COPY(BHND_IVECSET_SIZE, f, t)
#define	BHND_IVECSET_ISSET(n, p)	BIT_ISSET(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_SET(n, p)		BIT_SET(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_ZERO(p) 		BIT_ZERO(BHND_IVECSET_SIZE, p)
#define	BHND_IVECSET_FILL(p) 		BIT_FILL(BHND_IVECSET_SIZE, p)
#define	BHND_IVECSET_SETOF(n, p)	BIT_SETOF(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_EMPTY(p)		BIT_EMPTY(BHND_IVECSET_SIZE, p)
#define	BHND_IVECSET_ISFULLSET(p)	BIT_ISFULLSET(BHND_IVECSET_SIZE, p)
#define	BHND_IVECSET_SUBSET(p, c)	BIT_SUBSET(BHND_IVECSET_SIZE, p, c)
#define	BHND_IVECSET_OVERLAP(p, c)	BIT_OVERLAP(BHND_IVECSET_SIZE, p, c)
#define	BHND_IVECSET_CMP(p, c)		BIT_CMP(BHND_IVECSET_SIZE, p, c)
#define	BHND_IVECSET_OR(d, s)		BIT_OR(BHND_IVECSET_SIZE, d, s)
#define	BHND_IVECSET_AND(d, s)		BIT_AND(BHND_IVECSET_SIZE, d, s)
#define	BHND_IVECSET_NAND(d, s)		BIT_NAND(BHND_IVECSET_SIZE, d, s)
#define	BHND_IVECSET_CLR_ATOMIC(n, p)	BIT_CLR_ATOMIC(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_SET_ATOMIC(n, p)	BIT_SET_ATOMIC(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_SET_ATOMIC_ACQ(n, p)	\
	BIT_SET_ATOMIC_ACQ(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_AND_ATOMIC(n, p)	BIT_AND_ATOMIC(BHND_IVECSET_SIZE, n, p)
#define	BHND_IVECSET_OR_ATOMIC(d, s)	BIT_OR_ATOMIC(BHND_IVECSET_SIZE, d, s)
#define	BHND_IVECSET_COPY_STORE_REL(f, t)	\
	BIT_COPY_STORE_REL(BHND_IVECSET_SIZE, f, t)
#define	BHND_IVECSET_FFS(p)		BIT_FFS(BHND_IVECSET_SIZE, p)
#define	BHND_IVECSET_COUNT(p)		BIT_COUNT(BHND_IVECSET_SIZE, p)
#define	BHND_IVECSET_FULL			\
	BITSET_FSET(__bitset_words(BHND_IVECSET_SIZE))


#endif /* _BHND_BHND_INTR_H_ */
