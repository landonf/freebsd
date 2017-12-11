/*	$NetBSD: bus.h,v 1.11 2003/07/28 17:35:54 thorpej Exp $	*/

/*-
 * SPDX-License-Identifier: BSD-2-Clause-NetBSD AND BSD-4-Clause
 *
 * Copyright (c) 1996, 1997, 1998, 2001 The NetBSD Foundation, Inc.
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2017 The FreeBSD Foundation
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center.
 * 
 * Portions of this software were developed by Landon Fuller
 * under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*-
 * Copyright (c) 1996 Charles M. Hannum.  All rights reserved.
 * Copyright (c) 1996 Christopher G. Demetriou.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Christopher G. Demetriou
 *	for the NetBSD Project.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _BHND_BHND_BUS_SPACE_H_
#define _BHND_BHND_BUS_SPACE_H_

#include <machine/bus.h>

struct bhnd_resource;

struct bhnd_bus_space {
	/* cookie */
	void		*bs_cookie;

	/* barrier */
	void		(*bs_barrier) (void *, struct bhnd_resource *,
			    bus_size_t, bus_size_t, int);

	/* read (single) */
	uint8_t		(*bs_r_1) (void *, struct bhnd_resource *, bus_size_t);
	uint16_t	(*bs_r_2) (void *, struct bhnd_resource *, bus_size_t);
	uint32_t	(*bs_r_4) (void *, struct bhnd_resource *, bus_size_t);
	uint64_t	(*bs_r_8) (void *, struct bhnd_resource *, bus_size_t);

	/* read multiple */
	void		(*bs_rm_1) (void *, struct bhnd_resource *, bus_size_t,
	    uint8_t *, bus_size_t);
	void		(*bs_rm_2) (void *, struct bhnd_resource *, bus_size_t,
	    uint16_t *, bus_size_t);
	void		(*bs_rm_4) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t *, bus_size_t);
	void		(*bs_rm_8) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t *, bus_size_t);
					
	/* read region */
	void		(*bs_rr_1) (void *, struct bhnd_resource *,
			    bus_size_t, uint8_t *, bus_size_t);
	void		(*bs_rr_2) (void *, struct bhnd_resource *,
			    bus_size_t, uint16_t *, bus_size_t);
	void		(*bs_rr_4) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t *, bus_size_t);
	void		(*bs_rr_8) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t *, bus_size_t);
					
	/* write (single) */
	void		(*bs_w_1) (void *, struct bhnd_resource *,
			    bus_size_t, uint8_t);
	void		(*bs_w_2) (void *, struct bhnd_resource *,
			    bus_size_t, uint16_t);
	void		(*bs_w_4) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t);
	void		(*bs_w_8) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t);

	/* write multiple */
	void		(*bs_wm_1) (void *, struct bhnd_resource *,
			    bus_size_t, const uint8_t *, bus_size_t);
	void		(*bs_wm_2) (void *, struct bhnd_resource *,
			    bus_size_t, const uint16_t *, bus_size_t);
	void		(*bs_wm_4) (void *, struct bhnd_resource *,
			    bus_size_t, const uint32_t *, bus_size_t);
	void		(*bs_wm_8) (void *, struct bhnd_resource *,
			    bus_size_t, const uint64_t *, bus_size_t);
					
	/* write region */
	void		(*bs_wr_1) (void *, struct bhnd_resource *,
			    bus_size_t, const uint8_t *, bus_size_t);
	void		(*bs_wr_2) (void *, struct bhnd_resource *,
			    bus_size_t, const uint16_t *, bus_size_t);
	void		(*bs_wr_4) (void *, struct bhnd_resource *,
			    bus_size_t, const uint32_t *, bus_size_t);
	void		(*bs_wr_8) (void *, struct bhnd_resource *,
			    bus_size_t, const uint64_t *, bus_size_t);

	/* set multiple */
	void		(*bs_sm_1) (void *, struct bhnd_resource *,
			    bus_size_t, uint8_t, bus_size_t);
	void		(*bs_sm_2) (void *, struct bhnd_resource *,
			    bus_size_t, uint16_t, bus_size_t);
	void		(*bs_sm_4) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t, bus_size_t);
	void		(*bs_sm_8) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t, bus_size_t);

	/* set region */
	void		(*bs_sr_1) (void *, struct bhnd_resource *,
			    bus_size_t, uint8_t, bus_size_t);
	void		(*bs_sr_2) (void *, struct bhnd_resource *,
			    bus_size_t, uint16_t, bus_size_t);
	void		(*bs_sr_4) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t, bus_size_t);
	void		(*bs_sr_8) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t, bus_size_t);

#ifdef notyet
	/* copy */
	void		(*bs_c_1) (void *, struct bhnd_resource *, bus_size_t,
			    struct bhnd_resource *, bus_size_t, bus_size_t);
	void		(*bs_c_2) (void *, struct bhnd_resource *, bus_size_t,
			    struct bhnd_resource *, bus_size_t, bus_size_t);
	void		(*bs_c_4) (void *, struct bhnd_resource *, bus_size_t,
			    struct bhnd_resource *, bus_size_t, bus_size_t);
	void		(*bs_c_8) (void *, struct bhnd_resource *, bus_size_t,
			    struct bhnd_resource *, bus_size_t, bus_size_t);
#endif

	/* read stream (single) */
	uint8_t		(*bs_r_1_s) (void *, struct bhnd_resource *,
			    bus_size_t);
	uint16_t	(*bs_r_2_s) (void *, struct bhnd_resource *,
			    bus_size_t);
	uint32_t	(*bs_r_4_s) (void *, struct bhnd_resource *,
			    bus_size_t);
	uint64_t	(*bs_r_8_s) (void *, struct bhnd_resource *,
			    bus_size_t);

	/* read multiple stream */
	void		(*bs_rm_1_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint8_t *, bus_size_t);
	void		(*bs_rm_2_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint16_t *, bus_size_t);
	void		(*bs_rm_4_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t *, bus_size_t);
	void		(*bs_rm_8_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t *, bus_size_t);
					
	/* read region stream */
	void		(*bs_rr_1_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint8_t *, bus_size_t);
	void		(*bs_rr_2_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint16_t *, bus_size_t);
	void		(*bs_rr_4_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t *, bus_size_t);
	void		(*bs_rr_8_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t *, bus_size_t);
					
	/* write stream (single) */
	void		(*bs_w_1_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint8_t);
	void		(*bs_w_2_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint16_t);
	void		(*bs_w_4_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint32_t);
	void		(*bs_w_8_s) (void *, struct bhnd_resource *,
			    bus_size_t, uint64_t);

	/* write multiple stream */
	void		(*bs_wm_1_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint8_t *, bus_size_t);
	void		(*bs_wm_2_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint16_t *, bus_size_t);
	void		(*bs_wm_4_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint32_t *, bus_size_t);
	void		(*bs_wm_8_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint64_t *, bus_size_t);
					
	/* write region stream */
	void		(*bs_wr_1_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint8_t *, bus_size_t);
	void		(*bs_wr_2_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint16_t *, bus_size_t);
	void		(*bs_wr_4_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint32_t *, bus_size_t);
	void		(*bs_wr_8_s) (void *, struct bhnd_resource *,
			    bus_size_t, const uint64_t *, bus_size_t);
};


/*
 * Utility macros; INTERNAL USE ONLY.
 */
#define	__bhnd_bs_c(a,b)		__CONCAT(a,b)
#define	__bhnd_bs_opname(op,size)					\
	__bhnd_bs_c(__bs_c(__bs_c(bs_,op),_),size)

#define	__bhnd_bs_rs(sz, t, h, o)					\
	(*(t)->__bs_opname(r,sz))((t)->bs_cookie, h, o)
#define	__bhnd_bs_ws(sz, t, h, o, v)					\
	(*(t)->__bs_opname(w,sz))((t)->bs_cookie, h, o, v)
#define	__bhnd_bs_nonsingle(type, sz, t, h, o, a, c)			\
	(*(t)->__bs_opname(type,sz))((t)->bs_cookie, h, o, a, c)
#define	__bhnd_bs_set(type, sz, t, h, o, v, c)				\
	(*(t)->__bs_opname(type,sz))((t)->bs_cookie, h, o, v, c)
#define	__bhnd_bs_copy(sz, t, h1, o1, h2, o2, cnt)			\
	(*(t)->__bs_opname(c,sz))((t)->bs_cookie, h1, o1, h2, o2, cnt)

#define	__bhnd_bs_opname_s(op,size)					\
	__bhnd_bs_c(__bs_c(__bs_c(__bs_c(bs_,op),_),size),_s)
#define	__bhnd_bs_rs_s(sz, t, h, o)					\
	(*(t)->__bs_opname_s(r,sz))((t)->bs_cookie, h, o)
#define	__bhnd_bs_ws_s(sz, t, h, o, v)					\
	(*(t)->__bs_opname_s(w,sz))((t)->bs_cookie, h, o, v)
#define	__bhnd_bs_nonsingle_s(type, sz, t, h, o, a, c)			\
	(*(t)->__bs_opname_s(type,sz))((t)->bs_cookie, h, o, a, c)

/*
 * Bus barrier operations.
 */
#define	bhnd_bus_barrier(t, h, o, l, f)				\
	(*(t)->bs_barrier)((t)->bs_cookie, (h), (o), (l), (f))

/*
 * Bus read (single) operations.
 */
#define	bhnd_bus_read_1(r, o)	__bhnd_bs_rs(1,(t),(h),(o))
#define	bhnd_bus_read_2(r, o)	__bhnd_bs_rs(2,(t),(h),(o))
#define	bhnd_bus_read_4(r, o)	__bhnd_bs_rs(4,(t),(h),(o))
#define	bhnd_bus_read_8(r, o)	__bhnd_bs_rs(8,(t),(h),(o))

#define	bhnd_bus_read_stream_1(r, o)	__bhnd_bs_rs_s(1,(t), (h), (o))
#define	bhnd_bus_read_stream_2(r, o)	__bhnd_bs_rs_s(2,(t), (h), (o))
#define	bhnd_bus_read_stream_4(r, o)	__bhnd_bs_rs_s(4,(t), (h), (o))
#define	bhnd_bus_read_stream_8(r, o)	__bhnd_bs_rs_s(8,8,(t),(h),(o))

/*
 * Bus read multiple operations.
 */
#define	bhnd_bus_read_multi_1(r, o, a, c)				\
	__bhnd_bs_nonsingle(rm,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_multi_2(r, o, a, c)				\
	__bhnd_bs_nonsingle(rm,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_multi_4(r, o, a, c)				\
	__bhnd_bs_nonsingle(rm,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_multi_8(r, o, a, c)				\
	__bhnd_bs_nonsingle(rm,8,(t),(h),(o),(a),(c))

#define	bhnd_bus_read_multi_stream_1(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rm,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_multi_stream_2(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rm,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_multi_stream_4(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rm,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_multi_stream_8(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rm,8,(t),(h),(o),(a),(c))


/*
 * Bus read region operations.
 */
#define	bhnd_bus_read_region_1(r, o, a, c)				\
	__bhnd_bs_nonsingle(rr,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_region_2(r, o, a, c)				\
	__bhnd_bs_nonsingle(rr,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_region_4(r, o, a, c)				\
	__bhnd_bs_nonsingle(rr,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_region_8(r, o, a, c)				\
	__bhnd_bs_nonsingle(rr,8,(t),(h),(o),(a),(c))

#define	bhnd_bus_read_region_stream_1(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rr,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_region_stream_2(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rr,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_region_stream_4(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rr,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_read_region_stream_8(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(rr,8,(t),(h),(o),(a),(c))


/*
 * Bus write (single) operations.
 */
#define	bhnd_bus_write_1(r, o, v)	__bhnd_bs_ws(1,(t),(h),(o),(v))
#define	bhnd_bus_write_2(r, o, v)	__bhnd_bs_ws(2,(t),(h),(o),(v))
#define	bhnd_bus_write_4(r, o, v)	__bhnd_bs_ws(4,(t),(h),(o),(v))
#define	bhnd_bus_write_8(r, o, v)	__bhnd_bs_ws(8,(t),(h),(o),(v))

#define	bhnd_bus_write_stream_1(r, o, v)				\
	__bhnd_bs_ws_s(1,(t),(h),(o),(v))
#define	bhnd_bus_write_stream_2(r, o, v)				\
	__bhnd_bs_ws_s(2,(t),(h),(o),(v))
#define	bhnd_bus_write_stream_4(r, o, v)				\
	__bhnd_bs_ws_s(4,(t),(h),(o),(v))
#define	bhnd_bus_write_stream_8(r, o, v)				\
	__bhnd_bs_ws_s(8,(t),(h),(o),(v))


/*
 * Bus write multiple operations.
 */
#define	bhnd_bus_write_multi_1(r, o, a, c)				\
	__bhnd_bs_nonsingle(wm,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_multi_2(r, o, a, c)				\
	__bhnd_bs_nonsingle(wm,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_multi_4(r, o, a, c)				\
	__bhnd_bs_nonsingle(wm,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_multi_8(r, o, a, c)				\
	__bhnd_bs_nonsingle(wm,8,(t),(h),(o),(a),(c))

#define	bhnd_bus_write_multi_stream_1(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wm,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_multi_stream_2(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wm,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_multi_stream_4(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wm,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_multi_stream_8(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wm,8,(t),(h),(o),(a),(c))


/*
 * Bus write region operations.
 */
#define	bhnd_bus_write_region_1(r, o, a, c)				\
	__bhnd_bs_nonsingle(wr,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_region_2(r, o, a, c)				\
	__bhnd_bs_nonsingle(wr,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_region_4(r, o, a, c)				\
	__bhnd_bs_nonsingle(wr,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_region_8(r, o, a, c)				\
	__bhnd_bs_nonsingle(wr,8,(t),(h),(o),(a),(c))

#define	bhnd_bus_write_region_stream_1(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wr,1,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_region_stream_2(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wr,2,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_region_stream_4(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wr,4,(t),(h),(o),(a),(c))
#define	bhnd_bus_write_region_stream_8(r, o, a, c)			\
	__bhnd_bs_nonsingle_s(wr,8,(t),(h),(o),(a),(c))


/*
 * Set multiple operations.
 */
#define	bhnd_bus_set_multi_1(r, o, v, c)				\
	__bhnd_bs_set(sm,1,(t),(h),(o),(v),(c))
#define	bhnd_bus_set_multi_2(r, o, v, c)				\
	__bhnd_bs_set(sm,2,(t),(h),(o),(v),(c))
#define	bhnd_bus_set_multi_4(r, o, v, c)				\
	__bhnd_bs_set(sm,4,(t),(h),(o),(v),(c))
#define	bhnd_bus_set_multi_8(r, o, v, c)				\
	__bhnd_bs_set(sm,8,(t),(h),(o),(v),(c))


/*
 * Set region operations.
 */
#define	bhnd_bus_set_region_1(r, o, v, c)				\
	__bhnd_bs_set(sr,1,(t),(h),(o),(v),(c))
#define	bhnd_bus_set_region_2(r, o, v, c)				\
	__bhnd_bs_set(sr,2,(t),(h),(o),(v),(c))
#define	bhnd_bus_set_region_4(r, o, v, c)				\
	__bhnd_bs_set(sr,4,(t),(h),(o),(v),(c))
#define	bhnd_bus_set_region_8(r, o, v, c)				\
	__bhnd_bs_set(sr,8,(t),(h),(o),(v),(c))


/*
 * Copy operations.
 */
#define	bhnd_bus_copy_region_1(t, h1, o1, h2, o2, c)			\
	__bhnd_bs_copy(1, t, h1, o1, h2, o2, c)
#define	bhnd_bus_copy_region_2(t, h1, o1, h2, o2, c)			\
	__bhnd_bs_copy(2, t, h1, o1, h2, o2, c)
#define	bhnd_bus_copy_region_4(t, h1, o1, h2, o2, c)			\
	__bhnd_bs_copy(4, t, h1, o1, h2, o2, c)
#define	bhnd_bus_copy_region_8(t, h1, o1, h2, o2, c)			\
	__bhnd_bs_copy(8, t, h1, o1, h2, o2, c)

	
/*
 * Macros to provide prototypes for all the functions used in the
 * bus_space structure
 */

#define	bhnd_bs_map_proto(f)						\
int	__bhnd_bs_c(f,_bs_map) (void *t, bus_addr_t addr,		\
	    bus_size_t size, int cacheable, bus_space_handle_t *bshp);

#define	bhnd_bs_unmap_proto(f)						\
void	__bhnd_bs_c(f,_bs_unmap) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t size);

#define	bhnd_bs_subregion_proto(f)					\
int	__bhnd_bs_c(f,_bs_subregion) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, bus_size_t size, 			\
	    bus_space_handle_t *nbshp);

#define	bhnd_bs_alloc_proto(f)						\
int	__bhnd_bs_c(f,_bs_alloc) (void *t, bus_addr_t rstart,		\
	    bus_addr_t rend, bus_size_t size, bus_size_t align,		\
	    bus_size_t boundary, int cacheable, bus_addr_t *addrp,	\
	    bus_space_handle_t *bshp);

#define	bhnd_bs_free_proto(f)						\
void	__bhnd_bs_c(f,_bs_free) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t size);

#define	bhnd_bs_barrier_proto(f)						\
void	__bhnd_bs_c(f,_bs_barrier) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, bus_size_t len, int flags);

#define	bhnd_bs_r_1_proto(f)							\
u_int8_t	__bhnd_bs_c(f,_bs_r_1) (void *t, bus_space_handle_t bsh,	\
		    bus_size_t offset);

#define	bhnd_bs_r_2_proto(f)							\
u_int16_t	__bhnd_bs_c(f,_bs_r_2) (void *t, bus_space_handle_t bsh,	\
		    bus_size_t offset);

#define	bhnd_bs_r_4_proto(f)							\
u_int32_t	__bhnd_bs_c(f,_bs_r_4) (void *t, bus_space_handle_t bsh,	\
		    bus_size_t offset);

#define	bhnd_bs_r_8_proto(f)							\
u_int64_t	__bhnd_bs_c(f,_bs_r_8) (void *t, bus_space_handle_t bsh,	\
		    bus_size_t offset);

#define	bhnd_bs_r_1_s_proto(f)						\
u_int8_t	__bhnd_bs_c(f,_bs_r_1_s) (void *t, bus_space_handle_t bsh,	\
		    bus_size_t offset);

#define	bhnd_bs_r_2_s_proto(f)						\
u_int16_t	__bhnd_bs_c(f,_bs_r_2_s) (void *t, bus_space_handle_t bsh,	\
		    bus_size_t offset);

#define	bhnd_bs_r_4_s_proto(f)						\
u_int32_t	__bhnd_bs_c(f,_bs_r_4_s) (void *t, bus_space_handle_t bsh,	\
		    bus_size_t offset);

#define	bhnd_bs_w_1_proto(f)						\
void	__bhnd_bs_c(f,_bs_w_1) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int8_t value);

#define	bhnd_bs_w_2_proto(f)						\
void	__bhnd_bs_c(f,_bs_w_2) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int16_t value);

#define	bhnd_bs_w_4_proto(f)						\
void	__bhnd_bs_c(f,_bs_w_4) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int32_t value);

#define	bhnd_bs_w_8_proto(f)						\
void	__bhnd_bs_c(f,_bs_w_8) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int64_t value);

#define	bhnd_bs_w_1_s_proto(f)						\
void	__bhnd_bs_c(f,_bs_w_1_s) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int8_t value);

#define	bhnd_bs_w_2_s_proto(f)						\
void	__bhnd_bs_c(f,_bs_w_2_s) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int16_t value);

#define	bhnd_bs_w_4_s_proto(f)						\
void	__bhnd_bs_c(f,_bs_w_4_s) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int32_t value);

#define	bhnd_bs_rm_1_proto(f)						\
void	__bhnd_bs_c(f,_bs_rm_1) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int8_t *addr, bus_size_t count);

#define	bhnd_bs_rm_2_proto(f)						\
void	__bhnd_bs_c(f,_bs_rm_2) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int16_t *addr, bus_size_t count);

#define	bhnd_bs_rm_4_proto(f)						\
void	__bhnd_bs_c(f,_bs_rm_4) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int32_t *addr, bus_size_t count);		

#define	bhnd_bs_rm_8_proto(f)						\
void	__bhnd_bs_c(f,_bs_rm_8) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int64_t *addr, bus_size_t count);

#define	bhnd_bs_wm_1_proto(f)						\
void	__bhnd_bs_c(f,_bs_wm_1) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int8_t *addr, bus_size_t count);

#define	bhnd_bs_wm_2_proto(f)						\
void	__bhnd_bs_c(f,_bs_wm_2) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int16_t *addr, bus_size_t count);

#define	bhnd_bs_wm_4_proto(f)						\
void	__bhnd_bs_c(f,_bs_wm_4) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int32_t *addr, bus_size_t count);

#define	bhnd_bs_wm_8_proto(f)						\
void	__bhnd_bs_c(f,_bs_wm_8) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int64_t *addr, bus_size_t count);

#define	bhnd_bs_rr_1_proto(f)						\
void	__bhnd_bs_c(f, _bs_rr_1) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int8_t *addr, bus_size_t count);

#define	bhnd_bs_rr_2_proto(f)						\
void	__bhnd_bs_c(f, _bs_rr_2) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int16_t *addr, bus_size_t count);

#define	bhnd_bs_rr_4_proto(f)						\
void	__bhnd_bs_c(f, _bs_rr_4) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int32_t *addr, bus_size_t count);

#define	bhnd_bs_rr_8_proto(f)						\
void	__bhnd_bs_c(f, _bs_rr_8) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int64_t *addr, bus_size_t count);

#define	bhnd_bs_wr_1_proto(f)						\
void	__bhnd_bs_c(f, _bs_wr_1) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int8_t *addr, bus_size_t count);

#define	bhnd_bs_wr_2_proto(f)						\
void	__bhnd_bs_c(f, _bs_wr_2) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int16_t *addr, bus_size_t count);

#define	bhnd_bs_wr_4_proto(f)						\
void	__bhnd_bs_c(f, _bs_wr_4) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int32_t *addr, bus_size_t count);

#define	bhnd_bs_wr_8_proto(f)						\
void	__bhnd_bs_c(f, _bs_wr_8) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, const u_int64_t *addr, bus_size_t count);

#define	bhnd_bs_sm_1_proto(f)						\
void	__bhnd_bs_c(f,_bs_sm_1) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int8_t value, bus_size_t count);

#define	bhnd_bs_sm_2_proto(f)						\
void	__bhnd_bs_c(f,_bs_sm_2) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int16_t value, bus_size_t count);

#define	bhnd_bs_sm_4_proto(f)						\
void	__bhnd_bs_c(f,_bs_sm_4) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int32_t value, bus_size_t count);

#define	bhnd_bs_sm_8_proto(f)						\
void	__bhnd_bs_c(f,_bs_sm_8) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int64_t value, bus_size_t count);

#define	bhnd_bs_sr_1_proto(f)						\
void	__bhnd_bs_c(f,_bs_sr_1) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int8_t value, bus_size_t count);

#define	bhnd_bs_sr_2_proto(f)						\
void	__bhnd_bs_c(f,_bs_sr_2) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int16_t value, bus_size_t count);

#define	bhnd_bs_sr_4_proto(f)						\
void	__bhnd_bs_c(f,_bs_sr_4) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int32_t value, bus_size_t count);

#define	bhnd_bs_sr_8_proto(f)						\
void	__bhnd_bs_c(f,_bs_sr_8) (void *t, bus_space_handle_t bsh,	\
	    bus_size_t offset, u_int64_t value, bus_size_t count);

#define	bhnd_bs_c_1_proto(f)							\
void	__bhnd_bs_c(f,_bs_c_1) (void *t, bus_space_handle_t bsh1,	\
	    bus_size_t offset1, bus_space_handle_t bsh2,		\
	    bus_size_t offset2, bus_size_t count);

#define	bhnd_bs_c_2_proto(f)							\
void	__bhnd_bs_c(f,_bs_c_2) (void *t, bus_space_handle_t bsh1,	\
	    bus_size_t offset1, bus_space_handle_t bsh2,		\
	    bus_size_t offset2, bus_size_t count);

#define	bhnd_bs_c_4_proto(f)							\
void	__bhnd_bs_c(f,_bs_c_4) (void *t, bus_space_handle_t bsh1,	\
	    bus_size_t offset1, bus_space_handle_t bsh2,		\
	    bus_size_t offset2, bus_size_t count);

#define	bhnd_bs_c_8_proto(f)							\
void	__bhnd_bs_c(f,_bs_c_8) (void *t, bus_space_handle_t bsh1,	\
	    bus_size_t offset1, bus_space_handle_t bsh2,		\
	    bus_size_t offset2, bus_size_t count);

#define	BHND_DECLARE_BUS_SPACE_PROTOTYPES(f)		\
	bhnd_bs_map_proto(f);			\
	bhnd_bs_unmap_proto(f);			\
	bhnd_bs_subregion_proto(f);			\
	bhnd_bs_alloc_proto(f);			\
	bhnd_bs_free_proto(f);			\
	bhnd_bs_barrier_proto(f);			\
	bhnd_bs_r_1_proto(f);			\
	bhnd_bs_r_2_proto(f);			\
	bhnd_bs_r_4_proto(f);			\
	bhnd_bs_r_8_proto(f);			\
	bhnd_bs_r_1_s_proto(f);			\
	bhnd_bs_r_2_s_proto(f);			\
	bhnd_bs_r_4_s_proto(f);			\
	bhnd_bs_w_1_proto(f);			\
	bhnd_bs_w_2_proto(f);			\
	bhnd_bs_w_4_proto(f);			\
	bhnd_bs_w_8_proto(f);			\
	bhnd_bs_w_1_s_proto(f);			\
	bhnd_bs_w_2_s_proto(f);			\
	bhnd_bs_w_4_s_proto(f);			\
	bhnd_bs_rm_1_proto(f);			\
	bhnd_bs_rm_2_proto(f);			\
	bhnd_bs_rm_4_proto(f);			\
	bhnd_bs_rm_8_proto(f);			\
	bhnd_bs_wm_1_proto(f);			\
	bhnd_bs_wm_2_proto(f);			\
	bhnd_bs_wm_4_proto(f);			\
	bhnd_bs_wm_8_proto(f);			\
	bhnd_bs_rr_1_proto(f);			\
	bhnd_bs_rr_2_proto(f);			\
	bhnd_bs_rr_4_proto(f);			\
	bhnd_bs_rr_8_proto(f);			\
	bhnd_bs_wr_1_proto(f);			\
	bhnd_bs_wr_2_proto(f);			\
	bhnd_bs_wr_4_proto(f);			\
	bhnd_bs_wr_8_proto(f);			\
	bhnd_bs_sm_1_proto(f);			\
	bhnd_bs_sm_2_proto(f);			\
	bhnd_bs_sm_4_proto(f);			\
	bhnd_bs_sm_8_proto(f);			\
	bhnd_bs_sr_1_proto(f);			\
	bhnd_bs_sr_2_proto(f);			\
	bhnd_bs_sr_4_proto(f);			\
	bhnd_bs_sr_8_proto(f);			\
	bhnd_bs_c_1_proto(f);			\
	bhnd_bs_c_2_proto(f);			\
	bhnd_bs_c_4_proto(f);			\
	bhnd_bs_c_8_proto(f);

BHND_DECLARE_BUS_SPACE_PROTOTYPES(generic);

#endif /* _BHND_BHND_BUS_SPACE_H_ */
