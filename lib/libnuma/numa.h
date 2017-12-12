/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2017 Landon Fuller <landonf@FreeBSD.org>
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	__NUMA_H__
#define	__NUMA_H__

#include <sys/cdefs.h>

#include <unistd.h>

__BEGIN_DECLS

#define	LIBNUMA_API_VERSION	2

// TODO
//#define NUMA_NUM_NODES  2048

// TODO
struct bitmask;
typedef struct nodemask nodemask_t;

extern struct bitmask	*numa_all_cpus_ptr;
extern struct bitmask	*numa_all_nodes_ptr;
extern struct bitmask	*numa_nodes_ptr;
extern struct bitmask	*numa_no_nodes_ptr;

extern int		 numa_exit_on_error;
extern int		 numa_exit_on_warn;

int		 numa_available(void);

struct bitmask	*numa_bitmask_alloc(unsigned int nbits);
void		 numa_bitmask_free(struct bitmask *bmask);

int		 numa_bitmask_isbitset(const struct bitmask *bmask,
		     unsigned int bit);
struct bitmask	*numa_bitmask_clearall(struct bitmask *bmask);
struct bitmask	*numa_bitmask_clearbit(struct bitmask *bmask, unsigned int bit);
unsigned int	 numa_bitmask_nbytes(struct bitmask *bmask);
struct bitmask	*numa_bitmask_setall(struct bitmask *bmask);
struct bitmask	*numa_bitmask_setbit(struct bitmask *bmask, unsigned int bit);
unsigned int	 numa_bitmask_weight(const struct bitmask *bmask);
int		 numa_bitmask_equal(const struct bitmask *bmask1,
		     const struct bitmask *bmask2);

void		 copy_nodemask_to_bitmask(nodemask_t *, struct bitmask *);
void		 copy_bitmask_to_nodemask(struct bitmask *, nodemask_t *);
void		 copy_bitmask_to_bitmask(struct bitmask *, struct bitmask *);

int		 numa_max_node(void);
long long	 numa_node_size64(int node, long long *freep);
long		 numa_node_size(int node, long *freep);
int		 numa_pagesize(void);

struct bitmask	*numa_get_interleave_mask(void);
void		 numa_set_interleave_mask(struct bitmask *nodemask);
int		 numa_get_interleave_node(void);

void		 numa_set_localalloc(void);

void		 numa_set_membind(struct bitmask *nodemask);
struct bitmask	*numa_get_membind(void);

struct bitmask	*numa_get_mems_allowed(void);

int		 numa_migrate_pages(int pid, struct bitmask *from,
		     struct bitmask *to);

int		 numa_preferred(void);
void		 numa_set_preferred(int node);

void		 numa_bind(struct bitmask *nodes);
void		 numa_set_bind_policy(int strict);
void		 numa_set_strict(int strict);

int		 numa_sched_getaffinity(pid_t, struct bitmask *);
int		 numa_sched_setaffinity(pid_t, struct bitmask *);

struct bitmask	*numa_allocate_nodemask(void);
void		 numa_free_nodemask(struct bitmask *nodemask);

void		*numa_alloc(size_t size);
void		*numa_alloc_interleaved(size_t size);
void		*numa_alloc_interleaved_subset(size_t size,
		     struct bitmask *nodemask);
void		*numa_alloc_local(size_t size);
void		*numa_alloc_onnode(size_t size, int node);
void		 numa_free(void *mem, size_t size);
void		*numa_realloc(void *ptr, size_t size, size_t new_size);

void		 numa_interleave_memory(void *mem, size_t size,
		     struct bitmask *mask);
void		 numa_police_memory(void *mem, size_t size);
void		 numa_setlocal_memory(void *mem, size_t size);
void		 numa_tonode_memory(void *mem, size_t size, int node);
void		 numa_tonodemask_memory(void *mem, size_t size,
		     struct bitmask *mask);

int		 numa_run_on_node(int node);
int		 numa_run_on_node_mask_all(struct bitmask *bmp);
struct bitmask	*numa_get_run_node_mask(void);

int		 numa_num_configured_cpus(void);
int		 numa_num_configured_nodes(void);
int		 numa_num_possible_nodes(void);
int		 numa_num_task_nodes(void);

struct bitmask	*numa_allocate_cpumask(void);
void		 numa_free_cpumask(struct bitmask *bmask);

int		 numa_distance(int node1, int node2);
int		 numa_node_of_cpu(int cpu);
int		 numa_node_to_cpus(int, struct bitmask *);

void		 numa_error(char *desc);
void		 numa_warn(int warn_type, char *fmt, ...);

struct bitmask	*numa_parse_nodestring(const char *nodestr);
struct bitmask	*numa_parse_nodestring_all(const char *nodestr);
struct bitmask	*numa_parse_cpustring(const char *cpustr);
struct bitmask	*numa_parse_cpustring_all(const char *cpustr);

__END_DECLS

#endif	/* __NUMA_H__ */
