/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
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

#ifndef _BHND_NVRAM_BHND_NVRAMVAR_H_
#define _BHND_NVRAM_BHND_NVRAMVAR_H_

#ifdef _KERNEL

#define	bhnd_nv_malloc(size)		malloc((size), M_BHND_NVRAM, M_WAITOK)
#define	bhnd_nv_calloc(n, size)		malloc((n) * (size), M_BHND_NVRAM, \
					    M_WAITOK | M_ZERO)
#define	bhnd_nv_reallocf(buf, size)	reallocf((buf), (size), M_BHND_NVRAM, \
					    M_WAITOK)
#define	bhnd_nv_free(buf)		free((buf), M_BHND_NVRAM)
#define	bhnd_nv_strndup(str, len)	strndup(str, len, M_BHND_NVRAM)

#ifdef INVARIANTS
#define	BHND_NV_INVARIANTS
#endif
#define	BHND_NV_ASSERT(expr, ...)	KASSERT(expr, __VA_ARGS__)

#define	BHND_NV_VERBOSE			(bootverbose)
#define	BHND_NV_PANIC(...)		panic(__VA_ARGS__)
#define	BHND_NV_DEVLOG(dev, fmt, ...)	do {		\
	if (dev != NULL)				\
		device_printf(dev, fmt, ##__VA_ARGS__);	\
	else						\
		BHND_NV_LOG(fmt, ##__VA_ARGS__);	\
} while(0)
#define	BHND_NV_LOG(fmt, ...)		\
	printf("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)

#define	bhnd_nv_ummax(a, b)		ummax((a), (b))
#define	bhnd_nv_ummin(a, b)		ummin((a), (b))

#else /* !_KERNEL */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define	bhnd_nv_malloc(size)		malloc((size))
#define	bhnd_nv_calloc(n, size)		calloc((n), (size))
#define	bhnd_nv_reallocf(buf, size)	reallocf((buf), (size))
#define	bhnd_nv_free(buf)		free((buf))
#define	bhnd_nv_strndup(str, len)	strndup(str, len)

#ifndef NDEBUG
#define	BHND_NV_INVARIANTS
#endif
#define	BHND_NV_ASSERT(expr, ...)	assert(expr)

#define	BHND_NV_VERBOSE			(0)
#define	BHND_NV_PANIC(fmt, ...)		do {			\
	fprintf(stderr, "panic: " fmt "\n", ##__VA_ARGS__);	\
	abort();						\
} while(0)
#define	BHND_NV_DEVLOG(dev, fmt, ...)	BHND_NV_LOG(fmt, ## __VA_ARGS__)
#define	BHND_NV_LOG(fmt, ...)					\
	fprintf(stderr, "%s: " fmt, __FUNCTION__, ##__VA_ARGS__)

static inline uintmax_t
bhnd_nv_ummax(uintmax_t a, uintmax_t b)
{
        return (a > b ? a : b);
}

static inline uintmax_t
bhnd_nv_ummin(uintmax_t a, uintmax_t b)
{

        return (a < b ? a : b);
}


#endif /* _KERNEL */

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
