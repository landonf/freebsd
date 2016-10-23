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

#ifndef _BHND_NVRAM_BHND_NVRAM_PRIVATE_DATA_H_
#define _BHND_NVRAM_BHND_NVRAM_PRIVATE_DATA_H_

/*
 * Private BHND NVRAM variable and SPROM format definitions.
 */

#ifdef _KERNEL
#include <sys/param.h>
#else
#include <stdint.h>
#endif

#include "bhnd_nvram_private.h"

extern const struct bhnd_nvram_vardefn bhnd_nvram_vardefns[];
extern const size_t bhnd_nvram_num_vardefns;

extern const struct bhnd_sprom_layout bhnd_sprom_layouts[];
extern const size_t bhnd_sprom_num_layouts;

#endif /* _BHND_NVRAM_BHND_NVRAM_PRIVATE_DATA_H_ */
