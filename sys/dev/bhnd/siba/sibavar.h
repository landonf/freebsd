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

#ifndef _SIBA_SIBAVAR_H_
#define _SIBA_SIBAVAR_H_

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/limits.h>

#include <machine/bus.h>
#include <sys/rman.h>

#include "siba.h"

/*
 * Internal definitions shared by siba(4) driver implementations.
 */

struct siba_devinfo;

int			 siba_probe(device_t dev);
int			 siba_attach(device_t dev);
int			 siba_detach(device_t dev);

uint16_t		 siba_get_bhnd_mfgid(uint16_t ocp_vendor);
uint8_t			 siba_get_ncores(const struct bhnd_chipid *chipid);

int			 siba_add_children(device_t bus,
			     const struct bhnd_chipid *chipid);

struct siba_devinfo	*siba_alloc_dinfo(device_t dev,
			     const struct bhnd_core_info *core_info);
void			 siba_free_dinfo(struct siba_devinfo *dinfo);

/**
 * siba(4) per-device info
 */
struct siba_devinfo {
	struct resource_list	resources;	/**< per-core memory regions. */
	struct bhnd_core_info	core_info;	/**< IP core/block config */
};


/** siba(4) per-instance state */
struct siba_softc {
	struct bhnd_softc	bhnd_sc;	/**< bhnd state */
};

#endif /* _SIBA_SIBAVAR_H_ */