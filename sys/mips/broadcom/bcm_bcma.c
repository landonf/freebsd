/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 *
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>

#include <machine/bus.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_erom.h>

#include <dev/bhnd/bcma/bcma_eromvar.h>

#include "bcm_machdep.h"

#define	BCMFC_ERR(fmt, ...)	printf("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)

static struct kobj_ops		bcma_erom_kops;
static bool			bcma_erom_kops_compiled = false;

int
bcm_find_core_bcma(struct bhnd_chipid *chipid, bhnd_devclass_t devclass,
    int unit, struct bhnd_core_info *info, uintptr_t *addr)
{
	bhnd_erom_t		*erom;
	struct bcma_erom	 bcma_erom_sc;
	struct bhnd_core_match	 m;
	bhnd_addr_t		 b_addr;
	bhnd_size_t		 b_size;
	int			 error;

	if (!bcma_erom_kops_compiled) {
		kobj_class_compile_static(&bcma_erom_parser, &bcma_erom_kops);
		bcma_erom_kops_compiled = true;
	}

	/* Initialize EROM parser */
	erom = &bcma_erom_sc.obj;
	error = bhnd_erom_init_static(&bcma_erom_parser, erom,
	    sizeof(bcma_erom_sc), mips_bus_space_generic,
	    (bus_space_handle_t)BCM_SOC_ADDR(chipid->enum_addr, 0));
	if (error) {
		BCMFC_ERR("erom init failed: %d\n", error);
		return (error);
	}

	/* Fetch core info and port address mapping */
	m = (struct bhnd_core_match) {
		BHND_MATCH_CORE_CLASS(devclass),
		BHND_MATCH_CORE_UNIT(unit)
	};

	error = bhnd_erom_lookup_core_addr(erom, &m, BHND_PORT_DEVICE, 0, 0,
	    info, &b_addr, &b_size);
	if (error)
		goto cleanup;

	if (b_addr > UINTPTR_MAX) {
		BCMFC_ERR("core address %#jx overflows native address width\n",
		    (uintmax_t)b_addr);
		error = ERANGE;
		goto cleanup;
	}

	if (addr != NULL)
		*addr = b_addr;

cleanup:
	bhnd_erom_fini_static(erom);
	return (error);
}
