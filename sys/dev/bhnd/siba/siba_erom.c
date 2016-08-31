/*-
 * Copyright (c) 2015-2016 Landon Fuller <landonf@FreeBSD.org>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>

#include <machine/bus.h>

#include <dev/bhnd/erom/bhnd_erom.h>

#include <dev/bhnd/cores/chipc/chipcreg.h>

#include "sibareg.h"
#include "sibavar.h"

struct siba_erom {
	struct bhnd_erom	obj;
	u_int			ncores;	/**< core count */

	/* resource state */
	device_t	 	 dev;	/**< parent dev to use for resource allocations,
					     or NULL if initialized with bst/bsh */
	struct bhnd_resource	*res;	/**< siba bus mapping, or NULL */
	int			 rid;	/**< siba bus maping resource ID */

	/* bus tag state */
	bus_space_tag_t		 bst;	/**< chipc bus tag */
	bus_space_handle_t	 bsh;	/**< chipc bus handle */
};

#define	EROM_LOG(sc, fmt, ...)	do {					\
	if (sc->dev != NULL) {						\
		device_printf(sc->dev, "%s: " fmt, __FUNCTION__,	\
		    ##__VA_ARGS__);					\
	} else {							\
		printf("%s: " fmt, __FUNCTION__, ##__VA_ARGS__);	\
	}								\
} while(0)

static inline uint32_t
siba_erom_read_4(struct siba_erom *sc, u_int core_idx, bus_size_t offset)
{
	bus_size_t core_offset;

	/* Sanity check core index and offset */
	if (core_idx >= sc->ncores)
		panic("core index %u out of range (ncores=%u)", core_idx,
		    sc->ncores);

	if (offset > SIBA_CORE_SIZE - sizeof(uint32_t))
		panic("invalid core offset %#jx", (uintmax_t)offset);

	/* Perform read */
	core_offset = SIBA_CORE_OFFSET(core_idx) + offset;
	if (sc->res != NULL)
		return (bhnd_bus_read_4(sc->res, core_offset));
	else
		return (bus_space_read_4(sc->bst, sc->bsh, core_offset));
}

static struct siba_core_id
siba_erom_read_core_info(struct siba_erom *sc, u_int core_idx, int unit)
{
	uint32_t idhigh, idlow;

	idhigh = siba_erom_read_4(sc, core_idx, SB0_REG_ABS(SIBA_CFG0_IDHIGH));
	idlow = siba_erom_read_4(sc, core_idx, SB0_REG_ABS(SIBA_CFG0_IDLOW));

	return (siba_parse_core_id(idhigh, idlow, core_idx, unit));
}

static int
siba_erom_init_common(struct siba_erom *sc)
{
	struct siba_core_id	ccid;
	struct bhnd_chipid	chipid;
	uint32_t		idreg;
	int			error;

	/* There's always at least one core */
	sc->ncores = 1;

	/* Identify the chipcommon core */
	ccid = siba_erom_read_core_info(sc, 0, 0);
	if (ccid.core_info.vendor != BHND_MFGID_BCM ||
	    ccid.core_info.device != BHND_COREID_CC)
	{
		EROM_LOG(sc,
		    "first core not chipcommon (vendor=%#hx, core=%#hx)\n",
		    ccid.core_info.vendor, ccid.core_info.device);
		return (ENXIO);
	}

	/* Identify the chipset */
	idreg = siba_erom_read_4(sc, 0, CHIPC_ID);
	chipid = bhnd_parse_chipid(idreg, SIBA_ENUM_ADDR);

	/* Fix up the core count, if required */
	error = bhnd_chipid_fixed_ncores(&chipid, ccid.core_info.hwrev,
	    &chipid.ncores);

	if (chipid.ncores > SIBA_MAX_CORES)
		return (EINVAL);

	/* Update our core count */
	sc->ncores = chipid.ncores;

	return (0);
}

static int
siba_erom_init(bhnd_erom_t erom, device_t parent, int rid, bus_addr_t enum_addr)
{
	struct siba_erom *sc = (struct siba_erom *)erom;

	sc->dev = parent;
	sc->rid = rid;

	sc->res = bhnd_alloc_resource(sc->dev, SYS_RES_MEMORY, &sc->rid,
	    enum_addr, enum_addr + SIBA_ENUM_SIZE -1, SIBA_ENUM_SIZE,
	    RF_ACTIVE|RF_SHAREABLE);
	if (sc->res == NULL)
		return (ENOMEM);

	return (siba_erom_init_common(sc));
}

static int
siba_erom_init_static(bhnd_erom_t erom, bus_space_tag_t bst,
     bus_space_handle_t bsh)
{
	struct siba_erom *sc = (struct siba_erom *)erom;

	sc->dev = NULL;
	sc->rid = -1;
	sc->res = NULL;
	sc->bst = bst;
	sc->bsh = bsh;

	return (siba_erom_init_common(sc));
}

static void
siba_erom_fini(bhnd_erom_t erom)
{
	struct siba_erom *sc = (struct siba_erom *)erom;

	if (sc->res != NULL) {
		bhnd_release_resource(sc->dev, SYS_RES_MEMORY, sc->rid,
		    sc->res);

		sc->res = NULL;
		sc->rid = -1;
	}
}

static int
siba_erom_lookup_core(bhnd_erom_t erom, const struct bhnd_core_match *desc,
    struct bhnd_core_info *core)
{
	// TODO
	return (ENXIO);
}

static int
siba_erom_lookup_core_addr(bhnd_erom_t erom, const struct bhnd_core_match *desc,
    bhnd_port_type port_type, u_int port_num, u_int region_num,
    bhnd_addr_t *addr, bhnd_size_t *size)
{
	// TODO
	return (ENXIO);
};

/* BHND_EROM_GET_CORE_TABLE() */
static int
siba_erom_get_core_table(bhnd_erom_t erom, struct bhnd_core_info **cores,
    u_int *num_cores)
{
	// TODO
	return (ENXIO);
}

/* BHND_EROM_FREE_CORE_TABLE() */
static void
siba_erom_free_core_table(bhnd_erom_t erom, struct bhnd_core_info *cores)
{
	free(cores, M_BHND);
}

static kobj_method_t siba_erom_methods[] = {
	KOBJMETHOD(bhnd_erom_init,		siba_erom_init),
	KOBJMETHOD(bhnd_erom_init_static,	siba_erom_init_static),
	KOBJMETHOD(bhnd_erom_fini,		siba_erom_fini),
	KOBJMETHOD(bhnd_erom_get_core_table,	siba_erom_get_core_table),
	KOBJMETHOD(bhnd_erom_free_core_table,	siba_erom_free_core_table),
	KOBJMETHOD(bhnd_erom_lookup_core,	siba_erom_lookup_core),
	KOBJMETHOD(bhnd_erom_lookup_core_addr,	siba_erom_lookup_core_addr),

	KOBJMETHOD_END
};

DEFINE_CLASS_0(siba_erom, siba_erom_parser, siba_erom_methods, sizeof(struct siba_erom));
