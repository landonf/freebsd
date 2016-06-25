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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * BHND CFE NVRAM driver.
 * 
 * Provides access to device NVRAM via CFE.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>

#include "bhnd_nvram_if.h"
#include "bhnd_nvram_map.h"

#include "nvramvar.h"

#include <sys/kenv.h>

struct bhnd_nvcfe_softc {
	device_t		 dev;
	struct mtx		 mtx;		/**< nvram mutex */
};

#define	NVCFE_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev), \
	    "bhnd_cfe nvram lock", MTX_DEF)
#define	NVCFE_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	NVCFE_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	NVCFE_LOCK_ASSERT(sc, what)	mtx_assert(&(sc)->mtx, what)
#define	NVCFE_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->mtx)

#define	NVCFE_VARSIZE_MAX	64

static int
bhnd_nvram_cfe_probe(device_t dev)
{
	device_set_desc(dev, "CFE NVRAM");

	/* Refuse wildcard attachments */
	return (BUS_PROBE_NOWILDCARD);
}

static int
bhnd_nvram_cfe_attach(device_t dev)
{
	struct bhnd_nvcfe_softc	*sc;
	uint8_t			 sromrev;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Initialize mutex */
	NVCFE_LOCK_INIT(sc);
	
	int idx;
	char name[KENV_MNAMELEN], val[KENV_MVALLEN];

        idx = 0;
        while (1) {
                if (cfe_enumenv(idx, name, sizeof(name), val, sizeof(val)) != 0)
                        break;
		
		device_printf(dev, "%s=%s\n", name, val);

                ++idx;
        }


	error = bhnd_nvram_getvar(dev, BHND_NVAR_SROMREV, &sromrev,
	    sizeof(sromrev));
	if (error) {
		device_printf(dev, "error fetching sromrev: %d\n", error);
		return (error);
	}

	device_printf(dev, "sromrev %u\n", sromrev);

	return (0);
}

static int
bhnd_nvram_cfe_resume(device_t dev)
{
	return (0);
}

static int
bhnd_nvram_cfe_suspend(device_t dev)
{
	return (0);
}

static int
bhnd_nvram_cfe_detach(device_t dev)
{
	struct bhnd_nvcfe_softc	*sc;

	sc = device_get_softc(dev);

	NVCFE_LOCK_DESTROY(sc);

	return (0);
}

static int
bhnd_nvram_cfe_getvar(device_t dev, const char *name, void *buf, size_t *len)
{
	struct bhnd_nvcfe_softc		*sc;
	const struct bhnd_nvram_var	*var;
	char				 sbuf[NVCFE_VARSIZE_MAX];
	size_t				 nitems;
	size_t				 output_len;
	int				 cfe_err;
	int				 error;

	sc = device_get_softc(dev);

	/* Lookup variable definition */
	if ((var = bhnd_nvram_var_defn(name)) == NULL)
		return (ENOENT);

	/* Skip mfg-internal variables */
	if (var->flags & BHND_NVRAM_VF_MFGINT)
		return (ENOENT);

	/* TODO: Support all data types and vfmts. */
	switch (var->type) {
	case BHND_NVRAM_DT_UINT8:
		break;
	default:
		device_printf(dev, "%s: unsupported type: %u\n", name,
		    var->type);
		return (ENODEV);
	}

	switch (var->fmt) {
	case BHND_NVRAM_VFMT_DEC:
		break;
	default:
		device_printf(dev, "%s: unsupported format: %u\n", name,
		    var->fmt);
		return (ENODEV);
	}

	/* Perform fetch */
	NVCFE_LOCK(sc);

	char *n = strdup(name, M_DEVBUF);
	cfe_err = cfe_getenv(n, sbuf, sizeof(sbuf));
	free(n, M_DEVBUF);

	switch (cfe_err) {
	case CFE_ERR_ENVNOTFOUND:
		error = ENOENT;
		break;
	case CFE_OK:
		error = 0;
		break;
	default:
		device_printf(dev, "cfe_getenv() failed with %d\n", cfe_err);
		error = ENXIO;
		break;
	}

	NVCFE_UNLOCK(sc);

	/* If fetch failed, nothing to do */
	if (error)
		return (error);

	/* Verify NULL termination within NVCFE_VARSIZE_MAX */
	if (strnlen(sbuf, sizeof(sbuf)) == sizeof(sbuf)) {
		device_printf(dev, "%s exceeds max buflen %#zx\n", name,
		     sizeof(sbuf));
		return (ENXIO);
	}
	
	/* TODO: Parse array elements sequentially */
	if (var->flags & BHND_NVRAM_VF_ARRAY) {
		device_printf(dev, "%s: array unsupported\n", name);
		return (ENODEV);
	} else {
		nitems = 1;
	}

	/* Provide required size */
	output_len = bhnd_nvram_type_width(var->type) * nitems;
	if (buf == NULL) {
		*len = output_len * nitems;
		return (0);
	}

	/* Check (and update) target buffer len */
	if (*len < output_len)
		return (ENOMEM);
	else
		*len = output_len;

	/* Parse the value */
	KASSERT(var->type == BHND_NVRAM_DT_UINT8 &&
		var->fmt == BHND_NVRAM_VFMT_DEC, ("unsupported type/fmt"));
	if (sscanf(sbuf, "%u", buf) != 1) {
		device_printf(dev, "%s: could not parse '%s'\n", name, sbuf);
		return (EINVAL);
	}

	return (0);
}

/**
 * Default bhnd sprom driver implementation of BHND_NVRAM_SETVAR().
 */
static int
bhnd_nvram_cfe_setvar(device_t dev, const char *name, const void *buf,
    size_t len)
{
	struct bhnd_nvcfe_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	NVCFE_LOCK(sc);
	// TODO
	error = ENODEV;
	NVCFE_UNLOCK(sc);

	return (error);
}

static device_method_t bhnd_nvcfe_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_nvram_cfe_probe),
	DEVMETHOD(device_attach,		bhnd_nvram_cfe_attach),
	DEVMETHOD(device_resume,		bhnd_nvram_cfe_resume),
	DEVMETHOD(device_suspend,		bhnd_nvram_cfe_suspend),
	DEVMETHOD(device_detach,		bhnd_nvram_cfe_detach),

	/* NVRAM interface */
	DEVMETHOD(bhnd_nvram_getvar,		bhnd_nvram_cfe_getvar),
	DEVMETHOD(bhnd_nvram_setvar,		bhnd_nvram_cfe_setvar),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram, bhnd_nvcfe_driver, bhnd_nvcfe_methods, 
    sizeof(struct bhnd_nvcfe_softc));
EARLY_DRIVER_MODULE(bhnd_nvram_cfe, nexus, bhnd_nvcfe_driver,
    bhnd_nvram_devclass, NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_EARLY);

