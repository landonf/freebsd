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

/**
 * Known CFE NVRAM device names, in probe order.
 */
static char *bhnd_cfe_nvram_devs[] = {
	"nflash0.nvram",	/* NAND */
	"flash0.nvram",
	"flash1.nvram",
};

struct bhnd_nvram_header {
	uint32_t magic;
	uint32_t size;
	uint32_t cfg0;		/**< crc:8, version:8, sdram_init:16 */
	uint32_t cfg1;		/**< sdram_config:16, sdram_refresh:16 */
	uint32_t memc_ncdl;	/**< ncdl memc config */
};

struct bhnd_nvram_softc {
	device_t		 	dev;
	struct mtx		 	mtx;	/**< nvram mutex */
	struct bhnd_nvram_header	header;	/**< nvram header */
};

#define	BHND_NVRAM_CFE_DEVNAME_MAX	64
#define	BHND_NVRAM_MAGIC		0x48534C46	/**< 'FLSH' */

#define	BHND_NVRAM_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev), \
	    "bhnd_nvram lock", MTX_DEF)
#define	BHND_NVRAM_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	BHND_NVRAM_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	BHND_NVRAM_LOCK_ASSERT(sc, what)	mtx_assert(&(sc)->mtx, what)
#define	BHND_NVRAM_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->mtx)

#define	BHND_NVRAM_VARSIZE_MAX	64

static int
bhnd_nvram_cfe_probe(device_t dev)
{
	device_set_desc(dev, "CFE NVRAM");

	/* Refuse wildcard attachments */
	return (BUS_PROBE_NOWILDCARD);
}

static int
bhnd_nvram_cfe_identify(struct bhnd_nvram_softc *sc, char *devname)
{
	int64_t		 offset;
	int		 cerr, devinfo, dtype;
	int		 fd;

	/* Try to fetch device info */
	devinfo = cfe_getdevinfo(devname);

	if (devinfo == CFE_ERR_DEVNOTFOUND)
		return (ENODEV);

	if (devinfo < 0) {
		if (devinfo != CFE_ERR_DEVNOTFOUND) {
			device_printf(sc->dev, "cfe_getdevinfo() failed: %d",
			    devinfo);
		}

		return (ENXIO);
	}

	/* Verify device type */
	dtype = devinfo & CFE_DEV_MASK;
	switch (dtype) {
	case CFE_DEV_FLASH:
	case CFE_DEV_NVRAM:
		/* Valid device type */
		break;
	default:
		device_printf(sc->dev, "%s: unknown device type %d\n",
		    devname, dtype);
		return (ENXIO);
	}

	/* Open for reading */
	if ((fd = cfe_open(devname)) < 0) {
		device_printf(sc->dev, "%s: cfe_open() failed: %d\n",
		    devname, fd);
		return (ENXIO);
	}

	/* Find header */
	for (offset = 0;; offset += 1024) {
		cerr = cfe_readblk(fd, offset, (unsigned char *)&sc->header,
		    sizeof(sc->header));

		if (cerr < sizeof(sc->header))
			break;

		device_printf(sc->dev, "read at %ji: %#x\n", offset,
		    sc->header.magic);

		if (sc->header.magic == BHND_NVRAM_MAGIC)
			break;
	}

	cfe_close(fd);

	if (cerr < 0) {
		device_printf(sc->dev, "%s: cfe_readblk() failed: %d\n",
		    devname, cerr);
		return (ENXIO);
	}

	/* EOF */
	if (cerr == 0)
		return (ENODEV);

	if (cerr != sizeof(sc->header)) {
		device_printf(sc->dev, "%s: cfe_readblk() short read: %d\n",
			devname, cerr);
		return (ENODEV);
	}

	// TODO
	device_printf(sc->dev, "CFE NVRAM device: %s\n", devname);
	return (0);
}

static int
bhnd_nvram_cfe_attach(device_t dev)
{
	struct bhnd_nvram_softc	*sc;
	char			*devname;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Locate NVRAM device via CFE */
	for (u_int i = 0; i < nitems(bhnd_cfe_nvram_devs); i++) {
		devname = bhnd_cfe_nvram_devs[i];
		if ((error = bhnd_nvram_cfe_identify(sc, devname)))
			devname = NULL;
	}

	// TODO
	if (devname == NULL) {
		device_printf(dev, "cfe nvram dev not found\n");
		return (ENXIO);
	}

	/* Initialize mutex */
	BHND_NVRAM_LOCK_INIT(sc);

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
	struct bhnd_nvram_softc	*sc;

	sc = device_get_softc(dev);

	BHND_NVRAM_LOCK_DESTROY(sc);

	return (0);
}

static int
bhnd_nvram_cfe_getvar(device_t dev, const char *name, void *buf, size_t *len)
{
	struct bhnd_nvram_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	BHND_NVRAM_LOCK(sc);
	// TODO
	error = ENODEV;
	BHND_NVRAM_UNLOCK(sc);

	return (error);
}

/**
 * Default bhnd sprom driver implementation of BHND_NVRAM_SETVAR().
 */
static int
bhnd_nvram_cfe_setvar(device_t dev, const char *name, const void *buf,
    size_t len)
{
	struct bhnd_nvram_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	BHND_NVRAM_LOCK(sc);
	// TODO
	error = ENODEV;
	BHND_NVRAM_UNLOCK(sc);

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
    sizeof(struct bhnd_nvram_softc));
EARLY_DRIVER_MODULE(bhnd_nvram_cfe, nexus, bhnd_nvcfe_driver,
    bhnd_nvram_devclass, NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_EARLY);

