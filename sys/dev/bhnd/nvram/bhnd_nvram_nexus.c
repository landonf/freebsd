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
#include <dev/cfe/cfe_ioctl.h>

#include "bhnd_nvram_if.h"
#include "bhnd_nvram_map.h"

#include "bhnd_nvram_private.h"

#include "bhnd_nvramreg.h"

#include "bhnd_nvram_nexusvar.h"

static int	 nvram_open_cfedev(device_t dev, char *devname, int fd,
		     int64_t *offset, uint32_t *size, bhnd_nvram_format fmt);
static char	*nvram_find_cfedev(device_t dev, int *fd, int64_t *offset,
		     uint32_t *size, bhnd_nvram_format *fmt);

/** Known CFE NVRAM device names, in probe order. */
static char *nvram_cfe_devs[] = {
	"nflash0.nvram",	/* NAND */
	"flash0.nvram",
};

/** Supported NVRAM formats, in probe order. */
bhnd_nvram_format nvram_cfe_fmts[] = {
	BHND_NVRAM_FMT_BCM,
	BHND_NVRAM_FMT_TLV
};


static int
bhnd_nvram_nexus_probe(device_t dev)
{
	char				*devname;
	bhnd_nvram_format		 fmt;
	int64_t				 offset;
	uint32_t			 size;
	int				 fd;

	/* Locate a usable CFE device */
	devname = nvram_find_cfedev(dev, &fd, &offset, &size, &fmt);
	if (devname == NULL)
		return (ENXIO);
	cfe_close(fd);

	switch (fmt) {
	case BHND_NVRAM_FMT_BCM:
		device_set_desc(dev, "Broadcom NVRAM");
		break;
	case BHND_NVRAM_FMT_TLV:
		device_set_desc(dev, "Broadcom WGT634U NVRAM");
		break;
	default:
		device_printf(dev, "unknown NVRAM format: %d\n", fmt);
		return (ENXIO);
	}

	/* Refuse wildcard attachments */
	return (BUS_PROBE_NOWILDCARD);
}


static int
bhnd_nvram_nexus_attach(device_t dev)
{
	struct bhnd_nvram_nexus_softc	*sc;
	char				*devname;
	unsigned char			*buffer;
	struct bhnd_nvram_input		 input;
	bhnd_nvram_format		 fmt;
	int64_t				 offset;
	uint32_t			 size;
	int				 error;
	int				 fd;

	sc = device_get_softc(dev);
	sc->dev = dev;

	error = 0;
	buffer = NULL;
	fd = CFE_ERR;

	/* Locate NVRAM device via CFE */
	devname = nvram_find_cfedev(dev, &fd, &offset, &size, &fmt);
	if (devname == NULL) {
		device_printf(dev, "CFE NVRAM device not found\n");
		return (ENXIO);
	}

	/* Copy out NVRAM buffer */
	buffer = malloc(size, M_TEMP, M_WAITOK);
	for (size_t remain = size; remain > 0;) {
		int nr, req;
		
		req = ulmin(INT_MAX, remain);
		nr = cfe_readblk(fd, size-remain, buffer+(size-remain),
		    req);
		if (nr < 0) {
			device_printf(dev, "%s: cfe_readblk() failed: %d\n",
			    devname, fd);

			error = ENXIO;
			goto done;
		}

		remain -= nr;

		if (nr == 0 && remain > 0) {
			device_printf(dev, "%s: cfe_readblk() unexpected EOF: "
			    "%zu of %zu pending\n", devname, remain, size);

			error = ENXIO;
			goto done;
		}
	}

	device_printf(dev, "CFE NVRAM device: %s (%#jx+%#jx)\n",
	    devname, (uintmax_t)offset, (uintmax_t)size);

	/* Initialize NVRAM parser */
	input = (struct bhnd_nvram_input) {
		.buffer = buffer,
		.size = size
	};

	if ((error = bhnd_nvram_init(&sc->nvram, &input, fmt))) {
		device_printf(dev, "%s: parse failed: %d\n", devname, error);
		goto done;
	}

	/* Initialize mutex */
	BHND_NVRAM_LOCK_INIT(sc);

	error = 0;

done:
	if (buffer != NULL)
		free(buffer, M_TEMP);

	if (fd >= 0)
		cfe_close(fd);

	return (error);
}

static int
bhnd_nvram_nexus_resume(device_t dev)
{
	return (0);
}

static int
bhnd_nvram_nexus_suspend(device_t dev)
{
	return (0);
}

static int
bhnd_nvram_nexus_detach(device_t dev)
{
	struct bhnd_nvram_nexus_softc	*sc;

	sc = device_get_softc(dev);

	bhnd_nvram_fini(&sc->nvram);
	BHND_NVRAM_LOCK_DESTROY(sc);

	return (0);
}

static int
bhnd_nvram_nexus_getvar(device_t dev, const char *name, void *buf, size_t *len)
{
	struct bhnd_nvram_nexus_softc	*sc;
	int				 error;

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
bhnd_nvram_nexus_setvar(device_t dev, const char *name, const void *buf,
    size_t len)
{
	struct bhnd_nvram_nexus_softc	*sc;
	int				 error;

	sc = device_get_softc(dev);

	BHND_NVRAM_LOCK(sc);
	// TODO
	error = ENODEV;
	BHND_NVRAM_UNLOCK(sc);

	return (error);
}


/**
 * Identify and open a CFE NVRAM device.
 * 
 * @param	dev	bhnd_nvram_nexus device.
 * @param	devname	The name of the CFE device to be probed.
 * @param	fd	An open CFE file descriptor for @p devname.
 * @param[out]	offset	On success, the NVRAM data offset within @p @fd.
 * @param[out]	size	On success, maximum the NVRAM data size within @p fd.
 * @param	fmt	The expected NVRAM data format for this device.
 * 
 * @retval	0		success
 * @retval	non-zero	If probing @p devname fails, a regular unix
 * 				error code will be returned.
 */
static int
nvram_open_cfedev(device_t dev, char *devname, int fd, int64_t *offset,
    uint32_t *size, bhnd_nvram_format fmt)
{
	union bhnd_nvram_ident	ident;
	nvram_info_t		ninfo;
	int			cerr, devinfo, dtype, rlen;
	int			error;

	/* Try to fetch device info */
	if ((devinfo = cfe_getdevinfo(devname)) == CFE_ERR_DEVNOTFOUND)
		return (ENODEV);

	if (devinfo < 0) {
		device_printf(dev, "cfe_getdevinfo() failed: %d",
		    devinfo);
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
		device_printf(dev, "%s: unknown device type %d\n",
		    devname, dtype);
		return (ENXIO);
	}

	/* Fetch NVRAM info from CFE */
	cerr = cfe_ioctl(fd, IOCTL_NVRAM_GETINFO, (unsigned char *)&ninfo,
	    sizeof(ninfo), &rlen, 0);
	if (cerr != CFE_OK) {
		device_printf(dev, "%s: IOCTL_NVRAM_GETINFO failed: %d\n",
		    devname, cerr);
		return (ENXIO);
	}

	if (rlen != sizeof(ninfo)) {
		device_printf(dev,
		    "%s: IOCTL_NVRAM_GETINFO short read: %d\n", devname, cerr);
		return (ENXIO);
	}

	/* Try to read NVRAM identification */
	cerr = cfe_readblk(fd, 0, (unsigned char *)&ident, sizeof(ident));
	if (cerr < 0) {
		device_printf(dev, "%s: cfe_readblk() failed: %d\n",
		    devname, cerr);
		return (ENXIO);
	} else if (cerr == 0) {
		/* EOF */
		return (ENODEV);
	} else if (cerr != sizeof(ident)) {
		device_printf(dev, "%s: cfe_readblk() short read: %d\n",
			devname, cerr);
		return (ENXIO);
	}

	/* Verify expected format */
	if ((error = bhnd_nvram_identify(&ident, fmt)))
		return (error);

	/* Provide offset and size */
	switch (fmt) {
	case BHND_NVRAM_FMT_TLV:
		/* No size field is available; must assume the NVRAM data
		 * consumes up to the full CFE NVRAM range */
		*offset = ninfo.nvram_offset;
		*size = ninfo.nvram_size;
		break;
	case BHND_NVRAM_FMT_BCM:
		*offset = ninfo.nvram_offset;
		*size = ident.bcm.size;
		break;
	default:
		return (EINVAL);
	}

	return (0);
}

/**
 * Find (and open) a CFE NVRAM device.
 * 
 * @param	dev	bhnd_nvram_nexus device.
 * @param[out]	fd	On success, a valid CFE file descriptor. The callee
 *			is responsible for closing this file descriptor via
 *			cfe_close().
 * @param[out]	offset	On success, the NVRAM data offset within @p @fd.
 * @param[out]	size	On success, maximum the NVRAM data size within @p fd.
 * @param	fmt	The expected NVRAM data format for this device.
 * 
 * @return	On success, the opened CFE device's name will be returned. On
 *		error, returns NULL.
 */
static char *
nvram_find_cfedev(device_t dev, int *fd, int64_t *offset,
    uint32_t *size, bhnd_nvram_format *fmt)
{
	char	*devname;
	int	 error;

	for (u_int i = 0; i < nitems(nvram_cfe_fmts); i++) {
		*fmt = nvram_cfe_fmts[i];

		for (u_int j = 0; j < nitems(nvram_cfe_devs); j++) {
			devname = nvram_cfe_devs[j];

			/* Open for reading */
			*fd = cfe_open(devname);
			if (*fd == CFE_ERR_DEVNOTFOUND) {
				continue;
			} else if (*fd < 0) {
				device_printf(dev, "%s: cfe_open() failed: "
				    "%d\n", devname, *fd);
				continue;
			}

			/* Probe */
			error = nvram_open_cfedev(dev, devname, *fd, offset,
			    size, *fmt);
			if (error == 0)
				return (devname);

			/* Keep searching */
			devname = NULL;
			cfe_close(*fd);
		}
	}

	return (NULL);
}

static device_method_t bhnd_nvram_nexus_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_nvram_nexus_probe),
	DEVMETHOD(device_attach,		bhnd_nvram_nexus_attach),
	DEVMETHOD(device_resume,		bhnd_nvram_nexus_resume),
	DEVMETHOD(device_suspend,		bhnd_nvram_nexus_suspend),
	DEVMETHOD(device_detach,		bhnd_nvram_nexus_detach),

	/* NVRAM interface */
	DEVMETHOD(bhnd_nvram_getvar,		bhnd_nvram_nexus_getvar),
	DEVMETHOD(bhnd_nvram_setvar,		bhnd_nvram_nexus_setvar),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram, bhnd_nvram_nexus, bhnd_nvram_nexus_methods, 
    sizeof(struct bhnd_nvram_nexus_softc));
EARLY_DRIVER_MODULE(bhnd_nvram_nexus, nexus, bhnd_nvram_nexus,
    bhnd_nvram_devclass, NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_EARLY);

