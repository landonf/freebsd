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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <dev/bhnd/bhndb/bhndbvar.h>

#include "sibavar.h"

/*
 * Supports attachment of siba(4) bus devices via a sibab bridge.
 */

static int
siba_sibab_probe(device_t dev)
{
	int error;
	
	if ((error = siba_probe(dev)) > 0)
		return (error);

	return (BUS_PROBE_NOWILDCARD);
}

static int
siba_sibab_attach(device_t dev)
{
	int error;

	/* Enumerate our children. */
	// TODO
	if ((error = siba_add_children(dev)))
		return (error);

	/* Call our superclass' implementation */
	return (siba_attach(dev));
}

static device_method_t siba_sibab_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			siba_sibab_probe),
	DEVMETHOD(device_attach,		siba_sibab_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(siba, siba_sibab_driver, siba_sibab_methods,
    sizeof(struct siba_softc), siba_driver);

DRIVER_MODULE(siba_sibab, sibab, siba_sibab_driver, siba_devclass, NULL, NULL);
 
MODULE_VERSION(siba_sibab, 1);
MODULE_DEPEND(siba_sibab, sibab, 1, 1, 1);
