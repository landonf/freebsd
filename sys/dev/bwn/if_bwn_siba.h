/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2016 Landon J. Fuller <landonf@FreeBSD.org>.
 * Copyright (c) 2007 Bruce M. Simpson.
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

#ifndef _IF_BWN_SIBA_H_
#define _IF_BWN_SIBA_H_

struct bwn_softc;

/* Always false now that siba_bwn has been removed */
#define	BWN_USE_SIBA	0

/*
 * Legacy siba(4) bus API compatibility shims.
 */
struct bwn_bus_ops {
	/* bus-specific initialization/finalization */
	int		(*init)(device_t);
	void		(*fini)(device_t);

	/* compatibility shims */
	uint16_t	(*get_pci_vendor)(device_t);
	uint16_t	(*get_pci_device)(device_t);
	enum siba_type	(*get_type)(device_t);
	uint8_t		(*get_pcicore_revid)(device_t);
};

extern const struct bwn_bus_ops bwn_bhnd_bus_ops;

enum siba_type {
	SIBA_TYPE_SSB			/* unused */,
	SIBA_TYPE_PCI,
	SIBA_TYPE_PCMCIA
};

#define	BWN_BUS_OPS_SC(_sc)	\
	((_sc)->sc_bus_ops)

#define	BWN_BUS_OPS(_dev)	\
	BWN_BUS_OPS_SC((struct bwn_softc *)device_get_softc(_dev))

#define	BWN_BUS_OPS_ATTACH(_dev)	\
	BWN_BUS_OPS(_dev)->init(_dev)
#define	BWN_BUS_OPS_DETACH(_dev)	\
	BWN_BUS_OPS(_dev)->fini(_dev)

#define	siba_get_pci_vendor(_dev)	\
	BWN_BUS_OPS(_dev)->get_pci_vendor(_dev)
#define	siba_get_pci_device(_dev)	\
	BWN_BUS_OPS(_dev)->get_pci_device(_dev)
#define	siba_get_type(_dev)	\
	BWN_BUS_OPS(_dev)->get_type(_dev)

#endif /* _IF_BWN_SIBA_H_ */
