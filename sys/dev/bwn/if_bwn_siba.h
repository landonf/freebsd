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
	uint8_t		(*sprom_get_ccode)(device_t);
	uint8_t		(*sprom_get_ant_a)(device_t);
	uint8_t		(*sprom_get_ant_bg)(device_t);
	uint16_t	(*sprom_get_pa0b0)(device_t);
	uint16_t	(*sprom_get_pa0b1)(device_t);
	uint16_t	(*sprom_get_pa0b2)(device_t);
	uint8_t		(*sprom_get_gpio0)(device_t);
	uint8_t		(*sprom_get_gpio1)(device_t);
	uint8_t		(*sprom_get_gpio2)(device_t);
	uint8_t		(*sprom_get_gpio3)(device_t);
	uint16_t	(*sprom_get_maxpwr_bg)(device_t);
	void		(*sprom_set_maxpwr_bg)(device_t, uint16_t);
	uint8_t		(*sprom_get_rxpo2g)(device_t);
	uint8_t		(*sprom_get_rxpo5g)(device_t);
	uint8_t		(*sprom_get_tssi_bg)(device_t);
	uint8_t		(*sprom_get_tri2g)(device_t);
	uint8_t		(*sprom_get_tri5gl)(device_t);
	uint8_t		(*sprom_get_tri5g)(device_t);
	uint8_t		(*sprom_get_tri5gh)(device_t);
	uint8_t		(*sprom_get_rssisav2g)(device_t);
	uint8_t		(*sprom_get_rssismc2g)(device_t);
	uint8_t		(*sprom_get_rssismf2g)(device_t);
	uint8_t		(*sprom_get_bxa2g)(device_t);
	uint8_t		(*sprom_get_rssisav5g)(device_t);
	uint8_t		(*sprom_get_rssismc5g)(device_t);
	uint8_t		(*sprom_get_rssismf5g)(device_t);
	uint8_t		(*sprom_get_bxa5g)(device_t);
	uint16_t	(*sprom_get_cck2gpo)(device_t);
	uint32_t	(*sprom_get_ofdm2gpo)(device_t);
	uint32_t	(*sprom_get_ofdm5glpo)(device_t);
	uint32_t	(*sprom_get_ofdm5gpo)(device_t);
	uint32_t	(*sprom_get_ofdm5ghpo)(device_t);
	uint8_t		(*sprom_get_fem_2ghz_tssipos)(device_t);
	uint8_t		(*sprom_get_fem_2ghz_extpa_gain)(device_t);
	uint8_t		(*sprom_get_fem_2ghz_pdet_range)(device_t);
	uint8_t		(*sprom_get_fem_2ghz_tr_iso)(device_t);
	uint8_t		(*sprom_get_fem_2ghz_antswlut)(device_t);
	uint8_t		(*sprom_get_fem_5ghz_extpa_gain)(device_t);
	uint8_t		(*sprom_get_fem_5ghz_pdet_range)(device_t);
	uint8_t		(*sprom_get_fem_5ghz_antswlut)(device_t);
	uint8_t		(*sprom_get_txpid_2g_0)(device_t);
	uint8_t		(*sprom_get_txpid_2g_1)(device_t);
	uint8_t		(*sprom_get_txpid_5gl_0)(device_t);
	uint8_t		(*sprom_get_txpid_5gl_1)(device_t);
	uint8_t		(*sprom_get_txpid_5g_0)(device_t);
	uint8_t		(*sprom_get_txpid_5g_1)(device_t);
	uint8_t		(*sprom_get_txpid_5gh_0)(device_t);
	uint8_t		(*sprom_get_txpid_5gh_1)(device_t);
	uint16_t	(*sprom_get_stbcpo)(device_t);
	uint16_t	(*sprom_get_cddpo)(device_t);
	void		(*pcicore_intr)(device_t);
	int		(*sprom_get_mcs2gpo)(device_t, uint16_t *);
	int		(*sprom_get_mcs5glpo)(device_t, uint16_t *);
	int		(*sprom_get_mcs5gpo)(device_t, uint16_t *);
	int		(*sprom_get_mcs5ghpo)(device_t, uint16_t *);
	void		(*pmu_spuravoid_pllupdate)(device_t, int);
};

extern const struct bwn_bus_ops bwn_bhnd_bus_ops;

/*
 * Declared in:
 *    /usr/home/landonf/Documents/Code/FreeBSD/svn/head/sys/dev/siba/sibareg.h
 */

#define	SIBA_BOARDVENDOR_DELL		0x1028
#define	SIBA_BOARDVENDOR_BCM		0x14e4

#define	SIBA_BOARD_BCM4309G		0x0421
#define	SIBA_BOARD_BU4306		0x0416
#define	SIBA_BOARD_BCM4321		0x046d

/*
 * Declared in:
 *    /usr/home/landonf/Documents/Code/FreeBSD/svn/head/sys/dev/siba/sibavar.h
 */

enum siba_type {
	SIBA_TYPE_SSB			/* unused */,
	SIBA_TYPE_PCI,
	SIBA_TYPE_PCMCIA
};

/* TODO: need a real country code table */
enum {
	SIBA_CCODE_JAPAN,
	SIBA_CCODE_UNKNOWN
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
#define	siba_get_pcicore_revid(_dev)	\
	BWN_BUS_OPS(_dev)->get_pcicore_revid(_dev)
#define	siba_sprom_get_ccode(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_ccode(_dev)
#define	siba_sprom_get_ant_a(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_ant_a(_dev)
#define	siba_sprom_get_ant_bg(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_ant_bg(_dev)
#define	siba_sprom_get_pa0b0(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_pa0b0(_dev)
#define	siba_sprom_get_pa0b1(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_pa0b1(_dev)
#define	siba_sprom_get_pa0b2(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_pa0b2(_dev)
#define	siba_sprom_get_gpio0(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_gpio0(_dev)
#define	siba_sprom_get_gpio1(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_gpio1(_dev)
#define	siba_sprom_get_gpio2(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_gpio2(_dev)
#define	siba_sprom_get_gpio3(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_gpio3(_dev)
#define	siba_sprom_get_maxpwr_bg(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_maxpwr_bg(_dev)
#define	siba_sprom_set_maxpwr_bg(_dev, t)	\
	BWN_BUS_OPS(_dev)->sprom_set_maxpwr_bg(_dev, t)
#define	siba_sprom_get_rxpo2g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rxpo2g(_dev)
#define	siba_sprom_get_rxpo5g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rxpo5g(_dev)
#define	siba_sprom_get_tssi_bg(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_tssi_bg(_dev)
#define	siba_sprom_get_tri2g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_tri2g(_dev)
#define	siba_sprom_get_tri5gl(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_tri5gl(_dev)
#define	siba_sprom_get_tri5g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_tri5g(_dev)
#define	siba_sprom_get_tri5gh(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_tri5gh(_dev)
#define	siba_sprom_get_rssisav2g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rssisav2g(_dev)
#define	siba_sprom_get_rssismc2g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rssismc2g(_dev)
#define	siba_sprom_get_rssismf2g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rssismf2g(_dev)
#define	siba_sprom_get_bxa2g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_bxa2g(_dev)
#define	siba_sprom_get_rssisav5g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rssisav5g(_dev)
#define	siba_sprom_get_rssismc5g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rssismc5g(_dev)
#define	siba_sprom_get_rssismf5g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_rssismf5g(_dev)
#define	siba_sprom_get_bxa5g(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_bxa5g(_dev)
#define	siba_sprom_get_cck2gpo(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_cck2gpo(_dev)
#define	siba_sprom_get_ofdm2gpo(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_ofdm2gpo(_dev)
#define	siba_sprom_get_ofdm5glpo(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_ofdm5glpo(_dev)
#define	siba_sprom_get_ofdm5gpo(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_ofdm5gpo(_dev)
#define	siba_sprom_get_ofdm5ghpo(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_ofdm5ghpo(_dev)
#define	siba_sprom_get_fem_2ghz_tssipos(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_2ghz_tssipos(_dev)
#define	siba_sprom_get_fem_2ghz_extpa_gain(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_2ghz_extpa_gain(_dev)
#define	siba_sprom_get_fem_2ghz_pdet_range(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_2ghz_pdet_range(_dev)
#define	siba_sprom_get_fem_2ghz_tr_iso(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_2ghz_tr_iso(_dev)
#define	siba_sprom_get_fem_2ghz_antswlut(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_2ghz_antswlut(_dev)
#define	siba_sprom_get_fem_5ghz_extpa_gain(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_5ghz_extpa_gain(_dev)
#define	siba_sprom_get_fem_5ghz_pdet_range(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_5ghz_pdet_range(_dev)
#define	siba_sprom_get_fem_5ghz_antswlut(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_fem_5ghz_antswlut(_dev)
#define	siba_sprom_get_txpid_2g_0(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_2g_0(_dev)
#define	siba_sprom_get_txpid_2g_1(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_2g_1(_dev)
#define	siba_sprom_get_txpid_5gl_0(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_5gl_0(_dev)
#define	siba_sprom_get_txpid_5gl_1(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_5gl_1(_dev)
#define	siba_sprom_get_txpid_5g_0(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_5g_0(_dev)
#define	siba_sprom_get_txpid_5g_1(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_5g_1(_dev)
#define	siba_sprom_get_txpid_5gh_0(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_5gh_0(_dev)
#define	siba_sprom_get_txpid_5gh_1(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_txpid_5gh_1(_dev)
#define	siba_sprom_get_stbcpo(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_stbcpo(_dev)
#define	siba_sprom_get_cddpo(_dev)	\
	BWN_BUS_OPS(_dev)->sprom_get_cddpo(_dev)
#define	siba_sprom_get_mcs2gpo(_dev, _arg1)	\
	BWN_BUS_OPS(_dev)->sprom_get_mcs2gpo(_dev, _arg1)
#define	siba_sprom_get_mcs5glpo(_dev, _arg1)	\
	BWN_BUS_OPS(_dev)->sprom_get_mcs5glpo(_dev, _arg1)
#define	siba_sprom_get_mcs5gpo(_dev, _arg1)	\
	BWN_BUS_OPS(_dev)->sprom_get_mcs5gpo(_dev, _arg1)
#define	siba_sprom_get_mcs5ghpo(_dev, _arg1)	\
	BWN_BUS_OPS(_dev)->sprom_get_mcs5ghpo(_dev, _arg1)
#define	siba_pmu_spuravoid_pllupdate(_dev, _arg1)	\
	BWN_BUS_OPS(_dev)->pmu_spuravoid_pllupdate(_dev, _arg1)

#endif /* _IF_BWN_SIBA_H_ */
