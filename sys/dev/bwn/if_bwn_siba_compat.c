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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#define	BWN_USE_SIBA	0
#include "if_bwn_siba.h"


static int
bwn_bhnd_bus_ops_init(struct bwn_softc *sc)
{
	// TODO
	return (0);
}

static void
bwn_bhnd_bus_ops_fini(struct bwn_softc *sc)
{
}

/*
 * Disable PCI-specific MSI interrupt allocation handling
 */

/*
 * pci_find_cap()
 *
 * Referenced by:
 *   if_bwn.c:575
 */
static int
bhnd_compat_pci_find_cap(device_t dev, int capability, int *capreg)
{
	return (ENODEV);
}

/*
 * pci_alloc_msi()
 *
 * Referenced by:
 *   if_bwn.c:584
 */
static int
bhnd_compat_pci_alloc_msi(device_t dev, int *count)
{
	return (ENODEV);
}

/*
 * pci_release_msi()
 *
 * Referenced by:
 *   if_bwn.c:628, 751
 */
static int
bhnd_compat_pci_release_msi(device_t dev)
{
	return (ENODEV);
}

/*
 * pci_msi_count()
 *
 * Referenced by:
 *   if_bwn.c:576
 */
static int
bhnd_compat_pci_msi_count(device_t dev)
{
	return (0);
}

/*
 * siba_get_vendor()
 *
 * Referenced by:
 *   if_bwn.c:503
 */
static uint16_t
bhnd_compat_get_vendor(device_t dev)
{
	panic("siba_get_vendor() unimplemented");
}

/*
 * siba_get_device()
 *
 * Referenced by:
 *   if_bwn.c:504
 */
static uint16_t
bhnd_compat_get_device(device_t dev)
{
	panic("siba_get_device() unimplemented");
}

/*
 * siba_get_revid()
 *
 * Referenced by:
 *   if_bwn_phy_n_tables.c:3511, 3425
 *   if_bwn_phy_n_core.c:4321, 4212, 4195
 *   if_bwn_phy_g.c:3579
 *   if_bwn.c:6951, 6930
 *   if_bwn_phy_n_core.c:4300, 1557
 *   if_bwn.c:4542
 *   if_bwn_phy_n_core.c:6073, 6067
 *   if_bwn.c:3587, 3432, 3427, 2493...
 *   if_bwn_phy_n_core.c:606
 *   if_bwn.c:3481, 2138
 *   if_bwn_phy_g.c:3566
 *   if_bwn_phy_n_core.c:596
 *   if_bwn.c:1474, 2076, 1153
 *   if_bwn_phy_n_core.c:6631
 *   if_bwn.c:557, 3815, 2052, 505...
 */
static uint8_t
bhnd_compat_get_revid(device_t dev)
{
	panic("siba_get_revid() unimplemented");
}

/*
 * siba_get_pci_vendor()
 *
 * Referenced by:
 *   if_bwn.c:797, 799, 796, 795...
 */
static uint16_t
bhnd_compat_get_pci_vendor(device_t dev)
{
	panic("siba_get_pci_vendor() unimplemented");
}

/*
 * siba_get_pci_device()
 *
 * Referenced by:
 *   if_bwn.c:1206, 797, 532, 531...
 *   if_bwn_phy_n_core.c:6641
 *   if_bwn.c:1202, 533
 *   if_bwn_phy_n_core.c:6640
 *   if_bwn.c:798
 */
static uint16_t
bhnd_compat_get_pci_device(device_t dev)
{
	panic("siba_get_pci_device() unimplemented");
}

/*
 * siba_get_pci_subvendor()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6234
 *   if_bwn.c:6036, 797
 *   if_bwn_phy_g.c:1813, 3122
 *   if_bwn.c:7158
 *   if_bwn_phy_g.c:293
 *   if_bwn.c:799, 796, 795
 *   if_bwn_phy_n_core.c:6630
 *   if_bwn.c:794, 793, 783
 *   if_bwn_phy_g.c:317, 946
 *   if_bwn.c:788
 *   if_bwn_phy_g.c:343, 331, 349
 *   if_bwn_phy_n_core.c:1559
 *   if_bwn.c:798
 *   if_bwn_phy_g.c:323
 */
static uint16_t
bhnd_compat_get_pci_subvendor(device_t dev)
{
	panic("siba_get_pci_subvendor() unimplemented");
}

/*
 * siba_get_pci_subdevice()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:3123
 *   if_bwn.c:6037, 797
 *   if_bwn_phy_g.c:1814, 294, 325
 *   if_bwn_phy_n_core.c:6235
 *   if_bwn_phy_g.c:345, 333
 *   if_bwn.c:799, 796, 795, 794...
 *   if_bwn_phy_n_core.c:1560
 *   if_bwn_phy_g.c:319, 351
 *   if_bwn_phy_lp.c:1482
 *   if_bwn.c:798
 *   if_bwn_phy_g.c:947
 *   if_bwn_phy_n_core.c:3484
 */
static uint16_t
bhnd_compat_get_pci_subdevice(device_t dev)
{
	panic("siba_get_pci_subdevice() unimplemented");
}

/*
 * siba_get_pci_revid()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:1815, 298, 347
 *   if_bwn.c:789, 785
 *   if_bwn_phy_g.c:335, 321
 *   if_bwn_phy_lp.c:1305
 *   if_bwn_phy_g.c:295
 */
static uint8_t
bhnd_compat_get_pci_revid(device_t dev)
{
	panic("siba_get_pci_revid() unimplemented");
}

/*
 * siba_get_chipid()
 *
 * Referenced by:
 *   if_bwn_phy_n_tables.c:3510
 *   if_bwn_phy_lp.c:2739
 *   if_bwn_phy_n_core.c:1239
 *   if_bwn_phy_lp.c:1332, 1322
 *   if_bwn_phy_n_core.c:1281, 1251, 1238
 *   if_bwn_phy_lp.c:1506, 690
 *   if_bwn_phy_n_tables.c:3744
 *   if_bwn_phy_n_core.c:6474
 *   if_bwn.c:3471
 *   if_bwn_phy_n_core.c:1250
 *   if_bwn_phy_lp.c:1358
 *   if_bwn_phy_n_tables.c:3424
 *   if_bwn.c:1405, 789
 *   if_bwn_phy_g.c:354
 *   if_bwn_phy_n_core.c:1240
 *   if_bwn.c:557
 *   if_bwn_phy_g.c:928, 167
 *   if_bwn_phy_n_core.c:1282
 *   if_bwn_phy_common.c:91
 *   if_bwn_phy_lp.c:1880
 *   if_bwn.c:3594
 */
static uint16_t
bhnd_compat_get_chipid(device_t dev)
{
	panic("siba_get_chipid() unimplemented");
}

/*
 * siba_get_chiprev()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:2740, 1359, 1333, 1323
 *   if_bwn.c:3595, 1408, 1406
 */
static uint16_t
bhnd_compat_get_chiprev(device_t dev)
{
	panic("siba_get_chiprev() unimplemented");
}

/*
 * siba_get_chippkg()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:1241
 *   if_bwn_phy_lp.c:1507
 *   if_bwn_phy_g.c:929
 */
static uint8_t
bhnd_compat_get_chippkg(device_t dev)
{
	panic("siba_get_chippkg() unimplemented");
}

/*
 * siba_get_type()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6639
 *   if_bwn.c:6948, 792, 2100, 2069
 */
static enum siba_type
bhnd_compat_get_type(device_t dev)
{
	panic("siba_get_type() unimplemented");
}

/*
 * siba_get_cc_pmufreq()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:1677, 1595, 800, 901
 */
static uint32_t
bhnd_compat_get_cc_pmufreq(device_t dev)
{
	panic("siba_get_cc_pmufreq() unimplemented");
}

/*
 * siba_get_cc_caps()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:1593
 */
static uint32_t
bhnd_compat_get_cc_caps(device_t dev)
{
	panic("siba_get_cc_caps() unimplemented");
}

/*
 * siba_get_cc_powerdelay()
 *
 * Referenced by:
 *   if_bwn.c:2267
 */
static uint16_t
bhnd_compat_get_cc_powerdelay(device_t dev)
{
	panic("siba_get_cc_powerdelay() unimplemented");
}

/*
 * siba_get_pcicore_revid()
 *
 * Referenced by:
 *   if_bwn.c:2070
 */
static uint8_t
bhnd_compat_get_pcicore_revid(device_t dev)
{
	panic("siba_get_pcicore_revid() unimplemented");
}

/*
 * siba_sprom_get_rev()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:4219, 3878, 3079, 6634
 */
static uint8_t
bhnd_compat_sprom_get_rev(device_t dev)
{
	panic("siba_sprom_get_rev() unimplemented");
}

/*
 * siba_sprom_get_mac_80211bg()
 *
 * Referenced by:
 *   if_bwn.c:671
 */
static uint8_t *
bhnd_compat_sprom_get_mac_80211bg(device_t dev)
{
	panic("siba_sprom_get_mac_80211bg() unimplemented");
}

/*
 * siba_sprom_get_mac_80211a()
 *
 * Referenced by:
 *   if_bwn.c:671
 */
static uint8_t *
bhnd_compat_sprom_get_mac_80211a(device_t dev)
{
	panic("siba_sprom_get_mac_80211a() unimplemented");
}

/*
 * siba_sprom_get_brev()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:1561
 */
static uint8_t
bhnd_compat_sprom_get_brev(device_t dev)
{
	panic("siba_sprom_get_brev() unimplemented");
}

/*
 * siba_sprom_get_ccode()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:3268
 */
static uint8_t
bhnd_compat_sprom_get_ccode(device_t dev)
{
	panic("siba_sprom_get_ccode() unimplemented");
}

/*
 * siba_sprom_get_ant_a()
 *
 * Referenced by:
 *   if_bwn.c:6523
 */
static uint8_t
bhnd_compat_sprom_get_ant_a(device_t dev)
{
	panic("siba_sprom_get_ant_a() unimplemented");
}

/*
 * siba_sprom_get_ant_bg()
 *
 * Referenced by:
 *   if_bwn.c:6521
 */
static uint8_t
bhnd_compat_sprom_get_ant_bg(device_t dev)
{
	panic("siba_sprom_get_ant_bg() unimplemented");
}

/*
 * siba_sprom_get_pa0b0()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:163
 */
static uint16_t
bhnd_compat_sprom_get_pa0b0(device_t dev)
{
	panic("siba_sprom_get_pa0b0() unimplemented");
}

/*
 * siba_sprom_get_pa0b1()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:164
 */
static uint16_t
bhnd_compat_sprom_get_pa0b1(device_t dev)
{
	panic("siba_sprom_get_pa0b1() unimplemented");
}

/*
 * siba_sprom_get_pa0b2()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:165
 */
static uint16_t
bhnd_compat_sprom_get_pa0b2(device_t dev)
{
	panic("siba_sprom_get_pa0b2() unimplemented");
}

/*
 * siba_sprom_get_gpio0()
 *
 * Referenced by:
 *   if_bwn.c:7167
 */
static uint8_t
bhnd_compat_sprom_get_gpio0(device_t dev)
{
	panic("siba_sprom_get_gpio0() unimplemented");
}

/*
 * siba_sprom_get_gpio1()
 *
 * Referenced by:
 *   if_bwn.c:7168
 */
static uint8_t
bhnd_compat_sprom_get_gpio1(device_t dev)
{
	panic("siba_sprom_get_gpio1() unimplemented");
}

/*
 * siba_sprom_get_gpio2()
 *
 * Referenced by:
 *   if_bwn.c:7169
 */
static uint8_t
bhnd_compat_sprom_get_gpio2(device_t dev)
{
	panic("siba_sprom_get_gpio2() unimplemented");
}

/*
 * siba_sprom_get_gpio3()
 *
 * Referenced by:
 *   if_bwn.c:7170
 */
static uint8_t
bhnd_compat_sprom_get_gpio3(device_t dev)
{
	panic("siba_sprom_get_gpio3() unimplemented");
}

/*
 * siba_sprom_get_maxpwr_bg()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:660
 */
static uint16_t
bhnd_compat_sprom_get_maxpwr_bg(device_t dev)
{
	panic("siba_sprom_get_maxpwr_bg() unimplemented");
}

/*
 * siba_sprom_set_maxpwr_bg()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:666
 */
static void
bhnd_compat_sprom_set_maxpwr_bg(device_t dev, uint16_t t)
{
	panic("siba_sprom_set_maxpwr_bg() unimplemented");
}

/*
 * siba_sprom_get_rxpo2g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:600
 */
static uint8_t
bhnd_compat_sprom_get_rxpo2g(device_t dev)
{
	panic("siba_sprom_get_rxpo2g() unimplemented");
}

/*
 * siba_sprom_get_rxpo5g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:611
 */
static uint8_t
bhnd_compat_sprom_get_rxpo5g(device_t dev)
{
	panic("siba_sprom_get_rxpo5g() unimplemented");
}

/*
 * siba_sprom_get_tssi_bg()
 *
 * Referenced by:
 *   if_bwn_phy_g.c:162
 */
static uint8_t
bhnd_compat_sprom_get_tssi_bg(device_t dev)
{
	panic("siba_sprom_get_tssi_bg() unimplemented");
}

/*
 * siba_sprom_get_tri2g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:598
 */
static uint8_t
bhnd_compat_sprom_get_tri2g(device_t dev)
{
	panic("siba_sprom_get_tri2g() unimplemented");
}

/*
 * siba_sprom_get_tri5gl()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:607
 */
static uint8_t
bhnd_compat_sprom_get_tri5gl(device_t dev)
{
	panic("siba_sprom_get_tri5gl() unimplemented");
}

/*
 * siba_sprom_get_tri5g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:608
 */
static uint8_t
bhnd_compat_sprom_get_tri5g(device_t dev)
{
	panic("siba_sprom_get_tri5g() unimplemented");
}

/*
 * siba_sprom_get_tri5gh()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:609
 */
static uint8_t
bhnd_compat_sprom_get_tri5gh(device_t dev)
{
	panic("siba_sprom_get_tri5gh() unimplemented");
}

/*
 * siba_sprom_get_rssisav2g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:603
 */
static uint8_t
bhnd_compat_sprom_get_rssisav2g(device_t dev)
{
	panic("siba_sprom_get_rssisav2g() unimplemented");
}

/*
 * siba_sprom_get_rssismc2g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:602
 */
static uint8_t
bhnd_compat_sprom_get_rssismc2g(device_t dev)
{
	panic("siba_sprom_get_rssismc2g() unimplemented");
}

/*
 * siba_sprom_get_rssismf2g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:601
 */
static uint8_t
bhnd_compat_sprom_get_rssismf2g(device_t dev)
{
	panic("siba_sprom_get_rssismf2g() unimplemented");
}

/*
 * siba_sprom_get_bxa2g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:599
 */
static uint8_t
bhnd_compat_sprom_get_bxa2g(device_t dev)
{
	panic("siba_sprom_get_bxa2g() unimplemented");
}

/*
 * siba_sprom_get_rssisav5g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:614
 */
static uint8_t
bhnd_compat_sprom_get_rssisav5g(device_t dev)
{
	panic("siba_sprom_get_rssisav5g() unimplemented");
}

/*
 * siba_sprom_get_rssismc5g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:613
 */
static uint8_t
bhnd_compat_sprom_get_rssismc5g(device_t dev)
{
	panic("siba_sprom_get_rssismc5g() unimplemented");
}

/*
 * siba_sprom_get_rssismf5g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:612
 */
static uint8_t
bhnd_compat_sprom_get_rssismf5g(device_t dev)
{
	panic("siba_sprom_get_rssismf5g() unimplemented");
}

/*
 * siba_sprom_get_bxa5g()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:610
 */
static uint8_t
bhnd_compat_sprom_get_bxa5g(device_t dev)
{
	panic("siba_sprom_get_bxa5g() unimplemented");
}

/*
 * siba_sprom_get_cck2gpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:204
 */
static uint16_t
bhnd_compat_sprom_get_cck2gpo(device_t dev)
{
	panic("siba_sprom_get_cck2gpo() unimplemented");
}

/*
 * siba_sprom_get_ofdm2gpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:166
 */
static uint32_t
bhnd_compat_sprom_get_ofdm2gpo(device_t dev)
{
	panic("siba_sprom_get_ofdm2gpo() unimplemented");
}

/*
 * siba_sprom_get_ofdm5glpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:174
 */
static uint32_t
bhnd_compat_sprom_get_ofdm5glpo(device_t dev)
{
	panic("siba_sprom_get_ofdm5glpo() unimplemented");
}

/*
 * siba_sprom_get_ofdm5gpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:182
 */
static uint32_t
bhnd_compat_sprom_get_ofdm5gpo(device_t dev)
{
	panic("siba_sprom_get_ofdm5gpo() unimplemented");
}

/*
 * siba_sprom_get_ofdm5ghpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:190
 */
static uint32_t
bhnd_compat_sprom_get_ofdm5ghpo(device_t dev)
{
	panic("siba_sprom_get_ofdm5ghpo() unimplemented");
}

/*
 * siba_sprom_get_bf_lo()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:3382, 3370
 *   if_bwn_phy_n_core.c:6176
 *   if_bwn_phy_g.c:3429, 3385
 *   if_bwn_phy_lp.c:1484, 1489
 *   if_bwn.c:5725
 *   if_bwn_phy_g.c:2679, 1824, 661, 2220
 *   if_bwn.c:3387
 *   if_bwn_phy_g.c:1931, 1543, 1445
 *   if_bwn.c:801
 *   if_bwn_phy_lp.c:1453
 *   if_bwn.c:2056
 *   if_bwn_phy_n_core.c:2625
 *   if_bwn_phy_g.c:1115, 1843
 *   if_bwn.c:791, 787, 3475
 *   if_bwn_phy_g.c:896, 909
 *   if_bwn.c:2110
 *   if_bwn_phy_g.c:809
 *   if_bwn.c:3381
 *   if_bwn_phy_g.c:710
 *   if_bwn.c:2067
 */
static uint16_t
bhnd_compat_sprom_get_bf_lo(device_t dev)
{
	panic("siba_sprom_get_bf_lo() unimplemented");
}

/*
 * siba_sprom_set_bf_lo()
 *
 * Referenced by:
 *   if_bwn.c:800, 786, 790
 */
static void
bhnd_compat_sprom_set_bf_lo(device_t dev, uint16_t t)
{
	panic("siba_sprom_set_bf_lo() unimplemented");
}

/*
 * siba_sprom_get_bf_hi()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:3369, 1505, 1499, 1477...
 *   if_bwn_phy_n_core.c:2624
 *   if_bwn_phy_lp.c:1455, 3381, 3357
 */
static uint16_t
bhnd_compat_sprom_get_bf_hi(device_t dev)
{
	panic("siba_sprom_get_bf_hi() unimplemented");
}

/*
 * siba_sprom_get_bf2_lo()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6636, 3467, 1246, 1266...
 */
static uint16_t
bhnd_compat_sprom_get_bf2_lo(device_t dev)
{
	panic("siba_sprom_get_bf2_lo() unimplemented");
}

/*
 * siba_sprom_get_bf2_hi()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6186, 1259, 3080
 */
static uint16_t
bhnd_compat_sprom_get_bf2_hi(device_t dev)
{
	panic("siba_sprom_get_bf2_hi() unimplemented");
}

/*
 * siba_sprom_get_fem_2ghz_tssipos()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:4276
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_tssipos(device_t dev)
{
	panic("siba_sprom_get_fem_2ghz_tssipos() unimplemented");
}

/*
 * siba_sprom_get_fem_2ghz_extpa_gain()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6650
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_extpa_gain(device_t dev)
{
	panic("siba_sprom_get_fem_2ghz_extpa_gain() unimplemented");
}

/*
 * siba_sprom_get_fem_2ghz_pdet_range()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3353
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_pdet_range(device_t dev)
{
	panic("siba_sprom_get_fem_2ghz_pdet_range() unimplemented");
}

/*
 * siba_sprom_get_fem_2ghz_tr_iso()
 *
 * Referenced by:
 *   if_bwn_phy_n_tables.c:3885
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_tr_iso(device_t dev)
{
	panic("siba_sprom_get_fem_2ghz_tr_iso() unimplemented");
}

/*
 * siba_sprom_get_fem_2ghz_antswlut()
 *
 * Referenced by:
 *   if_bwn_phy_n_tables.c:3644, 3579
 */
static uint8_t
bhnd_compat_sprom_get_fem_2ghz_antswlut(device_t dev)
{
	panic("siba_sprom_get_fem_2ghz_antswlut() unimplemented");
}

/*
 * siba_sprom_get_fem_5ghz_extpa_gain()
 *
 * Referenced by:
 *   if_bwn_phy_n_tables.c:3810, 3796
 *   if_bwn_phy_n_core.c:6651
 */
static uint8_t
bhnd_compat_sprom_get_fem_5ghz_extpa_gain(device_t dev)
{
	panic("siba_sprom_get_fem_5ghz_extpa_gain() unimplemented");
}

/*
 * siba_sprom_get_fem_5ghz_pdet_range()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3355
 */
static uint8_t
bhnd_compat_sprom_get_fem_5ghz_pdet_range(device_t dev)
{
	panic("siba_sprom_get_fem_5ghz_pdet_range() unimplemented");
}

/*
 * siba_sprom_get_fem_5ghz_antswlut()
 *
 * Referenced by:
 *   if_bwn_phy_n_tables.c:3642, 3577
 */
static uint8_t
bhnd_compat_sprom_get_fem_5ghz_antswlut(device_t dev)
{
	panic("siba_sprom_get_fem_5ghz_antswlut() unimplemented");
}

/*
 * siba_sprom_get_txpid_2g_0()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3883
 */
static uint8_t
bhnd_compat_sprom_get_txpid_2g_0(device_t dev)
{
	panic("siba_sprom_get_txpid_2g_0() unimplemented");
}

/*
 * siba_sprom_get_txpid_2g_1()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3884
 */
static uint8_t
bhnd_compat_sprom_get_txpid_2g_1(device_t dev)
{
	panic("siba_sprom_get_txpid_2g_1() unimplemented");
}

/*
 * siba_sprom_get_txpid_5gl_0()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3886
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gl_0(device_t dev)
{
	panic("siba_sprom_get_txpid_5gl_0() unimplemented");
}

/*
 * siba_sprom_get_txpid_5gl_1()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3887
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gl_1(device_t dev)
{
	panic("siba_sprom_get_txpid_5gl_1() unimplemented");
}

/*
 * siba_sprom_get_txpid_5g_0()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3889
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5g_0(device_t dev)
{
	panic("siba_sprom_get_txpid_5g_0() unimplemented");
}

/*
 * siba_sprom_get_txpid_5g_1()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3890
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5g_1(device_t dev)
{
	panic("siba_sprom_get_txpid_5g_1() unimplemented");
}

/*
 * siba_sprom_get_txpid_5gh_0()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3892
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gh_0(device_t dev)
{
	panic("siba_sprom_get_txpid_5gh_0() unimplemented");
}

/*
 * siba_sprom_get_txpid_5gh_1()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:3893
 */
static uint8_t
bhnd_compat_sprom_get_txpid_5gh_1(device_t dev)
{
	panic("siba_sprom_get_txpid_5gh_1() unimplemented");
}

/*
 * siba_sprom_get_stbcpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:193, 185, 177, 169
 */
static uint16_t
bhnd_compat_sprom_get_stbcpo(device_t dev)
{
	panic("siba_sprom_get_stbcpo() unimplemented");
}

/*
 * siba_sprom_get_cddpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:192, 184, 176, 168
 */
static uint16_t
bhnd_compat_sprom_get_cddpo(device_t dev)
{
	panic("siba_sprom_get_cddpo() unimplemented");
}

/*
 * siba_powerup()
 *
 * Referenced by:
 *   if_bwn.c:2014, 1159, 2109
 */
static void
bhnd_compat_powerup(device_t dev, int dynamic)
{
	panic("siba_powerup() unimplemented");
}

/*
 * siba_powerdown()
 *
 * Referenced by:
 *   if_bwn.c:2180, 1331, 2122
 */
static int
bhnd_compat_powerdown(device_t dev)
{
	panic("siba_powerdown() unimplemented");
}

/*
 * siba_read_2()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6406, 6148, 6147, 6692
 *   if_bwn_phy_g.c:3600, 3337, 3279, 3128
 *   if_bwn.c:7405, 7346, 7236, 6057
 *   if_bwn_phy_n_core.c:6693
 *   if_bwn_phy_g.c:2232, 2231, 2761, 2755...
 *   if_bwn.c:4416
 *   if_bwn_phy_g.c:2253, 2250
 *   if_bwn_phy_lp.c:482
 *   if_bwn.c:3692, 3704, 3476, 3468...
 *   if_bwn_phy_g.c:3274
 *   if_bwn_phy_lp.c:496
 *   if_bwn_phy_g.c:2807
 *   if_bwn_phy_lp.c:465
 *   if_bwn.c:1390, 2517
 *   if_bwn_phy_g.c:1259, 1247
 *   if_bwn_phy_n.c:187
 *   if_bwn_phy_g.c:1391, 1215
 *   if_bwn.c:1548, 1480, 7324
 *   if_bwn_phy_g.c:489
 *   if_bwn.c:1416, 1414, 1571, 1545...
 *   if_bwn_phy_n_core.c:6416
 *   if_bwn.c:1577, 3687
 *   if_bwn_phy_g.c:2762, 1028
 *   if_bwn_phy_n.c:161
 *   if_bwn_phy_g.c:1251, 1252
 *   if_bwn.c:2234
 *   if_bwn_phy_g.c:472
 *   if_bwn.c:3698
 */
static uint16_t
bhnd_compat_read_2(device_t dev, uint16_t offset)
{
	panic("siba_read_2() unimplemented");
}

/*
 * siba_write_2()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6421, 6411, 6407, 6148...
 *   if_bwn_phy_g.c:3478, 3475, 3337, 3279...
 *   if_bwn.c:7348, 7326, 7278, 6662...
 *   if_bwn_phy_common.c:100
 *   if_bwn_phy_g.c:2864
 *   if_bwn_phy_n_core.c:6693
 *   if_bwn.c:5095
 *   if_bwn_phy_g.c:2755
 *   if_bwn_phy_common.c:126
 *   if_bwn_phy_g.c:2334, 2323
 *   if_bwn.c:4490, 4487, 4484
 *   if_bwn_phy_g.c:2253, 2250
 *   if_bwn_phy_lp.c:482
 *   if_bwn.c:4325, 6677
 *   if_bwn_phy_lp.c:1158
 *   if_bwn.c:6663
 *   if_bwn_phy_lp.c:473
 *   if_bwn.c:3682, 3676, 3668, 3667...
 *   if_bwn_phy_g.c:479, 2863
 *   if_bwn_phy_lp.c:661, 660
 *   if_bwn.c:3353
 *   if_bwn_phy_g.c:3274, 1512, 2807
 *   if_bwn_phy_lp.c:472, 464
 *   if_bwn_phy_g.c:2862, 1384, 1382
 *   if_bwn.c:2267, 2250, 2249, 2248...
 *   if_bwn_phy_n_core.c:6417
 *   if_bwn.c:4811
 *   if_bwn_phy_g.c:1259, 1389, 995
 *   if_bwn_phy_common.c:101
 *   if_bwn_phy_g.c:1247
 *   if_bwn_phy_n.c:200, 199
 *   if_bwn_phy_g.c:1391
 *   if_bwn_phy_n.c:186, 169
 *   if_bwn_phy_g.c:1218
 *   if_bwn.c:6071, 1629, 1609
 *   if_bwn_phy_g.c:488
 *   if_bwn.c:1476, 3669, 1413, 1011...
 *   if_bwn_phy_g.c:987, 1255
 *   if_bwn_phy_common.c:133
 *   if_bwn_phy_n.c:168
 *   if_bwn_phy_lp.c:495
 *   if_bwn_phy_g.c:963
 *   if_bwn.c:3655
 *   if_bwn_phy_g.c:825
 *   if_bwn_phy_lp.c:554, 481
 *   if_bwn_phy_common.c:118
 *   if_bwn_phy_g.c:1028
 *   if_bwn.c:1415
 *   if_bwn_phy_n.c:160
 *   if_bwn_phy_g.c:480
 *   if_bwn.c:4480
 *   if_bwn_phy_g.c:471
 *   if_bwn_phy_common.c:137, 122, 125
 *   if_bwn.c:2234
 *   if_bwn_phy_common.c:136, 105, 104
 *   if_bwn_phy_g.c:497
 *   if_bwn_phy_common.c:97, 132, 121
 *   if_bwn_phy_g.c:498
 *   if_bwn_phy_common.c:96, 117
 *   if_bwn_phy_lp.c:504, 505
 *   if_bwn_phy_g.c:1519
 */
static void
bhnd_compat_write_2(device_t dev, uint16_t offset, uint16_t value)
{
	panic("siba_write_2() unimplemented");
}

/*
 * siba_read_4()
 *
 * Referenced by:
 *   if_bwn_radio_2055.c:1373
 *   if_bwn_phy_n_core.c:6146, 6069, 4322, 4302...
 *   if_bwn_phy_g.c:3601
 *   if_bwn.c:7401, 6934, 6933, 5205...
 *   if_bwn_phy_n_core.c:1535, 1512, 1506, 1500...
 *   if_bwn.c:6064, 5078, 4267, 4185...
 *   if_bwn_phy_n_core.c:6074
 *   if_bwn.c:3788
 *   if_bwn_phy_common.c:189
 *   if_bwn.c:2911, 1365, 4724
 *   if_bwn_phy_g.c:3590
 *   if_bwn.c:1362, 4728, 1376, 2903
 *   if_bwn_radio_2055.c:1369
 *   if_bwn_phy_g.c:418
 *   if_bwn.c:3744, 1370, 2530
 *   if_bwn_phy_g.c:3592
 *   if_bwn.c:3811, 3221, 5073, 997...
 *   if_bwn_phy_n_core.c:6068
 *   if_bwn_phy_common.c:154
 */
static uint32_t
bhnd_compat_read_4(device_t dev, uint16_t offset)
{
	panic("siba_read_4() unimplemented");
}

/*
 * siba_write_4()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6146, 4322, 4301, 4213...
 *   if_bwn_phy_g.c:3601
 *   if_bwn.c:6612, 2259, 6078, 2237...
 *   if_bwn_phy_n_core.c:6074
 *   if_bwn.c:4844, 3788, 2534, 4163
 *   if_bwn_phy_common.c:194
 *   if_bwn.c:2911, 1364
 *   if_bwn_phy_g.c:3590
 *   if_bwn.c:4727, 2895, 2903, 3744...
 *   if_bwn_phy_common.c:159
 *   if_bwn.c:3124, 3125, 3790, 5075...
 *   if_bwn_phy_n_core.c:6068
 */
static void
bhnd_compat_write_4(device_t dev, uint16_t offset, uint32_t value)
{
	panic("siba_write_4() unimplemented");
}

/*
 * siba_dev_up()
 *
 * Referenced by:
 *   if_bwn.c:1358
 */
static void
bhnd_compat_dev_up(device_t dev, uint32_t flags)
{
	panic("siba_dev_up() unimplemented");
}

/*
 * siba_dev_down()
 *
 * Referenced by:
 *   if_bwn.c:2179, 1329
 */
static void
bhnd_compat_dev_down(device_t dev, uint32_t flags)
{
	panic("siba_dev_down() unimplemented");
}

/*
 * siba_dev_isup()
 *
 * Referenced by:
 *   if_bwn.c:2015
 */
static int
bhnd_compat_dev_isup(device_t dev)
{
	panic("siba_dev_isup() unimplemented");
}

/*
 * siba_pcicore_intr()
 *
 * Referenced by:
 *   if_bwn.c:2038
 */
static void
bhnd_compat_pcicore_intr(device_t dev)
{
	panic("siba_pcicore_intr() unimplemented");
}

/*
 * siba_dma_translation()
 *
 * Referenced by:
 *   if_bwn.c:3080, 2967, 2874
 */
static uint32_t
bhnd_compat_dma_translation(device_t dev)
{
	panic("siba_dma_translation() unimplemented");
}

/*
 * siba_read_multi_2()
 *
 * Referenced by:
 *   if_bwn.c:5500, 5556
 */
static void
bhnd_compat_read_multi_2(device_t dev, void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_read_multi_2() unimplemented");
}

/*
 * siba_read_multi_4()
 *
 * Referenced by:
 *   if_bwn.c:5497, 5539
 */
static void
bhnd_compat_read_multi_4(device_t dev, void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_read_multi_4() unimplemented");
}

/*
 * siba_write_multi_2()
 *
 * Referenced by:
 *   if_bwn.c:6625
 */
static void
bhnd_compat_write_multi_2(device_t dev, const void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_write_multi_2() unimplemented");
}

/*
 * siba_write_multi_4()
 *
 * Referenced by:
 *   if_bwn.c:6582
 */
static void
bhnd_compat_write_multi_4(device_t dev, const void *buffer, size_t count,
    uint16_t offset)
{
	panic("siba_write_multi_4() unimplemented");
}

/*
 * siba_barrier()
 *
 * Referenced by:
 *   if_bwn.c:4966, 4853, 4852, 4967...
 */
static void
bhnd_compat_barrier(device_t dev, int flags)
{
	panic("siba_barrier() unimplemented");
}

/*
 * siba_cc_pmu_set_ldovolt()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:1456
 */
static void
bhnd_compat_cc_pmu_set_ldovolt(device_t dev, int id, uint32_t volt)
{
	panic("siba_cc_pmu_set_ldovolt() unimplemented");
}

/*
 * siba_cc_pmu_set_ldoparef()
 *
 * Referenced by:
 *   if_bwn_phy_lp.c:1463, 1457
 */
static void
bhnd_compat_cc_pmu_set_ldoparef(device_t dev, uint8_t on)
{
	panic("siba_cc_pmu_set_ldoparef() unimplemented");
}

/*
 * siba_gpio_set()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6143
 *   if_bwn.c:3443, 2224, 2218, 3487
 */
static void
bhnd_compat_gpio_set(device_t dev, uint32_t value)
{
	panic("siba_gpio_set() unimplemented");
}

/*
 * siba_gpio_get()
 *
 * Referenced by:
 *   if_bwn.c:3484
 */
static uint32_t
bhnd_compat_gpio_get(device_t dev)
{
	panic("siba_gpio_get() unimplemented");
}

/*
 * siba_fix_imcfglobug()
 *
 * Referenced by:
 *   if_bwn.c:2040
 */
static void
bhnd_compat_fix_imcfglobug(device_t dev)
{
	panic("siba_fix_imcfglobug() unimplemented");
}

/*
 * siba_sprom_get_core_power_info()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:153
 *   if_bwn_phy_n_core.c:4186
 */
static int
bhnd_compat_sprom_get_core_power_info(device_t dev, int core,
    struct siba_sprom_core_pwr_info *c)
{
	panic("siba_sprom_get_core_power_info() unimplemented");
}

/*
 * siba_sprom_get_mcs2gpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:167
 */
static int
bhnd_compat_sprom_get_mcs2gpo(device_t dev, uint16_t *c)
{
	panic("siba_sprom_get_mcs2gpo() unimplemented");
}

/*
 * siba_sprom_get_mcs5glpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:175
 */
static int
bhnd_compat_sprom_get_mcs5glpo(device_t dev, uint16_t *c)
{
	panic("siba_sprom_get_mcs5glpo() unimplemented");
}

/*
 * siba_sprom_get_mcs5gpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:183
 */
static int
bhnd_compat_sprom_get_mcs5gpo(device_t dev, uint16_t *c)
{
	panic("siba_sprom_get_mcs5gpo() unimplemented");
}

/*
 * siba_sprom_get_mcs5ghpo()
 *
 * Referenced by:
 *   if_bwn_phy_n_ppr.c:191
 */
static int
bhnd_compat_sprom_get_mcs5ghpo(device_t dev, uint16_t *c)
{
	panic("siba_sprom_get_mcs5ghpo() unimplemented");
}

/*
 * siba_pmu_spuravoid_pllupdate()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6386
 */
static void
bhnd_compat_pmu_spuravoid_pllupdate(device_t dev, int spur_avoid)
{
	panic("siba_pmu_spuravoid_pllupdate() unimplemented");
}

/*
 * siba_cc_set32()
 *
 * Referenced by:
 *   if_bwn_phy_n_core.c:6180
 *   if_bwn_phy_common.c:210
 */
static void
bhnd_compat_cc_set32(device_t dev, uint32_t reg, uint32_t val)
{
	panic("siba_cc_set32() unimplemented");
}

/*
 * siba_cc_mask32()
 *
 * Referenced by:
 *   if_bwn_phy_common.c:209, 211
 */
static void
bhnd_compat_cc_mask32(device_t dev, uint32_t reg, uint32_t mask)
{
	panic("siba_cc_mask32() unimplemented");
}

/*
 * siba_cc_write32()
 *
 * Referenced by:
 *   if_bwn_phy_common.c:208
 */
static void
bhnd_compat_cc_write32(device_t dev, uint32_t reg, uint32_t val)
{
	panic("siba_cc_write32() unimplemented");
}

const struct bwn_bus_ops bwn_bhnd_bus_ops = {
	.init				= bwn_bhnd_bus_ops_init,
	.fini				= bwn_bhnd_bus_ops_fini,
	.pci_find_cap			= bhnd_compat_pci_find_cap,
	.pci_alloc_msi			= bhnd_compat_pci_alloc_msi,
	.pci_release_msi		= bhnd_compat_pci_release_msi,
	.pci_msi_count			= bhnd_compat_pci_msi_count,
	.get_vendor			= bhnd_compat_get_vendor,
	.get_device			= bhnd_compat_get_device,
	.get_revid			= bhnd_compat_get_revid,
	.get_pci_vendor			= bhnd_compat_get_pci_vendor,
	.get_pci_device			= bhnd_compat_get_pci_device,
	.get_pci_subvendor		= bhnd_compat_get_pci_subvendor,
	.get_pci_subdevice		= bhnd_compat_get_pci_subdevice,
	.get_pci_revid			= bhnd_compat_get_pci_revid,
	.get_chipid			= bhnd_compat_get_chipid,
	.get_chiprev			= bhnd_compat_get_chiprev,
	.get_chippkg			= bhnd_compat_get_chippkg,
	.get_type			= bhnd_compat_get_type,
	.get_cc_pmufreq			= bhnd_compat_get_cc_pmufreq,
	.get_cc_caps			= bhnd_compat_get_cc_caps,
	.get_cc_powerdelay		= bhnd_compat_get_cc_powerdelay,
	.get_pcicore_revid		= bhnd_compat_get_pcicore_revid,
	.sprom_get_rev			= bhnd_compat_sprom_get_rev,
	.sprom_get_mac_80211bg		= bhnd_compat_sprom_get_mac_80211bg,
	.sprom_get_mac_80211a		= bhnd_compat_sprom_get_mac_80211a,
	.sprom_get_brev			= bhnd_compat_sprom_get_brev,
	.sprom_get_ccode		= bhnd_compat_sprom_get_ccode,
	.sprom_get_ant_a		= bhnd_compat_sprom_get_ant_a,
	.sprom_get_ant_bg		= bhnd_compat_sprom_get_ant_bg,
	.sprom_get_pa0b0		= bhnd_compat_sprom_get_pa0b0,
	.sprom_get_pa0b1		= bhnd_compat_sprom_get_pa0b1,
	.sprom_get_pa0b2		= bhnd_compat_sprom_get_pa0b2,
	.sprom_get_gpio0		= bhnd_compat_sprom_get_gpio0,
	.sprom_get_gpio1		= bhnd_compat_sprom_get_gpio1,
	.sprom_get_gpio2		= bhnd_compat_sprom_get_gpio2,
	.sprom_get_gpio3		= bhnd_compat_sprom_get_gpio3,
	.sprom_get_maxpwr_bg		= bhnd_compat_sprom_get_maxpwr_bg,
	.sprom_set_maxpwr_bg		= bhnd_compat_sprom_set_maxpwr_bg,
	.sprom_get_rxpo2g		= bhnd_compat_sprom_get_rxpo2g,
	.sprom_get_rxpo5g		= bhnd_compat_sprom_get_rxpo5g,
	.sprom_get_tssi_bg		= bhnd_compat_sprom_get_tssi_bg,
	.sprom_get_tri2g		= bhnd_compat_sprom_get_tri2g,
	.sprom_get_tri5gl		= bhnd_compat_sprom_get_tri5gl,
	.sprom_get_tri5g		= bhnd_compat_sprom_get_tri5g,
	.sprom_get_tri5gh		= bhnd_compat_sprom_get_tri5gh,
	.sprom_get_rssisav2g		= bhnd_compat_sprom_get_rssisav2g,
	.sprom_get_rssismc2g		= bhnd_compat_sprom_get_rssismc2g,
	.sprom_get_rssismf2g		= bhnd_compat_sprom_get_rssismf2g,
	.sprom_get_bxa2g		= bhnd_compat_sprom_get_bxa2g,
	.sprom_get_rssisav5g		= bhnd_compat_sprom_get_rssisav5g,
	.sprom_get_rssismc5g		= bhnd_compat_sprom_get_rssismc5g,
	.sprom_get_rssismf5g		= bhnd_compat_sprom_get_rssismf5g,
	.sprom_get_bxa5g		= bhnd_compat_sprom_get_bxa5g,
	.sprom_get_cck2gpo		= bhnd_compat_sprom_get_cck2gpo,
	.sprom_get_ofdm2gpo		= bhnd_compat_sprom_get_ofdm2gpo,
	.sprom_get_ofdm5glpo		= bhnd_compat_sprom_get_ofdm5glpo,
	.sprom_get_ofdm5gpo		= bhnd_compat_sprom_get_ofdm5gpo,
	.sprom_get_ofdm5ghpo		= bhnd_compat_sprom_get_ofdm5ghpo,
	.sprom_get_bf_lo		= bhnd_compat_sprom_get_bf_lo,
	.sprom_set_bf_lo		= bhnd_compat_sprom_set_bf_lo,
	.sprom_get_bf_hi		= bhnd_compat_sprom_get_bf_hi,
	.sprom_get_bf2_lo		= bhnd_compat_sprom_get_bf2_lo,
	.sprom_get_bf2_hi		= bhnd_compat_sprom_get_bf2_hi,
	.sprom_get_fem_2ghz_tssipos	= bhnd_compat_sprom_get_fem_2ghz_tssipos,
	.sprom_get_fem_2ghz_extpa_gain	= bhnd_compat_sprom_get_fem_2ghz_extpa_gain,
	.sprom_get_fem_2ghz_pdet_range	= bhnd_compat_sprom_get_fem_2ghz_pdet_range,
	.sprom_get_fem_2ghz_tr_iso	= bhnd_compat_sprom_get_fem_2ghz_tr_iso,
	.sprom_get_fem_2ghz_antswlut	= bhnd_compat_sprom_get_fem_2ghz_antswlut,
	.sprom_get_fem_5ghz_extpa_gain	= bhnd_compat_sprom_get_fem_5ghz_extpa_gain,
	.sprom_get_fem_5ghz_pdet_range	= bhnd_compat_sprom_get_fem_5ghz_pdet_range,
	.sprom_get_fem_5ghz_antswlut	= bhnd_compat_sprom_get_fem_5ghz_antswlut,
	.sprom_get_txpid_2g_0		= bhnd_compat_sprom_get_txpid_2g_0,
	.sprom_get_txpid_2g_1		= bhnd_compat_sprom_get_txpid_2g_1,
	.sprom_get_txpid_5gl_0		= bhnd_compat_sprom_get_txpid_5gl_0,
	.sprom_get_txpid_5gl_1		= bhnd_compat_sprom_get_txpid_5gl_1,
	.sprom_get_txpid_5g_0		= bhnd_compat_sprom_get_txpid_5g_0,
	.sprom_get_txpid_5g_1		= bhnd_compat_sprom_get_txpid_5g_1,
	.sprom_get_txpid_5gh_0		= bhnd_compat_sprom_get_txpid_5gh_0,
	.sprom_get_txpid_5gh_1		= bhnd_compat_sprom_get_txpid_5gh_1,
	.sprom_get_stbcpo		= bhnd_compat_sprom_get_stbcpo,
	.sprom_get_cddpo		= bhnd_compat_sprom_get_cddpo,
	.powerup			= bhnd_compat_powerup,
	.powerdown			= bhnd_compat_powerdown,
	.read_2				= bhnd_compat_read_2,
	.write_2			= bhnd_compat_write_2,
	.read_4				= bhnd_compat_read_4,
	.write_4			= bhnd_compat_write_4,
	.dev_up				= bhnd_compat_dev_up,
	.dev_down			= bhnd_compat_dev_down,
	.dev_isup			= bhnd_compat_dev_isup,
	.pcicore_intr			= bhnd_compat_pcicore_intr,
	.dma_translation		= bhnd_compat_dma_translation,
	.read_multi_2			= bhnd_compat_read_multi_2,
	.read_multi_4			= bhnd_compat_read_multi_4,
	.write_multi_2			= bhnd_compat_write_multi_2,
	.write_multi_4			= bhnd_compat_write_multi_4,
	.barrier			= bhnd_compat_barrier,
	.cc_pmu_set_ldovolt		= bhnd_compat_cc_pmu_set_ldovolt,
	.cc_pmu_set_ldoparef		= bhnd_compat_cc_pmu_set_ldoparef,
	.gpio_set			= bhnd_compat_gpio_set,
	.gpio_get			= bhnd_compat_gpio_get,
	.fix_imcfglobug			= bhnd_compat_fix_imcfglobug,
	.sprom_get_core_power_info	= bhnd_compat_sprom_get_core_power_info,
	.sprom_get_mcs2gpo		= bhnd_compat_sprom_get_mcs2gpo,
	.sprom_get_mcs5glpo		= bhnd_compat_sprom_get_mcs5glpo,
	.sprom_get_mcs5gpo		= bhnd_compat_sprom_get_mcs5gpo,
	.sprom_get_mcs5ghpo		= bhnd_compat_sprom_get_mcs5ghpo,
	.pmu_spuravoid_pllupdate	= bhnd_compat_pmu_spuravoid_pllupdate,
	.cc_set32			= bhnd_compat_cc_set32,
	.cc_mask32			= bhnd_compat_cc_mask32,
	.cc_write32			= bhnd_compat_cc_write32,
};
