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

static uint16_t
bhnd_compat_get_vendor(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_device(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_get_revid(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_pci_vendor(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_pci_device(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_pci_subvendor(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_pci_subdevice(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_get_pci_revid(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_chipid(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_chiprev(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_get_chippkg(device_t dev)
{
	panic("unimplemented");
}

static enum siba_type
bhnd_compat_get_type(device_t dev)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_get_cc_pmufreq(device_t dev)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_get_cc_caps(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_get_cc_powerdelay(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_get_pcicore_revid(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rev(device_t dev)
{
	panic("unimplemented");
}

static uint8_t *
bhnd_compat_sprom_get_mac_80211bg(device_t dev)
{
	panic("unimplemented");
}

static uint8_t *
bhnd_compat_sprom_get_mac_80211a(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_brev(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_ccode(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_ant_a(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_ant_bg(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_pa0b0(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_pa0b1(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_pa0b2(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_gpio0(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_gpio1(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_gpio2(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_gpio3(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_maxpwr_bg(device_t dev)
{
	panic("unimplemented");
}

static void
bhnd_compat_sprom_set_maxpwr_bg(device_t dev, uint16_t t)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rxpo2g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rxpo5g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_tssi_bg(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_tri2g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_tri5gl(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_tri5g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_tri5gh(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rssisav2g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rssismc2g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rssismf2g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_bxa2g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rssisav5g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rssismc5g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_rssismf5g(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_bxa5g(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_cck2gpo(device_t dev)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_sprom_get_ofdm2gpo(device_t dev)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_sprom_get_ofdm5glpo(device_t dev)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_sprom_get_ofdm5gpo(device_t dev)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_sprom_get_ofdm5ghpo(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_bf_lo(device_t dev)
{
	panic("unimplemented");
}

static void
bhnd_compat_sprom_set_bf_lo(device_t dev, uint16_t t)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_bf_hi(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_bf2_lo(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_bf2_hi(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_2ghz_tssipos(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_2ghz_extpa_gain(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_2ghz_pdet_range(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_2ghz_tr_iso(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_2ghz_antswlut(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_5ghz_extpa_gain(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_5ghz_pdet_range(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_fem_5ghz_antswlut(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_2g_0(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_2g_1(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_5gl_0(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_5gl_1(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_5g_0(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_5g_1(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_5gh_0(device_t dev)
{
	panic("unimplemented");
}

static uint8_t
bhnd_compat_sprom_get_txpid_5gh_1(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_stbcpo(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_sprom_get_cddpo(device_t dev)
{
	panic("unimplemented");
}

static void
bhnd_compat_powerup(device_t dev, int dynamic)
{
	panic("unimplemented");
}

static int
bhnd_compat_powerdown(device_t dev)
{
	panic("unimplemented");
}

static uint16_t
bhnd_compat_read_2(device_t dev, uint16_t offset)
{
	panic("unimplemented");
}

static void
bhnd_compat_write_2(device_t dev, uint16_t offset, uint16_t value)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_read_4(device_t dev, uint16_t offset)
{
	panic("unimplemented");
}

static void
bhnd_compat_write_4(device_t dev, uint16_t offset, uint32_t value)
{
	panic("unimplemented");
}

static void
bhnd_compat_dev_up(device_t dev, uint32_t flags)
{
	panic("unimplemented");
}

static void
bhnd_compat_dev_down(device_t dev, uint32_t flags)
{
	panic("unimplemented");
}

static int
bhnd_compat_dev_isup(device_t dev)
{
	panic("unimplemented");
}

static void
bhnd_compat_pcicore_intr(device_t dev)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_dma_translation(device_t dev)
{
	panic("unimplemented");
}

static void
bhnd_compat_read_multi_2(device_t dev, void *buffer, size_t count, uint16_t offset)
{
	panic("unimplemented");
}

static void
bhnd_compat_read_multi_4(device_t dev, void *buffer, size_t count, uint16_t offset)
{
	panic("unimplemented");
}

static void
bhnd_compat_write_multi_2(device_t dev, const void *buffer, size_t count, uint16_t offset)
{
	panic("unimplemented");
}

static void
bhnd_compat_write_multi_4(device_t dev, const void *buffer, size_t count, uint16_t offset)
{
	panic("unimplemented");
}

static void
bhnd_compat_barrier(device_t dev, int flags)
{
	panic("unimplemented");
}

static void
bhnd_compat_cc_pmu_set_ldovolt(device_t dev, int id, uint32_t volt)
{
	panic("unimplemented");
}

static void
bhnd_compat_cc_pmu_set_ldoparef(device_t dev, uint8_t on)
{
	panic("unimplemented");
}

static void
bhnd_compat_gpio_set(device_t dev, uint32_t value)
{
	panic("unimplemented");
}

static uint32_t
bhnd_compat_gpio_get(device_t dev)
{
	panic("unimplemented");
}

static void
bhnd_compat_fix_imcfglobug(device_t dev)
{
	panic("unimplemented");
}

static int
bhnd_compat_sprom_get_core_power_info(device_t dev, int core,
    struct siba_sprom_core_pwr_info *c)
{
	panic("unimplemented");
}

static int
bhnd_compat_sprom_get_mcs2gpo(device_t dev, uint16_t *c)
{
	panic("unimplemented");
}

static int
bhnd_compat_sprom_get_mcs5glpo(device_t dev, uint16_t *c)
{
	panic("unimplemented");
}

static int
bhnd_compat_sprom_get_mcs5gpo(device_t dev, uint16_t *c)
{
	panic("unimplemented");
}

static int
bhnd_compat_sprom_get_mcs5ghpo(device_t dev, uint16_t *c)
{
	panic("unimplemented");
}

static void
bhnd_compat_pmu_spuravoid_pllupdate(device_t dev, int spur_avoid)
{
	panic("unimplemented");
}

static void
bhnd_compat_cc_set32(device_t dev, uint32_t reg, uint32_t val)
{
	panic("unimplemented");
}

static void
bhnd_compat_cc_mask32(device_t dev, uint32_t reg, uint32_t mask)
{
	panic("unimplemented");
}

static void
bhnd_compat_cc_write32(device_t dev, uint32_t reg, uint32_t val)
{
	panic("unimplemented");
}

const struct bwn_bus_ops bwn_bhnd_bus_ops = {
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
