/*
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2010 Broadcom Corporation
 * 
 * This file was initially derived from Broadcom's initial brcm80211 Linux
 * driver release of hndpmu.c, as contributed to the Linux staging repository.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>

#include <dev/bhnd/bhnd.h>

#include <dev/bhnd/bcma/bcma_dmp.h>

#include "bhnd_nvram_map.h"

#include "bhnd_pmureg.h"
#include "bhnd_pmuvar.h"

#ifdef BCMDBG
#define	PMU_MSG(args)	printf args
#define	PMU_ERROR(args)	do {	\
	panic args;		\
} while (0)
#else
#define	PMU_MSG(args)
#define	PMU_ERROR(args)	printf args
#endif

// XXX TODO: implement or import
static void
si_write_wrapperreg(struct bhnd_pmu_softc *sc, bus_size_t offset,
    uint32_t value)
{
	panic("unimplemented");
}

static inline bool
cst4330_chipmode_sdiod(void)
{
	panic("CST4330_CHIPMODE_SDIOD() unimplemented\n");
	return (false);
}
#define	CST4330_CHIPMODE_SDIOD(...)	cst4330_chipmode_sdiod()

/* PLL controls/clocks */
static void	bhnd_pmu1_pllinit0(struct bhnd_pmu_softc *sc, uint32_t xtal);
static uint32_t	bhnd_pmu1_cpuclk0(struct bhnd_pmu_softc *sc);
static uint32_t	bhnd_pmu1_alpclk0(struct bhnd_pmu_softc *sc);

/* PMU resources */
static bool	bhnd_pmu_res_depfltr_bb(struct bhnd_pmu_softc *sc);
static bool	bhnd_pmu_res_depfltr_ncb(struct bhnd_pmu_softc *sc);
static bool	bhnd_pmu_res_depfltr_paldo(struct bhnd_pmu_softc *sc);
static bool	bhnd_pmu_res_depfltr_npaldo(struct bhnd_pmu_softc *sc);
static uint32_t	bhnd_pmu_res_deps(struct bhnd_pmu_softc *sc, uint32_t rsrcs,
		    bool all);
static u_int	bhnd_pmu_res_uptime(struct bhnd_pmu_softc *sc, uint8_t rsrc);
static void	bhnd_pmu_res_masks(struct bhnd_pmu_softc *sc, uint32_t *pmin,
		    uint32_t *pmax);
static void	bhnd_pmu_spuravoid_pllupdate(struct bhnd_pmu_softc *sc,
		    uint8_t spuravoid);

static void	bhnd_pmu_set_4330_plldivs(struct bhnd_pmu_softc *sc);

/* FVCO frequency */
#define	FVCO_880	880000	/* 880MHz */
#define	FVCO_1760	1760000	/* 1760MHz */
#define	FVCO_1440	1440000	/* 1440MHz */
#define	FVCO_960	960000	/* 960MHz */

#define	BHND_PMU_READ_1(_sc, _reg)	bhnd_bus_read_1((_sc)->res, (_reg))
#define	BHND_PMU_READ_2(_sc, _reg)	bhnd_bus_read_2((_sc)->res, (_reg))
#define	BHND_PMU_READ_4(_sc, _reg)	bhnd_bus_read_4((_sc)->res, (_reg))
#define	BHND_PMU_WRITE_1(_sc, _reg, _val)	\
	bhnd_bus_write_1((_sc)->res, (_reg), (_val))
#define	BHND_PMU_WRITE_2(_sc, _reg, _val)	\
	bhnd_bus_write_2((_sc)->res, (_reg), (_val))
#define	BHND_PMU_WRITE_4(_sc, _reg, _val)	\
	bhnd_bus_write_4((_sc)->res, (_reg), (_val))

#define	BHND_PMU_AND_4(_sc, _reg, _val)		\
	BHND_PMU_WRITE_4((_sc), (_reg),		\
	    BHND_PMU_READ_4((_sc), (_reg)) & (_val))
#define	BHND_PMU_OR_4(_sc, _reg, _val)		\
	BHND_PMU_WRITE_4((_sc), (_reg),		\
	    BHND_PMU_READ_4((_sc), (_reg)) | (_val))

#define	BHND_PMU_IND_READ(_sc, _src, _reg)			\
	bhnd_pmu_ind_read((_sc), BHND_PMU_ ## _src ## _ADDR,	\
	    BHND_PMU_ ## _src ## _DATA, (_reg))
#define	BHND_PMU_IND_WRITE(_sc, _src, _reg, _val, _mask)	\
	bhnd_pmu_ind_write(sc, BHND_PMU_ ## _src ## _ADDR,	\
	    BHND_PMU_ ## _src ## _DATA, (_reg), (_val), (_mask))

#define	BHND_PMU_REV(_sc)			\
	((uint8_t)BHND_PMU_GET_BITS((_sc)->caps, BHND_PMU_CAP_REV))

#define	PMURES_BIT(_bit)			\
	(1 << (BHND_PMU_ ## _bit))

/* Perform an indirect register read */
static uint32_t
bhnd_pmu_ind_read(struct bhnd_pmu_softc *sc, bus_size_t addr, bus_size_t data,
    uint32_t reg)
{
	BHND_PMU_WRITE_4(sc, addr, reg);
	return (BHND_PMU_READ_4(sc, data));
}

/* Perform an indirect register write */
static void
bhnd_pmu_ind_write(struct bhnd_pmu_softc *sc, bus_size_t addr,
    bus_size_t data, uint32_t reg, uint32_t val, uint32_t mask)
{
	uint32_t rval;

	BHND_PMU_WRITE_4(sc, addr, reg);

	if (mask != UINT32_MAX) {
		rval = BHND_PMU_READ_4(sc, data);
		rval &= ~mask | (val & mask);
	} else {
		rval = val;
	}

	BHND_PMU_WRITE_4(sc, data, rval);
}

/* Read a chipcontrol register. */
static uint32_t
bhnd_pmu_cctrl_read(struct bhnd_pmu_softc *sc, uint32_t reg)
{
	return (BHND_PMU_IND_READ(sc, CHIPCTL, reg));
}

/* Write a chipcontrol register. */
static void
bhnd_pmu_cctrl_write(struct bhnd_pmu_softc *sc, uint32_t reg, uint32_t val,
    uint32_t mask)
{
	BHND_PMU_IND_WRITE(sc, CHIPCTL, reg, val, mask);
}

/* Read a regcontrol register */
static uint32_t
bhnd_pmu_regctrl_read(struct bhnd_pmu_softc *sc, uint32_t reg) 
{
	return (BHND_PMU_IND_READ(sc, REG_CONTROL, reg));
}

/* Write a regcontrol register */
static void
bhnd_pmu_regctrl_write(struct bhnd_pmu_softc *sc, uint32_t reg, uint32_t val,
    uint32_t mask)
{
	BHND_PMU_IND_WRITE(sc, REG_CONTROL, reg, val, mask);
}

/* Read a PLL control register */
static uint32_t
bhnd_pmu_pll_read(struct bhnd_pmu_softc *sc, uint32_t reg)
{
	return (BHND_PMU_IND_READ(sc, PLL_CONTROL, reg));
}

/* Write a pllcontrol reg */
static void
bhnd_pmu_pll_write(struct bhnd_pmu_softc *sc, uint32_t reg, uint32_t val,
    uint32_t mask)
{
	BHND_PMU_IND_WRITE(sc, PLL_CONTROL, reg, val, mask);
}

/* Flush any deferred PMU PLL control register writes */
static void
bhnd_pmu_pllupd(struct bhnd_pmu_softc *sc)
{
	BHND_PMU_OR_4(sc, BHND_PMU_CTRL, BHND_PMU_CTRL_PLL_PLLCTL_UPD);
}

/* Setup switcher voltage */
void
bhnd_pmu_set_switcher_voltage(struct bhnd_pmu_softc *sc, uint8_t bb_voltage,
    uint8_t rf_voltage)
{
	bhnd_pmu_regctrl_write(sc, 0x01, (bb_voltage & 0x1f) << 22, ~0);
	bhnd_pmu_regctrl_write(sc, 0x00, (rf_voltage & 0x1f) << 14, ~0);
}

void
bhnd_pmu_set_ldo_voltage(struct bhnd_pmu_softc *sc, uint8_t ldo,
    uint8_t voltage)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 regctrl;
	uint8_t				 shift;
	uint8_t				 mask;
	uint8_t				 addr;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4336:
		switch (ldo) {
		case SET_LDO_VOLTAGE_CLDO_PWM:
			addr = 4;
			shift = 1;
			mask = 0xf;
			break;
		case SET_LDO_VOLTAGE_CLDO_BURST:
			addr = 4;
			shift = 5;
			mask = 0xf;
			break;
		case SET_LDO_VOLTAGE_LNLDO1:
			addr = 4;
			shift = 17;
			mask = 0xf;
			break;
		default:
			panic("unknown BCM4336 LDO %hhu\n", ldo);
			return;
		}
		break;
	case BHND_CHIPID_BCM4330:
		switch (ldo) {
		case SET_LDO_VOLTAGE_CBUCK_PWM:
			addr = 3;
			shift = 0;
			mask = 0x1f;
			break;
		default:
			panic("unknown BCM4330 LDO %hhu\n", ldo);
			break;
		}
		break;
	default:
		panic("cannot set LDO voltage on unsupported chip %hu\n",
		    cid->chip_id);
		return;
	}

	regctrl = (voltage & mask) << shift;
	bhnd_pmu_regctrl_write(sc, addr, regctrl, mask << shift);
}

/* d11 slow to fast clock transition time in slow clock cycles */
#define	D11SCC_SLOW2FAST_TRANSITION	2

uint16_t
bhnd_pmu_fast_pwrup_delay(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 ilp;
	u_int				 delay;
	
	cid = bhnd_get_chipid(sc->dev);
	delay = BHND_PMU_MAX_TRANSITION_DLY;

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM43224:
	case BHND_CHIPID_BCM43225:
	case BHND_CHIPID_BCM43421:
	case BHND_CHIPID_BCM43235:
	case BHND_CHIPID_BCM43236:
	case BHND_CHIPID_BCM43238:
	case BHND_CHIPID_BCM4331:
	case BHND_CHIPID_BCM6362:
	case BHND_CHIPID_BCM4313:
		delay = 3700;
		break;
	case BHND_CHIPID_BCM4329:
		ilp = bhnd_pmu_ilp_clock(sc);
		delay =
			(bhnd_pmu_res_uptime(sc, BHND_PMU_RES4329_HT_AVAIL) +
			D11SCC_SLOW2FAST_TRANSITION) * ((1000000 + ilp -
							1) / ilp);
		delay = (11 * delay) / 10;
		break;
	case BHND_CHIPID_BCM4319:
		delay = 3700;
		break;
	case BHND_CHIPID_BCM4336:
		ilp = bhnd_pmu_ilp_clock(sc);
		delay =
			(bhnd_pmu_res_uptime(sc, BHND_PMU_RES4336_HT_AVAIL) +
			D11SCC_SLOW2FAST_TRANSITION) * ((1000000 + ilp -
							1) / ilp);
		delay = (11 * delay) / 10;
		break;
	case BHND_CHIPID_BCM4330:
		ilp = bhnd_pmu_ilp_clock(sc);
		delay =
			(bhnd_pmu_res_uptime(sc, BHND_PMU_RES4330_HT_AVAIL) +
			D11SCC_SLOW2FAST_TRANSITION) * ((1000000 + ilp -
							1) / ilp);
		delay = (11 * delay) / 10;
		break;
	default:
		break;
	}

	return ((uint16_t)delay);
}

uint32_t
bhnd_pmu_force_ilp(struct bhnd_pmu_softc *sc, bool force)
{
	uint32_t	orig;
	uint32_t	pctrl;

	pctrl = BHND_PMU_READ_4(sc, BHND_PMU_CTRL);
	orig = pctrl;

	if (force)
		pctrl &= ~(BHND_PMU_CTRL_HT_REQ_EN | BHND_PMU_CTRL_ALP_REQ_EN);
	else
		pctrl |= (BHND_PMU_CTRL_HT_REQ_EN | BHND_PMU_CTRL_ALP_REQ_EN);

	BHND_PMU_WRITE_4(sc, BHND_PMU_CTRL, pctrl);

	return (orig);
}

/* Setup resource up/down timers */
typedef struct {
	uint8_t		resnum;
	uint16_t	updown;
} pmu_res_updown_t;

typedef bool (*pmu_res_filter) (struct bhnd_pmu_softc *sc);

/* Change resource dependancies masks */
typedef struct {
	uint32_t	res_mask;	/* resources (chip specific) */
	int8_t		action;		/* action */
	uint32_t	depend_mask;	/* changes to the dependancies mask */
	pmu_res_filter	filter;		/* action is taken when filter is NULL or returns true */
} pmu_res_depend_t;

/* Resource dependancies mask change action */
#define	RES_DEPEND_SET		0	/* Override the dependancies mask */
#define	RES_DEPEND_ADD		1	/* Add to the  dependancies mask */
#define	RES_DEPEND_REMOVE	-1	/* Remove from the dependancies mask */

static const pmu_res_updown_t bcm4328a0_res_updown[] = {
	{
	BHND_PMU_RES4328_EXT_SWITCHER_PWM, 0x0101}, {
	BHND_PMU_RES4328_BB_SWITCHER_PWM, 0x1f01}, {
	BHND_PMU_RES4328_BB_SWITCHER_BURST, 0x010f}, {
	BHND_PMU_RES4328_BB_EXT_SWITCHER_BURST, 0x0101}, {
	BHND_PMU_RES4328_ILP_REQUEST, 0x0202}, {
	BHND_PMU_RES4328_RADIO_SWITCHER_PWM, 0x0f01}, {
	BHND_PMU_RES4328_RADIO_SWITCHER_BURST, 0x0f01}, {
	BHND_PMU_RES4328_ROM_SWITCH, 0x0101}, {
	BHND_PMU_RES4328_PA_REF_LDO, 0x0f01}, {
	BHND_PMU_RES4328_RADIO_LDO, 0x0f01}, {
	BHND_PMU_RES4328_AFE_LDO, 0x0f01}, {
	BHND_PMU_RES4328_PLL_LDO, 0x0f01}, {
	BHND_PMU_RES4328_BG_FILTBYP, 0x0101}, {
	BHND_PMU_RES4328_TX_FILTBYP, 0x0101}, {
	BHND_PMU_RES4328_RX_FILTBYP, 0x0101}, {
	BHND_PMU_RES4328_XTAL_PU, 0x0101}, {
	BHND_PMU_RES4328_XTAL_EN, 0xa001}, {
	BHND_PMU_RES4328_BB_PLL_FILTBYP, 0x0101}, {
	BHND_PMU_RES4328_RF_PLL_FILTBYP, 0x0101}, {
	BHND_PMU_RES4328_BB_PLL_PU, 0x0701}
};

static const pmu_res_depend_t bcm4328a0_res_depend[] = {
	/* Adjust ILP request resource not to force ext/BB switchers into burst mode */
	{
	PMURES_BIT(RES4328_ILP_REQUEST),
		    RES_DEPEND_SET,
		    PMURES_BIT(RES4328_EXT_SWITCHER_PWM) |
		    PMURES_BIT(RES4328_BB_SWITCHER_PWM), NULL}
};

static const pmu_res_updown_t bcm4325a0_res_updown[] = {
	{
	BHND_PMU_RES4325_XTAL_PU, 0x1501}
};

static const pmu_res_depend_t bcm4325a0_res_depend[] = {
	/* Adjust OTP PU resource dependencies - remove BB BURST */
	{
	PMURES_BIT(RES4325_OTP_PU),
		    RES_DEPEND_REMOVE,
		    PMURES_BIT(RES4325_BUCK_BOOST_BURST), NULL},
	/* Adjust ALP/HT Avail resource dependencies - bring up BB along if it is used. */
	{
	PMURES_BIT(RES4325_ALP_AVAIL) | PMURES_BIT(RES4325_HT_AVAIL),
		    RES_DEPEND_ADD,
		    PMURES_BIT(RES4325_BUCK_BOOST_BURST) |
		    PMURES_BIT(RES4325_BUCK_BOOST_PWM), bhnd_pmu_res_depfltr_bb},
	/* Adjust HT Avail resource dependencies - bring up RF switches along with HT. */
	{
	PMURES_BIT(RES4325_HT_AVAIL),
		    RES_DEPEND_ADD,
		    PMURES_BIT(RES4325_RX_PWRSW_PU) |
		    PMURES_BIT(RES4325_TX_PWRSW_PU) |
		    PMURES_BIT(RES4325_LOGEN_PWRSW_PU) |
		    PMURES_BIT(RES4325_AFE_PWRSW_PU), NULL},
	/* Adjust ALL resource dependencies - remove CBUCK dependancies if it is not used. */
	{
	PMURES_BIT(RES4325_ILP_REQUEST) |
		    PMURES_BIT(RES4325_ABUCK_BURST) |
		    PMURES_BIT(RES4325_ABUCK_PWM) |
		    PMURES_BIT(RES4325_LNLDO1_PU) |
		    PMURES_BIT(RES4325C1_LNLDO2_PU) |
		    PMURES_BIT(RES4325_XTAL_PU) |
		    PMURES_BIT(RES4325_ALP_AVAIL) |
		    PMURES_BIT(RES4325_RX_PWRSW_PU) |
		    PMURES_BIT(RES4325_TX_PWRSW_PU) |
		    PMURES_BIT(RES4325_RFPLL_PWRSW_PU) |
		    PMURES_BIT(RES4325_LOGEN_PWRSW_PU) |
		    PMURES_BIT(RES4325_AFE_PWRSW_PU) |
		    PMURES_BIT(RES4325_BBPLL_PWRSW_PU) |
		    PMURES_BIT(RES4325_HT_AVAIL), RES_DEPEND_REMOVE,
		    PMURES_BIT(RES4325B0_CBUCK_LPOM) |
		    PMURES_BIT(RES4325B0_CBUCK_BURST) |
		    PMURES_BIT(RES4325B0_CBUCK_PWM), bhnd_pmu_res_depfltr_ncb}
};

static const pmu_res_updown_t bcm4315a0_res_updown[] = {
	{
	BHND_PMU_RES4315_XTAL_PU, 0x2501}
};

static const pmu_res_depend_t bcm4315a0_res_depend[] = {
	/* Adjust OTP PU resource dependencies - not need PALDO unless write */
	{
	PMURES_BIT(RES4315_OTP_PU),
		    RES_DEPEND_REMOVE,
		    PMURES_BIT(RES4315_PALDO_PU), bhnd_pmu_res_depfltr_npaldo},
	/* Adjust ALP/HT Avail resource dependencies - bring up PALDO along if it is used. */
	{
	PMURES_BIT(RES4315_ALP_AVAIL) | PMURES_BIT(RES4315_HT_AVAIL),
		    RES_DEPEND_ADD,
		    PMURES_BIT(RES4315_PALDO_PU), bhnd_pmu_res_depfltr_paldo},
	/* Adjust HT Avail resource dependencies - bring up RF switches along with HT. */
	{
	PMURES_BIT(RES4315_HT_AVAIL),
		    RES_DEPEND_ADD,
		    PMURES_BIT(RES4315_RX_PWRSW_PU) |
		    PMURES_BIT(RES4315_TX_PWRSW_PU) |
		    PMURES_BIT(RES4315_LOGEN_PWRSW_PU) |
		    PMURES_BIT(RES4315_AFE_PWRSW_PU), NULL},
	/* Adjust ALL resource dependencies - remove CBUCK dependancies if it is not used. */
	{
	PMURES_BIT(RES4315_CLDO_PU) | PMURES_BIT(RES4315_ILP_REQUEST) |
		    PMURES_BIT(RES4315_LNLDO1_PU) |
		    PMURES_BIT(RES4315_OTP_PU) |
		    PMURES_BIT(RES4315_LNLDO2_PU) |
		    PMURES_BIT(RES4315_XTAL_PU) |
		    PMURES_BIT(RES4315_ALP_AVAIL) |
		    PMURES_BIT(RES4315_RX_PWRSW_PU) |
		    PMURES_BIT(RES4315_TX_PWRSW_PU) |
		    PMURES_BIT(RES4315_RFPLL_PWRSW_PU) |
		    PMURES_BIT(RES4315_LOGEN_PWRSW_PU) |
		    PMURES_BIT(RES4315_AFE_PWRSW_PU) |
		    PMURES_BIT(RES4315_BBPLL_PWRSW_PU) |
		    PMURES_BIT(RES4315_HT_AVAIL), RES_DEPEND_REMOVE,
		    PMURES_BIT(RES4315_CBUCK_LPOM) |
		    PMURES_BIT(RES4315_CBUCK_BURST) |
		    PMURES_BIT(RES4315_CBUCK_PWM), bhnd_pmu_res_depfltr_ncb}
};

/* 4329 specific. needs to come back this issue later */
static const pmu_res_updown_t bcm4329_res_updown[] = {
	{
	BHND_PMU_RES4329_XTAL_PU, 0x1501}
};

static const pmu_res_depend_t bcm4329_res_depend[] = {
	/* Adjust HT Avail resource dependencies */
	{
	PMURES_BIT(RES4329_HT_AVAIL),
		    RES_DEPEND_ADD,
		    PMURES_BIT(RES4329_CBUCK_LPOM) |
		    PMURES_BIT(RES4329_CBUCK_BURST) |
		    PMURES_BIT(RES4329_CBUCK_PWM) |
		    PMURES_BIT(RES4329_CLDO_PU) |
		    PMURES_BIT(RES4329_PALDO_PU) |
		    PMURES_BIT(RES4329_LNLDO1_PU) |
		    PMURES_BIT(RES4329_XTAL_PU) |
		    PMURES_BIT(RES4329_ALP_AVAIL) |
		    PMURES_BIT(RES4329_RX_PWRSW_PU) |
		    PMURES_BIT(RES4329_TX_PWRSW_PU) |
		    PMURES_BIT(RES4329_RFPLL_PWRSW_PU) |
		    PMURES_BIT(RES4329_LOGEN_PWRSW_PU) |
		    PMURES_BIT(RES4329_AFE_PWRSW_PU) |
		    PMURES_BIT(RES4329_BBPLL_PWRSW_PU), NULL}
};

static const pmu_res_updown_t bcm4319a0_res_updown[] = {
	{
	BHND_PMU_RES4319_XTAL_PU, 0x3f01}
};

static const pmu_res_depend_t bcm4319a0_res_depend[] = {
	/* Adjust OTP PU resource dependencies - not need PALDO unless write */
	{
	PMURES_BIT(RES4319_OTP_PU),
		    RES_DEPEND_REMOVE,
		    PMURES_BIT(RES4319_PALDO_PU), bhnd_pmu_res_depfltr_npaldo},
	    /* Adjust HT Avail resource dependencies - bring up PALDO along if it is used. */
	{
	PMURES_BIT(RES4319_HT_AVAIL),
		    RES_DEPEND_ADD,
		    PMURES_BIT(RES4319_PALDO_PU), bhnd_pmu_res_depfltr_paldo},
	    /* Adjust HT Avail resource dependencies - bring up RF switches along with HT. */
	{
	PMURES_BIT(RES4319_HT_AVAIL),
		    RES_DEPEND_ADD,
		    PMURES_BIT(RES4319_RX_PWRSW_PU) |
		    PMURES_BIT(RES4319_TX_PWRSW_PU) |
		    PMURES_BIT(RES4319_RFPLL_PWRSW_PU) |
		    PMURES_BIT(RES4319_LOGEN_PWRSW_PU) |
		    PMURES_BIT(RES4319_AFE_PWRSW_PU), NULL}
};

static const pmu_res_updown_t bcm4336a0_res_updown[] = {
	{
	BHND_PMU_RES4336_HT_AVAIL, 0x0D01}
};

static const pmu_res_depend_t bcm4336a0_res_depend[] = {
	/* Just a dummy entry for now */
	{
	PMURES_BIT(RES4336_RSVD), RES_DEPEND_ADD, 0, NULL}
};

static const pmu_res_updown_t bcm4330a0_res_updown[] = {
	{
	BHND_PMU_RES4330_HT_AVAIL, 0x0e02}
};

static const pmu_res_depend_t bcm4330a0_res_depend[] = {
	/* Just a dummy entry for now */
	{
	PMURES_BIT(RES4330_HT_AVAIL), RES_DEPEND_ADD, 0, NULL}
};

static int
bhnd_pmu_resfltr_board_info(struct bhnd_pmu_softc *sc,
    struct bhnd_board_info *board)
{
	int error;

	/* Fetch board info */
	if ((error = bhnd_read_board_info(sc->dev, board))) {
		device_printf(sc->dev, "error fetching board flags, PMU "
		    "configuration will likely be incorrect: %d\n", error);
		return (error);
	}

	return (0);
}

/* true if the power topology uses the buck boost to provide 3.3V to VDDIO_RF
 * and WLAN PA */
static bool
bhnd_pmu_res_depfltr_bb(struct bhnd_pmu_softc *sc)
{	
	struct bhnd_board_info	board;

	if (bhnd_pmu_resfltr_board_info(sc, &board) != 0)
		return (false);

	return (BHND_PMU_GET_FLAG(board.board_flags, BHND_BFL_BUCKBOOST));
}

/* true if the power topology doesn't use the cbuck. Key on chiprev also if
 * the chip is BCM4325. */
static bool
bhnd_pmu_res_depfltr_ncb(struct bhnd_pmu_softc *sc)
{
	struct bhnd_board_info	board;

	if (bhnd_pmu_resfltr_board_info(sc, &board) != 0)
		return (false);

	return (BHND_PMU_GET_FLAG(board.board_flags, BHND_BFL_NOCBUCK));
}

/* true if the power topology uses the PALDO */
static bool
bhnd_pmu_res_depfltr_paldo(struct bhnd_pmu_softc *sc)
{
	struct bhnd_board_info	board;

	if (bhnd_pmu_resfltr_board_info(sc, &board) != 0)
		return (false);

	return (BHND_PMU_GET_FLAG(board.board_flags, BHND_BFL_PALDO));
}

/* true if the power topology doesn't use the PALDO */
static bool
bhnd_pmu_res_depfltr_npaldo(struct bhnd_pmu_softc *sc)
{
	struct bhnd_board_info	board;

	if (bhnd_pmu_resfltr_board_info(sc, &board) != 0)
		return (false);

	return (!BHND_PMU_GET_FLAG(board.board_flags, BHND_BFL_PALDO));	
}

/* Determine min/max rsrc masks. Value 0 leaves hardware at default. */
static void
bhnd_pmu_res_masks(struct bhnd_pmu_softc *sc, uint32_t *pmin, uint32_t *pmax)
{
	const struct bhnd_chipid	*cid;	
	uint32_t			 max_mask, min_mask;
	uint32_t			 val;
	uint8_t				 rsrcs;
	int				 error;

	cid = bhnd_get_chipid(sc->dev);
	max_mask = 0;
	min_mask = 0;

	/* # resources */
	rsrcs = BHND_PMU_GET_BITS(sc->caps, BHND_PMU_CAP_RC);

	/* determine min/max rsrc masks */
	switch (cid->chip_id) {
	case BHND_CHIPID_BCM43224:
	case BHND_CHIPID_BCM43225:
	case BHND_CHIPID_BCM43421:
	case BHND_CHIPID_BCM43235:
	case BHND_CHIPID_BCM43236:
	case BHND_CHIPID_BCM43238:
	case BHND_CHIPID_BCM4331:
	case BHND_CHIPID_BCM6362:
		/* ??? */
		break;

	case BHND_CHIPID_BCM4329:
		/* 4329 spedific issue. Needs to come back this issue later */
		/* Down to save the power. */
		min_mask =
		    PMURES_BIT(RES4329_CBUCK_LPOM) |
		    PMURES_BIT(RES4329_CLDO_PU);
		/* Allow (but don't require) PLL to turn on */
		max_mask = 0x3ff63e;
		break;
	case BHND_CHIPID_BCM4319:
		/* We only need a few resources to be kept on all the time */
		min_mask = PMURES_BIT(RES4319_CBUCK_LPOM) |
		    PMURES_BIT(RES4319_CLDO_PU);

		/* Allow everything else to be turned on upon requests */
		max_mask = ~(~0 << rsrcs);
		break;
	case BHND_CHIPID_BCM4336:
		/* Down to save the power. */
		min_mask =
		    PMURES_BIT(RES4336_CBUCK_LPOM) | PMURES_BIT(RES4336_CLDO_PU)
		    | PMURES_BIT(RES4336_LDO3P3_PU) | PMURES_BIT(RES4336_OTP_PU)
		    | PMURES_BIT(RES4336_DIS_INT_RESET_PD);
		/* Allow (but don't require) PLL to turn on */
		max_mask = 0x1ffffff;
		break;

	case BHND_CHIPID_BCM4330:
		/* Down to save the power. */
		min_mask =
		    PMURES_BIT(RES4330_CBUCK_LPOM) | PMURES_BIT(RES4330_CLDO_PU)
		    | PMURES_BIT(RES4330_DIS_INT_RESET_PD) |
		    PMURES_BIT(RES4330_LDO3P3_PU) | PMURES_BIT(RES4330_OTP_PU);
		/* Allow (but don't require) PLL to turn on */
		max_mask = 0xfffffff;
		break;

	case BHND_CHIPID_BCM4313:
		min_mask = PMURES_BIT(RES4313_BB_PU_RSRC) |
		    PMURES_BIT(RES4313_XTAL_PU_RSRC) |
		    PMURES_BIT(RES4313_ALP_AVAIL_RSRC) |
		    PMURES_BIT(RES4313_BB_PLL_PWRSW_RSRC);
		max_mask = 0xffff;
		break;
	default:
		break;
	}

	/* Apply nvram override to min mask */
	error = bhnd_nvram_getvar(sc->dev, BHND_NVAR_RMIN, &val, sizeof(val));
	if (error == 0) {
		PMU_MSG(("Applying rmin=%#x to min_mask\n", val));
		min_mask = val;
	}

	/* Apply nvram override to max mask */
	error = bhnd_nvram_getvar(sc->dev, BHND_NVAR_RMAX, &val, sizeof(val));
	if (error == 0) {
		PMU_MSG(("Applying rmax=%#x to max_mask\n", val));
		min_mask = val;
	}

	if (pmin != NULL)
		*pmin = min_mask;

	if (pmax != NULL)
		*pmax = max_mask;
}

/* initialize PMU resources */
void bhnd_pmu_res_init(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	const pmu_res_updown_t		*pmu_res_updown_table;
	const pmu_res_depend_t		*pmu_res_depend_table;
	size_t				 pmu_res_updown_table_sz;
	size_t				 pmu_res_depend_table_sz;
	uint32_t			 max_mask, min_mask;
	uint8_t				 rsrcs;

	pmu_res_depend_table = NULL;
	pmu_res_depend_table_sz = 0;

	pmu_res_updown_table = NULL;
	pmu_res_updown_table_sz = 0;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4315:
		/* Optimize resources up/down timers */
		pmu_res_updown_table = bcm4315a0_res_updown;
		pmu_res_updown_table_sz = nitems(bcm4315a0_res_updown);

		/* Optimize resources dependencies */
		pmu_res_depend_table = bcm4315a0_res_depend;
		pmu_res_depend_table_sz = nitems(bcm4315a0_res_depend);
		break;

	case BHND_CHIPID_BCM4325:
		/* Optimize resources up/down timers */
		pmu_res_updown_table = bcm4325a0_res_updown;
		pmu_res_updown_table_sz = nitems(bcm4325a0_res_updown);

		/* Optimize resources dependencies */
		pmu_res_depend_table = bcm4325a0_res_depend;
		pmu_res_depend_table_sz = nitems(bcm4325a0_res_depend);
		break;

	case BHND_CHIPID_BCM4328:
		/* Optimize resources up/down timers */
		pmu_res_updown_table = bcm4328a0_res_updown;
		pmu_res_updown_table_sz = nitems(bcm4328a0_res_updown);

		/* Optimize resources dependencies */
		pmu_res_depend_table = bcm4328a0_res_depend;
		pmu_res_depend_table_sz = nitems(bcm4328a0_res_depend);
		break;

	case BHND_CHIPID_BCM4329:
		/* Optimize resources up/down timers */
		pmu_res_updown_table = bcm4329_res_updown;
		pmu_res_updown_table_sz = nitems(bcm4329_res_updown);

		/* Optimize resources dependencies */
		pmu_res_depend_table = bcm4329_res_depend;
		pmu_res_depend_table_sz = nitems(bcm4329_res_depend);
		break;

	case BHND_CHIPID_BCM4319:
		/* Optimize resources up/down timers */
		pmu_res_updown_table = bcm4319a0_res_updown;
		pmu_res_updown_table_sz = nitems(bcm4319a0_res_updown);
		
		/* Optimize resources dependancies masks */
		pmu_res_depend_table = bcm4319a0_res_depend;
		pmu_res_depend_table_sz = nitems(bcm4319a0_res_depend);
		break;

	case BHND_CHIPID_BCM4336:
		/* Optimize resources up/down timers */
		pmu_res_updown_table = bcm4336a0_res_updown;
		pmu_res_updown_table_sz = nitems(bcm4336a0_res_updown);

		/* Optimize resources dependancies masks */
		pmu_res_depend_table = bcm4336a0_res_depend;
		pmu_res_depend_table_sz = nitems(bcm4336a0_res_depend);
		break;

	case BHND_CHIPID_BCM4330:
		/* Optimize resources up/down timers */
		pmu_res_updown_table = bcm4330a0_res_updown;
		pmu_res_updown_table_sz = nitems(bcm4330a0_res_updown);

		/* Optimize resources dependancies masks */
		pmu_res_depend_table = bcm4330a0_res_depend;
		pmu_res_depend_table_sz = nitems(bcm4330a0_res_depend);
		break;
	default:
		break;
	}

	/* # resources */
	rsrcs = BHND_PMU_GET_BITS(sc->caps, BHND_PMU_CAP_RC);

	/* Program up/down timers */
	for (size_t i = 0; i < pmu_res_updown_table_sz; i++) {
		const pmu_res_updown_t	*updt;

		KASSERT(pmu_res_updown_table != NULL, ("no updown tables"));

		updt = &pmu_res_updown_table[pmu_res_updown_table_sz - i - 1];
	
		PMU_MSG(("Changing rsrc %d res_updn_timer to %#x\n",
			 updt->resnum, updt->updown));

		BHND_PMU_WRITE_4(sc, BHND_PMU_RES_TABLE_SEL, updt->resnum);
		BHND_PMU_WRITE_4(sc, BHND_PMU_RES_UPDN_TIMER, updt->updown);
	}

	/* Apply nvram overrides to up/down timers */
	for (uint8_t i = 0; i < rsrcs; i++) {
		char		name[6];
		uint32_t	val;

		snprintf(name, sizeof(name), "r%dt", i);
		if (bhnd_nvram_getvar(sc->dev, name, &val, sizeof(val)) != 0)
			continue;

		PMU_MSG(("Applying %s=%s to rsrc %d res_updn_timer\n", name,
			 val, i));

		BHND_PMU_WRITE_4(sc, BHND_PMU_RES_TABLE_SEL, i);
		BHND_PMU_WRITE_4(sc, BHND_PMU_RES_UPDN_TIMER, val);
	}

	/* Program resource dependencies table */
	for (size_t i = 0; i < pmu_res_depend_table_sz; i++) {
		const pmu_res_depend_t	*rdep;
		pmu_res_filter		 filter;
		uint32_t		 depend_mask;

		KASSERT(pmu_res_depend_table != NULL, ("no depend tables"));

		rdep = &pmu_res_depend_table[pmu_res_depend_table_sz - i - 1];
		filter = rdep->filter;

		if (filter != NULL && !filter(sc))
			continue;

		for (uint8_t i = 0; i < rsrcs; i++) {
			if ((rdep->res_mask & BHND_PMURES_BIT(i)) == 0)
				continue;

			BHND_PMU_WRITE_4(sc, BHND_PMU_RES_TABLE_SEL, i);
			depend_mask = BHND_PMU_READ_4(sc,
			    BHND_PMU_RES_DEP_MASK);
			switch (rdep->action) {
			case RES_DEPEND_SET:
				PMU_MSG(("Changing rsrc %hhu res_dep_mask to "
				    "%#x\n", i, table->depend_mask));
				depend_mask = rdep->depend_mask;
				break;

			case RES_DEPEND_ADD:
				PMU_MSG(("Adding %#x to rsrc %hhu "
				    "res_dep_mask\n", table->depend_mask, i));

				depend_mask |= rdep->depend_mask;
				break;

			case RES_DEPEND_REMOVE:
				PMU_MSG(("Removing %#x from rsrc %hhu "
				    "res_dep_mask\n", table->depend_mask, i));

				depend_mask &= ~(rdep->depend_mask);
				break;

			default:
				panic("unknown RES_DEPEND action: %d\n",
				    rdep->action);
				break;
			}
			
			
		}
	}

	/* Apply nvram overrides to dependancies masks */
	for (uint8_t i = 0; i < rsrcs; i++) {
		char		name[6];
		uint32_t	val;

		snprintf(name, sizeof(name), "r%dd", i);
		if (bhnd_nvram_getvar(sc->dev, name, &val, sizeof(val)) != 0)
			continue;
		PMU_MSG(("Applying %s=%s to rsrc %d res_dep_mask\n", name, val,
			 i));

		BHND_PMU_WRITE_4(sc, BHND_PMU_RES_TABLE_SEL, i);
		BHND_PMU_WRITE_4(sc, BHND_PMU_RES_DEP_MASK, val);
	}

	/* Determine min/max rsrc masks */
	bhnd_pmu_res_masks(sc, &min_mask, &max_mask);

	/* It is required to program max_mask first and then min_mask */

	/* Program max resource mask */
	if (max_mask != 0) {
		PMU_MSG(("Changing max_res_mask to 0x%x\n", max_mask));
		BHND_PMU_WRITE_4(sc, BHND_PMU_MAX_RES_MASK, max_mask);
	}

	/* Program min resource mask */

	if (min_mask != 0) {
		PMU_MSG(("Changing min_res_mask to 0x%x\n", min_mask));
		BHND_PMU_WRITE_4(sc, BHND_PMU_MIN_RES_MASK, min_mask);
	}

	/* Add some delay; allow resources to come up and settle. */
	DELAY(2000);
}

/* Requires missing implementation of PMU0 PLL initialization
 * for these devices */
#if defined(BHND_PMU0_BCM4328) || defined(BHND_PMU0_BCM5354)

/* setup pll and query clock speed */
typedef struct {
	uint16_t	freq;
	uint8_t		xf;
	uint8_t		wbint;
	uint32_t	wbfrac;
} pmu0_xtaltab0_t;

/* the following table is based on 880Mhz fvco */
static const pmu0_xtaltab0_t pmu0_xtaltab0[] = {
	{
	12000, 1, 73, 349525}, {
	13000, 2, 67, 725937}, {
	14400, 3, 61, 116508}, {
	15360, 4, 57, 305834}, {
	16200, 5, 54, 336579}, {
	16800, 6, 52, 399457}, {
	19200, 7, 45, 873813}, {
	19800, 8, 44, 466033}, {
	20000, 9, 44, 0}, {
	25000, 10, 70, 419430}, {
	26000, 11, 67, 725937}, {
	30000, 12, 58, 699050}, {
	38400, 13, 45, 873813}, {
	40000, 14, 45, 0}, {
	0, 0, 0, 0}
};

#define	PMU0_XTAL0_DEFAULT	8

#endif /* BHND_PMU0_BCM4328 || BHND_PMU0_BCM5354 */

/* setup pll and query clock speed */
typedef struct {
	uint16_t	fref;
	uint8_t		xf;
	uint8_t		p1div;
	uint8_t		p2div;
	uint8_t		ndiv_int;
	uint32_t	ndiv_frac;
} pmu1_xtaltab0_t;

static const pmu1_xtaltab0_t pmu1_xtaltab0_880_4329[] = {
	{
	12000, 1, 3, 22, 0x9, 0xFFFFEF}, {
	13000, 2, 1, 6, 0xb, 0x483483}, {
	14400, 3, 1, 10, 0xa, 0x1C71C7}, {
	15360, 4, 1, 5, 0xb, 0x755555}, {
	16200, 5, 1, 10, 0x5, 0x6E9E06}, {
	16800, 6, 1, 10, 0x5, 0x3Cf3Cf}, {
	19200, 7, 1, 4, 0xb, 0x755555}, {
	19800, 8, 1, 11, 0x4, 0xA57EB}, {
	20000, 9, 1, 11, 0x4, 0x0}, {
	24000, 10, 3, 11, 0xa, 0x0}, {
	25000, 11, 5, 16, 0xb, 0x0}, {
	26000, 12, 1, 1, 0x21, 0xD89D89}, {
	30000, 13, 3, 8, 0xb, 0x0}, {
	37400, 14, 3, 1, 0x46, 0x969696}, {
	38400, 15, 1, 1, 0x16, 0xEAAAAA}, {
	40000, 16, 1, 2, 0xb, 0}, {
	0, 0, 0, 0, 0, 0}
};

/* the following table is based on 880Mhz fvco */
static const pmu1_xtaltab0_t pmu1_xtaltab0_880[] = {
	{
	12000, 1, 3, 22, 0x9, 0xFFFFEF}, {
	13000, 2, 1, 6, 0xb, 0x483483}, {
	14400, 3, 1, 10, 0xa, 0x1C71C7}, {
	15360, 4, 1, 5, 0xb, 0x755555}, {
	16200, 5, 1, 10, 0x5, 0x6E9E06}, {
	16800, 6, 1, 10, 0x5, 0x3Cf3Cf}, {
	19200, 7, 1, 4, 0xb, 0x755555}, {
	19800, 8, 1, 11, 0x4, 0xA57EB}, {
	20000, 9, 1, 11, 0x4, 0x0}, {
	24000, 10, 3, 11, 0xa, 0x0}, {
	25000, 11, 5, 16, 0xb, 0x0}, {
	26000, 12, 1, 2, 0x10, 0xEC4EC4}, {
	30000, 13, 3, 8, 0xb, 0x0}, {
	33600, 14, 1, 2, 0xd, 0x186186}, {
	38400, 15, 1, 2, 0xb, 0x755555}, {
	40000, 16, 1, 2, 0xb, 0}, {
	0, 0, 0, 0, 0, 0}
};

#define	PMU1_XTALTAB0_880_12000K	0
#define	PMU1_XTALTAB0_880_13000K	1
#define	PMU1_XTALTAB0_880_14400K	2
#define	PMU1_XTALTAB0_880_15360K	3
#define	PMU1_XTALTAB0_880_16200K	4
#define	PMU1_XTALTAB0_880_16800K	5
#define	PMU1_XTALTAB0_880_19200K	6
#define	PMU1_XTALTAB0_880_19800K	7
#define	PMU1_XTALTAB0_880_20000K	8
#define	PMU1_XTALTAB0_880_24000K	9
#define	PMU1_XTALTAB0_880_25000K	10
#define	PMU1_XTALTAB0_880_26000K	11
#define	PMU1_XTALTAB0_880_30000K	12
#define	PMU1_XTALTAB0_880_37400K	13
#define	PMU1_XTALTAB0_880_38400K	14
#define	PMU1_XTALTAB0_880_40000K	15

/* the following table is based on 1760Mhz fvco */
static const pmu1_xtaltab0_t pmu1_xtaltab0_1760[] = {
	{
	12000, 1, 3, 44, 0x9, 0xFFFFEF}, {
	13000, 2, 1, 12, 0xb, 0x483483}, {
	14400, 3, 1, 20, 0xa, 0x1C71C7}, {
	15360, 4, 1, 10, 0xb, 0x755555}, {
	16200, 5, 1, 20, 0x5, 0x6E9E06}, {
	16800, 6, 1, 20, 0x5, 0x3Cf3Cf}, {
	19200, 7, 1, 18, 0x5, 0x17B425}, {
	19800, 8, 1, 22, 0x4, 0xA57EB}, {
	20000, 9, 1, 22, 0x4, 0x0}, {
	24000, 10, 3, 22, 0xa, 0x0}, {
	25000, 11, 5, 32, 0xb, 0x0}, {
	26000, 12, 1, 4, 0x10, 0xEC4EC4}, {
	30000, 13, 3, 16, 0xb, 0x0}, {
	38400, 14, 1, 10, 0x4, 0x955555}, {
	40000, 15, 1, 4, 0xb, 0}, {
	0, 0, 0, 0, 0, 0}
};

/* table index */
#define	PMU1_XTALTAB0_1760_12000K	0
#define	PMU1_XTALTAB0_1760_13000K	1
#define	PMU1_XTALTAB0_1760_14400K	2
#define	PMU1_XTALTAB0_1760_15360K	3
#define	PMU1_XTALTAB0_1760_16200K	4
#define	PMU1_XTALTAB0_1760_16800K	5
#define	PMU1_XTALTAB0_1760_19200K	6
#define	PMU1_XTALTAB0_1760_19800K	7
#define	PMU1_XTALTAB0_1760_20000K	8
#define	PMU1_XTALTAB0_1760_24000K	9
#define	PMU1_XTALTAB0_1760_25000K	10
#define	PMU1_XTALTAB0_1760_26000K	11
#define	PMU1_XTALTAB0_1760_30000K	12
#define	PMU1_XTALTAB0_1760_38400K	13
#define	PMU1_XTALTAB0_1760_40000K	14

/* the following table is based on 1440Mhz fvco */
static const pmu1_xtaltab0_t pmu1_xtaltab0_1440[] = {
	{
	12000, 1, 1, 1, 0x78, 0x0}, {
	13000, 2, 1, 1, 0x6E, 0xC4EC4E}, {
	14400, 3, 1, 1, 0x64, 0x0}, {
	15360, 4, 1, 1, 0x5D, 0xC00000}, {
	16200, 5, 1, 1, 0x58, 0xE38E38}, {
	16800, 6, 1, 1, 0x55, 0xB6DB6D}, {
	19200, 7, 1, 1, 0x4B, 0}, {
	19800, 8, 1, 1, 0x48, 0xBA2E8B}, {
	20000, 9, 1, 1, 0x48, 0x0}, {
	25000, 10, 1, 1, 0x39, 0x999999}, {
	26000, 11, 1, 1, 0x37, 0x627627}, {
	30000, 12, 1, 1, 0x30, 0x0}, {
	37400, 13, 2, 1, 0x4D, 0x15E76}, {
	38400, 13, 2, 1, 0x4B, 0x0}, {
	40000, 14, 2, 1, 0x48, 0x0}, {
	48000, 15, 2, 1, 0x3c, 0x0}, {
	0, 0, 0, 0, 0, 0}
};

/* table index */
#define	PMU1_XTALTAB0_1440_12000K	0
#define	PMU1_XTALTAB0_1440_13000K	1
#define	PMU1_XTALTAB0_1440_14400K	2
#define	PMU1_XTALTAB0_1440_15360K	3
#define	PMU1_XTALTAB0_1440_16200K	4
#define	PMU1_XTALTAB0_1440_16800K	5
#define	PMU1_XTALTAB0_1440_19200K	6
#define	PMU1_XTALTAB0_1440_19800K	7
#define	PMU1_XTALTAB0_1440_20000K	8
#define	PMU1_XTALTAB0_1440_25000K	9
#define	PMU1_XTALTAB0_1440_26000K	10
#define	PMU1_XTALTAB0_1440_30000K	11
#define	PMU1_XTALTAB0_1440_37400K	12
#define	PMU1_XTALTAB0_1440_38400K	13
#define	PMU1_XTALTAB0_1440_40000K	14
#define	PMU1_XTALTAB0_1440_48000K	15

#define	XTAL_FREQ_24000MHZ		24000
#define	XTAL_FREQ_30000MHZ		30000
#define	XTAL_FREQ_37400MHZ		37400
#define	XTAL_FREQ_48000MHZ		48000

static const pmu1_xtaltab0_t pmu1_xtaltab0_960[] = {
	{
	12000, 1, 1, 1, 0x50, 0x0}, {
	13000, 2, 1, 1, 0x49, 0xD89D89}, {
	14400, 3, 1, 1, 0x42, 0xAAAAAA}, {
	15360, 4, 1, 1, 0x3E, 0x800000}, {
	16200, 5, 1, 1, 0x39, 0x425ED0}, {
	16800, 6, 1, 1, 0x39, 0x249249}, {
	19200, 7, 1, 1, 0x32, 0x0}, {
	19800, 8, 1, 1, 0x30, 0x7C1F07}, {
	20000, 9, 1, 1, 0x30, 0x0}, {
	25000, 10, 1, 1, 0x26, 0x666666}, {
	26000, 11, 1, 1, 0x24, 0xEC4EC4}, {
	30000, 12, 1, 1, 0x20, 0x0}, {
	37400, 13, 2, 1, 0x33, 0x563EF9}, {
	38400, 14, 2, 1, 0x32, 0x0}, {
	40000, 15, 2, 1, 0x30, 0x0}, {
	48000, 16, 2, 1, 0x28, 0x0}, {
	0, 0, 0, 0, 0, 0}
};

/* table index */
#define	PMU1_XTALTAB0_960_12000K	0
#define	PMU1_XTALTAB0_960_13000K	1
#define	PMU1_XTALTAB0_960_14400K	2
#define	PMU1_XTALTAB0_960_15360K	3
#define	PMU1_XTALTAB0_960_16200K	4
#define	PMU1_XTALTAB0_960_16800K	5
#define	PMU1_XTALTAB0_960_19200K	6
#define	PMU1_XTALTAB0_960_19800K	7
#define	PMU1_XTALTAB0_960_20000K	8
#define	PMU1_XTALTAB0_960_25000K	9
#define	PMU1_XTALTAB0_960_26000K	10
#define	PMU1_XTALTAB0_960_30000K	11
#define	PMU1_XTALTAB0_960_37400K	12
#define	PMU1_XTALTAB0_960_38400K	13
#define	PMU1_XTALTAB0_960_40000K	14
#define	PMU1_XTALTAB0_960_48000K	15

/* select xtal table for each chip */
static const pmu1_xtaltab0_t *
bhnd_pmu1_xtaltab0(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4315:
		return (pmu1_xtaltab0_1760);
	case BHND_CHIPID_BCM4319:
		return (pmu1_xtaltab0_1440);
	case BHND_CHIPID_BCM4325:
		return (pmu1_xtaltab0_880);
	case BHND_CHIPID_BCM4329:
		return (pmu1_xtaltab0_880_4329);
	case BHND_CHIPID_BCM4336:
		return (pmu1_xtaltab0_960);
	case BHND_CHIPID_BCM4330:
		if (CST4330_CHIPMODE_SDIOD(sih->chipst))
			return (pmu1_xtaltab0_960);
		else
			return (pmu1_xtaltab0_1440);
	default:
		PMU_MSG(("bhnd_pmu1_xtaltab0: Unknown chipid %#hx\n",
		    cid->chip_id));
		return (NULL);
	}
}

/* select default xtal frequency for each chip */
static const pmu1_xtaltab0_t *
bhnd_pmu1_xtaldef0(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4315:
		/* Default to 26000Khz */
		return (&pmu1_xtaltab0_1760[PMU1_XTALTAB0_1760_26000K]);
	case BHND_CHIPID_BCM4319:
		/* Default to 30000Khz */
		return (&pmu1_xtaltab0_1440[PMU1_XTALTAB0_1440_30000K]);
	case BHND_CHIPID_BCM4325:
		/* Default to 26000Khz */
		return (&pmu1_xtaltab0_880[PMU1_XTALTAB0_880_26000K]);
	case BHND_CHIPID_BCM4329:
		/* Default to 38400Khz */
		return (&pmu1_xtaltab0_880_4329[PMU1_XTALTAB0_880_38400K]);
	case BHND_CHIPID_BCM4336:
		/* Default to 26000Khz */
		return (&pmu1_xtaltab0_960[PMU1_XTALTAB0_960_26000K]);
	case BHND_CHIPID_BCM4330:
		/* Default to 37400Khz */
		if (CST4330_CHIPMODE_SDIOD(sih->chipst))
			return (&pmu1_xtaltab0_960[PMU1_XTALTAB0_960_37400K]);
		else
			return (&pmu1_xtaltab0_1440[PMU1_XTALTAB0_1440_37400K]);
	default:
		PMU_MSG(("bhnd_pmu1_xtaldef0: Unknown chipid %#hx\n",
		    cid->chip_id));
		return (NULL);
	}
}

/* select default pll fvco for each chip */
static uint32_t
bhnd_pmu1_pllfvco0(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4329:
		return (FVCO_880);
	case BHND_CHIPID_BCM4319:
		return (FVCO_1440);
	case BHND_CHIPID_BCM4336:
		return (FVCO_960);
	case BHND_CHIPID_BCM4330:
		if (CST4330_CHIPMODE_SDIOD(sih->chipst))
			return (FVCO_960);
		else
			return (FVCO_1440);
	default:
		PMU_MSG(("bhnd_pmu1_pllfvco0: Unknown chipid %#hx\n",
		    cid->chip_id));
		return (0);
	}
}

/* query alp/xtal clock frequency */
static uint32_t
bhnd_pmu1_alpclk0(struct bhnd_pmu_softc *sc)
{
	const pmu1_xtaltab0_t	*xt;
	uint32_t		 xf;

	/* Find the frequency in the table */
	xf = BHND_PMU_READ_4(sc, BHND_PMU_CTRL);
	xf = BHND_PMU_GET_BITS(xf, BHND_PMU_CTRL_XTALFREQ);

	for (xt = bhnd_pmu1_xtaltab0(sc); xt != NULL && xt->fref != 0; xt++) {
		if (xt->xf == xf)
			break;
	}

	/* Could not find it so assign a default value */
	if (xt == NULL || xt->fref == 0)
		xt = bhnd_pmu1_xtaldef0(sc);

	if (xt == NULL || xt->fref == 0) {
		device_printf(sc->dev,
		    "no matching ALP/XTAL frequency found\n");
		return (0);
	}

	return (xt->fref * 1000);
}

/* Set up PLL registers in the PMU as per the crystal speed.
 * XtalFreq field in pmucontrol register being 0 indicates the PLL
 * is not programmed and the h/w default is assumed to work, in which
 * case the xtal frequency is unknown to the s/w so we need to call
 * bhnd_pmu1_xtaldef0() wherever it is needed to return a default value.
 */
static void
bhnd_pmu1_pllinit0(struct bhnd_pmu_softc *sc, uint32_t xtal)
{
	const struct bhnd_chipid	*cid;
	const pmu1_xtaltab0_t		*xt;
	uint32_t			 buf_strength;
	uint32_t			 plladdr, plldata;
	uint32_t			 pmuctrl;
	uint8_t				 ndiv_mode;
	

	cid = bhnd_get_chipid(sc->dev);

	buf_strength = 0;
	ndiv_mode = 1;

	/* Use h/w default PLL config */
	if (xtal == 0) {
		PMU_MSG(("Unspecified xtal frequency, skipping PLL "
		    "configuration\n"));
		return;
	}

	/* Find the frequency in the table */
	for (xt = bhnd_pmu1_xtaltab0(sc); xt != NULL && xt->fref != 0; xt++) {
		if (xt->fref == xtal)
			break;
	}

	/* Check current PLL state, bail out if it has been programmed or
	 * we don't know how to program it.
	 */
	if (xt == NULL || xt->fref == 0) {
		device_printf(sc->dev, "Unsupported XTAL frequency %d.%dMHz, "
		    "skipping PLL configuration\n", xtal / 1000, xtal % 1000);
		return;
	}

	/* For 4319 bootloader already programs the PLL but bootloader does not
	 * program the PLL4 and PLL5. So Skip this check for 4319. */
	pmuctrl = BHND_PMU_READ_4(sc, BHND_PMU_CTRL);
	if (BHND_PMU_GET_BITS(pmuctrl, BHND_PMU_CTRL_XTALFREQ) == xt->xf &&
	    cid->chip_id != BHND_CHIPID_BCM4319 &&
	    cid->chip_id != BHND_CHIPID_BCM4330)
	{   
		PMU_MSG(("PLL already programmed for %d.%dMHz\n",
		    xt->fref / 1000, xt->fref % 1000));
		return;
	}

	PMU_MSG(("XTAL %d.%dMHz (%d)\n", xtal / 1000, xtal % 1000, xt->xf));
	PMU_MSG(("Programming PLL for %d.%dMHz\n", xt->fref / 1000,
		 xt->fref % 1000));

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4329:
		/* Change the BBPLL drive strength to 8 for all channels */
		buf_strength = 0x888888;

		BHND_PMU_AND_4(sc, BHND_PMU_MIN_RES_MASK,
			~(PMURES_BIT(RES4329_BBPLL_PWRSW_PU) |
			  PMURES_BIT(RES4329_HT_AVAIL)));
		BHND_PMU_AND_4(sc, BHND_PMU_MAX_RES_MASK,
			~(PMURES_BIT(RES4329_BBPLL_PWRSW_PU) |
			  PMURES_BIT(RES4329_HT_AVAIL)));

		// XXX: need access to per-core clk_ctl_st register, or
		// bhnd-level API for HT availability.
#ifdef notyet
		SPINWAIT(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL,
			 PMU_MAX_TRANSITION_DLY);

		ASSERT(!(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL));
#endif

		/* Initialize PLL4 */
		plladdr = BHND_PMU1_PLL0_PLLCTL4;
		if (xt->fref == 38400)
			plldata = 0x200024C0;
		else if (xt->fref == 37400)
			plldata = 0x20004500;
		else if (xt->fref == 26000)
			plldata = 0x200024C0;
		else
			plldata = 0x200005C0;	/* Chip Dflt Settings */

		bhnd_pmu_pll_write(sc, plladdr, plldata, ~0);

		/* Initialize PLL5 */
		plladdr = BHND_PMU1_PLL0_PLLCTL5;

		plldata = bhnd_pmu_pll_read(sc, plladdr);
		plldata &= BHND_PMU1_PLL0_PC5_CLK_DRV_MASK;

		if (xt->fref == 38400 ||
		    xt->fref == 37400 || 
		    xt->fref == 26000) {
			plldata |= 0x15;
		} else {
			plldata |= 0x25;	/* Chip Dflt Settings */
		}

		bhnd_pmu_pll_write(sc, plladdr, plldata, ~0);
		break;

	case BHND_CHIPID_BCM4319:
		/* Change the BBPLL drive strength to 2 for all channels */
		buf_strength = 0x222222;

		/* Make sure the PLL is off */
		/* WAR65104: Disable the HT_AVAIL resource first and then
		 * after a delay (more than downtime for HT_AVAIL) remove the
		 * BBPLL resource; backplane clock moves to ALP from HT.
		 */
		BHND_PMU_AND_4(sc, BHND_PMU_MIN_RES_MASK,
			~(PMURES_BIT(RES4319_HT_AVAIL)));
		BHND_PMU_AND_4(sc, BHND_PMU_MAX_RES_MASK,
			~(PMURES_BIT(RES4319_HT_AVAIL)));

		DELAY(100);
		BHND_PMU_AND_4(sc, BHND_PMU_MIN_RES_MASK,
			~(PMURES_BIT(RES4319_BBPLL_PWRSW_PU)));
		BHND_PMU_AND_4(sc, BHND_PMU_MAX_RES_MASK,
			~(PMURES_BIT(RES4319_BBPLL_PWRSW_PU)));

		DELAY(100);

		// XXX: need access to per-core clk_ctl_st register, or
		// bhnd-level API for HT availability.
#ifdef notyet
		SPINWAIT(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL,
			 PMU_MAX_TRANSITION_DLY);
		ASSERT(!(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL));
#endif

		plldata = 0x200005c0;
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4, plldata, ~0);
		break;

	case BHND_CHIPID_BCM4336:
		BHND_PMU_AND_4(sc, BHND_PMU_MIN_RES_MASK,
			~(PMURES_BIT(RES4336_HT_AVAIL) |
			  PMURES_BIT(RES4336_MACPHY_CLKAVAIL)));
		BHND_PMU_AND_4(sc, BHND_PMU_MAX_RES_MASK,
			~(PMURES_BIT(RES4336_HT_AVAIL) |
			  PMURES_BIT(RES4336_MACPHY_CLKAVAIL)));
		DELAY(100);
		// XXX: need access to per-core clk_ctl_st register, or
		// bhnd-level API for HT availability.
#ifdef notyet
		SPINWAIT(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL,
			 PMU_MAX_TRANSITION_DLY);
		ASSERT(!(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL));
#endif
		break;

	case BHND_CHIPID_BCM4330:
		BHND_PMU_AND_4(sc, BHND_PMU_MIN_RES_MASK,
			~(PMURES_BIT(RES4330_HT_AVAIL) |
			  PMURES_BIT(RES4330_MACPHY_CLKAVAIL)));
		BHND_PMU_AND_4(sc, BHND_PMU_MAX_RES_MASK,
			~(PMURES_BIT(RES4330_HT_AVAIL) |
			  PMURES_BIT(RES4330_MACPHY_CLKAVAIL)));
		DELAY(100);
		// XXX: need access to per-core clk_ctl_st register, or
		// bhnd-level API for HT availability.
#ifdef notyet
		SPINWAIT(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL,
			 PMU_MAX_TRANSITION_DLY);
		ASSERT(!(R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL));
#endif
		break;

	default:
		panic("unsupported chipid %#hx\n", cid->chip_id);
	}

	PMU_MSG(("Done masking\n"));

	/* Write p1div and p2div to pllcontrol[0] */	
	bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
	    BHND_PMU_SET_BITS(xt->p1div, BHND_PMU1_PLL0_PC0_P1DIV) |
	    BHND_PMU_SET_BITS(xt->p2div, BHND_PMU1_PLL0_PC0_P2DIV),
	    BHND_PMU1_PLL0_PC0_P1DIV_MASK |
	    BHND_PMU1_PLL0_PC0_P2DIV_MASK);

	if (cid->chip_id == BHND_CHIPID_BCM4330)
		bhnd_pmu_set_4330_plldivs(sc);

	if (cid->chip_id == BHND_CHIPID_BCM4329 && cid->chip_rev == 0) {
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1,
		    BHND_PMU_DOT11MAC_880MHZ_CLK_DIVISOR_VAL,
		    BHND_PMU_DOT11MAC_880MHZ_CLK_DIVISOR_MASK);
	}

	/* Write ndiv_int and ndiv_mode to pllcontrol[2] */
	if (cid->chip_id == BHND_CHIPID_BCM4319 ||
	    cid->chip_id == BHND_CHIPID_BCM4336 ||
	    cid->chip_id == BHND_CHIPID_BCM4330)
		ndiv_mode = BHND_PMU1_PLL0_PC2_NDIV_MODE_MFB;
	else
		ndiv_mode = BHND_PMU1_PLL0_PC2_NDIV_MODE_MASH;

	bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
	    BHND_PMU_SET_BITS(xt->ndiv_int, BHND_PMU1_PLL0_PC2_NDIV_INT) |
	    BHND_PMU_SET_BITS(ndiv_mode, BHND_PMU1_PLL0_PC2_NDIV_MODE),
	    BHND_PMU1_PLL0_PC2_NDIV_INT_MASK |
	    BHND_PMU1_PLL0_PC2_NDIV_MODE_MASK);

	/* Write ndiv_frac to pllcontrol[3] */
	bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3,
	    BHND_PMU_SET_BITS(xt->ndiv_frac, BHND_PMU1_PLL0_PC3_NDIV_FRAC),
	    BHND_PMU1_PLL0_PC3_NDIV_FRAC_MASK);

	/* Write clock driving strength to pllcontrol[5] */
	if (buf_strength) {
		PMU_MSG(("Adjusting PLL buffer drive strength: %x\n",
			 buf_strength));

		plldata = BHND_PMU_SET_BITS(buf_strength,
		    BHND_PMU1_PLL0_PC5_CLK_DRV);

		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5, plldata,
		    BHND_PMU1_PLL0_PC5_CLK_DRV_MASK);
	}

	PMU_MSG(("Done pll\n"));

	/* to operate the 4319 usb in 24MHz/48MHz; chipcontrol[2][84:83] needs
	 * to be updated.
	 */
	if (cid->chip_id == BHND_CHIPID_BCM4319 &&
	    xt->fref != XTAL_FREQ_30000MHZ)
	{
		uint32_t pll_sel;
		
		switch (xt->fref) {
		case XTAL_FREQ_24000MHZ:
			pll_sel = BHND_PMU_CCTL_4319USB_24MHZ_PLL_SEL;
			break;
		case XTAL_FREQ_48000MHZ:
			pll_sel = BHND_PMU_CCTL_4319USB_48MHZ_PLL_SEL;
			break;
		default:
			panic("unsupported 4319USB XTAL frequency: %hu\n",
			    xt->fref);
		}

		bhnd_pmu_cctrl_write(sc, BHND_PMU1_PLL0_CHIPCTL2,
		    BHND_PMU_SET_BITS(pll_sel, BHND_PMU_CCTL_4319USB_XTAL_SEL),
		    BHND_PMU_CCTL_4319USB_XTAL_SEL_MASK);
	}

	/* Flush deferred pll control registers writes */
	if (BHND_PMU_REV(sc) >= 2)
		bhnd_pmu_pllupd(sc);

	/* Write XtalFreq. Set the divisor also. */
	pmuctrl = BHND_PMU_READ_4(sc, BHND_PMU_CTRL);
	pmuctrl = ~(BHND_PMU_CTRL_ILP_DIV_MASK | BHND_PMU_CTRL_XTALFREQ_MASK);
	pmuctrl |= BHND_PMU_SET_BITS(((xt->fref + 127) / 128) - 1,
	    BHND_PMU_CTRL_ILP_DIV);
	pmuctrl |= BHND_PMU_SET_BITS(xt->xf, BHND_PMU_CTRL_XTALFREQ);

	if (cid->chip_id == BHND_CHIPID_BCM4329 && cid->chip_rev == 0) {
		/* clear the htstretch before clearing HTReqEn */
		BHND_PMU_AND_4(sc, BHND_PMU_CLKSTRETCH, ~BHND_PMU_CLKSTRETCH);
		pmuctrl &= ~BHND_PMU_CTRL_HT_REQ_EN;
	}

	BHND_PMU_WRITE_4(sc, BHND_PMU_CTRL, pmuctrl);
}

/* query the CPU clock frequency */
static uint32_t
bhnd_pmu1_cpuclk0(struct bhnd_pmu_softc *sc)
{
	uint32_t tmp, m1div;
#ifdef BCMDBG
	uint32_t ndiv_int, ndiv_frac, p2div, p1div, fvco;
	uint32_t fref;
#endif
	uint32_t FVCO = bhnd_pmu1_pllfvco0(sc);

	/* Read m1div from pllcontrol[1] */
	tmp = bhnd_pmu_pll_read(sc, BHND_PMU1_PLL0_PLLCTL1);
	m1div = BHND_PMU_GET_BITS(tmp, BHND_PMU1_PLL0_PC1_M1DIV);

#ifdef BCMDBG
	/* Read p2div/p1div from pllcontrol[0] */
	tmp = bhnd_pmu_pll_read(sc, BHND_PMU1_PLL0_PLLCTL0);
	p2div = BHND_PMU_GET_BITS(tmp, BHND_PMU1_PLL0_PC0_P2DIV);
	p1div = BHND_PMU_GET_BITS(tmp, BHND_PMU1_PLL0_PC0_P1DIV);

	/* Calculate fvco based on xtal freq and ndiv and pdiv */
	tmp = bhnd_pmu_pll_read(sc, BHND_PMU1_PLL0_PLLCTL2);
	ndiv_int = BHND_PMU_GET_BITS(tmp, BHND_PMU1_PLL0_PC2_NDIV_INT);

	tmp = bhnd_pmu_pll_read(sc, BHND_PMU1_PLL0_PLLCTL3);
	ndiv_frac = BHND_PMU_GET_BITS(tmp, BHND_PMU1_PLL0_PC3_NDIV_FRAC);

	fref = bhnd_pmu1_alpclk0(sc) / 1000;

	fvco = (fref * ndiv_int) << 8;
	fvco += (fref * (ndiv_frac >> 12)) >> 4;
	fvco += (fref * (ndiv_frac & 0xfff)) >> 12;
	fvco >>= 8;
	fvco *= p2div;
	fvco /= p1div;
	fvco /= 1000;
	fvco *= 1000;

	PMU_MSG(("bhnd_pmu1_cpuclk0: ndiv_int %u ndiv_frac %u p2div %u "
	    "p1div %u fvco %u\n", ndiv_int, ndiv_frac, p2div, p1div, fvco));

	FVCO = fvco;
#endif				/* BCMDBG */

	/* Return ARM/SB clock */
	return (FVCO / m1div * 1000);
}

/* initialize PLL */
void 
bhnd_pmu_pll_init(struct bhnd_pmu_softc *sc, u_int xtalfreq)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4329:
		if (xtalfreq == 0)
			xtalfreq = 38400;
		bhnd_pmu1_pllinit0(sc, xtalfreq);
		break;
	case BHND_CHIPID_BCM4313:
	case BHND_CHIPID_BCM43224:
	case BHND_CHIPID_BCM43225:
	case BHND_CHIPID_BCM43421:
	case BHND_CHIPID_BCM43235:
	case BHND_CHIPID_BCM43236:
	case BHND_CHIPID_BCM43238:
	case BHND_CHIPID_BCM4331:
	case BHND_CHIPID_BCM6362:
		/* ??? */
		break;
	case BHND_CHIPID_BCM4319:
	case BHND_CHIPID_BCM4336:
	case BHND_CHIPID_BCM4330:
		bhnd_pmu1_pllinit0(sc, xtalfreq);
		break;
	default:
		PMU_MSG(("No PLL init done for chip %#hx rev %d pmurev %d\n",
		    cid->chip_id, cid->chip_rev, BHND_PMU_REV(sc)));
		break;
	}
}

/* query alp/xtal clock frequency */
uint32_t
bhnd_pmu_alp_clock(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 clock;

	clock = BHND_PMU_ALP_CLOCK;
	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM43224:
	case BHND_CHIPID_BCM43225:
	case BHND_CHIPID_BCM43421:
	case BHND_CHIPID_BCM43235:
	case BHND_CHIPID_BCM43236:
	case BHND_CHIPID_BCM43238:
	case BHND_CHIPID_BCM4331:
	case BHND_CHIPID_BCM6362:
	case BHND_CHIPID_BCM4716:
	case BHND_CHIPID_BCM4748:
	case BHND_CHIPID_BCM47162:
	case BHND_CHIPID_BCM4313:
	case BHND_CHIPID_BCM5357:
		/* always 20Mhz */
		clock = 20000 * 1000;
		break;
	case BHND_CHIPID_BCM4329:
	case BHND_CHIPID_BCM4319:
	case BHND_CHIPID_BCM4336:
	case BHND_CHIPID_BCM4330:
		clock = bhnd_pmu1_alpclk0(sc);
		break;
	case BHND_CHIPID_BCM5356:
		/* always 25Mhz */
		clock = 25000 * 1000;
		break;
	default:
		PMU_MSG(("No ALP clock specified "
			 "for chip %s rev %d pmurev %d, using default %d Hz\n",
			 bcm_chipname(sih->chip, chn, 8), sih->chiprev,
			 sih->pmurev, clock));
		break;
	}

	return (clock);
}

/* Find the output of the "m" pll divider given pll controls that start with
 * pllreg "pll0" i.e. 12 for main 6 for phy, 0 for misc.
 */
static uint32_t
bhnd_pmu5_clock(struct bhnd_pmu_softc *sc, u_int pll0, u_int m)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 div;
	uint32_t			 fc;
	uint32_t			 ndiv;
	uint32_t			 p1, p2;
	uint32_t			 tmp;

	cid = bhnd_get_chipid(sc->dev);

	if ((pll0 & 3) || (pll0 > BHND_PMU4716_MAINPLL_PLL0)) {
		PMU_ERROR(("%s: Bad pll0: %d\n", __func__, pll0));
		return 0;
	}

	/* Strictly there is an m5 divider, but I'm not sure we use it */
	if ((m == 0) || (m > 4)) {
		PMU_ERROR(("%s: Bad m divider: %d\n", __func__, m));
		return 0;
	}

	if (cid->chip_id == BHND_CHIPID_BCM5357) {
		// XXX TODO - need access to chipc chipstatus
#ifdef notyet
		/* Detect failure in clock setting */
		if ((R_REG(&cc->chipstatus) & 0x40000) != 0)
			return 133 * 1000000;
#endif
	}


	/* Fetch p1 and p2 */
	BHND_PMU_WRITE_4(sc, BHND_PMU_PLL_CONTROL_ADDR,
	    pll0 + BHND_PMU5_PLL_P1P2_OFF);
	BHND_PMU_READ_4(sc, BHND_PMU_PLL_CONTROL_ADDR);

	tmp = BHND_PMU_READ_4(sc, BHND_PMU_PLL_CONTROL_DATA);
	p1 = BHND_PMU_GET_BITS(tmp, BHND_PMU5_PLL_P1);
	p2 = BHND_PMU_GET_BITS(tmp, BHND_PMU5_PLL_P2);

	/* Fetch div */
	BHND_PMU_WRITE_4(sc, BHND_PMU_PLL_CONTROL_ADDR,
	    pll0 + BHND_PMU5_PLL_M14_OFF);
	BHND_PMU_READ_4(sc, BHND_PMU_PLL_CONTROL_ADDR);

	tmp = BHND_PMU_READ_4(sc, BHND_PMU_PLL_CONTROL_DATA);
	div = (tmp >> ((m - 1) * BHND_PMU5_PLL_MDIV_WIDTH));
	div &= BHND_PMU5_PLL_MDIV_MASK;

	/* Fetch ndiv */
	BHND_PMU_WRITE_4(sc, BHND_PMU_PLL_CONTROL_ADDR,
	    pll0 + BHND_PMU5_PLL_NM5_OFF);
	BHND_PMU_READ_4(sc, BHND_PMU_PLL_CONTROL_ADDR);

	tmp = BHND_PMU_READ_4(sc, BHND_PMU_PLL_CONTROL_DATA);
	ndiv = BHND_PMU_GET_BITS(tmp, BHND_PMU5_PLL_NDIV);

	/* Do calculation in Mhz */
	fc = bhnd_pmu_alp_clock(sc) / 1000000;
	fc = (p1 * ndiv * fc) / p2;

	PMU_MSG(("%s: p1=%d, p2=%d, ndiv=%d(0x%x), m%d=%d; fc=%d, clock=%d\n",
		  __func__, p1, p2, ndiv, ndiv, m, div, fc, fc / div));

	/* Return clock in Hertz */
	return ((fc / div) * 1000000);
}

/* query backplane clock frequency */
/* For designs that feed the same clock to both backplane
 * and CPU just return the CPU clock speed.
 */
uint32_t
bhnd_pmu_si_clock(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 clock;

	cid = bhnd_get_chipid(sc->dev);
	clock = BHND_PMU_HT_CLOCK;

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM43224:
	case BHND_CHIPID_BCM43225:
	case BHND_CHIPID_BCM43421:
	case BHND_CHIPID_BCM4331:
	case BHND_CHIPID_BCM6362:
		/* 96MHz backplane clock */
		clock = 96000 * 1000;
		break;

	case BHND_CHIPID_BCM4716:
	case BHND_CHIPID_BCM4748:
	case BHND_CHIPID_BCM47162:
		clock = bhnd_pmu5_clock(sc, BHND_PMU4716_MAINPLL_PLL0,
		    BHND_PMU5_MAINPLL_SI);
		break;

	case BHND_CHIPID_BCM4329:
		if (cid->chip_rev == 0)
			clock = 38400 * 1000;
		else
			clock = bhnd_pmu1_cpuclk0(sc);
		break;

	case BHND_CHIPID_BCM4319:
	case BHND_CHIPID_BCM4336:
	case BHND_CHIPID_BCM4330:
		clock = bhnd_pmu1_cpuclk0(sc);
		break;

	case BHND_CHIPID_BCM4313:
		/* 80MHz backplane clock */
		clock = 80000 * 1000;
		break;

	case BHND_CHIPID_BCM43235:
	case BHND_CHIPID_BCM43236:
	case BHND_CHIPID_BCM43238:
		// XXX TODO expose chip status
#ifdef notyet
		clock =
		    (cc->chipstatus & CST43236_BP_CLK) ? (120000 *
							  1000) : (96000 *
								   1000);
#endif
		break;
	case BHND_CHIPID_BCM5356:
		clock = bhnd_pmu5_clock(sc, BHND_PMU5356_MAINPLL_PLL0,
		    BHND_PMU5_MAINPLL_SI);
		break;
	case BHND_CHIPID_BCM5357:
		clock = bhnd_pmu5_clock(sc, BHND_PMU5357_MAINPLL_PLL0,
		    BHND_PMU5_MAINPLL_SI);
		break;
	default:
		device_printf(sc->dev, "No backplane clock specified for chip "
		    "%#hx rev %hhd pmurev %hhd, using default %dHz\n",
		    cid->chip_id, cid->chip_rev, BHND_PMU_REV(sc), clock);
		break;
	}

	return (clock);
}

/* query CPU clock frequency */
uint32_t 
bhnd_pmu_cpu_clock(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 clock;

	cid = bhnd_get_chipid(sc->dev);

	if (BHND_PMU_REV(sc) >= 5 &&
	    cid->chip_id != BHND_CHIPID_BCM4329 &&
	    cid->chip_id != BHND_CHIPID_BCM4319 &&
	    cid->chip_id != BHND_CHIPID_BCM4336 &&
	    cid->chip_id != BHND_CHIPID_BCM4330 &&
	    cid->chip_id != BHND_CHIPID_BCM43236)
	{
		u_int pll;

		switch (cid->chip_id) {
		case BHND_CHIPID_BCM5356:
			pll = BHND_PMU5356_MAINPLL_PLL0;
			break;
		case BHND_CHIPID_BCM5357:
			pll = BHND_PMU5357_MAINPLL_PLL0;
			break;
		default:
			pll = BHND_PMU4716_MAINPLL_PLL0;
			break;
		}

		clock = bhnd_pmu5_clock(sc, pll, BHND_PMU5_MAINPLL_CPU);
	} else {
		clock = bhnd_pmu_si_clock(sc);
	}

	return (clock);
}

/* query memory clock frequency */
uint32_t
bhnd_pmu_mem_clock(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 clock;

	cid = bhnd_get_chipid(sc->dev);

	if (BHND_PMU_REV(sc) >= 5 &&
	    cid->chip_id != BHND_CHIPID_BCM4329 &&
	    cid->chip_id != BHND_CHIPID_BCM4319 &&
	    cid->chip_id != BHND_CHIPID_BCM4336 &&
	    cid->chip_id != BHND_CHIPID_BCM4330 &&
	    cid->chip_id != BHND_CHIPID_BCM43236)
	{
		u_int pll;

		switch (cid->chip_id) {
		case BHND_CHIPID_BCM5356:
			pll = BHND_PMU5356_MAINPLL_PLL0;
			break;
		case BHND_CHIPID_BCM5357:
			pll = BHND_PMU5357_MAINPLL_PLL0;
			break;
		default:
			pll = BHND_PMU4716_MAINPLL_PLL0;
			break;
		}

		clock = bhnd_pmu5_clock(sc, pll, BHND_PMU5_MAINPLL_MEM);
	} else {
		clock = bhnd_pmu_si_clock(sc);
	}

	return clock;
}

/* Measure ILP clock frequency */
#define	ILP_CALC_DUR	10	/* ms, make sure 1000 can be divided by it. */

uint32_t
bhnd_pmu_ilp_clock(struct bhnd_pmu_softc *sc)
{
	uint32_t start, end, delta;

	if (sc->ilp_cps == 0) {
		start = BHND_PMU_READ_4(sc, BHND_PMU_TIMER);
		DELAY(ILP_CALC_DUR);
		end = BHND_PMU_READ_4(sc, BHND_PMU_TIMER);
		delta = end - start;
		sc->ilp_cps = delta * (1000 / ILP_CALC_DUR);
	}

	return (sc->ilp_cps);
}

/* SDIO Pad drive strength to select value mappings */
typedef struct {
	uint8_t	strength;	/* Pad Drive Strength in mA */
	uint8_t	sel;		/* Chip-specific select value */
} sdiod_drive_str_t;

/* SDIO Drive Strength to sel value table for PMU Rev 1 */
static const sdiod_drive_str_t sdiod_drive_strength_tab1[] = {
	{
	4, 0x2}, {
	2, 0x3}, {
	1, 0x0}, {
	0, 0x0}
	};

/* SDIO Drive Strength to sel value table for PMU Rev 2, 3 */
static const sdiod_drive_str_t sdiod_drive_strength_tab2[] = {
	{
	12, 0x7}, {
	10, 0x6}, {
	8, 0x5}, {
	6, 0x4}, {
	4, 0x2}, {
	2, 0x1}, {
	0, 0x0}
	};

/* SDIO Drive Strength to sel value table for PMU Rev 8 (1.8V) */
static const sdiod_drive_str_t sdiod_drive_strength_tab3[] = {
	{
	32, 0x7}, {
	26, 0x6}, {
	22, 0x5}, {
	16, 0x4}, {
	12, 0x3}, {
	8, 0x2}, {
	4, 0x1}, {
	0, 0x0}
	};

#define	SDIOD_DRVSTR_KEY(chip, pmu)	(((chip) << 16) | (pmu))

void
si_sdiod_drive_strength_init(struct bhnd_pmu_softc *sc, uint32_t drivestrength) 
{
	const struct bhnd_chipid	*cid;
	sdiod_drive_str_t		*str_tab;
	uint32_t			 str_mask;
	uint32_t			 str_shift;
	u_int				 intr_val;

	str_tab = NULL;
	str_mask = 0;
	str_shift = 0;
	intr_val = 0;

	cid = bhnd_get_chipid(sc->dev);

	switch (SDIOD_DRVSTR_KEY(cid->chip_id, BHND_PMU_REV(sc))) {
	case SDIOD_DRVSTR_KEY(BHND_CHIPID_BCM4325, 1):
		str_tab = (sdiod_drive_str_t *)&sdiod_drive_strength_tab1;
		str_mask = 0x30000000;
		str_shift = 28;
		break;
	case SDIOD_DRVSTR_KEY(BHND_CHIPID_BCM4325, 2):
	case SDIOD_DRVSTR_KEY(BHND_CHIPID_BCM4325, 3):
		str_tab = (sdiod_drive_str_t *)&sdiod_drive_strength_tab2;
		str_mask = 0x00003800;
		str_shift = 11;
		break;
	case SDIOD_DRVSTR_KEY(BHND_CHIPID_BCM4336, 8):
		str_tab = (sdiod_drive_str_t *) &sdiod_drive_strength_tab3;
		str_mask = 0x00003800;
		str_shift = 11;
		break;

	default:
		device_printf(sc->dev, "No SDIO Drive strength init done for "
		    "chip %#x rev %hhd pmurev %hhd\n", cid->chip_id,
		    cid->chip_rev, BHND_PMU_REV(sc));
		break;
	}

	if (str_tab != NULL) {
		uint32_t drivestrength_sel = 0;
		uint32_t cc_data_temp;

		for (u_int i = 0; str_tab[i].strength != 0; i++) {
			if (drivestrength >= str_tab[i].strength) {
				drivestrength_sel = str_tab[i].sel;
				break;
			}
		}

		cc_data_temp = bhnd_pmu_cctrl_read(sc, 1);
		cc_data_temp &= ~str_mask;
		drivestrength_sel <<= str_shift;
		cc_data_temp |= drivestrength_sel;
		bhnd_pmu_cctrl_write(sc, 1, cc_data_temp, ~0);

		PMU_MSG(("SDIO: %dmA drive strength selected, set to 0x%08x\n",
			 drivestrength, cc_data_temp));
	}
}

/* initialize PMU */
void 
bhnd_pmu_init(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	if (BHND_PMU_REV(sc) == 1) {
		BHND_PMU_AND_4(sc, BHND_PMU_CTRL, ~BHND_PMU_CTRL_NOILP_ON_WAIT);
	} else if (BHND_PMU_REV(sc) >= 2) {
		BHND_PMU_OR_4(sc, BHND_PMU_CTRL, BHND_PMU_CTRL_NOILP_ON_WAIT);
	}

	if (cid->chip_id == BHND_CHIPID_BCM4329 && cid->chip_rev == 2) {
		/* Fix for 4329b0 bad LPOM state. */
		bhnd_pmu_regctrl_write(sc, 2, 0x100, ~0);
		bhnd_pmu_regctrl_write(sc, 3, 0x4, ~0);
	}
}

/* Return up time in ILP cycles for the given resource. */
static uint32_t
bhnd_pmu_res_uptime(struct bhnd_pmu_softc *sc, uint8_t rsrc)
{
	uint32_t deps;
	uint32_t up, dup, dmax;
	uint32_t min_mask;

	/* uptime of resource 'rsrc' */
	BHND_PMU_WRITE_4(sc, BHND_PMU_RES_TABLE_SEL, rsrc);
	up = BHND_PMU_READ_4(sc, BHND_PMU_RES_UPDN_TIMER);
	up = BHND_PMU_GET_BITS(up, BHND_PMU_RES_UPDN_UPTME);

	/* Find direct dependancies of resource 'rsrc' */
	deps = bhnd_pmu_res_deps(sc, BHND_PMURES_BIT(rsrc), false);
	for (uint8_t i = 0; i <= BHND_PMU_RESNUM_MAX; i++) {
		if (!(deps & BHND_PMURES_BIT(i)))
			continue;
		deps &= ~bhnd_pmu_res_deps(sc, BHND_PMURES_BIT(i), true);
	}

	/* Exclude the minimum resource set */
	bhnd_pmu_res_masks(sc, &min_mask, NULL);
	deps &= ~min_mask;

	/* max uptime of direct dependancies */
	dmax = 0;
	for (uint8_t i = 0; i <= BHND_PMU_RESNUM_MAX; i++) {
		if (!(deps & BHND_PMURES_BIT(i)))
			continue;

		dup = bhnd_pmu_res_uptime(sc, i);
		if (dmax < dup)
			dmax = dup;
	}

	PMU_MSG(("bhnd_pmu_res_uptime: rsrc %hhu uptime %u(deps 0x%08x "
	    "uptime %u)\n", rsrc, up, deps, dmax));

	return up + dmax + BHND_PMURES_UP_TRANSITION;
}

/* Return dependancies (direct or all/indirect) for the given resources */
static uint32_t
bhnd_pmu_res_deps(struct bhnd_pmu_softc *sc, uint32_t rsrcs, bool all)
{
	uint32_t deps;

	deps = 0;
	for (uint8_t i = 0; i <= BHND_PMU_RESNUM_MAX; i++) {
		if (!(rsrcs & BHND_PMURES_BIT(i)))
			continue;

		BHND_PMU_WRITE_4(sc, BHND_PMU_RES_TABLE_SEL, i);
		deps |= BHND_PMU_READ_4(sc, BHND_PMU_RES_DEP_MASK);
	}

	/* None found? */
	if (deps == 0)
		return (0);

	/* Recurse dependencies */
	if (all)
		deps |= bhnd_pmu_res_deps(sc, deps, true);

	return (deps);
}

/* power up/down OTP through PMU resources */
int
bhnd_pmu_otp_power(struct bhnd_pmu_softc *sc, bool on)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 deps;
	uint32_t			 min_mask;
	uint32_t 			 rsrcs;

	cid = bhnd_get_chipid(sc->dev);

// XXX TODO: This belongs in ChipCommon, not in the PMU
#ifdef notyet
	/* Don't do anything if OTP is disabled */
	if (si_is_otp_disabled(sc)) {
		PMU_MSG(("bhnd_pmu_otp_power: OTP is disabled\n"));
		return (ENODEV);
	}
#endif

	/* Determine rsrcs to turn on/off OTP power */
	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4329:
		rsrcs = PMURES_BIT(RES4329_OTP_PU);
		break;
	case BHND_CHIPID_BCM4319:
		rsrcs = PMURES_BIT(RES4319_OTP_PU);
		break;
	case BHND_CHIPID_BCM4336:
		rsrcs = PMURES_BIT(RES4336_OTP_PU);
		break;
	case BHND_CHIPID_BCM4330:
		rsrcs = PMURES_BIT(RES4330_OTP_PU);
		break;
	default:
		/* Not required? */
		return (0);
	}

	/* Fetch all dependencies */
	deps = bhnd_pmu_res_deps(sc, rsrcs, true);

	/* Exclude the minimum resource set */
	bhnd_pmu_res_masks(sc, &min_mask, NULL);
	deps &= ~min_mask;

	/* Turn on/off the power */
	if (on) {
		uint32_t state;

		PMU_MSG(("Adding rsrc 0x%x to min_res_mask\n",
				rsrcs | deps));
		BHND_PMU_OR_4(sc, BHND_PMU_MIN_RES_MASK, (rsrcs|deps));

		/* Wait for all resources to become available */
		for (int i = 0; i < BHND_PMU_MAX_TRANSITION_DLY; i += 10) {	
			state = BHND_PMU_READ_4(sc, BHND_PMU_RES_STATE);
			if ((state & rsrcs) == rsrcs)
				break;

			DELAY(10);
		}

		if ((state & rsrcs) != rsrcs) {
			device_printf(sc->dev, "timeout waiting for OTP "
			   "resource enable\n");
			return (ENXIO);
		}
	} else {
		PMU_MSG(("Removing rsrc 0x%x from min_res_mask\n",
		    rsrcs | deps));
		BHND_PMU_AND_4(sc, BHND_PMU_MIN_RES_MASK, ~(rsrcs|deps));
	}

	// XXX TODO: This belongs in ChipCommon, not in the PMU
#ifdef notyet
	SPINWAIT((((otps = R_REG(&cc->otpstatus)) & OTPS_READY) !=
			(on ? OTPS_READY : 0)), 100);
	ASSERT((otps & OTPS_READY) == (on ? OTPS_READY : 0));
	if ((otps & OTPS_READY) != (on ? OTPS_READY : 0))
		PMU_MSG(("OTP ready bit not %s after wait\n",
				(on ? "ON" : "OFF")));
#endif

	return (0);
}

void 
bhnd_pmu_rcal(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 val;
	uint8_t				 rcal_code;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4329:
		/* Kick RCal */
		BHND_PMU_WRITE_4(sc, BHND_PMU_CHIPCTL_ADDR, 1);

		/* Power Down RCAL Block */
		BHND_PMU_AND_4(sc, BHND_PMU_CHIPCTL_DATA, ~0x04);

		/* Power Up RCAL block */
		BHND_PMU_AND_4(sc, BHND_PMU_CHIPCTL_DATA, 0x04);

		// XXX TODO - need access to chipc chipstatus
#ifdef notyet
		/* Wait for completion */
		SPINWAIT(0 == (R_REG(&cc->chipstatus) & 0x08),
				10 * 1000 * 1000);
		ASSERT(R_REG(&cc->chipstatus) & 0x08);

		/* Drop the LSB to convert from 5 bit code to 4 bit code */
		rcal_code =
			(uint8_t) (R_REG(&cc->chipstatus) >> 5) & 0x0f;

		PMU_MSG(("RCal completed, status 0x%x, code 0x%x\n",
				R_REG(&cc->chipstatus), rcal_code));
#else
		rcal_code = 0;
#endif

		/* Write RCal code into pmu_vreg_ctrl[32:29] */
		BHND_PMU_WRITE_4(sc, BHND_PMU_REG_CONTROL_ADDR, 0);
		val = BHND_PMU_READ_4(sc, BHND_PMU_REG_CONTROL_DATA);
		val &= ~((uint32_t) 0x07 << 29);
		val |= (uint32_t) (rcal_code & 0x07) << 29;
		BHND_PMU_WRITE_4(sc, BHND_PMU_REG_CONTROL_DATA, val);

		BHND_PMU_WRITE_4(sc, BHND_PMU_REG_CONTROL_ADDR, 1);
		val = BHND_PMU_READ_4(sc, BHND_PMU_REG_CONTROL_DATA);
		val &= ~(uint32_t) 0x01;
		val |= (uint32_t) ((rcal_code >> 3) & 0x01);
		BHND_PMU_WRITE_4(sc, BHND_PMU_REG_CONTROL_DATA, val);

		/* Write RCal code into pmu_chip_ctrl[33:30] */
		BHND_PMU_WRITE_4(sc, BHND_PMU_CHIPCTL_ADDR, 0);
		val = BHND_PMU_READ_4(sc, BHND_PMU_CHIPCTL_DATA);
		val &= ~((uint32_t) 0x03 << 30);
		val |= (uint32_t) (rcal_code & 0x03) << 30;
		BHND_PMU_WRITE_4(sc, BHND_PMU_CHIPCTL_DATA, val);

		BHND_PMU_WRITE_4(sc, BHND_PMU_CHIPCTL_ADDR, 1);
		val = BHND_PMU_READ_4(sc, BHND_PMU_CHIPCTL_DATA);
		val &= ~(uint32_t) 0x03;
		val |= (uint32_t) ((rcal_code >> 2) & 0x03);
		BHND_PMU_WRITE_4(sc, BHND_PMU_CHIPCTL_DATA, val);

		/* Set override in pmu_chip_ctrl[29] */
		BHND_PMU_WRITE_4(sc, BHND_PMU_CHIPCTL_ADDR, 0);
		BHND_PMU_OR_4(sc, BHND_PMU_CHIPCTL_DATA, (0x01 << 29));

		/* Power off RCal block */
		BHND_PMU_WRITE_4(sc, BHND_PMU_CHIPCTL_ADDR, 1);
		BHND_PMU_AND_4(sc, BHND_PMU_CHIPCTL_DATA, ~0x04);
		break;
	default:
		break;
	}
}

void 
bhnd_pmu_spuravoid(struct bhnd_pmu_softc *sc, uint8_t spuravoid)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	/* force the HT off  */
	if (cid->chip_id == BHND_CHIPID_BCM4336) {		
		BHND_PMU_AND_4(sc, BHND_PMU_MAX_RES_MASK,
		    ~BHND_PMU_RES4336_HT_AVAIL);

		// XXX: need access to per-core clk_ctl_st register, or
		// bhnd-level API for HT availability.
#ifdef notyet
		/* wait for the ht to really go away */
		SPINWAIT(((R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL) == 0),
			 10000);
		ASSERT((R_REG(&cc->clk_ctl_st) & CCS_HTAVAIL) == 0);
#endif
	}

	/* update the pll changes */
	bhnd_pmu_spuravoid_pllupdate(sc, spuravoid);

	/* enable HT back on  */
	if (cid->chip_id == BHND_CHIPID_BCM4336) {
		BHND_PMU_OR_4(sc, BHND_PMU_MAX_RES_MASK,
		    BHND_PMU_RES4336_HT_AVAIL);
	}
}

static void
bhnd_pmu_spuravoid_pllupdate(struct bhnd_pmu_softc *sc, uint8_t spuravoid)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 tmp;
	uint32_t			 pmuctrl;
	uint8_t				 phypll_offset;

	uint8_t bcm5357_bcm43236_p1div[] = { 0x1, 0x5, 0x5 };
	uint8_t bcm5357_bcm43236_ndiv[] = { 0x30, 0xf6, 0xfc };

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM5357:
	case BHND_CHIPID_BCM43235:
	case BHND_CHIPID_BCM43236:
	case BHND_CHIPID_BCM43238:
		KASSERT(spuravoid < nitems(bcm5357_bcm43236_p1div),
		    ("spuravoid %hhu outside p1div table\n", spuravoid));

		KASSERT(spuravoid < nitems(bcm5357_bcm43236_ndiv),
		    ("spuravoid %hhu outside ndiv table\n", spuravoid));

		/* BCM5357 needs to touch PLL1_PLLCTL[02], so offset
		 * PLL0_PLLCTL[02] by 6 */
		phypll_offset = 0;
		if (cid->chip_id == BHND_CHIPID_BCM5357)
			phypll_offset = 6;

		/* RMW only the P1 divider */
		tmp = BHND_PMU_SET_BITS(bcm5357_bcm43236_p1div[spuravoid],
		    BHND_PMU1_PLL0_PC0_P1DIV);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0 + phypll_offset,
		    tmp, BHND_PMU1_PLL0_PC0_P1DIV_MASK);

		/* RMW only the int feedback divider */
		tmp = BHND_PMU_SET_BITS(bcm5357_bcm43236_ndiv[spuravoid],
		    BHND_PMU1_PLL0_PC2_NDIV_INT);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2 + phypll_offset,
		    tmp, BHND_PMU1_PLL0_PC0_P1DIV_MASK);

		pmuctrl = BHND_PMU_CTRL_PLL_PLLCTL_UPD;
		break;

	case BHND_CHIPID_BCM4331:
		if (spuravoid == 2) {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11500014, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x0FC00a08, ~0);
		} else if (spuravoid == 1) {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11500014, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x0F600a08, ~0);
		} else {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11100014, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x03000a08, ~0);
		}
		pmuctrl = BHND_PMU_CTRL_PLL_PLLCTL_UPD;
		break;

	case BHND_CHIPID_BCM43224:
	case BHND_CHIPID_BCM43225:
	case BHND_CHIPID_BCM43421:
	case BHND_CHIPID_BCM6362:
		if (spuravoid == 1) {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11500010, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1,
			    0x000C0C06, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x0F600a08, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3,
			    0x00000000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4,
			    0x2001E920, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5,
			    0x88888815, ~0);
		} else {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11100010, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1,
			    0x000c0c06, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x03000a08, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3,
			    0x00000000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4,
			    0x200005c0, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5,
			    0x88888815, ~0);
		}
		pmuctrl = BHND_PMU_CTRL_PLL_PLLCTL_UPD;
		break;

	case BHND_CHIPID_BCM43111:
	case BHND_CHIPID_BCM43112:
	case BHND_CHIPID_BCM43222:
	case BHND_CHIPID_BCM43420:
		if (spuravoid == 1) {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11500008, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1,
			    0x0c000c06, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x0f600a08, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3,
			    0x00000000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4,
			    0x2001e920, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5,
			    0x88888815, ~0);
		} else {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11100008, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1,
			    0x0c000c06, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x03000a08, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3,
			    0x00000000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4,
			    0x200005c0, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5,
			    0x88888855, ~0);
		}

		pmuctrl = BHND_PMU_CTRL_PLL_PLLCTL_UPD;
		break;

	case BHND_CHIPID_BCM4716:
	case BHND_CHIPID_BCM4748:
	case BHND_CHIPID_BCM47162:
		if (spuravoid == 1) {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11500060, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1,
			    0x080C0C06, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x0F600000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3,
			    0x00000000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4,
			    0x2001E924, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5,
			    0x88888815, ~0);
		} else {
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0,
			    0x11100060, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1,
			    0x080c0c06, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x03000000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3,
			    0x00000000, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4,
			    0x200005c0, ~0);
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5,
			    0x88888815, ~0);
		}

		pmuctrl = BHND_PMU_CTRL_NOILP_ON_WAIT | 
			  BHND_PMU_CTRL_PLL_PLLCTL_UPD;
		break;

	case BHND_CHIPID_BCM4319:
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0, 0x11100070, ~0);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1, 0x1014140a, ~0);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5, 0x88888854, ~0);

		if (spuravoid == 1) {
			/* spur_avoid ON, enable 41/82/164Mhz clock mode */
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x05201828, ~0);
		} else {
			/* enable 40/80/160Mhz clock mode */
			bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2,
			    0x05001828, ~0);
		}

		pmuctrl = 0;
		break;

	case BHND_CHIPID_BCM4336:
		/* Looks like these are only for default xtal freq 26MHz */
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL0, 0x02100020, ~0);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1, 0x0C0C0C0C, ~0);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2, 0x01240C0C, ~0);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL4, 0x202C2820, ~0);
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL5, 0x88888825, ~0);

		if (spuravoid == 1) {
			tmp = 0x00EC4EC4;
		} else {
			tmp = 0x00762762;
		}
		bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL3, tmp, ~0);


		pmuctrl = BHND_PMU_CTRL_PLL_PLLCTL_UPD;
		break;

	default:
		PMU_ERROR(("%s: unknown spuravoidance settings for chip %#hx, "
		    "not changing PLL\n", __func__, cid->chip_id));
		break;
	}

	BHND_PMU_OR_4(sc, BHND_PMU_CTRL, pmuctrl);
}

bool
bhnd_pmu_is_otp_powered(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid	*cid;
	uint32_t			 otp_res;

	cid = bhnd_get_chipid(sc->dev);

	/* Determine per-chip OTP resource */
	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4329:
		otp_res = PMURES_BIT(RES4329_OTP_PU);
		break;
	case BHND_CHIPID_BCM4319:
		otp_res = PMURES_BIT(RES4319_OTP_PU);
		break;
	case BHND_CHIPID_BCM4336:
		otp_res = PMURES_BIT(RES4336_OTP_PU);
		break;
	case BHND_CHIPID_BCM4330:
		otp_res = PMURES_BIT(RES4330_OTP_PU);
		break;

	/* These chips don't use PMU bit to power up/down OTP. OTP always on.
	 * Use OTP_INIT command to reset/refresh state.
	 */
	case BHND_CHIPID_BCM43224:
	case BHND_CHIPID_BCM43225:
	case BHND_CHIPID_BCM43421:
	case BHND_CHIPID_BCM43236:
	case BHND_CHIPID_BCM43235:
	case BHND_CHIPID_BCM43238:
		return (true);

	default:
		return (true);
	}

	/* Check resource state */
	if ((BHND_PMU_READ_4(sc, BHND_PMU_RES_STATE) & otp_res) == 0)
		return (false);

	return (true);
}

/* initialize PMU switch/regulators */
void
bhnd_pmu_swreg_init(struct bhnd_pmu_softc *sc)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4336:
		/* Reduce CLDO PWM output voltage to 1.2V */
		bhnd_pmu_set_ldo_voltage(sc, SET_LDO_VOLTAGE_CLDO_PWM, 0xe);
		/* Reduce CLDO BURST output voltage to 1.2V */
		bhnd_pmu_set_ldo_voltage(sc, SET_LDO_VOLTAGE_CLDO_BURST, 0xe);
		/* Reduce LNLDO1 output voltage to 1.2V */
		bhnd_pmu_set_ldo_voltage(sc, SET_LDO_VOLTAGE_LNLDO1, 0xe);
		if (cid->chip_rev == 0)
			bhnd_pmu_regctrl_write(sc, 2, 0x400000, 0x400000);
		break;

	case BHND_CHIPID_BCM4330:
		/* CBUCK Voltage is 1.8 by default and set that to 1.5 */
		bhnd_pmu_set_ldo_voltage(sc, SET_LDO_VOLTAGE_CBUCK_PWM, 0);
		break;
	default:
		break;
	}
}

void
bhnd_pmu_radio_enable(struct bhnd_pmu_softc *sc, bool enable)
{
	const struct bhnd_chipid *cid;

	cid = bhnd_get_chipid(sc->dev);

	switch (cid->chip_id) {
	case BHND_CHIPID_BCM4319:
		if (enable)
			si_write_wrapperreg(sc, BCMA_DMP_OOBSELOUTB74,
			    0x868584);
		else
			si_write_wrapperreg(sc, BCMA_DMP_OOBSELOUTB74,
			    0x060584);
		break;
	}
}

/* Wait for a particular clock level to be on the backplane */
uint32_t
bhnd_pmu_waitforclk_on_backplane(struct bhnd_pmu_softc *sc, uint32_t clk,
    uint32_t delay)
{
	uint32_t pmu_st;

	for (uint32_t i = 0; i < delay; i += 10) {
		pmu_st = BHND_PMU_READ_4(sc, BHND_PMU_ST);
		if ((pmu_st & clk) == clk)
			return (clk);
		
		DELAY(10);
	}

	pmu_st = BHND_PMU_READ_4(sc, BHND_PMU_ST);
	return (pmu_st & clk);
}

/*
 * Measures the ALP clock frequency in KHz.  Returns 0 if not possible.
 * Possible only if PMU rev >= 10 and there is an external LPO 32768Hz crystal.
 */

#define	EXT_ILP_HZ 32768

uint32_t
bhnd_pmu_measure_alpclk(struct bhnd_pmu_softc *sc)
{
	uint32_t alp_khz;
	uint32_t pmu_st;

	if (BHND_PMU_REV(sc) < 10)
		return (0);

	pmu_st = BHND_PMU_READ_4(sc, BHND_PMU_ST);
	if (pmu_st & BHND_PMU_ST_EXTLPOAVAIL) {
		uint32_t alp_hz, ilp_ctr;

		/* Enable the reg to measure the freq, in case disabled before */
		BHND_PMU_WRITE_4(sc, BHND_PMU_XTALFREQ, 1U <<
		    BHND_PMU_XTALFREQ_REG_MEASURE_SHIFT);

		/* Delay for well over 4 ILP clocks */
		DELAY(1000);

		/* Read the latched number of ALP ticks per 4 ILP ticks */
		ilp_ctr = BHND_PMU_READ_4(sc, BHND_PMU_XTALFREQ);
		ilp_ctr = BHND_PMU_GET_BITS(ilp_ctr,
		    BHND_PMU_XTALFREQ_REG_ILPCTR);

		/* Turn off the PMU_XTALFREQ_REG_MEASURE_SHIFT bit to save power */
		BHND_PMU_WRITE_4(sc, BHND_PMU_XTALFREQ, 0);

		/* Calculate ALP frequency */
		alp_hz = (ilp_ctr * EXT_ILP_HZ) / 4;

		/* Round to nearest 100KHz, and at the same time convert to KHz */
		alp_khz = (alp_hz + 50000) / 100000 * 100;
	} else {
		alp_khz = 0;
	}

	return (alp_khz);
}

static void 
bhnd_pmu_set_4330_plldivs(struct bhnd_pmu_softc *sc)
{
	uint32_t FVCO = bhnd_pmu1_pllfvco0(sc) / 1000;
	uint32_t m1div, m2div, m3div, m4div, m5div, m6div;
	uint32_t pllc1, pllc2;

	m2div = m3div = m4div = m6div = FVCO / 80;
	m5div = FVCO / 160;

	if (CST4330_CHIPMODE_SDIOD(sih->chipst))
		m1div = FVCO / 80;
	else
		m1div = FVCO / 90;

	pllc1 = 0;
	pllc1 |= BHND_PMU_SET_BITS(m1div, BHND_PMU1_PLL0_PC1_M1DIV);
	pllc1 |= BHND_PMU_SET_BITS(m2div, BHND_PMU1_PLL0_PC1_M2DIV);
	pllc1 |= BHND_PMU_SET_BITS(m3div, BHND_PMU1_PLL0_PC1_M3DIV);
	pllc1 |= BHND_PMU_SET_BITS(m4div, BHND_PMU1_PLL0_PC1_M4DIV);

	bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL1, pllc1, ~0);

	pllc2 = 0;
	pllc2 |= BHND_PMU_SET_BITS(m5div, BHND_PMU1_PLL0_PC2_M5DIV);
	pllc2 |= BHND_PMU_SET_BITS(m6div, BHND_PMU1_PLL0_PC2_M6DIV);

	bhnd_pmu_pll_write(sc, BHND_PMU1_PLL0_PLLCTL2, pllc2,
	    BHND_PMU1_PLL0_PC2_M5DIV_MASK | BHND_PMU1_PLL0_PC2_M6DIV_MASK);
}
