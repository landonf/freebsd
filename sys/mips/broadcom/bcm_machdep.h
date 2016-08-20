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
 * 
 * $FreeBSD$
 */

#ifndef	_MIPS_BROADCOM_BCM_MACHDEP_H_
#define	_MIPS_BROADCOM_BCM_MACHDEP_H_

#include <machine/cpufunc.h>
#include <machine/cpuregs.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/cores/pmu/bhnd_pmuvar.h>

struct bcm_bus_ops;

extern const struct bhnd_pmu_io		bcm_pmu_soc_io;
extern const struct bcm_bus_ops		bcm_siba_ops __weak_symbol;
extern const struct bcm_bus_ops		bcm_bcma_ops __weak_symbol;

/**
 * Search @p chipid's enumeration table for a core with @p devclass and
 * @p unit, returning the corresponding core @p info and register
 * block @p addr.
 * 
 * @param	chipid		Chip identification data, including the address
 *				of the enumeration table to be searched.
 * @param	devclass	Search for a core matching this device class.
 * @param	unit		The core's required unit number.
 * @param[out]	info		On success, will be populated with the core
 *				info.
 */
typedef int	(bcm_bus_find_core)(struct bhnd_chipid *chipid,
		    bhnd_devclass_t devclass, int unit,
		    struct bhnd_core_info *info, uintptr_t *addr);

/**
 * Fix the core count in @p cid, if required.
 * 
 * This is currently only necessary on early siba(4) devices, where
 * early hardware may provide an invalid core count.
 * 
 * @param	cid		The chipid data.
 * @param	chipc_hwrev	The revision of the ChipCommon core from which
 *				@p cid was parsed.
 */
typedef int	(bcm_bus_fix_num_cores)(struct bhnd_chipid *cid,
		    uint16_t chipc_hwrev);

/**
 * Parse all core descriptors in @p chipid's enumeration table and write the
 * result to @p cores, and the count in @p ncores.
 * 
 * @param		chipid	Chip identification data, including the address
 *				of the enumeration table to be searched.
 * @param[out]		cores	The array to which core descriptors will be
 *				written. If NULL, the actual number of cores
 *				will be returned via @p num_cores.
 * @param[in,out]	ncores	The maximum number of core info entries to be
 *				written to @p cores. On success, will be set
 *				to the actual number of core
 *				records written.
 * 
 * @retval 0		success
 * @retval non-zero	If parsing the core table otherwise fails, a regular
 *			unix error code will be returned.
 */
typedef int	(bcm_bus_read_core_table)(struct bhnd_chipid *chipid,
		    struct bhnd_core_info *cores, u_int *ncores);

/**
 * Bus-specific operations.
 */
struct bcm_bus_ops {
	bcm_bus_find_core	*find_core;
	bcm_bus_read_core_table	*read_core_table;
	bcm_bus_fix_num_cores	*fix_num_cores;
};

/**
 * Platform configuration.
 */
struct bcm_platform {
	struct bhnd_chipid		 id;		/**< chip id */
	struct bhnd_core_info		 cc_id;		/**< chipc core info */
	uintptr_t			 cc_addr;	/**< chipc core phys address */
	uint32_t			 cc_caps;	/**< chipc capabilities */
	uint32_t			 cc_caps_ext;	/**< chipc extended capabilies */
	const struct bcm_bus_ops	*bus_ops;	/**< bus-specific operations */

	/* On non-AOB devices, the PMU register block is mapped to chipc;
	 * the pmu_id and pmu_addr values will be copied from cc_id
	 * and cc_addr. */
	struct bhnd_core_info		 pmu_id;	/**< PMU core info */
	uintptr_t			 pmu_addr;	/**< PMU core phys address, or
							     0x0 if no PMU */
	struct bhnd_pmu_query		 pmu;		/**< PMU query instance */

#ifdef CFE
	int				 cfe_console;	/**< Console handle, or -1 */
#endif
};

struct bcm_platform	*bcm_get_platform(void);

uint64_t		 bcm_get_cpufreq(struct bcm_platform *bp);
uint64_t		 bcm_get_sifreq(struct bcm_platform *bp);
uint64_t		 bcm_get_alpfreq(struct bcm_platform *bp);
uint64_t		 bcm_get_ilpfreq(struct bcm_platform *bp);

u_int			 bcm_get_uart_rclk(struct bcm_platform *bp);

/**
 * Search @p bp's enumeration table for a core with @p devclass and
 * @p unit, returning the corresponding core @p info and register
 * block @p addr.
 * 
 * @param	bp		Platform data.
 * @param	devclass	Search for a core matching this device class.
 * @param	unit		The core's required unit number.
 * @param[out]	info		On success, will be populated with the core
 *				info.
 */
static inline int
bcm_find_core(struct bcm_platform *bp, bhnd_devclass_t devclass, int unit,
    struct bhnd_core_info *info, uintptr_t *addr)
{
	return (bp->bus_ops->find_core(&bp->id, devclass, unit, info, addr));
}

/**
 * Parse all core descriptors in @p bp's enumeration table and write the
 * result to @p cores, and the count in @p ncores.
 * 
 * @param		bp	Platform data.
 * @param[out]		cores	The array to which core descriptors will be
 *				written. If NULL, the actual number of cores
 *				will be returned via @p num_cores.
 * @param[in,out]	ncores	The maximum number of core info entries to be
 *				written to @p cores. On success, will be set
 *				to the actual number of core
 *				records written.
 * 
 * @retval 0		success
 * @retval non-zero	If parsing the core table otherwise fails, a regular
 *			unix error code will be returned.
 */
static inline int
bcm_read_core_table(struct bcm_platform *bp, struct bhnd_core_info *cores,
    u_int *ncores)
{
	return (bp->bus_ops->read_core_table(&bp->id, cores, ncores));
}

#define	BCM_SOC_ADDR(_addr, _offset)			\
	MIPS_PHYS_TO_KSEG1((_addr) + (_offset))

#define	BCM_SOC_READ_4(_addr, _offset)			\
	readl(BCM_SOC_ADDR((_addr), (_offset)))
#define	BCM_SOC_WRITE_4(_addr, _reg, _val)		\
	writel(BCM_SOC_ADDR((_addr), (_offset)), (_val))

#define	BCM_CORE_ADDR(_bp, _name, _reg)			\
	BCM_SOC_ADDR(_bp->_name, (_reg))

#define	BCM_CORE_READ_4(_bp, _name, _reg)		\
	readl(BCM_CORE_ADDR(_bp, _name, (_reg)))
#define	BCM_CORE_WRITE_4(_bp, _name, _reg, _val)	\
	writel(BCM_CORE_ADDR(_bp, _name, (_reg)), (_val))

#define	BCM_CHIPC_READ_4(_bp, _reg)			\
	BCM_CORE_READ_4(_bp, cc_addr, (_reg))
#define	BCM_CHIPC_WRITE_4(_bp, _reg, _val)		\
	BCM_CORE_WRITE_4(_bp, cc_addr, (_reg), (_val))

#define	BCM_PMU_READ_4(_bp, _reg)			\
	BCM_CORE_READ_4(_bp, pmu_addr, (_reg))
#define	BCM_PMU_WRITE_4(_bp, _reg, _val)		\
	BCM_CORE_WRITE_4(_bp, pmu_addr, (_reg), (_val))

#endif /* _MIPS_BROADCOM_BCM_MACHDEP_H_ */
