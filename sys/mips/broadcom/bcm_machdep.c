/*-
 * Copyright (c) 2007 Bruce M. Simpson.
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 *
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/imgact.h>
#include <sys/bio.h>
#include <sys/buf.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/cons.h>
#include <sys/exec.h>
#include <sys/ucontext.h>
#include <sys/proc.h>
#include <sys/kdb.h>
#include <sys/ptrace.h>
#include <sys/reboot.h>
#include <sys/signalvar.h>
#include <sys/sysent.h>
#include <sys/sysproto.h>
#include <sys/user.h>

#include <vm/vm.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>

#include <machine/cache.h>
#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpuinfo.h>
#include <machine/cpufunc.h>
#include <machine/cpuregs.h>
#include <machine/hwfunc.h>
#include <machine/intr_machdep.h>
#include <machine/locore.h>
#include <machine/md_var.h>
#include <machine/pte.h>
#include <machine/sigframe.h>
#include <machine/trap.h>
#include <machine/vmparam.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndreg.h>

#include <dev/bhnd/bcma/bcma_eromvar.h>

#include <dev/bhnd/siba/sibareg.h>
#include <dev/bhnd/siba/sibavar.h>

#include <dev/bhnd/cores/chipc/chipcreg.h>
#include <dev/bhnd/cores/pmu/bhnd_pmureg.h>

#include "bcm_machdep.h"
#include "bcm_mips_exts.h"

#ifdef CFE
#include <dev/cfe/cfe_api.h>
#endif

#if 0
#define	BCM_TRACE(_fmt, ...)	printf(_fmt, ##__VA_ARGS__)
#else
#define	BCM_TRACE(_fmt, ...)
#endif

static int	bcm_find_core(struct bhnd_chipid *chipid,
		    bhnd_devclass_t devclass, int unit,
		    struct bhnd_core_info *info, uintptr_t *addr);
static int	bcm_init_platform_data(struct bcm_platform *pdata);

/* Allow bus-specific implementations to override bcm_find_core_(bcma|siba)
 * symbols, if included in the kernel build */
__weak_reference(bcm_find_core_default, bcm_find_core_bcma);
__weak_reference(bcm_find_core_default, bcm_find_core_siba);

extern int	*edata;
extern int	*end;

static struct bcm_platform	 bcm_platform_data;
static bool			 bcm_platform_data_avail = false;

struct bcm_platform *
bcm_get_platform(void)
{
	if (!bcm_platform_data_avail)
		panic("platform data not available");

	return (&bcm_platform_data);
}

/* Default (no-op) bcm_find_core() implementation. */
int
bcm_find_core_default(struct bhnd_chipid *chipid, bhnd_devclass_t devclass,
    int unit, struct bhnd_core_info *info, uintptr_t *addr)
{
	return (ENODEV);
}

/**
 * Search @p chipid's enumeration table for a core with @p devclass and
 * @p unit.
 * 
 * @param	chipid		Chip identification data, including the address
 *				of the enumeration table to be searched.
 * @param	devclass	Search for a core matching this device class.
 * @param	unit		The core's required unit number.
 * @param[out]	info		On success, will be populated with the core
 *				info.
 */
static int
bcm_find_core(struct bhnd_chipid *chipid, bhnd_devclass_t devclass, int unit,
    struct bhnd_core_info *info, uintptr_t *addr)
{
	switch (chipid->chip_type) {
	case BHND_CHIPTYPE_SIBA:
		return (bcm_find_core_siba(chipid, devclass, unit, info, addr));
		break;
	default:
		if (!BHND_CHIPTYPE_HAS_EROM(chipid->chip_type)) {
			printf("%s: unsupported chip type: %d\n", __FUNCTION__,
			    chipid->chip_type);
			return (ENXIO);
		}
		return (bcm_find_core_bcma(chipid, devclass, unit, info, addr));
	}
}

static bus_addr_t
bcm_get_bus_addr(void)
{
	long maddr;

	if (resource_long_value("bhnd", 0, "maddr", &maddr) == 0)
		return ((u_long)maddr);

	return (BHND_DEFAULT_CHIPC_ADDR);
}

/**
 * Probe and attach a bhnd_erom parser instance for the bhnd bus.
 * 
 * @param[out]	erom_cls	The probed EROM class.
 * @param[out]	erom_ops	The storage to be used when compiling
 *				@p erom_cls.
 * @param[out]	erom		The storage to be used when initializing the
 *				static instance of @p erom_cls.
 * @param	esize		The total available number of bytes allocated
 *				for @p erom. If this is less than is required
 *				by @p erom_cls ENOMEM will be returned.
 * @param[out]	cid		On success, the probed chip identification.
 */
static int
bcm_erom_probe_and_attach(bhnd_erom_class_t **erom_cls, kobj_ops_t erom_ops,
    bhnd_erom_t *erom, size_t esize, struct bhnd_chipid *cid)
{
	bhnd_erom_class_t	**clsp;
	bus_space_tag_t		  bst;
	bus_space_handle_t	  bsh;
	bus_addr_t		  bus_addr;
	int			  error, prio, result;

	bus_addr = bcm_get_bus_addr();
	*erom_cls = NULL;
	prio = 0;

	bst = mips_bus_space_generic;
	bsh = BCM_SOC_BSH(bus_addr, 0);

	SET_FOREACH(clsp, bhnd_erom_class_set) {
		struct bhnd_chipid	 pcid;
		bhnd_erom_class_t	*cls;
		struct kobj_ops		 kops;

		cls = *clsp;

		/* Compile the class' ops table */
		kobj_class_compile_static(cls, &kops);

		/* Probe the bus address */
		result = bhnd_erom_probe_static(cls, bst, bsh, bus_addr, &pcid);

		/* Drop pointer to stack allocated ops table */
		cls->ops = NULL;

		/* The parser did not match if an error was returned */
		if (result > 0)
			continue;

		/* Check for a new highest priority match */
		if (*erom_cls == NULL || result > prio) {
			prio = result;

			*cid = pcid;
			*erom_cls = cls;
		}

		/* Terminate immediately on BUS_PROBE_SPECIFIC */
		if (result == BUS_PROBE_SPECIFIC)
			break;
	}

	/* Valid EROM class probed? */
	if (*erom_cls == NULL) {
		printf("%s: no erom parser found for root bus at %#jx\n",
		    __FUNCTION__, (uintmax_t)bus_addr);
		return (ENOENT);
	}

	/* Using the provided storage, recompile the erom class ... */
	kobj_class_compile_static(*erom_cls, erom_ops);

	/* ... and initialize the erom parser instance */
	bsh = BCM_SOC_BSH(cid->enum_addr, 0);
	error = bhnd_erom_init_static(*erom_cls, erom, esize,
	    mips_bus_space_generic, bsh);

	return (error);
}

/**
 * Populate platform configuration data.
 */
static int
bcm_init_platform_data(struct bcm_platform *pdata)
{
	uint32_t		reg;
	bhnd_addr_t		enum_addr;
	uint8_t			chip_type;
	bool			aob, pmu;
	int			error;

	/* Fetch CFE console handle (if any). Must be initialized before
	 * any calls to printf/early_putc. */
#ifdef CFE
	if ((pdata->cfe_console = cfe_getstdhandle(CFE_STDHANDLE_CONSOLE)) < 0)
		pdata->cfe_console = -1;
#endif

	/* Probe and attach device table provider */
	error = bcm_erom_probe_and_attach(&pdata->erom_impl, &pdata->erom_ops,
	    &pdata->erom.obj, sizeof(pdata->erom), &pdata->cid);
	if (error) {
		printf("%s: error attaching erom parser: %d\n", __FUNCTION__,
		    error);
		return (error);
	}

	/* Fetch chipc address */
	// TODO
	pdata->cc_addr = bcm_get_bus_addr();

	/* Read chip identifier from ChipCommon */
	reg = BCM_SOC_READ_4(pdata->cc_addr, CHIPC_ID);
	chip_type = CHIPC_GET_BITS(reg, CHIPC_ID_BUS);

	if (BHND_CHIPTYPE_HAS_EROM(chip_type))
		enum_addr = BCM_SOC_READ_4(pdata->cc_addr, CHIPC_EROMPTR);
	else
		enum_addr = pdata->cc_addr;

	pdata->cid = bhnd_parse_chipid(reg, enum_addr);

	/* Fetch chipc core info and capabilities */
	pdata->cc_caps = BCM_SOC_READ_4(pdata->cc_addr, CHIPC_CAPABILITIES);

	error = bcm_find_core(&pdata->cid, BHND_DEVCLASS_CC, 0, &pdata->cc_id,
	    NULL);
	if (error) {
		printf("%s: error locating chipc core: %d", __FUNCTION__,
		    error);
		return (error);
	}

	if (CHIPC_HWREV_HAS_CAP_EXT(pdata->cc_id.hwrev)) {
		pdata->cc_caps_ext = BCM_SOC_READ_4(pdata->cc_addr,
		    CHIPC_CAPABILITIES_EXT);
	} else {
		pdata->cc_caps_ext = 0x0;	
	}

	/* Fetch PMU info */
	pmu = CHIPC_GET_FLAG(pdata->cc_caps, CHIPC_CAP_PMU);
	aob = CHIPC_GET_FLAG(pdata->cc_caps_ext, CHIPC_CAP2_AOB);

	if (pmu && aob) {
		/* PMU block mapped to a PMU core on the Always-on-Bus (aob) */
		error = bcm_find_core(&pdata->cid, BHND_DEVCLASS_PMU, 0,
		    &pdata->pmu_id,  &pdata->pmu_addr);

		if (error) {
			printf("%s: error locating pmu core: %d", __FUNCTION__,
			    error);
			return (error);
		}
	} else if (pmu) {
		/* PMU block mapped to chipc */
		pdata->pmu_addr = pdata->cc_addr;
		pdata->pmu_id = pdata->cc_id;
	} else {
		/* No PMU */
		pdata->pmu_addr = 0x0;
		memset(&pdata->pmu_id, 0, sizeof(pdata->pmu_id));
	}

	if (pmu) {
		error = bhnd_pmu_query_init(&pdata->pmu, NULL, pdata->cid,
		    &bcm_pmu_soc_io, pdata);
		if (error) {
			printf("%s: bhnd_pmu_query_init() failed: %d\n",
			    __FUNCTION__, error);
			return (error);
		}
	}

	bcm_platform_data_avail = true;
	return (0);
}

void
platform_cpu_init()
{
	/* Nothing special */
}

static void
mips_init(void)
{
	int i, j;

	printf("entry: mips_init()\n");

#ifdef CFE
	/*
	 * Query DRAM memory map from CFE.
	 */
	physmem = 0;
	for (i = 0; i < 10; i += 2) {
		int result;
		uint64_t addr, len, type;

		result = cfe_enummem(i / 2, 0, &addr, &len, &type);
		if (result < 0) {
			BCM_TRACE("There is no phys memory for: %d\n", i);
			phys_avail[i] = phys_avail[i + 1] = 0;
			break;
		}
		if (type != CFE_MI_AVAILABLE) {
			BCM_TRACE("phys memory is not available: %d\n", i);
			continue;
		}

		phys_avail[i] = addr;
		if (i == 0 && addr == 0) {
			/*
			 * If this is the first physical memory segment probed
			 * from CFE, omit the region at the start of physical
			 * memory where the kernel has been loaded.
			 */
			phys_avail[i] += MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);
		}
		
		BCM_TRACE("phys memory is available for: %d\n", i);
		BCM_TRACE(" => addr =  %jx\n", addr);
		BCM_TRACE(" => len =  %jd\n", len);

		phys_avail[i + 1] = addr + len;
		physmem += len;
	}

	BCM_TRACE("Total phys memory is : %ld\n", physmem);
	realmem = btoc(physmem);
#endif

	for (j = 0; j < i; j++)
		dump_avail[j] = phys_avail[j];

	physmem = realmem;

	init_param1();
	init_param2(physmem);
	mips_cpu_init();
	pmap_bootstrap();
	mips_proc0_init();
	mutex_init();
	kdb_init();
#ifdef KDB
	if (boothowto & RB_KDB)
		kdb_enter(KDB_WHY_BOOTFLAGS, "Boot flags requested debugger");
#endif
}

void
platform_reset(void)
{
	struct bcm_platform	*bp;
	bool			 bcm4785war;

	printf("bcm::platform_reset()\n");
	intr_disable();

#ifdef CFE
	/* Fall back on CFE if reset requested during platform
	 * data initialization */
	if (!bcm_platform_data_avail) {
		cfe_exit(0, 0);
		while (1);
	}
#endif

	bp = bcm_get_platform();
	bcm4785war = false;

	/* Handle BCM4785-specific behavior */
	if (bp->cid.chip_id == BHND_CHIPID_BCM4785) {
		bcm4785war = true;

		/* Switch to async mode */
		bcm_mips_wr_pllcfg3(MIPS_BCMCFG_PLLCFG3_SM);
	}

	/* Set watchdog (PMU or ChipCommon) */
	if (bp->pmu_addr != 0x0) {
		BCM_PMU_WRITE_4(bp, BHND_PMU_WATCHDOG, 1);
	} else
		BCM_CHIPC_WRITE_4(bp, CHIPC_WATCHDOG, 1);

	/* BCM4785 */
	if (bcm4785war) {
		mips_sync();
		__asm __volatile("wait");
	}

	while (1);
}

void
platform_start(__register_t a0, __register_t a1, __register_t a2,
	       __register_t a3)
{
	vm_offset_t 		 kernend;
	uint64_t		 platform_counter_freq;
	int			 error;

	/* clear the BSS and SBSS segments */
	kernend = (vm_offset_t)&end;
	memset(&edata, 0, kernend - (vm_offset_t)(&edata));

	mips_postboot_fixup();

	/* Initialize pcpu stuff */
	mips_pcpu0_init();

#ifdef CFE
	/*
	 * Initialize CFE firmware trampolines. This must be done
	 * before any CFE APIs are called, including writing
	 * to the CFE console.
	 *
	 * CFE passes the following values in registers:
	 * a0: firmware handle
	 * a2: firmware entry point
	 * a3: entry point seal
	 */
	if (a3 == CFE_EPTSEAL)
		cfe_init(a0, a2);
#endif

	/* Init BCM platform data */
	if ((error = bcm_init_platform_data(&bcm_platform_data)))
		panic("bcm_init_platform_data() failed: %d", error);

	platform_counter_freq = bcm_get_cpufreq(bcm_get_platform());

	/* CP0 ticks every two cycles */
	mips_timer_early_init(platform_counter_freq / 2);

	cninit();

	mips_init();

	mips_timer_init_params(platform_counter_freq, 1);
}

/*
 * CFE-based EARLY_PRINTF support. To use, add the following to the kernel
 * config:
 *	option EARLY_PRINTF
 *	option CFE
 *	device cfe
 */
#if defined(EARLY_PRINTF) && defined(CFE)
static void
bcm_cfe_eputc(int c)
{
	unsigned char	ch;
	int		handle;

	ch = (unsigned char) c;

	/* bcm_get_platform() cannot be used here, as we may be called
	 * from bcm_init_platform_data(). */
	if ((handle = bcm_platform_data.cfe_console) < 0)
		return;

	if (ch == '\n')
		early_putc('\r');

	while ((cfe_write(handle, &ch, 1)) == 0)
		continue;
}

early_putc_t *early_putc = bcm_cfe_eputc;
#endif /* EARLY_PRINTF */
