/*-
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

#include <dev/bhnd/bhnd.h>

#include <dev/bhnd/cores/chipc/chipcreg.h>
#include <dev/bhnd/cores/chipc/pwrctl/bhnd_pwrctlvar.h>

#include <dev/bhnd/cores/pmu/bhnd_pmuvar.h>

#include "bcm_machdep.h"

static uint32_t	bcm_pmu_read4(bus_size_t reg, void *ctx);
static void	bcm_pmu_write4(bus_size_t reg, uint32_t val, void *ctx);
static uint32_t	bcm_pmu_read_chipst(void *ctx);

static const struct bhnd_pmu_io bcm_pmu_soc_io = {
	.rd4		= bcm_pmu_read4,
	.wr4		= bcm_pmu_write4,
	.rd_chipst	= bcm_pmu_read_chipst
};

static uint64_t
bcm_get_cpufreq_pwrctl(void)
{
	struct bcm_platform	*bp;
	uint8_t			 pll_type;
	bus_size_t		 mreg;
	uint32_t		 n, m;

	bp = bcm_get_platform();

	pll_type = CHIPC_GET_BITS(bp->cc_caps, CHIPC_CAP_PLL);
	mreg = bhnd_pwrctl_cpu_clkreg_m(pll_type);

	n = BCM_CHIPC_READ_4(CHIPC_CLKC_N);
	m = BCM_CHIPC_READ_4(mreg);

	return (bhnd_pwrctl_cpu_clock_rate(&bp->id, pll_type, n, m));
}

static uint64_t 
bcm_get_cpufreq_pmu(void)
{
	struct bhnd_pmu_query	pq;
	uint64_t		cpufreq;
	int			error;

	error = bhnd_pmu_query_init(&pq, NULL, bcm_get_platform()->id,
	    &bcm_pmu_soc_io, NULL);
	if (error)
		panic("%s: bhnd_pmu_query_init() failed: %d\n", __FUNCTION__,
		    error);

	cpufreq = bhnd_pmu_cpu_clock(&pq);

	bhnd_pmu_query_fini(&pq);

	return (cpufreq);
}

uint64_t
bcm_get_cpufreq(void)
{
	if (bcm_get_platform()->pmu_addr != 0x0)
		return (bcm_get_cpufreq_pmu());
	else
		return (bcm_get_cpufreq_pwrctl());
}

static uint32_t
bcm_pmu_read4(bus_size_t reg, void *ctx) {
	return (BCM_PMU_READ_4(reg));
}

static void
bcm_pmu_write4(bus_size_t reg, uint32_t val, void *ctx) {
	BCM_PMU_WRITE_4(reg, val);
}

static uint32_t
bcm_pmu_read_chipst(void *ctx)
{
	return (BCM_CHIPC_READ_4(CHIPC_CHIPST));
}
