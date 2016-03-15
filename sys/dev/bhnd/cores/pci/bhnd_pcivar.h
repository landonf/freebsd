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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_CORES_PCI_BHND_PCIVAR_H_
#define _BHND_CORES_PCI_BHND_PCIVAR_H_

#include <sys/param.h>
#include <sys/bus.h>

/*
 * Shared PCI Bridge/PCI Host Bridge definitions.
 */

DECLARE_CLASS(bhnd_pci_driver);
struct bhnd_pci_softc;

int		bhnd_pci_generic_probe(device_t dev);
int		bhnd_pci_generic_attach(device_t dev);
int		bhnd_pci_generic_detach(device_t dev);
int		bhnd_pci_generic_suspend(device_t dev);
int		bhnd_pci_generic_resume(device_t dev);

uint32_t	bhnd_pci_read_pcie_proto_reg(struct bhnd_pci_softc *sc,
		    uint32_t addr);
void		bhnd_pci_write_pcie_proto_reg(struct bhnd_pci_softc *sc,
		    uint32_t addr, uint32_t val);

/* Device register families. */
typedef enum {
	BHND_PCI_REGFMT_PCI	= 0,	/* PCI register definitions */
	BHND_PCI_REGFMT_PCIE	= 1,	/* PCIe-Gen1 register definitions */
} bhnd_pci_regfmt_t;

/**
 * bhnd_pci child device info
 */
struct bhnd_pci_devinfo {
	struct resource_list	resources;
};

/*
 * Generic PCI bridge/end-point driver state.
 * 
 * Must be first member of all subclass softc structures.
 */
struct bhnd_pci_softc {
	device_t		 dev;		/**< pci device */
	uint32_t		 quirks;	/**< quirk flags */
	bhnd_pci_regfmt_t	 regfmt;	/**< register format */	
	device_t		 mdio_dev;	/**< child mdio device (PCIe-only) */

	struct mtx		 mtx;		/**< state mutex */
	struct bhnd_resource	*mem_res;	/**< device register block. */
	int			 mem_rid;	/**< register block RID */
	struct rman		 mem_rman;	/**< memory manager */
};


#define	BHND_PCI_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev), \
	    "BHND PCI driver lock", MTX_DEF)
#define	BHND_PCI_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	BHND_PCI_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	BHND_PCI_LOCK_ASSERT(sc, what)	mtx_assert(&(sc)->mtx, what)
#define	BHND_PCI_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->mtx)

/* Common BHND_PCI_*_REG_(EXTRACT|INSERT) implementation */
#define	_BHND_PCI_REG_EXTRACT(_regval, _mask, _shift)		\
	((_regval & _mask) >> _shift)
#define _BHND_PCI_REG_INSERT(_regval, _mask, _shift, _setval)	\
	(((_regval) & ~ _mask) | (((_setval) << _shift) & _mask))

/**
 * Extract a register value by applying _MASK and _SHIFT defines.
 * 
 * @param _regv The register value containing the desired attribute
 * @param _attr The register attribute name to which to append `_MASK`/`_SHIFT`
 * suffixes.
 */
#define	BHND_PCI_REG_EXTRACT(_regv, _attr)	\
	_BHND_PCI_REG_EXTRACT(_regv, _attr ## _MASK, _attr ## _SHIFT)

/**
 * Insert a value in @p _regv by applying _MASK and _SHIFT defines.
 * 
 * @param _regv The current register value.
 * @param _attr The register attribute name to which to append `_MASK`/`_SHIFT`
 * suffixes.
 * @param _val The value to be set in @p _regv.
 */
#define	BHND_PCI_REG_INSERT(_regv, _attr, _val)		\
	_BHND_PCI_REG_INSERT(_regv, _attr ## _MASK, _attr ## _SHIFT, _val)

/**
 * Extract a value by applying _MASK and _SHIFT defines to the common
 * PCI/PCIe register definition @p _regv
 * 
 * @param _regf The PCI core register format (BHNDB_PCI_REGFMT_*).
 * @param _regv The register value containing the desired attribute
 * @param _attr The register attribute name to which to prepend the register
 * definition prefix and append `_MASK`/`_SHIFT` suffixes.
 */
#define BHND_PCI_COMMON_REG_EXTRACT(_regf, _regv, _attr)		\
	_BHND_PCI_REG_EXTRACT(_regv,				\
	    BHND_PCI_COMMON_REG((_regf), _attr ## _MASK),	\
	    BHND_PCI_COMMON_REG((_regf), _attr ## _SHIFT))

/**
 * Insert a register value by applying _MASK and _SHIFT defines to the common
 * PCI/PCIe register definition @p _regv
 * 
 * @param _regf The PCI core register format (BHNDB_PCI_REGFMT_*).
 * @param _regv The register value containing the desired attribute
 * @param _attr The register attribute name to which to prepend the register
 * definition prefix and append `_MASK`/`_SHIFT` suffixes.
 * @param _val The value to bet set in @p _regv.
 */
#define BHND_PCI_COMMON_REG_INSERT(_regf, _regv, _attr, _val)	\
	_BHND_PCI_REG_INSERT(_regv,				\
	    BHND_PCI_COMMON_REG((_regf), _attr ## _MASK),	\
	    BHND_PCI_COMMON_REG((_regf), _attr ## _SHIFT),	\
	    _val)


/**
 * Evaluates to the offset of a common PCI/PCIe register definition. 
 * 
 * This will trigger a compile-time error if the register is not defined
 * for all supported PCI/PCIe cores.
 * 
 * This should be optimized down to a constant value if the register constant
 * is the same across the register definitions.
 * 
 * @param _regf The PCI core register format (BHNDB_PCI_REGFMT_*).
 * @param _name The base name of the register.
 */
#define	BHND_PCI_COMMON_REG(_regf, _name)	(			\
	(_regf) == BHND_PCI_REGFMT_PCI ? BHND_PCI_ ## _name :	\
	BHND_PCIE_ ## _name						\
)

#endif /* _BHND_CORES_PCI_BHND_PCIVAR_H_ */