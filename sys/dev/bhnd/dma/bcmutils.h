/*
 * Misc useful os-independent macros and functions.
 *
 * Copyright (C) 2015, Broadcom Corporation. All Rights Reserved.
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
 *
 * $Id: bcmutils.h 422014 2013-09-05 14:46:51Z $
 */

#ifndef	_bcmutils_h_
#define	_bcmutils_h_

#if defined(UNDER_CE)
#include <bcmsafestr.h>
#else
#define bcm_strcpy_s(dst, noOfElements, src)            strcpy((dst), (src))
#define bcm_strncpy_s(dst, noOfElements, src, count)    strncpy((dst), (src), (count))
#define bcm_strcat_s(dst, noOfElements, src)            strcat((dst), (src))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef PKTQ_LOG
#include <wlioctl.h>
#endif

/* ctype replacement */
#define _BCM_U	0x01	/* upper */
#define _BCM_L	0x02	/* lower */
#define _BCM_D	0x04	/* digit */
#define _BCM_C	0x08	/* cntrl */
#define _BCM_P	0x10	/* punct */
#define _BCM_S	0x20	/* white space (space/lf/tab) */
#define _BCM_X	0x40	/* hex digit */
#define _BCM_SP	0x80	/* hard space (0x20) */

#if defined(BCMROMBUILD)
extern const unsigned char BCMROMDATA(bcm_ctype)[];
#else
extern const unsigned char bcm_ctype[];
#endif
#define bcm_ismask(x)	(bcm_ctype[(int)(unsigned char)(x)])

#define bcm_isalnum(c)	((bcm_ismask(c)&(_BCM_U|_BCM_L|_BCM_D)) != 0)
#define bcm_isalpha(c)	((bcm_ismask(c)&(_BCM_U|_BCM_L)) != 0)
#define bcm_iscntrl(c)	((bcm_ismask(c)&(_BCM_C)) != 0)
#define bcm_isdigit(c)	((bcm_ismask(c)&(_BCM_D)) != 0)
#define bcm_isgraph(c)	((bcm_ismask(c)&(_BCM_P|_BCM_U|_BCM_L|_BCM_D)) != 0)
#define bcm_islower(c)	((bcm_ismask(c)&(_BCM_L)) != 0)
#define bcm_isprint(c)	((bcm_ismask(c)&(_BCM_P|_BCM_U|_BCM_L|_BCM_D|_BCM_SP)) != 0)
#define bcm_ispunct(c)	((bcm_ismask(c)&(_BCM_P)) != 0)
#define bcm_isspace(c)	((bcm_ismask(c)&(_BCM_S)) != 0)
#define bcm_isupper(c)	((bcm_ismask(c)&(_BCM_U)) != 0)
#define bcm_isxdigit(c)	((bcm_ismask(c)&(_BCM_D|_BCM_X)) != 0)
#define bcm_tolower(c)	(bcm_isupper((c)) ? ((c) + 'a' - 'A') : (c))
#define bcm_toupper(c)	(bcm_islower((c)) ? ((c) + 'A' - 'a') : (c))

/* Buffer structure for collecting string-formatted data
* using bcm_bprintf() API.
* Use bcm_binit() to initialize before use
*/

struct bcmstrbuf {
	char *buf;	/* pointer to current position in origbuf */
	unsigned int size;	/* current (residual) size in bytes */
	char *origbuf;	/* unmodified pointer to orignal buffer */
	unsigned int origsize;	/* unmodified orignal buffer size in bytes */
};

/* ** driver-only section ** */
#ifdef BCMDRIVER
#ifdef EFI
/* forward declare structyre type */
struct spktq;
#endif
#include "osl.h"

#define GPIO_PIN_NOTDEFINED 	0x20	/* Pin not defined */

/*
 * Spin at most 'us' microseconds while 'exp' is true.
 * Caller should explicitly test 'exp' when this completes
 * and take appropriate error action if 'exp' is still true.
 */
#ifdef MACOSX
#define SPINWAIT_POLL_PERIOD	20
#else
#define SPINWAIT_POLL_PERIOD	10
#endif

#define SPINWAIT(exp, us) { \
	u_int countdown = (us) + (SPINWAIT_POLL_PERIOD - 1); \
	while ((exp) && (countdown >= SPINWAIT_POLL_PERIOD)) {\
		OSL_DELAY(SPINWAIT_POLL_PERIOD); \
		countdown -= SPINWAIT_POLL_PERIOD; \
	} \
}

/* osl multi-precedence packet queue */
#define PKTQ_LEN_MAX            0xFFFF  /* Max uint16_t 65535 packets */
#ifndef PKTQ_LEN_DEFAULT
#define PKTQ_LEN_DEFAULT        128	/* Max 128 packets */
#endif
#ifndef PKTQ_MAX_PREC
#define PKTQ_MAX_PREC           16	/* Maximum precedence levels */
#endif

typedef struct pktq_prec {
	void *head;     /* first packet to dequeue */
	void *tail;     /* last packet to dequeue */
	uint16_t len;     /* number of queued packets */
	uint16_t max;     /* maximum number of queued packets */
} pktq_prec_t;

#ifdef PKTQ_LOG
typedef struct {
	uint32_t requested;    /* packets requested to be stored */
	uint32_t stored;	     /* packets stored */
	uint32_t saved;	     /* packets saved,
	                            because a lowest priority queue has given away one packet
	                      */
	uint32_t selfsaved;    /* packets saved,
	                            because an older packet from the same queue has been dropped
	                      */
	uint32_t full_dropped; /* packets dropped,
	                            because pktq is full with higher precedence packets
	                      */
	uint32_t dropped;      /* packets dropped because pktq per that precedence is full */
	uint32_t sacrificed;   /* packets dropped,
	                            in order to save one from a queue of a highest priority
	                      */
	uint32_t busy;         /* packets droped because of hardware/transmission error */
	uint32_t retry;        /* packets re-sent because they were not received */
	uint32_t ps_retry;     /* packets retried again prior to moving power save mode */
	uint32_t suppress;     /* packets which were suppressed and not transmitted */
	uint32_t retry_drop;   /* packets finally dropped after retry limit */
	uint32_t max_avail;    /* the high-water mark of the queue capacity for packets -
	                            goes to zero as queue fills
	                      */
	uint32_t max_used;     /* the high-water mark of the queue utilisation for packets -
						        increases with use ('inverse' of max_avail)
				          */
	uint32_t queue_capacity; /* the maximum capacity of the queue */
	uint32_t rtsfail;        /* count of rts attempts that failed to receive cts */
	uint32_t acked;          /* count of packets sent (acked) successfully */
	uint32_t txrate_succ;    /* running total of phy rate of packets sent successfully */
	uint32_t txrate_main;    /* running totoal of primary phy rate of all packets */
	uint32_t throughput;     /* actual data transferred successfully */
	uint32_t airtime;        /* cumulative total medium access delay in useconds */
	uint32_t  _logtime;      /* timestamp of last counter clear  */
} pktq_counters_t;

typedef struct {
	uint32_t                  _prec_log;
	pktq_counters_t*        _prec_cnt[PKTQ_MAX_PREC];     /* Counters per queue  */
#ifdef BCMDBG
	uint32_t pps_time;        /* time spent in ps pretend state */
#endif
} pktq_log_t;
#endif /* PKTQ_LOG */


#define PKTQ_COMMON	\
	uint16_t num_prec;        /* number of precedences in use */			\
	uint16_t hi_prec;         /* rapid dequeue hint (>= highest non-empty prec) */	\
	uint16_t max;             /* total max packets */					\
	uint16_t len;             /* total number of packets */

/* multi-priority pkt queue */
struct pktq {
	PKTQ_COMMON
	/* q array must be last since # of elements can be either PKTQ_MAX_PREC or 1 */
	struct pktq_prec q[PKTQ_MAX_PREC];
#ifdef PKTQ_LOG
	pktq_log_t*      pktqlog;
#endif
};

/* simple, non-priority pkt queue */
struct spktq {
	PKTQ_COMMON
	/* q array must be last since # of elements can be either PKTQ_MAX_PREC or 1 */
	struct pktq_prec q[1];
};

#define PKTQ_PREC_ITER(pq, prec)        for (prec = (pq)->num_prec - 1; prec >= 0; prec--)

/* fn(pkt, arg).  return true if pkt belongs to if */
typedef bool (*ifpkt_cb_t)(void*, int);

#ifdef BCMPKTPOOL
#define POOL_ENAB(pool)		((pool) && (pool)->inited)
#if defined(BCM4329C0)
#define SHARED_POOL		(pktpool_shared_ptr)
#else
#define SHARED_POOL		(pktpool_shared)
#endif /* BCM4329C0 */
#else /* BCMPKTPOOL */
#define POOL_ENAB(bus)		0
#define SHARED_POOL		((struct pktpool *)NULL)
#endif /* BCMPKTPOOL */

#ifndef PKTPOOL_LEN_MAX
#define PKTPOOL_LEN_MAX		40
#endif /* PKTPOOL_LEN_MAX */
#define PKTPOOL_CB_MAX		3

struct pktpool;
typedef void (*pktpool_cb_t)(struct pktpool *pool, void *arg);
typedef struct {
	pktpool_cb_t cb;
	void *arg;
} pktpool_cbinfo_t;

#ifdef BCMDBG_POOL
/* pkt pool debug states */
#define POOL_IDLE	0
#define POOL_RXFILL	1
#define POOL_RXDH	2
#define POOL_RXD11	3
#define POOL_TXDH	4
#define POOL_TXD11	5
#define POOL_AMPDU	6
#define POOL_TXENQ	7

typedef struct {
	void *p;
	uint32_t cycles;
	uint32_t dur;
} pktpool_dbg_t;

typedef struct {
	uint8_t txdh;	/* tx to host */
	uint8_t txd11;	/* tx to d11 */
	uint8_t enq;	/* waiting in q */
	uint8_t rxdh;	/* rx from host */
	uint8_t rxd11;	/* rx from d11 */
	uint8_t rxfill;	/* dma_rxfill */
	uint8_t idle;	/* avail in pool */
} pktpool_stats_t;
#endif /* BCMDBG_POOL */

typedef struct pktpool {
	bool inited;
	uint16_t r;
	uint16_t w;
	uint16_t len;
	uint16_t maxlen;
	uint16_t plen;
	bool istx;
	bool empty;
	uint8_t cbtoggle;
	uint8_t cbcnt;
	uint8_t ecbcnt;
	bool emptycb_disable;
	pktpool_cbinfo_t *availcb_excl;
	pktpool_cbinfo_t cbs[PKTPOOL_CB_MAX];
	pktpool_cbinfo_t ecbs[PKTPOOL_CB_MAX];
	void *q[PKTPOOL_LEN_MAX + 1];

#ifdef BCMDBG_POOL
	uint8_t dbg_cbcnt;
	pktpool_cbinfo_t dbg_cbs[PKTPOOL_CB_MAX];
	uint16_t dbg_qlen;
	pktpool_dbg_t dbg_q[PKTPOOL_LEN_MAX + 1];
#endif
} pktpool_t;

#if defined(BCM4329C0)
extern pktpool_t *pktpool_shared_ptr;
#else
extern pktpool_t *pktpool_shared;
#endif /* BCM4329C0 */

extern int pktpool_init(osl_t *osh, pktpool_t *pktp, int *pktplen, int plen, bool istx);
extern int pktpool_deinit(osl_t *osh, pktpool_t *pktp);
extern int pktpool_fill(osl_t *osh, pktpool_t *pktp, bool minimal);
extern void* pktpool_get(pktpool_t *pktp);
extern void pktpool_free(pktpool_t *pktp, void *p);
extern int pktpool_add(pktpool_t *pktp, void *p);
extern uint16_t pktpool_avail(pktpool_t *pktp);
extern int pktpool_avail_notify_normal(osl_t *osh, pktpool_t *pktp);
extern int pktpool_avail_notify_exclusive(osl_t *osh, pktpool_t *pktp, pktpool_cb_t cb);
extern int pktpool_avail_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg);
extern int pktpool_empty_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg);
extern int pktpool_setmaxlen(pktpool_t *pktp, uint16_t maxlen);
extern int pktpool_setmaxlen_strict(osl_t *osh, pktpool_t *pktp, uint16_t maxlen);
extern void pktpool_emptycb_disable(pktpool_t *pktp, bool disable);
extern bool pktpool_emptycb_disabled(pktpool_t *pktp);

#define POOLPTR(pp)			((pktpool_t *)(pp))
#define pktpool_len(pp)			(POOLPTR(pp)->len - 1)
#define pktpool_plen(pp)		(POOLPTR(pp)->plen)
#define pktpool_maxlen(pp)		(POOLPTR(pp)->maxlen)

#ifdef BCMDBG_POOL
extern int pktpool_dbg_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg);
extern int pktpool_start_trigger(pktpool_t *pktp, void *p);
extern int pktpool_dbg_dump(pktpool_t *pktp);
extern int pktpool_dbg_notify(pktpool_t *pktp);
extern int pktpool_stats_dump(pktpool_t *pktp, pktpool_stats_t *stats);
#endif /* BCMDBG_POOL */

/* forward definition of ether_addr structure used by some function prototypes */

struct ether_addr;

extern int ether_isbcast(const void *ea);
extern int ether_isnulladdr(const void *ea);

/* operations on a specific precedence in packet queue */

#define pktq_psetmax(pq, prec, _max)	((pq)->q[prec].max = (_max))
#define pktq_pmax(pq, prec)		((pq)->q[prec].max)
#define pktq_plen(pq, prec)		((pq)->q[prec].len)
#define pktq_pavail(pq, prec)		((pq)->q[prec].max - (pq)->q[prec].len)
#define pktq_pfull(pq, prec)		((pq)->q[prec].len >= (pq)->q[prec].max)
#define pktq_pempty(pq, prec)		((pq)->q[prec].len == 0)

#define pktq_ppeek(pq, prec)		((pq)->q[prec].head)
#define pktq_ppeek_tail(pq, prec)	((pq)->q[prec].tail)

extern void *pktq_penq(struct pktq *pq, int prec, void *p);
extern void *pktq_penq_head(struct pktq *pq, int prec, void *p);
extern void *pktq_pdeq(struct pktq *pq, int prec);
extern void *pktq_pdeq_prev(struct pktq *pq, int prec, void *prev_p);
extern void *pktq_pdeq_tail(struct pktq *pq, int prec);
/* Empty the queue at particular precedence level */
extern void pktq_pflush(osl_t *osh, struct pktq *pq, int prec, bool dir,
	ifpkt_cb_t fn, int arg);
/* Remove a specified packet from its queue */
extern bool pktq_pdel(struct pktq *pq, void *p, int prec);

/* operations on a set of precedences in packet queue */

extern int pktq_mlen(struct pktq *pq, u_int prec_bmp);
extern void *pktq_mdeq(struct pktq *pq, u_int prec_bmp, int *prec_out);
extern void *pktq_mpeek(struct pktq *pq, u_int prec_bmp, int *prec_out);

/* operations on packet queue as a whole */

#define pktq_len(pq)		((int)(pq)->len)
#define pktq_max(pq)		((int)(pq)->max)
#define pktq_avail(pq)		((int)((pq)->max - (pq)->len))
#define pktq_full(pq)		((pq)->len >= (pq)->max)
#define pktq_empty(pq)		((pq)->len == 0)

/* operations for single precedence queues */
#define pktenq(pq, p)		pktq_penq(((struct pktq *)(void *)pq), 0, (p))
#define pktenq_head(pq, p)	pktq_penq_head(((struct pktq *)(void *)pq), 0, (p))
#define pktdeq(pq)		pktq_pdeq(((struct pktq *)(void *)pq), 0)
#define pktdeq_tail(pq)		pktq_pdeq_tail(((struct pktq *)(void *)pq), 0)
#define pktqinit(pq, len)	pktq_init(((struct pktq *)(void *)pq), 1, len)

extern void pktq_init(struct pktq *pq, int num_prec, int max_len);
extern void pktq_set_max_plen(struct pktq *pq, int prec, int max_len);

/* prec_out may be NULL if caller is not interested in return value */
extern void *pktq_deq(struct pktq *pq, int *prec_out);
extern void *pktq_deq_tail(struct pktq *pq, int *prec_out);
extern void *pktq_peek(struct pktq *pq, int *prec_out);
extern void *pktq_peek_tail(struct pktq *pq, int *prec_out);
extern void pktq_flush(osl_t *osh, struct pktq *pq, bool dir, ifpkt_cb_t fn, int arg);

/* externs */
/* packet */
extern u_int pktcopy(osl_t *osh, void *p, u_int offset, int len, u_char *buf);
extern u_int pktfrombuf(osl_t *osh, void *p, u_int offset, int len, u_char *buf);
extern u_int pkttotlen(osl_t *osh, void *p);
extern void *pktlast(osl_t *osh, void *p);
extern u_int pktsegcnt(osl_t *osh, void *p);
extern u_int pktsegcnt_war(osl_t *osh, void *p);
extern uint8_t *pktdataoffset(osl_t *osh, void *p,  u_int offset);
extern void *pktoffset(osl_t *osh, void *p,  u_int offset);

/* Get priority from a packet and pass it back in scb (or equiv) */
#define	PKTPRIO_VDSCP	0x100		/* DSCP prio found after VLAN tag */
#define	PKTPRIO_VLAN	0x200		/* VLAN prio found */
#define	PKTPRIO_UPD	0x400		/* DSCP used to update VLAN prio */
#define	PKTPRIO_DSCP	0x800		/* DSCP prio found */

extern u_int pktsetprio(void *pkt, bool update_vtag);

/* string */
extern int BCMROMFN(bcm_atoi)(const char *s);
extern u_long BCMROMFN(bcm_strtoul)(const char *cp, char **endp, u_int base);
extern char *BCMROMFN(bcmstrstr)(const char *haystack, const char *needle);
extern char *BCMROMFN(bcmstrcat)(char *dest, const char *src);
extern char *BCMROMFN(bcmstrncat)(char *dest, const char *src, u_int size);
extern u_long wchar2ascii(char *abuf, u_short *wbuf, u_short wbuflen, u_long abuflen);
char* bcmstrtok(char **string, const char *delimiters, char *tokdelim);
int bcmstricmp(const char *s1, const char *s2);
int bcmstrnicmp(const char* s1, const char* s2, int cnt);


/* ethernet address */
extern char *bcm_ether_ntoa(const struct ether_addr *ea, char *buf);
extern int BCMROMFN(bcm_ether_atoe)(const char *p, struct ether_addr *ea);

/* ip address */
struct ipv4_addr;
extern char *bcm_ip_ntoa(struct ipv4_addr *ia, char *buf);
extern char *bcm_ipv6_ntoa(void *ipv6, char *buf);

/* delay */
extern void bcm_mdelay(u_int ms);
/* variable access */
#if defined(DONGLEBUILD) && !defined(WLTEST)
#ifdef BCMDBG
#define NVRAM_RECLAIM_CHECK(name)							\
	if (attach_part_reclaimed == TRUE) {						\
		printf("%s: NVRAM already reclaimed, %s\n", __FUNCTION__, (name));	\
		*(char*) 0 = 0; /* TRAP */						\
		return NULL;								\
	}
#else /* BCMDBG */
#define NVRAM_RECLAIM_CHECK(name)							\
	if (attach_part_reclaimed == TRUE) {						\
		*(char*) 0 = 0; /* TRAP */						\
		return NULL;								\
	}
#endif /* BCMDBG */
#else /* DONGLEBUILD && !WLTEST && !BCMINTERNAL && !BCMDBG_DUMP */
#define NVRAM_RECLAIM_CHECK(name)
#endif 

extern char *getvar(char *vars, const char *name);
extern int getintvar(char *vars, const char *name);
extern int getintvararray(char *vars, const char *name, int index);
extern int getintvararraysize(char *vars, const char *name);
extern u_int getgpiopin(char *vars, char *pin_name, u_int def_pin);
#ifdef BCMDBG
extern void prpkt(const char *msg, osl_t *osh, void *p0);
#endif /* BCMDBG */
#ifdef BCMPERFSTATS
extern void bcm_perf_enable(void);
extern void bcmstats(char *fmt);
extern void bcmlog(char *fmt, u_int a1, u_int a2);
extern void bcmdumplog(char *buf, int size);
extern int bcmdumplogent(char *buf, u_int idx);
#else
#define bcm_perf_enable()
#define bcmstats(fmt)
#define	bcmlog(fmt, a1, a2)
#define	bcmdumplog(buf, size)	*buf = '\0'
#define	bcmdumplogent(buf, idx)	-1
#endif /* BCMPERFSTATS */

#define TSF_TICKS_PER_MS	1024
#if defined(BCMTSTAMPEDLOGS)
/* Store a TSF timestamp and a log line in the log buffer */
extern void bcmtslog(uint32_t tstamp, char *fmt, u_int a1, u_int a2);
/* Print out the log buffer with timestamps */
extern void bcmprinttslogs(void);
/* Print out a microsecond timestamp as "sec.ms.us " */
extern void bcmprinttstamp(uint32_t us);
/* Dump to buffer a microsecond timestamp as "sec.ms.us " */
extern void bcmdumptslog(char *buf, int size);
#else
#define bcmtslog(tstamp, fmt, a1, a2)
#define bcmprinttslogs()
#define bcmprinttstamp(us)
#define bcmdumptslog(buf, size)
#endif /* BCMTSTAMPEDLOGS */

extern char *bcm_nvram_vars(u_int *length);
extern int bcm_nvram_cache(void *sih);

/* Support for sharing code across in-driver iovar implementations.
 * The intent is that a driver use this structure to map iovar names
 * to its (private) iovar identifiers, and the lookup function to
 * find the entry.  Macros are provided to map ids and get/set actions
 * into a single number space for a switch statement.
 */

/* iovar structure */
typedef struct bcm_iovar {
	const char *name;	/* name for lookup and display */
	uint16_t varid;		/* id for switch */
	uint16_t flags;		/* driver-specific flag bits */
	uint16_t type;		/* base type of argument */
	uint16_t minlen;		/* min length for buffer vars */
} bcm_iovar_t;

/* varid definitions are per-driver, may use these get/set bits */

/* IOVar action bits for id mapping */
#define IOV_GET 0 /* Get an iovar */
#define IOV_SET 1 /* Set an iovar */

/* Varid to actionid mapping */
#define IOV_GVAL(id)		((id) * 2)
#define IOV_SVAL(id)		((id) * 2 + IOV_SET)
#define IOV_ISSET(actionid)	((actionid & IOV_SET) == IOV_SET)
#define IOV_ID(actionid)	(actionid >> 1)

/* flags are per-driver based on driver attributes */

extern const bcm_iovar_t *bcm_iovar_lookup(const bcm_iovar_t *table, const char *name);
extern int bcm_iovar_lencheck(const bcm_iovar_t *table, void *arg, int len, bool set);
#if defined(WLTINYDUMP) || defined(BCMDBG) || defined(WLMSG_INFORM) || \
	defined(WLMSG_ASSOC) || defined(WLMSG_PRPKT) || defined(WLMSG_WSEC)
extern int bcm_format_ssid(char* buf, const u_char ssid[], u_int ssid_len);
#endif /* WLTINYDUMP || BCMDBG || WLMSG_INFORM || WLMSG_ASSOC || WLMSG_PRPKT */
#endif	/* BCMDRIVER */

/* Base type definitions */
#define IOVT_VOID	0	/* no value (implictly set only) */
#define IOVT_BOOL	1	/* any value ok (zero/nonzero) */
#define IOVT_INT8	2	/* integer values are range-checked */
#define IOVT_UINT8	3	/* unsigned int 8 bits */
#define IOVT_INT16	4	/* int 16 bits */
#define IOVT_UINT16	5	/* unsigned int 16 bits */
#define IOVT_INT32	6	/* int 32 bits */
#define IOVT_UINT32	7	/* unsigned int 32 bits */
#define IOVT_BUFFER	8	/* buffer is size-checked as per minlen */
#define BCM_IOVT_VALID(type) (((unsigned int)(type)) <= IOVT_BUFFER)

/* Initializer for IOV type strings */
#define BCM_IOV_TYPE_INIT { \
	"void", \
	"bool", \
	"int8_t", \
	"uint8_t", \
	"int16_t", \
	"uint16_t", \
	"int32_t", \
	"uint32_t", \
	"buffer", \
	"" }

#define BCM_IOVT_IS_INT(type) (\
	(type == IOVT_BOOL) || \
	(type == IOVT_INT8) || \
	(type == IOVT_UINT8) || \
	(type == IOVT_INT16) || \
	(type == IOVT_UINT16) || \
	(type == IOVT_INT32) || \
	(type == IOVT_UINT32))

/* ** driver/apps-shared section ** */

#define BCME_STRLEN 		64	/* Max string length for BCM errors */
#define VALID_BCMERROR(e)  ((e <= 0) && (e >= BCME_LAST))


/*
 * error codes could be added but the defined ones shouldn't be changed/deleted
 * these error codes are exposed to the user code
 * when ever a new error code is added to this list
 * please update errorstring table with the related error string and
 * update osl files with os specific errorcode map
*/

#define BCME_OK				0	/* Success */
#define BCME_ERROR			-1	/* Error generic */
#define BCME_BADARG			-2	/* Bad Argument */
#define BCME_BADOPTION			-3	/* Bad option */
#define BCME_NOTUP			-4	/* Not up */
#define BCME_NOTDOWN			-5	/* Not down */
#define BCME_NOTAP			-6	/* Not AP */
#define BCME_NOTSTA			-7	/* Not STA  */
#define BCME_BADKEYIDX			-8	/* BAD Key Index */
#define BCME_RADIOOFF 			-9	/* Radio Off */
#define BCME_NOTBANDLOCKED		-10	/* Not  band locked */
#define BCME_NOCLK			-11	/* No Clock */
#define BCME_BADRATESET			-12	/* BAD Rate valueset */
#define BCME_BADBAND			-13	/* BAD Band */
#define BCME_BUFTOOSHORT		-14	/* Buffer too short */
#define BCME_BUFTOOLONG			-15	/* Buffer too long */
#define BCME_BUSY			-16	/* Busy */
#define BCME_NOTASSOCIATED		-17	/* Not Associated */
#define BCME_BADSSIDLEN			-18	/* Bad SSID len */
#define BCME_OUTOFRANGECHAN		-19	/* Out of Range Channel */
#define BCME_BADCHAN			-20	/* Bad Channel */
#define BCME_BADADDR			-21	/* Bad Address */
#define BCME_NORESOURCE			-22	/* Not Enough Resources */
#define BCME_UNSUPPORTED		-23	/* Unsupported */
#define BCME_BADLEN			-24	/* Bad length */
#define BCME_NOTREADY			-25	/* Not Ready */
#define BCME_EPERM			-26	/* Not Permitted */
#define BCME_NOMEM			-27	/* No Memory */
#define BCME_ASSOCIATED			-28	/* Associated */
#define BCME_RANGE			-29	/* Not In Range */
#define BCME_NOTFOUND			-30	/* Not Found */
#define BCME_WME_NOT_ENABLED		-31	/* WME Not Enabled */
#define BCME_TSPEC_NOTFOUND		-32	/* TSPEC Not Found */
#define BCME_ACM_NOTSUPPORTED		-33	/* ACM Not Supported */
#define BCME_NOT_WME_ASSOCIATION	-34	/* Not WME Association */
#define BCME_SDIO_ERROR			-35	/* SDIO Bus Error */
#define BCME_DONGLE_DOWN		-36	/* Dongle Not Accessible */
#define BCME_VERSION			-37 	/* Incorrect version */
#define BCME_TXFAIL			-38 	/* TX failure */
#define BCME_RXFAIL			-39	/* RX failure */
#define BCME_NODEVICE			-40 	/* Device not present */
#define BCME_NMODE_DISABLED		-41 	/* NMODE disabled */
#define BCME_NONRESIDENT		-42 /* access to nonresident overlay */
#define BCME_SCANREJECT			-43 	/* reject scan request */
/* Leave gap between -44 and -46 to synchronize with trunk. */
#define BCME_DISABLED                   -47     /* Disabled in this build */
#define BCME_LAST			BCME_DISABLED

/* These are collection of BCME Error strings */
#define BCMERRSTRINGTABLE {		\
	"OK",				\
	"Undefined error",		\
	"Bad Argument",			\
	"Bad Option",			\
	"Not up",			\
	"Not down",			\
	"Not AP",			\
	"Not STA",			\
	"Bad Key Index",		\
	"Radio Off",			\
	"Not band locked",		\
	"No clock",			\
	"Bad Rate valueset",		\
	"Bad Band",			\
	"Buffer too short",		\
	"Buffer too long",		\
	"Busy",				\
	"Not Associated",		\
	"Bad SSID len",			\
	"Out of Range Channel",		\
	"Bad Channel",			\
	"Bad Address",			\
	"Not Enough Resources",		\
	"Unsupported",			\
	"Bad length",			\
	"Not Ready",			\
	"Not Permitted",		\
	"No Memory",			\
	"Associated",			\
	"Not In Range",			\
	"Not Found",			\
	"WME Not Enabled",		\
	"TSPEC Not Found",		\
	"ACM Not Supported",		\
	"Not WME Association",		\
	"SDIO Bus Error",		\
	"Dongle Not Accessible",	\
	"Incorrect version",		\
	"TX Failure",			\
	"RX Failure",			\
	"Device Not Present",		\
	"NMODE Disabled",		\
	"Nonresident overlay access", \
	"Scan Rejected",		\
	"unused",			\
	"unused",			\
	"unused",			\
	"Disabled",			\
}

#ifndef ABS
#define	ABS(a)			(((a) < 0) ? -(a) : (a))
#endif /* ABS */

#ifndef MIN
#define	MIN(a, b)		(((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define	MAX(a, b)		(((a) > (b)) ? (a) : (b))
#endif /* MAX */

/* limit to [min, max] */
#ifndef LIMIT_TO_RANGE
#define LIMIT_TO_RANGE(x, min, max) \
	((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif /* LIMIT_TO_RANGE */

/* limit to  max */
#ifndef LIMIT_TO_MAX
#define LIMIT_TO_MAX(x, max) \
	(((x) > (max) ? (max) : (x)))
#endif /* LIMIT_TO_MAX */

/* limit to min */
#ifndef LIMIT_TO_MIN
#define LIMIT_TO_MIN(x, min) \
	(((x) < (min) ? (min) : (x)))
#endif /* LIMIT_TO_MIN */

#define CEIL(x, y)		(((x) + ((y) - 1)) / (y))
#define	ROUNDUP(x, y)		((((x) + ((y) - 1)) / (y)) * (y))
#define	ISALIGNED(a, x)		(((uintptr_t)(a) & ((x) - 1)) == 0)
#define ALIGN_ADDR(addr, boundary) (void *)(((uintptr_t)(addr) + (boundary) - 1) \
	                                         & ~((boundary) - 1))
#define ALIGN_SIZE(size, boundary) (((size) + (boundary) - 1) \
	                                         & ~((boundary) - 1))
#define	ISPOWEROF2(x)		((((x) - 1) & (x)) == 0)
#define VALID_MASK(mask)	!((mask) & ((mask) + 1))

#ifndef OFFSETOF
#ifdef __ARMCC_VERSION
/*
 * The ARM RVCT compiler complains when using OFFSETOF where a constant
 * expression is expected, such as an initializer for a static object.
 * offsetof from the runtime library doesn't have that problem.
 */
#include <stddef.h>
#define	OFFSETOF(type, member)	offsetof(type, member)
#elif __GNUC__ >= 4
/* New versions of GCC are also complaining if the usual macro is used */
#define OFFSETOF(type, member)  __builtin_offsetof(type, member)
#else
#define	OFFSETOF(type, member)	((u_int)(uintptr)&((type *)0)->member)
#endif /* __ARMCC_VERSION */
#endif /* OFFSETOF */

#ifndef ARRAYSIZE
#define ARRAYSIZE(a)		(sizeof(a) / sizeof(a[0]))
#endif

/* Reference a function; used to prevent a static function from being optimized out */
extern void *_bcmutils_dummy_fn;
#define REFERENCE_FUNCTION(f)	(_bcmutils_dummy_fn = (void *)(f))

#if defined(__NetBSD__)
/* use internal
 * setbit/clrbit since it has a cast and netbsd Xbit funcs dont
 * and the wl driver doesnt cast.  this results in us offsetting
 * incorrectly and corrupting memory.
 */
#ifdef setbit
#undef setbit
#undef clrbit
#undef isset
#undef isclr
#undef NBBY
#endif
#endif /* __NetBSD__ */

/* bit map related macros */
#ifndef setbit
#ifndef NBBY		/* the BSD family defines NBBY */
#define	NBBY	8	/* 8 bits per byte */
#endif /* #ifndef NBBY */
#define	setbit(a, i)	(((uint8_t *)a)[(i) / NBBY] |= 1 << ((i) % NBBY))
#define	clrbit(a, i)	(((uint8_t *)a)[(i) / NBBY] &= ~(1 << ((i) % NBBY)))
#define	isset(a, i)	(((const uint8_t *)a)[(i) / NBBY] & (1 << ((i) % NBBY)))
#define	isclr(a, i)	((((const uint8_t *)a)[(i) / NBBY] & (1 << ((i) % NBBY))) == 0)
#endif /* setbit */

#define	isbitset(a, i)	(((a) & (1 << (i))) != 0)

#define	NBITS(type)	(sizeof(type) * 8)
#define NBITVAL(nbits)	(1 << (nbits))
#define MAXBITVAL(nbits)	((1 << (nbits)) - 1)
#define	NBITMASK(nbits)	MAXBITVAL(nbits)
#define MAXNBVAL(nbyte)	MAXBITVAL((nbyte) * 8)

/* basic mux operation - can be optimized on several architectures */
#define MUX(pred, true, false) ((pred) ? (true) : (false))

/* modulo inc/dec - assumes x E [0, bound - 1] */
#define MODDEC(x, bound) MUX((x) == 0, (bound) - 1, (x) - 1)
#define MODINC(x, bound) MUX((x) == (bound) - 1, 0, (x) + 1)

/* modulo inc/dec, bound = 2^k */
#define MODDEC_POW2(x, bound) (((x) - 1) & ((bound) - 1))
#define MODINC_POW2(x, bound) (((x) + 1) & ((bound) - 1))

/* modulo add/sub - assumes x, y E [0, bound - 1] */
#define MODADD(x, y, bound) \
    MUX((x) + (y) >= (bound), (x) + (y) - (bound), (x) + (y))
#define MODSUB(x, y, bound) \
    MUX(((int)(x)) - ((int)(y)) < 0, (x) - (y) + (bound), (x) - (y))

/* module add/sub, bound = 2^k */
#define MODADD_POW2(x, y, bound) (((x) + (y)) & ((bound) - 1))
#define MODSUB_POW2(x, y, bound) (((x) - (y)) & ((bound) - 1))

/* crc defines */
#define CRC8_INIT_VALUE  0xff		/* Initial CRC8 checksum value */
#define CRC8_GOOD_VALUE  0x9f		/* Good final CRC8 checksum value */
#define CRC16_INIT_VALUE 0xffff		/* Initial CRC16 checksum value */
#define CRC16_GOOD_VALUE 0xf0b8		/* Good final CRC16 checksum value */
#define CRC32_INIT_VALUE 0xffffffff	/* Initial CRC32 checksum value */
#define CRC32_GOOD_VALUE 0xdebb20e3	/* Good final CRC32 checksum value */

/* use for direct output of MAC address in printf etc */
#define MACF				"%02x:%02x:%02x:%02x:%02x:%02x"
#define ETHERP_TO_MACF(ea)	((struct ether_addr *) (ea))->octet[0], \
							((struct ether_addr *) (ea))->octet[1], \
							((struct ether_addr *) (ea))->octet[2], \
							((struct ether_addr *) (ea))->octet[3], \
							((struct ether_addr *) (ea))->octet[4], \
							((struct ether_addr *) (ea))->octet[5]

#define ETHER_TO_MACF(ea) 	(ea).octet[0], \
							(ea).octet[1], \
							(ea).octet[2], \
							(ea).octet[3], \
							(ea).octet[4], \
							(ea).octet[5]

/* bcm_format_flags() bit description structure */
typedef struct bcm_bit_desc {
	uint32_t	bit;
	const char* name;
} bcm_bit_desc_t;

/* bcm_format_field */
typedef struct bcm_bit_desc_ex {
	uint32_t mask;
	const bcm_bit_desc_t *bitfield;
} bcm_bit_desc_ex_t;


/* tag_ID/length/value_buffer tuple */
typedef struct bcm_tlv {
	uint8_t	id;
	uint8_t	len;
	uint8_t	data[1];
} bcm_tlv_t;

/* Check that bcm_tlv_t fits into the given buflen */
#define bcm_valid_tlv(elt, buflen) ((buflen) >= 2 && (int)(buflen) >= (int)(2 + (elt)->len))

/* buffer length for ethernet address from bcm_ether_ntoa() */
#define ETHER_ADDR_STR_LEN	18	/* 18-bytes of Ethernet address buffer length */

/* crypto utility function */
/* 128-bit xor: *dst = *src1 xor *src2. dst1, src1 and src2 may have any alignment */
static inline void
xor_128bit_block(const uint8_t *src1, const uint8_t *src2, uint8_t *dst)
{
	if (
#ifdef __i386__
	    1 ||
#endif
	    (((uintptr_t)src1 | (uintptr_t)src2 | (uintptr_t)dst) & 3) == 0) {
		/* ARM CM3 rel time: 1229 (727 if alignment check could be omitted) */
		/* x86 supports unaligned.  This version runs 6x-9x faster on x86. */
		((uint32_t *)dst)[0] = ((const uint32_t *)src1)[0] ^ ((const uint32_t *)src2)[0];
		((uint32_t *)dst)[1] = ((const uint32_t *)src1)[1] ^ ((const uint32_t *)src2)[1];
		((uint32_t *)dst)[2] = ((const uint32_t *)src1)[2] ^ ((const uint32_t *)src2)[2];
		((uint32_t *)dst)[3] = ((const uint32_t *)src1)[3] ^ ((const uint32_t *)src2)[3];
	} else {
		/* ARM CM3 rel time: 4668 (4191 if alignment check could be omitted) */
		int k;
		for (k = 0; k < 16; k++)
			dst[k] = src1[k] ^ src2[k];
	}
}

/* externs */
/* crc */
extern uint8_t BCMROMFN(hndcrc8)(uint8_t *p, u_int nbytes, uint8_t crc);
extern uint16_t BCMROMFN(hndcrc16)(uint8_t *p, u_int nbytes, uint16_t crc);
extern uint32_t BCMROMFN(hndcrc32)(uint8_t *p, u_int nbytes, uint32_t crc);

/* format/print */
#if defined(BCMDBG) || defined(DHD_DEBUG) || defined(BCMDBG_ERR) || \
	defined(WLMSG_PRHDRS) || defined(WLMSG_PRPKT) || defined(WLMSG_ASSOC)
/* print out the value a field has: fields may have 1-32 bits and may hold any value */
extern int bcm_format_field(const bcm_bit_desc_ex_t *bd, uint32_t field, char* buf, int len);
/* print out which bits in flags are set */
extern int bcm_format_flags(const bcm_bit_desc_t *bd, uint32_t flags, char* buf, int len);
#endif

#if defined(BCMDBG) || defined(DHD_DEBUG) || defined(BCMDBG_ERR) || \
	defined(WLMSG_PRHDRS) || defined(WLMSG_PRPKT) || defined(WLMSG_ASSOC) || \
	defined(WLMEDIA_PEAKRATE)
extern int bcm_format_hex(char *str, const void *bytes, int len);
#endif

#ifdef BCMDBG
extern void deadbeef(void *p, u_int len);
#endif
extern const char *bcm_crypto_algo_name(u_int algo);
extern char *bcm_chipname(u_int chipid, char *buf, u_int len);
extern char *bcm_brev_str(uint32_t brev, char *buf);
extern void printbig(char *buf);
extern void prhex(const char *msg, u_char *buf, u_int len);

/* IE parsing */
extern bcm_tlv_t *BCMROMFN(bcm_next_tlv)(bcm_tlv_t *elt, int *buflen);
extern bcm_tlv_t *BCMROMFN(bcm_parse_tlvs)(void *buf, int buflen, u_int key);
extern bcm_tlv_t *BCMROMFN(bcm_parse_ordered_tlvs)(void *buf, int buflen, u_int key);

/* bcmerror */
extern const char *bcmerrorstr(int bcmerror);
extern bcm_tlv_t *BCMROMFN(bcm_parse_tlvs)(void *buf, int buflen, u_int key);

/* multi-bool data type: set of bools, mbool is true if any is set */
typedef uint32_t mbool;
#define mboolset(mb, bit)		((mb) |= (bit))		/* set one bool */
#define mboolclr(mb, bit)		((mb) &= ~(bit))	/* clear one bool */
#define mboolisset(mb, bit)		(((mb) & (bit)) != 0)	/* TRUE if one bool is set */
#define	mboolmaskset(mb, mask, val)	((mb) = (((mb) & ~(mask)) | (val)))

/* generic datastruct to help dump routines */
struct fielddesc {
	const char *nameandfmt;
	uint32_t 	offset;
	uint32_t 	len;
};

extern void bcm_binit(struct bcmstrbuf *b, char *buf, u_int size);
extern void bcm_bprhex(struct bcmstrbuf *b, const char *msg, bool newline, uint8_t *buf, int len);

extern void bcm_inc_bytes(u_char *num, int num_bytes, uint8_t amount);
extern int bcm_cmp_bytes(const u_char *arg1, const u_char *arg2, uint8_t nbytes);
extern void bcm_print_bytes(const char *name, const u_char *cdata, int len);

typedef  uint32_t (*bcmutl_rdreg_rtn)(void *arg0, u_int arg1, uint32_t offset);
extern u_int bcmdumpfields(bcmutl_rdreg_rtn func_ptr, void *arg0, u_int arg1, struct fielddesc *str,
                          char *buf, uint32_t bufsize);
extern u_int BCMROMFN(bcm_bitcount)(uint8_t *bitmap, u_int bytelength);

extern int bcm_bprintf(struct bcmstrbuf *b, const char *fmt, ...);

/* power conversion */
extern uint16_t BCMROMFN(bcm_qdbm_to_mw)(uint8_t qdbm);
extern uint8_t BCMROMFN(bcm_mw_to_qdbm)(uint16_t mw);
extern u_int bcm_mkiovar(char *name, char *data, u_int datalen, char *buf, u_int len);

unsigned int process_nvram_vars(char *varbuf, unsigned int len);

/* calculate a * b + c */
extern void bcm_uint64_t_multiple_add(uint32_t* r_high, uint32_t* r_low, uint32_t a, uint32_t b, uint32_t c);
/* calculate a / b */
extern void bcm_uint64_t_divide(uint32_t* r, uint32_t a_high, uint32_t a_low, uint32_t b);
/* calculate a >> b */
void bcm_uint64_t_right_shift(uint32_t* r, uint32_t a_high, uint32_t a_low, uint32_t b);
#ifdef __cplusplus
	}
#endif
#endif	/* _bcmutils_h_ */
