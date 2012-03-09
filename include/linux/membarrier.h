#ifndef _LINUX_MEMBARRIER_H
#define _LINUX_MEMBARRIER_H

/* First argument to membarrier syscall */

/*
 * Mandatory flags to the membarrier system call that the kernel must
 * understand are in the low 16 bits.
 */
#define MEMBARRIER_MANDATORY_MASK	0x0000FFFF	/* Mandatory flags */

/*
 * Optional hints that the kernel can ignore are in the high 16 bits.
 */
#define MEMBARRIER_OPTIONAL_MASK	0xFFFF0000	/* Optional hints */

/* Expedited: adds some overhead, fast execution (few microseconds) */
#define MEMBARRIER_EXPEDITED		(1 << 0)
/* Delayed: Low overhead, but slow execution (few milliseconds) */
#define MEMBARRIER_DELAYED		(1 << 1)

/* Query flag support, without performing synchronization */
#define MEMBARRIER_QUERY		(1 << 16)


/*
 * All memory accesses performed in program order from each process threads are
 * guaranteed to be ordered with respect to sys_membarrier(). If we use the
 * semantic "barrier()" to represent a compiler barrier forcing memory accesses
 * to be performed in program order across the barrier, and smp_mb() to
 * represent explicit memory barriers forcing full memory ordering across the
 * barrier, we have the following ordering table for each pair of barrier(),
 * sys_membarrier() and smp_mb() :
 *
 * The pair ordering is detailed as (O: ordered, X: not ordered):
 *
 *                        barrier()   smp_mb() sys_membarrier()
 *        barrier()          X           X            O
 *        smp_mb()           X           O            O
 *        sys_membarrier()   O           O            O
 *
 * This synchronization only takes care of threads using the current process
 * memory map. It should not be used to synchronize accesses performed on memory
 * maps shared between different processes.
 */

#endif
