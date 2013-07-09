#ifndef SCC_H
#define SCC_H

#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include "scc_config.h"
#include "bool.h"
#include "input.h"


#define PAGE_SIZE           (16*1024*1024)
#define LINUX_PRIV_PAGES    (20)
#define PAGES_PER_CORE      (41)
#define MAX_PAGES           (152) //instead of 192, because first 41 can not be used to map into
//#define SHM_MEMORY_SIZE		(PAGE_SIZE*(MAX_PAGES-1))
#define SHM_MEMORY_SIZE		0x98000000
#define LOCAL_SHMSIZE  		SHM_MEMORY_SIZE/NR_WORKERS
#define MEMORY_OFFSET(id) 	(id *(SHM_MEMORY_SIZE/NR_WORKERS))
#define MAILBOX_OFFSET 8


#define CORES               (NUM_ROWS * NUM_COLS * NUM_CORES)
#define IRQ_BIT             (0x01 << GLCFG_XINTR_BIT)

#define B_OFFSET            64
#define FOOL_WRITE_COMBINE  (mpbs[node_location][0] = 1)
#define START(i)            (*((volatile uint16_t *) (mpbs[i] + B_OFFSET)))
#define END(i)              (*((volatile uint16_t *) (mpbs[i] + B_OFFSET + 2)))
#define HANDLING(i)         (*(mpbs[i] + B_OFFSET + 4))
#define WRITING(i)          (*(mpbs[i] + B_OFFSET + 5))
#define B_START             (B_OFFSET + 32)
#define B_SIZE              (MPBSIZE - B_START)

#define LUT(loc, idx)       (*((volatile uint32_t*)(&luts[loc][idx])))

//added by Simon start

#define DLPEL_ACTIVE_NODES			4	
#define DLPEL_SUCCESS                   	0
//#define MPB_LINE_SIZE                     	5
#define MPB_LINE_SIZE                     	(1<<LOG2_LINE_SIZE)
// RCCE_BUFF_SIZE_MAX is space per UE, which is half of the space per tile
#define MPB_BUFF_SIZE_MAX                   (1<<13)
#define MPB_BUFF_SIZE						(1<<13) //8000
#define MPB_META_DATA_OFFSET(ID)			(ID % 2) * MPB_BUFF_SIZE
#define MPB_BUFFER_OFFSET					(1<<5)	//32
#define SCC_MASTER_NODE						0
#define WRITING_FLAG_OFFSET					(0<<0)	//0
#define READING_FLAG_OFFSET					(1<<0)	//1
#define MSG_TYPE_OFFSET						(1<<2)	//2
#define FLAG_SIZE							1

//added by Simon end

extern bool remap;
extern int node_location;

extern t_vcharp mpbs[CORES];
extern t_vcharp locks[CORES];
extern volatile int *irq_pins[CORES];
extern volatile uint64_t *luts[CORES];


static inline int min(int x, int y) { return x < y ? x : y; }

/* Flush MPBT from L1. */
static inline void flush() { __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" ); }

static inline void lock(int core) { while (!(*locks[core] & 0x01)); }

static inline void unlock(int core) { *locks[core] = 0; }


void cpy_mpb_to_mem(int node, void *dst, int size);
void cpy_mem_to_mpb(int node, void *src, int size);

#endif /*SCC_H*/

