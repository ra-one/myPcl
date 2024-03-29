#ifndef SCC_H
#define SCC_H

#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>

#include <pthread.h>

#include "scc_config.h"
#include "debugging.h"

#define MAX_PAGES       151 
#define START_PAGE      0x29 // 41
#define END_PAGE        0xBF // 191
#define SHM_MEMORY_SIZE 0x97000000 // 151 * 16 = 2416: 2416 * 1024 * 1024 = 0x97000000
#define SHM_ADDR        0x29000000 // start at 0x29 (41) to 0xBE (191): 149 pages

#define MBXSZ               20 //mailbox size d=16, x=0x10
#define INFOSZ              10 //size of memInfo d=8 x=0x8

#define CORES               (NUM_ROWS * NUM_COLS * NUM_CORES)
#define FOOL_WRITE_COMBINE  ((*((volatile int*)firstMPB)) = 1)

#define SNETGLOBWAITVAL      6
#define WAITWORKERSVAL       9
#define SNETGLOBWAIT        (*((volatile int*)(firstMPB + 34))) // on MPB line 0
#define WAITWORKERS         (*((volatile int*)(firstMPB + 66)))
#define MALLOCADDR          (firstMPB + 98)
#define OBADDR              (firstMPB + 130)
#define OBSET               (*((volatile int*)(firstMPB + 162)))
#define TIMEADDR            (firstMPB + 194)
#define MEM_FREE_LL         (firstMPB + 226)

#define FREQFLAG            (*((volatile int*)(firstMPB + 258)))
#define FREQPROP            (*((volatile double*)(firstMPB + 290)))
#define DEMA                (*((volatile double*)(firstMPB + 322)))
#define EMA                 (*((volatile double*)(firstMPB + 354)))

/*
#define 1ALPHAW              (*((volatile double*)(firstMPB + 386)))
#define 1THW                 (*((volatile double*)(firstMPB + 418)))
#define 1ALPHAOV             (*((volatile double*)(firstMPB + 450)))
#define 1THOV                (*((volatile double*)(firstMPB + 482)))
*/


// 4 5 16 17 1MC1
#define atomicLCK             17 // not used         
#define mallocLCK             4
#define freqLCK               5

// 28 29 40 41 1MC2
//#define atomicLCK             28 // not used         
//#define mallocLCK             40
//#define freqLCK               41

// 6 7 18 19 1MC3
//#define atomicLCK             19 // not used         
//#define mallocLCK             6
//#define freqLCK               7

// 30 31 42 43 1MC4
//#define atomicLCK             30 // not used         
//#define mallocLCK             42
//#define freqLCK               43


// 17 18 29 30 NORMAL 4MC
// 16 17 30 31 PWR 2MC
//#define atomicLCK             17 // not used         
//#define mallocLCK             30
//#define freqLCK               18



/* Power defines */
#define RC_MAX_FREQUENCY_DIVIDER     16  // maximum divider value, so lowest F
#define RC_MIN_FREQUENCY_DIVIDER     2   // minimum divider value, so highest F
#define RC_NUM_VOLTAGE_LEVELS        7
#define RC_GLOBAL_CLOCK_MHZ          1600
#define RC_VOLTAGE_DOMAINS           6

extern uintptr_t  *allMbox;

extern t_vcharp firstMPB;
extern t_vcharp locks[CORES];

typedef struct block_t{
  struct block_t *next;
  size_t size;
  int coreId;
}block_t;

typedef struct block_t_free{
  block_t *list;
}block_t_free;

block_t_free *mem_free_arr; //array of linked list of memory to be freed

/* Flush MPBT from L1. */
static inline void INVMPBTL1() { __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" );}

void acquire_lock();
void release_lock();

/*sync functions*/
typedef volatile struct _AIR {
        int *counter;
        int *init;
} AIR;

extern AIR atomic_inc_regs[2*CORES];

/* Power functions */

typedef struct {
float volt; 
int   VID;
int   MHz_cap; 
} triple;

typedef struct {
int current_volt_level; 
int current_freq_div; 
} dom_fv;

typedef struct{
  unsigned long    tv_sec;    // seconds 
  //unsigned long    tv_usec;    // microseconds
  unsigned long    tv_nsec;   // nanoseconds  
}timespecSCC;

// allocate by master, access by src/sink/master

typedef struct {
	int change_mess; //change sleep after this number of messages generated
	int change_percent; // change sleep by this much %
	
	int skip_update;    // skip update frequency by a number of output messages, should be >= window_size
  int window_size;    // window of observing output messages
  double thresh_hold;   //TODO: thresh_hold to change the freq
  
  int skip_count;     //count number of output
  int startChange;	 // start changing freq and inp rate
  
  int output_index;     // index in the window of observed output rate
  double *output_interval;  // window of output interval
  struct timeval last_output; // timestamp of last output
  unsigned long *output_interval_scc;  // window of output interval
  timespecSCC last_output_scc; // timestamp of last output

  double input_rate;
  double output_rate;
  double volt;
  int freq;
} observer_t;

extern int activeDomains[6];
extern int RC_COREID[CORES];
extern int DVFS;
extern double ALPHAW,THW,ALPHAOV,THOV;
extern observer_t *obs;

void set_min_freq();
void change_freq(double prop,char c);

int set_frequency_divider(int Fdiv, int *new_Fdiv, int domain);
int set_freq_volt_level(int Fdiv, int *new_Fdiv, int *new_Vlevel, int domain);
void startPowerMeasurement(int start);
void powerMeasurement(FILE *fileHand);
int get_volt_level(int logicalDomain);
int isDvfsActive();

void SCCGetTimeAll(timespecSCC *t);
double SCCGetTime();               /* sec  (seconds) */
unsigned long long SCCGetTimMS(); 
unsigned long long SCCGetTimNS(); 


/* Support Functions */
void SCCInit();
void SCCStop();
int  SCCGetNodeID();
int  SCCGetNodeRank();
int  SCCIsMaster();
int  SCCGetNumWrappers();

void atomic_incR(AIR *reg, int *value);
void atomic_decR(AIR *reg, int value);
void atomic_readR(AIR *reg, int *value);
void atomic_writeR(AIR *reg, int value);

/* malloc functions */

void SCCMallocInit(uintptr_t *addr);
void SCCMallocStop(void);
void *SCCFirstMalloc(void);
void *SCCMallocPtr(size_t size);
void SCCFreePtr(void *p);
int DCMflush();
//call when you want to free blocks from LL
void SCC_Free_Ptr_rpc_to_local(); 

//#define _dbg_

#ifdef _dbg_
static inline void lock(int core) {
  printf("------------------------------------- Will try to get lock %p %f\n",locks[core],SCCGetTime());
  while(!(*locks[core] & 0x01)){
      printf(". ");
      fflush(stdout);
      usleep(node_id);
  }
  printf("\n------------------------------------- Lock acquired %p %f\n",locks[core],SCCGetTime());
}

static inline void unlock(int core) {
 *locks[core] = 0; 
 printf("------------------------------------- Lock Released %p %f\n",locks[core],SCCGetTime());
}
#else
/* Semantics of test&set register: a read returns zero if another core has
 * previously read it and no reset has occurred since then. Otherwise, the read
 * returns one.
 */
 
/* Try to lock the HW lock.
 * \param[in] id : HW lock id (0~47).
 * \retval 1 : got and held the lock.
 * \retval 0 : didn't get the lock.
 */
static inline int tryLock(int core) { return *(int *) locks[core] & 0x01; }
static inline void lock(int core) { while (!(*locks[core] & 0x01)); }
static inline void unlock(int core) { *locks[core] = 0; }
#endif // _nodbg_

#endif /*SCC_H*/

/*
typedef struct {
	int change_mess; //change sleep after this number of messages generated
	int change_percent; // change sleep by this much %
	
	int skip_update;    // skip update frequency by a number of output messages, should be >= window_size
  int window_size;    // window of observing output messages
  double thresh_hold;   //TODO: thresh_hold to change the freq
  
  int skip_count;     //count number of output
  int startChange;	 // start changing freq and inp rate
  
  unsigned long *output_interval;  // window of output interval
  int output_index;     // index in the window of observed output rate
  timespecSCC last_output; // timestamp of last output

  double input_rate;
  double output_rate;
  double volt;
  int freq;
} observer_t;
*/
