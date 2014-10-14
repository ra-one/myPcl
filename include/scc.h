#ifndef SCC_H
#define SCC_H

#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>

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
#define SNETGLOBWAIT        (*((volatile int*)(firstMPB + 2))) // on MPB line 0
#define WAITWORKERS         (*((volatile int*)(firstMPB + 34)))
#define MALLOCADDR          (firstMPB + 66)
#define OBADDR              (firstMPB + 98)
#define OBSET               (*((volatile int*)(firstMPB + 130)))
#define TIMEADDR            (firstMPB + 160)


/* Power defines */
#define RC_MAX_FREQUENCY_DIVIDER     16  // maximum divider value, so lowest F
#define RC_MIN_FREQUENCY_DIVIDER     2   // minimum divider value, so highest F
#define RC_NUM_VOLTAGE_LEVELS        7
#define RC_GLOBAL_CLOCK_MHZ          1600
#define RC_VOLTAGE_DOMAINS           6

extern uintptr_t  *allMbox;

extern t_vcharp firstMPB;
extern t_vcharp locks[CORES];

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

extern int activeDomains[6];
extern int RC_COREID[CORES];
extern int DVFS;

// allocate by source, access by sink
typedef struct {
	int change_mess; //change sleep after this number of messages generated
	int change_percent; // change sleep by this much %
	
	int skip_update;    // skip update frequency by a number of output messages, should be >= window_size
  int window_size;    // window of observing output messages
  double thresh_hold;   //TODO: thresh_hold to change the freq
  
  int skip_count;     //count number of output
  int startChange;	 // start changing freq and inp rate
  
  double *output_interval;  // window of output interval
  int output_index;     // index in the window of observed output rate
  struct timeval last_output; // timestamp of last output

  double input_rate;
  double output_rate;
  double volt;
  int freq;
} observer_t;

extern observer_t *obs;

void set_min_freq();
void change_freq(double prop,char c);


int set_frequency_divider(int Fdiv, int *new_Fdiv, int domain);
int set_freq_volt_level(int Fdiv, int *new_Fdiv, int *new_Vlevel, int domain);
void startPowerMeasurement(int start);
void powerMeasurement(FILE *fileHand);
int isDvfsActive();

typedef struct{
  unsigned long    tv_sec;    // seconds 
  unsigned long    tv_nsec;   // nanoseconds  
}timespecSCC;

void SCCGetTimeAll(timespecSCC *t);
double SCCGetTime();               /* sec  (seconds) */

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
static inline void lock(int core) { while (!(*locks[core] & 0x01)); }
static inline void unlock(int core) { *locks[core] = 0; }
#endif // _nodbg_

#endif /*SCC_H*/

