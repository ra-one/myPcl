#ifndef SCC_H
#define SCC_H

#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include "scc_config.h"
#include "debugging.h"

#define PAGE_SIZE           (16*1024*1024)
#define LINUX_PRIV_PAGES    (20)
#define PAGES_PER_CORE      (41)

#define _MAX_MEM_2384__

#ifdef _MAX_MEM_2384__
#define MAX_PAGES       149 
#define START_PAGES     0X2A
#define END_PAGE        0xBE
#define SHM_MEMORY_SIZE 0x95000000 // 149 * 16 = 2384: 2384 * 1024 * 1024 = 0x95000000
#define SHM_ADDR        0x42000000 // start at 0x2A (41) to 0xBE (191): 149 pages
#else
#define MAX_PAGES       59 //instead of 192, because first 41 can not be used to map into
#define START_PAGES     0X84
#define END_PAGE        0xBE
#define SHM_MEMORY_SIZE 0x3B000000 // 59 * 16 = 944: 944*1024*1024 = 3B00 0000
#define SHM_ADDR        0x84000000 // start at 0x84 (132) to 0xbe (190): 59 pages
#endif //_MAX_MEM_2384__


#define MEMORY_OFFSET(id) 	(id *(SHM_MEMORY_SIZE/(num_worker+num_wrapper)))

#define CORES               (NUM_ROWS * NUM_COLS * NUM_CORES)

#define FOOL_WRITE_COMBINE  ((*((volatile int*)firstMPB)) = 1)
#define MPB_LINE_SIZE       (1<<5) // 32

#define SNETGLOBWAITVAL      6
#define WAITWORKERSVAL       9
#define SNETGLOBWAIT        (*((volatile int*)(firstMPB + 2))) // on MPB line 0
#define WAITWORKERS         (*((volatile int*)(firstMPB + 34)))
#define MALLOCADDR          (firstMPB + 66)
#define MESSTOP             (*((volatile int*)(firstMPB + 98)))
#define SOSIADDR            (firstMPB + 130)
#define OBSSET              (*((volatile int*)(firstMPB + 160)))

/* Power defines */
#define RC_MAX_FREQUENCY_DIVIDER     16  // maximum divider value, so lowest F
#define RC_MIN_FREQUENCY_DIVIDER     2   // minimum divider value, so highest F
#define RC_NUM_VOLTAGE_LEVELS        7
#define RC_GLOBAL_CLOCK_MHZ          1600
#define RC_VOLTAGE_DOMAINS           6

extern int node_id;
extern int master_id;
extern int num_worker;
extern int num_wrapper;
extern int num_mailboxes;
extern uintptr_t  *allMbox;

extern t_vcharp mbox_start_addr;

extern t_vcharp firstMPB;
extern t_vcharp locks[CORES];


static inline int min(int x, int y) { return x < y ? x : y; }

/* Flush MPBT from L1. */
static inline void flush() { __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" );}

static inline void lock(int core) { while (!(*locks[core] & 0x01)); }
static inline void unlock(int core) { *locks[core] = 0; }

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
	int no_mess; // number of messages to generate
	int mess_size; // size of each message
	int num_pipeline; // number of parallel pipelines
	int sleep_micro; // sleep in micro second
	int change_mess; //change sleep after this number of messages generated
	int change_percent; // change sleep by this much %
	
	int skip_update;    // skip update frequency by a number of ouput messages, should be >= window_size
  int window_size;    // window of observing ouput messages
  int thresh_hold;   //TODO: thresh_hold to change the freq
  
  int skip_count;     //count number of output
  
  double *output_interval;  // window of output interval
  int output_index;     // index in the window of observed output rate
  double last_output; // timestamp of last output

  double input_rate;
  double output_rate;
  
  int freq;
} observer_t;

extern observer_t *obs;

void set_min_freq();
void change_freq(int inc);


int set_frequency_divider(int Fdiv, int *new_Fdiv, int domain);
int set_freq_volt_level(int Fdiv, int *new_Fdiv, int *new_Vlevel, int domain);
void startPowerMeasurement(int start);
void powerMeasurement(FILE *fileHand);

/* Support Functions */
void SCCInit(int numWorkers, int numWrapper, int enableDVFS, char *hostFile, char *masterFilePath);
void SCCStop();
int  SCCGetNodeID();
int  SCCGetNodeRank();
int  SCCIsMaster();
int  SCCGetNumWrappers();
int  SCCGetNumWrappers();
double SCCGetTime();
void atomic_incR(AIR *reg, int *value);
void atomic_decR(AIR *reg, int value);
void atomic_readR(AIR *reg, int *value);
void atomic_writeR(AIR *reg, int value);

/* malloc functions */

void SCCMallocInit(uintptr_t *addr,int numMailboxes);
void SCCMallocStop(void);
void *SCCGetlocal(void);
void *SCCGetLocalMemStart(void);
void *SCCMallocPtr(size_t size);
void *SCCVMallocPtr(size_t size);
void SCCFreePtr(void *p);

int DCMflush();

/*
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
*/

#endif /*SCC_H*/

