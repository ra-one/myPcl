#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h> /*for uint16_t*/
#include <stdarg.h>

#include "scc.h"
#include "RCCE_memcpy.c"

//used to write SHM start address into MPB
uintptr_t  addr=0x0;

// global variables for the MPB, LUT, PINS, LOCKS and AIR
t_vcharp mpbs[48];
t_vcharp locks[CORES];
volatile int *irq_pins[CORES];
volatile uint64_t *luts[CORES];
AIR atomic_inc_regs[2*CORES];

int node_location;
int num_worker = -1;
int master_ID = -1;

void sccInit(int masterNode, int numWorkers){
  //variables for the MPB init
  int core,size;
  int x, y, z, address;
  unsigned char cpu;

  unsigned char num_pages;

  //variables for the AIR init
  int *air_baseE, *air_baseF;

  InitAPI(0);

  num_worker = numWorkers;
  master_ID = masterNode;
	 
  z = ReadConfigReg(CRB_OWN+MYTILEID);
  x = (z >> 3) & 0x0f; // bits 06:03
  y = (z >> 7) & 0x0f; // bits 10:07
  z = z & 7; // bits 02:00
  node_location = PID(x, y, z);

  air_baseE = (int *) MallocConfigReg(FPGA_BASE + 0xE000);
  air_baseF = (int *) MallocConfigReg(FPGA_BASE + 0xF000);

  for (cpu = 0; cpu < 48; cpu++){
    x = X_PID(cpu);
    y = Y_PID(cpu);
    z = Z_PID(cpu);

    if (cpu == node_location) address = CRB_OWN;
    else address = CRB_ADDR(x, y);

    //LUT, PINS, LOCK allocation
    irq_pins[cpu] = MallocConfigReg(address + (z ? GLCFG1 : GLCFG0));
    luts[cpu] = (uint64_t*) MallocConfigReg(address + (z ? LUT1 : LUT0));
    locks[cpu] = (t_vcharp) MallocConfigReg(address + (z ? LOCK1 : LOCK0));
    
    //MPB allocation
    MPBalloc(&mpbs[cpu], x, y, z, cpu == node_location);
    //printf("mpbs[%d] %p\n",cpu,mpbs[cpu]);

    //FIRST SET OF AIR
    atomic_inc_regs[cpu].counter = air_baseE + 2*cpu;
    atomic_inc_regs[cpu].init 	 = air_baseE + 2*cpu + 1;

    //SECOND SET OF AIR
    atomic_inc_regs[CORES+cpu].counter 	= air_baseF + 2*cpu;
    atomic_inc_regs[CORES+cpu].init 	= air_baseF + 2*cpu + 1;
    if(node_location == master_ID){
      // only one core(master) needs to call this
      *atomic_inc_regs[cpu].init = 0;
      *atomic_inc_regs[CORES+cpu].init = 0;
    }
  } //end for

  //***********************************************
  //LUT remapping

  num_pages = PAGES_PER_CORE - LINUX_PRIV_PAGES;
  int max_pages = MAX_PAGES-1;
  int i, lut, origin;
  
  /* Because we used all the AIR in the mailbox to run a first version,
   * atomic_inc_regs 30 is used here because to test it we never run worker Nr. 30
   * but in a proper version this should be changed back to the out-commented line above each atomic operation
   */  

  if(node_location != master_ID){
    int value=-1;
    PRT_DBG1("Wait for MASTER'S LUT MAPPING!!! \n");
    while(value != num_worker){
      //atomic_readR(&atomic_inc_regs[master_ID],&value);
      //atomic_readR(&atomic_inc_regs[master_ID],&value);
      memcpy((void*)&value, (const void*)mpbs[master_ID]+8, sizeof(int));
    }
    PRT_DBG("value= %d\n",value);
    PRT_DBG1("AIR==1 -> MASTER has finished LUT mapping.\n");
  }
  /*
   * LUT MAPPPING WHICH TAKES UNUSED ENTRIES 0-40 FROM EACH UNUSED CORE AND MAPPS THEM INTO THE MASTER CORE,
   * AFTERWARDS EACH CORE MAPS THE MASTER LUT ENTRY 41-192 IN HIS OWN CORE
   * the problem of this mapping is that it can not be used for 48 cores, because in that case we don't have unused cores,
   * therefore change it back to the version above but don't forget to check if the mapping above is correct
   */
	num_pages=0;
	i=1;
	lut=0;
	for (i = 1; i < CORES / num_worker && num_pages < max_pages; i++) {
    for (lut = 0; lut < PAGES_PER_CORE && num_pages < max_pages; lut++) {
      LUT(node_location, PAGES_PER_CORE + num_pages++) = LUT(i * num_worker, lut);
    	//PRT_DBG("Copy to %i  node's LUT entry Nr.: %i / %x from %i node's LUT entry Nr.: %i / %x. Num_pages: %i, Max_pages: %i\n",
      //                             node_location, PAGES_PER_CORE+ (num_pages-1),PAGES_PER_CORE+(num_pages-1), i*num_worker,  lut, lut, num_pages-1, max_pages);
	
		}
  }

  //***********************************************
  // some inits for the MPB
  flush();
  //false = 0, true = 1
  START(node_location) = 0;
  END(node_location) = 0;
  /* Start with an initial handling run to avoid a cross-core race. */
  HANDLING(node_location) = 1;
  WRITING(node_location) = 0;

  //***********************************************
  /*
  * synchronisation in the init state:
  * The Master maps the SHM and writes the SHM Start-address to the MPB such that each worker can read it and we can get a proper SHM
  *
  */
  if(node_location == master_ID){
    SCCMallocInit((void *)&addr);
    PRT_DBG1("addr: %p\n",addr);
    memcpy((void*)mpbs[master_ID]+16, (const void*)&addr, sizeof(uintptr_t));
    //memcpy((void*)mpbs[master_ID]+8, (const void*)&num_worker, sizeof(int));
    //cpy_mem_to_mpb(0, (void *)&addr, sizeof(uintptr_t));	
    //atomic_writeR(&atomic_inc_regs[master_ID],num_worker);
  }else{
    /*
     * READING AND WRITING BACK AFTERWARDS BECAUSE OF POINTER MOVING IN CPY_MPB_TO_MEM
     * solve this case, by using a write to a fixed unused MPB address
     */
    
    //cpy_mpb_to_mem(0, (void *)&addr, sizeof(uintptr_t));
    //cpy_mem_to_mpb(0, (void *)&addr, sizeof(uintptr_t));
    memcpy((void*)&addr, (const void*)mpbs[master_ID]+16, sizeof(uintptr_t));
    PRT_DBG1("addr: %p\n",addr);
    SCCMallocInit((void *)&addr);
  }

  //***********************************************
  //FOOL_WRITE_COMBINE;
  unlock(node_location);
  if(node_location == master_ID){
    memcpy((void*)mpbs[master_ID]+8, (const void*)&num_worker, sizeof(int));
  }
  FOOL_WRITE_COMBINE;
}

//--------------------------------------------------------------------------------------
// FUNCTION: SccGetNodeID
//--------------------------------------------------------------------------------------
// Returns node's ID
//--------------------------------------------------------------------------------------
int SccGetNodeID(void){
	return node_location;
}

//--------------------------------------------------------------------------------------
// FUNCTION: atomic_inc
//--------------------------------------------------------------------------------------
// Increments an AIR register and returns its privious content
//--------------------------------------------------------------------------------------
void atomic_incR(AIR *reg, int *value)
{
  (*value) = (*reg->counter);
}

//--------------------------------------------------------------------------------------
// FUNCTION: atomic_dec
//--------------------------------------------------------------------------------------
// Decrements an AIR register and returns its privious content
//--------------------------------------------------------------------------------------
void atomic_decR(AIR *reg, int value)
{
  (*reg->counter) = value;
}

//--------------------------------------------------------------------------------------
// FUNCTION: atomic_read
//--------------------------------------------------------------------------------------
// Returns the current value of an AIR register with-out modification to AIR
//--------------------------------------------------------------------------------------
void atomic_readR(AIR *reg, int *value)
{
  (*value) = (*reg->init);
}

//--------------------------------------------------------------------------------------
// FUNCTION: atomic_write
//--------------------------------------------------------------------------------------
// Initializes an AIR register by writing a start value
//--------------------------------------------------------------------------------------
void atomic_writeR(AIR *reg, int value)
{
  (*reg->init) = value;
}

void cpy_mpb_to_mem(int node, void *dst, int size)
{
  int start, end, cpy;

  flush();
  start = START(node);
  end = END(node);

  while (size) {
    if (end < start) cpy = min(size, B_SIZE - start);
    else cpy = size;

    flush();
    memcpy(dst, (void*) (mpbs[node] + B_START + start), cpy);
    start = (start + cpy) % B_SIZE;
    dst = ((char*) dst) + cpy;
    size -= cpy;
  }

  flush();
  START(node) = start;
  FOOL_WRITE_COMBINE;
}

void cpy_mem_to_mpb(int node, void *src, int size)
{
  int start, end, free;

  if (size >= B_SIZE) {
    printf("Message to big!");
    exit(3);
  }

  flush();
  WRITING(node) = 1; //false = 0, true = 1
  FOOL_WRITE_COMBINE;

  while (size) {
    flush();
    start = START(node);
    end = END(node);

    if (end < start) free = start - end - 1;
    else free = B_SIZE - end - (start == 0 ? 1 : 0);
    free = min(free, size);

    if (!free) {
      unlock(node);
      usleep(1);
      lock(node);
      continue;
    }

		PRT_DBG("memcpy_put:\n node: %d B_START+end: %d src: %s size: %d\n", node,B_START + end,src,free);
		memcpy_put((void*) (mpbs[node] + B_START + end), src, free);

    flush();
    size -= free;
    src += free;
    END(node) = (end + free) % B_SIZE;
    WRITING(node) = 0; //false = 0, true = 1
    FOOL_WRITE_COMBINE;
  }
}
