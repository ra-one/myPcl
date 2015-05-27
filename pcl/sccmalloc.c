#include <fcntl.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>
#include <malloc.h>
#include <assert.h>

#include "scc.h"

#define gMemB  infoGlobal->memleft
#define gMemKB ((double)infoGlobal->memleft/1024)
#define gMemMB ((double)infoGlobal->memleft/(1024*1024))

#define PRT_ADR //
#define DBG_ADR //
#define PRT_MLK //

#define PRT_FR_DBG //

static int myId = 0;

typedef struct memInfoG{
  char *freePtr;
  size_t memleft;
}memInfoG;

pthread_mutex_t malloc_lock;

static void *local;

uintptr_t start,end;

extern FILE *logFile;

pthread_mutex_t malloc_lock;

block_t *freeList;
memInfoG *infoGlobal,tempInfo;

void SCC_Free_Ptr(void *p);

/*
 * SCCMallocInit creates a new mapping for the SHM and sets the "addr" pointer to the beginning address of this SHM
 */

void SCCMallocInit(uintptr_t *addr)
{  
  pthread_mutex_init(&malloc_lock, NULL);
  
  pthread_mutex_lock(&malloc_lock);

  /* create a new mapping for the SHM  if the addr ptr is unset then the calling 
   * node is the MASTER and has to create the mapping and set the start-address
   * if the addr ptr is set then the calling node is a WORKER and just has to 
   * map the memory to a fixed start-address gotten from the MASTER
   */
  unsigned int alignedAddr = (SHM_ADDR) & (~(getpagesize()-1));
  unsigned int pageOffset = (SHM_ADDR) - alignedAddr;
  
  PRT_ADR("alignedAddr: %zu, pageOffset: %zu",alignedAddr,pageOffset);
  
	if (*addr==0x0){
		PRT_ADR("MASTER MMAP\n\n");
    //void *mmap(void *addr, size_t length, int prot, int flags,int fd, off_t offset);
		local = mmap(NULL, 		SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, NCMDeviceFD, alignedAddr);
		
		if (local == NULL) { fprintf(stderr, "Couldn't map memory!\n"); exit(-1); }
		else	munmap(local, SHM_MEMORY_SIZE);
		
		local = mmap((void*)local, 	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, NCMDeviceFD, alignedAddr);
   	
		if (local == NULL) { fprintf(stderr, "Couldn't map memory!\n"); exit(-1); }
    
		*addr=local;
  }else{
		PRT_ADR("WORKER MMAP\n\n");
		local=*addr;	
		local = mmap((void*)local, SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, NCMDeviceFD, alignedAddr);
   	if (local == NULL) { fprintf(stderr, "Couldn't map memory!"); exit(-1); }
  }  

  start = *addr;
  end = start + SHM_MEMORY_SIZE;

  infoGlobal = malloc(sizeof(memInfoG*));
  
  int cores = SCCGetNumCores();
  myId = SCCGetNodeID(); //physical id
  
  if(SCCIsMaster()){
    tempInfo.freePtr = local+sizeof(memInfoG)+(MBXSZ*cores);
    tempInfo.memleft = SHM_MEMORY_SIZE-(sizeof(memInfoG)+(MBXSZ*cores));
    memcpy((void*)(local+(MBXSZ*cores)),(const void*)&tempInfo,sizeof(memInfoG)); 
  }  
  infoGlobal = (local+(MBXSZ*cores)); 
  pthread_mutex_unlock(&malloc_lock);
}

/*
 * SCCMallocPtr is used to allocate memory in the SHM
 */
void *SCCMallocPtrGlobal(size_t size)
{
   void* ptr;
   lock(mallocLCK);
   //printf("sccMallocGlob size: %zu, mem left B:%zu, KB:%f, MB:%f\n",size,gMemB,gMemKB,gMemMB);
   //assert(infoGlobal->memleft >= size);

   if(infoGlobal->memleft < size)
   {
     FILE *logFile = fopen("/shared/nil/Out/memError.log", "w");
     fprintf(logFile, "Out of memory! %d\n",SCCGetNodeID());
     fclose(logFile);
     unlock(mallocLCK);
     SNETGLOBWAIT = SNETGLOBWAITVAL;
     exit(-1);
   }

   ptr = infoGlobal->freePtr;
   infoGlobal->freePtr += size;
   infoGlobal->memleft -= size;
   NO_SCRIPT_DBG("sccMallocGlob size: %zu, mem left B:%zu, KB:%f, MB:%f, returned %p, \n", size, gMemB, gMemKB, gMemMB, ptr);
   unlock(mallocLCK);
   return ptr;  
}

void *SCC_Malloc_Ptr(size_t size)
{
  block_t *curr, *prev;
  size_t nunits;

  nunits = (size+sizeof(block_t)-1)/sizeof(block_t) + 1;
  
  prev = freeList;

  for (curr = prev->next; ; prev = curr, curr = curr->next) {
    /* big enough */
    if (curr->size >= nunits) { 
      /* exactly */
      if (curr->size == nunits){ 
        prev->next = curr->next;
      } else { /* allocate tail end */
        curr->size -= nunits;
        curr += curr->size;
        curr->size = nunits;
      }
      freeList = prev;
      curr->coreId = myId; //physical id
      return (void *)(curr+1);
    }
    /* wrapped around free list none left*/
    if (curr == freeList){ 
        return NULL;
    }
  }
}

void *SCCMallocPtr(size_t size)
{
  DBG_ADR("malloc start\n");
  void *res;
  
  pthread_mutex_lock(&malloc_lock);
  res = SCC_Malloc_Ptr(size);
  pthread_mutex_unlock(&malloc_lock);

  if (res == NULL){
    SCC_Free_Ptr_rpc_to_local();
    pthread_mutex_lock(&malloc_lock);
    res = SCC_Malloc_Ptr(size);
    pthread_mutex_unlock(&malloc_lock);
  }
  
  if (res == NULL){
    block_t *ptr;
    if(size > (6*1024*1024)){
      ptr = SCCMallocPtrGlobal(size*2);
      ptr->size = (size*2)/sizeof(block_t);
    } else {
      ptr = SCCMallocPtrGlobal((6*1024*1024)); // multiple of block_t
      ptr->size = (6*1024*1024)/sizeof(block_t);
    }
    ptr->coreId = myId; //physical id
    
    pthread_mutex_lock(&malloc_lock);
    SCC_Free_Ptr(ptr+1);
    res = SCC_Malloc_Ptr(size);
    pthread_mutex_unlock(&malloc_lock);
  }
  if(res == NULL){
    fprintf(stderr,"Could not allocate Memory!\n");
    exit(EXIT_FAILURE);
  }
  DBG_ADR("malloc finish\n");
  PRT_FR_DBG("malloc returned %p\n\n",res);
  return res;
}
/* free: put block p in free list */
void SCC_Free_Ptr(void *p)
{
  block_t *block, *curr;
  
  block = (block_t *)p - 1; /* point to block header */
  curr = freeList;
  
  if (freeList == NULL) {
    freeList = block;
    freeList->next = freeList;
    return;
  }
  
  while(!(block > curr && block < curr->next)){
    if (curr >= curr->next && (block > curr || block < curr->next)){
      break; /* freed block at start or end of arena */
    }
    curr = curr->next;
  }
  /* join to upper nbr */    
  if (block + block->size == curr->next) { 
    block->size += curr->next->size;
    block->next = curr->next->next;
  } else {
    block->next = curr->next;
  }
  /* join to lower nbr */
  if (curr + curr->size == block) { 
    curr->size += block->size;
    curr->next = block->next;
  } else {
    curr->next = block;
  }
  freeList = curr;
}

void SCC_Free_Ptr_rpc(int idx, block_t* val){ 
  PRT_FR_DBG("\t $\n");
  int value=-1;
  while(value != 0){
    atomic_incR(&atomic_inc_regs[CORES+idx],&value);
  }
  PRT_FR_DBG("\t - ");
  
  val->next=mem_free_arr[idx].list;
  mem_free_arr[idx].list = val;
  
  atomic_writeR(&atomic_inc_regs[CORES+idx],0);
  PRT_FR_DBG(".\n");
}

// myId is physical location
void SCC_Free_Ptr_rpc_to_local(){ 
  int value=-1;
  while(value != 0){
    atomic_incR(&atomic_inc_regs[CORES+myId],&value);
  }
  PRT_FR_DBG("\t - ");
  block_t *node = mem_free_arr[myId].list;
  block_t *next;
  mem_free_arr[myId].list = NULL;
  atomic_writeR(&atomic_inc_regs[CORES+myId],0);
    
  pthread_mutex_lock(&malloc_lock);
  size_t frd=0;
  while(node != NULL){
    next = node->next;
    frd += node->size;
    SCC_Free_Ptr((block_t*)node + 1);
    node = next;
  }
  printf("Freed RPC_TO_LOCAL %zu\n",frd);
  pthread_mutex_unlock(&malloc_lock);
  PRT_FR_DBG(".\n");  
}

void SCCFreePtr(void *p){ 
  if( (p == NULL) || (!( (start<p) && (p<end) )) ){
    free(p);
    return;
  }
  
  block_t *block = (block_t*) p - 1;
  int id = block->coreId;
  if(id == myId){
    pthread_mutex_lock(&malloc_lock);
    SCC_Free_Ptr(p);
    pthread_mutex_unlock(&malloc_lock);
  } else {
    PRT_FR_DBG("Will free %p on core %d ",p,id);
    SCC_Free_Ptr_rpc(id,block);
  }
}

void *SCCFirstMalloc(void){
  pthread_mutex_lock(&malloc_lock);
  DBG_ADR("sizeof(block_t) %zu\n",sizeof(block_t));
  if(SCCIsMaster()){
    // 32 MB chunk at first time 
    freeList = SCCMallocPtrGlobal((32*1024*1024)-(sizeof(memInfoG)+(MBXSZ*SCCGetNumCores()))); 
    freeList->next = freeList;
    freeList->size = ((32*1024*1024)-(sizeof(memInfoG)+(MBXSZ*SCCGetNumCores())))/ sizeof(block_t);
    freeList->coreId = myId; //physical id
    
  } else {
    // 8 MB chunk at first time 
    freeList = SCCMallocPtrGlobal(8*1024*1024); 
    freeList->next = freeList;
    freeList->size = (8*1024*1024)/sizeof(block_t);
    freeList->coreId = myId; //physical id
  }
  pthread_mutex_unlock(&malloc_lock);
}

void SCCMallocStop(void)
{
  NO_SCRIPT_DBG("****************************\nsccmalloc stop at %f\n\
  Global size B:%zu, KB:%f, MB:%f\n\n\n",SCCGetTime(),gMemB,gMemKB,gMemMB);
  
  munmap(local, SHM_MEMORY_SIZE);
}

int DCMflush(){}
