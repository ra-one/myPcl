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
#include <pthread.h>

#define lMemB  infoLocal->memleft
#define lMemKB (double)(infoLocal->memleft/1024)
#define lMemMB (double)(infoLocal->memleft/(1024*1024))
#define MemB   info->memleft
#define MemKB  (double)(info->memleft/1024)
#define MemMB  (double)(info->memleft/(1024*1024))

#define PRT_ADR //

pthread_mutex_t malloc_lock;


static void *local;
static int mem;

uintptr_t start;

static size_t memleft;
static char *freePtr;

typedef struct memInfo{
  char *freePtr;
  size_t memleft;
}memInfo;

pthread_mutex_t malloc_lock;

struct memInfo *info,*infoLocal,tempInfo;

/*
 * SCCMallocInit creates a new mapping for the SHM and sets the "addr" pointer to the beginning address of this SHM
 */

void SCCMallocInit(uintptr_t *addr,int numMailboxes)
{  
  pthread_mutex_init(&malloc_lock, NULL);
  
  pthread_mutex_lock(&malloc_lock);
  
  mem = open("/dev/rckncm", O_RDWR|O_SYNC);
  PRT_ADR("mem: %i\n", mem);
  if (mem < 0) {
		fprintf(stderr, "Opening /dev/rckncm failed!\n");
    exit(-1);
  }

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
      void *mmap(void *addr, size_t length, int prot, int flags,int fd, off_t offset);
		local = mmap(NULL, 		SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, alignedAddr);
		
		if (local == NULL) { fprintf(stderr, "Couldn't map memory!\n"); exit(-1); }
		else	munmap(local, SHM_MEMORY_SIZE);
		
		local = mmap((void*)local, 	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, alignedAddr);
   	
		if (local == NULL) { fprintf(stderr, "Couldn't map memory!\n"); exit(-1); }
    
		*addr=local;
  }else{
		PRT_ADR("WORKER MMAP\n\n");
		local=*addr;	
		local = mmap((void*)local, SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, alignedAddr);
   	if (local == NULL) { fprintf(stderr, "Couldn't map memory!"); exit(-1); }
  }  

  start = *addr;

  info = malloc(sizeof(memInfo*));
  infoLocal = malloc(sizeof(memInfo*));
  
  if(SCCIsMaster()){
    tempInfo.freePtr = local+10+((48*numMailboxes) * 2);
    tempInfo.memleft = (SHM_MEMORY_SIZE-(10+(48*numMailboxes)*2));
    memcpy((void*)(local+((48*numMailboxes) * 2)),(const void*)&tempInfo,sizeof(memInfo)); 
    info->freePtr=NULL;
    info->memleft = 0;
  } else {
    info->freePtr=NULL;
    info->memleft = 0;
  }
  
  infoLocal = (local+((48*numMailboxes) * 2));
  fprintf(stderr,"start %p, End %p,freelist %p, size B:%zu, KB:%f, MB:%f\n",start,(start+SHM_MEMORY_SIZE),infoLocal->freePtr,lMemB,lMemKB,lMemMB); 
  pthread_mutex_unlock(&malloc_lock);
}

/*
 * SCCMallocPtr is used to allocate memory in the SHM
 */
void *SCCMallocPtrLocal(size_t size)
{
   void* ptr;
   lock(29);
   assert(infoLocal->memleft > size+1);
   ptr = infoLocal->freePtr;
   infoLocal->freePtr += size+1;
   infoLocal->memleft -= size+1;
   //fprintf(stderr,"sccMallocLocal size: %zu, returned: %p, mem left %zuB,%fKB %fMB\n",size,ptr,lMemB,lMemKB,lMemMB);
   unlock(29);
   return ptr;  
}
void *SCCMallocPtr(size_t size)
{
   void* ptr;

   pthread_mutex_lock(&malloc_lock);
   if (info->memleft < size+1){
    info->freePtr = SCCMallocPtrLocal(32*1024*1024); // 32 MB chunk at a time 
    info->memleft = ((32*1024*1024)-10); // -10 to leave small space at the end
   }
   assert(info->memleft > size+1);
   ptr = info->freePtr;
   info->freePtr += size+1;
   info->memleft -= size+1;
   //fprintf(stderr,"sccMalloc size: %zu, returned: %p, mem left %zuB,%fKB %fMB\n",size,ptr,MemB,MemKB,MemMB);
   pthread_mutex_unlock(&malloc_lock);
   return ptr;  
}

void *SCCGetLocal(void){
	return local;
}

void SCCMallocStop(void)
{
  //fprintf(stderr, "****************************\nsccmalloc stop at %f, size %zu\n\n\n",SCCGetTime(),freeList->hdr.size);
  fprintf(stderr, "****************************\nsccmalloc stop at %f\n\
  Info  size B:%zu, KB:%f, MB:%f\n\
  Local size B:%zu, KB:%f, MB:%f\n\n\n",SCCGetTime(),MemB,MemKB,MemMB,lMemB,lMemKB,lMemMB);
  munmap(local, SHM_MEMORY_SIZE);
  close(mem);
}

void *SCCVMallocPtr(size_t size){}

void SCCFreePtr(void *p){ p = NULL;}
int DCMflush(){}