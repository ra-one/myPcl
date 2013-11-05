#include <fcntl.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>
#include <malloc.h>


#include "scc.h"
#include <pthread.h>

#define PRT_ADR //
#define PRT_MALLOC //
#define PRT_MALLOCX //
#define PRT_MALLOCX1 //
//#define PRT_MALLOCX1 fprintf

//#define USE_MALLOC_HOOK

//mutex variable used to lock SCCMallocPtr, because of the possibility of different threads running on the same core
pthread_mutex_t malloc_lock;

typedef union block {
  struct {
    union block *next;
    size_t size;
  } hdr;
  uint32_t align;   // Forces proper allignment
} block_t;

static void *local;
static int mem;
static block_t *freeList;

uintptr_t start;

/*
 * SCCMallocInit creates a new mapping for the SHM and sets the "addr" pointer to the beginning address of this SHM
 */

void SCCMallocInit(uintptr_t *addr,int numMailboxes)
{
  //init mutex variable for the SCCMallocPtr function
  pthread_mutex_init(&malloc_lock, NULL);
  
  pthread_mutex_lock(&malloc_lock);
  
  mem = open("/dev/rckncm", O_RDWR|O_SYNC);
  PRT_ADR("mem: %i\n", mem);
  if (mem < 0) {
		fprintf(stderr, "Opening /dev/rckdyn011 failed!\n");
    exit(-1);
  }	

  /*
   * create a new mapping for the SHM
   * if the addr ptr. is unset then the calling node is the MASTER and has to create the mapping and set the start-address
   * if the addr ptr. is set then the calling node is a WORKER and just has to map the memory to a fixed start-address gotten from the MASTER
   *
   */
  unsigned int alignedAddr = (SHM_ADDR) & (~(getpagesize()-1));
  unsigned int pageOffset = (SHM_ADDR) - alignedAddr;
  
  PRT_ADR("alignedAddr: %zu, pageOffset: %zu",alignedAddr,pageOffset);
  
	if (*addr==0x0){
		PRT_ADR("MASTER MMAP\n\n");
		local = mmap(NULL, 		SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, alignedAddr);
		
		if (local == NULL) { fprintf(stderr, "Couldn't map memory!\n"); exit(-1); }
		else	munmap(local, SHM_MEMORY_SIZE);
		
		local = mmap((void*)local, 	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, alignedAddr);
		
		if (local == NULL) { fprintf(stderr, "Couldn't map memory!\n"); exit(-1); }
		*addr=local;
  }else{
		PRT_ADR("WORKER MMAP\n\n");
		local=*addr;	
		local = mmap((void*)local,     	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, alignedAddr);
		if (local == NULL) { fprintf(stderr, "Couldn't map memory!"); exit(-1); }
  }  

  PRT_ADR("sccmalloc: addr: %p\n",*addr);
  
  //calculate the start-address in the SHM, depending on the max. number of participating WORKERS and the ID of the calling WORKER
  if(SCCIsMaster()){
    freeList = local+(48*numMailboxes);
    freeList->hdr.size = ((SHM_MEMORY_SIZE/numMailboxes)-(48*numMailboxes))/ sizeof(block_t);
  } else {
    //get logical id and offset into memory to get local start
    freeList = local+MEMORY_OFFSET(SCCGetNodeRank());  
    freeList->hdr.size = (SHM_MEMORY_SIZE/numMailboxes)/ sizeof(block_t);
  }
  	
	
  PRT_ADR("SHM_MEMORY_SIZE: %zu, sizeof(block_t): %d\n",SHM_MEMORY_SIZE,sizeof(block_t));
	PRT_ADR("\nsccmalloc: freeList %p\n",freeList);
  
  freeList->hdr.next = freeList;
  PRT_ADR("sccmalloc: next: %p\n",freeList->hdr.next);
  
  //freeList->hdr.size = (SHM_MEMORY_SIZE/numMailboxes)/ sizeof(block_t);
  PRT_ADR("sccmalloc: size: %zu\n\n",freeList->hdr.size);
  
  // this is used in free to see if address is given by normal malloc or sccmalloc
  start = *addr; 
  //PRT_ADR("start %p, freelist %p, size %zu\n",start,freeList,freeList->hdr.size);
  fprintf(stderr,"start %p, freelist %p, size %zu\n",start,freeList,freeList->hdr.size);
  
  pthread_mutex_unlock(&malloc_lock);  
}

void *SCCGetLocal(void){
	return local;
}

void SCCMallocStop(void)
{
  fprintf(stderr, "****************************\nsccmalloc stop at %f, size %zu\n\n\n",SCCGetTime(),freeList->hdr.size);
  munmap(local, SHM_MEMORY_SIZE);
  close(mem);
}

/*
 * SCCMallocPtr is used to allocate memory in the SHM
 */

void *SCCMallocPtr(size_t size)
{
  size_t nunits;
  block_t *curr, *prev, *new;
	pthread_mutex_lock(&malloc_lock);

  if (freeList == NULL){
    fprintf(stderr, "Couldn't allocate memory freelist is NULL!\n");
    pthread_mutex_unlock(&malloc_lock);
    exit(-1);
    return NULL;
  }
  
  prev = freeList;
  curr = prev->hdr.next;
  nunits = (size + sizeof(block_t) - 1) / sizeof(block_t) + 1;
  PRT_MALLOCX("SCCMallocPtr is called for size: %zu, nunits %zu\n",size,nunits);
  do {
		/* the following debugging printout is very useful to check if there is a Problem 
		 * with the memory allocation, usually forced by a not allowed write to the SHM 
		 * either by a normal malloc or a manual write to an address in the SHM
		 */
     //size and next will always be same as there is only one element in free list
     PRT_MALLOCX("prev->hdr.size %zu,prev->hdr.next %p, curr->hdr.size %zu, curr->hdr.next %p\n",prev->hdr.size,prev->hdr.next,curr->hdr.size,curr->hdr.next);
			if (curr->hdr.size >= nunits){
				if (curr->hdr.size == nunits){
					if (prev == curr){
						PRT_MALLOCX("SET prev TO NULL in malloc\n");
						prev = NULL;
					}else{
						prev->hdr.next = curr->hdr.next;
					}
				} else if (curr->hdr.size > nunits){
					new = curr + nunits;
					*new = *curr;
					new->hdr.size -= nunits;
					curr->hdr.size = nunits;
					if (prev == curr) prev = new;
						prev->hdr.next = new;
				}
				freeList = prev;
				pthread_mutex_unlock(&malloc_lock);
        PRT_MALLOC(stderr,"SCCMalloc: returns %p at time: %f\n",(void*) (curr + 1),SCCGetTime());
        PRT_MALLOCX1(stderr,"SCCMalloc: returns %p current size: %zu\n",(void*) (curr + 1),freeList->hdr.size);
        return (void*) (curr + 1);
			}
		} while (curr != freeList && (prev = curr, curr = curr->hdr.next));

		pthread_mutex_unlock(&malloc_lock);

		fprintf(stderr, "Couldn't allocate memory: not enough available!\n");
    exit(-1);
		return NULL;
}

/*
 * SCCFreePtr is used to free memory in the SHM
 */
void SCCFreePtr(void *p){}
void SCCFreePtr1(void *p)
{
  // this deals with NULL or some normal malloc trying to be freed by this function
  // specially from snet-rts lexer.c
  if(p == NULL || start > p) return;
  /*
  if(start > p){
    PRT_MALLOC(stderr, "SCCFree going to call free of %p at %f\n",p,SCCGetTime());
    free(p);    
    return;
  }
  */
  
  PRT_MALLOC(stderr, "SCCFree pointer %p at %f\n",p,SCCGetTime());
  pthread_mutex_lock(&malloc_lock);
  
  block_t *block = (block_t*) p - 1,
          *curr = freeList;

  if (freeList == NULL) {
    freeList = block;
    freeList->hdr.next = freeList;
    pthread_mutex_unlock(&malloc_lock);
    return;
  }

  while (!(block > curr && block < curr->hdr.next)) {
    if (curr >= curr->hdr.next && (block > curr || block < curr->hdr.next)) break;
    curr = curr->hdr.next;
  }


  if (block + block->hdr.size == curr->hdr.next) {
    block->hdr.size += curr->hdr.next->hdr.size;
    if (curr == curr->hdr.next) block->hdr.next = block;
    else block->hdr.next = curr->hdr.next->hdr.next;
  } else {
    block->hdr.next = curr->hdr.next;
  }

  if (curr + curr->hdr.size == block) {
    curr->hdr.size += block->hdr.size;
    curr->hdr.next = block->hdr.next;
  } else {
    curr->hdr.next = block;
  }

  freeList = curr;
  
  pthread_mutex_unlock(&malloc_lock);
}


/*
 * used to flush the whole L2-cache
 */
int DCMflush() {
   //flushes the whole L2 cache
   write(mem,0,65536);
//   write(mem,0,0);
   return 1;
}


