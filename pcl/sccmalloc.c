#include <fcntl.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>


#include "scc.h"
#include <pthread.h>
//#define PRT_DBG printf
//#define PRT_DBG1 printf
#define PRT_ADR printf

//mutex variable used to lock SCCMallocPtr, because of the possibility of different threads running on the same core
pthread_mutex_t malloc_lock;

typedef union block {
  struct {
    union block *next;
    size_t size;
  } hdr;
  uint32_t align;   // Forces proper allignment
} block_t;

unsigned char local_pages=MAX_PAGES;
int node_ID;

static void *local;
static int mem;
static block_t *freeList;

uintptr_t start;
/*
 * returns the LUT-entry where an address is located in the SHM
 */
lut_addr_t SCCPtr2Addr(void *p)
{
  uint32_t offset;
  unsigned char lut;
  if (local <= p && p <= local + SHM_MEMORY_SIZE) {
    offset = (p - local) % PAGE_SIZE;
    lut = LOCAL_LUT + (p - local) / PAGE_SIZE;
  } else {
    fprintf(stderr, "Invalid pointer\n");
  }

  lut_addr_t result = {node_location, lut, offset};
  return result;
}
/*
 * returns the address to an given LUT-entry
 */
void *SCCAddr2Ptr(lut_addr_t addr)
{
  if (LOCAL_LUT <= addr.lut && addr.lut < LOCAL_LUT + local_pages) {
    return (void*) ((addr.lut - LOCAL_LUT) * PAGE_SIZE + addr.offset + local);
  } else {
    fprintf(stderr, "Invalid SCC LUT address\n");
  }

  return NULL;
}

/*
 * SCCMallocInit creates a new mapping for the SHM and sets the "addr" pointer to the beginning address of this SHM
 */

void SCCMallocInit(uintptr_t *addr,int numMailboxes)
{
  node_ID = SCCGetNodeRank(); //get logical id
  // Open driver device "/dev/rckdyn011" to map memory in write-through mode 
  //mem = open("/dev/rckdcm", O_RDWR|O_SYNC);
  //mem = open("/dev/rckdyn010", O_RDWR|O_SYNC); //works in wrapper on master
  mem = open("/dev/rckncm", O_RDWR|O_SYNC); //works in wrapper on master
  PRT_DBG("mem: %i\n", mem);
  if (mem < 0) {
		fprintf(stderr, "Opening /dev/rckdyn011 failed!\n");
  }	

  /*
   * create a new mapping for the SHM
   * if the addr ptr. is unset then the calling node is the MASTER and has to create the mapping and set the start-address
   * if the addr ptr. is set then the calling node is a WORKER and just has to map the memory to a fixed start-address gotten from the MASTER
   *
   */
	if (*addr==0x0){
		PRT_ADR("MASTER MMAP\n\n");
		local = mmap(NULL, 		SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, LOCAL_LUT << 24);
		
		if (local == NULL) fprintf(stderr, "Couldn't map memory!\n");
		else	munmap(local, SHM_MEMORY_SIZE);
		
		local = mmap((void*)local, 	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, LOCAL_LUT << 24);
		
		if (local == NULL) fprintf(stderr, "Couldn't map memory!\n");
		*addr=local;
  }else{
		PRT_ADR("WORKER MMAP\n\n");
		local=*addr;	
		local = mmap((void*)local,     	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, LOCAL_LUT << 24);
		if (local == NULL) fprintf(stderr, "Couldn't map memory!");
  }  

  PRT_ADR("sccmalloc: addr: %p\n",*addr);
  

  //calculate the start-address in the SHM, depending on the max. number of participating WORKERS and the ID of the calling WORKER
  if(RC_COREID[node_ID] == master_ID){
    freeList = local+(48*numMailboxes);
    /*int *a = local+MEMORY_OFFSET(2)-200;
    int *a1 = local+MEMORY_OFFSET(2)-175;
    int *a2 = local+MEMORY_OFFSET(2)-150;
    int *a3 = local+MEMORY_OFFSET(2)-100;
    int *a4 = local+MEMORY_OFFSET(2);
    int *a5 = local+MEMORY_OFFSET(2)+100;
    int *a6 = local+MEMORY_OFFSET(2)+200;
    int *a7 = local+MEMORY_OFFSET(2)+300;
    int *b = local+MEMORY_OFFSET(3);
    *a = 990;
    *a1 = 991;
    *a2 = 992;
    *a3 = 993;
    *a4 = 994;
    *a5 = 995;
    *a6 = 996;
    *a7 = 997;
    *b = 999;
    printf("a: %p, %d\n",a,*a);
    printf("a1: %p, %d\n",a1,*a1);
    printf("a2: %p, %d\n",a2,*a2);
    printf("a3: %p, %d\n",a3,*a3);
    printf("a4: %p, %d\n",a4,*a4);
    printf("a5: %p, %d\n",a5,*a5);
    printf("a6: %p, %d\n",a6,*a6);
    printf("a7: %p, %d\n",a7,*a7);
    printf("b: %p, %d\n",b,*b);
    printf("Tot: 0x%x\n",local+SHM_MEMORY_SIZE);*/
  } else {
    freeList = local+MEMORY_OFFSET(node_ID);
  }
  	
	/*
  lut_addr_t *addr_t=(lut_addr_t*)malloc(sizeof(lut_addr_t));
  *addr_t= SCCPtr2Addr(freeList);
  PRT_DBG("LUT-entry of freelist:		%d\n",addr_t->lut);
  PRT_DBG("freelist's LUT offset: 		%u\n",addr_t->offset);
	*/
  PRT_ADR("SHM_MEMORY_SIZE: %zu, sizeof(block_t): %d\n",SHM_MEMORY_SIZE,sizeof(block_t));
	PRT_ADR("\nsccmalloc: freeList %p\n",freeList);
  
  freeList->hdr.next = freeList;
  PRT_ADR("sccmalloc: next: %p\n",freeList->hdr.next);
  
  freeList->hdr.size = SHM_MEMORY_SIZE / sizeof(block_t);
  //freeList->hdr.size = 10000;
  PRT_ADR("sccmalloc: size: %zu\n\n",freeList->hdr.size);
  
  //PRT_ADR("sccmalloc: Node_ID: %d, freelist: %p, local: %p\n\toffset: 0x%x, hdr.size: %d\n",node_ID,freeList,local,MEMORY_OFFSET(node_ID),freeList->hdr.size);
  //PRT_ADR("\nsccmalloc: freeList %p, next: %p, size :0x%x\n",freeList,freeList->hdr.next,freeList->hdr.size);
  start = *addr;
  PRT_DBG("start %x %p\n",start,start);
  
  //init mutex variable for the SCCMallocPtr function
  pthread_mutex_init(&malloc_lock, NULL);
}

void *SCCGetLocal(void){
	return local;
}

void SCCMallocStop(void)
{
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

  if (freeList == NULL) fprintf(stderr, "Couldn't allocate memory freelist is NULL!\n");
  prev = freeList;
  curr = prev->hdr.next;
  nunits = (size + sizeof(block_t) - 1) / sizeof(block_t) + 1;
  PRT_DBG("SCCMallocPtr is called for size: %zu, nunits %zu\n",size,nunits);
  do {
		/* the following debugging printout is very useful to check if there is a Problem 
		 * with the memory allocation, usually forced by a not allowed write to the SHM 
		 * either by a normal malloc or a manual write to an address in the SHM
		 */
		 //PRT_DBG("\ncurr->hdr.size:					%zu\n",curr->hdr.size);
     //size and next will always be same as there is only one element in free list
     PRT_DBG("prev->hdr.size %zu,prev->hdr.next %p, curr->hdr.size %zu, curr->hdr.next %p\n",prev->hdr.size,prev->hdr.next,curr->hdr.size,curr->hdr.next);
			if (curr->hdr.size >= nunits){
				if (curr->hdr.size == nunits){
					if (prev == curr){
						PRT_DBG("SET prev TO NULL in malloc\n");
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
        PRT_DBG1("SCCMalloc: returned %p at time: %f of size: %d 0x%x\n",(void*) (curr + 1),SCCGetTime(),size,size);
				return (void*) (curr + 1);
			}
		} while (curr != freeList && (prev = curr, curr = curr->hdr.next));

		pthread_mutex_unlock(&malloc_lock);

		fprintf(stderr, "Couldn't allocate memory: not enough available!\n");
		return NULL;
}

/*
 * SCCFreePtr is used to free memory in the SHM
 */
void SCCFreePtr1(void *p){
  if(p == NULL) return;
  
  if(start > p){
    fprintf(stderr,"\nSCCMalloc: NOT MAY MALLOC Free %p at time: %f\n",p,SCCGetTime()); 
    free(p);
    fprintf(stderr,"SCCMalloc: FREE success at %f\n",SCCGetTime()); 
    return;
  }
}

void SCCFreePtr(void *p)
{
  // this deals with NULL or some normal malloc trying to be freed by this function
  // specially from snet-rts lexer.c
  if((p == NULL) || (start > p)){
    free(p);    
    return;
  }
  
  pthread_mutex_lock(&malloc_lock);
  
  block_t *block = (block_t*) p - 1,
          *curr = freeList;

  if (freeList == NULL) {
    freeList = block;
    freeList->hdr.next = freeList;
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


