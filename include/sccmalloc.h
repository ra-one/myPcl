#ifndef SCCMALLOC_H
#define SCCMALLOC_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

//#define LOCAL_LUT   0x14
#define LOCAL_LUT   0x29
#define REMOTE_LUT  (LOCAL_LUT + local_pages)


typedef struct {
  unsigned char node, lut;
  uint32_t offset;
} lut_addr_t;

lut_addr_t SCCPtr2Addr(void *p);
void *SCCAddr2Ptr(lut_addr_t addr);


void SCCInit(uintptr_t  *addr);
void SCCStop(void);
void *SCCGetlocal(void);
void *SCCMallocPtr(size_t size);
void SCCFree(void *p);
void SCCFreePtr(void *p);

int DCMflush();

#endif
