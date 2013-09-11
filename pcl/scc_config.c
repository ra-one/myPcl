// ------------------------------------------------------------------------------------------------
// 
// ****
// ****  (C) Copyright 2006, 2007 by Intel Corporation
// ****      proprietary and confidential
// ****
// 
// Project       : bareMetal BIOS
// File name     : config.h
// Author        : mriepen
// Date          : 2008-06-24
// Revision      : 1.01
// 
// Description   : Header file for config.c
// 
// Revision history:
// 
// mri 1.01 2008-06-24
// - Initial implementation
// 
// ------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include "scc_config.h"

// Variables
int NCMDeviceFD; // File descriptor for non-cachable memory (e.g. config regs).
int MPBDeviceFD; // File descriptor for message passing buffers.
t_vcharp mpbAdr[47];

void setmpbAdr(){
  mpbAdr[0] = (void*) 0xb785d000; mpbAdr[1] = (void*) 0xb7857000; mpbAdr[2] = (void*) 0xb7852000;
  mpbAdr[3] = (void*) 0xb784d000; mpbAdr[4] = (void*) 0xb7848000; mpbAdr[5] = (void*) 0xb7843000;
  mpbAdr[6] = (void*) 0xb783e000; mpbAdr[7] = (void*) 0xb7839000; mpbAdr[8] = (void*) 0xb7834000;
  mpbAdr[9] = (void*) 0xb782f000; mpbAdr[10] = (void*) 0xb782a000; mpbAdr[11] = (void*) 0xb7825000;
  mpbAdr[12] = (void*) 0xb7820000; mpbAdr[13] = (void*) 0xb781b000; mpbAdr[14] = (void*) 0xb7816000;
  mpbAdr[15] = (void*) 0xb7811000; mpbAdr[16] = (void*) 0xb780c000; mpbAdr[17] = (void*) 0xb7807000;
  mpbAdr[18] = (void*) 0xb7802000; mpbAdr[19] = (void*) 0xb77fd000; mpbAdr[20] = (void*) 0xb77f8000;
  mpbAdr[21] = (void*) 0xb77f3000; mpbAdr[22] = (void*) 0xb77ee000; mpbAdr[23] = (void*) 0xb77e9000;
  mpbAdr[24] = (void*) 0xb77e4000; mpbAdr[25] = (void*) 0xb77df000; mpbAdr[26] = (void*) 0xb77da000;
  mpbAdr[27] = (void*) 0xb77d5000; mpbAdr[28] = (void*) 0xb77d0000; mpbAdr[29] = (void*) 0xb77cb000;
  mpbAdr[30] = (void*) 0xb77c6000; mpbAdr[31] = (void*) 0xb77c1000; mpbAdr[32] = (void*) 0xb77bc000;
  mpbAdr[33] = (void*) 0xb77b7000; mpbAdr[34] = (void*) 0xb77b2000; mpbAdr[35] = (void*) 0xb77ad000;
  mpbAdr[36] = (void*) 0xb77a8000; mpbAdr[37] = (void*) 0xb77a3000; mpbAdr[38] = (void*) 0xb779e000;
  mpbAdr[39] = (void*) 0xb7799000; mpbAdr[40] = (void*) 0xb7794000; mpbAdr[41] = (void*) 0xb778f000;
  mpbAdr[42] = (void*) 0xb778a000; mpbAdr[43] = (void*) 0xb7785000; mpbAdr[44] = (void*) 0xb7780000;
  mpbAdr[45] = (void*) 0xb777b000; mpbAdr[46] = (void*) 0xb7776000; mpbAdr[47] = (void*) 0xb7771000;
}
// InitAPI opens the RCKMEM device drivers. This routine needs to be invoked
// once before using any other API functions! The successmessage can be disabled.
// 
// Parameter: printMessages (0: No messages / 1: Messages enabled)
// Return value: %
// 
void InitAPI(int printMessages) {
  //setmpbAdr();
  // Open driver device "/dev/rckncm" for memory mapped register access
  // or access to other non cachable memory locations...
  if ((NCMDeviceFD=open("/dev/rckncm", O_RDWR|O_SYNC))<0) {
    perror("open");
    exit(-1);
  }

  // Open driver device "/dev/rckmpb" for message passing buffer access...
  if ((MPBDeviceFD=open("/dev/rckmpb", O_RDWR))<0) {
      perror("open");
      exit(-1);
  }
  // Success message
  if (printMessages) printf("Successfully opened RCKMEM driver devices!\n");
}

// SetConfigBit writes a bit to a specified config register using read-modify-write. Only use
// function to access memory locations that are not (!) performance critical (e.g. Tile-ID).
// Use MallocConfigReg() function for performance critical memory locations!
// 
// Parameter: ConfigAddr                - Address of configuration register...
//            BitPos                    - Bit position within config register to set/reset
//            BitValue                  - Value to write to specified bit...
// 
void SetConfigBit(unsigned int ConfigAddr, int BitPos, int BitValue) {
  int Register = ReadConfigReg(ConfigAddr);
  if (DEBUG) printf("RMW Read: %0x...\n", Register);
  if (BitValue) {
    Register = Register | (1<<BitPos);
  } else {
    Register = Register & ~(1<<BitPos);
  }
  if (DEBUG) printf("RMW Write: %0x...\n", Register);
  SetConfigReg(ConfigAddr, Register);
}

// SetConfigReg writes a value to a specified config register using a single write. Only use
// function to access memory locations that are not (!) performance critical (e.g. Tile-ID).
// Use MallocConfigReg() function for performance critical memory locations!
// 
// Parameter: ConfigAddr                - Address of configuration register...
//            RegValue                  - Value to write to specified register...
// 
void SetConfigReg(unsigned int ConfigAddr, int RegValue) {
  t_vcharp MappedAddr;
  unsigned int alignedAddr = ConfigAddr & (~(getpagesize()-1));
  unsigned int pageOffset = ConfigAddr - alignedAddr;

  MappedAddr = (t_vcharp) mmap(NULL, getpagesize(), PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
  if (MappedAddr == MAP_FAILED) {
          perror("mmap");
          exit(-1);
  }

  *(int*)(MappedAddr+pageOffset) = RegValue;
  munmap((void*)MappedAddr, getpagesize());
  return;
}

// ReadConfigReg reads a value from a specified config register using a single read. Only use
// function to access memory locations that are not (!) performance critical (e.g. Tile-ID).
// Use MallocConfigReg() function for performance critical memory locations!
// 
// Parameter: ConfigAddr                - Address of configuration register...
// 
// Return value: Content of the specified config register
// 
int ReadConfigReg(unsigned int ConfigAddr) {
  int result;
  t_vcharp MappedAddr;
  unsigned int alignedAddr = ConfigAddr & (~(getpagesize()-1));
  unsigned int pageOffset = ConfigAddr - alignedAddr;

  MappedAddr = (t_vcharp) mmap(NULL, getpagesize(), PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
  if (MappedAddr == MAP_FAILED) {
          perror("mmap");
          exit(-1);
  }

  result = *(int*)(MappedAddr+pageOffset);
  munmap((void*)MappedAddr, getpagesize());
  return result;
}

// MallocConfigReg performs a memory map operation on ConfigAddr (physical address) and
// returns a virtual address that can be used in the application. Use this function to
// allocate memory locations that you access frequently!
// 
// Parameter: ConfigAddr                - Physical address of configuration register.
// 
// Return value: ConfigRegVirtualAddr   - Virtual address of configuration register.
// 
int* MallocConfigReg(unsigned int ConfigAddr) {
  t_vcharp MappedAddr;
  unsigned int alignedAddr = ConfigAddr & (~(getpagesize()-1));
  unsigned int pageOffset = ConfigAddr - alignedAddr;

  MappedAddr = (t_vcharp) mmap(NULL, getpagesize(), PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
  if (MappedAddr == MAP_FAILED) {
          perror("mmap");
          exit(-1);
  }

  return (int*)(MappedAddr+pageOffset);
}

// FreeConfigReg unmaps a memory location that has been mapped with the MallocConfigReg()
// function...
// 
// Parameter: ConfigRegVirtualAddr      - Virtual address of configuration register.
// 
void FreeConfigReg(int* ConfigRegVirtualAddr) {
  t_vcharp MappedAddr;
  unsigned int alignedAddr = (int)ConfigRegVirtualAddr & (~(getpagesize()-1));
  munmap((void*)alignedAddr, getpagesize());
  return;
}

// MPBalloc allocates MPBSIZE bytes of MessagePassing buffer Memory at MPB_ADDR(x,y,core).
// 
// Parameter: MPB                   - Pointer to MPB area (return value, virtal address)
//            x,y,core              - Position of tile (x,y) and core...
//

/*
void MPBalloc(t_vcharp *MPB, int x, int y, int core, unsigned char isOwnMPB) {
  t_vcharp MappedAddr;
  unsigned int alignedAddr = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) & (~(getpagesize()-1));
  unsigned int pageOffset = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) - alignedAddr;
  int node_loc;
  node_loc = PID(x, y, core);
  
  
  if ((x>=NUM_COLS) || (y>=NUM_ROWS) || (core>=NUM_CORES)) {
    printf("MPBalloc: Invalid coordinates (x=%0d, y=%0d, core=%0d)\n", x,y,core);
    *MPB = NULL;
    return;
  }
  MappedAddr = (t_vcharp) mmap((void*)mpbAdr[node_loc], MPBSIZE, PROT_WRITE|PROT_READ, MAP_SHARED | MAP_FIXED, MPBDeviceFD, alignedAddr);
  //MappedAddr = (t_vcharp) mmap(NULL, MPBSIZE, PROT_WRITE|PROT_READ, MAP_SHARED, MPBDeviceFD, alignedAddr);
  printf("node_loc %d, MappedAddr %p %X, alignedAddr %lu\n",node_loc,MappedAddr,MappedAddr,alignedAddr);
  if (MappedAddr == MAP_FAILED)
  {
          perror("mmap");
          exit(-1);
  }

  *MPB = MappedAddr+pageOffset;
}
*/
void MPBalloc(t_vcharp *MPB, int x, int y, int core, unsigned char isOwnMPB) {
  t_vcharp MappedAddr;
  unsigned int alignedAddr = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) & (~(getpagesize()-1));
  unsigned int pageOffset = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) - alignedAddr;
  
  if ((x>=NUM_COLS) || (y>=NUM_ROWS) || (core>=NUM_CORES)) {
    printf("MPBalloc: Invalid coordinates (x=%0d, y=%0d, core=%0d)\n", x,y,core);
    *MPB = NULL;
    return;
  }
  
  MappedAddr = (t_vcharp) mmap(NULL, MPBSIZE, PROT_WRITE|PROT_READ, MAP_SHARED, MPBDeviceFD, alignedAddr);
  //printf("core %d X %d Y %d, MappedAddr %p %X, alignedAddr %lu\n",core,x,y,MappedAddr,MappedAddr,alignedAddr);
  //local = mmap((void*)local, SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, LOCAL_LUT << 24);
  if (MappedAddr == MAP_FAILED)
  {
          perror("mmap");
          exit(-1);
  }

  *MPB = MappedAddr+pageOffset;
}


// MPBunalloc unallocates an allocated MPB area.
// 
// Parameter: MPB             - Pointer to MPB area (virtual address)
// 
void MPBunalloc(t_vcharp *MPB) {
  munmap((void*)*MPB, MPBSIZE);
  *MPB = NULL;
}

// readFpgaGrb allows to read a register value from the SIF FPGA register bank
// by passing the register bank address as specified in the SIF FPGA documentation.
// Some addresses have already been specified in config.h!
// 
// Parameter: addr           - Register address
//            value ref      - pointer to the unsigned int variable that will be
//                             updated with the result...
// 
void readFpgaGrb(int addr, unsigned int* value) {
  *value=ReadConfigReg(FPGA_BASE+addr);
}

// writeFpgaGrb allows to write a register value to the SIF FPGA register bank
// by passing the register bank address as specified in the SIF FPGA documentation
// and the value to be written...
// Some addresses have already been specified in config.h!
// 
// Parameter: addr           - Register address
//            value          - Value to be written
// 
void writeFpgaGrb(int addr, unsigned int value) {
  SetConfigReg(FPGA_BASE+addr,value);
}

// readStatus reads the "human readable" value of a DVFS register. For that purpose
// it needs the register address and the factor to process the RAW register value.
// Some addresses have already been specified in config.h! Please note that these defines
// already contain address AND factor. So, the routine can be called like this:
// EXAMPLE: double tmp = readStatus(DVFS_U1V65ADJ);
// 
// Parameter: addr           - Register address
//            factor         - Value to be written
// 
// Return value: double      - Human readable DVFS register content
// 
double readStatus(int addr, double factor) {
  unsigned int tmp;
  readFpgaGrb(addr, &tmp);
  return (double)tmp * factor;
}
