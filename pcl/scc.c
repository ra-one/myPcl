#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h> /*for uint16_t*/
#include <stdarg.h>
#include <malloc.h> /* Prototypes for __malloc_hook, __free_hook */
#include <fcntl.h> /* file handling */
#include <sys/mman.h>

#include "scc.h"

#define PRT_SYNC //
#define PRT_ADR //
#define PRT_MBX //
#define PRT_FILE //

FILE *masterFile;
FILE *logFile;
long long int requestServiced = 0;

//used to write SHM start address into MPB
uintptr_t  addr=0x0;
uintptr_t  *allMbox;

// global variables for the MPB, LUT, PINS, LOCKS and AIR
t_vcharp firstMPB;
t_vcharp locks[CORES];
AIR atomic_inc_regs[2*CORES];

// global variables for power management
static triple RC_V_MHz_cap[] = {
/* 0 */ {0.9, 0x90, 460},
/* 1 */ {0.9, 0x90, 598},
/* 2 */ {0.9, 0x90, 644},
/* 3 */ {1.0, 0xA0, 748},
/* 4 */ {1.1, 0xB0, 875},
/* 5 */ {1.2, 0xC0, 1024},
/* 6 */ {1.3, 0xD0, 1198}
};

// tile clock change words, assuming constant router ratio of 2 
static unsigned int RC_frequency_change_words[][3] = {
// rtr clock ratio, bits 25:8
/* NOP */ {    2,   0x00000000, 000}, /**/
/*  1 */  {    2,   0x000038e0, 000}, /* */
/*  2 */  {    2,   0x000070e1, 800}, /* 800 */
/*  3 */  {    2,   0x0000a8e2, 533}, /* 533 */
/*  4 */  {    2,   0x0000e0e3, 400}, /* 400 */
/*  5 */  {    2,   0x000118e4, 320}, /* 320 */
/*  6 */  {    2,   0x000150e5, 266}, /* 266 */
/*  7 */  {    2,   0x000188e6, 228}, /* 228 */
/*  8 */  {    2,   0x0001c0e7, 200}, /* 200 */
/*  9 */  {    2,   0x0001f8e8, 178}, /* 178 */
/* 10 */  {    2,   0x000230e9, 160}, /* 160 */
/* 11 */  {    2,   0x000268ea, 145}, /* 145 */
/* 12 */  {    2,   0x0002a0eb, 133}, /* 133 */
/* 13 */  {    2,   0x0002d8ec, 123}, /* 123 */
/* 14 */  {    2,   0x000310ed, 114}, /* 114 */
/* 15 */  {    2,   0x000348ee, 106}, /* 106 */
/* 16 */  {    2,   0x000380ef, 100}  /* 100 */
};
/*
static int RC_domains[6][4] = {
    { 0,  2, 12, 14}, { 4,  6, 16, 18},{ 8, 10, 20, 22},
    {24, 26, 36, 38}, {28, 30, 40, 42},{32, 34, 44, 46}
};*/

static const int RC_domains[6][4]={ // 4 5 7 0 1 3
  {0, 1, 6, 7},    {2, 3, 8, 9},    {4, 5, 10, 11},
  {12, 13, 18, 19},{14, 15, 20, 21},{16, 17, 22, 23}
};

//static int VCCADDR[] = {0x8410,0x8414,0x8418,0x8400,0x8404,0x840C};
// VCC values are shiftred by 1, as bug 328 on scc
//static int VCCADDR[] = {0x8404,0x8408,0x8410,0x8414,0x8418,0x8400}; // 0 1 3 4 5 7
static int VCCADDRP[] = {0x8414,0x8418,0x8400,0x8404,0x8408,0x8410}; // 4 5 7 0 1 3
static int* VCCADDRV[6];
static int PD[] = {4,5,7,0,1,3};
static int changeVLT = 1;

t_vintp fChange_vAddr[CORES/2];
t_vintp RPC_virtual_address; // only one RPC on chip
dom_fv  RC_current_val[RC_VOLTAGE_DOMAINS];
// array of physical core IDs for all participating cores, sorted by rank
int     RC_COREID[CORES];

int RC_voltage_level(int Fdiv);
int RC_set_frequency_divider(int tile_ID, int Fdiv);
unsigned int FID_word(int Fdiv, int tile_ID);
unsigned int VID_word(float voltage, int domain);
int get_divider(int tile_ID);
int getInt(float voltage);
double readVCC(int domain); // takes logical domain gives value for physical domain
static int read_current_frequency(int tileId);

// register for timer
int *tlo,*thi;
double startTime;

//register for voltages
int DVFS;
int *C3v3SCC, *V3v3SCC;

int node_id = -1;  // physical location
int node_rank = -1;  // logical location
int num_worker = -1; // number of workers including master
int num_wrapper = -1; // number of wrappers
int num_cores = -1; // number of mailboxes (worker + wrappers)
int master_id = -1;
char *hostFile;
int activeDomains[6];

void SCCFill_RC_COREID();
void remapLUT(int myCoreID);
void setSCCVars();

void SCCInit(){
  //variables for the MPB init
  int core,size, x, y, z, address, offset,i;
  unsigned char cpu;
  
  InitAPI(0);
 
  z = ReadConfigReg(CRB_OWN+MYTILEID);
  x = (z >> 3) & 0x0f; // bits 06:03
  y = (z >> 7) & 0x0f; // bits 10:07
  z = z & 7; // bits 02:00
  node_id = PID(x, y, z);
 
  /* Read input.txt in /shared and get all info and set local vars */
  setSCCVars();
  // master_id gets value in this function
  SCCFill_RC_COREID();
  
  /* Rmap LUT entries for shared memory */
  remapLUT(z);

  if (SCCIsMaster()){
    MPBalloc(&firstMPB, X_PID(0), Y_PID(0), Z_PID(0), 1);
    for (offset=0; offset < 0x2000; offset+=8){
      *(volatile unsigned long long int*)(firstMPB+offset) = 0;
    }
    SCCMallocInit((void *)&addr);
    ALL_DBG("scc.c: addr %p, firstMPB %p\n",(void*)addr,firstMPB);
    memcpy((void*)MALLOCADDR, (const void*)&addr, sizeof(uintptr_t));
    WAITWORKERS = WAITWORKERSVAL;
    FOOL_WRITE_COMBINE;
    MPBunalloc(&firstMPB);
  } else {
    MPBalloc(&firstMPB, X_PID(0), Y_PID(0), Z_PID(0), 0);
    ALL_DBG("scc.c: going into wait workers, WAITWORKERS = %d\n",(*((volatile int*)(firstMPB + 34))));
    while(WAITWORKERS != WAITWORKERSVAL);
    // worker waits untill it gets address to map shared memory from master
    memcpy((void*)&addr, (const void*)MALLOCADDR, sizeof(uintptr_t));
    ALL_DBG("scc.c: addr %p, firstMPB %p\n",(void*)addr,firstMPB);
    MPBunalloc(&firstMPB);
    SCCMallocInit((void *)&addr);
  } 
	
  // Timer registers
  tlo = (int *) MallocConfigReg(FPGA_BASE+0x08224);
  thi = (int *) MallocConfigReg(FPGA_BASE+0x8228);
  
  V3v3SCC = (int *) MallocConfigReg(FPGA_BASE+0x841c);
  C3v3SCC = (int *) MallocConfigReg(FPGA_BASE+0x8424);
  
  for(i=0;i<6;i++){
    VCCADDRV[i] = (int *) MallocConfigReg(FPGA_BASE+VCCADDRP[i]);
  }
  
  for (cpu = 0; cpu < 48; cpu++){
    x = X_PID(cpu);
    y = Y_PID(cpu);
    z = Z_PID(cpu);

    if (cpu == node_id) address = CRB_OWN;
    else address = CRB_ADDR(x, y);

    //LUT, PINS, LOCK allocation
    locks[cpu] = (t_vcharp) MallocConfigReg(address + (z ? LOCK1 : LOCK0));
   
   //MPB allocation
    if(cpu == 0) {
      MPBalloc(&firstMPB, x, y, z, cpu == node_id);
      ALL_DBG("scc.c: firstMPB = %p\n",firstMPB); 
    }
    //FIRST SET OF AIR
    atomic_inc_regs[cpu].counter = (int *) MallocConfigReg(air_baseE + (8*cpu));
    atomic_inc_regs[cpu].init    = (int *) MallocConfigReg(air_baseE + (8*cpu) + 4);
    
    if(SCCIsMaster()){
      *atomic_inc_regs[cpu].init = 0;
      unlock(cpu);
    }
  }

  SCCFirstMalloc();
  
  if(SCCIsMaster()){ // only master executes this code
    RPC_virtual_address = (t_vintp) MallocConfigReg(RPC_BASE);
    //fprintf(stderr,"RPCBASE %p vaddr %p\n",RPC_BASE,RPC_virtual_address);
    for (i=0; i<CORES/2; i++) {
      x = i%6; y = i/6; 
      fChange_vAddr[i] = (t_vintp) MallocConfigReg(CRB_ADDR(x,y)+TILEDIVIDER); 
    }
    // get values for current voltage and freq
    //get_divider(node_id);
    for (i=0; i<RC_VOLTAGE_DOMAINS; i++){
      int fdiv = read_current_frequency(RC_domains[i][0]);
      RC_current_val[i].current_freq_div = fdiv;
      RC_current_val[i].current_volt_level = RC_voltage_level(fdiv); 
      //printf("logical domain %d, fdiv %d, vlt %d\n",i,RC_current_val[i].current_freq_div,RC_current_val[i].current_volt_level);
    }

    //printf("freq %d, vlt %f\n",RC_frequency_change_words[RC_current_val[0].current_freq_div][2],RC_V_MHz_cap[RC_current_val[0].current_volt_level].volt);
    
    // set all inactive domains to minimum if DVFS is enabled
    //if(DVFS) set_min_freq();
  }
  
  //assign mailbox address into array
  allMbox = SCCMallocPtr (sizeof(uintptr_t)*num_cores);  
  uintptr_t temp = addr;
  for (i=0; i < num_cores;i++){
    allMbox[i] = (void*)temp;
    temp = (void*)temp + MBXSZ;
    PRT_MBX("scc.c: allMbox[%d] %p\n",i,allMbox[i]);
  } 
  startTime = SCCGetTime();
  NO_SCRIPT_DBG( "****************************\nSCC INIT at %f\n",startTime);
}

//--------------------------------------------------------------------------------------
// FUNCTION: free all config registers and close shared memory
//--------------------------------------------------------------------------------------
void SCCStop(){
  unsigned char cpu;
  int i,offset;
  double stopTime;
  
  char *fname;
  fname = malloc(sizeof(char)*64);
  snprintf( fname, 64, "/shared/nil/Out/rck%02d.log",node_id);
  logFile = fopen(fname, "w");
  
  for (cpu = 0; cpu < 48; cpu++){
    FreeConfigReg((int*) locks[cpu]);
    if(cpu == 0) MPBunalloc( &firstMPB);
    
    //First SET OF AIR
    FreeConfigReg((int*) atomic_inc_regs[cpu].counter);
    FreeConfigReg((int*) atomic_inc_regs[cpu].init);
  }
  if(SCCIsMaster()){
    FreeConfigReg((int*) RPC_virtual_address);
    for (i=0; i<CORES/2; i++) {
      FreeConfigReg((int*) fChange_vAddr[i]);
    }
  }
  stopTime = SCCGetTime();
  fprintf(logFile, "%f#%f#%f\n",startTime,stopTime,stopTime-startTime);
  
  SCCMallocStop();
  
  fprintf(logFile, "%lld\n",requestServiced);
  fclose(logFile);
  fclose(masterFile);
  
  NO_SCRIPT_DBG("************************************************************\n");
  NO_SCRIPT_DBG("\tStart Time: %f\n\tStop Time: %f\n\tTotal Runtime: %f\n",startTime,stopTime,stopTime-startTime);
  NO_SCRIPT_DBG("************************************************************\n");
  
  FreeConfigReg((int*) tlo);
  FreeConfigReg((int*) thi);
  FreeConfigReg((int*) V3v3SCC);
  FreeConfigReg((int*) C3v3SCC);
  
  for(i=0;i<6;i++){
    FreeConfigReg((int*)VCCADDRV[i]);
  }
  
}

void SCCFill_RC_COREID(){
  FILE *fd;
  int np,cid;
  char * line = NULL;
  size_t len = 0;
  
  fd = fopen (hostFile,"r");
  
  // fill with invalid value
  for(np=0;np<CORES;np++) RC_COREID[np] = -1;
  
	for(np=0;np<num_cores;np++){
	  getline(&line,&len,fd);
    int l = atoi(line);
    PRT_FILE("np %d, line %d\n",np,l);
    //ALL_DBG("scc.c: SCC RC np %d, line %d\n",np,l);
    RC_COREID[np] = l;
    if(np == 0) master_id = RC_COREID[np]; // set master id
    // node_rank is virtual address
    if(RC_COREID[np] == node_id) node_rank = np;
    PRT_FILE("np %d, RC_COREID[%d] %d, node_id %d, RC_COREID[%d]==node_id %d\
            \n\n",np,np,RC_COREID[np],node_id,np,(RC_COREID[np] == node_id));
	}
   
  fclose(fd);
  
  for(np=0;np<RC_VOLTAGE_DOMAINS;np++) activeDomains[np] = 1;
  /*
  for(np=0;np<RC_VOLTAGE_DOMAINS;np++) activeDomains[np] = -1;
  for(np=0;np<num_cores;np++){
    cid = RC_COREID[np];
    if((cid >= 0 && cid <= 3) || (cid >= 12 && cid <= 15)) activeDomains[0] = 1;
    if((cid >= 4 && cid <= 7) || (cid >= 16 && cid <= 19)) activeDomains[1] = 1;
    if((cid >= 8 && cid <= 11) || (cid >= 20 && cid <= 23)) activeDomains[2] = 1;
    if((cid >= 24 && cid <= 27) || (cid >= 36 && cid <= 39)) activeDomains[3] = 1;
    if((cid >= 28 && cid <= 31) || (cid >= 40 && cid <= 43)) activeDomains[4] = 1;
    if((cid >= 32 && cid <= 35) || (cid >= 44 && cid <= 47)) activeDomains[5] = 1;
  }*/
}

void setSCCVars(){
  FILE *fd;
  char *key = NULL;
  char *value;
  size_t len = 0;
   
  fd = fopen ("/shared/nil/input.txt","r");

	getline(&key, &len, fd); strtok_r(key, "=", &value); num_worker=atoi(value);
	getline(&key, &len, fd); strtok_r(key, "=", &value); num_wrapper=atoi(value);
	getline(&key, &len, fd); strtok_r(key, "=", &value); DVFS=atoi(value);
  getline(&key, &len, fd); strtok_r(key, "=", &hostFile); hostFile[strcspn ( hostFile, "\n" )] = '\0';

  num_cores = num_worker + num_wrapper;
  
  if(DVFS > 1) {
    DVFS = 1;
    changeVLT = 0;
  }
  
  masterFile = fopen("/shared/nil/Out/master.txt", "w");
  if (masterFile == NULL)fprintf(stderr, "Can't open output file /shared/nil/Out/master.txt for master!\n");
  
  fclose(fd);
}


void remapLUT(int myCoreID) {
  // coreID is Z coordinate
  int page_size, i,NCMDeviceFD;

  t_vcharp     MappedAddr;
  unsigned int result,alignedAddr, pageOffset, ConfigAddr;
  unsigned int ConfigAddrLUT;
  
  page_size  = getpagesize(); // set page size
  
  if ((NCMDeviceFD=open("/dev/rckncm", O_RDWR|O_SYNC))<0) {
    perror("open"); exit(-1);
  }
  
  if(myCoreID==1){ 
    ConfigAddrLUT = CRB_OWN+LUT1; 
  } else { 
    ConfigAddrLUT = CRB_OWN+LUT0; 
  }

  int idx=0,page=0;
  //unsigned int lutValArr[] = {6595,45302,268493,307200}; // values from core 17,18,29,30
  unsigned int lutValArr[] = {6554,6595,307200,307241}; // values from core 16,17,30,31
  unsigned int value = lutValArr[idx];

  unsigned int lutSlot = START_PAGE, max = END_PAGE; // max mem is 944 M

  for(lutSlot; lutSlot<=max;lutSlot++){
    if(page == 4) { // map four pages for each MC
			lutValArr[idx] = value; // update array with new value
			page = 0; // set page to 0 again
			idx = (idx+1)%4; // update idx between 0 - 3
			value = lutValArr[idx]; // get next value from array
	  }
    
    ConfigAddr = ConfigAddrLUT + (lutSlot*0x08);
    alignedAddr = ConfigAddr & (~(page_size-1));
    pageOffset  = ConfigAddr - alignedAddr;
    
    MappedAddr = (t_vcharp) mmap(NULL, page_size, PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
    if (MappedAddr == MAP_FAILED) {
      perror("mmap");exit(-1);
    }
    //printf("scc.c: lutSlot 0x%x (%d) oldEntry 0x%x (%d) ",lutSlot,lutSlot,*((int*)(MappedAddr+pageOffset)),*((int*)(MappedAddr+pageOffset)));
    *(int*)(MappedAddr+pageOffset) = value;
    value++;
    page++;
    //printf(" after edit: 0x%x (%d)\n",*((int*)(MappedAddr+pageOffset)),*((int*)(MappedAddr+pageOffset)));
    munmap((void*)MappedAddr, page_size);
  }
}

//////////////////////////////////////////////////////////////////////////////////////// 
// Start of Power and freq functions // virtual domain is passed in
//////////////////////////////////////////////////////////////////////////////////////// 
static int read_current_frequency(int tileId){
  int i;
  unsigned int lower_bits = (*(fChange_vAddr[tileId]))&((1<<8)-1);
  unsigned int orig_bits = ((*(fChange_vAddr[tileId])) - lower_bits) >> 8;

  for(i=1; i<=16; i++) {
    if(orig_bits == RC_frequency_change_words[i][1]){
      return i;
    }
  }
  return -1;
}

// takes in logical domain
double readVCC(int domain){
  //fprintf(stderr,"domain %d PD %d, physical 0x%x logical %p\n",domain,PD[domain],VCCADDRP[domain],VCCADDRV[domain]);
  volatile unsigned int val = *(int*)(VCCADDRV[domain]);
  return (double)val * 0.0000251770;
}

void startPowerMeasurement(int start){
  //if(start) writeFpgaGrb(DVFS_CONFIG, 0x00000a80 );
  //else      writeFpgaGrb(DVFS_CONFIG, 0x00000800 );
}

void powerMeasurement(FILE *fileHand){
  //fprintf(fileHand,"%5.3f V\t%6.3f A\n", readStatus(DVFS_STATUS_U3V3SCC), readStatus(DVFS_STATUS_I3V3SCC));
  unsigned int volti = *(int*)V3v3SCC;
  unsigned int currenti = *(int*)C3v3SCC;
  double volt = (double)volti * 0.0002500000; //0.004;
  double current = (double)currenti * 0.0062490250; //0.0990099;
  
  //fprintf(fileHand,"V %f \tA %f \tT %f\n", volt, current,SCCGetTime());
  //fprintf(fileHand,"V,%f,A,%f#", volt, current);
  fprintf(fileHand,"%f,%f#", volt, current);
  //fflush(fileHand);
  //fprintf(stderr,"V %f \tA %f\n", volt, current);
}

void set_min_freq(){
  int reqFreqDiv = 15,i,new_Fdiv,new_Vlevel;;
  
  for(i=1;i<RC_VOLTAGE_DOMAINS;i++){
    if(activeDomains[i] != 1){
      set_freq_volt_level(reqFreqDiv, &new_Fdiv, &new_Vlevel, i);
      fprintf(stderr,"domain %d PD %d is inactive, set to lowest frequency %d\n\n\n",i,PD[i],RC_frequency_change_words[reqFreqDiv][2]);
      fprintf(masterFile,"domain %d PD %d is inactive, set to lowest frequency %d\n\n\n",i,PD[i],RC_frequency_change_words[reqFreqDiv][2]);
      //fflush(masterFile);
    }
  }
}

void change_freq(double prop, char c){
  static int first=1;
  static observer_t *obs;
  
  // get observer address
  if(first){
    while (OBSET !=9);
    first=0; 
    memcpy((void*)&obs, (const void*)OBADDR, sizeof(observer_t*));
    obs->freq = RC_frequency_change_words[RC_current_val[0].current_freq_div][2];
    obs->volt = RC_V_MHz_cap[RC_current_val[0].current_volt_level].volt;
    printf("change freq freq %d, vlt %f\n",RC_frequency_change_words[RC_current_val[0].current_freq_div][2],RC_V_MHz_cap[RC_current_val[0].current_volt_level].volt);
    printf("MSTR: obs->window_size %d, obs->thresh_hold %f, obs->skip_update %d\n",obs->window_size, obs->thresh_hold, obs->skip_update);
  }

  if(!DVFS || !obs->startChange){
    fprintf(masterFile,"Frequency can not be changed, DVFS is disabled\n");
    return;
  }
  
  // do not change anything on active domain0 as it runs master
  int reqFreqDiv = -1,currFreqDiv=-1,i,retVal;
  double reqFreq = 0.0;
  
  currFreqDiv = RC_current_val[0].current_freq_div;
  
  // increase already max
  if(prop > 0.0 && currFreqDiv == RC_MIN_FREQUENCY_DIVIDER) {
    fprintf(masterFile,"\nMaster: Freq already maximum at %d\n",RC_frequency_change_words[currFreqDiv][2]);
    return;
  } else if (prop < 0.0 && currFreqDiv == RC_MAX_FREQUENCY_DIVIDER){   // decrease already min
    fprintf(masterFile,"\nMaster: Freq already minimum at %d\n",RC_frequency_change_words[currFreqDiv][2]);
    return;
  }
  
  
  reqFreq  = (1+prop)*RC_frequency_change_words[currFreqDiv][2];
  
  fprintf(masterFile,"\nMaster: FREQCHANGE %c,  prop %f, currFreq %d, reqFreq %f\n",c,prop,RC_frequency_change_words[currFreqDiv][2],reqFreq);
  
  if(prop > 0.0){ // increase
    fprintf(masterFile,"\nMaster: th %f, increase frequency by %f\n",obs->thresh_hold,prop);
    for(i=(currFreqDiv-1);i>=RC_MIN_FREQUENCY_DIVIDER;i--){
      if(RC_frequency_change_words[i][2] > reqFreq){
        reqFreqDiv = i;
        break;
      }
    } 
  } else { // decrease
    if(currFreqDiv == 15 ) {
      reqFreqDiv = currFreqDiv + 1;
    } else {
      reqFreqDiv = currFreqDiv + 2;
    }
    
    fprintf(masterFile,"\nMaster: decrease frequency to %d from %d\n",RC_frequency_change_words[reqFreqDiv][2],RC_frequency_change_words[currFreqDiv][2]);
  /*
    fprintf(masterFile,"\nMaster: th 0.2, decrease frequency by %f\n",prop);
    for(i=(currFreqDiv+1);i<=RC_MAX_FREQUENCY_DIVIDER;i++){
      if(reqFreq > RC_frequency_change_words[i][2]){
        reqFreqDiv = i-1; // select previous one
        break;
      }
    } 
  */
  } 
  
  if (reqFreqDiv == currFreqDiv) {
  	fprintf(masterFile,"Frequency can not be changed, as requested is same as current\n");
  	return;
 	}
  
  if (reqFreqDiv < 0){
    fprintf(masterFile,"Frequency can not be changed at this point, reqFreqDiv is -1\n");
    return;
  }
  
  if (reqFreqDiv > 16){
    fprintf(masterFile,"Frequency can not be changed at this point, reqFreqDiv iscan not be > 16\n");
    return;
  }
  
  /*
  if (reqFreqDiv >= RC_MAX_FREQUENCY_DIVIDER){
    fprintf(masterFile,"Frequency can not be changed at this point, already min\n");
    return;
  } else if (reqFreqDiv <= RC_MIN_FREQUENCY_DIVIDER){
    fprintf(masterFile,"Frequency can not be changed at this point, already max\n");
    return;
  }*/
  
  int new_Fdiv,new_Vlevel;

  for(i=0;i<RC_VOLTAGE_DOMAINS;i++){
    retVal = set_freq_volt_level(reqFreqDiv, &new_Fdiv, &new_Vlevel, i);
    if(retVal == -1 ) {
      //fprintf(stderr,"domain %d PD %d frequency can not be changed\n\n\n",i,PD[i]);
      fprintf(masterFile,"domain %d PD %d frequency can not be changed\n",i,PD[i]);
    } else {
      //fprintf(stderr,"domain %d PD %d frequency changed to %d\n\n\n",i,PD[i],RC_frequency_change_words[new_Fdiv][2]);
      fprintf(masterFile,"domain %d PD %d frequency %d from %d time %f\n",i,PD[i],RC_frequency_change_words[new_Fdiv][2],RC_frequency_change_words[currFreqDiv][2],SCCGetTime());
    }
  }
  
  obs->freq = RC_frequency_change_words[new_Fdiv][2];
  obs->volt = RC_V_MHz_cap[new_Vlevel].volt;
  //start timer for message skip in sink
  //obs->skip_count = 0;
}

int set_frequency_divider(int Fdiv, int *new_Fdiv, int domain) {
	int Vlevel,tile;

	// if Fdiv is under or over min/max allowed the set it to min/max
	if (Fdiv > RC_MAX_FREQUENCY_DIVIDER) Fdiv = RC_MAX_FREQUENCY_DIVIDER;
  else 
  if (Fdiv < RC_MIN_FREQUENCY_DIVIDER) Fdiv = RC_MIN_FREQUENCY_DIVIDER;
  
  // check to see if the new frequency divider is valid
  Vlevel = RC_voltage_level(Fdiv);
  
  // check current volts support requested freq
  if (RC_current_val[domain].current_volt_level < Vlevel || Vlevel < 0){
    fprintf(stderr,"Voltage leve is low for given divider\n");
    *new_Fdiv = -1;
    return(-1);
  }
  *new_Fdiv = Fdiv;
  
  // need to set frequency divider on all tiles of the domain
  for(tile=0; tile < 4; tile++) {
    //RC_set_frequency_divider(tile, Fdiv);
    RC_set_frequency_divider(RC_domains[domain][tile],Fdiv);
  }
  RC_current_val[domain].current_freq_div = Fdiv;
  return(1);
}

// takes in logical domain
int set_freq_volt_level(int Fdiv, int *new_Fdiv, int *new_Vlevel, int domain) {
	int Vlevel,tile;
  
  int changeVoltLvl=0;

	// if Fdiv is under or over min/max allowed the set it to min/max
	if (Fdiv > RC_MAX_FREQUENCY_DIVIDER) Fdiv = RC_MAX_FREQUENCY_DIVIDER;
  else 
  if (Fdiv < RC_MIN_FREQUENCY_DIVIDER) Fdiv = RC_MIN_FREQUENCY_DIVIDER;
  
  // check to see if the new frequency divider is valid
  Vlevel = RC_voltage_level(Fdiv);
  
  // check volt level support requested freq
  if (Vlevel < 0){
    fprintf(stderr,"Voltage level is low for given divider\n");
    *new_Fdiv = -1;
    *new_Vlevel = -1;
    return(-1);
  }
  *new_Vlevel = Vlevel;
  *new_Fdiv = Fdiv;
  
  // only change volts if its different then previous level
  changeVoltLvl = (RC_current_val[domain].current_volt_level != Vlevel);
  
  // if new frequency divider greater than current (decrease freq), adjust frequency immediately;
  // this can always be done safely if the current power state is feasible
  if (Fdiv >= RC_current_val[domain].current_freq_div){
    // need to set frequency divider on all tiles of the voltage domain
    for(tile=0; tile < 4; tile++) {
      //RC_set_frequency_divider(tile, Fdiv);
      RC_set_frequency_divider(RC_domains[domain][tile],Fdiv);
      //fprintf(stderr,"domain %d PD %d tile %02d\n",domain,PD[domain],RC_domains[domain][tile]);
    }
    RC_current_val[domain].current_freq_div = Fdiv;
  } 
  
  if(changeVoltLvl && changeVLT){
    // write the VID word to RPC, need to send in physical domain
    unsigned int VID = VID_word(RC_V_MHz_cap[Vlevel].volt, PD[domain]);
    *RPC_virtual_address = VID;
    *RPC_virtual_address = VID;
    *RPC_virtual_address = VID;
  
    fprintf(masterFile,"wrote 0x%x to RPC for domain %d PD %d\n",VID,domain,PD[domain]);
    //fflush(masterFile);
  
    // set newVolt to requested so we can check later 
    int oldVolti, newVolti, changed;
    volatile double volRead;
    oldVolti = getInt(RC_V_MHz_cap[RC_current_val[domain].current_volt_level].volt);
    newVolti = getInt(RC_V_MHz_cap[Vlevel].volt);
  
    fprintf(masterFile,"in loop old %d new %d\n",oldVolti,newVolti);
    //fflush(masterFile);
    volRead = readVCC(domain);
 
    // read status register untill value reflects change
    do{
      //volRead = readStatus(VCCADDR[domain],0.0000251770);
      volRead = readVCC(domain);
      printf("oldVolti %d, newVolti %d, volRead %d\n",oldVolti,newVolti,getInt(volRead));
      usleep(100);
      if(newVolti > oldVolti)      { changed = newVolti <= getInt(volRead); } 
      else if(newVolti < oldVolti) { changed = newVolti >= getInt(volRead); }
      else if(newVolti == oldVolti){ changed = 1; }
    }while(!changed);
    fprintf(masterFile,"Volt int after change:  %f, %d\n",volRead,getInt(volRead));
    //fflush(masterFile);
    RC_current_val[domain].current_volt_level = Vlevel;
  }
  
  // if we asked for a decrease in the clock divider (increase in freq), apply it 
  // now, after the required target voltage has been reached.
  if (Fdiv < RC_current_val[domain].current_freq_div) {
    // need to set frequency divider on all tiles of the voltage domain    
    for(tile=0; tile < 4; tile++) {
      //RC_set_frequency_divider(tile, Fdiv);
      RC_set_frequency_divider(RC_domains[domain][tile],Fdiv);
      //fprintf(stderr,"domain %d PD %d tile %02d\n",domain,PD[domain],RC_domains[domain][tile]);
    }
    RC_current_val[domain].current_freq_div = Fdiv;
  }
  return(1);
}

// takes in physical domain number
unsigned int VID_word(float voltage, int domain) {
  int VID, voltage_value;

  voltage_value = (int)(voltage/0.00625 + 0.5);
  VID =  0x10000 |( (domain << 8 ) | voltage_value);
  return VID;
}

// voltage level corresponding to the chosen frequency
int RC_voltage_level(int Fdiv) {
  int Vlevel, found, MHz;

  MHz = RC_GLOBAL_CLOCK_MHZ/Fdiv;
  found = 0;

  for (Vlevel=0; Vlevel<=RC_NUM_VOLTAGE_LEVELS; Vlevel++){
    if (RC_V_MHz_cap[Vlevel].MHz_cap >= MHz) {found=1; break;}
  }
  
  if (found)  return(Vlevel);
  else        return(-1);
}

// set frequency divider on a single tile 
int RC_set_frequency_divider(int tile_ID, int Fdiv) {
  if(tile_ID == 8 || tile_ID == 15) return 1; // used to create SHM
  
  *(fChange_vAddr[tile_ID]) = FID_word(Fdiv, tile_ID);
  //fprintf(stderr,"FID_word 0x%x written for tile %02d\n",FID_word(Fdiv, tile_ID),tile_ID);
  //fprintf(masterFile,"FID_word 0x%x %d written for tile %02d\n",FID_word(Fdiv, tile_ID),RC_frequency_change_words[Fdiv][2],tile_ID);
  return (1);
}

// generate the complete word to be written to the CRB for tile clock freq 
unsigned int FID_word(int Fdiv, int tile_ID) {
  unsigned int lower_bits = (*(fChange_vAddr[tile_ID]))&((1<<8)-1);
  return(((RC_frequency_change_words[Fdiv][1])<<8)+lower_bits);
  //return(((RC_frequency_change_words[Fdiv][1])<<8)+0xf0);
}

// get the complete word from the CRB for tile clock frequency
int get_divider(int tile_ID) {
  int step;
  unsigned int word;
  // shift to the right to get the correct bits
  word = (*(fChange_vAddr[tile_ID]))>>8;
  for (step=0; step<=16; step++) {
	  if (word==RC_frequency_change_words[step][1]) break;
  }
  
  if (word==RC_frequency_change_words[step][1]) return(step);
  else                                          return(-1);
}

int getInt(float voltage)
{
  return (int)(voltage * 10 + 0.5);
}

int isDvfsActive(){
  return DVFS;
}

//////////////////////////////////////////////////////////////////////////////////////// 
// End of Power and freq functions
//////////////////////////////////////////////////////////////////////////////////////// 

//--------------------------------------------------------------------------------------
// FUNCTION: read value of register holding start time from FPGA
//--------------------------------------------------------------------------------------
static inline unsigned long long gtsc(void)
{   
   unsigned int lo, hi;
   lo = *tlo;
   hi = *thi;
   return (unsigned long long)hi << 32 | lo;
}


//--------------------------------------------------------------------------------------
// FUNCTION: generates time based on 125MHz system clock of the FPGA
// Returns elapsed time in seconds
//--------------------------------------------------------------------------------------
double SCCGetTime()
{ 
  /* second = cycles / freq in Hz SO second = gtsc() / 125000000 */
  // this is to generate time based on 125MHz system clock of the FPGA
  return ( ((double)gtsc())/(0.125*1.e9));
  // this is to generate time for 533 MHz clock
  //return ( ((double)gtsc())/(0.533*1.e9));
}

void SCCGetTimeAll(timespecSCC *t){
  unsigned long long tval = gtsc();
  unsigned long long nsec,sec;
  
  sec =  (tval/125000000);
  nsec = (((tval*1000)/125) - (sec * 1.e9));
  
  t->tv_sec =  sec;
  t->tv_nsec = nsec;
}

//--------------------------------------------------------------------------------------
// FUNCTION: SCCGetNodeID
//--------------------------------------------------------------------------------------
// Returns node's physical ID
//--------------------------------------------------------------------------------------
int SCCGetNodeID(void){
	return node_id;
}

//--------------------------------------------------------------------------------------
// FUNCTION: SCCGetNodeRank
//--------------------------------------------------------------------------------------
// Returns node's logical ID
//--------------------------------------------------------------------------------------
int SCCGetNodeRank(void){
	return node_rank;
}

//--------------------------------------------------------------------------------------
// FUNCTION: SCCGetNumWrappers
//--------------------------------------------------------------------------------------
// Returns number of wrappers
//--------------------------------------------------------------------------------------
int SCCGetNumWrappers(void){
	return num_wrapper;
}

//--------------------------------------------------------------------------------------
// FUNCTION: SCCGetNumCores
//--------------------------------------------------------------------------------------
// Returns number of participating cores
//--------------------------------------------------------------------------------------
int SCCGetNumCores(void){
	return num_cores;
}

//--------------------------------------------------------------------------------------
// FUNCTION: SCCIsMaster
//--------------------------------------------------------------------------------------
// Returns 1 if node is master 0 otherwise
//--------------------------------------------------------------------------------------
int SCCIsMaster(void){
  // if logical address is 0 then its master
	return (node_rank == 0);
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

//--------------------------------------------------------------------------------------
// Just for info
/*        nano    00 00 00 00 0
 *        micro   00 00 00
 *        mili    00 0
 *        struct timeval {
 *          long tv_sec;    // seconds 
 *          long tv_usec;   // microseconds
 *        };
 *        struct timespec {
 *            time_t  tv_sec    // seconds
 *            long    tv_nsec;  // nanoseconds
 *        };
*/

/*
void SCCGetTimeAll(timespecSCC *t){
  unsigned long long tval = gtsc();
  unsigned long long nsec,sec;
  
  sec =  (tval/125000000);
  nsec = (((tval*1000)/125) - (sec * 1.e9));
  
  t->tv_sec =  sec;             // seconds 
  t->tv_msec = (tval/125000);   // milliseconds
  t->tv_usec = (tval/125);      // microseconds
  t->tv_nsec = nsec;            // nanoseconds 
}
*/


/*
void remapLUT(int myCoreID) {
  // coreID is Z coordinate
  int page_size, i,NCMDeviceFD;

  t_vcharp     MappedAddr;
  unsigned int result,alignedAddr, pageOffset, ConfigAddr;
  unsigned int ConfigAddrLUT;
  
  page_size  = getpagesize(); // set page size
  
  if ((NCMDeviceFD=open("/dev/rckncm", O_RDWR|O_SYNC))<0) {
    perror("open"); exit(-1);
  }
  
  if(myCoreID==1){ 
    ConfigAddrLUT = CRB_OWN+LUT1; 
  } else { 
    ConfigAddrLUT = CRB_OWN+LUT0; 
  }

  int idx=0,page=0;
  unsigned int lutValArr[] = {6595,45302,268493,307200}; // values from core 17,18,29,30
  unsigned int value = lutValArr[idx];

  unsigned int lutSlot = START_PAGE, max = END_PAGE; // max mem is 944 M

  for(lutSlot; lutSlot<=max;lutSlot++){
    if(page == 4) { // map four pages for each MC
			lutValArr[idx] = value; // update array with new value
			page = 0; // set page to 0 again
			idx = (idx+1)%4; // update idx between 0 - 3
			value = lutValArr[idx]; // get next value from array
	  }
    
    ConfigAddr = ConfigAddrLUT + (lutSlot*0x08);
    alignedAddr = ConfigAddr & (~(page_size-1));
    pageOffset  = ConfigAddr - alignedAddr;
    
    MappedAddr = (t_vcharp) mmap(NULL, page_size, PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
    if (MappedAddr == MAP_FAILED) {
      perror("mmap");exit(-1);
    }
    //printf("scc.c: lutSlot 0x%x (%d) oldEntry 0x%x (%d) ",lutSlot,lutSlot,*((int*)(MappedAddr+pageOffset)),*((int*)(MappedAddr+pageOffset)));
    *(int*)(MappedAddr+pageOffset) = value;
    value++;
    page++;
    //printf(" after edit: 0x%x (%d)\n",*((int*)(MappedAddr+pageOffset)),*((int*)(MappedAddr+pageOffset)));
    munmap((void*)MappedAddr, page_size);
  }
}
*/

/*
// popshm style
void remapLUT1(int myCoreID) {
  // coreID is Z coordinate
  int page_size, i,NCMDeviceFD;

  t_vcharp     MappedAddr;
  unsigned int result,alignedAddr, pageOffset, ConfigAddr;
  unsigned int ConfigAddrLUT;
  
  page_size  = getpagesize(); // set page size
  
  int lutValArr[] ={
  6181,6182,6183,6184, 45093,45094,45095,45096, 268325,268326,268327,268328, 307237,307238,307239,307240,
	6222,6223,6224,6225, 45134,45135,45136,45137, 268366,268367,268368,268369, 307278,307279,307280,307281,
	6263,6264,6265,6266, 45175,45176,45177,45178, 268407,268408,268409,268410, 307319,307320,307321,307322,
	6304,6305,6306,6307, 45216,45217,45218,45219, 268448,268449,268450,268451, 307360,307361,307362,307363,
	6345,6346,6347,6348, 45257,45258,45259,45260, 268489,268490,268491,268492, 307401,307402,307403,307404,
	6386,6387,6388,6389, 45298,45299,45300,45301, 268530,268531,268532,268533, 307442,307443,307444,307445,
	6427,6428,6429,6430, 45339,45340,45341,45342, 268571,268572,268573,268574, 307483,307484,307485,307486,
	6468,6469,6470,6471, 45380,45381,45382,45383, 268612,268613,268614,268615, 307524,307525,307526,307527,
	6509,6510,6511,6512, 45421,45422,45423,45424, 268653,268654,268655,268656, 307565,307566,307567,307568,
	6550,6551,6552,6553, 45462,45463,45464,45465, 268694,268695,268696,268697, 307606,307607,307608,307609,
	6591,6592,6593,6594, 45503,45504,45505,45506, 268735,268736,268737,268738, 307647,307648,307649,307650,
	6632,6633,6634,6635, 45544,45545,45546,45547, 268776,268777,268778,268779, 307688,307689,307690,307691};
  
  if ((NCMDeviceFD=open("/dev/rckncm", O_RDWR|O_SYNC))<0) {
    perror("open"); exit(-1);
  }
  
  if(myCoreID==1){ 
    ConfigAddrLUT = CRB_OWN+LUT1; 
  } else { 
    ConfigAddrLUT = CRB_OWN+LUT0; 
  }

  unsigned int lutSlot = START_PAGE, max = END_PAGE; // max mem is 944 M
  int idx=0;
  unsigned int value;
  
  for(lutSlot; lutSlot<=max;lutSlot++){
    value = lutValArr[idx++]; // get next value from array

    ConfigAddr = ConfigAddrLUT + (lutSlot*0x08);
    alignedAddr = ConfigAddr & (~(page_size-1));
    pageOffset  = ConfigAddr - alignedAddr;
    
    MappedAddr = (t_vcharp) mmap(NULL, page_size, PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
    if (MappedAddr == MAP_FAILED) {
      perror("mmap");exit(-1);
    }
    //printf("scc.c: lutSlot 0x%x (%d) oldEntry 0x%x (%d) ",lutSlot,lutSlot,*((int*)(MappedAddr+pageOffset)),*((int*)(MappedAddr+pageOffset)));
    *(int*)(MappedAddr+pageOffset) = value;
    //printf(" after edit: 0x%x (%d)\n",*((int*)(MappedAddr+pageOffset)),*((int*)(MappedAddr+pageOffset)));
    munmap((void*)MappedAddr, page_size);
  }
}*/


/*  
  if(prop > 0.0){ // increase
    fprintf(masterFile,"\nMaster: th %f, increase frequency by %f\n",obs->thresh_hold,prop);
    for(i=currFreqDiv;i>=RC_MIN_FREQUENCY_DIVIDER;i--){
      if(reqFreq > RC_frequency_change_words[i][2]){
        reqFreqDiv = i;
      }
    } 
  } else { // decrease
  
    reqFreqDiv = currFreqDiv + 2;
    fprintf(masterFile,"\nMaster: decrease frequency to %d from %d\n",RC_frequency_change_words[reqFreqDiv][2],RC_frequency_change_words[currFreqDiv][2]);
  /*
    fprintf(masterFile,"\nMaster: th 0.2, decrease frequency by %f\n",prop);
    for(i=currFreqDiv;i<=RC_MAX_FREQUENCY_DIVIDER;i++){
      if(reqFreq > RC_frequency_change_words[i][2]){
        reqFreqDiv = i;
      }
    } 
  *
  } 
*/  
