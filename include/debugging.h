//#define _USE_DBG__

#define NO_SCRIPT_DBG printf

#ifdef _USE_DBG__
#define PRT_DBG //
#define PRT_DBG1 printf
#else
#define PRT_DBG	//
#define PRT_DBG1	//
#define ALL_DBG //
#endif
