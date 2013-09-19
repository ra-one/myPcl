//#define _USE_DBG__

#ifdef _USE_DBG__
#define PRT_DBG //
#define PRT_DBG1 printf
#else
#define PRT_DBG	//
#define PRT_DBG1	//
#endif
