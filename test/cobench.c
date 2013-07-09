/*
 *  CoBench by Davide Libenzi (Portable Coroutine Library bench tester)
 *  Copyright (C) 2003..2010  Davide Libenzi
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Davide Libenzi <davidel@xmailserver.org>
 *
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <pcl.h>
#include <stdlib.h>
#include <scc_comm_func.h>
#include <sccmalloc.h>

#define CO_STACK_SIZE (8 * 1024)

coroutine_t coro1,coro2;

void fun(void *id)
{
  
  int idd = *((int*)id);
	printf("From coro %d\n",idd);

	int a = 0;
	while(1){
		printf("Coro %d, a %d\n",idd,a);
		a++;
		if(a%5 == 0)co_resume();
	}
}

void *thread_proc()
{
		
	int i, ntimes;

	//fprintf(stdout, "[%p] measuring co_create+co_delete performance ...\n",pthread_self());
	printf("[%p] Inside runTest now create two coro and run them\n",pthread_self());
	sleep(3);
	//coroutine_t co_create(void *func, void *data, void *stack, int stacksize);
	
	for(i=0;i<10;i++)
	{
		if(i%2)
		{
			DCMflush();
			co_call(coro1);
			DCMflush();
		}
		else
		{
			DCMflush();
			co_call(coro2);
			DCMflush();	
		}
		sleep(3);
	}
	

}

int main(int argc, char **argv)
{

	scc_init();
	void* local=SCCGetlocal();
  	co_thread_init();
	pthread_t *thid;
	
	int *data1,*data2;

if (SccGetNodeID()==0){
//	data1 = malloc(sizeof(int));
//	data2 = malloc(sizeof(int));
	data1 = SCCMallocPtr(sizeof(int));
        data2 = SCCMallocPtr(sizeof(int));
	*data1	= 1;
	*data2	= 2;

	coro1 = co_create(fun, (void*) data1, NULL, CO_STACK_SIZE);
	coro2 = co_create(fun, (void*) data2, NULL, CO_STACK_SIZE);
	
	printf("coro1: %p\n",coro1);
        printf("coro2: %p\n",coro2);

//	thid = malloc(sizeof(pthread_t));
	thid =SCCMallocPtr(sizeof(pthread_t));
	int res = pthread_create(thid, NULL, thread_proc, NULL);
	if(res)
	{
		perror("creating worker threads");
		return 1;
	}
	
//	co_delete(coro1);
//	co_delete(coro2);
	
	pthread_join(*thid, NULL);
	co_thread_cleanup();
}else{
	coro1= local+0x11b0;
	coro2=local+0x33b8;
	printf("coro1: %p\n",coro1);
	printf("coro2: %p\n",coro2);
	
	thid =SCCMallocPtr(sizeof(pthread_t));
        int res = pthread_create(thid, NULL, thread_proc, NULL);
        if(res)
        {
                perror("creating worker threads");
                return 1;
        }
	
//	co_delete(coro1);
//        co_delete(coro2);

        pthread_join(*thid, NULL);
        co_thread_cleanup();

}


	return 0;
}
