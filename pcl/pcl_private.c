/*
 *  PCL by Davide Libenzi (Portable Coroutine Library)
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
#include <stdlib.h>
#include <string.h>
#include "pcl_config.h"
#include "pcl.h"
#include "pcl_private.h"
#include "scc.h"
#include "debugging.h"


static cothread_ctx *co_get_global_ctx(void)
{
	static cothread_ctx tctx;

	if (tctx.co_curr == NULL)
		tctx.co_curr = &tctx.co_main;
	
	return &tctx;
}

#if !defined(CO_MULTI_THREAD)
/*
 * Simple case, the single thread one ...
 */

int co_thread_init(void)
{
	return 0;
}

void co_thread_cleanup(void)
{

}

cothread_ctx *co_get_thread_ctx(void)
{
	return co_get_global_ctx();
}

#else
/*
 * MultiThread cases ...
 */

#if defined(_WIN32) && defined(_MSC_VER)
/*
 * On Windows, we can use the native TLS capabilities. Pretty easy ...
 */
static __declspec(thread) cothread_ctx *tctx;

int co_thread_init(void)
{
	//if ((tctx = (cothread_ctx *)malloc(sizeof(cothread_ctx))) == NULL) {
	if ((tctx = (cothread_ctx *)SCCMallocPtr(sizeof(cothread_ctx))) == NULL) {
        perror("allocating context");
		return -1;
	}
	memset(tctx, 0, sizeof(*tctx));
	tctx->co_curr = &tctx->co_main;

	return 0;
}

void co_thread_cleanup(void)
{
	free(tctx);
}

cothread_ctx *co_get_thread_ctx(void)
{
	/*
	 * Even in MT mode, allows for the main thread to not call
	 * the co_thread_init()/co_thread_cleanup() functions.
	 */
	return tctx != NULL ? tctx: co_get_global_ctx();
}

#else
/*
 * On Unix, we use pthread. Sligthly more complicated ...
 */
#include <pthread.h>

static int valid_key;
static pthread_key_t key;
static pthread_once_t once_control = PTHREAD_ONCE_INIT;

static void co_once_init(void)
{
	//if (pthread_key_create(&key, free))
	if (pthread_key_create(&key, SCCFreePtr))
		perror("creating TLS key");
	else
		valid_key++;
}

int co_thread_init(void)
{
	cothread_ctx *tctx;

	pthread_once(&once_control, co_once_init);
	if (!valid_key)
		return -1;

	//if ((tctx = (cothread_ctx *)
	     //malloc(sizeof(cothread_ctx))) == NULL) {
	if ((tctx = (cothread_ctx *)SCCMallocPtr(sizeof(cothread_ctx))) == NULL) {
		perror("allocating context");
		return -1;
	}
	memset(tctx, 0, sizeof(*tctx));
	PRT_DBG("1 inside pcl tctx->co_curr %p tctx->co_main %p key %d\n" ,tctx->co_curr,&tctx->co_main,key);
	tctx->co_curr = &tctx->co_main;
	PRT_DBG("2 inside pcl tctx->co_curr %p tctx->co_main %p key %d\n" ,tctx->co_curr,&tctx->co_main,key);
	PRT_DBG("INSIDE CO_THREAD_INIT, tctx: %p\n",tctx);
	PRT_DBG("INSIDE CO_THREAD_INIT, tctx->co_curr: %p\n",tctx->co_curr);

	if (pthread_setspecific(key, tctx)) {
		perror("setting thread context");
		//free(tctx);
		SCCFreePtr(tctx);
		return -1;
	}
	PRT_DBG("3 inside pcl tctx->co_curr %p tctx->co_main %p key %d\n" ,tctx->co_curr,&tctx->co_main,key);
	return 0;
}

void co_thread_cleanup(void)
{

}

cothread_ctx *co_get_thread_ctx(void)
{
//	PRT_DBG("INSIDE CO_GET_THREAD_CTX, valid_key:		%d\n",valid_key);
//	PRT_DBG("INSIDE CO_GET_THREAD_CTX, key:			%d\n",key);
	
	cothread_ctx *tctx = (cothread_ctx *)
		(valid_key ? pthread_getspecific(key): NULL);
	
	PRT_DBG("INSIDE CO_GET_THREAD_CTX, tctx:			%p\n",tctx);
	/*
	 * Even in MT mode, allows for the main thread to not call
	 * the co_thread_init()/co_thread_cleanup() functions.
	 */
	if(tctx != NULL){
		ALL_DBG("pcl_private.c: get thrd cntx will return ctx %p\n",tctx);
	} else {
		ALL_DBG("pcl_private.c: get thrd cntx will return glbl %p\n",co_get_global_ctx());
	}

	return tctx != NULL ? tctx: co_get_global_ctx();
}

#endif
#endif

