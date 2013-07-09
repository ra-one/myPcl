#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h> /*for uint16_t*/

#include "scc.h"
#include "bool.h"
#include "RCCE_memcpy.c"
#include "debugging.h"


void cpy_mpb_to_mem(int node, void *dst, int size)
{
  int start, end, cpy;

  flush();
  start = START(node);
  end = END(node);

  while (size) {
    if (end < start) cpy = min(size, B_SIZE - start);
    else cpy = size;

    flush();
    memcpy(dst, (void*) (mpbs[node] + B_START + start), cpy);
    start = (start + cpy) % B_SIZE;
    dst = ((char*) dst) + cpy;
    size -= cpy;
  }

  flush();
  START(node) = start;
  FOOL_WRITE_COMBINE;
}

void cpy_mem_to_mpb(int node, void *src, int size)
{
  int start, end, free;

  if (size >= B_SIZE) {
    printf("Message to big!");
    exit(3);
  }

  flush();
  WRITING(node) = true;
  FOOL_WRITE_COMBINE;

  while (size) {
    flush();
    start = START(node);
    end = END(node);

    if (end < start) free = start - end - 1;
    else free = B_SIZE - end - (start == 0 ? 1 : 0);
    free = min(free, size);

    if (!free) {
      unlock(node);
      usleep(1);
      lock(node);
      continue;
    }

//    memcpy((void*) (mpbs[node] + B_START + end), src, free);

//cpy_mem_to_mpb
	PRT_DBG("memcpy_put:\n node: %d B_START+end: %d src: %s size: %d\n", node,B_START + end,src,free);
	memcpy_put((void*) (mpbs[node] + B_START + end), src, free);

    flush();
    size -= free;
    src += free;
    END(node) = (end + free) % B_SIZE;
    WRITING(node) = false;
    FOOL_WRITE_COMBINE;
  }
}

