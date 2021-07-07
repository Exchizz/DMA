#include <stdio.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>


//#define BUS_PHYS_ADDR(a) ((void *)((uint32_t)(a)&~0xC0000000))
#define BUS_TO_PHYS(x) ((void *)((uint32_t)(x) + 0x80000000))



int main(){
	uint32_t dma_phys = BUS_TO_PHYS(0x7e007000);
	printf("DMA bus  at: 0x7E007000\n");
	printf("DMA phys at: 0x%X\n", dma_phys);
	return 0;
}
