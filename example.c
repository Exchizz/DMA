#include<stdio.h>
#include <bcm_host.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define FAIL(x) {printf(x); }
#define PAGE_ROUNDUP(n) ((n)%PAGE_SIZE==0 ? (n) : ((n)+PAGE_SIZE)&~(PAGE_SIZE-1))
#define PHYS_REG_BASE  0xFE000000  // Pi 4

// Videocore mailbox memory allocation flags, see:
//     https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
typedef enum {
    MEM_FLAG_DISCARDABLE    = 1<<0, // can be resized to 0 at any time. Use for cached data
    MEM_FLAG_NORMAL         = 0<<2, // normal allocating alias. Don't use from ARM
    MEM_FLAG_DIRECT         = 1<<2, // 0xC alias uncached
    MEM_FLAG_COHERENT       = 2<<2, // 0x8 alias. Non-allocating in L2 but coherent
    MEM_FLAG_ZERO           = 1<<4, // initialise buffer to all zeros
    MEM_FLAG_NO_INIT        = 1<<5, // don't initialise (default is initialise to all ones)
    MEM_FLAG_HINT_PERMALOCK = 1<<6, // Likely to be locked for long periods of time
    MEM_FLAG_L1_NONALLOCATING=(MEM_FLAG_DIRECT | MEM_FLAG_COHERENT) // Allocating in L2
} VC_ALLOC_FLAGS;

// DMA register definitions
#define DMA_CHAN        5
#define DMA_PWM_DREQ    5
#define DMA_BASE        (PHYS_REG_BASE + 0x007000)
#define DMA_CS          (DMA_CHAN*0x100)
#define DMA_CONBLK_AD   (DMA_CHAN*0x100 + 0x04)
#define DMA_TI          (DMA_CHAN*0x100 + 0x08)
#define DMA_SRCE_AD     (DMA_CHAN*0x100 + 0x0c)
#define DMA_DEST_AD     (DMA_CHAN*0x100 + 0x10)
#define DMA_TXFR_LEN    (DMA_CHAN*0x100 + 0x14)
#define DMA_STRIDE      (DMA_CHAN*0x100 + 0x18)
#define DMA_NEXTCONBK   (DMA_CHAN*0x100 + 0x1c)
#define DMA_DEBUG       (DMA_CHAN*0x100 + 0x20)
#define DMA_ENABLE      0xff0
#define VIRT_DMA_REG(a) ((volatile uint32_t *)((uint32_t)virt_dma_regs + a))

#define DMA_MEM_FLAGS (MEM_FLAG_DIRECT|MEM_FLAG_ZERO)


// Convert virtual DMA data address to a bus address
#define BUS_DMA_MEM(a)  ((uint32_t)a-(uint32_t)virt_dma_mem+(uint32_t)bus_dma_mem)

int mbox_fd, dma_mem_h;
void *bus_dma_mem;
// Virtual memory for DMA descriptors and data buffers (uncached)
void *virt_dma_mem, *virt_dma_regs;


// Mailbox command/response structure
typedef struct {
    uint32_t len,   // Overall length (bytes)
        req,        // Zero for request, 1<<31 for response
        tag,        // Command number
        blen,       // Buffer length (bytes)
        dlen;       // Data length (bytes)
        uint32_t uints[32-5];   // Data (108 bytes maximum)
} VC_MSG __attribute__ ((aligned (16)));


// fetch from system
//#define PHYS_REG_BASE  0xFE000000  // Pi 4 in  Low Peripheral mode (check with 'sudo vcgencmd get_config arm_peri_high')

//DMA channel 0 offset (both in bus and physical)
#define PAGE_SIZE   4096
//#define DMA_BASE 0x007000
#define DMA_MEM_SIZE    PAGE_SIZE

#define DMA_CHANNEL 5
#define DMA_SPACING 0x100

// Convert bus address to physical address
//#define BUS_TO_PHYS(x) ((x) & ~0xC0000000

//#define BUS_TO_PHYS(x) ((void *)((uint32_t)(x) + 0x80000000)) // For RPI 4
#define BUS_TO_PHYS(a) ((void *)((uint32_t)(a)&~0xC0000000))


// Documentation: BCM2835 ARM Peripherals @4.2.1.2
struct dma_channel_header {
  uint32_t cs;        // control and status.
  uint32_t cblock;    // control block address.
};

// @4.2.1.1
struct dma_cb {    // 32 bytes.
  uint32_t info;   // transfer information.
  uint32_t src;    // physical source address.
  uint32_t dst;    // physical destination address.
  uint32_t length; // transfer length.
  uint32_t stride; // stride mode.
  uint32_t next;   // next control block; Physical address. 32 byte aligned.
  uint32_t pad[2];
};


typedef struct DMAMemHandle
{
    void *virtual_addr; // Virutal base address of the memory block
    uint32_t bus_addr;  // Bus address of the memory block
    uint32_t mb_handle; // Used internally by mailbox property interface
} DMAMemHandle;

void *map_segment(void *addr, int size)
{
    int fd;
    void *mem;

    size = PAGE_ROUNDUP(size);
    if ((fd = open ("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC)) < 0)
        FAIL("Error: can't open /dev/mem, run using sudo\n");
    mem = mmap(0, size, PROT_WRITE|PROT_READ, MAP_SHARED, fd, (uint32_t)addr);
    close(fd);
    printf("Map %p -> %p\n", (void *)addr, mem);
    if (mem == MAP_FAILED)
        FAIL("Error: can't map memory\n");
    return(mem);
}


void *map_peripheral(uint32_t peri_offset, uint32_t size){
    uint32_t physical_base = bcm_host_get_peripheral_address();
  
    // Check mem(4) man page for "/dev/mem"
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);

    uint32_t *result = (uint32_t *)mmap(
        NULL,
        size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        physical_base + peri_offset); 

    close(mem_fd);

    return result;
}


// Open mailbox interface, return file descriptor
int open_mbox(void)
{
   int fd;
 
   if ((fd = open("/dev/vcio", 0)) < 0)
       FAIL("Error: can't open VC mailbox\n");
   return(fd);
}
void disp_vc_msg(VC_MSG *msgp)
{
    int i;

    printf("VC msg len=%X, req=%X, tag=%X, blen=%x, dlen=%x, data ",
        msgp->len, msgp->req, msgp->tag, msgp->blen, msgp->dlen);
    for (i=0; i<msgp->blen/4; i++)
        printf("%08X ", msgp->uints[i]);
    printf("\n");
}

// Send message to mailbox, return first response int, 0 if error
uint32_t msg_mbox(int fd, VC_MSG *msgp)
{
    uint32_t ret=0, i;
 
    for (i=msgp->dlen/4; i<=msgp->blen/4; i+=4)
        msgp->uints[i++] = 0;
    msgp->len = (msgp->blen + 6) * 4;
    msgp->req = 0;
    if (ioctl(fd, _IOWR(100, 0, void *), msgp) < 0)
        printf("VC IOCTL failed\n");
    else if ((msgp->req&0x80000000) == 0)
        printf("VC IOCTL error\n");
    else if (msgp->req == 0x80000001)
        printf("VC IOCTL partial error\n");
    else
        ret = msgp->uints[0];
    disp_vc_msg(msgp);
    return(ret);
}
// Allocate memory on PAGE_SIZE boundary, return handle
uint32_t alloc_vc_mem(int fd, uint32_t size, VC_ALLOC_FLAGS flags)
{
    VC_MSG msg={.tag=0x3000c, .blen=12, .dlen=12,
        .uints={PAGE_ROUNDUP(size), PAGE_SIZE, flags}};
    return(msg_mbox(fd, &msg));
}
// Lock allocated memory, return bus address
void *lock_vc_mem(int fd, int h)
{
    VC_MSG msg={.tag=0x3000d, .blen=4, .dlen=4, .uints={h}};
    return(h ? (void *)msg_mbox(fd, &msg) : 0);
}

// DMA control block (must be 32-byte aligned)
typedef struct {
    uint32_t ti,    // Transfer info
        srce_ad,    // Source address
        dest_ad,    // Destination address
        tfr_len,    // Transfer length
        stride,     // Transfer stride
        next_cb,    // Next control block
        debug,      // Debug register
        unused;
} DMA_CB __attribute__ ((aligned(32)));
#define DMA_CB_DEST_INC (1<<4)
#define DMA_CB_SRC_INC  (1<<8)


void enable_dma(void)
{
    *VIRT_DMA_REG(DMA_ENABLE) |= (1 << DMA_CHAN);
    *VIRT_DMA_REG(DMA_CS) = 1 << 31;
}

void stop_dma(void)
{
    if (virt_dma_regs)
        *VIRT_DMA_REG(DMA_CS) = 1 << 31;
}

void start_dma(DMA_CB *cbp)
{
	printf("Step 3.1\n");
    *VIRT_DMA_REG(DMA_CONBLK_AD) = BUS_DMA_MEM(cbp);
	printf("Step 3.2\n");
    *VIRT_DMA_REG(DMA_CS) = 2;       // Clear 'end' flag
	printf("Step 3.3\n");
    *VIRT_DMA_REG(DMA_DEBUG) = 7;    // Clear error bits
	printf("Step 3.4\n");
    *VIRT_DMA_REG(DMA_CS) = 1;       // Start DMA

}
char *dma_regstrs[] = {"DMA CS", "CB_AD", "TI", "SRCE_AD", "DEST_AD",
    "TFR_LEN", "STRIDE", "NEXT_CB", "DEBUG", ""};
void disp_dma(void)
{
    uint32_t *p=(uint32_t *)VIRT_DMA_REG(DMA_CS);
    int i=0;

    while (dma_regstrs[i][0])
    {
        printf("%-7s %08X ", dma_regstrs[i++], *p++);
        if (i%5==0 || dma_regstrs[i][0]==0)
            printf("\n");
    }
}

int main(void) {
//	void * peripheral_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
//	volatile dma_channel_hdr = (dma_channel_header *)(peripheral_base_ptr + DMA_CHANNEL * DMA_SPACING);

//	DMAMemHandle * dma_cbs = dma_malloc(1 * sizeof(DMAControlBlock));
//
//	printf("Base address: %X\n", dmac_cbs[0]->virtual_addr)
        virt_dma_regs = map_segment((void *)DMA_BASE, PAGE_SIZE);

	enable_dma();
	mbox_fd = open_mbox();
	    if ((dma_mem_h = alloc_vc_mem(mbox_fd, DMA_MEM_SIZE, DMA_MEM_FLAGS)) <= 0 ||
	        (bus_dma_mem = lock_vc_mem(mbox_fd, dma_mem_h)) == 0 ||
	        (virt_dma_mem = map_segment(BUS_TO_PHYS(bus_dma_mem), DMA_MEM_SIZE)) == 0)
	            FAIL("Error: can't allocate uncached memory\n");
	    printf("VC mem handle %u, phys %p, virt %p\n", dma_mem_h, bus_dma_mem, virt_dma_mem);

	printf("Step 1\n");

	DMA_CB *cbp = virt_dma_mem;
	char *srce = (char *)(cbp+1);
	char *dest = srce + 0x100;
	
	strcpy(srce, "memory transfer OK");
	printf("Step 2\n");
	memset(cbp, 0, sizeof(DMA_CB));
	cbp->ti = DMA_CB_SRC_INC | DMA_CB_DEST_INC;
	cbp->srce_ad = BUS_DMA_MEM(srce);
	cbp->dest_ad = BUS_DMA_MEM(dest);
	cbp->tfr_len = strlen(srce) + 1;
	    disp_dma();
	printf("Step 3\n");
	start_dma(cbp);
	printf("Step 4\n");
	usleep(100000);
	    disp_dma();
	printf("Step 4\n");
printf("%X\n", dest);
	printf("DMA test: %s\n", dest[0] ? dest : "failed");
	stop_dma();

}
