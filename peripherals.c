#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "peripherals.h"

char *dma_regstrs[] = {"DMA CS", "CB_AD", "TI", "SRCE_AD", "DEST_AD",
    "TFR_LEN", "STRIDE", "NEXT_CB", "DEBUG", ""};

void init_memory() {
    // Map GPIO, DMA and PWM registers into virtual mem (user space)
    virt_gpio_regs = map_segment((void *)GPIO_BASE, PAGE_SIZE);
    virt_dma_regs = map_segment((void *)DMA_BASE, PAGE_SIZE);
    virt_pwm_regs = map_segment((void *)PWM_BASE, PAGE_SIZE);
    virt_clk_regs = map_segment((void *)CLK_BASE, PAGE_SIZE);


    // Use mailbox to get uncached memory for DMA decriptors and buffers
    mbox_fd = open_mbox();
    if ((dma_mem_h = alloc_vc_mem(mbox_fd, DMA_MEM_SIZE, DMA_MEM_FLAGS)) <= 0 ||
        (bus_dma_mem = lock_vc_mem(mbox_fd, dma_mem_h)) == 0 ||
        (virt_dma_mem = map_segment(BUS_PHYS_ADDR(bus_dma_mem), DMA_MEM_SIZE)) == 0)
            FAIL("Error: can't allocate uncached memory\n");
    printf("VC mem handle %u, phys %p, virt %p\n", dma_mem_h, bus_dma_mem, virt_dma_mem);
    pwm_data = (uint32_t *)(virt_dma_mem + 2112);
    memset(virt_dma_mem, 0, DMA_MEM_SIZE);
}

// Free memory segments and exit 
void terminate(int sig) 
{ 
    printf("Closing\n"); 
    stop_pwm(); 
    stop_dma(); 
    gpio_out(LED_PIN, 0); 
    unmap_segment(virt_dma_mem, DMA_MEM_SIZE); 
    unlock_vc_mem(mbox_fd, dma_mem_h); 
    free_vc_mem(mbox_fd, dma_mem_h); 
    close_mbox(mbox_fd); 
    unmap_segment(virt_clk_regs, PAGE_SIZE); 
    unmap_segment(virt_pwm_regs, PAGE_SIZE); 
    unmap_segment(virt_dma_regs, PAGE_SIZE); 
    unmap_segment(virt_gpio_regs, PAGE_SIZE); 
    exit(0); 
} 


// ----- GPIO -----

// Set input or output
void gpio_mode(int pin, int mode) {
    uint32_t *reg = VIRT_GPIO_REG(GPIO_MODE0) + pin / 10, shift = (pin % 10) * 3;
    *reg = (*reg & ~(7 << shift)) | (mode << shift);
}

// Set an O/P pin
void gpio_out(int pin, int val) {
    uint32_t *reg = VIRT_GPIO_REG(val ? GPIO_SET0 : GPIO_CLR0) + pin/32;
    *reg = 1 << (pin % 32);
}

// Get an I/P pin value
uint8_t gpio_in(int pin) {
    uint32_t *reg = VIRT_GPIO_REG(GPIO_LEV0) + pin/32;
    return (((*reg) >> (pin % 32)) & 1);
}

// ----- VIDEOCORE MAILBOX -----

// Open mailbox interface, return file descriptor
int open_mbox(void) {
   int fd;

   if ((fd = open("/dev/vcio", 0)) < 0)
       FAIL("Error: can't open VC mailbox\n");
   return(fd);
}
// Close mailbox interface
void close_mbox(int fd) {
    if (fd >= 0) close(fd);
}

// Send message to mailbox, return first response int, 0 if error
uint32_t msg_mbox(int fd, VC_MSG *msgp) {
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
#if DEBUG
    disp_vc_msg(msgp);
#endif
    return(ret);
}

// Allocate memory on PAGE_SIZE boundary, return handle
uint32_t alloc_vc_mem(int fd, uint32_t size, VC_ALLOC_FLAGS flags) {
    VC_MSG msg={.tag=0x3000c, .blen=12, .dlen=12,
        .uints={PAGE_ROUNDUP(size), PAGE_SIZE, flags}};
    return(msg_mbox(fd, &msg));
}
// Lock allocated memory, return bus address
void *lock_vc_mem(int fd, int h) {
    VC_MSG msg={.tag=0x3000d, .blen=4, .dlen=4, .uints={h}};
    return(h ? (void *)msg_mbox(fd, &msg) : 0);
}
// Unlock allocated memory
uint32_t unlock_vc_mem(int fd, int h) {
    VC_MSG msg={.tag=0x3000e, .blen=4, .dlen=4, .uints={h}};
    return(h ? msg_mbox(fd, &msg) : 0);
}
// Free memory
uint32_t free_vc_mem(int fd, int h) {
    VC_MSG msg={.tag=0x3000f, .blen=4, .dlen=4, .uints={h}};
    return(h ? msg_mbox(fd, &msg) : 0);
}

// Display mailbox message
void disp_vc_msg(VC_MSG *msgp) {
    int i;

    printf("VC msg len=%X, req=%X, tag=%X, blen=%x, dlen=%x, data ",
        msgp->len, msgp->req, msgp->tag, msgp->blen, msgp->dlen);
    for (i=0; i<msgp->blen/4; i++)
        printf("%08X ", msgp->uints[i]);
    printf("\n");
}

// ----- VIRTUAL MEMORY -----

// Get virtual memory segment for peripheral regs or physical mem
void *map_segment(void *addr, int size)
{
    int fd;
    void *mem;

    size = PAGE_ROUNDUP(size);
    if ((fd = open ("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC)) < 0)
        FAIL("Error: can't open /dev/mem, run using sudo\n");
    mem = mmap(0, size, PROT_WRITE|PROT_READ, MAP_SHARED, fd, (uint32_t)addr);
    close(fd);
#if DEBUG
    printf("Map %p -> %p\n", (void *)addr, mem);
#endif
    if (mem == MAP_FAILED)
        FAIL("Error: can't map memory\n");
    return(mem);
}
// Free mapped memory
void unmap_segment(void *mem, int size)
{
    if (mem)
        munmap(mem, PAGE_ROUNDUP(size));
}

// ----- DMA -----

// Enable and reset DMA
void enable_dma(void)
{
    *VIRT_DMA_REG(DMA_ENABLE) |= (1 << DMA_CHAN);
    *VIRT_DMA_REG(DMA_CS) = 1 << 31;
}

// Start DMA, given first control block
void start_dma(DMA_CB *cbp)
{
//    printf("Starting %p (value is %x)\n", cbp, BUS_DMA_MEM(cbp));
    *VIRT_DMA_REG(DMA_CONBLK_AD) = BUS_DMA_MEM(cbp);
    *VIRT_DMA_REG(DMA_CS) = 2;       // Clear 'end' flag
    *VIRT_DMA_REG(DMA_DEBUG) = 7;    // Clear error bits
    *VIRT_DMA_REG(DMA_CS) = 1;       // Start DMA
}

// Halt current DMA operation by resetting controller
void stop_dma(void)
{
    if (virt_dma_regs)
        *VIRT_DMA_REG(DMA_CS) = 1 << 31;
}

// Display DMA registers
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

// ----- PWM -----

// Initialise PWM
void init_pwm(int freq)
{
    stop_pwm();
    if (*VIRT_PWM_REG(PWM_STA) & 0x100)
    {
        printf("PWM bus error\n");
        *VIRT_PWM_REG(PWM_STA) = 0x100;
    }

    int divi=(CLOCK_KHZ*1000) / freq;
    *VIRT_CLK_REG(CLK_PWM_CTL) = CLK_PASSWD | (1 << 5);
    while (*VIRT_CLK_REG(CLK_PWM_CTL) & (1 << 7)) ;
    *VIRT_CLK_REG(CLK_PWM_DIV) = CLK_PASSWD | (divi << 12);
    *VIRT_CLK_REG(CLK_PWM_CTL) = CLK_PASSWD | 6 | (1 << 4);
    while ((*VIRT_CLK_REG(CLK_PWM_CTL) & (1 << 7)) == 0) ;

    usleep(100);
//    *VIRT_PWM_REG(PWM_RNG1) = PWM_RANGE;
//    *VIRT_PWM_REG(PWM_FIF1) = PWM_RANGE/2;
#if PWM_OUT
    gpio_mode(PWM_PIN, GPIO_ALT5);
#endif
}

// Start PWM operation
void start_pwm(void)
{
    *VIRT_PWM_REG(PWM_CTL) = PWM_CTL_USEF1 | PWM_ENAB | (1<<7) | (1<<6);
}

// Stop PWM operation
void stop_pwm(void)
{
    if (virt_pwm_regs)
    {
        *VIRT_PWM_REG(PWM_CTL) = 0;
        usleep(100);
    }
}

