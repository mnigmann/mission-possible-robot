// DMA tests for the Raspberry Pi, see https://iosoft.blog for details
//
// Copyright (c) 2020 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.10 JPB 25/5/20
//

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "pwm.h"



// Main program
int main(int argc, char *argv[])
{
    // Ensure cleanup if user hits ctrl-C
    signal(SIGINT, terminate);

    

    pwm_begin(32, PWM_FREQ);
    // Set LED pin as output, and set high
    gpio_mode(LED_PIN, GPIO_OUT);
    gpio_mode(23, GPIO_OUT);
    
    //put_cb(0, (1<<LED_PIN)|(1<<23), 40);
    //put_cb(1, (1<<LED_PIN), 40);
    //put_cb(2, (1<<23), 80);
    pwm_set_period(4000);
    pwm_set_channel(LED_PIN, 500);
    pwm_set_channel(23, 100);
    DMA_CB *cbs = virt_dma_mem;
    for (int i=0; i < 9; i++) printf("  %08X: %08X\n", BUS_DMA_MEM(&pwm_data[i]), pwm_data[i]);
    for (int i=0; i < 6; i++) {
        printf("%02d: CB_ADDR: %08X, SRCE_AD: %08X, DEST_AD: %08X, NEXT_CB: %08X, OFFSET: %08X\n", i, BUS_DMA_MEM(&cbs[i]), cbs[i].srce_ad, cbs[i].dest_ad, cbs[i].next_cb, cbs[i].unused);
    }
    sleep(1);
    for (uint32_t servo = 500; servo > 100; servo-=4) {
        pwm_set_channel(LED_PIN, servo);
        pwm_set_channel(23, 600-servo);
        usleep(100000);
    }
    terminate(0);
}

void pwm_set_period(uint32_t period) {
    DMA_CB *cbs = virt_dma_mem;
    uint8_t last = 0;
    for (uint8_t i=1; i < 33; i++) {
        if (cbs[i*2].unused > cbs[last*2].unused) last = i;
    }
    printf("Last in chain: %d with %d\n", last, cbs[last*2].unused);
    pwm_data[last*3 + 1] = period - cbs[last*2].unused;
}

void put_cb(uint8_t idx, uint32_t mask, uint32_t delay) {
    idx = 3*idx;
    pwm_period += (delay - pwm_data[idx+1]);
    pwm_data[idx] = mask;
    pwm_data[idx+1] = delay;
    pwm_data[idx+2] = 0;
}

void pwm_set_channel(uint8_t pin, uint32_t on_time) {
    DMA_CB *cbs = virt_dma_mem;
    if (on_time) {
        pwm_data[0] |= (1<<pin);
        uint8_t old_pos = 0;
        uint8_t empty_pos = 0;
        uint8_t above = 0;      // Index of the CB executed after the new time
        uint8_t below = 0;      // Index of the CB executed before the new time
        uint8_t equal = 0;
        for (uint8_t i=1; i < 33; i++) {
            if (cbs[i*2].unused >= on_time - 4 && cbs[i*2].unused <= on_time + 4) equal = i;
            if (pwm_data[i*3] & (1<<pin)) old_pos = i;
            if (pwm_data[i*3] == 0 && empty_pos == 0) empty_pos = i;
            if (cbs[i*2].unused < on_time && cbs[i*2].unused > cbs[below*2].unused) below = i;
            if (cbs[i*2].unused > on_time && (above == 0 || cbs[i*2].unused < cbs[above*2].unused)) above = i;
        }

        if (old_pos > 0) {
            if (pwm_data[old_pos*3] == (1<<pin)) {
                uint8_t below_old = 0;
                for (uint8_t i=1; i < 33; i++) {
                    if (cbs[i*2+1].next_cb == BUS_DMA_MEM(&cbs[old_pos*2])) below_old = i;
                }
                // If a control block exists that only controls this pin, empty that control block and bypass it
                // When doing this, one must be careful about the fact that each CB points to the pwm_data block
                // that is one edge later.
                printf("CB %d controls %d, the previous is at %d\n", old_pos, pin, below_old);
                fflush(stdout);
                cbs[2*below_old + 1].next_cb = cbs[2*old_pos + 1].next_cb;
                cbs[2*below_old + 1].srce_ad = cbs[2*old_pos + 1].srce_ad;
                cbs[2*old_pos].unused = 0;
                pwm_data[old_pos*3] = 0;
                pwm_data[below_old*3 + 1] += pwm_data[old_pos*3 + 1];
            } else {
                pwm_data[old_pos*3] &= ~(1<<pin);
            }

            pwm_set_channel(pin, on_time);
            return;
        } else if (equal > 0) {
            pwm_data[3*equal] |= (1<<pin);
            return;
        }

        printf("No CB found controlling pin %d, %d is empty; %d above and %d below\n", pin, empty_pos, above, below);
        printf("above time: %d, below time: %d, on time: %d, calc. delay: %d\n", cbs[2*above].unused, cbs[2*below].unused, on_time, cbs[below*2].unused+pwm_data[below*3+1]-on_time);
        cbs[empty_pos*2+1].next_cb = BUS_DMA_MEM(&cbs[above*2]);
        cbs[empty_pos*2+1].srce_ad = BUS_DMA_MEM(&pwm_data[above*3 + 1]);
        cbs[below*2+1].next_cb = BUS_DMA_MEM(&cbs[empty_pos*2]);
        cbs[below*2+1].srce_ad = BUS_DMA_MEM(&pwm_data[empty_pos*3 + 1]);
        pwm_data[empty_pos*3+1] = cbs[below*2].unused + pwm_data[below*3+1] - on_time;
        pwm_data[below*3+1] = on_time - cbs[below*2].unused;
        pwm_data[empty_pos*3] = (1<<pin);
        cbs[empty_pos*2].unused = on_time;
        
    } else {
        pwm_data[0] &= ~(1<<pin);
    }
}

void pwm_begin(uint8_t num_ch, int freq) {
    pwm_n_channels = num_ch;
    // Map GPIO, DMA and PWM registers into virtual mem (user space)
    virt_gpio_regs = map_segment((void *)GPIO_BASE, PAGE_SIZE);
    virt_dma_regs = map_segment((void *)DMA_BASE, PAGE_SIZE);
    virt_pwm_regs = map_segment((void *)PWM_BASE, PAGE_SIZE);
    virt_clk_regs = map_segment((void *)CLK_BASE, PAGE_SIZE);

    enable_dma();

    // Use mailbox to get uncached memory for DMA decriptors and buffers
    mbox_fd = open_mbox();
    if ((dma_mem_h = alloc_vc_mem(mbox_fd, DMA_MEM_SIZE, DMA_MEM_FLAGS)) <= 0 ||
        (bus_dma_mem = lock_vc_mem(mbox_fd, dma_mem_h)) == 0 ||
        (virt_dma_mem = map_segment(BUS_PHYS_ADDR(bus_dma_mem), DMA_MEM_SIZE)) == 0)
            FAIL("Error: can't allocate uncached memory\n");
    printf("VC mem handle %u, phys %p, virt %p\n", dma_mem_h, bus_dma_mem, virt_dma_mem);
    pwm_data = (uint32_t *)(virt_dma_mem + 2112);
    memset(virt_dma_mem, 0, DMA_MEM_SIZE);

    // The "unused" field in the DMA control block struct is used to store the time
    // between the execution of the first control block and the execution of that
    // control block. This is equivalent to the sum of all of the delays following
    // all the control blocks up to but not including that control block.
    DMA_CB * cbs = virt_dma_mem;
    num_ch = (num_ch + 1)*2;
    for (uint8_t i=0; i < num_ch; i += 2) {
        cbs[i].ti = (1 << 6) | (DMA_PWM_DREQ << 16) | DMA_CB_SRC_INC | DMA_CB_DEST_INC;
        cbs[i].srce_ad = BUS_DMA_MEM(pwm_data + i/2 * 3);
        cbs[i].dest_ad = BUS_GPIO_REG(i ? GPIO_CLR0 : GPIO_SET0);
        cbs[i].tfr_len = 4;
        cbs[i].next_cb = BUS_DMA_MEM(&cbs[i+1]);
        put_cb(i/2, 0, 6);
    }
    for (uint8_t i=1; i < num_ch; i += 2) {
        cbs[i].ti = (1 << 6) | (DMA_PWM_DREQ << 16) | DMA_CB_SRC_INC | DMA_CB_DEST_INC | (1<<1);
        if (i+1 < num_ch) cbs[i].srce_ad = BUS_DMA_MEM(pwm_data + (i-1)/2 * 3 + 4);
        else cbs[i].srce_ad = BUS_DMA_MEM(pwm_data + 1);
        //cbs[i].srce_ad = BUS_DMA_MEM(pwm_data + (i-1)/2 * 3 + 1);
        cbs[i].dest_ad = BUS_PWM_REG(PWM_RNG1);
        cbs[i].tfr_len = (2 << 16) | 4;
        cbs[i].stride = 4 << 16;
        //cbs[i].next_cb = BUS_DMA_MEM(&cbs[(i+1)%num_ch]);
    }
    cbs[1].next_cb = BUS_DMA_MEM(cbs);
    pwm_period = num_ch*3;
    init_pwm(freq);
    *VIRT_PWM_REG(PWM_DMAC) = PWM_DMAC_ENAB|16;
    start_pwm();
    // This line is very important. Without this printf, the DMA doesn't load correctly.
    // One could also use usleep(100000) instead, even though the delay caused by the
    // printf is much lower. Why this is needed is still not known...
    printf("\n");
    start_dma(virt_dma_mem);
    disp_dma();
}

// DMA trigger test: fLash LED using PWM trigger
void attach_pwm(int pin)
{
    gpio_out(pin, 0);
    DMA_CB *cbs=virt_dma_mem;
    uint32_t n, *pindata=(uint32_t *)(cbs+4), *pwmdata_h=pindata+1, *pwmdata_l=pindata+4;

    printf("DMA test: PWM trigger, ctrl-C to exit (%p (%08X) %p %p %p)\n", cbs, BUS_DMA_MEM(cbs), pindata, pwmdata_h, pwmdata_l);
    memset(cbs, 0, sizeof(DMA_CB)*4);
    *VIRT_PWM_REG(PWM_RNG1) = 1;
    // Transfers are triggered by PWM request
    cbs[0].ti = cbs[2].ti = (1 << 6) | (DMA_PWM_DREQ << 16) | DMA_CB_SRC_INC | DMA_CB_DEST_INC;
    cbs[1].ti = cbs[3].ti = (1 << 6) | (DMA_PWM_DREQ << 16) | DMA_CB_SRC_INC | DMA_CB_DEST_INC | (1<<1);
    // Control block 0 and 2: clear & set LED pin, 4-byte transfer
    cbs[0].srce_ad = cbs[2].srce_ad = BUS_DMA_MEM(pindata);
    cbs[0].dest_ad = BUS_GPIO_REG(GPIO_CLR0);
    cbs[2].dest_ad = BUS_GPIO_REG(GPIO_SET0);
    cbs[0].tfr_len = cbs[2].tfr_len = 4;
    *pindata = 1 << pin;
    // Control block 1 and 3: update PWM FIFO (to clear DMA request)
    cbs[1].srce_ad = BUS_DMA_MEM(pwmdata_h);
    cbs[3].srce_ad = BUS_DMA_MEM(pwmdata_l);
    cbs[1].dest_ad = cbs[3].dest_ad = BUS_PWM_REG(PWM_RNG1);
    cbs[1].tfr_len = cbs[3].tfr_len = (2 << 16) | 4;
    cbs[1].stride = cbs[3].stride = (4 << 16);
    pwmdata_h[0] = 2*PWM_INCREMENT*3;
    pwmdata_h[1] = 1;
    pwmdata_l[0] = 2*PWM_INCREMENT*37;
    pwmdata_l[1] = 1;
    // Link control blocks 0 to 3 in endless loop
    for (n=0; n<4; n++)
        cbs[n].next_cb = BUS_DMA_MEM(&cbs[(n+1)%4]);
    // Enable PWM with data threshold 1, and DMA
    init_pwm(PWM_FREQ);
    *VIRT_PWM_REG(PWM_DMAC) = PWM_DMAC_ENAB|1;
    start_pwm();
    uint32_t sta = *VIRT_PWM_REG(PWM_STA);
    uint32_t ctl = *VIRT_PWM_REG(PWM_CTL);
    uint32_t rng = *VIRT_PWM_REG(PWM_RNG1);
    uint32_t dstat = *VIRT_DMA_REG(DMA_CS);
//    while ((*VIRT_PWM_REG(PWM_STA) & (1<<1)) == 0);
//    printf("loading %p with value %x\n", (&cbs[0]), BUS_DMA_MEM(&cbs[0]));
//    printf("loading ################################# %x\n", *VIRT_PWM_REG(PWM_STA));
//    while ((*VIRT_PWM_REG(PWM_STA) & (1<<1)) == 0);
//    printf("\n");
    usleep(100000);
    start_dma(cbs);
    disp_dma();
    printf("PWM status was %08X, control was %08X, range was %08X, dma status was %08X, current is %08X\n", sta, ctl, rng, dstat, *VIRT_DMA_REG(DMA_CONBLK_AD));
    // Nothing to do while LED is flashing
    printf("Output to center\n");
    sleep(1);
    pwmdata_h[0] = 2*PWM_INCREMENT*2;
    pwmdata_l[0] = 2*PWM_INCREMENT*38;
    printf("Output to min\n");
    sleep(1);
    pwmdata_h[0] = 2*PWM_INCREMENT*4;
    pwmdata_l[0] = 2*PWM_INCREMENT*36;
    printf("Output to max\n");
    sleep(1);
    pwmdata_h[0] = 2*PWM_INCREMENT*3;
    pwmdata_l[0] = 2*PWM_INCREMENT*37;
    printf("Output to center\n");
    sleep(1);
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
uint32_t set_vc_clock(int fd, int id, uint32_t freq) {
    VC_MSG msg1={.tag=0x38001, .blen=8, .dlen=8, .uints={id, 1}};
    VC_MSG msg2={.tag=0x38002, .blen=12, .dlen=12, .uints={id, freq, 0}};
    msg_mbox(fd, &msg1);
    disp_vc_msg(&msg1);
    msg_mbox(fd, &msg2);
    disp_vc_msg(&msg2);
    return(0);
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
#if USE_VC_CLOCK_SET
    set_vc_clock(mbox_fd, PWM_CLOCK_ID, freq);
#else
    int divi=(CLOCK_KHZ*1000) / freq;
    *VIRT_CLK_REG(CLK_PWM_CTL) = CLK_PASSWD | (1 << 5);
    while (*VIRT_CLK_REG(CLK_PWM_CTL) & (1 << 7)) ;
    *VIRT_CLK_REG(CLK_PWM_DIV) = CLK_PASSWD | (divi << 12);
    *VIRT_CLK_REG(CLK_PWM_CTL) = CLK_PASSWD | 6 | (1 << 4);
    while ((*VIRT_CLK_REG(CLK_PWM_CTL) & (1 << 7)) == 0) ;
#endif
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

// EOF
