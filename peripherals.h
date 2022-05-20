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

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

// Location of peripheral registers in physical memory
#define PHYS_REG_BASE  0x20000000  // Pi Zero or 1
//#define PHYS_REG_BASE    0x3F000000  // Pi 2 or 3
//#define PHYS_REG_BASE  0xFE000000  // Pi 4

// Location of peripheral registers in bus memory
#define BUS_REG_BASE    0x7E000000

// If non-zero, print debug information
#ifndef DEBUG
#define DEBUG           0
#endif

// Output pin to use for LED
//#define LED_PIN         47    // Pi Zero onboard LED
#define LED_PIN         22      // Offboard LED pin

// PWM clock frequency and range (FREQ/RANGE = LED flash freq)
#define PWM_FREQ        100000
#define PWM_INCREMENT   50
#define PWM_RANGE       20000

// If non-zero, set PWM clock using VideoCore mailbox
#define USE_VC_CLOCK_SET 0

// Size of memory page
#define PAGE_SIZE       0x1000
// Round up to nearest page
#define PAGE_ROUNDUP(n) ((n)%PAGE_SIZE==0 ? (n) : ((n)+PAGE_SIZE)&~(PAGE_SIZE-1))

// Size of uncached memory for DMA control blocks and data
#define DMA_MEM_SIZE    PAGE_SIZE

// GPIO definitions
#define GPIO_BASE       (PHYS_REG_BASE + 0x200000)
#define GPIO_MODE0      0x00
#define GPIO_SET0       0x1c
#define GPIO_CLR0       0x28
#define GPIO_LEV0       0x34
#define GPIO_GPPUD      0x94
#define GPIO_GPPUDCLK0  0x98
#define VIRT_GPIO_REG(a) ((uint32_t *)((uint32_t)virt_gpio_regs + (a)))
#define BUS_GPIO_REG(a) (GPIO_BASE-PHYS_REG_BASE+BUS_REG_BASE+(uint32_t)(a))
#define GPIO_IN         0
#define GPIO_OUT        1
#define GPIO_ALT0       4
#define GPIO_ALT2       6
#define GPIO_ALT3       7
#define GPIO_ALT4       3
#define GPIO_ALT5       2
#define GPIO_PUD_OFF    0
#define GPIO_PUD_DOWN   1
#define GPIO_PUD_UP     2

// Virtual memory pointers to acceess GPIO, DMA and PWM from user space
void *virt_gpio_regs, *virt_dma_regs, *virt_pwm_regs, *virt_spi_regs;
// VC mailbox file descriptor & handle, and bus memory pointer
int mbox_fd, dma_mem_h;
void *bus_dma_mem;

// Convert memory bus address to physical address (for mmap)
#define BUS_PHYS_ADDR(a) ((void *)((uint32_t)(a)&~0xC0000000))

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
// VC flags for unchached DMA memory
#define DMA_MEM_FLAGS (MEM_FLAG_DIRECT|MEM_FLAG_ZERO)

// Mailbox command/response structure
typedef struct {
    uint32_t len,   // Overall length (bytes)
        req,        // Zero for request, 1<<31 for response
        tag,        // Command number
        blen,       // Buffer length (bytes)
        dlen;       // Data length (bytes)
        uint32_t uints[32-5];   // Data (108 bytes maximum)
} VC_MSG __attribute__ ((aligned (16)));

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
char *dma_regstrs[10];

// DMA control block (must be 32-byte aligned)
typedef struct {
    uint32_t ti,    // Transfer info
        srce_ad,    // Source address
        dest_ad,    // Destination address
        tfr_len,    // Transfer length
        stride,     // Transfer stride
        next_cb,    // Next control block
        debug,      // Debug register, zero in control block
        unused;
} DMA_CB __attribute__ ((aligned(32)));
#define DMA_CB_DEST_INC (1<<4)
#define DMA_CB_SRC_INC  (1<<8)

// Virtual memory for DMA descriptors and data buffers (uncached)
void *virt_dma_mem;

// Convert virtual DMA data address to a bus address
#define BUS_DMA_MEM(a)  ((uint32_t)(a)-(uint32_t)virt_dma_mem+(uint32_t)bus_dma_mem)

// PWM controller
#define PWM_BASE        (PHYS_REG_BASE + 0x20C000)
#define PWM_CTL         0x00   // Control
#define PWM_STA         0x04   // Status
#define PWM_DMAC        0x08   // DMA control
#define PWM_RNG1        0x10   // Channel 1 range
#define PWM_DAT1        0x14   // Channel 1 data
#define PWM_FIF1        0x18   // Channel 1 fifo
#define PWM_RNG2        0x20   // Channel 2 range
#define PWM_DAT2        0x24   // Channel 2 data
#define VIRT_PWM_REG(a) ((volatile uint32_t *)((uint32_t)virt_pwm_regs + (a)))
#define BUS_PWM_REG(a)  (PWM_BASE-PHYS_REG_BASE+BUS_REG_BASE+(uint32_t)(a))
#define PWM_CTL_RPTL1   (1<<2)  // Chan 1: repeat last data when FIFO empty
#define PWM_CTL_USEF1   (1<<5)  // Chan 1: use FIFO
#define PWM_DMAC_ENAB   (1<<31) // Start PWM DMA
#define PWM_ENAB        1       // Enable PWM
#define PWM_PIN         18      // GPIO pin for PWM output
#define PWM_OUT         1       // Set non-zero to enable PWM output pin

// SPI control registers
#define SPI_BASE        (PHYS_REG_BASE + 0x204000)
#define SPI_CS          0x00
#define SPI_FIFO        0x04
#define SPI_CLK         0x08
#define SPI_DLEN        0x0c
#define SPI_LTOH        0x10
#define SPI_DC          0x14
#define VIRT_SPI_REG(a) ((volatile uint32_t *)((uint32_t)virt_spi_regs + (a)))

// Clock
void *virt_clk_regs;
#define CLK_BASE        (PHYS_REG_BASE + 0x101000)
#define CLK_PWM_CTL     0xa0
#define CLK_PWM_DIV     0xa4
#define VIRT_CLK_REG(a) ((volatile uint32_t *)((uint32_t)virt_clk_regs + (a)))
#define CLK_PASSWD      0x5a000000
#define CLOCK_KHZ       250000
#define PWM_CLOCK_ID    0xa

#define FAIL(x) {printf(x); terminate(0);}
#define LOG(fmt, ...) fprintf(logptr, "[%12ld] " fmt, (long int)(time(NULL)), ##__VA_ARGS__)
#define LOG_FLUSH fflush(logptr)

FILE *logptr;

// Define a location for the pwm cycle lengths.
// With a total of 32 channels, there must be 64 CBs total.
// Each channel has one CB for setting the pin (even) and
// one CB for setting the timer (odd). The even CB uses one
// byte to store the pin mask, and the odd CB uses two to
// store the range and on-time. There is one additional
// set of bytes (the first set) for the common CBs that
// reset the PWM cycle. Therefore, pwm_data[0] is the mask
// of channels with PWM enabled.
// 2112 = (32 bytes per CB) * (2 common CB + (32 channels) * (2 CB per channel))
uint32_t *pwm_data;
uint32_t *pwm_data_buffer;

DMA_CB *cbs, *cbs_buffer;

uint32_t pwm_period;
uint32_t pwm_n_channels;

int main_server(int *pipes);
void close_conn(int c);
uint8_t serve_file(char *headers, char *filename);

int main_remote(int *pipes);

//int dma_test_mem_transfer(void);
//void dma_test_led_flash(int pin);
void pwm_set_period(uint32_t period);
void put_cb(uint8_t idx, uint32_t mask, uint32_t delay);
void pwm_set_channel(uint8_t pin, uint32_t on_time);
void pwm_begin(uint8_t num_ch, int freq);
void pwm_update();

double measure(int ch, uint32_t sampling_interval, uint8_t csmode);
double measure_precise(int ch, uint32_t sampling_interval, uint8_t csmode);

void init_memory();

void terminate(int sig);
void gpio_mode(int pin, int mode); 
void gpio_out(int pin, int val); 
uint8_t gpio_in(int pin);
void gpio_pud(int pin, uint8_t mode);
int open_mbox(void); 
void close_mbox(int fd); 
uint32_t msg_mbox(int fd, VC_MSG *msgp); 
void *map_segment(void *addr, int size); 
void unmap_segment(void *addr, int size); 
uint32_t alloc_vc_mem(int fd, uint32_t size, VC_ALLOC_FLAGS flags); 
void *lock_vc_mem(int fd, int h); 
uint32_t unlock_vc_mem(int fd, int h); 
uint32_t free_vc_mem(int fd, int h); 
uint32_t set_vc_clock(int fd, int id, uint32_t freq); 
void disp_vc_msg(VC_MSG *msgp);
void enable_dma(void);
void start_dma(DMA_CB *cbp);
void stop_dma(void);
void disp_dma(void);
void init_pwm(int freq);
void start_pwm(void);
void stop_pwm(void);

#endif

