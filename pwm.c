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
#include "peripherals.h"



// Main program
// It is necessary to include -DMAIN_IS_PWM when compiling
#ifdef MAIN_IS_PWM
int main(int argc, char *argv[])
{
    // Ensure cleanup if user hits ctrl-C
    signal(SIGINT, terminate);

    
    init_memory();
    pwm_begin(32, PWM_FREQ);
    // Set LED pin as output, and set high
    gpio_mode(17, GPIO_OUT);
    gpio_mode(23, GPIO_OUT);
    gpio_mode(18, GPIO_ALT5);

    //put_cb(0, (1<<LED_PIN)|(1<<23), 40);
    //put_cb(1, (1<<LED_PIN), 40);
    //put_cb(2, (1<<23), 80);
    pwm_set_period(4000);
    
//    pwm_set_channel(18, 500);
    pwm_set_channel(23, 100);
    pwm_set_channel(17, 500);
    pwm_update();
    for (int i=0; i < 9; i++) printf("  %08X: %08X\n", BUS_DMA_MEM(&pwm_data[i]), pwm_data[i]);
    for (int i=0; i < 6; i++) {
        printf("%02d: CB_ADDR: %08X, SRCE_AD: %08X, DEST_AD: %08X, NEXT_CB: %08X, OFFSET: %08X\n", i, BUS_DMA_MEM(&cbs[i]), cbs[i].srce_ad, cbs[i].dest_ad, cbs[i].next_cb, cbs[i].unused);
    }
    sleep(2);
    disp_dma();
    pwm_set_channel(17, 300);
    pwm_set_channel(23, 300);
    pwm_update();
    sleep(2);
    pwm_set_channel(17, 100);
    pwm_set_channel(23, 500);
    pwm_update();
    sleep(2);
    /*
    for (uint32_t servo = 500; servo > 100; servo-=4) {
        pwm_set_channel(18, servo);
        while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
        pwm_update();
        for (int i=0; i < 9; i++) printf("  %08X: %08X\n", BUS_DMA_MEM(&pwm_data[i]), pwm_data[i]);
        // pwm_set_channel(23, 600-servo);
        usleep(100000);
    }*/
    terminate(0);
}
#endif

void pwm_set_period(uint32_t period) {
    DMA_CB *cbs = virt_dma_mem;
    uint8_t last = 0;
    for (uint8_t i=1; i < 33; i++) {
        if (cbs[i*2].unused > cbs[last*2].unused) last = i;
    }
#if DEBUG
    printf("Last in chain: %d with %d\n", last, cbs[last*2].unused);
#endif
    pwm_data[last*3 + 1] = period - cbs[last*2].unused;
    memcpy(pwm_data_buffer, pwm_data, 396);
    memcpy(cbs_buffer, cbs, 2112);
}

void put_cb(uint8_t idx, uint32_t mask, uint32_t delay) {
    idx = 3*idx;
    pwm_period += (delay - pwm_data[idx+1]);
    pwm_data[idx] = mask;
    pwm_data[idx+1] = delay;
    pwm_data[idx+2] = 1;
}

void pwm_set_channel(uint8_t pin, uint32_t on_time) {
    DMA_CB *cbs = virt_dma_mem;
    if (on_time) {
        pwm_data_buffer[0] |= (1<<pin);
        uint8_t old_pos = 0;
        uint8_t empty_pos = 0;
        uint8_t above = 0;      // Index of the CB executed after the new time
        uint8_t below = 0;      // Index of the CB executed before the new time
        uint8_t equal = 0;
        for (uint8_t i=1; i < 33; i++) {
            if (cbs_buffer[i*2].unused >= on_time - 4 && cbs_buffer[i*2].unused <= on_time + 4) equal = i;
            if (pwm_data_buffer[i*3] & (1<<pin)) old_pos = i;
            if (pwm_data_buffer[i*3] == 0 && empty_pos == 0) empty_pos = i;
            if (cbs_buffer[i*2].unused < on_time && cbs_buffer[i*2].unused > cbs_buffer[below*2].unused) below = i;
            if (cbs_buffer[i*2].unused > on_time && (above == 0 || cbs_buffer[i*2].unused < cbs_buffer[above*2].unused)) above = i;
        }

        if (old_pos > 0) {
            if (pwm_data_buffer[old_pos*3] == (1<<pin)) {
                uint8_t below_old = 0;
                for (uint8_t i=1; i < 33; i++) {
                    if (cbs_buffer[i*2+1].next_cb == BUS_DMA_MEM(&cbs[old_pos*2])) below_old = i;
                }
                // If a control block exists that only controls this pin, empty that control block and bypass it
                // When doing this, one must be careful about the fact that each CB points to the pwm_data block
                // that is one edge later.
#if DEBUG
                printf("CB %d controls %d, the previous is at %d\n", old_pos, pin, below_old);
                fflush(stdout);
#endif
                cbs_buffer[2*below_old + 1].next_cb = cbs_buffer[2*old_pos + 1].next_cb;
                cbs_buffer[2*below_old + 1].srce_ad = cbs_buffer[2*old_pos + 1].srce_ad;
                cbs_buffer[2*old_pos].unused = 0;
                pwm_data_buffer[old_pos*3] = 0;
                pwm_data_buffer[below_old*3 + 1] += pwm_data_buffer[old_pos*3 + 1];
            } else {
                pwm_data_buffer[old_pos*3] &= ~(1<<pin);
            }

            pwm_set_channel(pin, on_time);
            return;
        } else if (equal > 0) {
            pwm_data_buffer[3*equal] |= (1<<pin);
            return;
        }
#if DEBUG
        printf("No CB found controlling pin %d (%d), %d is empty; %d above and %d below\n", pin, on_time, empty_pos, above, below);
        printf("above time: %d, below time: %d, on time: %d, calc. delay: %d\n", cbs_buffer[2*above].unused, cbs_buffer[2*below].unused, on_time, cbs_buffer[below*2].unused+pwm_data_buffer[below*3+1]-on_time);
#endif
        cbs_buffer[empty_pos*2+1].next_cb = BUS_DMA_MEM(&cbs[above*2]);
        cbs_buffer[empty_pos*2+1].srce_ad = BUS_DMA_MEM(&pwm_data[above*3 + 1]);
        cbs_buffer[below*2+1].next_cb = BUS_DMA_MEM(&cbs[empty_pos*2]);
        cbs_buffer[below*2+1].srce_ad = BUS_DMA_MEM(&pwm_data[empty_pos*3 + 1]);
        pwm_data_buffer[empty_pos*3+1] = cbs_buffer[below*2].unused + pwm_data_buffer[below*3+1] - on_time;
        pwm_data_buffer[below*3+1] = on_time - cbs_buffer[below*2].unused;
        pwm_data_buffer[empty_pos*3] = (1<<pin);
        cbs_buffer[empty_pos*2].unused = on_time;
        
    } else {
        pwm_data_buffer[0] &= ~(1<<pin);
    }
#if DEBUG
    uint8_t idx = 0;
    do {
        printf("%03d  (%05d)  ", idx, pwm_data_buffer[idx/2*3+1]);
        idx = (cbs_buffer[idx].next_cb - BUS_DMA_MEM(cbs))/sizeof(DMA_CB);
    } while (idx != 0);
    printf("000\n");
#endif
}

void pwm_update() {
    memcpy(cbs, cbs_buffer, 2112);
    memcpy(pwm_data, pwm_data_buffer, 396);
}

void pwm_begin(uint8_t num_ch, int freq) {
    pwm_n_channels = num_ch;

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
        cbs[i].next_cb = BUS_DMA_MEM(cbs);
    }
    cbs[1].next_cb = BUS_DMA_MEM(cbs);
    pwm_period = num_ch*6;
    init_pwm(freq);
    *VIRT_PWM_REG(PWM_DMAC) = PWM_DMAC_ENAB|16;
    start_pwm();
    // This line is very important. Without this printf, the DMA doesn't load correctly.
    // One could also use usleep(100000) instead, even though the delay caused by the
    // printf is much lower. Why this is needed is still not known...
    printf("\n");
    start_dma(virt_dma_mem);
    disp_dma();
    for (int i=0; i < 9; i++) printf("  %08X: %08X\n", BUS_DMA_MEM(&pwm_data[i]), pwm_data[i]);
    for (int i=0; i < 6; i++) {
        printf("%02d: CB_ADDR: %08X, SRCE_AD: %08X, DEST_AD: %08X, NEXT_CB: %08X, OFFSET: %08X\n", i, BUS_DMA_MEM(&cbs[i]), cbs[i].srce_ad, cbs[i].dest_ad, cbs[i].next_cb, cbs[i].unused);
    }
    memcpy(pwm_data_buffer, pwm_data, 396);
    memcpy(cbs_buffer, cbs, 2112);
}


// EOF

