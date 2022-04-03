#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "peripherals.h"

#define SPI_DIV         976

#define MOD_A 8.1491E-7
#define MOD_B 0.047312
#define MOD_C 30.9837

#define CONF_OUTPUT 24
#define CONF_INPUT_CH0 25


/**
 * Measure the resistance of a sensor (such as a potentiometer or a
 * thermistor) by charging a capacitor through the unknown resistance
 * and measuring the amount of time it takes for the digital pin to read
 * the voltage on the capacitor as "high"
 */
double measure(int ch, uint32_t sampling_interval) {
    gpio_mode(11, 4);
    gpio_mode(8, 4);
    gpio_mode(9, 4);
    *VIRT_SPI_REG(SPI_CS) = (1<<11) | (3<<4) | (1<<21);
    uint32_t cs = *VIRT_SPI_REG(SPI_CS);
    *VIRT_SPI_REG(SPI_CLK) = (uint32_t)(CLOCK_KHZ / 512.0 * sampling_interval / 1000);
    usleep(100);
    *VIRT_SPI_REG(SPI_DLEN) = 4;
    *VIRT_SPI_REG(SPI_CS) |= (1<<7);
    for (uint8_t i=0; i<64; i++) *VIRT_SPI_REG(SPI_FIFO) = 0;
    cs = *VIRT_SPI_REG(SPI_CS) & (1<<16);
    while ((*VIRT_SPI_REG(SPI_CS) & (1<<16)) == 0);
    uint32_t cs2 = *VIRT_SPI_REG(SPI_CS) & (1<<16);
    *VIRT_SPI_REG(SPI_CS) &= ~(1<<7);
    uint8_t val;
    uint32_t time_diff = 0;
    while (*VIRT_SPI_REG(SPI_CS) & (1<<17)) {
        val = *VIRT_SPI_REG(SPI_FIFO);
        val = ~val;
        for (uint8_t i=0; i<8; i++) {
            time_diff += (val & 1);
            val = val >> 1;
        }
    }
    gpio_out(9, 0);
    gpio_mode(9, 1);
    usleep(1000);
    gpio_mode(9, 4);
    return time_diff / 512.0 * sampling_interval;
}

/**
 * Make a more precise resistance measurement by measuring with the given
 * sampling interval, draining the capacitor, and measuring again, this
 * time with (first measurement)*1.5 as the sampling interval. This gives
 * a more precise measurement, as only 512 samples are taken during a the
 * smapling interval, and a shorter interval means they are closer
 * together.
 */
double measure_precise(int ch, uint32_t sampling_interval) {
    double m1 = measure(ch, sampling_interval);
    return measure(ch, m1*1.5);
}

int main() {
    virt_gpio_regs = map_segment((void*)GPIO_BASE, PAGE_SIZE);
    virt_spi_regs = map_segment((void*)SPI_BASE, PAGE_SIZE);


    double m_time = measure_precise(0, 4000);
    double res = (sqrt(MOD_B*MOD_B - 4*MOD_A*(MOD_C - m_time)) - MOD_B)/(2*MOD_A);
    printf("Time is %fus, estimated resistance is %f\n", m_time, res);

    //gpio_mode(CONF_OUTPUT, 1);
    //gpio_mode(CONF_INPUT_CH0, 0);
    //double m = measure(CONF_INPUT_CH0);
    //printf("Measured %f, estimated resistance %f\n", m, m*0.01926421909); //m*0.0326237493579244);
    unmap_segment(virt_gpio_regs, PAGE_SIZE);
    unmap_segment(virt_spi_regs, PAGE_SIZE);

    return 0;
}
