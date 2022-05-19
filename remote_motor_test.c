#include <ncurses.h>
#include "peripherals.h"

#define MOTOR_LEFT      17
#define MOTOR_RIGHT     23

#define SERVO_AZIMUTH   24
#define SERVO_ELEVATION 18

#define MOTOR_SHOVEL    15  // RXD
#define MOTOR_ARM       14  // TXD
#define MOTOR_FLAG      21  

#define MOTOR_ARM_ONTIME    1500000
#define MOTOR_SHOVEL_ONTIME 400000
#define MOTOR_SHOVEL_SPEED  50

uint16_t pwm_value;
char temp[32];
uint16_t c;

void getSunAngle(double *az, double *el) {
    int r = 3;
    pwm_set_channel(SERVO_ELEVATION, 480);
    while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    uint32_t bestAngle = 0;
    uint32_t bestValue = 100000;
    for (int i=120; i <= 480; i += 10) {
        pwm_set_channel(SERVO_AZIMUTH, i);
        while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
        pwm_update();
        usleep(100000);
        double m_time = measure_precise(0, 4000, 0);
        sprintf(temp, "%03d: %f", i, m_time);
        if (m_time < bestValue) {
            bestAngle = i;
            bestValue = m_time;
        }
        mvaddstr(r, 2, temp);
        refresh();
        usleep(150000);
        r++;
    }
    *az = (bestAngle - 120) / 2;
    pwm_set_channel(SERVO_AZIMUTH, bestAngle);
    while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    usleep(500000);
    bestValue = 100000;
    r = 3;
    for (int i=480; i >= 300; i-=10) {
        pwm_set_channel(SERVO_ELEVATION, i);
        while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
        pwm_update();
        usleep(100000);
        double m_time = measure_precise(0, 4000, 0);
        if (m_time < bestValue) {
            bestAngle = i;
            bestValue = m_time;
        }
        sprintf(temp, "%03d: %f", i, m_time);
        mvaddstr(r, 30, temp);
        refresh();
        usleep(150000);
        r++;
    }
    sprintf(temp, "Best angle found: %d", bestAngle);
    mvaddstr(r, 30, temp);
    refresh();
    *el = (480 - bestAngle) / 2;
    pwm_set_channel(SERVO_ELEVATION, bestAngle);
    while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
}

int main() {
    WINDOW *win;

    init_memory();
    pwm_begin(32, 100000);
    
    win = initscr();

    nodelay(win, 0);
    noecho();
    cbreak();
    keypad(win, true);

    gpio_mode(MOTOR_LEFT, 1);
    gpio_mode(MOTOR_RIGHT, 1);
    gpio_mode(SERVO_AZIMUTH, 1);
    gpio_mode(SERVO_ELEVATION, 1);
    gpio_mode(MOTOR_ARM, 1);
    gpio_mode(MOTOR_SHOVEL, 1);

    pwm_set_period(4000);
    pwm_set_channel(MOTOR_LEFT, 300);
    pwm_set_channel(MOTOR_RIGHT, 300);
    pwm_set_channel(SERVO_AZIMUTH, 120);
    pwm_set_channel(SERVO_ELEVATION, 480);
    pwm_set_channel(MOTOR_ARM, 300);
    pwm_set_channel(MOTOR_SHOVEL, 300);
    pwm_update();    
    pwm_value = 300;

    while (1) {
        sprintf(temp, "Motor speed: %-3d", pwm_value);
        mvaddstr(1, 2, temp);
        c = getch();
        if (c == ']') {
            pwm_value = (pwm_value == 500 ? 500 : pwm_value+1);
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_set_channel(MOTOR_LEFT, pwm_value);
            pwm_set_channel(MOTOR_RIGHT, pwm_value);
        } else if (c == '[') {
            pwm_value = (pwm_value == 100 ? 100 : pwm_value-1);
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_set_channel(MOTOR_LEFT, pwm_value);
            pwm_set_channel(MOTOR_RIGHT, pwm_value);
        } else if (c == KEY_UP) {
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_set_channel(MOTOR_LEFT, pwm_value);
            pwm_set_channel(MOTOR_RIGHT, pwm_value);
        } else if (c == KEY_DOWN) {
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_set_channel(MOTOR_LEFT, 600-pwm_value);
            pwm_set_channel(MOTOR_RIGHT, 600-pwm_value);
        } else if (c == KEY_RIGHT) {
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
//            pwm_set_channel(MOTOR_LEFT, pwm_value);
            pwm_set_channel(MOTOR_RIGHT, 600-pwm_value);
        } else if (c == KEY_LEFT) {
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_set_channel(MOTOR_LEFT, 600-pwm_value);
//            pwm_set_channel(MOTOR_RIGHT, pwm_value);
        } else if (c == ' ') {
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_set_channel(MOTOR_LEFT, 300);
            pwm_set_channel(MOTOR_RIGHT, 300);
        } else if (c == 's') {
            mvaddstr(2, 2, "testing servo");
            refresh();
            double elevation, azimuth;
            getSunAngle(&azimuth, &elevation);
            sprintf(temp, "found az %f, el %f", azimuth, elevation);
            mvaddstr(2, 2, temp);
            refresh();
        } else if (c == 'a') {
            pwm_set_channel(MOTOR_ARM, 370);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
            usleep(MOTOR_ARM_ONTIME);
            pwm_set_channel(MOTOR_ARM, 300);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
        } else if (c == 'A') {
            pwm_set_channel(MOTOR_ARM, 230);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
            usleep(MOTOR_ARM_ONTIME);
            pwm_set_channel(MOTOR_ARM, 300);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
        } else if (c == 'g') {
            printf("Running shovel");
            pwm_set_channel(MOTOR_SHOVEL, 300+MOTOR_SHOVEL_SPEED);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
            usleep(MOTOR_SHOVEL_ONTIME);
            pwm_set_channel(MOTOR_SHOVEL, 300);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
        } else if (c == 'G') {
            pwm_set_channel(MOTOR_SHOVEL, 300-MOTOR_SHOVEL_SPEED);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
            usleep(MOTOR_SHOVEL_ONTIME);
            pwm_set_channel(MOTOR_SHOVEL, 300);
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
        } else if (c == 'q') break;
        if (c) {
            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_update();
        }
    }
    
    delwin(win);
    endwin();
    refresh();
    terminate(0);

    return 0;
}
