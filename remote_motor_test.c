#include <ncurses.h>
#include <time.h>
#include <fcntl.h>
#include "peripherals.h"
#include "config.h"
#include "remote_motor_test.h"

char temp[100];
char inbuf[10];
int32_t totals[16];
uint16_t c;

void getSunAngle(double *az, double *el) {
    int r = 3;
    lower_shovel();
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
        LOG("%03d: %f\n", i, m_time); LOG_FLUSH;
        if (m_time < bestValue) {
            bestAngle = i;
            bestValue = m_time;
        }
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
        LOG("%03d: %f\n", i, m_time); LOG_FLUSH;
        usleep(150000);
        r++;
    }
    LOG("Best angle found: %d\n", bestAngle); LOG_FLUSH;
    *el = (480 - bestAngle) / 2;
    pwm_set_channel(SERVO_ELEVATION, bestAngle);
    pwm_set_channel(SERVO_AZIMUTH, 300);
    while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    raise_shovel();
}

void deploy_arm() {
    mvaddstr(3, UI_ST, "deploying ");
    refresh();
    pwm_set_channel(MOTOR_ARM, 370);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    usleep(MOTOR_ARM_ONTIME);
    pwm_set_channel(MOTOR_ARM, 300);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    mvaddstr(3, UI_ST, "deployed  ");
    refresh();
}

void retract_arm() {
    mvaddstr(3, UI_ST, "retracting");
    refresh();
    pwm_set_channel(MOTOR_ARM, 230);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    usleep(MOTOR_ARM_ONTIME);
    pwm_set_channel(MOTOR_ARM, 300);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    mvaddstr(3, UI_ST, "retracted ");
    refresh();
}

void lower_shovel() {
    mvaddstr(5, UI_ST, "lowering");
    refresh();
    pwm_set_channel(MOTOR_SHOVEL, 300+MOTOR_SHOVEL_SPEED);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    usleep(MOTOR_SHOVEL_ONTIME);
    pwm_set_channel(MOTOR_SHOVEL, 300);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    mvaddstr(5, UI_ST, "lowered ");
    refresh();
}

void release_flag() {
    pwm_set_channel(MOTOR_FLAG, 300+MOTOR_FLAG_SPEED);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    usleep(MOTOR_FLAG_ONTIME);
    pwm_set_channel(MOTOR_FLAG, 300);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
}

void raise_shovel() {
    mvaddstr(5, UI_ST, "raising ");
    refresh();
    pwm_set_channel(MOTOR_SHOVEL, 300-MOTOR_SHOVEL_SPEED);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    usleep(MOTOR_SHOVEL_ONTIME);
    pwm_set_channel(MOTOR_SHOVEL, 300);
    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
    pwm_update();
    mvaddstr(5, UI_ST, "raised  ");
    refresh();
}

void get_temperature() {
    mvaddstr(4, UI_ST, "reading          ");
    refresh();
    FILE *tptr = fopen("/sys/bus/w1/devices/28-000006d75ce4/w1_slave", "r");
    int len = fread(temp, 1, 75, tptr);
    temp[75] = 0;
    LOG("Received temperature (%d): %s\n", len, temp+69);
    if (len == 75) {
        mvaddstr(4, UI_VA, temp + 69);
        refresh();
    } else {
        mvaddstr(4, UI_ST, "error        ");
        mvaddstr(4, UI_VA, "      ");
        refresh();
    }
    fclose(tptr);
}

#ifdef MAIN_IS_REMOTE
int main() {
    logptr = fopen("/home/pi/robot_log.txt", "a");
    LOG("------------ STARTING ROBOT ------------\n"); LOG_FLUSH;

    init_memory();
    pwm_begin(32, 100000);
    
    pwm_set_period(4000);

    return main_remote(NULL);
}
#endif

int main_remote(int *pipes) {
    gpio_mode(SERVO_AZIMUTH, 1);
    gpio_mode(SERVO_ELEVATION, 1);
    gpio_mode(MOTOR_ARM, 1);
    gpio_mode(MOTOR_SHOVEL, 1);
    gpio_mode(MOTOR_FLAG, 1);
    gpio_mode(ENC_CLK, 0);
    gpio_mode(ENC_DIR, 0);
    
    pwm_set_channel(SERVO_AZIMUTH, 300);
    pwm_set_channel(SERVO_ELEVATION, 480);
    pwm_set_channel(MOTOR_ARM, 300);
    pwm_set_channel(MOTOR_SHOVEL, 300);
    pwm_set_channel(MOTOR_FLAG, 300);
    pwm_update();    
    

#ifndef MAIN_IS_REMOTE
    fcntl(pipes[0], F_SETFL, O_NONBLOCK);
#endif

    WINDOW *win;

    win = initscr();

    nodelay(win, 1);
    noecho();
    cbreak();
    keypad(win, true);

    attron(A_BOLD);
    mvaddstr(2, UI_SU, "Subsystem");
    mvaddstr(2, UI_ST, "State");
    mvaddstr(2, UI_VA, "Last value");
    attroff(A_BOLD);
    mvaddstr(3, UI_SU, "Arm");
    mvaddstr(3, UI_ST, "retracted");
    mvaddstr(4, UI_SU + 4, "Temperature sensor");
    mvaddstr(4, UI_ST, "not connected");
    mvaddstr(5, UI_SU, "Shovel");
    mvaddstr(5, UI_ST, "raised");
    mvaddstr(6, UI_SU + 4, "Salinity sensor");
    mvaddstr(7, UI_SU, "Solar sensor");
    mvaddstr(7, UI_ST, "idle");
    mvaddstr(8, UI_SU, "Anemometer");
    mvaddstr(8, UI_ST, "not sampling");
    mvaddstr(9, UI_SU, "Drive motors");
#ifdef MAIN_IS_REMOTE
    mvaddstr(9, UI_ST, "not supported");
#else
    mvaddstr(9, UI_ST, "connected");
#endif
    refresh();

    raise_shovel();

    while (1) {
        c = getch();
        if (c == ' ') {
//            while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
            pwm_set_channel(MOTOR_LEFT, 300);
            pwm_set_channel(MOTOR_RIGHT, 300);
        } else if (c == 's') {
            mvaddstr(7, UI_ST, "searching");
            refresh();
            double elevation = 0, azimuth = 0;
            getSunAngle(&azimuth, &elevation);
            sprintf(temp, "azimuth %f, elevation %f      ", azimuth, elevation);
            LOG("Sun position: az %f, el %f\n", azimuth, elevation); LOG_FLUSH;
            mvaddstr(7, UI_VA, temp);
            mvaddstr(7, UI_ST, "idle     ");
            refresh();
        } else if (c == 'a') {
            deploy_arm();
        } else if (c == 'A') {
            retract_arm();
        } else if (c == 'g') {
            lower_shovel();
        } else if (c == 'G') {
            raise_shovel();
        } else if (c == 'f') {
            release_flag();
        } else if (c == 't') {
            get_temperature();
        } else if (c == 'v') {
            mvaddstr(8, UI_ST, "sampling    ");
            refresh();
            clock_t start = clock();
            uint32_t total = 0, val, last_value = 0;
            double time_diff = 0;
            uint8_t last_idx = 0;
            uint8_t idx = 0;
            double avg = 0;
            uint8_t n_int = 0;
            memset(totals, 0, sizeof(int32_t)*16);

            while (1) {
                val = gpio_in(ENC_CLK);
                time_diff = ((double)(clock() - start))/CLOCKS_PER_SEC;
                idx = (uint8_t)(time_diff / ENC_INTSIZE) % ENC_NUMINT;
                if (idx != last_idx) {
                    if (n_int < ENC_NUMINT) n_int ++;
                    avg = 0;
                    for (uint8_t i=0; i < n_int; i++) {
                        avg += totals[i];
                    }
                    avg /= (n_int - 1);
                    sprintf(temp, "average speed: %f, probably %c              ", avg, (avg < ENC_OFFMAX ? 'O' : (avg < ENC_LOWMAX ? 'L' : (avg < ENC_MEDMAX ? 'M' : 'H'))));
                    mvaddstr(8, UI_VA, temp);
                    refresh();
                    last_idx = idx;
                    totals[idx] = 0;
                }
                if (val > last_value) {
                    if (gpio_in(ENC_DIR)) total++;
                    else total--;
                    totals[idx] ++; 
                    usleep(100);
                } else if (val < last_value) usleep(100);
                last_value = gpio_in(ENC_CLK);
                if (getch() == 'V') {
                    mvaddstr(8, UI_ST, "not sampling");
                    refresh();
                    LOG("Anemometer sampling interval was %f, average speed: %f, probably %c\n", ((double)(clock() - start))/CLOCKS_PER_SEC, avg, (avg < 10 ? 'O' : (avg < 40 ? 'L' : (avg < 70 ? 'M' : 'H'))));
                    break;
                }
            }
        } else if (c == 'S') {
            double sal = measure_precise(0, 10000, 0b100);
            sprintf(temp, "Measured value: %f", sal);
            mvaddstr(6, UI_VA, temp);
            refresh();
        } else if (c == 'q') break;
        //if (c) {
        //    while (*VIRT_DMA_REG(DMA_CONBLK_AD) != BUS_DMA_MEM(virt_dma_mem));
        //    pwm_update();
        //}
#ifndef MAIN_IS_REMOTE
        if (read(pipes[0], inbuf, 10) > 0) {
            int left = 300, right = 300;
            right = strtol(inbuf + 5, NULL, 16);
            inbuf[5] = 0;
            left = strtol(inbuf + 1, NULL, 16);
            pwm_set_channel(MOTOR_LEFT, left);
            pwm_set_channel(MOTOR_RIGHT, right);
            pwm_update();
        }
#endif
    }
    
    delwin(win);
    endwin();
    refresh();
    terminate(0);
    fclose(logptr);

    return 0;
}
