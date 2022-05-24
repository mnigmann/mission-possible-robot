#include "peripherals.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

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

char inbuf[10];

int main() {
    signal(SIGINT, terminate);
    logptr = fopen("/home/pi/robot_log.txt", "a");
    LOG("------------ STARTING ROBOT ------------\n"); LOG_FLUSH;

    init_memory();
    pwm_begin(32, 100000);

    gpio_mode(MOTOR_LEFT, 1);
    gpio_mode(MOTOR_RIGHT, 1);

    pwm_set_period(4000);
    pwm_set_channel(MOTOR_LEFT, 300);
    pwm_set_channel(MOTOR_RIGHT, 300);
    pwm_update();

    int pipes[2];
    if (pipe(pipes) < 0) {
        exit(1);
    }

    int pid = fork();
    if (pid < 0) {
        printf("Error forking process");
        exit(0);
    }
    if (pid == 0) {
        main_server(pipes);
        return 0;
    } else {
        /*int nbytes;
        while ((nbytes = read(pipes[0], inbuf, 10)) > 0) {
            int left = 300, right = 300;
            right = strtol(inbuf + 5, NULL, 16);
            inbuf[5] = 0;
            left = strtol(inbuf + 1, NULL, 16);
            pwm_set_channel(MOTOR_LEFT, left);
            pwm_set_channel(MOTOR_RIGHT, right);
            pwm_update();
        }
        terminate(0);*/
        main_remote(pipes);
    }


    return 0;
}
