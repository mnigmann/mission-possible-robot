#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "peripherals.h"
#include "config.h"

#define HTTP_CHUNK 128

int sockfd, connfd, len;
struct sockaddr_in servaddr, cli;
struct timeval tv;

char file_buffer[HTTP_CHUNK];

void close_conn(int c) {
    printf("Closing\n");
    close(sockfd);
    terminate(0);    // Close/cleanup memory for peripherals
    exit(0);
}

uint8_t serve_file(char *headers, char *filename) {
    FILE *html_file = fopen(filename, "r");
    if (html_file == NULL) {
#if DEBUG
        printf("File %s not found\n", filename);
#endif
        return 0;
    }
    fseek(html_file, 0L, SEEK_END);
    int size = ftell(html_file);
#if DEBUG
    printf("Serving %s with length %d\n", filename, size);
#endif
    rewind(html_file);
    write(connfd, headers, strlen(headers));
    sprintf(file_buffer, "Content-Length: %d\r\n\r\n", size);
    write(connfd, file_buffer, strlen(file_buffer));
    while ((size = fread(file_buffer, 1, HTTP_CHUNK, html_file))) write(connfd, file_buffer, size);
    return 1;
}

#ifdef MAIN_IS_SERVER
int main() {
    signal(SIGINT, close_conn);

    init_memory();
    pwm_begin(32, PWM_FREQ);
    gpio_mode(23, 1);
    gpio_mode(17, 1);
    usleep(10000);
    pwm_set_period(4000);
    pwm_set_channel(23, 300);
    pwm_set_channel(17, 300);
    pwm_update();
    main_server(NULL);
}
#endif

int main_server(int* pipes) {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        printf("socket creation failed\n");
        exit(0);
    }
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &(int){1}, sizeof(int)) < 0) {
        printf("setsockopt failed\n");
        exit(0);
    }
    bzero(&servaddr, sizeof(servaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(80);
    if (bind(sockfd, (struct sockaddr *)(&servaddr), sizeof(servaddr))) {
        printf("bind failed\n");
        exit(0);
    }

    while (1) {
        if ((listen(sockfd, 5)) != 0) {
#if DEBUG
            printf("Listen failed\n");
#endif
            exit(0);
        }
#if DEBUG
        else printf("\033[31mServer listening\033[0m\n");
#endif

        connfd = accept(sockfd, (struct sockaddr *)(&cli), &len);
        if (connfd < 0) {
#if DEBUG
            printf("server accept failed\n");
#endif
            exit(0);
        }
#if DEBUG
        else printf("server accepting the client\n");
#endif

        char buff[1024];
        char url[64];
        char method[8];
        int i, j;
        while (1) {
            if (getppid() == 1) {
                terminate(0);
            }
            if (read(connfd, buff, sizeof(buff)) == 0) break;
            for (i = 0; (i < 1024) && (buff[i] != ' '); i++) method[i] = buff[i];
            method[i] = 0;
            for (j = i+1; (j < 1024) && (buff[j] != ' '); j++) {
                url[j - i - 1] = buff[j];
            }
            url[j - i - 1] = 0;
            gettimeofday(&tv, NULL);
#if DEBUG
            printf("Received %s request to %s at %d\n", method, url, tv.tv_usec);
#endif
            if (strcmp(url, "/") == 0) {
#if DEBUG
                printf("Serving main file\n");
#endif
                serve_file("HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Type: text/html; charset=utf-8\r\n", "main.html");
                continue;
            }
            if (strncmp(url, "/cmd", 4) == 0) {
                uint32_t left = 300;
                uint32_t right = 300;
                if (url[4] == '?') {
                    char *token, *rest = url+5;
                    while ((token = strtok_r(rest, "&", &rest))) {
                        if (strncmp(token, "left", 4) == 0) sscanf(token+5, "%d", &left);
                        if (strncmp(token, "right", 5) == 0) sscanf(token+6, "%d", &right);
                    }
                }
#if DEBUG
                printf("left: %d, right: %d\n", left, right);
#endif
                // TODO: find the second to last control block, wait for it, and then update the PWM
                // this must be done because the pwm_data references are one edge ahead of the CBs.
                // otherwise there are glitches that affect the duty cycle and the period
#ifdef MAIN_IS_SERVER
                while (*VIRT_DMA_REG(DMA_NEXTCONBK) != BUS_DMA_MEM(virt_dma_mem));
                pwm_set_channel(MOTOR_LEFT, left);
                pwm_set_channel(MOTOR_RIGHT, right);

                pwm_update();
#if DEBUG
                for (int i=0; i < 9; i++) printf("  %08X: %08X\n", BUS_DMA_MEM(&pwm_data[i]), pwm_data[i]);
                for (int i=0; i < 6; i++) {
                    printf("%02d: CB_ADDR: %08X, SRCE_AD: %08X, DEST_AD: %08X, NEXT_CB: %08X, OFFSET: %08X\n", i, BUS_DMA_MEM(&cbs[i]), cbs[i].srce_ad, cbs[i].dest_ad, cbs[i].next_cb, cbs[i].unused);
                }
#endif
#else
                sprintf(url, "#%04X%04X;", left, right);
                write(pipes[1], url, 10);
#endif
//                strcpy(buff, "HTTP/1.1 200 OK\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n");
//                write(connfd, buff, strlen(buff));
//                continue;
            }
            code404:
                strcpy(buff, "HTTP/1.1 404 Not Found\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n");
                write(connfd, buff, strlen(buff));
        }
    }
    close_conn(0);
}
