#include "peripherals.h"
#include "time.h"

#define ENC_CLK 3
#define ENC_DIR 2


int main() {
    init_memory();

    uint8_t last_value=0;
    int32_t total = 0;
    uint8_t val = 0;

    clock_t start = clock();

    while (1) {
        val = gpio_in(ENC_CLK);
        if (val > last_value) {
            if (gpio_in(ENC_DIR)) total++;
            else total--;
            double avg = (double)total*CLOCKS_PER_SEC/((double)(clock() - start));
            printf("\rValue: %d, average speed: %f, probably %c", total, avg, (avg < 10 ? 'O' : (avg < 40 ? 'L' : (avg < 70 ? 'M' : 'H'))));
            usleep(100);
        } else if (val < last_value) usleep(100);
        last_value = gpio_in(ENC_CLK);
    }

    return 0;
}
