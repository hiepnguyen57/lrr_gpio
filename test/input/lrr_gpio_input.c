#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <lrr_gpio.h>

static volatile int keep_running = 1;

void exit_handler(int signum) {
    printf("Terminate program\n");
    keep_running = 0;
}

int main(int argc, char *argv[]) {
    char options[] = "g:a:m:s:p:";
    int opt;
    int ret;
    unsigned int pin = 0;
    unsigned int another_pin= 0;
    lrr_gpio_pud_t pud = LRR_GPIO_PUD_OFF;
    lrr_gpio_int_t mode = LRR_GPIO_INT_SETUP;
    lrr_gpio_state_t state_level = LRR_GPIO_STATE_LOW;

    while ((opt = getopt(argc, argv, options)) != -1) {
        switch (opt) {
        case 'g':
            pin = strtol(optarg, NULL, 0);
            break;
        case 'a':
            another_pin = strtol(optarg, NULL, 0);
            break;
        case 'm':
            mode = (lrr_gpio_mode_t)strtol(optarg, NULL, 0);
            break;
        case 's':
            state_level = (lrr_gpio_state_t)strtol(optarg, NULL, 0);
            break;
        case 'p':
            pud = (lrr_gpio_pud_t)strtol(optarg, NULL, 0);
            break;
        }
    }

    // Register signal and signal handler
    signal(SIGINT, exit_handler);

    printf("============= DUMP INFO =============\n");
    printf("gpio pin: %d, another pin: %d\n", pin, another_pin);
    printf("pud: %d, mode: %d, state: %d\n", pud, mode, state_level);

    if (lrr_gpio_init(LRR_PI_MODEL_BP) != 0) {
        printf("lrr_gpio_init failed");
        return -1;
    }

    printf("============= INPUT MODE =============\n");
    ret = lrr_gpio_request(pin, false);
    if (ret != 0) {
        printf("lrr_gpio_request failed");
        return -1;
    }

    ret = lrr_gpio_request(another_pin, false);
    if (ret != 0) {
        printf("lrr_gpio_request failed");
        return -1;
    }
    
    // Set gpio as input mode
    lrr_gpio_set_input_mode(pin, pud);
    lrr_gpio_set_input_mode(another_pin, pud);

    // Connect your gpio pin to GND or 3.3V
    // Then see logging status
    int read_val;
    while (keep_running) {
        read_val = lrr_gpio_digital_read(pin);
        printf("pin %d level: %d\n", pin, read_val);
        read_val = lrr_gpio_digital_read(another_pin);
        printf("another pin %d level: %d\n", another_pin, read_val);
        sleep(2);
    }

    // Release gpio after using
    lrr_gpio_free(pin);
    lrr_gpio_free(another_pin);

    return 0;
}