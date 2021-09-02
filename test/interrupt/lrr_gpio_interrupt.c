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

void exit_handler(int dummy) {
    printf("Terminate program\n");
    keep_running = 0;
}

void loraradio_pin_callback(void) {
    static int count = 0;
    printf("pin irq attached !!! counting: %d\n", count++);
}

void loraradio_another_pin_callback(void) {
    static int count = 0;
    printf("another pin irq attached !!! counting: %d\n", count++);
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

    printf("============= INTERRUPT MODE =============\n");
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

    // Register gpio as interrupt mode
    lrr_gpio_register_irq(pin, mode, pud, loraradio_pin_callback);
    lrr_gpio_register_irq(another_pin, mode, pud, loraradio_another_pin_callback);

    // We set GPIO23, GPIO24 as output mode for generating pulses low and high
    // Connect pin to GPIO23 and another pin to GPIO24
    lrr_gpio_request(23, true);
    lrr_gpio_request(24, true);
    lrr_gpio_set_output_mode(23, state_level);
    lrr_gpio_set_output_mode(24, state_level);
    lrr_gpio_state_t output_level = state_level;

    while (keep_running) {
        lrr_gpio_digital_write(23, output_level);
        lrr_gpio_digital_write(24, output_level);

        // Change output level
        if (output_level == LRR_GPIO_STATE_LOW) {
            output_level = LRR_GPIO_STATE_HIGH;
        }
        else {
            output_level = LRR_GPIO_STATE_LOW;
        }

        sleep(2);
    }

    // Release gpio after using
    lrr_gpio_free(pin);
    lrr_gpio_free(another_pin);
    lrr_gpio_free(23);
    lrr_gpio_free(24);

    return 0;
}