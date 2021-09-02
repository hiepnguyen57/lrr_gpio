/**
 * @file loraradio_gpio.c
 * @author Hiep Nguyen (hoahiepk10@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <stdlib.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <time.h>
#include <pthread.h>
#include <sys/mman.h>
#include <stdbool.h>

#include "lrr_gpio.h"
//------------------------------------------------------------------------------------------------//
#define BUF_STR_SIZE (128)
#define POLL_FOREVER (-1)
#define NUM_GPIO_MAX (64)
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

#define GPIO_PERI_BASE (0x20000000)
#define GPIO_PERI_BASE_2835 (0x3F000000)
#define GPIO_PERI_BASE_2711 (0xFE000000)

#define GPPUD (37)
#define GPPUPPDN0 (57)
//------------------------------------------------------------------------------------------------//
typedef struct {
    bool is_created;
    int sys_fd;
    pthread_t thread_id;
    pthread_mutex_t ready;
    lrr_callback_t cb_fn;
} lrr_gpio_intr_t;
//------------------------------------------------------------------------------------------------//
static lrr_gpio_intr_t lrr_gpio_intr[NUM_GPIO_MAX];
static volatile unsigned int GPIO_BASE_ADDR;
static volatile unsigned int gpio_base = 0 ;
static volatile unsigned int gpio_pup_offset = 0;
static volatile unsigned int *gpio_map;
//------------------------------------------------------------------------------------------------//
static void lrr_gpio_unexport(int pin);
static void *interrupt_handler(void *arg);
static void lrr_gpio_interrupt_cancel(int pin);
static int lrr_gpio_poll(int fd, int timeout_ms);
//------------------------------------------------------------------------------------------------//
static const char gpio_level_str[2][BUF_STR_SIZE] = { "0", "1" };
static const char gpio_edge_str[4][BUF_STR_SIZE] = { "none", "falling", "rising", "both" };
static const uint8_t gpio_to_pud_clk[] = {
    38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
    39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
};
//------------------------------------------------------------------------------------------------//
// PUBLIC FUNCTIONS
//------------------------------------------------------------------------------------------------//
int lrr_gpio_init(pi_model_t model) {
    switch (model) {
    case LRR_PI_MODEL_A:
    case LRR_PI_MODEL_B:
    case LRR_PI_MODEL_AP:
    case LRR_PI_MODEL_BP:
    case LRR_PI_ALPHA:
    case LRR_PI_MODEL_CM:
    case LRR_PI_MODEL_ZERO:
    case LRR_PI_MODEL_ZERO_W:
        gpio_base = GPIO_PERI_BASE;
        gpio_pup_offset = GPPUD;
        break;
    
    case LRR_PI_MODEL_4B:
    case LRR_PI_MODEL_400:
    case LRR_PI_MODEL_CM4:
        gpio_base = GPIO_PERI_BASE_2711;
        gpio_pup_offset = GPPUPPDN0;
        break;

    default:
        gpio_base = GPIO_PERI_BASE_2835;
        gpio_pup_offset = GPPUD;
        break;
    }

    // Try /dev/mem. If fails, then we will
    // Try /dev/gpiomem. If that fails, we will say good game :)
    int fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
    if (fd < 0) {
        fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC);
        if (fd > 0) {
            lrr_dbg("Using gpiomem");
            gpio_base = 0;
        }
        else {
            lrr_err("Can't open /dev/mem and /dev/gpiomem");
            return -1;
        }
    }

    // Access GPIO with physical address
    GPIO_BASE_ADDR = gpio_base + 0x00200000;
    gpio_map = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_ADDR);
    close(fd);

    if (gpio_map == MAP_FAILED) {
        lrr_err("mmap error");
        return -1;
    }

    // Initial local variables
    for (int i = 0; i < NUM_GPIO_MAX; i++) {
        lrr_gpio_intr[i].is_created = false;
        lrr_gpio_intr[i].cb_fn = NULL;
    }

    return 0;
}

int lrr_gpio_request(int pin, bool forced) {
    int fd;
    char path[BUF_STR_SIZE];
    sprintf (path, "/sys/class/gpio/gpio%d/value", pin);

    if (access(path, F_OK) == 0) {
        lrr_dbg("pin already exported");
        if (forced == true) {
            lrr_gpio_unexport(pin);
        }
        else {
            return -1;
        }
    }

    fd = open("/sys/class/gpio/export", O_SYNC | O_WRONLY);
    if (fd < 0) {
        lrr_err("Failed to open export for writing");
        return -1;
    }

    sprintf(path, "%d", pin);
    if (write(fd, path, strlen(path)) < 0) {
        close(fd);
        lrr_err("Failed to write export");
        return -1;
    }

    close(fd);

    return 0;
}

void lrr_gpio_free(int pin) {
    char path[BUF_STR_SIZE];
    sprintf(path, "/sys/class/gpio/gpio%d/value", pin);

    if (access(path, F_OK) == 0) {
        lrr_gpio_unexport(pin);
        lrr_dbg("success");
    }
}

void lrr_gpio_set_input_mode(int pin, lrr_gpio_pud_t pud) {
    int fd;
    char path[BUF_STR_SIZE];

    lrr_gpio_pud_control(pin, pud);

    sprintf(path, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_SYNC | O_WRONLY);
    if (fd < 0) {
        lrr_err("Failed to open direction for writing");
        return;
    }

    if (write(fd, "in", strlen("in")) < 0) {
        lrr_err("Failed to set direction");
    }

    close(fd);
}

void lrr_gpio_pud_control(int pin, lrr_gpio_pud_t pud) {
    if (gpio_pup_offset == GPPUPPDN0) {
        int pullreg = GPPUPPDN0 + (pin >> 4);
        int pullshift = (pin & 0xf) << 1;
        unsigned int pullbits;
        unsigned int pull = 0;

        switch (pud) {
        case LRR_GPIO_PUD_OFF:
            pull = 0;
            break;
        case LRR_GPIO_PUD_UP:
            pull = 1;
            break;
        case LRR_GPIO_PUD_DOWN:
            pull = 2;
            break;
        default:
            return;
        }

        pullbits = *(gpio_map + pullreg);
        pullbits &= ~(3 << pullshift);
        pullbits |= (pull << pullshift);
        *(gpio_map + pullreg) = pullbits;
    }
    else {
        *(gpio_map + GPPUD) = pud & 3;
        usleep(5);
        *(gpio_map + gpio_to_pud_clk[pin]) = 1 << (pin & 31);
        usleep(5);

        *(gpio_map + GPPUD) = 0;
        usleep(5);
        *(gpio_map + gpio_to_pud_clk[pin]) = 0;
        usleep(5);
    }
}

void lrr_gpio_set_output_mode(int pin, lrr_gpio_state_t state) {
    int fd;
    char path[BUF_STR_SIZE];

    sprintf(path, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_SYNC | O_WRONLY);
    if (fd < 0) {
        lrr_err("Failed to open gpio direction for writing");
        return;
    }

    if (write(fd, "out", strlen("out")) < 0) {
        lrr_err("Failed to set direction");
        close(fd);
        return;
    }

    close(fd);

    lrr_dbg("pin %d state: %d", pin, state);
    lrr_gpio_digital_write(pin, state);
}

int lrr_gpio_digital_read(int pin) {
    int fd;
    char path[BUF_STR_SIZE];
	char val_str[3];

	sprintf(path, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_SYNC | O_RDONLY);
	if (fd < 0) {
        lrr_err("Failed to open gpio value for reading");
		return -1;
	}

    lseek(fd, 0, SEEK_SET);

    if (read(fd, val_str, 3) < 0) {
        close(fd);
		lrr_err("Failed to read value");
		return -1;
	}

	close(fd);

	return atoi(val_str);
}

void lrr_gpio_digital_write(int pin, lrr_gpio_state_t output) {
    int fd;
    char path[BUF_STR_SIZE];

	sprintf(path, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_SYNC | O_WRONLY);

    if (fd < 0) {
        lrr_err("Failed to open gpio value for writing");
        return;
    }
    
    lrr_dbg("pin %d output: %s", pin, gpio_level_str[output]);
    if (write(fd, gpio_level_str[output], strlen(gpio_level_str[output])) < 0) {
        lrr_err("Failed to write value");
    }

    close(fd);
}

int lrr_gpio_register_irq(int pin, lrr_gpio_int_t mode, lrr_gpio_pud_t pud, lrr_callback_t callback) {
    int fd;
    char path[BUF_STR_SIZE];
    int count;
    char c;
    lrr_gpio_intr_t *intr = &lrr_gpio_intr[pin];

    // Sanity check
    if (pin < 0 || pin > 63)  {
        lrr_err("pin must be 0-63");
        return -1;
    }

    if (intr->is_created == true) {
        lrr_err("pin is in use");
        return -1;
    }

    lrr_gpio_pud_control(pin, pud);

    // Setup edge interrupt mode
    sprintf(path, "/sys/class/gpio/gpio%d/edge", pin);
    fd = open(path, O_SYNC | O_WRONLY);
    if (fd < 0) {
        lrr_err("Failed to open gpio value for writing");
        return -1;
    }

    lrr_dbg("pin %d mode: %s", pin, gpio_edge_str[mode]);
    if (write(fd, gpio_edge_str[mode], strlen(gpio_edge_str[mode])) < 0) {
        lrr_err("Failed to write value");
        close(fd);
        return -1; 
    }

    close(fd);
    
    // Open a file descriptor for polling
    sprintf(path, "/sys/class/gpio/gpio%d/value", pin) ;
    fd = open(path, O_RDWR);
    if (fd < 0) {
        lrr_err("Failed to open gpio value for polling");
        return -1;
    }
    
    // Save file descriptor and callback into database
    intr->sys_fd = fd;
    intr->cb_fn = callback;
    intr->is_created = true;

    // Clear any initial pending interrupt
    ioctl(fd, FIONREAD, &count);
    for (int i = 0; i < count; ++i) {
        read(fd, &c, 1);
    }
    
    // Create interrupt handler thread
    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_mutex_init(&intr->ready, NULL);
    pthread_mutex_lock(&intr->ready);

    int ret = pthread_create(&intr->thread_id, NULL, interrupt_handler, intr);
    if (ret == 0) {
        // Wait for thread to be initialised and ready
        pthread_mutex_lock(&intr->ready);
    }
    else {
        intr->cb_fn = NULL;
        intr->is_created = false;
        pthread_mutex_unlock(&intr->ready);
        pthread_mutex_destroy(&intr->ready);
        lrr_err("Register failed");
        return -1;
    }

    return 0;
}
//------------------------------------------------------------------------------------------------//
// PRIVATE FUNCTIONS
//------------------------------------------------------------------------------------------------//
static void lrr_gpio_unexport(int pin) {
    int fd;
    char path[BUF_STR_SIZE];

    // Free interrupt mode if in using
    if (lrr_gpio_intr[pin].is_created == true) {
        lrr_gpio_interrupt_cancel(pin);
    }

    fd = open("/sys/class/gpio/unexport", O_SYNC | O_WRONLY);
    if (fd < 0) {
        lrr_err("Failed to open unexport for writing");
    }

    sprintf(path, "%d", pin);
    if (write(fd, path, strlen(path)) < 0) {
        lrr_err("Failed to write unexport");
    }

    close(fd);
}

static void *interrupt_handler(void *arg) {
    lrr_gpio_intr_t *intr = (lrr_gpio_intr_t *)arg;
    pthread_mutex_unlock(&intr->ready);

    while (intr->is_created == true) {
        if (lrr_gpio_poll(intr->sys_fd, POLL_FOREVER) > 0 && intr->cb_fn) {
            intr->cb_fn();
        }
    }

    lrr_dbg("exiting ...");
    return NULL;
}

static void lrr_gpio_interrupt_cancel(int pin) {
    lrr_gpio_intr_t *intr = &lrr_gpio_intr[pin];

    close(intr->sys_fd);
    intr->is_created = false;
    intr->cb_fn = NULL;

    pthread_cancel(intr->thread_id);
    pthread_join(intr->thread_id, NULL);

    pthread_mutex_unlock(&intr->ready);
    pthread_mutex_destroy(&intr->ready);
    lrr_dbg("success");
}

static int lrr_gpio_poll(int fd, int timeout_ms) {
    int rc;
    uint8_t c;
    struct pollfd pfd;

    // Setup poll structure
    pfd.fd = fd;
    pfd.events = POLLPRI | POLLERR;
    
    rc = poll(&pfd, 1, timeout_ms);
    if (rc > 0) {
        lseek(fd, 0, SEEK_SET); // Rewind
        (void)read(fd, &c, 1);  // Read & clear
    }

    return rc;
}
//------------------------------------------------------------------------------------------------//