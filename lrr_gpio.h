#pragma once

#include <stdlib.h>
//------------------------------------------------------------------------------------------------//
#if (LG_DEBUG)
#define lrr_dbg(fmt, ...) fprintf(stderr, "[%s] " fmt "\n", __func__, ##__VA_ARGS__)
#define lrr_err(fmt, ...) fprintf(stderr, "[%s][ERROR] " fmt "\n", __func__, ##__VA_ARGS__)
#else
#define lrr_dbg(fmt, ...)
#define lrr_err(fmt, ...)
#endif
//------------------------------------------------------------------------------------------------//
typedef void (*lrr_callback_t)(void);

typedef enum {
    LRR_GPIO_MODE_INPUT = 0,
    LRR_GPIO_MODE_OUTPUT = 1
} lrr_gpio_mode_t;

typedef enum {
    LRR_GPIO_STATE_LOW = 0,
    LRR_GPIO_STATE_HIGH = 1,
} lrr_gpio_state_t;

typedef enum {
    LRR_GPIO_PUD_OFF = 0,
    LRR_GPIO_PUD_DOWN = 1,
    LRR_GPIO_PUD_UP = 2,
} lrr_gpio_pud_t;

typedef enum {
    LRR_GPIO_INT_SETUP = 0,
    LRR_GPIO_INT_FALLING = 1,
    LRR_GPIO_INT_RISING = 2,
    LRR_GPIO_INT_BOTH = 3,
} lrr_gpio_int_t;

typedef enum {
    LRR_PI_MODEL_A = 0,
    LRR_PI_MODEL_B,
    LRR_PI_MODEL_AP,
    LRR_PI_MODEL_BP,
    LRR_PI_ALPHA,
    LRR_PI_MODEL_CM,
    LRR_PI_MODEL_ZERO,
    LRR_PI_MODEL_ZERO_W,
    LRR_PI_MODEL_4B,
    LRR_PI_MODEL_400,
    LRR_PI_MODEL_CM4,
} pi_model_t;

//------------------------------------------------------------------------------------------------//
// APIs EXPORTED
//------------------------------------------------------------------------------------------------//
/**
 * @brief Initialize lora gpio module
 * @return @c 0 is success. Otherwise is failed
 */
int lrr_gpio_init(pi_model_t model);

/**
 * @brief Request a gpio to use
 * 
 * @param pin The gpio pin
 * @param forced true if you want to forced owner
 * @return @c 0 is success. Otherwise is failed
 */
int lrr_gpio_request(int pin, bool forced);

/**
 * @brief Free a gpio after using
 * 
 * @param pin The gpio pin
 */
void lrr_gpio_free(int pin);

/**
 * @brief Set a gpio as input mode
 * 
 * @param pin The gpio pin
 * @param pud The pull up/down/none settings
 */
void lrr_gpio_set_input_mode(int pin, lrr_gpio_pud_t pud);

/**
 * @brief Set a gpio as output mode
 * 
 * @param pin The gpio pin
 * @param state 
 */
void lrr_gpio_set_output_mode(int pin, lrr_gpio_state_t state);

/**
 * @brief Configure pullup/down/none of the gpio
 * 
 * @param pin The gpio pin
 * @param pud The pull up/down/none settings
 */
void lrr_gpio_pud_control(int pin, lrr_gpio_pud_t pud);

/**
 * @brief Get the current gpio level
 * 
 * @param pin The gpio pin
 * @return int 
 */
int lrr_gpio_digital_read(int pin);

/**
 * @brief Set the current gpio level to high or low
 * 
 * @param pin The gpio pin
 * @param output enumerated @c lrr_gpio_state_t
 */
void lrr_gpio_digital_write(int pin, lrr_gpio_state_t output);

/**
 * @brief Set the gpio as interrupt mode
 * 
 * @param pin The gpio pin
 * @param mode enumerated @c lrr_gpio_int_t
 * @param pud The pull up/down/none settings
 * @param callback The callback function
 * @return @c 0 is success. Otherwise is failed 
 */
int lrr_gpio_register_irq(int pin, lrr_gpio_int_t mode, lrr_gpio_pud_t pud, lrr_callback_t callback);
//------------------------------------------------------------------------------------------------//