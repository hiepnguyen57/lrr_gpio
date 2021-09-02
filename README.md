# Create lora gpio shared library:
- Run command to build:
    ```sh
    cd liblrrgpio
    mkdir build
    cd build
    cmake ..
    make
    ```

- Install into pi system:
    ```sh
    sudo make install
    ```

    Note: Disable debug logging, set `LR_DEBUG=0` in `CMakeLists.txt`

# Using shared library
- First, initialize gpio library with your pi model. For examples, init with pi 3 b+:
    ```
    lrr_gpio_init(LRR_PI_MODEL_B)
    ```
    Function return 0 in case of success. Otherwise is false.

    See `pi_model_t` to find definition of raspberry pi version

- Then, request gpio pin that you want to use:
    ```
    lrr_gpio_request(pin, forced)
    ```
    `forced`: set true if you want to make sure that you should owner this pin if it is using by other.
    Function return 0 in case of success. Otherwise is false.

- Okay, you can do anything with requested pin:
    ```
    lrr_gpio_setInputMode
    lrr_gpio_setOutputMode
    lrr_gpio_pullUpDnControl
    lrr_gpio_digitalRead
    lrr_gpio_digitalWrite
    lrr_gpio_registerISR
    ```

- Finally, remember free gpio after using:
    ```
    lrr_gpio_free(pin)
    ```

Read `loraradio_gpio.h` for more detail.

# Testing:
List of options for application:
```
-g gpio pin
-a another gpio pin
-m mode is interrupt mode (none:0/falling:1/rising:2/both:3)
-s state is default output level (high:1/low:0)
-p pullup:2/pulldown:1/off:0
```

## Input mode
```sh
cd liblrrgpio/test/input
mkdir build
cd build
cmake ..
make
```

Using GPIO5 and GPIO6 as input mode, pud off. Run commands:
```sh
sudo ./lrr_gpio_input -g 5 -a 6 -p 0
```
Try to connect GPIO5 and GPIO6 to 3.3V or GND. Then see logging result.

## Output mode
```sh
cd liblrrgpio/test/output
mkdir build
cd build
cmake ..
make
```

Using GPIO17 and GPIO18 as output mode, state high, pud down. Run commands:
```sh
sudo ./lrr_gpio_output -g 17 -a 18 -s 1 -p 1
```

We used GPIO23, GPIO24 as input mode for capturing output level.
Connecte GPIO17 to GPIO23 and GPIO18 to GPIO24. Then see logging result.

## Interrupt mode
```sh
cd liblrrgpio/test/interrupt
mkdir build
cd build
cmake ..
make
```

Using GPIO20 and GPIO21 as interrupt mode, falling edge, pud up. Run commands:
```sh
sudo ./lrr_gpio_interrupt -g 20 -a 21 -m 1 -p 2
```

We used GPIO23, GPIO24 as output mode for generating pulses
Connect GPIO20 to GPIO23 and GPIO21 to GPIO24. Then see logging result.

HAVE FUN !!!!
