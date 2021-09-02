cmake_minimum_required(VERSION 3.1)

project(lrr_gpio VERSION 1.0)

set(LRR_GPIO_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(lrr_gpio_src ${LRR_GPIO_PATH}/lrr_gpio.c)

add_library(lrrgpio SHARED ${lrr_gpio_src})
target_compile_definitions(lrrgpio PUBLIC obj)

find_package(Threads)
target_compile_definitions(lrrgpio PRIVATE
    LR_DEBUG=0
)

install(TARGETS lrrgpio DESTINATION /usr/lib)
install(FILES lrr_gpio.h DESTINATION include)