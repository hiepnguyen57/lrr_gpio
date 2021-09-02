cmake_minimum_required(VERSION 3.1)

project(lrr_gpio_interrupt VERSION 1.0)

add_executable(lrr_gpio_interrupt lrr_gpio_interrupt.c)
find_library(LRR_GPIO_LIB liblrrgpio.so)
message(STATUS ${LRR_GPIO_LIB})
find_package(Threads)
target_link_libraries(lrr_gpio_interrupt PRIVATE ${LRR_GPIO_LIB} ${CMAKE_THREAD_LIBS_INIT})
