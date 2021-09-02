cmake_minimum_required(VERSION 3.1)

project(lrr_gpio_output VERSION 1.0)

add_executable(lrr_gpio_output lrr_gpio_output.c)
find_library(LRR_GPIO_LIB liblrrgpio.so)
message(STATUS ${LRR_GPIO_LIB})
find_package(Threads)
target_link_libraries(lrr_gpio_output PRIVATE ${LRR_GPIO_LIB} ${CMAKE_THREAD_LIBS_INIT})
