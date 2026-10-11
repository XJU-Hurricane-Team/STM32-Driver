/**
 * @file    microros_allocators.h
 * @author  xinglu
 * @brief   micro-ROS 内存分配器接口
 * @version 1.0
 * @date    2026-10-11
 * @copyright Copyright (c) 2026
 */

#ifndef MICROROS_ALLOCATORS_H
#define MICROROS_ALLOCATORS_H



void * microros_allocate(size_t size, void * state);

void microros_deallocate(void * pointer, void * state);

void * microros_reallocate(void * pointer, size_t size, void * state);

void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#endif //MICROROS_ALLOCATORS_H