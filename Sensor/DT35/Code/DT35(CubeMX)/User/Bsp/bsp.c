/**
 * @file    bsp.c
 * @author  Deadline039
 * @brief   Bsp layer initialize.
 * @version 1.0
 * @date    2024-09-18
 */

#include <bsp.h>

/**
 * @brief Bsp layer initiallize.
 *
 */
void bsp_init(void) {
    // 采样率测量期间关闭不必要的任务
    // uart_ex_init(&huart1);                 // printf/UART
    // can_list_add_can(can1_selected, 4, 0); // CAN
}

#ifdef USE_FULL_ASSERT

#include <stdio.h>

/**
 * @brief HAL assert failed.
 *
 * @param file File name.
 * @param line Line.
 */
void assert_failed(uint8_t *file, uint32_t line) {
    fprintf(stderr, "HAL assert failed. In file: %s, line: %u. \n", file, line);
}

#endif /* USE_FULL_ASSERT */