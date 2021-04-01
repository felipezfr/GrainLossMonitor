/*
 * nextion.c
 *
 *  Created on: 31 de mar de 2021
 *      Author: ipizf
 */

#include "main.h"
#include "nextion.h"
#include <stdio.h>

UART_HandleTypeDef huart1;
uint8_t Cmd_End[3] = {0xFF, 0xFF, 0xFF};

void SendToProgressBar(char *obj, uint16_t value)
{
  char buf[30];
  int len = sprintf(buf, "%s=%u", obj, value);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
  HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

void SendToGauge(char *obj, uint16_t value)
{
  char buf[30];

  value += 315;
  if (value >= 360)
    value = value - 360;

  int len = sprintf(buf, "%s=%u", obj, value);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
  HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}
void SendToWave(char *obj, uint16_t value)
{
  char buf[30];
  int len = sprintf(buf, "%s,%u", obj, value);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
  HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}
