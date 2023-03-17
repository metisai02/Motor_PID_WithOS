/*
 * stringCut.c
 *
 *  Created on: Mar 16, 2023
 *      Author: Admin
 */
#include "stringCut.h"
//
//
char data_uart[50] = {0};
char data_rx = 0;
uint8_t uart_count = 0;
uint8_t uart_flag = 0;
float number_k = 0.0f;
float number_div = 0.0f;
float number_add = 0.0f;
float number_real = 0.0f;
char data_recFromPC[50] = {0};
char data_sendtoPC[50] = {0};
char sendDataToSTM[100] = {0};
float string_cut(char *buff_receiveFromPC, char *this_want)
{
  char *pwant = strstr(buff_receiveFromPC, this_want);
  if (pwant != NULL)
  {
    pwant = pwant + strlen(this_want);
    if (*pwant == ':')
    {
      pwant++;
      number_k = 0.0f;
      number_div = 0.0f;
      number_real = 0.0f;
      number_add = 0.0f;
      uint8_t x = 1;
      while (1)
      {
        if (*pwant >= '0' && *pwant <= '9')
        {
          number_k *= 10;
          number_k = number_k + (*pwant - '0'); // chuyen tu ky tu sang so (dua vao bang ma ASCII)
          number_real = number_k;
          pwant++;
        }
        else if (*pwant == '.')
        {
          while (1)
          {
            pwant++;
            if (*pwant == ';')
            {
              break;
            }
            number_div = 1.0f * (((*pwant) - '0') / (1.0f * (10 * x)));
            number_real = number_real + number_div;
            //						number_add *= 10;
            //						number_add = (*pwant - '0') + number_add;
            x = x * 10;
          }
          //					number_div = number_add/(1.0f*(10*x));
          //					number_real = number_k + number_div;
        }
        else if (*pwant == ';')
        {
          break;
        }
      }
      return number_real;
    }
  }
}


