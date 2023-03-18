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
float setpointQt = 0.0f;
float number_k = 0.0f;
float number_div = 0.0f;
float number_add = 0.0f;
float number_real = 0.0f;
char data_recFromPC[50] = {0};
char data_sendtoPC[50] = {0};
char data_recFromPCMode[30] = {0};
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
          number_k = 0.0f;
          while (1)
          {
            pwant++;
            if (*pwant == ';')
            {
              break;
            }
            //           number_div = 1.0f * (((*pwant) - '0') / (1.0f * (10 * x)));
            number_k *= 10;
            number_k = number_k + (*pwant - '0');
            number_div = number_k;
            //       number_real = number_real + number_div;

            //           x = x * 10;
            x += 1;
          }
          //					number_div = number_add/(1.0f*(10*x));
          //					number_real = number_k + number_div;
          number_real = number_real + number_div * pow(10, -(x - 1));
        }
        else if (*pwant == ';')
        {
          break;
        }
      }
      return number_real; // Kp hoac Ki hoac Kd
    }
  }
}

int string_cut_checkMode(char *buff_receiveFromPC)
{
  float flagAmDuong = 1;
  uint8_t checkMode = 0;
  char *pwant = buff_receiveFromPC;
  if (pwant != NULL)
  {
    number_k = 0.0f;
    number_div = 0.0f;
    number_real = 0.0f;
    number_add = 0.0f;
    uint8_t x = 1;
    while (1)
    {
      if (*pwant == '-')
      {
        flagAmDuong = -1;
        pwant++;
      }
      else if (*pwant == 'S')
      {
        pwant++;
      }
      else if (*pwant >= '0' && *pwant <= '9')
      {
        number_k *= 10;
        number_k = number_k + (*pwant - '0'); // chuyen tu ky tu sang so (dua vao bang ma ASCII)
        number_real = number_k;
        pwant++;
      }
      else if (*pwant == '.')
      {
        number_k = 0.0f;
        while (1)
        {
          pwant++;
          if (*pwant == ';')
          {
            break;
          }
          //           number_div = 1.0f * (((*pwant) - '0') / (1.0f * (10 * x)));
          number_k *= 10;
          number_k = number_k + (*pwant - '0');
          number_div = number_k;
          //       number_real = number_real + number_div;

          //           x = x * 10;
          x += 1;
        }
        //					number_div = number_add/(1.0f*(10*x));
        //					number_real = number_k + number_div;
        number_real = number_real + number_div * pow(10, -(x - 1));
      }
      else if (*pwant == ';')
      {
        pwant++;
        checkMode = (uint8_t *)((*pwant) - '0'); // checkMode la 1 thi Position, la 2 thi Velocity
        break;
      }
      else
      {
        break;
      }
    }
  }
  setpointQt = (flagAmDuong)*number_real;
  return checkMode; // Mode velo or posi
}
