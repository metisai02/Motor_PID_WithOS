/*
 * stringCut.h
 *
 *  Created on: Mar 16, 2023
 *      Author: Admin
 */

#ifndef STRINGCUT_H_
#define STRINGCUT_H_
#include "main.h"
#include "stm32f1xx.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
//extern char data_sendtoPC[50];
//extern char data_recFromPC[50];   // data nay dung de luu du lieu real
extern char data_uart[50]; // ban sao nay tao ra chi de chua du lieu nhan duoc xong quang vao data_recFromPC
//extern char data_recFromPCMode[30];
extern char data_rx;
//extern char sendDataToSTM[100];
extern uint8_t uart_count;
extern uint8_t uart_flag;
extern float number_k;
extern float number_div;
extern float number_add;
extern float number_real;
extern float setpointQt;
////extern float Kp_nha, Ki_nha, Kd_nha;
//int a;
//float string_cut(char *buff_receiveFromPC, char *this_want);
//int string_cut_checkMode(char *buff_receiveFromPC);
#endif /* STRINGCUT_H_ */

