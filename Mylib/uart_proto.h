#ifndef __UART_PROTO_H
#define __UART_PROTO_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include "stdint.h"

#define		PROTO_DATA_SIZE_TX	5 // 4 byte data + 1 byte mode
#define		PROTO_DATA_SIZE_RX	13 // 4 byte data * 3 + 1 byte mode
//#define		PROTO_DATA_SIZE_RX	6
#define		PROTO_BUFF_SIZE_TX	PROTO_DATA_SIZE_TX+4
#define		PROTO_BUFF_SIZE_RX	PROTO_DATA_SIZE_RX+4

#define   PROTO_START_BYTE        0x02 // STX
#define   PROTO_ESC_BYTE          0x7E
#define   PROTO_END_BYTE          0x03 // ETX

typedef struct 
{
	uint8_t au8TxBuffer[PROTO_BUFF_SIZE_TX];
	uint8_t au8RxBuffer[PROTO_BUFF_SIZE_RX];
	_Bool bRxCpltflag;
	_Bool bTxCpltflag;
	int8_t i8ErrorCode;
} uart_proto_handle_t;

/**********************************************************
* Function Name: UART_get_data
* Description: check and get data from UART_RX buffer
* Arguments: uart - the pointer to uart_proto handle struct
*						 dest, dest_led - destination for valid data receive
* Return value: errorcode
**********************************************************/
int8_t UART_get_data(uart_proto_handle_t *uart, uint8_t *pu8Dest, uint16_t *pu16Dest_len);

/**********************************************************
* Function Name: UART_frame_data
* Description: add data into specified frame, (add CRC bytes)
* Arguments: uart - the pointer to uart_proto handle struct
*						 pu8Src, pu16Scr_len - Source data for transmission
* Return value: errorcode
**********************************************************/
void UART_frame_data(uart_proto_handle_t *uart, uint8_t *pu8Src, uint16_t u16Src_len,
	uint8_t *pu8Dest, uint16_t *pu16Dest_len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __UART_PROTO_H */
