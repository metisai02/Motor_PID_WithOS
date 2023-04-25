#include "uart_proto.h"
#include "crc16.h"

int8_t UART_get_data(uint8_t *pu8Src, uint16_t u16Src_len, uint8_t *pu8Dest, uint16_t *pu16Dest_len)
{   
	const uint8_t *pu8Src_start = pu8Src;
	const uint8_t *pu8Src_findStart = pu8Src + (u16Src_len);
	char checkFindStart = 0;
	uint8_t checkESC = 0;
	uint8_t index = 0;
	uint16_t crc_check = 0;
	// Advance src to start byte
	if(*pu8Src == PROTO_START_BYTE)   // neu vua vo ma nhan duoc byte start luon thi khoi vo ham while
	{
	    checkFindStart = 1;
	}
	while(pu8Src < pu8Src_findStart && *pu8Src != PROTO_START_BYTE)
	{
		pu8Src++;
	}
	if(*pu8Src == PROTO_START_BYTE)   // neu vua vo ma nhan duoc byte start luon thi khoi vo ham while
	{
		checkFindStart = 1;
	}
	// Set our error return val for dest_len
	if(checkFindStart == 0)
	{
	   return Phuc_no_valid;
	}
	// Loop through the data
	pu8Src++;
	while(index < PROTO_DATA_SIZE_RX)
	{
		if (*pu8Src == PROTO_ESC_BYTE) // 7E
		{
			crc_check = crc16_floating(*pu8Src, crc_check);
			*(pu8Dest++)  = (*(++pu8Src)) ^ 0x20;
			crc_check = crc16_floating(*pu8Src, crc_check);
			checkESC++;
		}
		else
		{
			crc_check = crc16_floating(*pu8Src, crc_check);
			*(pu8Dest++) = *pu8Src;
		}
		pu8Src++;
		index++;
	}

	if (*(pu8Src + 2) != PROTO_END_BYTE)
		{  // luc nay pu8Src dang o byte CRC dau tien
			return Phuc_no_valid;
		}
		uint8_t byte2_crc = (crc_check) & 0xFF;
		uint8_t byte1_crc = (crc_check >> 8) & 0xFF;
		if(*(pu8Src++) != byte1_crc || *(pu8Src) != byte2_crc)
		{
			return Phuc_false_CRC;
		}
		*pu16Dest_len =(++pu8Src - pu8Src_start - 4 - checkESC); // do dai data thu duoc
		return Phuc_right;
}
void UART_frame_data(uint8_t *pu8Src, uint8_t u8Src_len, uint8_t *pu8Dest, uint16_t *pu16Dest_len)
{
	uint8_t index = 0;
	uint8_t checkESC = 0;
	uint16_t crc = 0;
	*(pu8Dest++) = PROTO_START_BYTE;

	while(index < u8Src_len) {
			if (*pu8Src == PROTO_START_BYTE || *pu8Src == PROTO_ESC_BYTE || *pu8Src == PROTO_END_BYTE) {
					*(pu8Dest++) = PROTO_ESC_BYTE;
					crc = crc16_floating(*(pu8Dest-1), crc);
					*(pu8Dest++) = (*pu8Src) ^ 0x20;
					crc = crc16_floating(*(pu8Dest-1), crc);
					checkESC++;
			}
			else {
					crc = crc16_floating(*pu8Src, crc);
					*(pu8Dest++) = *pu8Src;
			}
			++pu8Src;
			index++;
	}

	// Set the CRC

	//Casting the CRC to lets the word be assigned to a non-word boundary in memory
	*(pu8Dest) = (char)(crc >>8);
	pu8Dest++;
	*(pu8Dest) = (char)crc;
	pu8Dest++;
	*(pu8Dest++) = PROTO_END_BYTE;
//	*(pu16Dest_len) = pu8Dest - pu8Dest_start;
	*(pu16Dest_len) = u8Src_len + checkESC + 4;   // do dai frame, 4 la start crc crc stop
}

