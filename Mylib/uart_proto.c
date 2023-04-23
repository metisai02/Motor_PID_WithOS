#include <uart_proto.h>
#include <crc16.h>

int8_t UART_get_data(uart_proto_handle_t *uart, uint8_t *pu8Dest, uint16_t *pu16Dest_len)
{   
	uint8_t *pu8Src = uart->au8RxBuffer;   // RxBuffer  = TT 0.5 0.6 1 CR RC ST
	const uint8_t *pu8Src_start = pu8Src;  // neu gap bien DA : Kp 
	const uint8_t *pu8Src_end = pu8Src + PROTO_BUFF_SIZE_RX - 1;
	const uint8_t *pu8Dest_start = pu8Dest;
	uint16_t crc_check = 0;

	// Advance src to start byte
	while(pu8Src < pu8Src_end && *pu8Src != PROTO_START_BYTE) ++pu8Src;
	// Set our error return val for dest_len
	*pu16Dest_len = pu8Src - pu8Src_start;
	if(pu8Src >= pu8Src_end)
			return -1;

	// Loop through the data
	++pu8Src;
	int prev_escape = 0;
	char orig_char;
	while(pu8Src < pu8Src_end - 2) {
			if(prev_escape) {
					prev_escape = 0;
					orig_char = (*pu8Src) ^ 0x20;
					crc_check = crc16_floating(orig_char, crc_check);
					*(pu8Dest++) = orig_char;
			}
			else if (*pu8Src == PROTO_ESC_BYTE) {
					prev_escape = 1;
			}
			else {
					orig_char = *pu8Src;
					crc_check = crc16_floating(orig_char, crc_check);
					*(pu8Dest++) = orig_char;
			}
			++pu8Src;
	}

	// Check that we actually hit the end
	if(pu8Src[2] != PROTO_END_BYTE)
			return -1;

	// Check CRC
	uint16_t crc = (uint16_t)(*pu8Src)<<8 | *(pu8Src+1);
	pu8Src += 3;
	if(crc != crc_check) {
			*pu16Dest_len = pu8Src - pu8Src_start;
			return -2;
	}

	*pu16Dest_len = pu8Dest - pu8Dest_start;
	return (pu8Src - pu8Src_start) - 1;
}
 // TT (DA TA) CR CR ST
 // 
void UART_frame_data(uart_proto_handle_t *uart, uint8_t *pu8Src, uint16_t u8Src_len, uint8_t *pu8Dest, uint16_t *pu16Dest_len)
{
//	uint8_t *pu8Dest = uart->au8TxBuffer;
	const uint8_t *pu8Src_end = pu8Src + u8Src_len;
	const uint8_t *pu8Dest_start = pu8Dest;
	uint16_t crc = 0;

	*(pu8Dest++) = PROTO_START_BYTE;

	int prev_escape = 0;
	while(pu8Src < pu8Src_end) {
			if(prev_escape) {
					prev_escape = 0;
					crc = crc16_floating(*pu8Src, crc);
					*(pu8Dest++) = *(pu8Src) ^ 0x20;
			}
			else if (*pu8Src == PROTO_START_BYTE || *pu8Src == PROTO_ESC_BYTE || *pu8Src == PROTO_END_BYTE) {
					prev_escape = 1;
					*(pu8Dest++) = PROTO_ESC_BYTE;
					continue;
			}
			else {
					crc = crc16_floating(*pu8Src, crc);
					*(pu8Dest++) = *pu8Src;
			}
			++pu8Src;
	}

	// Set the CRC

	//Casting the CRC to lets the word be assigned to a non-word boundary in memory
	*(pu8Dest) = (char)(crc >>8);
	pu8Dest++;
	*(pu8Dest) = (char)crc;
	pu8Dest++;

	*(pu8Dest++) = PROTO_END_BYTE;
	*(pu16Dest_len) = pu8Dest - pu8Dest_start;
}

