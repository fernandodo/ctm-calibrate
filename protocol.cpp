/*
 * protocol.cpp
 *
 *  Created on: 30/09/2019
 *      Author: Fei
 */
#include <Arduino.h>
#include "protocol.h"

#define LDPC_BUF_SIZE			PACKET_LEN

#define MSG_HEADER				(0x0A)
#define MSG_END					(0x0D)




// used to generator the 12-bit code-words from the 8-bit data-bytes
const uint8_t		u8a_generator_matrix[4]				= { 0xD5, 0xB3, 0x0F, 0xE8 };

// used to determine which bits have been corrupted
const uint16_t		u16a_parity_check_matrix[4]			= { 0x8E8, 0x40F, 0x2B3, 0x1D5 };

// used to determine which bits are to be flipped
const uint16_t		u16a_coset_leader_table[16]			= { 0x000, 0x800, 0x400, 0x008,
														    0x200, 0x020, 0x002, 0x208,
														    0x100, 0x040, 0x004, 0x084,
														    0x010, 0x080, 0x001, 0x801 };


protocol::protocol(){
	_packet_num = 0;
}

protocol::protocol(uint8_t u8a_data_buf[], uint8_t u8_packet_num) {
	_packet_num = u8_packet_num;
	for(int i = 0; i< u8_packet_num; i++) {
		fu8_decap_packet(&u8a_data_buf[fu8_msg_len(i)], \
				_packets[i].u8_type, _packets[i].u16_data);
	}
}

protocol::protocol(uint8_t u8_type, uint16_t u16_data) {

	if(u8_type == SENSOR_TYPE_EMPTY)
	{
		_packet_num = 0;
	}
	else{
		_packet_num = 1;
		_packets[0].u8_type = u8_type;
		_packets[0].u16_data = u16_data;
	}
}

protocol::protocol(uint8_t u8_packet_num, uint8_t u8_type[], uint16_t u16_data[]){
	_packet_num = u8_packet_num;
	for(int i = 0; i< u8_packet_num; i++) {
		_packets[i].u8_type = u8_type[i];
		_packets[i].u16_data = u16_data[i];
	}
}

protocol::~protocol() {
	// TODO Auto-generated destructor stub
}

/*
 * encode the message in the buffer
 * add a header and end
 * return a string for tx
 */
String protocol::output()
{
	String msg;		//start of the message
	char packet[PACKET_LEN + 1];
	packet[PACKET_LEN] = '\0';	//put an end
	msg +=  (char)MSG_HEADER;
	for(int i = 0; i< _packet_num; i++){
		fv_encap_packet(_packets[i].u8_type, _packets[i].u16_data, (uint8_t *) packet);
		msg += packet;
	}
	msg +=  (char)MSG_END;

	return msg;
}

String protocol::report_packets()
{
	String ret;

	if(_packet_num == 0) {
		ret = "Empty message.";
	}
	else {
		ret += _packet_num;
		ret += " packets";
		for(int i = 0 ; i < _packet_num; i++)
		{
			ret += "\r\nType: ";
			ret += _packets[i].u8_type;
			ret += " Data: ";
			if(_packets[i].u8_type == 4)
			{
				int16_t s16_temp = (int16_t)(_packets[i].u16_data);
				if(s16_temp < 0)
				{
					ret += '-';
					s16_temp = -s16_temp;
				}
				ret += s16_temp;
			}
			else ret += _packets[i].u16_data;
		}
	}

	return ret;
}

/*
 * check the message in u8a_data_buf if it is valid
 * only check header, end and length
 * return the index of header and put the number of packets in u8_packet_num
 * if it is not valid return UNKNOWN_MSG_HDR(0xFF)
 */
static uint8_t protocol::fu8_is_valid(uint8_t u8a_data_buf[], uint8_t u8_len, uint8_t& u8_packet_num)
{
	uint8_t u8_i, u8_msg_start = 0, u8_rx_packet_wp = 0; //
	bool msg_ok = false;
	u8_packet_num = 0;

	for(u8_i = 1; u8_i < u8_len && !msg_ok ; u8_i ++)
	{
		switch(u8a_data_buf[u8_msg_start])
		{
		case UNKNOWN_MSG_HDR:
		default:
			//throw away garbage message..
			u8_msg_start = u8_i;
			break;

		//we have a valid header..
		case MSG_HEADER:
			if(u8_rx_packet_wp == 0 && u8a_data_buf[u8_i] == MSG_END)
			{//ignore MSG_END
				msg_ok				= true;		//valid message received
			}
			else//not end
			{
				if( ++ u8_rx_packet_wp == PACKET_LEN)
				{
					u8_packet_num++;		// a whole packet received
					u8_rx_packet_wp = 0 ;		// reset packet pointer, so we can check next byte if it is END
				}
			}
			break;
		}
	}

	return msg_ok? u8_msg_start: UNKNOWN_MSG_HDR;
}



/*******************************************************************************
  * Encodes with low-density parity-check codes
  * The first byte (header) is not encoded, thereafter the 8-bits are encoded
  * into 12-bits. The bytes are sent in the order;
  *   B1   B2   LDPC   B3   B4   LDPC   ...
  * Hence every 3rd-byte has the error correction bits for the previous 2-bytes.
  * The interleaving is done over blocks of 6-bytes.
  *
  * Returns the length of the encoded message
  *******************************************************************************/
static uint8_t		protocol::fu8_fec_encode(uint8_t p_msg[], uint8_t u8_msg_len)
 {
 	uint16_t	u16a_code_word[LDPC_BUF_SIZE];
 	uint8_t		u8_i, u8_encode, u8_g_matrix, u8_j;
 #ifdef INTERLACING
 	uint8_t		u8_k, u8_g, u8_h;
 #endif
 	uint8_t		u8a_tmp[LDPC_BUF_SIZE];

 	// ensure that packet size is modulus 4
 	u8_i	= 3 - ((u8_msg_len + 3) & 0x03);
 	while (u8_i) {
 		// pad-out with 0xFF
 		p_msg[u8_msg_len]	= 0xFF;
 		u8_msg_len++;
 		u8_i--;
 	} // end of while padding bytes needed.

 	for (u8_i = 0; u8_i < u8_msg_len; u8_i++) {
 		// initialise code-word with orginial byte
 		u16a_code_word[u8_i]	= p_msg[u8_i];

 		// encode using the generator matrix
 		for (u8_g_matrix = 0; u8_g_matrix < 4; u8_g_matrix++) {
 			// bit-mask byte with generator matrix
 			u8_encode	= u8a_generator_matrix[u8_g_matrix] & p_msg[u8_i];

 			// calculate odd-parity
 			u8_encode	^= u8_encode >> 8;
 			u8_encode	^= u8_encode >> 4;
 			u8_encode	^= u8_encode >> 2;
 			u8_encode	^= u8_encode >> 1;

 			// modulus 2
 			u8_encode	&= 0x01;

 			// calculate code-word
 			u16a_code_word[u8_i]	+= u8_encode << (u8_g_matrix + 8);
 		}
 	} // end of for each byte in message

 	// increase message length for parity bits
 	u8_msg_len	= (3 * u8_msg_len) >> 1;

 	u8_encode	= 0;
 	// new message with added ldpc
 	for (u8_i = 0; u8_i < u8_msg_len; u8_i += 3) {		// create information bit array

 		// convert code-words into data-bytes
 		u8a_tmp[u8_i+0]	= u16a_code_word[u8_encode];
 		u8a_tmp[u8_i+1]	= u16a_code_word[u8_encode + 1];
 		u8a_tmp[u8_i+2]	= (u16a_code_word[u8_encode] >> 8) + ((u16a_code_word[u8_encode + 1] & 0x0F00) >> 4);
 		u8_encode	+= 2;
 	} // end of for each 3byte block in message

 	// interleaving bits across the message in block sizes of six to provide burst-noise immunity, step across the 8-bits in each byte
 	for (u8_i = 0; u8_i < u8_msg_len; u8_i+= 6) {
 #ifdef INTERLACING
 		// reset indexes
 		u8_g	= 0;
 		u8_h	= 0;
 #endif

 		for (u8_j = 0; u8_j < 6; u8_j++) {
 #ifdef INTERLACING
 			// reset bytes
 			p_msg[u8_i + u8_j]	= 0;

 			for (u8_k = 0; u8_k < 8; u8_k++) {
 				// interleave bits
 				p_msg[u8_i + u8_j]	+= ( (u8a_tmp[u8_i + u8_g] >> u8_h) & 0x01 ) << u8_k;

 				// increment indexex
 				if (++u8_g > 5) {
 					u8_g	= 0;
 					u8_h++;
 				}
 			} // end of for each bit in a byte
 #else
 			p_msg[u8_i + u8_j]	= u8a_tmp[u8_i + u8_j];
 #endif
 		} // end of for each byte in a block
 	} // end of for each block of 6 bytes

 	// return length of encoded message
 	return u8_msg_len;
 } // end of fu8_fec_encode function --------------------


 /*******************************************************************************
  * Decodes the low-density parity-check codes via lookup table
  * Applies the parity check matrix to calculate the syndrome which is used, via
  * a lookup table for error correction.
  *
  * Returns the length of the decoded message
  *******************************************************************************/
static uint8_t		protocol::fu8_fec_decode(uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len)
 {
 	static uint8_t	u8a_decode[LDPC_BUF_SIZE];
 //	static uint8_t	u8a_save[LDPC_BUF_SIZE];
 	uint16_t		u16a_code_word[4];
 	uint8_t			u8_i, u8_j, u8_len;
 #ifdef INTERLACING
 	uint8_t			u8_k, u8_g, u8_h;
 #endif
 	uint8_t			u8a_tmp[LDPC_BUF_SIZE];

 	// only decode in blocks of six
 	u8_msg_len	= ((u8_msg_len / 6) * 6);
 	u8_len		= 0;

 	if (u8_msg_len > 6) {
 		u8_i	= u8_msg_len - 6;
 	}
 	else {
 		u8_i	= 0;
 	}

 #if 0
 	// check for previous decoding
 	for (u8_j = 0; u8_j < u8_msg_len; u8_j += 6) {
 		if ( (u8a_save[u8_j+0] == en_msg[u8_j+0]) && (u8a_save[u8_j+1] == en_msg[u8_j+1]) && (u8a_save[u8_j+2] == en_msg[u8_j+2]) &&
 			 (u8a_save[u8_j+3] == en_msg[u8_j+3]) && (u8a_save[u8_j+4] == en_msg[u8_j+4]) && (u8a_save[u8_j+5] == en_msg[u8_j+5]) ) {
 			// indexing translations, used to calculate the decoded message length
 			u8_len	= ((u8_i << 1) + 1) / 3;
 			u8_i	+= 6;
 		}
 	}

 	// save previous message
 	for (u8_j = 0; u8_j < u8_msg_len; u8_j++) {
 		u8a_save[u8_j]	= en_msg[u8_j];
 	}
 #endif

 	// re-distribute the bits into their correct byte
 //	for (; u8_i < u8_msg_len; u8_i += 6) {
 	for (u8_i = 0; u8_i < u8_msg_len; u8_i += 6) {
 		for (u8_j = 0; u8_j < 6; u8_j++) {
 #ifdef INTERLACING
 			// reset bytes
 			u8a_tmp[u8_i + u8_j]	= 0;

 			// reset indexes
 			u8_g		= 0;
 			u8_h		= u8_j;

 			for (u8_k = 0; u8_k < 8; u8_k++) {
 				// remove interleaving
 				u8a_tmp[u8_i + u8_j]	+= ( (en_msg[u8_i + u8_g] >> u8_h) & 0x01 ) << u8_k;

 				// increment indexes
 				u8_h		+= 6;
 				if (u8_h > 7) {
 					u8_g++;
 					u8_h	-= 8;
 				}
 			} // end of for each bit of a byte
 #else
 			u8a_tmp[u8_i + u8_j]	= en_msg[u8_i + u8_j];
 #endif
 		} // end of for each byte in a block

 		// code-words
 		u16a_code_word[0]	= u8a_tmp[u8_i + 0] + ( (u8a_tmp[u8_i + 2] & 0x0F) << 8);
 		u16a_code_word[1]	= u8a_tmp[u8_i + 1] + ( (u8a_tmp[u8_i + 2] & 0xF0) << 4);
 		u16a_code_word[2]	= u8a_tmp[u8_i + 3] + ( (u8a_tmp[u8_i + 5] & 0x0F) << 8);
 		u16a_code_word[3]	= u8a_tmp[u8_i + 4] + ( (u8a_tmp[u8_i + 5] & 0xF0) << 4);

 		for (u8_j = 0; u8_j < 4; u8_j++) {
 			// decode byte in packet
 			fv_fec_byte_decode(&u16a_code_word[u8_j]);

 			// reduce to 8-bit
 			u16a_code_word[u8_j] &= 0xFF;
 		} // end of for each block of 4 bytes

 		// decode
 		u8a_decode[u8_len + 0]	= u16a_code_word[0];
 		u8a_decode[u8_len + 1]	= u16a_code_word[1];
 		u8a_decode[u8_len + 2]	= u16a_code_word[2];
 		u8a_decode[u8_len + 3]	= u16a_code_word[3];

 		// since we've decoded another 4 bytes add 4 to the length.
 		u8_len	+= 4;
 	} // end of for each block of 6 bytes

 	// decoded message length
 	u8_msg_len	= u8_len;

 	for (u8_i = 0; u8_i < u8_msg_len; u8_i++) {
 		de_msg[u8_i]		= u8a_decode[u8_i];
 	}

 	// return decoded message length
 	return u8_msg_len;
 } // end of fu8_fec_decode function --------------------


 /*******************************************************************************
  * Decodes a 12-bits into a 8-bits using the fec
  *******************************************************************************/
static void		protocol::fv_fec_byte_decode(uint16_t *u16_code_word)
{
	uint8_t			u8_p_matrix, u8_syndrome = 0;
	uint16_t		u16_decode;

	for (u8_p_matrix = 0; u8_p_matrix < 4; u8_p_matrix++) {
		// bit-mask byte with generator matrix
		u16_decode		= u16a_parity_check_matrix[u8_p_matrix] & (*u16_code_word);

		// calculate odd-parity
		u16_decode		^= u16_decode >> 8;
		u16_decode		^= u16_decode >> 4;
		u16_decode		^= u16_decode >> 2;
		u16_decode		^= u16_decode >> 1;

		// modulo 2
		u16_decode		&= 1;

		// sydrome
		u8_syndrome		+= u16_decode << u8_p_matrix;
	}

	// error correction to the codeword, XOR the codeword with the coset leader
	(*u16_code_word)	^= u16a_coset_leader_table[u8_syndrome];
} // end of fv_fec_byte_decode function ----------------

/***********************************************************
	@brief returns checksum for a packet.
	u8a_buf[] should point to first byte of segment to check.
	u8_msg_len is the number of bytes covered by checksum.
***********************************************************/
uint8_t protocol::fu8_checksum(uint8_t u8_msg_len, uint8_t u8a_buf[])
{
	uint8_t u8_i, u8_sum = 0;
	for(u8_i = 0; u8_i < u8_msg_len; u8_i++)
	{
		u8_sum += u8a_buf[u8_i];
	}
	return u8_sum;
}// end of fu8_checksum --------------------------------------


void protocol::fv_encap_packet(uint8_t u8_type, uint16_t u16_value, uint8_t u8a_packet[])
{
	int8_t s8_i;
	uint8_t u8a_data[DATA_LEN];
	u8a_data[0]					= u8_type;
	u8a_data[1]					= u16_value>> 8;//value h
	u8a_data[2]					= u16_value;	//value l
#ifdef USE_CS
	//step 1: add check sum, length 4
	u8a_data[DATA_LEN - 1]		= fu8_checksum(DATA_LEN - 1, u8a_data);
#endif


#ifdef USE_FEC
	//step 3: fec added
	fu8_fec_encode(u8a_data, DATA_LEN);
#endif

	//step 2: ASCII encoded
	for(s8_i = ENCODED_LEN - 1 ; s8_i >=  0 ; s8_i--)
	{
		u8a_packet[s8_i * 2]	= BYTE2ASCII(u8a_data[s8_i] >> 4);
		u8a_packet[s8_i * 2 +1]	= BYTE2ASCII(u8a_data[s8_i] & 0xF);
	}
}

/***********************************************************
	@brief	Decapsulation: gets type and value from a packet
	@param	u8a_packet the packet to be processed
			u8_type_p, u16_val_p store the data type and value
	@retval	TRUE if succeed
***********************************************************/
uint8_t protocol::fu8_decap_packet(uint8_t u8a_packet[], uint8_t& u8_type_p, uint16_t& u16_val_p)
{

	uint8_t u8a_data[PACKET_LEN + 1];		//double the length of data buffer
	uint8_t u8_i;
	u8_type_p 				= SENSOR_TYPE_EMPTY;	//assume failed

	memcpy(u8a_data, u8a_packet, PACKET_LEN);
	u8a_data[PACKET_LEN] = '\0';

	//step 2.2: ASCII to value, single digit
	for(u8_i = 0; u8_i < PACKET_LEN; u8_i ++)
	{
		u8a_data[u8_i] 			= ASCII2BYTE(u8a_data[u8_i]);
	}


	//step 2.1:  2 nibbles combined to 1 byte
	for(u8_i = 0; u8_i < ENCODED_LEN; u8_i ++)
	{
		u8a_data[u8_i] 			= (u8a_data[u8_i * 2]<< 4) + u8a_data[u8_i * 2 + 1];
	}

	//step 3: fec decode, decoded message is in u8a_data
#ifdef USE_FEC
	if( DATA_LEN != fu8_fec_decode(u8a_data, u8a_data, ENCODED_LEN)) return false;
#endif

#ifdef USE_CS
	//step 1: check sum
	if(u8a_data[DATA_LEN - 1]	!= fu8_checksum(DATA_LEN - 1, u8a_data))  return false;
#endif

	//finally: get type and value
	u8_type_p 				= u8a_data[0];
	u16_val_p 				= (((uint16_t)u8a_data[1])<<8) + u8a_data[2];

	return true;
}

