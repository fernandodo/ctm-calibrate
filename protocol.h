/*
 * protocol.h
 *
 *  Created on: 30/09/2019
 *      Author: Fei
 */

#ifndef LIB_PROTOCOL_H_
#define LIB_PROTOCOL_H_

#include <Arduino.h>

/* global variable definitions/declarations */
#define USE_FEC
#define USE_CS

#ifdef USE_CS
	#define DATA_LEN				(4)			//type(1) + value(2) + checksum(1)
#else
	#define DATA_LEN				(3)			//type(1) + value(2)
#endif

#ifdef USE_FEC
#define ENCODED_LEN				((DATA_LEN + 1) >> 1) * 3
#else
#define ENCODED_LEN				DATA_LEN
#endif

#define PACKET_LEN				(ENCODED_LEN * 2)

#define BUFFER_SIZE				(SERIAL_RX_BUFFER_SIZE)
#define PACKET_MAX_NUM			((BUFFER_SIZE - 2)/PACKET_LEN)
#define UNKNOWN_MSG_HDR			(0xFF)

#if 0
#define SENSOR_TYPE_NF			(0)
#define SENSOR_TYPE_DAYLIGHT	(1)
#define SENSOR_TYPE_LIGHT		(2)
#define SENSOR_TYPE_MOTION		(3)
#define SENSOR_TYPE_TEMPERATURE	(4)
#define SENSOR_TYPE_HUMIDITY	(5)
#define SENSOR_TYPE_FIRE		(6)
#define SENSOR_TYPE_LEVEL		(7)
#define SENSOR_TYPE_FLOW		(8)
#define SENSOR_TYPE_DIRECTION	(9)
#define SENSOR_TYPE_CO2			(10)
//#define SENSOR_TYPE_H2			(11)
#define SENSOR_TYPE_PRESSURE	(12)
//#define SENSOR_TYPE_CO			(13)
#define SENSOR_TYPE_POSITION	(14)
#define SENSOR_TYPE_SPEED		(15)
#define SENSOR_TYPE_POWERFACTOR	(16)
#define SENSOR_TYPE_SOUND		(17)
#define SENSOR_TYPE_VOLTAGE		(18)
#define SENSOR_TYPE_CURRENT		(19)
#define SENSOR_TYPE_POWER		(20)
#define SENSOR_TYPE_VOC			(21)
#define SENSOR_TYPE_DUST		(22)
#define SENSOR_TYPE_FREQUENCY	(23)

#define SENSOR_TYPE_FLAGS		(27)
#define SENSOR_TYPE_CNTR_VAL	(28)
#define SENSOR_TYPE_SWITCH_NO	(29)
#define SENSOR_TYPE_SWITCH_NC	(30)
#define SENSOR_TYPE_ENABLE		(31)
#define SENSOR_TYPE_ANALOG		(32)
#endif

enum Sensor_Type
{
	SENSOR_TYPE_NF			= 0,
	SENSOR_TYPE_DAYLIGHT	= 1,
	SENSOR_TYPE_LIGHT		,
	SENSOR_TYPE_MOTION		,
	SENSOR_TYPE_TEMPERATURE	,
	SENSOR_TYPE_HUMIDITY	,
	SENSOR_TYPE_FIRE		,
	SENSOR_TYPE_LEVEL		,
	SENSOR_TYPE_FLOW		,
	SENSOR_TYPE_DIRECTION	,
	SENSOR_TYPE_CO2			,
	SENSOR_TYPE_VOC			,
	SENSOR_TYPE_PRESSURE	,
	SENSOR_TYPE_CO			,
	SENSOR_TYPE_POSITION	,
	SENSOR_TYPE_SPEED		,
	SENSOR_TYPE_POWERFACTOR	,
	SENSOR_TYPE_SOUND		,
	SENSOR_TYPE_VOLTAGE		,
	SENSOR_TYPE_CURRENT		,
	SENSOR_TYPE_POWER		,
	SENSOR_TYPE_MILLIVOLTS	,
	SENSOR_TYPE_DUST		,
	SENSOR_TYPE_FREQUENCY	,
	SENSOR_TYPE_NOT_USED_1	,
	SENSOR_TYPE_NOT_USED_2	,
	SENSOR_TYPE_CALI		,
	SENSOR_TYPE_FLAGS		,
	SENSOR_TYPE_CNTR_VAL	,
	SENSOR_TYPE_SWITCH_NO	,
	SENSOR_TYPE_SWITCH_NC	,
	SENSOR_TYPE_ENABLE		,
	SENSOR_TYPE_EMPTY		= 255,

};


#define BYTE2ASCII(x)			((x) < 10 ? ('0' + (x)) : ('A' + ((x) - 10)))
#define ASCII2BYTE(x)			(0x0f & ((x) <= '9'? (x) - '0': (x) -'a' + 10)) //...0~9...A~F...a~f

typedef struct {
	uint8_t		u8_type;
	uint16_t	u16_data;
}Packet;

class protocol {
private:
	uint8_t	fu8_fec_encode(uint8_t p_msg[], uint8_t u8_msg_len);
	uint8_t	fu8_fec_decode(uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len);
	void	fv_fec_byte_decode(uint16_t *u16_code_word);
	void 	fv_encap_packet(uint8_t u8_type, uint16_t u16_value, uint8_t u8a_packet[]);
	uint8_t	fu8_decap_packet(uint8_t u8a_packet[], uint8_t& u8_type_p, uint16_t& u16_val_p);
	uint8_t	fu8_checksum(uint8_t u8_msg_len, uint8_t u8a_buf[]);

	uint8_t			_packet_num;
	Packet			_packets[PACKET_MAX_NUM];

public:
	const uint8_t&	u8_packets = _packet_num;
	Packet* const	packets = _packets;

	//constructors
	protocol();
	protocol(uint8_t u8a_data_buf[], uint8_t u8_packet_num);
	protocol(uint8_t u8_type, uint16_t u16_data);
	protocol(uint8_t u8_packet_num, uint8_t u8_type[], uint16_t u16_data[]);

	//destructor:
	virtual ~protocol();

	String output();
	String report_packets();

	static uint8_t fu8_is_valid(uint8_t u8a_data_buf[], uint8_t u8_len, uint8_t& u8_packet_num);
	static uint8_t fu8_msg_len(uint8_t u8_packet){ return	(1 + u8_packet * PACKET_LEN);}

};



#endif /* LIB_PROTOCOL_H_ */
