/*
 * protocol.cpp
 *
 *  Created on: 30/09/2019
 *      Author: Fei
 */

#include <Arduino.h>
#include "protocol.h"

#define DEBUG_BAUD		(9600)    // baudrate for debug comms to the console
#define CELL_BAUD		(19200)

#define CELL_LOOP_MS	(3000)    // 3s - interval between cell data reads - use an odd number of seconds.
#define CELL_READ_MS	(700)     // delay after send for the data to shift in



#define TYPE_LOOP

#ifdef TYPE_LOOP
#define TYPE_TOT		(9)
uint8_t	u8a_msg[TYPE_TOT]	= { SENSOR_TYPE_CNTR_VAL, SENSOR_TYPE_FLAGS,SENSOR_TYPE_MILLIVOLTS,SENSOR_TYPE_TEMPERATURE , SENSOR_TYPE_FLAGS,SENSOR_TYPE_FLAGS,SENSOR_TYPE_MILLIVOLTS,SENSOR_TYPE_TEMPERATURE , SENSOR_TYPE_FLAGS };
uint16_t u16a_msg_data[TYPE_TOT] = { 0, 0, 0,0,0 , 0, 0,0,0};
#else
#define LONG_MSG
#define MSG_NUM			(4)
#define MSG_LENGTH		(4)
uint8_t	u8a_msg[MSG_NUM][MSG_LENGTH]	= {
		{SENSOR_TYPE_CNTR_VAL, SENSOR_TYPE_CNTR_VAL, SENSOR_TYPE_CNTR_VAL, SENSOR_TYPE_CNTR_VAL},
		{SENSOR_TYPE_FLAGS, SENSOR_TYPE_FLAGS, SENSOR_TYPE_FLAGS, SENSOR_TYPE_FLAGS},
		{SENSOR_TYPE_CNTR_VAL, SENSOR_TYPE_CNTR_VAL, SENSOR_TYPE_CNTR_VAL, SENSOR_TYPE_CNTR_VAL},
		{SENSOR_TYPE_FLAGS, SENSOR_TYPE_FLAGS, SENSOR_TYPE_FLAGS, SENSOR_TYPE_FLAGS}};
uint16_t u16a_msg_data[MSG_NUM][MSG_LENGTH] = {
		{ 4,0,0,0 },
		{0,0,0,0},
		{0,0,0,0},
		{0,0,0,0}};
#endif

void sendCellMsg(int type, uint16_t data) {
	protocol msg(type, data);	//build message

	//send via cell port
	Serial2.flush();
	Serial2.print(msg.output());
	Serial.println(msg.output());

	//show the packet we sent via debug port
	Serial.print("Sending ");
	Serial.println(msg.report_packets());

}


void sendCellMsg(uint8_t u8_num, uint8_t type[], uint16_t data[]) {
	protocol msg(u8_num, type, data);	//build message

	//send via cell port
	Serial2.flush();
	Serial2.print(msg.output());
	Serial.println(msg.output());

	//show the packet we sent via debug port
	Serial.print("Sending ");
	Serial.println(msg.report_packets());

}


void sendCellMsg(void) {
	sendCellMsg(SENSOR_TYPE_NF, 0 );
}

void sendCellMsg(int type) {
	sendCellMsg(type, 0 );
}


void readCellMsg(void) {
	uint8_t		u8a_msg[SERIAL_RX_BUFFER_SIZE];

	int bytes_available		= Serial2.available();
	Serial.print(bytes_available);
	Serial.println(" bytes got. ");

	for(int i = 0; i< bytes_available; i++)	{
		u8a_msg[i] = Serial2.read();
	}

	uint8_t u8_msg_start = 0;

	while(u8_msg_start < bytes_available)	{
		//check the message
		uint8_t u8_packet_num;
		u8_msg_start = protocol::fu8_is_valid(&u8a_msg[u8_msg_start],\
				bytes_available, u8_packet_num);

		//if message is valid
		if(u8_msg_start != UNKNOWN_MSG_HDR)
		{
			//put the message in a protocol to get packet information
			protocol rcvd_msg(&u8a_msg[u8_msg_start], u8_packet_num);
			//send the packet information via debug port
			Serial.println(rcvd_msg.report_packets());
			//move forward
			u8_msg_start += protocol::fu8_msg_len(rcvd_msg.u8_packets);
			u8_msg_start ++; //skip an end
		}
		else break;//invalid message
	}

	Serial.println("--End--");
}


void setup() {
	  Serial.begin(DEBUG_BAUD);
	  Serial.println("Test Starting...");
	  Serial2.begin(CELL_BAUD);
}

void loop() {

	static unsigned long  sendMillis	= millis() + 1000;
	static bool read_f = false;
	static int	msg_type = 0;


	unsigned long readMillis, currentMillis;


	currentMillis = millis();
	//time to send msg
	if (currentMillis >= sendMillis) {

		////////loop send///////////
#ifdef TYPE_LOOP
			sendCellMsg(u8a_msg[msg_type], u16a_msg_data[msg_type]); //TYPE circulation
			msg_type++;
			msg_type %= TYPE_TOT;
#endif

#ifdef LONG_MSG
			sendCellMsg(MSG_LENGTH,u8a_msg[msg_type], u16a_msg_data[msg_type]); //TYPE circulation
			msg_type++;
			msg_type %= MSG_NUM;
#endif

		// wait for the cell data to arrive
		readMillis        = sendMillis + CELL_READ_MS;
		read_f = true;
		// set time for next sending
		sendMillis      += CELL_LOOP_MS;
	} // end of sendMillis

	//read msg
	if (read_f == true && currentMillis >= readMillis) {
		readCellMsg();
		read_f = false;
	}
}


