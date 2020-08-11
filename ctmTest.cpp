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


#define TYPE_LOOP		(1)
#define	LOAD_LOOP		(2)

#define TYPE_TOT		(4)
#define LOOP_TOT		(0)


#define LOOP_MSG	TYPE_LOOP
#if (LOOP_MSG == TYPE_LOOP)
uint8_t	u8a_msg[TYPE_TOT]	= {SENSOR_TYPE_CNTR_VAL,SENSOR_TYPE_MILLIVOLTS, SENSOR_TYPE_MILLIVOLTS, SENSOR_TYPE_MILLIVOLTS};
uint16_t u16a_msg_data[TYPE_TOT] = {3288, 0,0,0};
//uint16_t u16a_msg_data[TYPE_TOT] = {0, 1};
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
	static bool read_f = false, load_on = false;
	static int	msg_type = 0, loop = 0;


	unsigned long readMillis, currentMillis;


	currentMillis = millis();
	//time to send msg
	if (currentMillis >= sendMillis) {



		////////loop send///////////
		if(LOOP_TOT <= 0 || loop < LOOP_TOT){
#if (LOOP_MSG == TYPE_LOOP)
			load_on = true;
			sendCellMsg(u8a_msg[msg_type], u16a_msg_data[msg_type]); //TYPE circulation
#else if(LOOP_MSG == LOAD_LOOP)
//			load_on = true;
			sendCellMsg(SENSOR_TYPE_FLAGS, load_on? (0x0001<<msg_type) : 0);//Load circulation
#endif
			if(load_on)
			{
				msg_type++;
				load_on = false;
			}
			else
			{
				load_on = true;
			}

			if(msg_type >= TYPE_TOT) {
				//1 circulation
				msg_type = 0;
				loop++;
			}
		}
		/////////loop end////////
		else{
		///////periodical  send
			sendCellMsg();
	//		sendCellMsg(SENSOR_TYPE_TEMPERATURE);

		}



		//		sendCellMsg(SENSOR_TYPE_CNTR_VAL, 3300);

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


