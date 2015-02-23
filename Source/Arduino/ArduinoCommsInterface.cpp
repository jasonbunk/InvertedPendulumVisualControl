/*
 * Communicate with the Arduino UNO that drives the printer motor.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "ArduinoCommsInterface.h"
#include "ArduinoSerial.h"
#include <iostream>
#include <assert.h>
#include <unistd.h>




void ArduinoSerialComm::CloseConnection()
{
	if(serial_port_connection != -1) close(serial_port_connection);
	serial_port_connection = -1;
}


void ArduinoSerialComm::Connect(std::string where, int baudRate)
{
	assert(where.empty() == false);
	assert(baudRate > 0);
	
	//where = "/dev/ttyACM0";
	//baudRate = B57600;
	
	serial_port_connection = serialport_init(where.c_str(), baudRate);
	
	if(serial_port_connection != -1) {
		std::cout<<"================================================================================="<<std::endl;
		std::cout<<"===================== SUCCESSFULLY CONNECTED TO ARDUINO VIA SERIAL =============="<<std::endl;
		std::cout<<"================================================================================="<<std::endl;
	}
}


void ArduinoSerialComm::UpdateLEDblinker(double dt)
{
	LED_blink_timer += dt;
	
	if(LED_blink_timer > LED_blink_period)
	{
		LED_blink_timer = 0.0;
		LED_state = !LED_state;
		
		if(serial_port_connection != -1)
		{
			int rc = serialport_writebyte(serial_port_connection, LED_state ? 'h' : 'l');
			if(rc==-1) std::cout << "failed to write byte to Arduino" << std::endl;
		}
	}
}


void ArduinoSerialComm::SendByte(uint8_t thebyte)
{
	if(serial_port_connection != -1)
	{
		int rc = serialport_writebyte(serial_port_connection, thebyte);
		if(rc==-1) std::cout << "failed to write byte to Arduino" << std::endl;
	}
}




