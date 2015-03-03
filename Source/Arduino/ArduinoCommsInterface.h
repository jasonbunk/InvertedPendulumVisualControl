#ifndef ___ARDUINO_COMMS_INTERFACE_H_____
#define ___ARDUINO_COMMS_INTERFACE_H_____

#include <string>
#include <stdint.h>

class ArduinoSerialComm
{
protected:
	int serial_port_connection;
	bool LED_state;
public:
	double LED_blink_timer;
	double LED_blink_period;
	
	ArduinoSerialComm() : serial_port_connection(-1), LED_state(false), LED_blink_timer(0.0), LED_blink_period(1.0) {}
	
	
	void CloseConnection();
	void Connect(std::string where, int baudRate);
	void SendByte(uint8_t thebyte);
	void FakeSimulationSendByte(uint8_t thebyte) {}
	
	void UpdateLEDblinker(double dt);
	
	
	~ArduinoSerialComm() {CloseConnection();}
};

#endif
