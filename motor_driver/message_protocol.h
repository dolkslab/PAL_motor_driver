
// INGOING MESSAGE FORMAT:
// | DLEN (1 byte) | CMD (1 byte) | DATA ('DLEN' bytes) |
// Data length varies per command, some have fixed data lengths, while some can vary
// i.e. the length of path that is sent over
// DLEN is sent before the command to allow the arduino to peek() and test if the 
// required amount of commands have been recieved


enum mp_cmd {
	//Ping command, no data. Returns pong response if succesful
	MP_CMD_PING = 0x01,
	
	//Stop both driving motors, i.e. force all driving pins to the H-bridges low.
	//No data, so DLEN is zero.
	MP_CMD_STOP = 0x02,
	
	//Set drive current for motor directly. 
	//Command length is three bytes: 
	//Motor A/B select (0x00 or 0xff), drive direction (0x00 or 0xff), 
	//and drive duty cycle (0x00 - 0xff)
	//Used for testing chasis	
	MP_CMD_SET_MOT_CURRENT = 0x03,
	
	//Set angular velocity of motor, the driver will then use PID to
	//maintain this speed.
	//Command length is three bytes:
	//Motor A/B select (0x00 or 0xff), and velocity, which is two bytes that together form a signed int (16 bit)
	//Direction is based of the sign, 2^15-1 will be interpreted as maximum angular velocity ~60RPM
	MP_CMD_SET_MOT_VEL = 0x04,
};

enum mp_status {
	//Command was caried out succesfully
	MP_STATUS_SUCCES = 0x01,
	
	//Unkown command issued
	MP_STATUS_UNKOWN = 0xff,
	
	//More specific error cases go here
};

#define MP_MAX_DLEN 255
