#include "pinout.h"
#include "message_protocol.h"


//This needs to go 
int motor_pins[3] = {MOT_L_A, MOT_L_B, MOT_L_PWM};
//Initialize various variables
int cpr = 500000;  //encoder counts per revolution
int gear_ratio = 46700; //motor gearbox ratio * 1000

byte MOT_L_CUR = 0;
bool MOT_L_DIR = true;

volatile long int quad_A = 0; //encoder position

int K_P_A = 3000;
int deadband = 100;

long int t0 = millis();

// faster way to read encoder pins
#define readAb bitRead(PIND,ENC_L_A)


void enc_A_ISR() {
  if (readAb == 0){
    quad_A++;
  }   
  else {
    quad_A--;
  }
}

int bytes_to_int(byte bytes[]) { 
	return bytes[0] + bytes[1] << 8;
}


void cmd_stop() {
	digitalWrite(MOT_L_PWM, false);
	digitalWrite(MOT_L_A, false);
	digitalWrite(MOT_L_B, false);
	digitalWrite(MOT_R_PWM, false);
	digitalWrite(MOT_R_A, false);
	digitalWrite(MOT_R_B, false);
	
	Serial.write(MP_STATUS_SUCCES);
	Serial.write(0);
}

byte cmd_set_cur(byte msg[]) {
	
	//mysterius bit operations :)
	digitalWrite((MOT_L_A & (~msg[0])) + (MOT_R_A & msg[0]) , msg[1]);
	digitalWrite((MOT_L_B & (~msg[0])) + (MOT_R_B & msg[0]), ~msg[1]);
	analogWrite((MOT_L_PWM & (~msg[0])) + (MOT_R_PWM & msg[0]), msg[2]);

	Serial.write(MP_STATUS_SUCCES);
	Serial.write(0);
	return 0;
}


void setup() {
	
  	for (int i = 0; i < 3; i++) {
   		pinMode(i, OUTPUT);
   		digitalWrite(i, LOW);
  	}
  
	pinMode(ENC_L_A, INPUT);
	pinMode(ENC_L_B, INPUT);
	attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_A_ISR, RISING);
	Serial.begin(9600);
	while (!Serial) {
    	; // wait for serial port to connect. Needed for native USB port only
  	}


}



void loop() {
	byte avail = Serial.available();
	byte dlen = Serial.peek();
	 
	if (avail > (1 + dlen)) {
		Serial.read(); // Now delete the first message
		byte cmd = Serial.read();
		byte exec_cmd = MP_CMD_STOP;
		if (cmd != MP_CMD_PING) {
			exec_cmd = cmd;
		}
		byte msg_data[dlen];
		Serial.readBytes(msg_data, dlen);
		switch (cmd) {
		case MP_CMD_PING:
			Serial.write(MP_STATUS_SUCCES);
			Serial.write(0);
			break;
		case MP_CMD_STOP:
			cmd_stop();
			break;
		case MP_CMD_SET_MOT_CURRENT:
			cmd_set_cur(msg_data);
			break;
		case MP_CMD_SET_MOT_VEL:
			int mot_vel = bytes_to_int(msg_data);
			Serial.write(MP_STATUS_SUCCES);
			int a = 200;
			Serial.write(a);
			
		default:
			Serial.write(MP_STATUS_UNKOWN);
			Serial.write(0x00);
			break;
		
		}
	}	
}




