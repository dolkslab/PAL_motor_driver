#include "pinout.h"
#include "message_protocol.h"


byte motor_pins[] = {MOT_L_A, MOT_L_B, MOT_L_PWM, MOT_R_A, MOT_R_B, MOT_R_PWM};
//Initialize various variables
int cpr_gr = 23350; //motor gearbox ratio * encoder counts per revolution

volatile long int quad_L = 0; //encoder position
volatile long int quad_R = 0;

int v_sp_R = 0; //velocity setpoint for pid-velocity control
int v_sp_L = 0;

void enc_L_ISR() {
  if (bitRead(PIND,ENC_L_A) == 0){
    quad_L++;
  }   
  else {
    quad_L--;
  }
}

void enc_R_ISR() {
  if (bitRead(PIND,ENC_L_A) == 0){
    quad_R++;
  }   
  else {
    quad_R--;
  }
}

//helper functions
//Yes I would prefer to put these in a separate file, but arduino is stupid and doesn't allow me to do this like normal C.

//Convert an array of bytes into a 16-bit int
int bytes_to_int(byte bytes[]) { 
	return bytes[0] + bytes[1] << 8;
}

//Convert an 16-bit int to an array of bytes
void int_to_bytes(byte *buf, int x) {
	buf[0] = ((x >> 8) & 0xFF);
	buf[1] = (x & 0xFF);
}

//left sided first order numerical difference
int ldf(long int *x, int dt) {
	return (int) ((x[0] - x[1]))/dt;
}
//trapezoidal integration of 1 unit
int trapz(int *x, int dt) {
	return (int) ((x[0]+x[1])/2)*dt;
} 



//command functions
void cmd_stop() {
	for (int i = 0; i < 6; i++) {
   		digitalWrite(i, LOW);
  	}
	
	Serial.write(MP_STATUS_SUCCES);
	Serial.write(0);
}

void cmd_set_cur(byte msg[]) {
	//For byte format, see message_protocal.h
	//mysterious bit operations :)
	digitalWrite((MOT_L_A & (~msg[0])) + (MOT_R_A & msg[0]) , msg[1]);
	digitalWrite((MOT_L_B & (~msg[0])) + (MOT_R_B & msg[0]), ~msg[1]);
	analogWrite((MOT_L_PWM & (~msg[0])) + (MOT_R_PWM & msg[0]), msg[2]);

}

void pid_velocity() {
	/*only implement left channel for now
	Also only does PI control, rate feedback to be implemented later
	*/
	
	//Measure the timestep since when the function was last called
	static unsigned long int t1 = 0;
	unsigned long int t0 = micros();
	unsigned int dt = (int) t0 - t1;
	
	//Save a history of the wheel angle for integration and derivation
	static long int pos_hist_L[2] = {0};
	static long int ERR_L = 0; //integral of error
	
	int Kp = 1;
	int Ki = 0.1*Kp;

	//pos_hist_L[2] = pos_hist_L[1];
	pos_hist_L[1] = pos_hist_L[0];
	pos_hist_L[0] = quad_L;
	
	int vel_L = ldf(pos_hist_L, dt);
	int sp_I_L[] = {v_sp_L, v_sp_L};
	ERR_L += (long int) trapz(sp_I_L, dt) - (pos_hist_L[0] - pos_hist_L[1]);
	
	int L_cur = Kp * (v_sp_L - vel_L) + Ki * ERR_L; 
	byte cmd_L[] = {0x00, (byte) 0xFF*(L_cur > 0), (byte) min(abs(L_cur), 255)};
	cmd_set_cur(cmd_L);
	
	
	t1 = t0;
}


void setup() {
	//setup code, set correct pinmodde for each pin and write outputs low initially
  	for (int i = 0; i < 6; i++) {
   		pinMode(i, OUTPUT);
   		digitalWrite(i, LOW);
  	}
  
	pinMode(ENC_L_A, INPUT);
	pinMode(ENC_L_B, INPUT);
	attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_L_ISR, RISING);
	Serial.begin(9600);
	while (!Serial) {
    	; // wait for serial port to connect. Needed for native USB port only
  	}
}



void loop() {
	byte avail = Serial.available();
	byte dlen = Serial.peek();
	byte exec_cmd = MP_CMD_STOP;
	if (avail > (1 + dlen)) {
		Serial.read(); // Now delete the first message
		byte cmd = Serial.read();
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
			Serial.write(MP_STATUS_SUCCES);
			Serial.write(0);
			break;
		case MP_CMD_SET_MOT_VEL:
			
			Serial.write(MP_STATUS_SUCCES);
			v_sp_L = bytes_to_int(msg_data);
			Serial.write(MP_STATUS_SUCCES);
			Serial.write(0x00);
			break;
		default:
			Serial.write(MP_STATUS_UNKOWN);
			Serial.write(0x00);
			break;
		
		}
	}
	switch (exec_cmd) {
	case MP_CMD_SET_MOT_VEL:
		pid_velocity();
		break;
	default:
		break;
	}	
}




