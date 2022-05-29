// Define pinout of components
// arduino-cli compile -b arduino:avr:nano
const byte mot_A_1 = 8;
const byte mot_A_2 = 7;
const byte mot_A_PWM = 9; //should be a PWM capable output

const byte enc_A_a = 2; //should be an interrupt capable output
const byte enc_A_b = 4;

//Initialize various variables
double cpr = 500;  //encoder counts per revolution
double gear_ratio = 46.7; //motor gearbox ratio

volatile long int quad_A = 0; //encoder position

double K_P_A = 3.;
double deadband = 0.1;

long int t0 = millis();

// faster way to read encoder pins
#define readAb bitRead(PIND,enc_A_b)


void setup() {
  int motor_pins[3] = {mot_A_1, mot_A_2, mot_A_PWM};
  for (int i = 0; i < 3; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
  pinMode(enc_A_a, INPUT);
  pinMode(enc_A_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc_A_a), enc_A_ISR, RISING);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  quad_A = 0;

  double target_pos = 360;
  int stop = 0;
  
  while (stop < 100){
    stop += control_A(target_pos);
  }
  
  digitalWrite(mot_A_1, LOW);
  digitalWrite(mot_A_2, LOW);
  digitalWrite(mot_A_PWM, LOW);
  Serial.print("position: ");
  Serial.println(360.*double(quad_A)) / (cpr*gear_ratio);
}

void loop() {

}

//functions 
// interrupt service routine for encoder A, simply increase or decrease encoder count depending on state of lines
void enc_A_ISR() {
  if (readAb == 0){
    quad_A++;
  }   
  else {
    quad_A--;
  }
}


int control_A(double target_pos) {
    int mot_A_pos = (360.*double(quad_A)) / (cpr*gear_ratio);
    int error_A = target_pos - mot_A_pos;
    int d = prop_term(error_A, K_P_A);
    int in_deadband = 0;

    if (error_A >= deadband) {
      digitalWrite(mot_A_1, HIGH);
      digitalWrite(mot_A_2, LOW);  
    }
    else if (error_A <= -deadband) {
      digitalWrite(mot_A_1, LOW);
      digitalWrite(mot_A_2, HIGH);  
    }
    else {
      digitalWrite(mot_A_1, LOW);
      digitalWrite(mot_A_2, LOW);
      in_deadband = 1;
    }

    analogWrite(mot_A_PWM, d);

    return in_deadband;
}

int prop_term(double error, double K_P) {
  return int(max(min((error*K_P), 255), 40));
}

