#include <Stepper.h>
#include <Encoder.h>

#define Ppin1  6
#define Ppin2 7
#define Ppin3 8
#define Ppin4 9
#define PpinEnable 10
#define pinPWM 11
#define pot A0
#define sensor_dir 4
#define sensor_esq 5
#define pinPause 12
#define pinChangeDir 13

const int stepfuso = 8;
const int stepsPerRevolution = 200;
const int vel_step_motor = 300;
const long motor_step_cte = 48;


int steps_to_run = 24;

// ✅ usando biblioteca Encoder (pinos 2 e 3)
Encoder myEnc(2, 3);

// ✅ agora é long
long pos_motor_encoder = 0;

Stepper myStepper(stepsPerRevolution, Ppin1, Ppin2, Ppin3, Ppin4);

int calculateStepsPerStep(float diameter){
  int total_steps, steps;
  total_steps = ceil(diameter / ((1.8/360)*stepfuso)); 
  float aa = (diameter / ((1.8/360)*stepfuso));

  steps = ceil(total_steps * steps_to_run / motor_step_cte);
  Serial.println(aa);
  Serial.println("Passos por passo do motor dc:");
  Serial.println(steps);
  if(steps < 1) return 1;

  return steps;
}

void move(int steps){
  myStepper.setSpeed(vel_step_motor);
  digitalWrite(PpinEnable, HIGH); 
  myStepper.step(steps);
}

void run(float total_turns, float diametro_fio){
 
  int velMotor = map(analogRead(pot),0,1023,0,255);
 
  int turns = 0;
  int last_turns = 0;
  
  bool pause = 1;
  bool state = 0;
  bool state_direction = 0;
  bool last_state = 0;
  bool last_state_direction = 0;
  unsigned long last_time = 0;
  unsigned long last_time_dir = 0;

  float parcial_period;
  long pos_motor=0;
  long last_pos_motor = 0;
  int direction = 1;
  float pos_fio = diametro_fio;
  int j=0;
  bool ativa = 1;
  unsigned long tempo; 
  float erro = 0;
  unsigned long tempos_desloc[100], inter_t_desloc[100];
  int k=0, l=0;
  unsigned long tp_1; 
  bool hab_esq=0, hab_dir=1;
  int vel_motor_rpm = 0;

  int step_motor_steps = calculateStepsPerStep(diametro_fio);

  while (turns<total_turns){
    
    state = digitalRead(pinPause);
    if (state != last_state && (millis() - last_time) > 50){
      last_time=millis();
      if (state){
        pause=!pause;
      }
      last_state = state;  
    }

    state_direction = digitalRead(pinChangeDir);
    if (state_direction != last_state_direction && (millis() - last_time_dir) > 50){
      last_time_dir=millis();
      if(state_direction){
        direction = -direction;
      }
      last_state_direction = state_direction;
    }

    if (!pause){

      // ✅ leitura via biblioteca Encoder
      pos_motor_encoder = myEnc.read();
      pos_motor = pos_motor_encoder;

      if (pos_motor - last_pos_motor >= steps_to_run){
        turns = pos_motor / motor_step_cte;
        last_pos_motor = pos_motor;
        Serial.println(turns);
        //Serial.println(pos_motor);
        move(step_motor_steps * direction);   
      }

      velMotor = map(analogRead(pot),0,1023,0,255);
      analogWrite(pinPWM, velMotor);

    }
    else{
      analogWrite(pinPWM, 255);
      digitalWrite(PpinEnable, LOW);
    }
  }

  analogWrite(pinPWM, 255);
  digitalWrite(PpinEnable, LOW);
}

void setup() {
  pinMode(sensor_esq, INPUT);
  pinMode(sensor_dir, INPUT);
  pinMode(Ppin1, OUTPUT);
  pinMode(Ppin2, OUTPUT);
  pinMode(Ppin3, OUTPUT);
  pinMode(Ppin4, OUTPUT);

  // ✅ CORRIGIDO
  pinMode(PpinEnable, OUTPUT);

  pinMode(pinPWM, OUTPUT);
  pinMode(pinPause, INPUT_PULLUP);
  pinMode(pinChangeDir, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  Serial.println("entrou");
  run(10000, 1.12);
}