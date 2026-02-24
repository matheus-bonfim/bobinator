#include <Stepper.h>
#include <Encoder.h>


#define Ppin1  6// pins of stepmotor 
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

const int stepfuso = 8; //medida em milimetro
const int stepsPerRevolution = 200;  
const int velMotor_Passo = 300; 
const long pos_per_turn = 24;
volatile int pos_motor_encoder = 0;  // Variável para armazenar a posição do encoder
volatile int ultimaA = LOW;  // Armazena o último estado do pino A
const int period_sample = 4; // representa o inverso da porcentagem do periodo da rotacao do motor para estimar a vel absoluta

Stepper myStepper(stepsPerRevolution, Ppin1, Ppin2, Ppin3, Ppin4);



void move(float diameter, int vel_ang_motor){
  int steps;
  myStepper.setSpeed(vel_ang_motor);
  unsigned long tp3 = micros();
  steps = round(diameter / ((1.8/360)*stepfuso)); 
  digitalWrite(PpinEnable, HIGH); 
  myStepper.step(steps);
  digitalWrite(PpinEnable, LOW); 
}

void encoderRotacao() {
  int estadoA = digitalRead(2);  // Lê o estado atual do pino A
  int estadoB = digitalRead(3);  // Lê o estado atual do pino B

  // Verifica a direção de rotação
  if (estadoA != ultimaA) {
    if (estadoA == HIGH) {
      // Se B estiver LOW, estamos girando no direction horário
      if (estadoB == LOW) {
        pos_motor_encoder--;
      } else {
        pos_motor_encoder++;
      }
    } else {
      // Se B estiver HIGH, estamos girando no direction anti-horário
      if (estadoB == HIGH) {
        pos_motor_encoder--;
      } else {
        pos_motor_encoder++;
      }
    }
    ultimaA = estadoA;  // Atualiza o último estado de A
  }
}
void run(float total_turns, float diametro_fio){
  long oldMpos = 0;
  int velMotor = map(analogRead(pot),0,1023,0,255);
  unsigned long motor_period = 0;
  int turns = 0;
  int last_turns = 0;
  bool key = 1;
  bool pause = 1;
  bool state = 0;
  bool state_direction = 0;
  bool last_state = 0;
  bool last_state_direction = 0;
  unsigned long last_time = 0;
  unsigned long last_time_dir = 0;
  unsigned long oldtime = 0;
  unsigned long last_time_enc = 0;
  unsigned long parcial_time = 0;
  float parcial_period;
  long pos_motor=0;
  long last_pos_motor = 0;
  int direction = 1; // comeca girando anti horario empurrando carrinho p direita
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

  tempos_desloc[0] = 0;
  
  while (turns<total_turns){
    
    state = digitalRead(pinPause);
    if (state != last_state && (millis() - last_time) > 50){
      last_time=millis();
      if (state){
        pause=!pause;
        parcial_time = millis();
      }
      
      last_state = state;  
    }
    //Serial.println(pause);

    state_direction = digitalRead(pinChangeDir);
    if (state_direction != last_state_direction && (millis() - last_time_dir) > 50){
      last_time_dir=millis();
      if(state_direction){
        direction = -direction;
      }
      last_state_direction = state_direction;
    }

    
    if (!pause){
      Serial.println("tempo no inicio");
      Serial.println(parcial_period);
      
      pos_motor = pos_motor_encoder;
      if (pos_motor != 0){
        turns = pos_motor / pos_per_turn;  
      }
      //Serial.println(turns);

      if (pos_motor - last_pos_motor > pos_per_turn/period_sample){
        last_pos_motor = pos_motor;
        
        parcial_period = millis() - parcial_time;
        parcial_time = millis();
        Serial.println("parcial Period");
        Serial.println(parcial_period);


        vel_motor_rpm = round(1 / (parcial_period * period_sample / 1000) * 60);
        Serial.println("motor rpm");
        Serial.println(vel_motor_rpm);
      }

      if (turns != last_turns){   // uma volta completa
        last_turns = turns;
        Serial.println("entrou aqui");
        if(vel_motor_rpm > 0){
          move(diametro_fio * direction, vel_motor_rpm);
        }
        
        Serial.println("passou aqui");

      }

      velMotor = map(analogRead(pot),0,1023,0,255); //liga o motor
      //Serial.println(velMotor);
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
  pinMode(PpinEnable, INPUT);
  pinMode(pinPWM, OUTPUT);
  pinMode(pinPause, INPUT_PULLUP);
  pinMode(pinChangeDir, INPUT_PULLUP);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(2), encoderRotacao, CHANGE);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("entrou");
  run(381, 0.6);
 
}
