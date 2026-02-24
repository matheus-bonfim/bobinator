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
volatile int posicao = 0;  // Variável para armazenar a posição do encoder
volatile int ultimaA = LOW;  // Armazena o último estado do pino A

Stepper myStepper(stepsPerRevolution, Ppin1, Ppin2, Ppin3, Ppin4);


void move(float diameter, int velPasso){
  int steps;
  myStepper.setSpeed(velPasso);
  unsigned long tp3 = micros();
  steps = round(diameter / ((1.8/360)*stepfuso));  
  myStepper.step(steps);
}

void encoderRotacao() {
  int estadoA = digitalRead(2);  // Lê o estado atual do pino A
  int estadoB = digitalRead(3);  // Lê o estado atual do pino B

  // Verifica a direção de rotação
  if (estadoA != ultimaA) {
    if (estadoA == HIGH) {
      // Se B estiver LOW, estamos girando no direction horário
      if (estadoB == LOW) {
        posicao--;
      } else {
        posicao++;
      }
    } else {
      // Se B estiver HIGH, estamos girando no direction anti-horário
      if (estadoB == HIGH) {
        posicao--;
      } else {
        posicao++;
      }
    }
    ultimaA = estadoA;  // Atualiza o último estado de A
  }
}
void run(float voltas_total, float diametro_fio){
  long oldMpos = 0;
  int velMotor = map(analogRead(pot),0,1023,0,255);
  float voltas = 0;
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
  long newMpos=0;
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

  tempos_desloc[0] = 0;
  
  while (voltas<voltas_total){
    
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
      
      newMpos = posicao;
      if (newMpos==0) tempo = millis();
      
      
      if (newMpos != oldMpos){
        
        if ((newMpos - oldMpos)>0){
          voltas = voltas + 0.0416666666666667;
          
        }
        else{
          voltas = voltas - 0.0416666666666667;
        }
        oldMpos = newMpos;
        /*if (abs(voltas - round(voltas))<0.001){
          Serial.println("voltas:");
          Serial.println(voltas);
        }
        
       */
       Serial.println(voltas);
        //Serial.println(oldMpos);
          
      }
      
      
      
      if (newMpos >= abs(pos_per_turn + 1)){
        //Serial.println(tempo);
        
        //Serial.println(voltas);
        posicao = 0;
        newMpos = 0;
        ativa = 1;
         
        
        // comando motor de passo
        
        
          
      }
      
      
      
      if ((newMpos >= 6) && (voltas<voltas_total - 1) && ativa){
        
        digitalWrite(PpinEnable, HIGH); 
        Serial.println("Direction");
        Serial.println(direction);
        move(diametro_fio*direction, velMotor_Passo);
        digitalWrite(PpinEnable, LOW);
        pos_fio = pos_fio + diametro_fio;
        tempos_desloc[k] = millis();
        
        if(l==16){
          digitalWrite(PpinEnable, HIGH);    /// correcao a cada 16 voltas?
          move((diametro_fio/2)*direction, velMotor_Passo);
          digitalWrite(PpinEnable, LOW);
          l=0;
          
        }
        k++;
        l++;
        ativa = 0;
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
  //for(int i=1;i<k;i++){
   // Serial.println(tempos_desloc[i] - tempos_desloc[i-1]);
  //}
  
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
