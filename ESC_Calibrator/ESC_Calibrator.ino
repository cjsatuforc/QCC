// File: ESC_Calibrator 
// Created on: 02/06/2016
//   Author: Sergio Vigorra Treviño
//   e-mail: svigorra@gmail.com 
//   Description: programa para generar la señal PWM máxima y mínima del control de Throttle, para poder
//                calibrar los reguladores.
//   Version: v0.3

int ESC_Signal_1;
int ESC_Signal_2;
int ESC_Signal_3;
int ESC_Signal_4;

unsigned long Motor_Controller_Timer;
unsigned long ESC_Timer;
// Pines de entrada
#define PPM_Input_Pin 2

int a = 0;
int b = 0;
int c = 0;
int d = 0; 


//// Pines de salida
#define Motor_1_Pin 4
#define Motor_2_Pin 5
#define Motor_3_Pin 6
#define Motor_4_Pin 7

unsigned long prev_time = 0;

// Parámetros del software
#define serial_baudrate 115200      // Velocidad del puerto serie

// Variables globales
volatile int overflows = 0;
volatile float pulse_time =0;

volatile int channel=0;


int Channel_3_Lower_Limit = 787;

int Channel_3_Upper_Limit = 1387; 

float Channel_1_Raw;
float Channel_2_Raw;
float Channel_3_Raw;
float Channel_4_Raw;
float Channel_5_Raw;


int Channel_3_Calibrated;



unsigned long ESC_1_Pulse_Time;
unsigned long ESC_2_Pulse_Time;
unsigned long ESC_3_Pulse_Time;
unsigned long ESC_4_Pulse_Time;


void setup() {
  Serial.begin(serial_baudrate);                                                // Inicia periferico UART
  TCCR1A = 0;                                                                   // Timer1 parado (16 bits)
  TCCR1B = 0;                                                                   // Timer1 parado
  TCNT1  = 0;                                                                   // Inicializa Timer1 a cero
  TIMSK1 = 0B00000010;                                                          // Habilitada interruptción de comparación 
  OCR1A  = 65535;      // FULL                                                  // Valor de comparación
  attachInterrupt(digitalPinToInterrupt(PPM_Input_Pin), calcSignal, CHANGE);    // Configurar interrupción por cambio de estado en pin
  pinMode(Motor_1_Pin, OUTPUT);
  pinMode(Motor_2_Pin, OUTPUT);
  pinMode(Motor_3_Pin, OUTPUT);
  pinMode(Motor_4_Pin, OUTPUT);
  interrupts();
}

void loop() {

  Channel_3_Calibrated = Channel_Calibrator(Channel_3_Raw, Channel_3_Lower_Limit, Channel_3_Upper_Limit);  
  ESC_Signal_1 = Channel_3_Calibrated;
  ESC_Signal_2 = Channel_3_Calibrated;
  ESC_Signal_3 = Channel_3_Calibrated;
  ESC_Signal_4 = Channel_3_Calibrated;

  if (micros() - Motor_Controller_Timer >= 10000) { // Entramos en el condicional si han pasado los 4 ms entre pulsos
    Motor_Controller_Timer = micros();
    digitalWrite(Motor_1_Pin,HIGH);
    digitalWrite(Motor_2_Pin,HIGH);
    digitalWrite(Motor_3_Pin,HIGH);
    digitalWrite(Motor_4_Pin,HIGH);
  
    ESC_1_Pulse_Time = ESC_Signal_1 + Motor_Controller_Timer;       
                                  
    ESC_2_Pulse_Time = ESC_Signal_2 + Motor_Controller_Timer;              
    ESC_3_Pulse_Time = ESC_Signal_3 + Motor_Controller_Timer;              
    ESC_4_Pulse_Time = ESC_Signal_4 + Motor_Controller_Timer;                                     
    
    while( (a + b + c +d) < 4) {
      ESC_Timer = micros();
      if (ESC_Timer >= ESC_1_Pulse_Time) {
        digitalWrite(Motor_1_Pin,LOW);
        a = 1;
      }
      if (ESC_Timer >= ESC_2_Pulse_Time) {
        digitalWrite(Motor_2_Pin,LOW);
        b = 1;
      }
      if (ESC_Timer >= ESC_3_Pulse_Time) {
        digitalWrite(Motor_3_Pin,LOW);
        c = 1;
      }
      if (ESC_Timer >= ESC_4_Pulse_Time) {
        digitalWrite(Motor_4_Pin,LOW);
        d = 1;
      }
     
    }
     a = 0; 
      b = 0;
      c = 0;
      d = 0;
  }

  Serial.println(ESC_Signal_1);
  Serial.print("\t");
  Serial.print(ESC_Signal_2);
  Serial.print("\t");
  Serial.print(ESC_Signal_3);
  Serial.print("\t");
  Serial.print(ESC_Signal_4);
  Serial.println("\t");
  
    

}



void calcSignal() {   
  noInterrupts();
  if(digitalRead(PPM_Input_Pin) == HIGH) {  // Si la transición es a HIGH, hay que empezar a contar.
    TCNT1=0;                                // Registro del contador puesto a 0
    overflows=0;                             // Contador de overflows a 0
    TCCR1B=0B00001001;                       // Timer 1 en modo CTC sin preescaler           
  }
  
  if(digitalRead(PPM_Input_Pin) == LOW )     {                               // Si la transición es a LOW, hay que parar de contar
    TCCR1B = 0;                                                        // Parar Timer1
    pulse_time = round(0.0625*(overflows * 65535 + (unsigned long)TCNT1));             // Cálculo de la duración del pulso (pulse_time)
        
    if (pulse_time >= 3000){   // 3000
        channel = 0;
      } 
      else {
        switch (channel) {
          case 0:
            Channel_1_Raw = pulse_time;
            break;  
          case 1:
            Channel_2_Raw = pulse_time;
            break;  
          case 2:
            Channel_3_Raw = pulse_time;
            break;  
          case 3:
            Channel_4_Raw = pulse_time;
            break;  
          case 4:
            Channel_5_Raw = pulse_time;
            break;  
        }
        channel++;
      }
      interrupts();
        
  }
}

ISR(TIMER1_COMPA_vect) {
  noInterrupts();
  overflows++;           // Contador de desbordamientos 
  interrupts();
}

int Channel_Calibrator(float Channel_X_Raw, int Channel_X_Lower_Limit,int Channel_X_Upper_Limit) {
  float x;
  int b;
  x = 1000*(Channel_X_Raw - Channel_X_Lower_Limit)/(Channel_X_Upper_Limit - Channel_X_Lower_Limit) + 1000;
  b = round(x);
  if (b < 1000) {
    b = 1000;
  }
  if (b > 2000) {
    b = 2000;
  }
  return (b);
}















