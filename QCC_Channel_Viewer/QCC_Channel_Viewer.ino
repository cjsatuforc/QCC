// File: QCC_v.01 
// Created on: 02/06/2016
//   Author: Sergio Vigorra Treviño
//   e-mail: svigorra@gmail.com 
//   Description: QuadCopterController
//   Version: v0.1

int Channel_1_Lower_Limit = 718;
int Channel_2_Lower_Limit = 714;
int Channel_3_Lower_Limit = 787;
int Channel_4_Lower_Limit = 723;
int Channel_5_Lower_Limit = 714;

int Channel_1_Upper_Limit = 1462;
int Channel_2_Upper_Limit = 1477;
int Channel_3_Upper_Limit = 1387; 
int Channel_4_Upper_Limit = 1477;
int Channel_5_Upper_Limit = 1477;

int Channel_1_Calibrated;
int Channel_2_Calibrated;
int Channel_3_Calibrated;
int Channel_4_Calibrated;
// Pines de entrada

#define PPM_Input_Pin 2

//// Pines de salida
//#define Motor_1_Pin 1
//#define Motor_2_Pin 1
//#define Motor_3_Pin 1
//#define Motor_4_Pin 1

unsigned long prev_time = 0;
 
 

// Parámetros del software
#define serial_baudrate 115200      // Velocidad del puerto serie

// Variables globales
volatile int overflows = 0;
volatile float pulse_time =0;

volatile int channel=0;

float Channel_1_Raw;
float Channel_2_Raw;
float Channel_3_Raw;
float Channel_4_Raw;
float Channel_5_Raw;





  

void setup() {
  
  //noInterrupts();                                                               // Deshabilita las interrupciones
  Serial.begin(serial_baudrate);                                                // Inicia periferico UART
  Serial.println("LLEGO");
  TCCR1A = 0;                                                                   // Timer1 parado (16 bits)
  TCCR1B = 0;                                                                   // Timer1 parado
  TCNT1  = 0;                                                                   // Inicializa Timer1 a cero
  TIMSK1 = 0B00000010;                                                          // Habilitada interruptción de comparación 
  OCR1A  = 65535;      // FULL                                                  // Valor de comparación
  attachInterrupt(digitalPinToInterrupt(PPM_Input_Pin), calcSignal, CHANGE);    // Configurar interrupción por cambio de estado en pin
  interrupts();
}

void loop() {
  
Channel_1_Calibrated = Channel_Calibrator(Channel_1_Raw, Channel_1_Lower_Limit, Channel_1_Upper_Limit);  
Channel_2_Calibrated = Channel_Calibrator(Channel_2_Raw, Channel_2_Lower_Limit, Channel_2_Upper_Limit);  
Channel_3_Calibrated = Channel_Calibrator(Channel_3_Raw, Channel_3_Lower_Limit, Channel_3_Upper_Limit);  
Channel_4_Calibrated = Channel_Calibrator(Channel_4_Raw, Channel_4_Lower_Limit, Channel_4_Upper_Limit);  

Serial.print("Canal 1: ");
Serial.print(Channel_1_Calibrated);
Serial.print("\t");
Serial.print("Canal 2: ");
Serial.print(Channel_2_Calibrated);
Serial.print("\t");
Serial.print("Canal 3: ");
Serial.print(Channel_3_Calibrated);
Serial.print("\t");
Serial.print("Canal 4: ");
Serial.println(Channel_4_Calibrated);



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









