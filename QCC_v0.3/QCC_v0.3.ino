// File: QCC_v.01 
// Created on: 02/06/2016
//   Author: Sergio Vigorra Treviño
//   e-mail: svigorra@gmail.com 
//   Description: QuadCopterController
//   Version: v0.3

// Parámetros del controlador
  // Roll
  #define Roll_P_Gain             3
  #define Roll_I_Gain             0.002
  #define Roll_D_Gain             0
  #define Roll_I_Max_Correction   700
  #define PID_Roll_Output_Max     700
  
  // Pitch
  #define Pitch_P_Gain            3
  #define Pitch_I_Gain            0.002
  #define Pitch_D_Gain            0
  #define Pitch_I_Max_Correction  700
  #define PID_Pitch_Output_Max    700
  
  // Yaw
  #define Yaw_P_Gain              1
  #define Yaw_I_Gain              0
  #define Yaw_D_Gain              0
  #define Yaw_I_Max_Correction    0
  #define PID_Yaw_Output_Max      200

// Pines de entrada
#define PPM_Input_Pin 2

// Pines de salida
#define Motor_1_Pin 4
#define Motor_2_Pin 5
#define Motor_3_Pin 6
#define Motor_4_Pin 7

// Parámetros del software
#define serial_baudrate 115200      // Velocidad del puerto serie

// Variables auxiliares del controlador
float Roll_Error;
float Pitch_Error;
float Yaw_Error;
float Roll_I_Correction;
float Pitch_I_Correction;
float Yaw_I_Correction;
float PID_Roll_Output;
float PID_Pitch_Output;
float PID_Yaw_Output;
float Last_Roll_Error;
float Last_Pitch_Error;
float Last_Yaw_Error;

// Variables globales
volatile int overflows = 0;
volatile float pulse_time = 0;
volatile int channel = 0;
volatile float Channel_1_Raw;
volatile float Channel_2_Raw;
volatile float Channel_3_Raw;
volatile float Channel_4_Raw;
volatile float Channel_5_Raw;
volatile float filter_time = 275;

unsigned long loop_timer = 0;
unsigned long ESC_1_High_Timer, ESC_2_High_Timer, ESC_3_High_Timer, ESC_4_High_Timer;
unsigned long ESC_1_Pulse_Time;
unsigned long ESC_2_Pulse_Time;
unsigned long ESC_3_Pulse_Time;
unsigned long ESC_4_Pulse_Time;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long Zero_Timer, timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long prev_time = 0;

int Channel_1_Calibrated_prev = 1500;
int Channel_2_Calibrated_prev = 1500;
int Channel_3_Calibrated_prev;
int Channel_4_Calibrated_prev = 1500;

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

int ESC_Signal_1;
int ESC_Signal_2;
int ESC_Signal_3;
int ESC_Signal_4;

float Max_Roll_Angle  = 30;
float Max_Pitch_Angle = 30;
float Max_Yaw_Rate    = 150;

int Channel_1_Calibrated;
int Channel_2_Calibrated;
int Channel_3_Calibrated;
int Channel_4_Calibrated;
int Channel_5_Calibrated;

float Desired_Roll_Angle;
float Desired_Pitch_Angle;
float Desired_Throttle;
float Desired_Yaw_Rate;
float IMU_Pitch_Angle = 0;
float IMU_Roll_Angle  = 0;
float IMU_Yaw_Rate    = 0;
float Gy_X_Cal = 0;
float Gy_Y_Cal = 0;
float Gy_Z_Cal = 0;

// Declaraciones para el MPU6050
#include <Wire.h>

//Direccion I2C de la IMU
#define MPU 0x68

//Ratios de conversion
#define A_R 16384.0
#define G_R 131.0

//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779

//MPU-6050 da los valores en enteros de 16 bits
//Valores sin refinar
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, Tmp;
//Angulos
float Acc[2];
float Gy[3];

void setup() {
  pinMode(13,OUTPUT);                                                           // LED Pin 13 como salida
  Serial.begin(serial_baudrate);                                                // Inicia periferico UART
  // Timer 1
  TCCR1A = 0;                                                                   // Timer1 parado (16 bits)
  TCCR1B = 0;                                                                   // Timer1 parado
  TCNT1  = 0;                                                                   // Inicializa Timer1 a cero
  TIMSK1 = 0B00000010;                                                          // Habilitada interruptción de comparación 
  OCR1A  = 65535;      // FULL                                                  // Valor de comparación
  attachInterrupt(digitalPinToInterrupt(PPM_Input_Pin), calcSignal, CHANGE);    // Configurar interrupción por cambio de estado en pin
  // I2C
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  // Conf. Pines de motores como salidas
  pinMode(Motor_1_Pin, OUTPUT);
  pinMode(Motor_2_Pin, OUTPUT);
  pinMode(Motor_3_Pin, OUTPUT);
  pinMode(Motor_4_Pin, OUTPUT);
  
  // Calibración de los giróscopos
  digitalWrite(13,LOW);
  for (int i=0;i<2000;i++) {
    Serial.print("Calibrando....");
    Serial.println(i);
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);                // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);   // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    // Calculo del angulo del Giroscopio
    Gy[0] = GyX/G_R;
    Gy[1] = GyY/G_R;
    Gy[2] = GyZ/G_R;
    Gy_X_Cal = Gy_X_Cal + Gy[0];
    Gy_Y_Cal = Gy_Y_Cal + Gy[1];
    Gy_Z_Cal = Gy_Z_Cal + Gy[2];
  }
  Gy_X_Cal = Gy_X_Cal/2000;
  Gy_Y_Cal = Gy_Y_Cal/2000;
  Gy_Z_Cal = Gy_Z_Cal/2000;
  digitalWrite(13,HIGH);
  Serial.println("Calibrado"); 
  Zero_Timer = micros();                             

  Channel_3_Calibrated = 1000;
  Channel_3_Calibrated_prev = 1000;
  interrupts();                       // Habilitar interrupciones
}

void loop() {
  // Lectura de los canales -- Ya tenemos los valores en las variables Channel_X_Raw gracias a la ISR 
  // Colocar lectura de canales en rango 1000-2000us
  Channel_1_Calibrated = Channel_Calibrator(Channel_1_Raw, Channel_1_Lower_Limit, Channel_1_Upper_Limit);
  Channel_2_Calibrated = Channel_Calibrator(Channel_2_Raw, Channel_2_Lower_Limit, Channel_2_Upper_Limit);
  Channel_3_Calibrated = Channel_Calibrator(Channel_3_Raw, Channel_3_Lower_Limit, Channel_3_Upper_Limit);  
  Channel_4_Calibrated = Channel_Calibrator(Channel_4_Raw, Channel_4_Lower_Limit, Channel_4_Upper_Limit);
  Channel_5_Calibrated = Channel_Calibrator(Channel_5_Raw, Channel_5_Lower_Limit, Channel_5_Upper_Limit);

  // Filtro Canal 1
  if (((Channel_1_Calibrated - Channel_1_Calibrated_prev) > filter_time) || ((Channel_1_Calibrated - Channel_1_Calibrated_prev) < ((-1) * filter_time))) {
    Channel_1_Calibrated = Channel_1_Calibrated_prev;
  }
  Channel_1_Calibrated_prev = Channel_1_Calibrated;

  // Filtro Canal 2
  if (((Channel_2_Calibrated - Channel_2_Calibrated_prev) > filter_time) || ((Channel_2_Calibrated - Channel_2_Calibrated_prev) < ((-1) * filter_time))) {
    Channel_2_Calibrated = Channel_2_Calibrated_prev;
  }
  Channel_2_Calibrated_prev = Channel_2_Calibrated;

  // Filtro Canal 3
  if (((Channel_3_Calibrated - Channel_3_Calibrated_prev) > filter_time) || ((Channel_3_Calibrated - Channel_3_Calibrated_prev) < ((-1) * filter_time))) {
    Channel_3_Calibrated = Channel_3_Calibrated_prev;
  }
  Channel_3_Calibrated_prev = Channel_3_Calibrated;

  // Filtro Canal 4
  if (((Channel_4_Calibrated - Channel_4_Calibrated_prev) > filter_time) || ((Channel_4_Calibrated - Channel_4_Calibrated_prev) < ((-1) * filter_time))) {
    Channel_4_Calibrated = Channel_4_Calibrated_prev;
  }
  Channel_4_Calibrated_prev = Channel_4_Calibrated;

  // Cálculo de los angulos consigna y posición de throttle
  Desired_Roll_Angle  = Desired_Roll_Pitch_Angle_Calculator(Channel_1_Calibrated, Max_Roll_Angle);
  Desired_Pitch_Angle = Desired_Roll_Pitch_Angle_Calculator(Channel_2_Calibrated, Max_Pitch_Angle);
  Desired_Yaw_Rate    = Desired_Yaw_Rate_Calculator(Channel_4_Calibrated, Max_Yaw_Rate);
  Desired_Throttle    = Channel_3_Calibrated; 
  
  // Lectura de los datos de la IMU -- Aceleraciones y Rotaciones
  if ((millis() - prev_time) > 10) {
    prev_time = millis();
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);   // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    // A partir de los valores del acelerometro, se calculan los angulos Y, X
    // respectivamente, con la formula de la tangente.
    Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
    Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
    // Calculo del angulo del Giroscopio
    Gy[0] = GyX/G_R - Gy_X_Cal;
    Gy[1] = GyY/G_R - Gy_Y_Cal;
    Gy[2] = GyZ/G_R - Gy_Z_Cal; // Actualzamos con los offsets
    // Aplicar el Filtro Complementario

    IMU_Pitch_Angle = 0.98 *(IMU_Pitch_Angle + Gy[0]*0.010) + 0.02*Acc[0];
    IMU_Roll_Angle  = 0.98 *(IMU_Roll_Angle  + Gy[1]*0.010) + 0.02*Acc[1];
    IMU_Yaw_Rate    = Gy[2];
  }
  
  // Control PID
  
  Serial.print(IMU_Roll_Angle);
  Serial.print("\t");
  Serial.println(IMU_Pitch_Angle);
  
  // Control en Roll
  Roll_Error = Desired_Roll_Angle - IMU_Roll_Angle;
  Roll_I_Correction = Roll_I_Correction + Roll_I_Gain * Roll_Error;
  if (Roll_I_Correction > Roll_I_Max_Correction) {
    Roll_I_Correction = Roll_I_Max_Correction;
  }  
  if (Roll_I_Correction < Roll_I_Max_Correction*(-1)) {
    Roll_I_Correction = Roll_I_Correction * (-1);
  }  
  PID_Roll_Output = Roll_P_Gain * Roll_Error + Roll_I_Correction + Roll_D_Gain * (Roll_Error - Last_Roll_Error);
  if (PID_Roll_Output > PID_Roll_Output_Max) { 
    PID_Roll_Output = PID_Roll_Output_Max;
  }
  if (PID_Roll_Output < PID_Roll_Output_Max*(-1)) { 
    PID_Roll_Output = PID_Roll_Output_Max*(-1);
  } 
  Last_Roll_Error = Roll_Error;
  
  // Control en Pitch
  Pitch_Error = Desired_Pitch_Angle - IMU_Pitch_Angle;
  Pitch_I_Correction = Pitch_I_Correction + Pitch_I_Gain * Pitch_Error;
  if (Pitch_I_Correction > Pitch_I_Max_Correction) {
    Pitch_I_Correction = Pitch_I_Max_Correction;
  }  
  if (Pitch_I_Correction < Pitch_I_Max_Correction*(-1)) {
    Pitch_I_Correction = Pitch_I_Correction * (-1);
  }  
  PID_Pitch_Output = Pitch_P_Gain * Pitch_Error + Pitch_I_Correction + Pitch_D_Gain * (Pitch_Error - Last_Pitch_Error);
  if (PID_Pitch_Output > PID_Pitch_Output_Max) { 
    PID_Pitch_Output = PID_Pitch_Output_Max;
  }
  if (PID_Pitch_Output < PID_Pitch_Output_Max*(-1)) { 
    PID_Pitch_Output = PID_Pitch_Output_Max*(-1);
  } 
  Last_Pitch_Error = Pitch_Error;

  // Control en Yaw
  Yaw_Error = Desired_Yaw_Rate - IMU_Yaw_Rate;
  Yaw_I_Correction = Yaw_I_Correction + Yaw_I_Gain * Yaw_Error;
  if (Yaw_I_Correction > Yaw_I_Max_Correction) {
    Yaw_I_Correction = Yaw_I_Max_Correction;
  }  
  if (Yaw_I_Correction < Yaw_I_Max_Correction*(-1)) {
    Yaw_I_Correction = Yaw_I_Correction * (-1);
  }  
  PID_Yaw_Output = Yaw_P_Gain * Yaw_Error + Yaw_I_Correction + Yaw_D_Gain * (Yaw_Error - Last_Yaw_Error);
  if (PID_Yaw_Output > PID_Yaw_Output_Max) { 
    PID_Yaw_Output = PID_Yaw_Output_Max;
  }
  if (PID_Yaw_Output < PID_Yaw_Output_Max*(-1)) { 
    PID_Yaw_Output = PID_Yaw_Output_Max*(-1);
  } 
  Last_Yaw_Error = Yaw_Error;

  // Señales a los motores
  if (Desired_Throttle > 1800) {
    Desired_Throttle = 1800;
  }

  // Armado
  if (Desired_Throttle < 1100) {
    ESC_Signal_1 = 1000;
    ESC_Signal_2 = 1000;
    ESC_Signal_3 = 1000;
    ESC_Signal_4 = 1000;
  }

  else { 
    ESC_Signal_1 = Desired_Throttle - PID_Roll_Output + PID_Pitch_Output - PID_Yaw_Output;
    ESC_Signal_2 = Desired_Throttle - PID_Roll_Output - PID_Pitch_Output + PID_Yaw_Output;
    ESC_Signal_3 = Desired_Throttle + PID_Roll_Output - PID_Pitch_Output - PID_Yaw_Output;
    ESC_Signal_4 = Desired_Throttle + PID_Roll_Output + PID_Pitch_Output + PID_Yaw_Output;

    // Nos aseguramos de que las señales quedan en el rango adecuado
    if (ESC_Signal_1 < 1100) {
      ESC_Signal_1 = 1100;
    }
    else if (ESC_Signal_1 > 2000) {
      ESC_Signal_1 = 2000;
    }
  
    if (ESC_Signal_2 < 1100) {
      ESC_Signal_2 = 1100;
    }
    else if (ESC_Signal_2 > 2000) {
      ESC_Signal_2 = 2000;
    }
  
    if (ESC_Signal_3 < 1100) {
      ESC_Signal_3 = 1100;
    }
    else if (ESC_Signal_3 > 2000) {
      ESC_Signal_3 = 2000;
    }
  
    if (ESC_Signal_4 < 1100) {
      ESC_Signal_4 = 1100;
    }
    else if (ESC_Signal_4 > 2000) {
      ESC_Signal_4 = 2000;
    }
  }

  // Generación señal PWM a los motores
  if ((Zero_Timer + 4000) <= micros()){ 
    Zero_Timer = micros();                                     
    PORTD |= B11110000;                                        
    ESC_1_High_Timer = ESC_Signal_1 + Zero_Timer; 
    ESC_2_High_Timer = ESC_Signal_2 + Zero_Timer; 
    ESC_3_High_Timer = ESC_Signal_3 + Zero_Timer; 
    ESC_4_High_Timer = ESC_Signal_4 + Zero_Timer; 
  
    while(PORTD >= 16){                                         
      esc_loop_timer = micros();                                
      if(ESC_1_High_Timer <= esc_loop_timer)PORTD &= B11101111; 
      if(ESC_2_High_Timer <= esc_loop_timer)PORTD &= B11011111; 
      if(ESC_3_High_Timer <= esc_loop_timer)PORTD &= B10111111; 
      if(ESC_4_High_Timer <= esc_loop_timer)PORTD &= B01111111; 
    }
  }
}



void calcSignal() {   
  noInterrupts();
  if(digitalRead(PPM_Input_Pin) == HIGH) {   // Si la transición es a HIGH, hay que empezar a contar.
    TCNT1=0;                                 // Registro del contador puesto a 0
    overflows=0;                             // Contador de overflows a 0
    TCCR1B=0B00001001;                       // Timer 1 en modo CTC sin preescaler           
  }
  
  if(digitalRead(PPM_Input_Pin) == LOW )     {                               // Si la transición es a LOW, hay que parar de contar
    TCCR1B = 0;                                                              // Parar Timer1
    pulse_time = round(0.0625*(overflows * 65535 + (unsigned long)TCNT1));   // Cálculo de la duración del pulso (pulse_time)
        
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

int Channel_Calibrator(float Channel_X_Raw, int Channel_X_Lower_Limit, int Channel_X_Upper_Limit) {
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

float Desired_Roll_Pitch_Angle_Calculator(int Channel_X_Calibrated, float Max_Angle) {
  float Angle;
  Angle = (Channel_X_Calibrated - 1000) * (Max_Angle+Max_Angle) / 1000 - Max_Angle;
  if (Angle < (-Max_Angle)) {
    Angle = -Max_Angle;
  }
  if (Angle > (Max_Angle)) {
    Angle = Max_Angle;  
  }
  return Angle;
}

float Desired_Yaw_Rate_Calculator(int Channel_4_Calibrated, float Max_Yaw_Rate) {
  float Rate;
  Rate = (Channel_4_Calibrated - 1000) * (Max_Yaw_Rate + Max_Yaw_Rate) / 1000 - Max_Yaw_Rate;
  if (Rate < (-Max_Yaw_Rate)) {
    Rate = -Max_Yaw_Rate;
  }
  if (Rate > (Max_Yaw_Rate)) {
    Rate = Max_Yaw_Rate;  
  }
  return Rate;
}

