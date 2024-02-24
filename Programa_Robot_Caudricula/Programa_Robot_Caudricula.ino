// Programa para poner en bucle al robot de ALT+F4 en una cuadricula
// Cortesía del equipo ALT+F4
// © AFM 2024


// ----------------------------------------------------------------
//                         CONFIGURACIÓN
// ----------------------------------------------------------------


// Sensores infrarrojos de linea (adelante)
const byte pin_IR_INI = A1;
const byte pin_IR_INE = A2;
const byte pin_IR_R = A3;
const byte pin_IR_L = A4;
const byte pin_IR_LE = A5;
const byte pin_IR_LI = A6;

// Sensores infrarrojos de linea (atrás)
const byte pin_IRS_LC = A11;
const byte pin_IRS_LL = A10;
const byte pin_IRS_FC = A14;
const byte pin_IRS_FL = A15;
const byte pin_IRS_RC = A12;
const byte pin_IRS_RL = A13;

// Sensores infrarrojos de distancia
const byte pin_IRD_LEFT__P = 15;
const byte pin_IRD_LEFT__D = 14;
const byte pin_IRD_FRONT_P = 19;
const byte pin_IRD_FRONT_D = 18;
const byte pin_IRD_RIGHT_P = 16;
const byte pin_IRD_RIGHT_D = 17;

// Motores
const byte pin_R_A = 11;
const byte pin_R_B = 10;
const byte pin_L_A = 9;
const byte pin_L_B = 8;

// Resto
const byte pin_led = 14;
const byte pin_zumbador = A0;
const byte pin_boton_motores = 12;


/* CONFIGURACIÓN DE LA APP PARA CUADRICULA : */
#define TAMANO_VECTOR 16
char vector_cuadricula_init[TAMANO_VECTOR] = {'u', 'w', 'v', 'u', 'u', 'v', 'v', 'v', 'w', 'u', 'u', 't', 't', 't', 't', 't'};
char vector_cuadricula_1oop[TAMANO_VECTOR] = {'w', 'w', 'w', 'v', 'u', 'v', 'v', 'v', 'w', 'u', 't', 't', 't', 't', 't', 't'};
const char AVANZAR_SALIDA         = 's';
const char CASILLA_BLANCO         = 't';
const char INTERSECCION_ADELANTE  = 'u';
const char INTERSECCION_DERECHA   = 'v';
const char INTERSECCION_IZQUIERDA = 'w';
const char AVANZAR_META_Y_FIN_C   = 'x';
const char AVANZAR_META_Y_FIN_L   = 'y';
const char DETENER_CUADRICULA     = 'z';





// ----------------------------------------------------------------
//                          CÓDIGO
// ----------------------------------------------------------------

#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
bool blinkState = false;
float val_calibrado = 253;   // MPU definitivo D/9/MAY/2021


// Constantes cinemáticas
const float velocidad =  100.0 / 2.9 ;     // 93.0 / 4.25;
const float diametro = 11;
const float circunferencia;
const float rps;
const float distancia = 44.5;
const float grosor = 1.5;
const float anchorobot = 21;


//#define DELAY_GIRO_90                  ((3.1456 * anchorobot / 4) / velocidad) * 1000
#define DELAY_RECTO_ANTES_DEL_GIRO_90  1000*((3)/velocidad)
#define ANGULO_GIRO_MPU_90             90
#define DELAY_SALIDA                   1000*((26+11.5)/velocidad)
#define DELAY_LLEGADA_C                1000*((11.5)/velocidad)
#define DELAY_LLEGADA_L                1000*((13+11.5)/velocidad)
#define VEL_REL_GIRO_90_o              0.50  // En tanto por uno
#define VEL_REL_GIRO_90_f              0.3  // En tanto por uno
#define ANGULO_CAMBIO_VEL              60
#define VEL_REL_GIRO_IR                0.2  // En tanto por uno
#define TIEMPO_GIRO_IR                 2.5   // En ms
#define ANGULO_MPU_COMPRUEBA_IR        55


// Variables
bool IR_INI = 0;
bool IR_INE = 0;
bool IR_R = 0;
bool IR_L = 0;
bool IR_LE = 0;
bool IR_LI = 0;

char bt = 0;
float angulo = 0.0;
unsigned long m_o = millis();
bool escape = 0;



void setup() {
  SetPines();
  Enciende_MPU();

  delay(3000);
  for (int i = 0; i < TAMANO_VECTOR; i++) {
    movimiento(vector_cuadricula_init[i]);
  }
}

void loop() {

  for (int i = 0; i < TAMANO_VECTOR; i++) {
    movimiento(vector_cuadricula_1oop[i]);
  }

}



void movimiento(char modo) {
  if (modo == AVANZAR_SALIDA) {
    adelante();
    delay(DELAY_SALIDA);
  }
  if (modo == INTERSECCION_ADELANTE) {
    sigue_lineas_hasta_interseccion();
    if (escape == 0) {
      adelante();
      delay(2 * DELAY_RECTO_ANTES_DEL_GIRO_90);
    }
  }
  if (modo == INTERSECCION_DERECHA) {
    sigue_lineas_hasta_interseccion();
    if (escape == 0) {
      adelante();
      delay(DELAY_RECTO_ANTES_DEL_GIRO_90);
      Rota_MPU(ANGULO_GIRO_MPU_90);
      adelante();
    }
  }
  if (modo == INTERSECCION_IZQUIERDA) {
    sigue_lineas_hasta_interseccion();
    if (escape == 0) {
      adelante();
      delay(DELAY_RECTO_ANTES_DEL_GIRO_90);
      Rota_MPU(-ANGULO_GIRO_MPU_90);
      adelante();
    }
  }
  if (modo == AVANZAR_META_Y_FIN_C) {
    adelante();
    delay(DELAY_LLEGADA_C);
    parar();
  }
  if (modo == AVANZAR_META_Y_FIN_L) {
    adelante();
    delay(DELAY_LLEGADA_L);
    parar();
  }
  if (modo == CASILLA_BLANCO) {
    parar();
  }
  if (modo == DETENER_CUADRICULA) {
    parar();
  }
}


void sigue_lineas_hasta_interseccion() {
  read_ir();
  while (!IR_INI) {
    if (Serial1.read() == DETENER_CUADRICULA) {
      escape = 1;
      break;
    }
    read_ir();
    if (IR_L == 0 && IR_R == 0 && !IR_INI) {
      adelante();
    }
    if (IR_L == 1 && IR_R == 1 && !IR_INI) {
      adelante();
    }
    if (IR_L == 1 && IR_R == 0 && !IR_INI) {
      for (int i = 0; i < (TIEMPO_GIRO_IR * 10); i++) {
        izquierda();
        delay(VEL_REL_GIRO_IR * 0.1);
        parar();
        delay((1 - VEL_REL_GIRO_IR) * 0.1);
      }
    }
    if (IR_L == 0 && IR_R == 1 && !IR_INI) {
      for (int i = 0; i < (TIEMPO_GIRO_IR * 10); i++) {
        derecha();
        delay(VEL_REL_GIRO_IR * 0.1);
        parar();
        delay((1 - VEL_REL_GIRO_IR) * 0.1);
      }
    }
  }
}



void read_ir() {
  IR_INI = digitalRead(pin_IR_INI);
  IR_INE = digitalRead(pin_IR_INE);
  IR_R = digitalRead(pin_IR_R);
  IR_L = digitalRead(pin_IR_L);
  IR_LE = digitalRead(pin_IR_LE);
  IR_LI = digitalRead(pin_IR_LI);
}



void Rota_MPU(float angulo_destino) {

  tone(pin_zumbador, 1000);
  digitalWrite(pin_led, 1);
  m_o = millis();
  angulo = 0.0;

  while (abs(angulo) < abs(angulo_destino)) {

    if (angulo_destino > 0) {
      if (abs(angulo) < ANGULO_CAMBIO_VEL) {
        for (int i = 0; i < 100; i++) {
          parar();
          delay(0.1 * (1 - VEL_REL_GIRO_90_o));
          derecha();
          delay(0.1 * VEL_REL_GIRO_90_o);
        }
      }
      else {
        for (int i = 0; i < 100; i++) {
          parar();
          delay(0.1 * (1 - VEL_REL_GIRO_90_f));
          derecha();
          delay(0.1 * VEL_REL_GIRO_90_f);
        }
      }
    }

    else {
      if (abs(angulo) < ANGULO_CAMBIO_VEL) {
        for (int i = 0; i < 100; i++) {
          parar();
          delay(0.1 * (1 - VEL_REL_GIRO_90_o));
          izquierda();
          delay(0.1 * VEL_REL_GIRO_90_o);
        }
      }
      else {
        for (int i = 0; i < 100; i++) {
          parar();
          delay(0.1 * (1 - VEL_REL_GIRO_90_f));
          izquierda();
          delay(0.1 * VEL_REL_GIRO_90_f);
        }
      }
    }

    if (abs(angulo) > ANGULO_MPU_COMPRUEBA_IR) {
      read_ir();
      if (angulo_destino > 0 && IR_L == 1) {
        parar();
        //delay(1000);
        break;
      }
      if (angulo_destino < 0 && IR_R == 1) {
        parar();
        //delay(1000);
        break;
      }

    }


    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angulo = angulo + ((gy - val_calibrado) * 0.007907 * (millis() - m_o) * 0.001);
    m_o = millis();
    //Serial.println(angulo);
  }

  noTone(pin_zumbador);
  digitalWrite(pin_led, 0);

}



/********************
**Movimiento básico**
********************/


void parar() {
  if (digitalRead(pin_boton_motores) == 0) {
    digitalWrite(pin_R_A, 0);
    digitalWrite(pin_R_B, 0);
    digitalWrite(pin_L_A, 0);
    digitalWrite(pin_L_B, 0);
  }
  else {
    digitalWrite(pin_R_A, 0);
    digitalWrite(pin_R_B, 0);
    digitalWrite(pin_L_A, 0);
    digitalWrite(pin_L_B, 0);
  }
}
void izquierda() {
  if (digitalRead(pin_boton_motores) == 0) {
    digitalWrite(pin_R_A, 1);
    digitalWrite(pin_R_B, 0);
    digitalWrite(pin_L_A, 0);
    digitalWrite(pin_L_B, 1);
  }
  else {
    digitalWrite(pin_R_A, 0);
    digitalWrite(pin_R_B, 1);
    digitalWrite(pin_L_A, 1);
    digitalWrite(pin_L_B, 0);
  }
}
void derecha() {
  if (digitalRead(pin_boton_motores) == 0) {
    digitalWrite(pin_R_A, 0);
    digitalWrite(pin_R_B, 1);
    digitalWrite(pin_L_A, 1);
    digitalWrite(pin_L_B, 0);
  }
  else {
    digitalWrite(pin_R_A, 1);
    digitalWrite(pin_R_B, 0);
    digitalWrite(pin_L_A, 0);
    digitalWrite(pin_L_B, 1);
  }
}
void adelante() {
  if (digitalRead(pin_boton_motores) == 0) {
    digitalWrite(pin_R_A, 1);
    digitalWrite(pin_R_B, 0);
    digitalWrite(pin_L_A, 1);
    digitalWrite(pin_L_B, 0);
  }
  else {
    digitalWrite(pin_R_A, 0);
    digitalWrite(pin_R_B, 1);
    digitalWrite(pin_L_A, 0);
    digitalWrite(pin_L_B, 1);
  }
}
void atras() {
  if (digitalRead(pin_boton_motores) == 0) {
    digitalWrite(pin_R_A, 0);
    digitalWrite(pin_R_B, 1);
    digitalWrite(pin_L_A, 0);
    digitalWrite(pin_L_B, 1);
  }
  else {
    digitalWrite(pin_R_A, 1);
    digitalWrite(pin_R_B, 0);
    digitalWrite(pin_L_A, 1);
    digitalWrite(pin_L_B, 0);
  }
}






/*****************
**Inicialización**
*****************/


void SetPines()
{
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(24, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);
  pinMode(33, INPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(36, INPUT);
  pinMode(37, INPUT);
  pinMode(38, INPUT);
  pinMode(39, INPUT);
  pinMode(40, INPUT);
  pinMode(41, INPUT);
  pinMode(42, INPUT);
  pinMode(43, INPUT);
  pinMode(44, INPUT);
  pinMode(45, INPUT);
  pinMode(46, INPUT);
  pinMode(47, INPUT);
  pinMode(48, INPUT);
  pinMode(49, INPUT);
  pinMode(50, INPUT);
  pinMode(51, INPUT);
  pinMode(52, INPUT);
  pinMode(53, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  pinMode(A14, INPUT);
  pinMode(A15, INPUT);
  pinMode(pin_boton_motores, INPUT_PULLUP);
  pinMode(pin_IR_INI, INPUT);
  pinMode(pin_IR_INE, INPUT);
  pinMode(pin_IR_R, INPUT);
  pinMode(pin_IR_L, INPUT);
  pinMode(pin_IR_LE, INPUT);
  pinMode(pin_IR_LI, INPUT);
  pinMode(pin_R_A, OUTPUT);
  pinMode(pin_R_B, OUTPUT);
  pinMode(pin_L_A, OUTPUT);
  pinMode(pin_L_B, OUTPUT);
  pinMode(pin_IRS_LL, INPUT);
  pinMode(pin_IRS_LC, INPUT);
  pinMode(pin_IRS_FL, INPUT);
  pinMode(pin_IRS_FC, INPUT);
  pinMode(pin_IRS_RL, INPUT);
  pinMode(pin_IRS_RC, INPUT);
  pinMode(pin_boton_motores, OUTPUT);
  pinMode(pin_zumbador, OUTPUT);
  pinMode(pin_led, OUTPUT);
  Serial1.begin(9600);
}


void Enciende_MPU()
{
  digitalWrite(pin_led, 1);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  accelgyro.initialize();

  if (accelgyro.testConnection() == 1)
  {
    tone(pin_zumbador, 1000, 200);
    delay(400);
    tone(pin_zumbador, 1000, 200);
    delay(400);
  }
  else
  {
    while (true)
    {
      tone(pin_zumbador, 2000);
    }
  }


  for (int i = 0; i < 3000; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(1);
    if ((i / 100) % 2 > 0) {
      digitalWrite(pin_led, 1);
    }
    else {
      digitalWrite(pin_led, 0);
    }
  }
  tone(pin_zumbador, 1000, 200);
  delay(400);
  tone(pin_zumbador, 1000, 200);
  delay(400);
  digitalWrite(pin_led, 0);
}
