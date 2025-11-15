/*
   Sistema de control de presión para la cámara de sputtering Intercovamex H2.
   Un MFC para Argón (señal analógica generada con PCF8591)
   y un MFC para Nitrógeno (señal analógica generada mediante PWM filtrado).
   El control incluye un lazo externo de presión y un lazo interno de posición
   para accionar la válvula mediante un motor DC con encoder.

   Plataforma: Arduino Mega 2560
   Interfaz HMI: Nextion (Serial2)
*/

#include <Wire.h>

// CONFIGURACIÓN GENERAL 

// DAC para el MFC de Argón
const uint8_t PCF_AR_ADDR = 0x48;

// Salida PWM para el MFC de Nitrógeno (PWM + filtro RC)
const int PIN_N2_PWM = 5;

// Entradas de retroalimentación (0–5 V, escala 0–1)
const int PIN_MFC_AR_FLOW_FB = A1;
const int PIN_MFC_N2_FLOW_FB = A2;

// Motor y encoder
const int PIN_AIN1 = 7, PIN_AIN2 = 8, PIN_PWMA = 9, PIN_STBY = 6;
const int encoderA = 18, encoderB = 19;

// Sensor Pirani (0–5 V)
const int PIN_PIRANI = A0;

// Límites mecánicos de la válvula
long POS_MIN_COUNTS = 0;
long POS_MAX_COUNTS = 12000;

const int PWM_MAX  = 255;
const int PWM_SLEW = 8;

// Setpoints normalizados MFC
float MFC_AR_SP = 0.0f;   // 0–1
float MFC_N2_SP = 0.0f;   // 0–1

// Modos de operación
enum class PSource { PIRANI, MANUAL };
enum class RunMode { STOP, AUTO };

PSource P_SOURCE = PSource::PIRANI;
RunMode RUN_MODE = RunMode::AUTO;

// Presiones en Torr
float P_SET    = 0.120f;
float P_MANUAL = 0.120f;
float P_meas_lpf = 0.0f;

// Frecuencias de operación de cada lazo
const uint16_t LOOP_HZ_OUTER = 50;
const uint16_t LOOP_HZ_INNER = 200;
const uint16_t LOOP_HZ_IO    = 50;
const uint16_t LOOP_HZ_UI    = 10;

unsigned long tOuter=0, tInner=0, tIO=0, tUI=0;

// ESTADO 
volatile long enc_counts = 0;
volatile uint8_t lastEncoded = 0;

long pos_counts = 0;
float pos_ref_counts = 0;
int pwm_cmd = 0;

// FILTRO Y PID 
template<typename T>
struct LPF {
  float a;
  T y;
  bool first;
  void init(float aa){ a=aa; y=0; first=true; }
  T step(T x){
    if(first){ y=x; first=false; }
    else y = a*x + (1-a)*y;
    return y;
  }
};

LPF<float> lpfP;

struct PID {
  float Kp,Ki,Kd,Ts;
  float u,umin,umax;
  float e,ep,i;
  bool first;

  void init(float kp,float ki,float kd,float ts,float umin_,float umax_){
    Kp=kp; Ki=ki; Kd=kd; Ts=ts;
    umin=umin_; umax=umax_;
    u=0; e=ep=0; i=0;
    first=true;
  }

  float step(float r,float y){
    e = r - y;
    float p = Kp * e;

    i += Ki * Ts * e;
    if(i > umax) i = umax;
    if(i < umin) i = umin;

    float d = 0;
    if(!first) d = Kd * (e - ep) / Ts;
    first = false;

    u = p + i + d;
    u = constrain(u, umin, umax);

    ep = e;
    return u;
  }

  void resetI(){ i=0; }
};

PID pidP, pidPos;

// Nextion 
HardwareSerial &NX = Serial2;

void nxSend(const String& cmd){
  NX.print(cmd);
  NX.write(0xFF); NX.write(0xFF); NX.write(0xFF);
}

void nxSetTxt(const char* comp, const String& txt){
  nxSend(String(comp) + "=\"" + txt + "\"");
}

void nxSetVal(const char* comp, long v){
  nxSend(String(comp) + "=" + String(v));
}

void nxHandleInput();

// SETUP 
void setup(){
  Serial.begin(115200);
  NX.begin(115200);
  Wire.begin();

  // Configuración del driver del motor
  pinMode(PIN_AIN1,OUTPUT);
  pinMode(PIN_AIN2,OUTPUT);
  pinMode(PIN_PWMA,OUTPUT);
  pinMode(PIN_STBY,OUTPUT);
  digitalWrite(PIN_STBY,HIGH);

  // Encoder incremental
  pinMode(encoderA,INPUT_PULLUP);
  pinMode(encoderB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderISR, CHANGE);

  // Sensores
  pinMode(PIN_PIRANI,INPUT);
  pinMode(PIN_MFC_AR_FLOW_FB,INPUT);
  pinMode(PIN_MFC_N2_FLOW_FB,INPUT);
  pinMode(PIN_N2_PWM,OUTPUT);

  // Inicialización de los controladores PID
  pidP.init(400.0f, 60.0f, 0.0f, 1.0f/LOOP_HZ_OUTER, POS_MIN_COUNTS, POS_MAX_COUNTS);
  pidPos.init(0.6f, 0.05f, 0.0f, 1.0f/LOOP_HZ_INNER, -PWM_MAX, PWM_MAX);
  lpfP.init(0.25f);

  // MFCs en cero
  pcf8591_write(PCF_AR_ADDR, 0);
  analogWrite(PIN_N2_PWM, 0);

  // Estado inicial en HMI
  nxSetTxt("tSrc.txt","PIRANI");
  nxSetTxt("tMode.txt","AUTO");
  nxSetTxt("tMsg.txt","Ready");
}

// LOOP 
void loop(){
  unsigned long now = millis();

  // Entrada desde el HMI
  nxHandleInput();

  //--- Lazo interno de posición (200 Hz) ---
  if(now - tInner >= 1000/LOOP_HZ_INNER){
    tInner = now;

    noInterrupts();
    pos_counts = enc_counts;
    interrupts();

    pos_counts = constrain(pos_counts, POS_MIN_COUNTS, POS_MAX_COUNTS);

    int u = (RUN_MODE == RunMode::AUTO)
              ? pidPos.step(pos_ref_counts, pos_counts)
              : (pidPos.resetI(), 0);

    // Rampa de PWM para evitar discontinuidades
    if(u > pwm_cmd + PWM_SLEW) pwm_cmd += PWM_SLEW;
    else if(u < pwm_cmd - PWM_SLEW) pwm_cmd -= PWM_SLEW;
    else pwm_cmd = u;

    motorSetPWM(pwm_cmd);
  }

  //--- Lecturas analógicas y salidas a MFC (50 Hz) ---
  if(now - tIO >= 1000/LOOP_HZ_IO){
    tIO = now;

    float P_used = 0.0f;

    if(P_SOURCE == PSource::PIRANI){
      int adcP = analogRead(PIN_PIRANI);
      float P_raw = pirani_ADC_to_Torr(adcP);
      P_meas_lpf = lpfP.step(P_raw);
      P_used = P_meas_lpf;

      // Detección simple de falla del Pirani
      if(adcP < 2 || adcP > 1021){
        RUN_MODE = RunMode::STOP;
        motorSetPWM(0);
        pidPos.resetI(); pidP.resetI();
        pcf8591_write(PCF_AR_ADDR,0);
        analogWrite(PIN_N2_PWM,0);
        nxSetTxt("tMsg.txt","Alerta Pirani");
      }
    }
    else {
      P_used = P_MANUAL;
    }

    // Retroalimentación de caudal
    float flow_ar = analogRead(PIN_MFC_AR_FLOW_FB) / 1023.0f;
    float flow_n2 = analogRead(PIN_MFC_N2_FLOW_FB) / 1023.0f;

    // Salida a los MFC
    uint8_t dac_ar = constrain(MFC_AR_SP,0,1) * 255.0f;
    uint8_t pwm_n2 = constrain(MFC_N2_SP,0,1) * 255.0f;

    pcf8591_write(PCF_AR_ADDR, dac_ar);
    analogWrite(PIN_N2_PWM, pwm_n2);
  }

  //--- Lazo externo de presión (50 Hz) ---
  if(now - tOuter >= 1000/LOOP_HZ_OUTER){
    tOuter = now;

    if(RUN_MODE == RunMode::AUTO){
      float P_now = (P_SOURCE==PSource::PIRANI) ? P_meas_lpf : P_MANUAL;
      float pos_ref = pidP.step(0.0f, -(P_SET - P_now)); // signo según apertura
      pos_ref_counts = constrain(pos_ref, POS_MIN_COUNTS, POS_MAX_COUNTS);
    }
    else {
      pidP.resetI();
    }
  }

  //--- Actualización del HMI (10 Hz) ---
  if(now - tUI >= 1000/LOOP_HZ_UI){
    tUI = now;

    float P_show = (P_SOURCE==PSource::PIRANI) ? P_meas_lpf : P_MANUAL;

    nxSetTxt("tP.txt",   String(P_show,6));
    nxSetTxt("tSP.txt",  String(P_SET,6));
    nxSetVal("nPOS.val", pos_counts);
    nxSetVal("nPWM.val", pwm_cmd);

    nxSetTxt("tARsp.txt", String(MFC_AR_SP,3));
    nxSetTxt("tN2sp.txt", String(MFC_N2_SP,3));

    nxSetTxt("tARfb.txt", String(analogRead(PIN_MFC_AR_FLOW_FB)/1023.0f,3));
    nxSetTxt("tN2fb.txt", String(analogRead(PIN_MFC_N2_FLOW_FB)/1023.0f,3));

    nxSetTxt("tSrc.txt", (P_SOURCE==PSource::PIRANI) ? "PIRANI" : "MANUAL");
    nxSetTxt("tMode.txt",(RUN_MODE==RunMode::AUTO)  ? "AUTO"   : "STOP");
  }
}

// FUNCIONES
void motorSetPWM(int pwm){
  pwm = constrain(pwm,-PWM_MAX,PWM_MAX);

  if(pwm==0){
    digitalWrite(PIN_AIN1,LOW);
    digitalWrite(PIN_AIN2,LOW);
    analogWrite(PIN_PWMA,0);
    return;
  }

  if(pwm>0){
    digitalWrite(PIN_AIN1,HIGH);
    digitalWrite(PIN_AIN2,LOW);
    analogWrite(PIN_PWMA,pwm);
  }
  else {
    digitalWrite(PIN_AIN1,LOW);
    digitalWrite(PIN_AIN2,HIGH);
    analogWrite(PIN_PWMA,-pwm);
  }
}

void encoderISR(){
  uint8_t MSB = digitalRead(encoderA);
  uint8_t LSB = digitalRead(encoderB);
  uint8_t encoded = (MSB << 1) | LSB;
  uint8_t sum = (lastEncoded << 2) | encoded;

  if(sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) enc_counts++;
  if(sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) enc_counts--;

  lastEncoded = encoded;
}

// Modelo logarítmico del Pirani
float pirani_ADC_to_Torr(int adc){
  const float a = -1.2f;
  const float b = 3.0f;

  float V = 5.0f * adc / 1023.0f;
  float P = pow(10.0f, (V-b)/a);

  return constrain(P,1e-5f,10.0f);
}

// DAC del PCF8591
void pcf8591_write(uint8_t addr, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(0x40);
  Wire.write(val);
  Wire.endTransmission();
}

// Parser de comandos del Nextion
void nxHandleInput(){
  static String buf;

  while(NX.available()){
    char c = NX.read();

    if(c=='\n' || c=='\r'){
      if(buf.length()){
        String s=buf; buf="";
        s.trim(); s.toLowerCase();

        if(s.startsWith("sp="))      P_SET = s.substring(3).toFloat();
        else if(s.startsWith("ar=")) MFC_AR_SP = constrain(s.substring(3).toFloat(),0,1);
        else if(s.startsWith("n2=")) MFC_N2_SP = constrain(s.substring(3).toFloat(),0,1);
        else if(s=="src=pirani")    P_SOURCE=PSource::PIRANI;
        else if(s=="src=manual")    P_SOURCE=PSource::MANUAL;
        else if(s=="start=auto")    RUN_MODE=RunMode::AUTO;
        else if(s=="start=stop")    RUN_MODE=RunMode::STOP;
        else if(s.startsWith("pman=")) P_MANUAL = s.substring(5).toFloat();
      }
    }
    else {
      buf += c;
      if(buf.length()>64) buf.remove(0, buf.length()-64);
    }
  }
}

