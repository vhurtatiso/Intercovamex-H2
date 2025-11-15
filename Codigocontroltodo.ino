/*
  Control presión + válvula + 2 MFC (Ar/O2) con PCF8591 + UI Nextion (Serial2)
  - Fuente de presión seleccionable: PIRANI o MANUAL (desde Nextion)
  - Modo: AUTO (control activo) / STOP (salida segura)
  - Setpoints MFC Ar/O2 desde Nextion
  - Reporte de estado a Nextion

  Placa: Arduino Mega 2560
  Nextion en Serial2: 16(TX2)->RX de Nextion, 17(RX2)->TX de Nextion, GND común

  © Valentina Project
*/
#include <Wire.h>

//================== CONFIG GENERAL ==================
#define DRIVE_TB6612 true
#define VALVE_OPENS_LOWER_PRESSURE true

//---- PCF8591 ----
const bool HAVE_PCF_AR = true;
const bool HAVE_PCF_O2 = true;    // pon false si no tienes el segundo PCF8591
const uint8_t PCF_AR_ADDR = 0x48; // A0..A2 GND => 0x48
const uint8_t PCF_O2_ADDR = 0x49; // p.ej. A0=VCC => 0x49
const bool O2_PWM_RC = false;     // true => O2 por PWM+RC si no hay 2º PCF
const int PIN_O2_PWM = 5;

//---- Pines MFC feedback (0–5V)
const int PIN_MFC_AR_FLOW_FB = A1;
const int PIN_MFC_O2_FLOW_FB = A2;

//---- Motor / Encoder / Pirani ----
const int PIN_AIN1 = 7, PIN_AIN2 = 8, PIN_PWMA = 9, PIN_STBY = 6;
const int encoderA = 18, encoderB = 19;
const int PIN_PIRANI = A0;

//---- Límites mecánicos y control ----
long POS_MIN_COUNTS = 0;
long POS_MAX_COUNTS = 12000;
const int PWM_MAX = 255;
const int PWM_SLEW = 8;

//---- Setpoints y estado de MFC ----
float MFC_AR_SP = 0.00f;  // 0..1
float MFC_O2_SP = 0.00f;  // 0..1

//---- Control Presión / UI ----
enum class PSource { PIRANI, MANUAL };
enum class RunMode { STOP, AUTO };

PSource P_SOURCE = PSource::PIRANI;
RunMode  RUN_MODE = RunMode::AUTO;

float P_SET = 0.120f;        // Torr (setpoint de presión)
float P_MANUAL = 0.120f;     // Torr (si fuente=MANUAL, esta es la "medida" usada por el lazo)
float P_meas_lpf = 0.0f;     // Presión filtrada (si PIRANI)

//---- Tasas de muestreo ----
const uint16_t LOOP_HZ_OUTER = 50, LOOP_HZ_INNER = 200, LOOP_HZ_IO = 50, LOOP_HZ_UI = 10;
unsigned long tOuter=0, tInner=0, tIO=0, tUI=0;
const unsigned long TsOuter=1000/LOOP_HZ_OUTER, TsInner=1000/LOOP_HZ_INNER, TsIO=1000/LOOP_HZ_IO, TsUI=1000/LOOP_HZ_UI;

//================== ESTADO ==================
volatile long enc_counts=0; volatile uint8_t lastEncoded=0;
long pos_counts=0; float pos_ref_counts=0; int pwm_cmd=0;

//================== UTILIDADES ==================
template<typename T> struct LPF{ float a; T y; bool first; void init(float aa){a=aa; first=true; y=0;}
  T step(T x){ if(first){y=x; first=false;} else y=a*x+(1-a)*y; return y; } };
LPF<float> lpfP;

struct PID{ float Kp,Ki,Kd,Ts,u,umin,umax,e,ep,i; bool first;
  void init(float kp,float ki,float kd,float ts,float umin_,float umax_){Kp=kp;Ki=ki;Kd=kd;Ts=ts;umin=umin_;umax=umax_;u=0;e=ep=0;i=0;first=true;}
  float step(float r,float y){ e=r-y; float p=Kp*e; i += Ki*Ts*e; if(i>umax)i=umax; if(i<umin)i=umin;
    float d=0; if(first){d=0; first=false;} else d=Kd*(e-ep)/Ts; u=p+i+d; if(u>umax)u=umax; if(u<umin)u=umin; ep=e; return u; }
  void resetI(){ i=0; }
};
PID pidP, pidPos;

//================== PROTOS ==================
void motorSetPWM(int pwm);
void encoderISR();
float pirani_ADC_to_Torr(int adc);
void pcf8591_write(uint8_t addr, uint8_t val);

//========== Nextion (Serial2) ==========
HardwareSerial &NX = Serial2;
void nxSend(const String& cmd){
  NX.print(cmd);
  NX.write(0xFF); NX.write(0xFF); NX.write(0xFF);
}
void nxSetTxt(const char* comp, const String& txt){
  String cmd = String(comp) + "=\"" + txt + "\"";
  nxSend(cmd);
}
void nxSetVal(const char* comp, long v){
  String cmd = String(comp) + "=" + String(v);
  nxSend(cmd);
}
void nxHandleInput();  // parsea líneas tipo: SP=0.120, AR=0.25, O2=0.05, SRC=pirani/manual, START=auto/stop, PMAN=0.200

//================== SETUP ==================
void setup(){
  Serial.begin(115200);
  NX.begin(115200);    // Nextion baud (ajusta a tu HMI)
  Wire.begin();

  // Motor TB6612
  pinMode(PIN_AIN1,OUTPUT); pinMode(PIN_AIN2,OUTPUT); pinMode(PIN_PWMA,OUTPUT);
  pinMode(PIN_STBY,OUTPUT); digitalWrite(PIN_STBY,HIGH);

  // Encoder
  pinMode(encoderA,INPUT_PULLUP); pinMode(encoderB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderISR, CHANGE);

  // Sensores
  pinMode(PIN_PIRANI,INPUT);
  pinMode(PIN_MFC_AR_FLOW_FB,INPUT);
  pinMode(PIN_MFC_O2_FLOW_FB,INPUT);
  if(O2_PWM_RC) pinMode(PIN_O2_PWM, OUTPUT);

  // PIDs
  pidP.init( 400.0f, 60.0f, 0.0f, 1.0f/LOOP_HZ_OUTER, (float)POS_MIN_COUNTS, (float)POS_MAX_COUNTS );
  pidPos.init( 0.6f, 0.05f, 0.0f, 1.0f/LOOP_HZ_INNER, -PWM_MAX, PWM_MAX );
  lpfP.init(0.25f);

  // MFCs a 0V
  if(HAVE_PCF_AR)  pcf8591_write(PCF_AR_ADDR, 0);
  if(HAVE_PCF_O2)  pcf8591_write(PCF_O2_ADDR, 0);
  if(O2_PWM_RC)    analogWrite(PIN_O2_PWM, 0);

  // UI inicial
  nxSetTxt("tSrc.txt", "PIRANI");
  nxSetTxt("tMode.txt", "AUTO");
  nxSetTxt("tMsg.txt",  "Ready");
}

//================== LOOP ==================
void loop(){
  unsigned long now=millis();

  // Gestionar entrada de Nextion
  nxHandleInput();

  // ------ Lazo interno (posición) ------
  if(now - tInner >= TsInner){
    tInner = now;
    noInterrupts(); pos_counts = enc_counts; interrupts();
    if(pos_counts<POS_MIN_COUNTS) pos_counts=POS_MIN_COUNTS;
    if(pos_counts>POS_MAX_COUNTS) pos_counts=POS_MAX_COUNTS;

    int u = 0;
    if (RUN_MODE == RunMode::AUTO) {
      u = (int)pidPos.step(pos_ref_counts, (float)pos_counts);
    } else {
      // STOP: mantener motor quieto
      pidPos.resetI();
      u = 0;
    }

    // rampa
    if     (u > pwm_cmd + PWM_SLEW) pwm_cmd += PWM_SLEW;
    else if(u < pwm_cmd - PWM_SLEW) pwm_cmd -= PWM_SLEW;
    else                            pwm_cmd  = u;
    motorSetPWM(pwm_cmd);
  }

  // ------ IO: presión, MFCs, feedback ------
  static float flow_ar=0, flow_o2=0;
  if(now - tIO >= TsIO){
    tIO = now;

    // Medición de presión según fuente
    float P_used = 0.0f;
    if (P_SOURCE == PSource::PIRANI){
      int adcP = analogRead(PIN_PIRANI);
      float P_raw = pirani_ADC_to_Torr(adcP);
      P_meas_lpf = lpfP.step(P_raw);
      P_used = P_meas_lpf;

      // Falla sensor => STOP + salidas seguras
      if(adcP<2 || adcP>1021){
        RUN_MODE = RunMode::STOP;
        motorSetPWM(0); pidPos.resetI(); pidP.resetI();
        if(HAVE_PCF_AR) pcf8591_write(PCF_AR_ADDR, 0);
        if(HAVE_PCF_O2) pcf8591_write(PCF_O2_ADDR, 0);
        if(O2_PWM_RC)   analogWrite(PIN_O2_PWM, 0);
        nxSetTxt("tMsg.txt", "Alerta Pirani");
      }
    } else {
      // Fuente manual desde Nextion
      P_used = P_MANUAL;
    }

    // Feedback de caudal (0..1 de escala de su salida 0–5 V)
    int adcAR = analogRead(PIN_MFC_AR_FLOW_FB);
    int adcO2 = analogRead(PIN_MFC_O2_FLOW_FB);
    flow_ar = adcAR / 1023.0f;
    flow_o2 = adcO2 / 1023.0f;

    // Salida a MFCs
    uint8_t d_ar = (uint8_t)(constrain(MFC_AR_SP,0,1)*255.0f + 0.5f);
    uint8_t d_o2 = (uint8_t)(constrain(MFC_O2_SP,0,1)*255.0f + 0.5f);
    if(HAVE_PCF_AR)  pcf8591_write(PCF_AR_ADDR, d_ar);
    if(HAVE_PCF_O2)  pcf8591_write(PCF_O2_ADDR, d_o2);
    if(O2_PWM_RC)    analogWrite(PIN_O2_PWM, d_o2);
  }

  // ------ Lazo externo (presión) ------
  if(now - tOuter >= TsOuter){
    tOuter = now;

    if (RUN_MODE == RunMode::AUTO){
      float P_now = (P_SOURCE == PSource::PIRANI) ? P_meas_lpf : P_MANUAL;
      float sign = VALVE_OPENS_LOWER_PRESSURE ? -1.0f : +1.0f;
      float pos_ref = pidP.step(0.0f, sign*(P_SET - P_now));
      if(pos_ref < POS_MIN_COUNTS) pos_ref = POS_MIN_COUNTS;
      if(pos_ref > POS_MAX_COUNTS) pos_ref = POS_MAX_COUNTS;
      pos_ref_counts = pos_ref;
    } else {
      pidP.resetI();
      // Mantener pos_ref actual o congelarla
    }
  }

  // ------ UI periódica: publicar a Nextion ------
  if(now - tUI >= TsUI){
    tUI = now;
    // Mostrar P usada, SP, POS, PWM, MFCs, modo y fuente
    float P_show = (P_SOURCE==PSource::PIRANI) ? P_meas_lpf : P_MANUAL;
    nxSetTxt("tP.txt", String(P_show, 6));
    nxSetTxt("tSP.txt", String(P_SET, 6));
    nxSetVal("nPOS.val", pos_counts);
    nxSetVal("nPWM.val", pwm_cmd);
    nxSetTxt("tARsp.txt", String(MFC_AR_SP,3));
    nxSetTxt("tO2sp.txt", String(MFC_O2_SP,3));
    // flows (0..1)
    nxSetTxt("tARfb.txt", String((double)(analogRead(PIN_MFC_AR_FLOW_FB)/1023.0f),3));
    nxSetTxt("tO2fb.txt", String((double)(analogRead(PIN_MFC_O2_FLOW_FB)/1023.0f),3));
    nxSetTxt("tSrc.txt", (P_SOURCE==PSource::PIRANI) ? "PIRANI" : "MANUAL");
    nxSetTxt("tMode.txt", (RUN_MODE==RunMode::AUTO) ? "AUTO" : "STOP");
  }

  // Consola serie de depuración opcional
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if(cmd.startsWith("SP "))      { P_SET = cmd.substring(3).toFloat(); }
    else if(cmd.startsWith("AR=")) { MFC_AR_SP = constrain(cmd.substring(3).toFloat(),0,1); }
    else if(cmd.startsWith("O2=")) { MFC_O2_SP = constrain(cmd.substring(3).toFloat(),0,1); }
    else if(cmd=="SRC pirani")     { P_SOURCE=PSource::PIRANI; }
    else if(cmd=="SRC manual")     { P_SOURCE=PSource::MANUAL; }
    else if(cmd=="START auto")     { RUN_MODE=RunMode::AUTO; }
    else if(cmd=="START stop")     { RUN_MODE=RunMode::STOP; motorSetPWM(0); }
    else if(cmd.startsWith("PMAN=")){ P_MANUAL = cmd.substring(5).toFloat(); }
  }
}

//================== BAJO NIVEL ==================
void motorSetPWM(int pwm){
  if(pwm>PWM_MAX) pwm=PWM_MAX; if(pwm<-PWM_MAX) pwm=-PWM_MAX;
  if(pwm==0){ digitalWrite(PIN_AIN1,LOW); digitalWrite(PIN_AIN2,LOW); analogWrite(PIN_PWMA,0); return; }
  if(pwm>0){ digitalWrite(PIN_AIN1,HIGH); digitalWrite(PIN_AIN2,LOW);  analogWrite(PIN_PWMA,pwm); }
  else     { digitalWrite(PIN_AIN1,LOW);  digitalWrite(PIN_AIN2,HIGH); analogWrite(PIN_PWMA,-pwm); }
}

void encoderISR(){
  uint8_t MSB=digitalRead(encoderA), LSB=digitalRead(encoderB);
  uint8_t encoded=(MSB<<1)|LSB; uint8_t sum=(lastEncoded<<2)|encoded;
  if(sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) enc_counts++;
  if(sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) enc_counts--;
  lastEncoded=encoded;
}

// Calibración Pirani: V = a*log10(P) + b  -> P = 10^((V-b)/a)
float pirani_ADC_to_Torr(int adc){
  const float a=-1.2f, b=3.0f;   // EDITAR según tu transductor
  float V = 5.0f * adc / 1023.0f;
  float P = pow(10.0f, (V-b)/a);
  if(P<1e-5f) P=1e-5f; if(P>10.0f) P=10.0f; return P;
}

void pcf8591_write(uint8_t addr, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(0x40);     // DAC enable
  Wire.write(val);      // 0..255 => 0..5 V
  Wire.endTransmission();
}

//========== Nextion: parser simple ==========
void nxHandleInput(){
  static String buf;
  while(NX.available()){
    char c = NX.read();
    if(c=='\n' || c=='\r'){
      if(buf.length()){
        // parse
        String s=buf; s.trim(); buf="";
        s.toLowerCase();

        if(s.startsWith("sp=")){
          P_SET = s.substring(3).toFloat();
          nxSetTxt("tMsg.txt", "SP ok");
        } else if(s.startsWith("ar=")){
          MFC_AR_SP = constrain(s.substring(3).toFloat(),0,1);
          nxSetTxt("tMsg.txt", "Ar ok");
        } else if(s.startsWith("o2=")){
          MFC_O2_SP = constrain(s.substring(3).toFloat(),0,1);
          nxSetTxt("tMsg.txt", "O2 ok");
        } else if(s=="src=pirani"){
          P_SOURCE = PSource::PIRANI; nxSetTxt("tSrc.txt","PIRANI");
        } else if(s=="src=manual"){
          P_SOURCE = PSource::MANUAL; nxSetTxt("tSrc.txt","MANUAL");
        } else if(s=="start=auto"){
          RUN_MODE = RunMode::AUTO; nxSetTxt("tMode.txt","AUTO");
        } else if(s=="start=stop"){
          RUN_MODE = RunMode::STOP; motorSetPWM(0); nxSetTxt("tMode.txt","STOP");
        } else if(s.startsWith("pman=")){
          P_MANUAL = s.substring(5).toFloat(); nxSetTxt("tMsg.txt","Pman ok");
        } else if(s.startsWith("posmax=")){
          POS_MAX_COUNTS = s.substring(7).toInt(); nxSetTxt("tMsg.txt","POSmax ok");
        } else if(s.startsWith("posmin=")){
          POS_MIN_COUNTS = s.substring(7).toInt(); nxSetTxt("tMsg.txt","POSmin ok");
        } else {
          nxSetTxt("tMsg.txt","cmd?");
        }
      }
    } else {
      buf += c;
      if(buf.length()>64) buf.remove(0, buf.length()-64);
    }
  }
}
