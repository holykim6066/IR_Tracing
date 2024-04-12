HardwareSerial Serial2(PA3, PA2); // 특정 보드에서 PA3, PA2를 사용하도록 설정
int ir1 = 0;
int ir2 = 0;
int ir3 = 0;
bool rcv_status = false;
bool rcv_ready = false;
byte rcv_data = 0;
byte rcv_checksum = 0;
byte rx_buffer[4];
byte rx_data[4];
int rcv_count = 0;
int rcv_index = 0;

/*모터 관련*/
#include "BlackpillTimer.h"
#define ENC1_CHA  PB6
#define ENC1_CHB  PB7
#define ENC2_CHA PB8
#define ENC2_CHB PB9
#define M1_PWM    PB0
#define M1_DIR1   PB12
#define M1_DIR2   PB13
#define M2_PWM PB1
#define M2_DIR1 PB14
#define M2_DIR2 PB15
#define STBY      PA8
BlackpillTimer myTimer(TIM1);
float Kp = 1.0;
float Ki = 0.2;
float Kd = 0.1;
/*왼쪽 바퀴 PDI*/

float Kp_m2 = 1.5;
float Ki_m2 = 0.2;
float Kd_m2 = 0.3;
/*오른쪽 바퀴 PDI*/

int e1cnt = 0;
int e1cnt_k = 0, e1cnt_k_1 = 0, d_e1cnt = 0;
int e2cnt = 0;
int e2cnt_k = 0, e2cnt_k_1 = 0, d_e2cnt = 0;
/*encoder 현재값,과거값,증감값 저장변수*/

float m1speed = 0;
float m1_ref_spd = 0;
float m1_err_spd = 0;
float m1_err_spd_k_1 = 0;
float m1_derr_spd = 0;
float m1_err_sum = 0;
/*모터1 변수*/

float m2speed = 0;
float m2_ref_spd = 0;
float m2_err_spd = 0;
float m2_err_spd_k_1 = 0;
float m2_derr_spd = 0;
float m2_err_sum = 0;
/*모터2 변수*/

float ctrl_up = 0;
float ctrl_ui = 0;
float ctrl_ud = 0;
float ctrl_up_m2 = 0;
float ctrl_ui_m2 = 0;
float ctrl_ud_m2 = 0;
int ctrl_u = 0;
int ipwm_u = 0;
int ctrl_u_m2 = 0;
int ipwm_u_m2 = 0;
bool t2_flag = 0;
char input = NULL;
static int refSpd = 50;


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  /*모터 관련*/
  //motor1
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  //motor2
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(STBY, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_CHB), Enc1chB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), Enc2chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHB), Enc2chB_ISR, CHANGE);

  digitalWrite(M1_DIR1, LOW);
  digitalWrite(M1_DIR2, HIGH);
  digitalWrite(M2_DIR1, LOW);
  digitalWrite(M2_DIR2, HIGH);
  digitalWrite(STBY, HIGH);

  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);

  myTimer.setIntervalMs(50);
  myTimer.attachUpdateInterrupt(t2_ISR);
  myTimer.start();
}

void loop() {
  /*통신*/
  if (Serial2.available()) {
    rcv_data = Serial2.read();
    switch (rcv_count) {
      case 0:
        if ((rcv_ready == false) && (rcv_data == 0xFF)) { // 괄호 추가
          rcv_count = 1;
        } else {
          rcv_count = 0;
        }
        break;

      case 1:
        if ((rcv_ready == false) && (rcv_data == 0xFF)) { // 괄호 추가
          rcv_count = 2;
          rcv_ready = true;
        } else {
          rcv_count = 0;
        }
        break;

      case 2:
        rx_buffer[rcv_index] = rcv_data;
        rcv_index++;
        if (rcv_index > 3) {
          rcv_checksum = 0;
          for (int i = 0; i < 3; i++) {
            rcv_checksum ^= rx_buffer[i];
          }
          rcv_checksum += 1;
          if (rcv_checksum == rx_buffer[rcv_index - 1]) {
            rcv_status = true;
            memcpy((char*)rx_data, (char*)rx_buffer, 4);
          }
          rcv_count = 0;
          rcv_index = 0;
          rcv_ready = false;
        }
        break;

      default:
        rcv_count = 0;
        rcv_index = 0;
        rcv_ready = false;
        break;
    }
    if (rcv_status) {
      rcv_status = false;

      memcpy((char*)&ir1, (char*)rx_data, 1);
      memcpy((char*)&ir2, (char*)&rx_data[1], 1);
      memcpy((char*)&ir3, (char*)&rx_data[2], 1);

      // 기본 속도 설정
      if (ir2 == 0) {
        m1_ref_spd = 60;
        m2_ref_spd = 60;
        if (ir1 == 0) { // 우회전
          m1_ref_spd = 40;
          m2_ref_spd = 120;
        } else if (ir3 == 0) { // 좌회전
          m1_ref_spd = 120;
          m2_ref_spd = 60;
        }

        // 속도 제한
        m1_ref_spd = constrain(m1_ref_spd, -65, 65);
        m2_ref_spd = constrain(m2_ref_spd, -65, 65);
      }
    }
  }

  /*모터관련*/
  if (t2_flag) {
    t2_flag = 0;
    m1speed = (d_e1cnt * 10 / 11);
    m2speed = (d_e2cnt * 10 / 11);

    m1_err_spd = m1_ref_spd - m1speed;
    m2_err_spd = m2_ref_spd - m2speed;

    m1_derr_spd = m1_err_spd - m1_err_spd_k_1;
    m2_derr_spd = m2_err_spd - m2_err_spd_k_1;

    m1_err_sum = m1_err_sum + m1_err_spd;
    m2_err_sum = m2_err_sum + m2_err_spd;

    m1_err_spd_k_1 = m1_err_spd;
    m2_err_spd_k_1 = m2_err_spd;

    ctrl_up = Kp * m1_err_spd;
    ctrl_ui = Ki * m1_err_sum;
    ctrl_ud = Kd * m1_derr_spd;
    ctrl_u = (int)(ctrl_up + ctrl_ud + ctrl_ui);

    ctrl_up_m2 = Kp_m2 * m2_err_spd;
    ctrl_ui_m2 = Ki_m2 * m2_err_sum;
    ctrl_ud_m2 = Kd_m2 * m2_derr_spd;
    ctrl_u_m2 = (int)(ctrl_up_m2 + ctrl_ud_m2 + ctrl_ui_m2);

    if (ctrl_u >= 0) {
      digitalWrite(M1_DIR1, HIGH);
      digitalWrite(M1_DIR2, LOW); //ccw

      if (ctrl_u > 255)
        ipwm_u = 255;
      else
        ipwm_u = (int)ctrl_u;
    }
    else {
      digitalWrite(M1_DIR1, LOW);
      digitalWrite(M1_DIR2, HIGH); //cw

      if (ctrl_u < -255)
        ipwm_u = 255;
      else
        ipwm_u = (int)ctrl_u * (-1);
    }

    if (ctrl_u_m2 >= 0) {
      digitalWrite(M2_DIR1, HIGH);
      digitalWrite(M2_DIR2, LOW); //ccw

      if (ctrl_u_m2 > 255)
        ipwm_u_m2 = 255;
      else
        ipwm_u_m2 = (int)ctrl_u_m2;
    }
    else {
      digitalWrite(M2_DIR1, LOW);
      digitalWrite(M2_DIR2, HIGH); //cw

      if (ctrl_u_m2 < -255)
        ipwm_u_m2 = 255;
      else
        ipwm_u_m2 = (int)ctrl_u_m2 * (-1);
    }

    analogWrite(M1_PWM, ipwm_u);
    analogWrite(M2_PWM, ipwm_u_m2);
  }
}

void t2_ISR() {
  t2_flag = 1;
  e1cnt_k = e1cnt;
  e2cnt_k = e2cnt;
  d_e1cnt = e1cnt_k - e1cnt_k_1; //delta_error
  d_e2cnt = e2cnt_k - e2cnt_k_1;
  e1cnt_k_1 = e1cnt_k;
  e2cnt_k_1 = e2cnt_k;
}

/*오른쪽 바퀴 엔코더 측정*/
void Enc1chA_ISR() {
  if (digitalRead(ENC1_CHA) == HIGH) {
    if (digitalRead(ENC1_CHB) == LOW)
      e1cnt--;
    else
      e1cnt++;
  }
  else {
    if (digitalRead(ENC1_CHB) == HIGH)
      e1cnt--;
    else
      e1cnt++;
  }
}

void Enc1chB_ISR() {
  if (digitalRead(ENC1_CHB) == HIGH) {
    if (digitalRead(ENC1_CHA) == HIGH)
      e1cnt--;
    else
      e1cnt++;
  }
  else {
    if (digitalRead(ENC1_CHA) == LOW)
      e1cnt--;
    else
      e1cnt++;
  }
}

/*왼쪽 바퀴 엔코더 측정*/
void Enc2chA_ISR() {
  if (digitalRead(ENC2_CHA) == HIGH) {
    if (digitalRead(ENC2_CHB) == LOW)
      e2cnt++;
    else
      e2cnt--;
  } else {
    if (digitalRead(ENC2_CHB) == HIGH)
      e2cnt++;
    else
      e2cnt--;
  }
}

void Enc2chB_ISR() {
  if (digitalRead(ENC2_CHB) == HIGH) {
    if (digitalRead(ENC2_CHA) == HIGH)
      e2cnt++;
    else
      e2cnt--;
  } else {
    if (digitalRead(ENC2_CHA) == LOW)
      e2cnt++;
    else
      e2cnt--;
  }
}
