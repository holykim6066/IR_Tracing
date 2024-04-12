#include <R4Timer.h>

#define IR1 A0
#define IR2 A1
#define IR3 A2

int ir1 = 0;
int ir2 = 0;
int ir3 = 0;

byte tx_data[6];
byte temp_button = 0;
byte checksum = 0;
volatile int t2_flag = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  tx_data[0] = 0xFF;
  tx_data[1] = 0xFF;

  Timer1.initialize(50);
  Timer1.attachInterrupt(T2ISR);
  Timer1.start();

}

void loop() {
  ir1 = analogRead(IR1);
  ir2 = analogRead(IR2);
  ir3 = analogRead(IR3);

  if(ir1 > 350) ir1 = 0;
  else ir1 = 1;

  if(ir2 > 100) ir2 = 0;
  else ir2 = 1;

  if(ir3 > 120) ir3 = 0;
  else ir3 = 1;

  Serial.print(ir1);
  Serial.print('\t');
  Serial.print(ir2);
  Serial.print('\t');
  Serial.println(ir3);
  
  
  if (t2_flag) {
    t2_flag = 0;
    memcpy(&tx_data[2], &ir1, 1);
    memcpy(&tx_data[3], &ir2, 1);
    memcpy(&tx_data[4], &ir3, 1);

    checksum = 0;
    for (int i = 2; i < 6; i++) checksum ^= tx_data[i];
    checksum += 1;
    tx_data[5] = checksum;

    Serial1.write(tx_data, 6);
  }
}

void T2ISR() {
  t2_flag = 1;
}
