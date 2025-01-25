#include "BluetoothSerial.h"   // library
 #include <ESP32Servo.h>
static const int servoPin = 5;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

Servo servo1;

BluetoothSerial SerialBT;   // 

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
   pinMode(15, OUTPUT);
  pinMode(2, OUTPUT);
   pinMode(4, OUTPUT);
    pinMode(17, OUTPUT);
     pinMode(26, OUTPUT);
       pinMode(2, OUTPUT);
        pinMode(18, OUTPUT);
         pinMode(19, OUTPUT);
         pinMode(23, OUTPUT);
         pinMode(22, OUTPUT);
         digitalWrite(22, HIGH);
         delay(500);
         digitalWrite(23, HIGH);
         delay(500);
         digitalWrite(22, LOW);
         delay(500);
         digitalWrite(23, LOW);
         delay(500);
        servo1.attach(servoPin);
        //myServo.attach(ServoMotor);
        //myServo.write(0);
        servo1.write(0);
}
// error detection
uint8_t calculate_checksum(uint8_t *data) {
  uint8_t checksum = 0;
  checksum |= 0b11000000 & data[1];
  checksum |= 0b00110000 & data[2];
  checksum |= 0b00001100 & data[3];
  checksum |= 0b00000011 & data[4];
  return checksum;
}

void loop() {
  uint8_t recv_data[6];
  if (SerialBT.available()) {
    SerialBT.readBytes(recv_data, 6);
    digitalWrite(22, HIGH);
    if (recv_data[0] != 'T') {
      Serial.print("Receive error!");
      digitalWrite(22, LOW);
      return;
    }

    if (recv_data[5] != calculate_checksum(recv_data)) {
      Serial.print("Decode error!");
      return;
    }
    Serial.printf("left_x: %d, left_y: %d, right_x: %d, right_y: %d\n", recv_data[1], recv_data[2], recv_data[3], recv_data[4]);
  int rithvik = recv_data[1];
  Serial.print("rithvik =  ");
Serial.println(rithvik);
if(rithvik < 10){
     digitalWrite(15, HIGH);
  digitalWrite(2, HIGH);
   digitalWrite(4, HIGH);
    digitalWrite(17, HIGH);
     digitalWrite(26, HIGH);
       digitalWrite(2, HIGH);
        digitalWrite(18, HIGH);
         digitalWrite(19, HIGH);
         servo1.write(0);


}
else {
       digitalWrite(15, LOW);
  digitalWrite(2, LOW);
   digitalWrite(4, LOW);
    digitalWrite(17, LOW);
     digitalWrite(16, LOW);
       digitalWrite(2, LOW);
        digitalWrite(18, LOW);
         digitalWrite(19, LOW);
         servo1.write(180); 

}


if(rithvik > 10 && rithvik < 20 ){
     digitalWrite(15, LOW);
  digitalWrite(2, HIGH);
   digitalWrite(4, HIGH);
    digitalWrite(17, HIGH);
     digitalWrite(26, HIGH);
      digitalWrite(5, HIGH);
       digitalWrite(2, HIGH);
        digitalWrite(18, HIGH);
         digitalWrite(19, HIGH);
          // Move the servo to 180 degrees
   // delay(1000);        // Wait for 1 second
  //  servo1.write(0);
}
else {
       digitalWrite(15, LOW);
  digitalWrite(2, LOW);
   digitalWrite(4, LOW);
    digitalWrite(17, LOW);
     digitalWrite(26, LOW);
      digitalWrite(5, LOW);
       digitalWrite(2, LOW);
        digitalWrite(18, LOW);
         digitalWrite(19, LOW);
         // Move the servo to 180 degrees
     //    delay(1000);        // Wait for 1 second
      //   servo1.write(0);
}
  }
  delay(20);
}

/*

   pinMode(15, OUTPUT);
  pinMode(2, OUTPUT);
   pinMode(4, OUTPUT);
    pinMode(17, OUTPUT);
     pinMode(16, OUTPUT);
      pinMode(5, OUTPUT);
       pinMode(2, OUTPUT);
        pinMode(18, OUTPUT);
         pinMode(19, OUTPUT);
         */# SafeCross
