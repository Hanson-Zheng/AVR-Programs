#include <SoftwareSerial.h>
SoftwareSerial ArduinoSlave(10,11);
String answer;
String msg;
void setup(){
  Serial.begin(9600);
  Serial.println("ENTER Commands:");
  ArduinoSlave.begin(9600);
}
void loop(){
  //Read command from monitor
  readSerialPort();
  //Read answer from slave
  while (ArduinoSlave.available()) {
    delay(10);  
    if (ArduinoSlave.available() >0) {
      char c = ArduinoSlave.read();  //gets one byte from serial buffer
      answer += c; //makes the string readString
    }
  }
  //Send data to slave
  if(msg!=""){
    Serial.print("Master sent : ");
    Serial.println(msg);
    ArduinoSlave.print(msg);
    msg="";
  }
  //Send answer to monitor
  if(answer!=""){
    Serial.print("Slave recieved : ");
    Serial.println(answer);
    answer="";
  }
}
void readSerialPort(){
  while (Serial.available()) {
    delay(10);  
    if (Serial.available() >0) {
      char c = Serial.read();  //gets one byte from serial buffer
      msg += c; //makes the string readString
    }
  }
  Serial.flush();
}
