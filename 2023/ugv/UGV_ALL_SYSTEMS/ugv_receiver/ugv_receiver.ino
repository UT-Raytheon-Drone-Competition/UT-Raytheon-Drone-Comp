#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

#define RECEIVER 4
#define TRANSMITTER 2
#define PTT 5 //idk what this is but DONT TOUCH

RH_ASK driver(2000, TRANSMITTER, RECEIVER, PTT);
//RH_ASK driver(2000, RECEIVER,TRANSMITTER, PTT);

void setup(){
  Serial.begin(115200);	// Debugging only
    if (!driver.init()){
      Serial.println("init failed");
    }
}

void loop(){
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)){ // Non-blocking
    String rcv;
    for(int i = 0; i < buflen; i++){
      rcv += (char)buf[i];
    }
    Serial.print(rcv);
    Serial.println();
  }
}