// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module

#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

RH_ASK driver(2000, 2, 4, 5);

void setup()
{
    Serial.begin(115200);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {
	    //int i;

	    // Message with a good checksum received, dump it.
	    //driver.printBuffer("Got:", buf, buflen);

      String rcv;
      for(int i = 0; i < buflen; i++){
        rcv += (char)buf[i];
      }
      Serial.print(rcv);
      Serial.println();
    }
}