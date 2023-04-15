// add wifi library
// add wifi pass,user,port
WiFiServer server(port);

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print("WiFi not connected");
  }
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

}

void loop() {
  WiFiClient client = server.available();
  while(client.connected()){
    if(client.available){
      uint8_t buf;
      size_t length = 1;
      client.read(&buf, length);
      client.write(handleCmd(buf));
    }
  }
}

char* handleCmd(uint8_t cmd){
  Serial.println(cmd);
  switch(cmd){
    case 48:
      UGV_WAS_KILLED = true;
      return "UGV killed."
    default:
      return "Press 0 to kill.\n"
  }

}
