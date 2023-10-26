int addTwoInts(int a, int b){
  return a + b;
}

void setup() {
  Serial.begin(9600);
  int result = addTwoInts(4,3);
  Serial.println(result);
}

void loop() {


}
