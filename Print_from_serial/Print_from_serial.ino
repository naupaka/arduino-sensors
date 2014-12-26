void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  
  // put your main code here, to run repeatedly:
  while (Serial.available() <= 0) {
    Serial.println("Hello world");
    delay(300);
  }
  
  Serial.println("Goodbye world");
  while(1) { }
}
