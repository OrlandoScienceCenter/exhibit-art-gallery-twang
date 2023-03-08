void setup() {
  Serial.begin(115200);
  
  Serial3.begin(9600);
}

void loop() {
  int incomingByte;

  if (Serial3.available() > 0) {
    incomingByte = Serial3.read();
    
    Serial.print("UART received: ");
    Serial.println(char(incomingByte));
  }
}
