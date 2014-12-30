#include <Wire.h>

void writeI2CRegister8bit(int addr, int value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

unsigned int readI2CRegister16bit(int addr, int reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(addr, 2);
  unsigned int t = Wire.read() << 8;
  t = t | Wire.read();
  return t;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  writeI2CRegister8bit(0x20, 6); //reset
}

void loop() {
  Serial.print(readI2CRegister16bit(0x20, 0)); //read capacitance register
  Serial.print(", ");
  Serial.println(readI2CRegister16bit(0x20, 5)); //temperature register
  // Serial.print(", ");
  // writeI2CRegister8bit(0x20, 3); //request light measurement 
  // Serial.println(readI2CRegister16bit(0x20, 4)); //read light register
  delay(1000);
}
