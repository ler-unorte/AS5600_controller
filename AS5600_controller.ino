#include <Wire.h>

#define AS5600_ADDR 0x36  // Endereço I2C do sensor AS5600

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Configuração inicial do AS5600
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0B);  // Endereço do registro para configurar a resolução (12 bits)
  Wire.write(0x0C);  // Valor para configurar a resolução
  Wire.endTransmission();
}

void loop() {
  // Realiza leitura do sensor AS5600
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);  // Endereço do registro para ler a posição angular
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);  // Solicita 2 bytes de dados
  if (Wire.available() >= 2) {
    int msb = Wire.read();
    int lsb = Wire.read();
    int position = (msb << 8) | lsb;  // Combina os bytes lidos

    // Calcula a posição absoluta em graus (considerando 360 graus completos)
    float absolutePosition = position * 360.0 / 4096.0;

    // Exibe a posição absoluta no monitor serial
    Serial.print("Posição absoluta: ");
    Serial.print(absolutePosition);
    Serial.println(" graus");
  }

  delay(500);
}
