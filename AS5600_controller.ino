#include <Wire.h>
#define SENSOR_ADDRESS 0x36
#define DIR_PIN 2
#define STEP_PIN 3

float stepsPerDegree = 0.0879;
bool isMotorMoving = false;

int initialPosition = 0;
int actualPosition = 0;

float initialAngle = 0;
float actualAngle = 0;
int desiredAngle = 0;

void setup() {

  Wire.begin();
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  Serial.begin(9600);

  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(0x0E);
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDRESS, 2);

  initialPosition = Wire.read() << 8 | Wire.read();
  initialAngle = calculateCurrentAngle(initialPosition);

  actualPosition = initialPosition;
  actualAngle = initialAngle;
  desiredAngle = initialAngle;
}

void loop() {

  if (Serial.available() > 0) {
    desiredAngle = Serial.parseInt();
    desiredAngle = constrain(desiredAngle, 0, 360);
    isMotorMoving = true;
  }

  int currentPosition = getCurrentPosition();
  float currentAngle = calculateCurrentAngle(currentPosition);

  Serial.print("Angulo atual: ");
  Serial.print(currentAngle, 2);
  Serial.println(" graus");

  Serial.print("Angulo desejado: ");
  Serial.print(desiredAngle);
  Serial.println(" graus");

  if (isMotorMoving) {
    int currentPosition = getCurrentPosition();
    int desiredPosition = calculateDesiredPosition(desiredAngle);

    Serial.println("-------------------");
    Serial.print("Posicao atual: ");
    Serial.println(currentPosition);
    Serial.print("Posicao desejado: ");
    Serial.println(desiredPosition);
    Serial.println("-------------------");

    if (((currentPosition - desiredPosition) > 5) || ((desiredPosition - currentPosition) > 5)) {
      moveMotorToPosition(desiredPosition);
    } else {
      stopMotor();
      isMotorMoving = false;
    }
  }

  delay(500);
}

int calculateDesiredPosition(int angle) {
  return map(angle, 0, 360, 0, 4095);
}


void moveMotorToPosition(int desiredPosition) {

  Serial.println("---- ATIVANDO MOTOR ------- ");

  int currentPosition = getCurrentPosition();

  Serial.print("Angulo atual: ");
  Serial.println(actualAngle);
  Serial.print("Angulo desejada: ");
  Serial.println(desiredAngle);

  Serial.print("Posicao atual: ");
  Serial.println(currentPosition);
  Serial.print("Posicao desejada: ");
  Serial.println(desiredPosition);

  int direction = (desiredPosition > currentPosition) ? HIGH : LOW;
  digitalWrite(DIR_PIN, direction);

  Serial.print("Direcao: ");
  Serial.println((desiredPosition > currentPosition) ? "Horario" : "Anti-Horario");

  int steps = 0;

  if (desiredPosition > currentPosition) {
    steps = abs((desiredPosition - currentPosition) * 0.788177339);
  } else {
    steps = abs((currentPosition - desiredPosition) * 0.788177339);
  }

  Serial.print("Numero de passos: ");
  Serial.println(steps);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(200);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(200);
    currentPosition = getCurrentPosition();
  }

  actualPosition = getCurrentPosition();
  actualAngle = calculateCurrentAngle(actualPosition);

  Serial.println("------------------ ");

  Serial.print("Angulo atual: ");
  Serial.println(actualAngle);
  Serial.print("Angulo desejada: ");
  Serial.println(desiredAngle);

  Serial.print("Posicao atual: ");
  Serial.println(actualPosition);
  Serial.print("Angulo atual: ");
  Serial.println(actualAngle, 2);

  Serial.println("------ FIM ------- ");
}

void stopMotor() {
  digitalWrite(STEP_PIN, LOW);
}

int getCurrentPosition() {
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(0x0E);
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDRESS, 2);
  int currentPosition = Wire.read() << 8 | Wire.read();
  return currentPosition;
}

float calculateCurrentAngle(int currentPosition) {
  float currentAngle = map(currentPosition, 0, 4095, 0, 360);
  return currentAngle;
}
