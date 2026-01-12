// --------------------
// IME12 Sensor Reader
// 0 = ACTIVE
// 1 = INACTIVE
// Output format: x,x
// --------------------

const int sensor1Pin = 22;
const int sensor2Pin = 23;

// Set this to true if your sensors are NPN
// false = PNP (most common)
const bool NPN_SENSOR = false;

void setup() {
  Serial.begin(115200);

  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);

  Serial.println("IME12 serial output started");
}

int readSensor(int pin) {
  int raw = digitalRead(pin);

  // Convert raw logic to ACTIVE / INACTIVE
  bool active;

  if (NPN_SENSOR) {
    active = (raw == LOW);   // NPN active pulls LOW
  } else {
    active = (raw == HIGH);  // PNP active drives HIGH
  }

  // 0 = active, 1 = inactive
  return active ? 0 : 1;
}

void loop() {
  int s1 = readSensor(sensor1Pin);
  int s2 = readSensor(sensor2Pin);

  // Send as: x,x
  Serial.print(s1);
  Serial.print(",");
  Serial.println(s2);

  delay(100);  // 10 Hz update
}
