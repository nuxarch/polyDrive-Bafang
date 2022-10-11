#include <SimpleFOC.h>
HallSensor sensor = HallSensor(32, 35, 34, 50);
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

void setup() {
  Serial.begin(115200);
  sensor.pullup = Pullup::USE_EXTERN;
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  sensor.update();
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}