#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(800, 3, 2, 4);

void setup() {
  Serial.begin(115200);

  // режим следования к целевй позиции
  stepper.setRunMode(FOLLOW_POS);

  // установка макс. скорости в шагах/сек
  stepper.setMaxSpeed(-30000);
  stepper.autoPower(1);
  // установка ускорения в шагах/сек/сек
  stepper.setAcceleration(10000);
stepper.setTarget(-30000,RELATIVE);
stepper.autoPower(1);
}
bool stepper_moves = 1;
void loop() {


     stepper_moves = stepper.tick();
  
}