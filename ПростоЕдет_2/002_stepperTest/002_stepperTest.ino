
#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper1(800, 19, 18, 17);
GStepper<STEPPER2WIRE> stepper2(800, 23, 22, 21);
//GStepper<STEPPER2WIRE> stepper(steps, step, dir, en); 

void setup() {
  Serial.begin(115200);


  // режим следования к целевй позиции
  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setRunMode(FOLLOW_POS);


  stepper1.setMaxSpeed(200);
  stepper1.setAcceleration(300);

  stepper2.setMaxSpeed(200);
  stepper2.setAcceleration(300);

  // отключать мотор при достижении цели
  stepper1.autoPower(true);
  stepper2.autoPower(true);

  // включить мотор (если указан пин en)
  stepper1.enable();
  stepper2.enable();
  stepper1.setTarget(1000,RELATIVE);
  stepper2.setTarget(1000,RELATIVE);
}

void loop() {

stepper1.tick();
stepper2.tick();
}
