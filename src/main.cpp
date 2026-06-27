#include <MatrixMiniR4.h>

const float DEG_PER_CM = 17.6;
const float TURN_90_DEG = 230;

const int DRIVE_SPEED = 30;
const int TURN_SPEED = 30;
const int LINE_SPEED = 25;
const float Kp = 17.5; // 15.5

void applyLineControl(bool reverse, int baseSpeed, float &filteredError) {
  float error = MiniR4.I2C0.MXLineTracer.getError();
  bool online = MiniR4.I2C0.MXLineTracer.isOnline();

  filteredError = (filteredError * 0.75f) + (error * 0.25f);

  float correction = constrain(Kp * filteredError, -18.0f, 18.0f);
  int speed = online ? baseSpeed : 15;
  int left = constrain((int)(speed + correction), -100, 100);
  int right = constrain((int)(speed - correction), -100, 100);

  if (reverse) {
    MiniR4.M1.setSpeed(-left);
    MiniR4.M2.setSpeed(right);
  } else {
    MiniR4.M1.setSpeed(left);
    MiniR4.M2.setSpeed(-right);
  }
}

void df(float cm) {
  int target = (int)(cm * DEG_PER_CM);
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  MiniR4.M1.setSpeed(DRIVE_SPEED);
  MiniR4.M2.setSpeed(-DRIVE_SPEED);
  while (abs(MiniR4.M1.getDegrees()) < target) delay(5);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(150);
}

void db(float cm) {
  int target = (int)(cm * DEG_PER_CM);
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  MiniR4.M1.setSpeed(-DRIVE_SPEED);
  MiniR4.M2.setSpeed(DRIVE_SPEED);
  while (abs(MiniR4.M1.getDegrees()) < target) delay(5);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(150);
}

void tr90() {
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  MiniR4.M1.setSpeed(TURN_SPEED);
  MiniR4.M2.setSpeed(TURN_SPEED);
  while (abs(MiniR4.M1.getDegrees()) < TURN_90_DEG) delay(5);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(150);
}

void tl90() {
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  MiniR4.M1.setSpeed(-TURN_SPEED);
  MiniR4.M2.setSpeed(-TURN_SPEED);
  while (abs(MiniR4.M1.getDegrees()) < TURN_90_DEG) delay(5);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(150);
}


void tr(float fraction) {
  int target = (int)(TURN_90_DEG * fraction);
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  MiniR4.M1.setSpeed(TURN_SPEED);
  MiniR4.M2.setSpeed(TURN_SPEED);
  while (abs(MiniR4.M1.getDegrees()) < target) delay(5);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(150);
}

void tl(float fraction) {
  int target = (int)(TURN_90_DEG * fraction);
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  MiniR4.M1.setSpeed(-TURN_SPEED);
  MiniR4.M2.setSpeed(-TURN_SPEED);
  while (abs(MiniR4.M1.getDegrees()) < target) delay(5);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(150);
}

void lineuntil() {
  float filteredError = 0;
  while (true) {
    uint8_t width = MiniR4.I2C0.MXLineTracer.getLineWidth();

    if (width == 10) {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(150);
      break;
    }

    applyLineControl(false, LINE_SPEED, filteredError);
    delay(10);
  }
}

void lineuntilBack() {
  float filteredError = 0;
  while (true) {
    uint8_t width = MiniR4.I2C0.MXLineTracer.getLineWidth();

    if (width == 10) {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(150);
      break;
    }

    applyLineControl(true, LINE_SPEED, filteredError);
    delay(10);
  }
}

void followUntilEnd() {
  float filteredError = 0;
  while (true) {
    uint8_t width = MiniR4.I2C0.MXLineTracer.getLineWidth();

    if (width == 0) {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(150);
      break;
    }

    applyLineControl(false, LINE_SPEED, filteredError);
    delay(10);
  }
}

void line(float seconds) {
  unsigned long start = millis();
  unsigned long duration = (unsigned long)(seconds * 1000);
  float filteredError = 0;
  
  while (millis() - start < duration) {
    applyLineControl(false, LINE_SPEED, filteredError);
    delay(10);
  }

  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(150);
}

void armDown() {
  MiniR4.RC1.setAngle(100);
  delay(2400);
  MiniR4.RC1.setAngle(90);
}

void armForceDown() {
  MiniR4.RC1.setAngle(100);
  delay(500);
  MiniR4.RC1.setAngle(90);
}

void armUp() {
  MiniR4.RC1.setAngle(80);
  delay(2000);
  MiniR4.RC1.setAngle(90);
}

void arm2Down() {
  MiniR4.RC2.setAngle(100);
  delay(2400);
  MiniR4.RC2.setAngle(90);
}

void arm2Up() {
  MiniR4.RC2.setAngle(80);
  delay(2000);
  MiniR4.RC2.setAngle(90);
}

void setup() {
  MiniR4.begin();
  MiniR4.PWR.setBattCell(2);
  MiniR4.RC1.begin();
  MiniR4.I2C0.MXLineTracer.begin();

  // Function Reference Table:
  // ┌─────────────────────┬──────────────────────────────────────────┐
  // │ Function            │ Description                              │
  // ├─────────────────────┼──────────────────────────────────────────┤
  // │ df(cm)              │ Drive forward specified distance         │
  // │ db(cm)              │ Drive backward specified distance        │
  // │ tr90()              │ Turn right 90 degrees                    │
  // │ tl90()              │ Turn left 90 degrees                     │
  // │ tr(fraction)        │ Turn right by fraction of 90 degrees     │
  // │ tl(fraction)        │ Turn left by fraction of 90 degrees      │
  // │ line(seconds)       │ Follow line for specified seconds        │
  // │ lineuntil()         │ Follow line until 4-way crossing         │
  // │ followUntilEnd()    │ Follow line until end                    │
  // │ armDown()           │ Lower the arm to grab object             │
  // │ armForceDown()      │ Force arm down quickly                   │
  // │ armUp()             │ Raise the arm                            │
  // └─────────────────────┴──────────────────────────────────────────┘

  // Wait for BTN_DOWN to raise arm manually
  //Serial.begin(9600);
  //Serial.println("Press BTN_DOWN to raise arm, then BTN_UP to start");
  //if (!MiniR4.BTN_DOWN.getState()) delay(10);
  //armUp();
  //delay(300);

  // Wait for BTN_UP to start mission
  while (!MiniR4.BTN_UP.getState()) delay(10);
  delay(300);

  // Exit blue square
  df(10);

  // Slight left nudge toward line
  tl(0.3);

  line(2.5);
  db(5);
  tl(1);
  
  line(3.25);
  tr(1);
    
  lineuntil();
  db(3);
  armDown();

  db(37);
  lineuntil();
  df(10);
  tr(0.9);

  line(1.5);
  lineuntil();
  df(7);
  tr(1);
  df(7);
  tl(0.5);
  df(8);
  armUp();
  tl(0.5);
  df(20);
  db(15);
  tl(1);
  line(2.5);
  lineuntil();

  df(10);
  tl(1);
  db(5);
  lineuntil();
  db(7);
  armDown();
  db(15); 
  tl(0.9);
  line(4);
  lineuntil();



















}



void loop() {}