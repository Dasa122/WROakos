#include <MatrixMiniR4.h>

const float DEG_PER_CM = 17.6;
const float TURN_90_DEG = 230;

const int DRIVE_SPEED = 30;
const int TURN_SPEED = 30;
const int LINE_SPEED = 25;
const float Kp = 15.5; // 15.5

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
  while (true) {
    float error = MiniR4.I2C0.MXLineTracer.getError();
    bool online = MiniR4.I2C0.MXLineTracer.isOnline();
    uint8_t width = MiniR4.I2C0.MXLineTracer.getLineWidth();

    if (width == 10) {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(150);
      break;
    }

    float correction = Kp * error;
    int speed = online ? LINE_SPEED : 20;
    int L = constrain((int)(speed + correction), -100, 100);
    int R = constrain((int)(speed - correction), -100, 100);
    MiniR4.M1.setSpeed(L);
    MiniR4.M2.setSpeed(-R);
    delay(10);
  }
}

void lineuntilBack() {
  while (true) {
    float error = MiniR4.I2C0.MXLineTracer.getError();
    bool online = MiniR4.I2C0.MXLineTracer.isOnline();
    uint8_t width = MiniR4.I2C0.MXLineTracer.getLineWidth();

    if (width == 10) {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(150);
      break;
    }

    float correction = Kp * error;
    int speed = online ? LINE_SPEED : 20;
    int L = constrain((int)(speed + correction), -100, 100);
    int R = constrain((int)(speed - correction), -100, 100);
    MiniR4.M1.setSpeed(-L);
    MiniR4.M2.setSpeed(R);
    delay(10);
  }
}

void followUntilEnd() {
  while (true) {
    float error = MiniR4.I2C0.MXLineTracer.getError();
    bool online = MiniR4.I2C0.MXLineTracer.isOnline();
    uint8_t width = MiniR4.I2C0.MXLineTracer.getLineWidth();

    if (width == 0) {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(150);
      break;
    }

    float correction = Kp * error;
    int speed = online ? LINE_SPEED : 20;
    int L = constrain((int)(speed + correction), -100, 100);
    int R = constrain((int)(speed - correction), -100, 100);
    MiniR4.M1.setSpeed(L);
    MiniR4.M2.setSpeed(-R);
    delay(10);
  }
}

void line(float seconds) {
  unsigned long start = millis();
  unsigned long duration = (unsigned long)(seconds * 1000);
  
  while (millis() - start < duration) {
    float error = MiniR4.I2C0.MXLineTracer.getError();
    bool online = MiniR4.I2C0.MXLineTracer.isOnline();
    uint8_t width = MiniR4.I2C0.MXLineTracer.getLineWidth();

    float correction = Kp * error;
    int speed = online ? LINE_SPEED : 20;
    int L = constrain((int)(speed + correction), -100, 100);
    int R = constrain((int)(speed - correction), -100, 100);
    MiniR4.M1.setSpeed(L);
    MiniR4.M2.setSpeed(-R);
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
  delay(300);

  // Follow line until 4-way crossing
  line(4.5);
  lineuntil();

  // Position for mission
  df(10);
  tr90();
  db(3.5);

  // Lower cage to capture object
  armDown();

  // Travel w Object
  tl(0.9);
  db(10);//15
  tr(0.1);
  df(5);
  line(4);
  lineuntil();
  tl(1.30);

  tr(0.15);
  db(3);
  armUp();
  tr(0.5);
  df(4);
  line(2);
  lineuntil();


  df(10);
  tr(0.9);
  db(10);
  line(2);
  lineuntil();
  db(2);
  armDown();

  //lobalozas

  tr(0.1);
  armForceDown();
  tl(0.2);
  armForceDown();
  tr(0.2);
  armForceDown();
  tl(0.1);
  armForceDown();
  df(2);


  db(10);
  tl(2);
  delay(300);
  lineuntil();
  db(6);

  //visszamenetel
  armUp();
  db(25);
  tr(0.9);

  // sarga
  df(25);
  tr(1);
  db(10);
  delay(300);
  line(0.5);
  lineuntil();
  db(2);
 
  //lobalozas

  armDown();
  tr(0.1);
  armForceDown();
  tl(0.2);
  armForceDown();
  tr(0.2);
  armForceDown();
  tl(0.1);
  armForceDown();
  df(3);

  tr(1.9);
  db(17);
  line(2);

  df(16);
  tl(1);
  lineuntil();
  line(3.6);
  tr(0.2);
  df(2);
  tr(0.5);
  line(1);
  lineuntil(); 

  df(10);
  tr(0.9);
  lineuntil();

  //sargatol vissza

  db(6);
  armUp();

  db(1);
  tl(0.9);
  line(4);
  lineuntil();
  db(10);
  tr(1);
  line(3);





















}



void loop() {}