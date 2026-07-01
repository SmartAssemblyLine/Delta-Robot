#include <AccelStepper.h>

#define EN_PIN     15
#define BUTTON_PIN A4   // change to your pin
#define PUMP_PIN   A0   // change to your pin
#define PUMP_IN2  A1 

AccelStepper stepper0(AccelStepper::DRIVER, 4, 5);
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 6, 7);

AccelStepper* steppers[3] = {&stepper0, &stepper1, &stepper2};

const int   LIMIT_PINS[3] = {10, 8, 9};  // fill your first pin
const bool  INVERT_DIR[3] = {false, true, true};
const float MAX_SPEED     = 8000.0;
const float MAX_ACCEL     = 6000.0;
const float HOME_SPEED    = 600.0;

bool joint_homed[3] = {false, false, false};
bool is_homed       = false;
bool started        = false;
bool moving         = false;      // true while executing a trajectory
static String buf   = "";

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN,     OUTPUT);
  digitalWrite(EN_PIN, LOW);
  

  // pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PUMP_IN2, OUTPUT);
  digitalWrite(PUMP_IN2, LOW);


  pinMode(PUMP_PIN,   OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  for (int i = 0; i < 3; i++) {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
    steppers[i]->setMaxSpeed(HOME_SPEED);
    steppers[i]->setAcceleration(MAX_ACCEL);
    steppers[i]->setCurrentPosition(0);
    steppers[i]->setPinsInverted(INVERT_DIR[i], false, false);
  }

  while (!started) {
    Serial.println("READY");
    unsigned long t = millis();
    while (millis() - t < 500) {
      waitForStart();
      if (started) break;
      delay(10);
    }
  }

  for (int i = 0; i < 3; i++) {
    steppers[i]->moveTo(100000);
  }
}

// ─────────────────────────────────────────────────────────
void loop() {
  if (!is_homed) {
    doHoming();
  } else {
    checkButton();
    readSerial();

    if (moving) {
      bool all_done = true;
      for (int i = 0; i < 3; i++) {
        if (steppers[i]->distanceToGo() != 0) {
          all_done = false;
          break;
        }
      }
      if (all_done) {
        moving = false;
        Serial.println("DONE");
      }
    }
    
    // checkDone();   // ← sends "DONE" when all motors reach target
  }

  for (int i = 0; i < 3; i++) steppers[i]->run();
}

// ─────────────────────────────────────────────────────────
void waitForStart() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') {
      started = true;
      Serial.flush();
      return;
    }
  }
}



// ─────────────────────────────────────────────────────────
void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("PICK");
      // wait for release
      while (digitalRead(BUTTON_PIN) == LOW);
    }
  }
}

// ─────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────
void doHoming() {
  for (int i = 0; i < 3; i++) {
    if (!joint_homed[i] && digitalRead(LIMIT_PINS[i]) == LOW) {
      steppers[i]->setSpeed(0);
      steppers[i]->moveTo(steppers[i]->currentPosition());
      steppers[i]->setCurrentPosition(0);
      joint_homed[i] = true;
    }
  }

  bool all_triggered = joint_homed[0] && joint_homed[1] && joint_homed[2];
  if (all_triggered) {
    // for (int i = 0; i < 3; i++) {
    //   steppers[i]->moveTo(-1000);
    // }
    steppers[0]->moveTo(-1000);
    steppers[1]->moveTo(-1000);
    steppers[2]->moveTo(-1100);
    delay(100);

    bool all_done = false;
    while (!all_done) {
      all_done = true;
      for (int i = 0; i < 3; i++) {
        steppers[i]->run();
        if (steppers[i]->distanceToGo() != 0) all_done = false;
      }
    }

    for (int i = 0; i < 3; i++) {
      steppers[i]->setCurrentPosition(0);
      steppers[i]->setMaxSpeed(MAX_SPEED);
    }

    is_homed = true;
    Serial.println("HOMED");
  }
}

// ─────────────────────────────────────────────────────────
void readSerial() {
  while (Serial.available() && is_homed==true) {
    char c = Serial.read();

    for (int i = 0; i < 3; i++) steppers[i]->run();

    if (c == '\n') {
      // ── Check for pump commands ───────────────────────
      if (buf == "PUMP_ON") {

        // Serial.println("pump on");
        digitalWrite(PUMP_PIN, HIGH);
        digitalWrite(PUMP_IN2, LOW);   
        buf = "";
        return;
      }
      if (buf == "PUMP_OFF") {
        // Serial.println(" pump off");
        digitalWrite(PUMP_PIN, LOW);
        digitalWrite(PUMP_IN2, LOW);   
        buf = "";
        return;
      }
      
      // ── Parse 3 step values ───────────────────────────
      // Serial.print("motors steps are : ");
      // Serial.println(buf);
      long values[3] = {0, 0, 0};
      int  idx       = 0;
      String token   = "";

      for (int i = 0; i <= (int)buf.length() && idx < 3; i++) {
        if (i == (int)buf.length() || buf[i] == ' ') {
          if (token.length() > 0) {
            values[idx++] = token.toInt();
            token = "";
          }
        } else {
          token += buf[i];
        }
      }

      if (idx == 3) {
        for (int j = 0; j < 3; j++) {
          steppers[j]->moveTo(values[j]);
        }
        moving = true;   // ← start watching for DONE
      }
      buf = "";

    } else if (c != '\r') {
      buf += c;
    }
  }
}