
#define RIGHT_XOR 2
#define LEFT_XOR 3
#define RIGHT_A 4
#define RIGHT_B 5
#define LEFT_A 6
#define LEFT_B 7
#define RIGHT_PH 8
#define RIGHT_PWM 9
#define LEFT_PWM 10
#define LEFT_PH 11
#define LED_OUT 12
#define VIN_READ A0

#define VLOW_THRESH 10.0 // volts
#define RATE 20 // millis
#define DEAD_COUNT_THRESH 50 // loop counts to wait before canceling movement
#define VEL_P 150 // bits per m/s err
#define PACKET_LEN 8 // two floats
#define SYNC_BYTE 0xAA // Unlikely to show up in our real data

#define WHEEL_RATIO 0.3192 // meters per revolution
#define TRACK_WIDTH 0.2646 // meters
#define COUNTS_PER_REV 280 // count/rev
#define DIST_PER_COUNT 0.000285 // m per count
#define RPS 2.333 // revolutions per second free speed
#define TOP_SPEED 0.7448 // meters per second free speed
#define BITS_PER_MSEC 342.37379 // bits/(meter/sec)
#define VIN_PER_BIT 0.01798 // battery voltage per adc bit

void isrRightXor();
void isrLeftXor();
static inline int8_t sgn(int val);
void setMotor(int phPin, int pwmPin, int value);

volatile long countLeft = 0;
volatile long countRight = 0;
volatile byte lastLeftA;
volatile byte lastLeftB;
volatile byte lastRightA;
volatile byte lastRightB;

unsigned long lastTime;
long lastCountLeft = 0;
long lastCountRight = 0;
float cmdFwd = 0;
float cmdRot = 0;
int deadCounts = 0;

void setup() {
  pinMode(RIGHT_XOR, INPUT);
  pinMode(LEFT_XOR, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);
  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_PH, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_PH, OUTPUT);
  pinMode(LED_OUT, OUTPUT);

  // Set PWM frequency to 32k
  TCCR1B = TCCR1B & B11111000 | B00000001;

  attachInterrupt(digitalPinToInterrupt(RIGHT_XOR), isrRightXor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_XOR), isrLeftXor, CHANGE);

  Serial.begin(115200);
  lastTime = millis();
}

void loop() {
  unsigned long curTime = millis();
  if (curTime - lastTime > RATE) {
    lastTime = curTime;
    // Read voltage input and determine if it's too low
    int vin = analogRead(VIN_READ);
    float voltage = ((float) vin) * VIN_PER_BIT;
    if (voltage <= VLOW_THRESH) {
      digitalWrite(LED_OUT, HIGH);
    }
    else {
      digitalWrite(LED_OUT, LOW);
    }

    // Calculate speed of each wheel from encoder
    int countLeftDiff = countLeft - lastCountLeft;
    float leftSpeedCounts = 1000 * ((float) countLeftDiff) / RATE;
    float leftSpeedmsec = leftSpeedCounts * DIST_PER_COUNT;
    int countRightDiff = countRight - lastCountRight;
    float rightSpeedCounts = 1000 * ((float) countRightDiff) / RATE;
    float rightSpeedmsec = rightSpeedCounts * DIST_PER_COUNT;
    lastCountRight = countRight;
    lastCountLeft = countLeft;

    float velFwd = (rightSpeedmsec + leftSpeedmsec) / 2;
    float velRot = (rightSpeedmsec - leftSpeedmsec) / TRACK_WIDTH;

    // Read in command speed
    if (Serial.available() >= 8) {
      deadCounts = 0;
      // Read directly into command memory
      Serial.readBytes((byte*) &cmdFwd, sizeof(float));
      Serial.readBytes((byte*) &cmdRot, sizeof(float));
    }
    else {
      ++deadCounts;
    }
    if (deadCounts > DEAD_COUNT_THRESH) {
      // We haven't heard from the host in a while, might be dead or frozen
      // Fail to no movement
      cmdFwd = 0;
      cmdRot = 0;
    }
    
    float speedRight = -(cmdFwd + cmdRot * TRACK_WIDTH / 2);
    float speedLeft = (cmdFwd - cmdRot * TRACK_WIDTH / 2);
    
    int cmdRight = speedRight * BITS_PER_MSEC;// + VEL_P * (speedRight - rightSpeedmsec);
    int cmdLeft = speedLeft * BITS_PER_MSEC + VEL_P * (speedLeft - leftSpeedmsec);
    setMotor(RIGHT_PH, RIGHT_PWM, cmdRight);
    setMotor(LEFT_PH, LEFT_PWM, cmdLeft);
    
    // Write out
    /*Serial.print(voltage);
    Serial.print(" Speed: ");
    Serial.print(velFwd);
    Serial.print(" Rotate: ");
    Serial.print(velRot);
    Serial.print(" CountR ");
    Serial.print(countRight);
    Serial.print(" CountL ");
    Serial.print(countLeft);
    Serial.print(" SpeedR ");
    Serial.print(rightSpeedmsec);
    Serial.print(" SpeedL ");
    Serial.println(leftSpeedmsec);*/
    Serial.write((byte*) &velFwd, sizeof(float));
    Serial.write((byte*) &velRot, sizeof(float));
    Serial.write((byte*) &voltage, sizeof(float));
    // Write two sync bytes to denote end
    Serial.write(SYNC_BYTE);
    Serial.write(SYNC_BYTE);
  }
}

void setMotor(int phPin, int pwmPin, int value) {
  if (sgn(value) < 0) {
    digitalWrite(phPin, HIGH);
  }
  else {
    digitalWrite(phPin, LOW);
  }

  analogWrite(pwmPin, abs(value));
}

void isrRightXor() {
  byte newRightB = digitalRead(RIGHT_B);
  byte newRightA = digitalRead(RIGHT_XOR) ^ newRightB;
  countRight += (newRightA ^ lastRightB) - (lastRightA ^ newRightB);
  if ((lastRightA ^ newRightA) & (lastRightB ^ newRightB)) {
    // error
  }
  lastRightA = newRightA;
  lastRightB = newRightB;
}

void isrLeftXor() {
  byte newLeftB = digitalRead(LEFT_B);
  byte newLeftA = digitalRead(LEFT_XOR) ^ newLeftB;
  countLeft -= (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB);
  if ((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB)) {
    // error
  }
  lastLeftA = newLeftA;
  lastLeftB = newLeftB;
}

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
