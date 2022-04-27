#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#define DC_Motor_LEFT 10, 11
uint8_t servonum = 2;
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();




void setup() {
    Serial.begin(115200);

    // Motors
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(1000);
    Wire.setClock(400000);
}
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  pulselength /= 4096;  // 12 bits of resolution
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
}

// 'state' is the motor directions, the real direction also depends on how
// mottor wires are connected to the motorshield
void setPWM(int chan1, int chan2, bool state, uint16_t val) {
    // Serial.print(val);
    // Serial.print("\t");
    // Serial.println(state);
    if (state)  // state = 1 clockwise rotation
    {
        pwm.setPWM(chan1, 0, 4095);
        pwm.setPWM(chan2, val, 4095 - val);
    } else  // state = 0 couter-clockwise rotation
    {
        pwm.setPWM(chan2, 0, 4095);
        pwm.setPWM(chan1, val, 4095 - val);
    }
}

// Value range: [-100, 100]
// 0 -> 100: Forward
// 0 -> -100: Backward
void controlWheels(int16_t leftWheel) {
    leftWheel = leftWheel * 40.96;
    setPWM(DC_Motor_LEFT, 0x8000 & leftWheel, abs(leftWheel));
}

void loop() {
  pwm.setPWM(servonum, 0, 300);
  delay(500);
  pwm.setPWM(servonum, 0, 450);
  delay(500);
  pwm.setPWM(servonum, 0, 150);
  delay(500);
}
