#if defined(ESP32)

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

/**************************************************************************************/
/***************       ESP32 LEDC PWM Configuration                 *******************/
/**************************************************************************************/

// PWM Configuration
// Note: ESP32 Arduino Core 3.x uses pin numbers directly in ledcWrite()
// instead of channel numbers (legacy API used channels)
#define PWM_FREQUENCY 32000  // 32kHz for brushed motors (can be adjusted)
#define PWM_RESOLUTION 10    // 10-bit resolution (0-1023)

// Motor pins for ESP32 MH ET Live MiniKit
uint8_t ESP32_PWM_PIN[8] = {
  ESP32_MOTOR1_PIN,  // Motor 1 - GPIO13
  ESP32_MOTOR2_PIN,  // Motor 2 - GPIO25
  ESP32_MOTOR3_PIN,  // Motor 3 - GPIO14
  ESP32_MOTOR4_PIN,  // Motor 4 - GPIO27
  0, 0, 0, 0         // Additional motors if needed
};

/**************************************************************************************/
/***************       Initialize ESP32 LEDC PWM                    *******************/
/**************************************************************************************/
void initOutput() {
  // Configure LEDC PWM for each motor
  // ESP32 Arduino Core 3.x uses ledcAttach instead of ledcSetup+ledcAttachPin
  for (uint8_t i = 0; i < 4; i++) {
    ledcAttach(ESP32_PWM_PIN[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcWrite(ESP32_PWM_PIN[i], 0);  // Start with motors off
  }

  // Configure buzzer PWM
  ledcAttach(ESP32_BUZZER_PIN, 2000, 8);  // 2kHz, 8-bit resolution for buzzer
  ledcWrite(ESP32_BUZZER_PIN, 0);

  // Configure LED pin
  pinMode(ESP32_LED_PIN, OUTPUT);
  digitalWrite(ESP32_LED_PIN, LOW);

  Serial.println("ESP32 PWM Output Initialized");
}

/**************************************************************************************/
/***************  Writes the Motors values to LEDC PWM              *******************/
/**************************************************************************************/
void writeMotors() {
  // ESP32 motor control using LEDC
  // Input range: [1000:2000] => Output range: [0:1023] for 10-bit PWM

  #if (NUMBER_MOTOR > 0)
    // Convert from servo range (1000-2000) to PWM duty cycle (0-1023)
    uint16_t pwm0 = map(motor[0], 1000, 2000, 0, 1023);
    ledcWrite(ESP32_MOTOR1_PIN, pwm0);
  #endif

  #if (NUMBER_MOTOR > 1)
    uint16_t pwm1 = map(motor[1], 1000, 2000, 0, 1023);
    ledcWrite(ESP32_MOTOR2_PIN, pwm1);
  #endif

  #if (NUMBER_MOTOR > 2)
    uint16_t pwm2 = map(motor[2], 1000, 2000, 0, 1023);
    ledcWrite(ESP32_MOTOR3_PIN, pwm2);
  #endif

  #if (NUMBER_MOTOR > 3)
    uint16_t pwm3 = map(motor[3], 1000, 2000, 0, 1023);
    ledcWrite(ESP32_MOTOR4_PIN, pwm3);
  #endif

  // For additional motors (hexacopter, octocopter), add more GPIO pins here
}

/**************************************************************************************/
/***************         Servo support for ESP32 (if needed)        *******************/
/**************************************************************************************/
void writeServos() {
  #if defined(SERVO)
    // If servos are needed, implement LEDC-based servo control here
    // Servos typically use 50Hz frequency and pulse width 1000-2000µs
  #endif
}

/**************************************************************************************/
/***************              Buzzer Control                        *******************/
/**************************************************************************************/
void buzzerOn() {
  ledcWrite(ESP32_BUZZER_PIN, 128);  // 50% duty cycle for buzzer
}

void buzzerOff() {
  ledcWrite(ESP32_BUZZER_PIN, 0);
}

/**************************************************************************************/
/***************            Motor Mixing Table (QUADX)              *******************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;

  // PIDMIX macro: combines throttle, roll, pitch, and yaw commands
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  // QUADX configuration (X-frame quadcopter)
  #if defined(QUADX)
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #else
    // For other frame types, implement mixing here or include from Output.cpp
    #error "Only QUADX frame is currently supported for ESP32"
  #endif

  // Limit motor values to valid range [MINTHROTTLE:MAXTHROTTLE]
  maxMotor=motor[0];
  for(i=1;i< NUMBER_MOTOR;i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for(i=0;i< NUMBER_MOTOR;i++) {
    if (maxMotor > MAXTHROTTLE)
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
    if ((rcData[THROTTLE] < MINCHECK))
      #ifndef MOTOR_STOP
        motor[i] = conf.minthrottle;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (!f.ARMED)
      motor[i] = MINCOMMAND;
  }
}

// Note: blinkLED() is defined in Alarms.cpp for all platforms

#endif // ESP32
