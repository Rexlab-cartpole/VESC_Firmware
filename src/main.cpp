#include <Arduino.h>
#include <board_vesc_4_12.h>
#include <SimpleFOC.h>

#define MOTOR_PENDULUM 1
// #define MOTOR_RAIL 1

#ifdef MOTOR_RAIL
  #define CPR_ENCODER 540
#endif

#ifdef MOTOR_PENDULUM
  #define CPR_ENCODER 600
#endif

#define MOTOR_KV 300
#define MOTOR_POLES 12
#define MOTOR_R 0.340
#define MOTOR_L 0.000216

#define MAX_TORQUE 10
#define MIN_DUTY_PERIOD_PWM 0
#define MAX_DUTY_PERIOD_PWM 1000

#define CURRENT_SENSE_GAIN 10
#define CURRENT_SENSE_SHUNT 0.001

#define INPUT_VOLTAGE 24
#define CALIBRATION_VOLTAGE 1
#define MAX_RAD_PER_SEC 100

#define LOW_PASS_CONSTANT 0.1

#define TORQUE_IN PA5
#define MOTOR_EN PA6

Encoder sensor = Encoder(ENC_A, ENC_B, CPR_ENCODER);
HardwareSerial Serial6(USART6_RX, USART6_TX);
BLDCMotor motor = BLDCMotor(MOTOR_POLES, MOTOR_R, MOTOR_KV, MOTOR_L);
BLDCDriver6PWM driver = BLDCDriver6PWM(H1, L1, H2, L2, H3, L3, EN_GATE);
LowsideCurrentSense current_sense  = LowsideCurrentSense(CURRENT_SENSE_SHUNT, CURRENT_SENSE_GAIN, BR_SO1, BR_SO2);

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }

volatile long servo_duty_period_micros = 0; // typically 800us to 2200us

static float target = 0.0f;
float get_torque(){
  uint32_t input = analogRead(TORQUE_IN);

  // do a manual map 
  float ratio = (float)(input - MIN_DUTY_PERIOD_PWM) / (float)(MAX_DUTY_PERIOD_PWM - MIN_DUTY_PERIOD_PWM);
  float new_target = ratio * (MAX_TORQUE - (-MAX_TORQUE)) + (-MAX_TORQUE);
  target = (LOW_PASS_CONSTANT * new_target + ((1 - LOW_PASS_CONSTANT) * target));
  return target;
}

void setup()
{
  Serial6.begin(115200);
  delay(1000);
  Serial6.println("Setup Triggered");

  pinMode(TORQUE_IN, INPUT);
  pinMode(MOTOR_EN, INPUT);
  // attachInterrupt(SERVO, onServo, CHANGE);

  sensor.enableInterrupts(doA, doB);
  sensor.init();

  motor.linkSensor(&sensor);

  driver.voltage_power_supply = INPUT_VOLTAGE;
  motor.useMonitoring(Serial6);

  driver.init();

  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 1;
  motor.voltage_limit = INPUT_VOLTAGE;  // [V]
  motor.velocity_limit = MAX_RAD_PER_SEC; // [rad/s]

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.output_ramp = 200;
  motor.P_angle.P = 2;

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage; // Torque control type

  // current_sense.linkDriver(&driver);

  motor.init();

  // current_sense.init();
  // motor.linkCurrentSense(&current_sense);
  current_sense.gain_a *= -1;
  current_sense.gain_b *= -1;
  current_sense.gain_c *= -1;

  motor.initFOC();

  int torque_zero_counter = 0;
  int torque_zero_counter_max = 200;
  float torque_zero_threshold = 0.25;

  // expect a sustained 0 torque before starting
  while (torque_zero_counter < torque_zero_counter_max){
    float torque = get_torque();
    if (abs(torque) < torque_zero_threshold){
      torque_zero_counter++;
    }
    else{
      torque_zero_counter = 0;
    }
    Serial6.print("Waiting for torque to be zero. Input torque: ");
    Serial6.print(torque);
    Serial6.print(". Counter:");
    Serial6.println(torque_zero_counter);
  }
}

void loop()
{
  static bool state = false;
  target = get_torque();

  // if (state == false && analogRead(MOTOR_EN) > 900 && abs(target) > 0.1){
  if (state == false && analogRead(MOTOR_EN) > 900){
    motor.enable();
    state = true;
  }
  // else if ((state == true && analogRead(MOTOR_EN) < 900) || abs(target) < 0.1){
  else if ((state == true && analogRead(MOTOR_EN) < 900)){
    motor.disable();
    state = false;
  }
   
  
  if (state == true){
    motor.loopFOC();
    motor.move(target);

    Serial6.print("Vesc Active, target torque: ");
    Serial6.println(target);
  }
  else{
    Serial6.print("Input voltage: ");
    Serial6.print(analogRead(MOTOR_EN));
    Serial6.println("; Motor disabled");
  }
}
