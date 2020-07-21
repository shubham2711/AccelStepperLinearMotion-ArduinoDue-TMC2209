

/*We are getting 4 pinches per second*/

#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>
#define STEPPER_SERIAL Serial1
// v0.1.1 pin def

static const int Y_dir = 27;

static const int Y_step = 29;
static const int Y_driver_uart = 25;
static const int Y_en = 39; 
static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;

TMC2209Stepper Y_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);

AccelStepper stepper_Y = AccelStepper(AccelStepper::DRIVER, Y_step, Y_dir);
    static const long steps_per_mm_XY = 120;                                         // for PL35L-024-VLB8
  //constexpr float MAX_VELOCITY_Y_mm = 30;                                          // for PL35L-024-VLB8
    constexpr float MAX_VELOCITY_Y_mm = 200;                                         // for PL35L-024-VLB8
    constexpr float MAX_ACCELERATION_Y_mm = 700; 
    static const long Y_NEG_LIMIT_MM = -12;
    static const long Y_POS_LIMIT_MM = 12;

void setup() {
  
  pinMode(Y_driver_uart, OUTPUT);
  pinMode(Y_dir, OUTPUT);
  pinMode(Y_step, OUTPUT);

  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);
  
  digitalWrite(Y_driver_uart, true);
  while(!STEPPER_SERIAL);
  Y_driver.begin();
  Y_driver.I_scale_analog(false);  
  Y_driver.rms_current(450); //I_run and holdMultiplier
  Y_driver.microsteps(4);
  Y_driver.pwm_autoscale(true);
  Y_driver.TPOWERDOWN(2);
  Y_driver.en_spreadCycle(true);
  Y_driver.toff(4);
  digitalWrite(Y_driver_uart, false);

  stepper_Y.setEnablePin(Y_en);
  stepper_Y.setPinsInverted(false, false, true);
  stepper_Y.setMaxSpeed(MAX_VELOCITY_Y_mm*steps_per_mm_XY);
  stepper_Y.setAcceleration(MAX_ACCELERATION_Y_mm*steps_per_mm_XY);
  stepper_Y.enableOutputs();

}

void loop() {
  
  stepper_Y.runToNewPosition(3*steps_per_mm_XY);
  delay(3000);
  stepper_Y.runToNewPosition(-3*steps_per_mm_XY);
  
}
