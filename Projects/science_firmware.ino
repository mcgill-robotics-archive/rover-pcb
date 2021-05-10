/*
 * ----------[PINOUT]----------
 * (Also see documentation: 
 * https://github.com/mcgill-robotics/rover-pcb/blob/science-dev/Projects/science.pdf)
 * 
 * ----[Fault Detection/Shutdown Lines]----
 * D51 - POWER_ON: Set high to enable the relay, allowing the rest of the board to get power.
 * D52 - STEPPER_LINE_OK: High when the 5V regulator channel feeding the stepper motors is within an acceptable range.
 * D53 - OTHER_LINE_OK: High when the 5V regulator channel feeding everything else is within an acceptable range.
 * D7 - REG_SHUTDOWN: Set high to shut down 5V regulator
 * 
 * D38 - notFAULT1: High when Stepper #1 is not in a fault condition
 * D36 - notFAULT2: High when Stepper #2 is not in a fault condition
 * D44 - STEPPER1_notENABLE: Stepper #1 is enabled when this pin is low.
 * D50 - STEPPER2_notENABLE: Stepper #2 is enabled when this pin is low.
 * (IMPORTANT: There isn't enough current to run both steppers at once. Either D44 or D50 has to be high at any
 * given time!)
 * 
 * ----[Stepper Control Lines]----
 * D42 - STEPPER1_STEP: Stepper #1 will advance one step on the rising edge of this pin.
 * D40 - STEPPER1_DIRECTION: Stepper #1 will change direction on the rising edge of this pin.
 * D48 - STEPPER2_STEP: Stepper #2 will advance one step on the rising edge of this pin.
 * D46 - STEPPER2_DIRECTION: Stepper #2 will change direction on the rising edge of this pin.
 * 
 * ----[DC Motor Control Lines]----
 * Will use this library: https://github.com/Infineon/DC-Motor-Control-BTN8982TA
 * 
 * D3 - IN_1: (Input Bridge 1)
 * D11 - IN_2: (Input Bridge 2)
 * D12 - INH_1: (Inhibit Bridge 1)
 * D13 - INH_2: (Inhibit Bridge 2)
 * 
 * ----[Laser]----
 * D8 - LASER1_CONTROL: Laser is on when this pin is low
 * 
 * ----[LED's]----
 * D32 - LED_CONTROL: LED's are on when this pin is low
 * 
 * ----[Solenoid]----
 * D22 - SOLENOID_ON: Solenoid is on when this pin is high
 * 
 * ----[CCD Sensor]----
 * TODO: Figure out how the sensor works
 * A3 - OutputSignal_out: “out” as in “out to Arduino”.
 * D2 - phiM(CCD_Clock): Clock output to CCD sensor.
 * D10 - SH: “Shift Gate” (???)
 * D9 - ICG: “Integration Clear Gate” (???)
 * 
 * ----[COMMUNICATION LINES]----
 * 
 * ----[USB Debugging]----
 * D0, D1 (already assigned by default)
 * 
 * ----[Serial 1]----
 * D18 - UART TX
 * D19 - UART RX
 * 
 * TODO: Figure out the rest of UART(serial) communications, and maybe free up some pins from the CCD sensor.
 * TODO: Figure out what to write to UART(serial) ports
 * 
 * ----[SPI]----
 * TODO: Decide which pins we should sacrifice for SPI because any pin can be used in Due.
 * Source for SPI: https://www.arduino.cc/en/Reference/DueExtendedSPI
 * 
 * ----[I2C]----
 * Will use the built-in Wire library: https://www.arduino.cc/en/reference/wire
 * D20 - SDA
 * D21 - SCL
 * 
 * There are also two designated pins in the middle of the board for an extra I2C interface if need be.
 * 
 * ----[ADC]----
 * Analog pins can do ADC by default.
 */

#include <Wire.h>
#include <SPI.h>
#include <IfxMotorControlShield.h>

const int POWER_ON = 51;
const int STEPPER_LINE_OK = 52;
const int OTHER_LINE_OK = 53;
const int REG_SHUTDOWN = 7;
const int notFAULT1 = 38;
const int notFAULT2 = 36;
const int STEPPER1_notENABLE = 44;
const int STEPPER2_notENABLE = 50;
const int STEPPER1_STEP = 42;
const int STEPPER1_DIRECTION = 40;
const int STEPPER2_STEP = 48;
const int STEPPER2_DIRECTION = 46;
const int IN_1 = 3;
const int IN_2 = 11;
const int INH_1 = 12;
const int INH_2 = 13;
const int LASER1_CONTROL = 8;
#define OutputSignal_out A3
const int CCD_Clock = 2;
const int SH = 10;
const int ICG = 9;
const int SOLENOID_ON = 22;
const int LED_CONTROL = 32;

// Failure conditions, in case they need to be communicated to the central computer
volatile int STEPPER_LINE_FAULT = 0;
volatile int OTHER_LINE_FAULT = 0;
volatile int STEPPER1_FAULT = 0;
volatile int STEPPER2_FAULT = 0;

const int UART_BAUD_RATE = 9600;
const int I2C_BUS_ADDRESS = 0;

// Brushless DC motor constants
bool MOTOR_INIT_FAILED = false;
bool MOTOR_ON = false;
int speedvalue = 0;
int acceleration = 5;

void setup() {
  // TODO: Look into CCD pins and figure out whether they are outputs or inputs
  pinMode(POWER_ON, OUTPUT);
  pinMode(SOLENOID_ON, OUTPUT);
  pinMode(LED_CONTROL, OUTPUT);
  pinMode(STEPPER_LINE_OK, INPUT);
  pinMode(OTHER_LINE_OK, INPUT);
  pinMode(REG_SHUTDOWN, OUTPUT);
  pinMode(notFAULT1, INPUT);
  pinMode(notFAULT2, INPUT);
  pinMode(STEPPER1_notENABLE, OUTPUT);
  pinMode(STEPPER2_notENABLE, OUTPUT);
  pinMode(STEPPER1_STEP, OUTPUT);
  pinMode(STEPPER1_DIRECTION, OUTPUT);
  pinMode(STEPPER2_STEP, OUTPUT);
  pinMode(STEPPER2_DIRECTION, OUTPUT);
  pinMode(LASER1_CONTROL, OUTPUT);
  pinMode(OutputSignal_out, INPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(INH_1, OUTPUT);
  pinMode(INH_2, OUTPUT);
  pinMode(CCD_Clock, OUTPUT);
  pinMode(SH, OUTPUT);
  pinMode(ICG, OUTPUT);

  // TODO: Communications setup (UART, I2C, SPI)
  Serial.begin(UART_BAUD_RATE);
  Serial1.begin(UART_BAUD_RATE);
  Wire.begin(I2C_BUS_ADDRESS);

  // Initial conditions for active low pins
  digitalWrite(LASER1_CONTROL, HIGH);
  digitalWrite(STEPPER1_notENABLE, HIGH);
  digitalWrite(STEPPER2_notENABLE, HIGH);
  digitalWrite(REG_SHUTDOWN, HIGH);
  digitalWrite(LED_CONTROL, HIGH);

  // Update(18/03/2021): We now have an onboard relay
  digitalWrite(POWER_ON, HIGH);

  // Switch on the regulator
  digitalWrite(REG_SHUTDOWN, LOW);

  while(!(STEPPER_LINE_OK and OTHER_LINE_OK)){} // Wait until regulator starts up
  while(!(notFAULT1 and notFAULT2)){}           // Wait until both stepper IC's start up properly

  //Attach interrupts to their ISR's (Interrupt Service Routines):
  attachInterrupt(digitalPinToInterrupt(STEPPER_LINE_OK), stepperlinefault_ISR, LOW);
  attachInterrupt(digitalPinToInterrupt(OTHER_LINE_OK), otherlinefault_ISR, LOW);
  attachInterrupt(digitalPinToInterrupt(notFAULT1), stepperonefault_ISR, LOW);
  attachInterrupt(digitalPinToInterrupt(notFAULT2), steppertwofault_ISR, LOW);
  //(ISR's are at the bottom)

  // Initialize brushless DC motor controller.
  // Function returns true on failure, wtf?
  if(ifxMcsBiDirectionalMotor.begin()){
    MOTOR_INIT_FAILED = true;
  }

  // Turn on LED
  digitalWrite(LED_CONTROL, LOW);


  // TODO: Tell the power board and the main computer that the science board is fully booted
  // and can receive commands
}

void loop() {
  // put your main code here, to run repeatedly:

  // Closed loop motor control
  if(MOTOR_ON){
    
    if(ifxMcsBiDirectionalMotor.getCurrentSense() < IFX_MCS_CRITICALCURRENTSENSE)
   {

      speedvalue += acceleration;
      if(speedvalue > 255 && acceleration > 0)
      {
        acceleration = -acceleration;
        speedvalue = 255;
      }
      if(speedvalue < -255 && acceleration < 0)
      {
        acceleration = -acceleration;
        speedvalue = -255;
      }
      
      ifxMcsBiDirectionalMotor.setBiDirectionalSpeed(speedvalue);
      
    //if speed was set to 0, the motor has to be restarted
      if(!ifxMcsBiDirectionalMotor.getRunning())
        ifxMcsBiDirectionalMotor.start();
   }
   else
   {
    // Something went wrong, stop the motor
     stopMotor();
   }
  }

}

// ___[BRUSHLESS DC MOTOR]___________________________________________
void startMotorWithSpeed(int speedvalue){
  if(!MOTOR_INIT_FAILED){
    ifxMcsBiDirectionalMotor.setBiDirectionalSpeed(speedvalue);
    ifxMcsBiDirectionalMotor.start();
    MOTOR_ON = true;
  }
}

void stopMotor(){
  ifxMcsBiDirectionalMotor.stop();
  speedvalue = 0;
  MOTOR_ON = false;
}

// ___[LED]____________________________________________________________

void LEDon(){
  digitalWrite(LED_CONTROL, LOW);
}

void LEDoff(){
  digitalWrite(LED_CONTROL, HIGH);
}

// ___[SOLENOID]_______________________________________________________

void solenoidOn(){
  digitalWrite(SOLENOID_ON, HIGH);
}

void solenoidOff(){
  digitalWrite(SOLENOID_ON, LOW);
}

// ___[LASER]__________________________________________________________

void laserOn(){
  digitalWrite(LASER1_CONTROL, LOW);
}

void laserOff(){
  digitalWrite(LASER1_CONTROL, HIGH);
}

// ___[STEPPERS]_______________________________________________________
// Important note: There isn't enough current to run both steppers at once,
// so it's very important to make sure at most one of them is active at any given time.

unsigned int MINIMUM_STEP_PULSE = 3; // Microseconds
int MINIMUM_DISABLE_TIME = 1;        // Milliseconds
int MINIMUM_ENABLE_TIME = 1;         // Milliseconds

// 0 for forward, 1 for backward
bool STEPPER1_FORWARD = true;
bool STEPPER2_FORWARD = true;

double STEPPER1_CURRENT_ANGLE = 45;
double STEPPER2_CURRENT_ANGLE = 45;
double ANGLE_PER_STEP = 1.8;

void stepper1_step(){

  // Disable the other stepper and give it time to
  // shut down, just to make sure
  digitalWrite(STEPPER2_notENABLE, HIGH);
  delay(MINIMUM_DISABLE_TIME);

  // Enable this stepper and give it time to turn on
  digitalWrite(STEPPER1_notENABLE, LOW);
  delay(MINIMUM_ENABLE_TIME);
  
  // One last check if the other stepper is disabled, and then pulse
  // the driver to step forward

  // The checks might seem paranoid, but they could be the difference
  // between a working board and e-waste
  
  if(STEPPER1_notENABLE){
    digitalWrite(STEPPER1_STEP, HIGH);
    delayMicroseconds(MINIMUM_STEP_PULSE);
    digitalWrite(STEPPER1_STEP, LOW);
  }

  // Finally, disable this stepper
  digitalWrite(STEPPER1_notENABLE, HIGH);
  delay(MINIMUM_DISABLE_TIME);

  // Internally keep track of the current angle,
  // after everything else is done

  if(STEPPER1_FORWARD){
    
    STEPPER1_CURRENT_ANGLE += ANGLE_PER_STEP;
    if(STEPPER1_CURRENT_ANGLE >= 360){
      STEPPER1_CURRENT_ANGLE -= 360;
    }
    
  }else{
    
    STEPPER1_CURRENT_ANGLE -= ANGLE_PER_STEP;
    if(STEPPER1_CURRENT_ANGLE < 0){
      STEPPER1_CURRENT_ANGLE += 360;
    }
  }
  
}

void stepper2_step(){

  // Disable the other stepper and give it time to
  // shut down, just to make sure
  digitalWrite(STEPPER1_notENABLE, HIGH);
  delay(MINIMUM_DISABLE_TIME);

  // Enable this stepper and give it time to turn on
  digitalWrite(STEPPER2_notENABLE, LOW);
  delay(MINIMUM_ENABLE_TIME);
  
  // One last check if the other stepper is disabled, and then pulse
  // the driver to step forward

  // The checks might seem paranoid, but they could be the difference
  // between a working board and e-waste
  
  if(STEPPER1_notENABLE){
    digitalWrite(STEPPER2_STEP, HIGH);
    delayMicroseconds(MINIMUM_STEP_PULSE);
    digitalWrite(STEPPER2_STEP, LOW);
  }

  // Finally, disable this stepper
  digitalWrite(STEPPER2_notENABLE, HIGH);
  delay(MINIMUM_DISABLE_TIME);

  // Internally keep track of the current angle,
  // after everything else is done

  if(STEPPER1_FORWARD){
    
    STEPPER1_CURRENT_ANGLE += ANGLE_PER_STEP;
    if(STEPPER1_CURRENT_ANGLE >= 360){
      STEPPER1_CURRENT_ANGLE -= 360;
    }
    
  }else{
    
    STEPPER1_CURRENT_ANGLE -= ANGLE_PER_STEP;
    if(STEPPER1_CURRENT_ANGLE < 0){
      STEPPER1_CURRENT_ANGLE += 360;
    }
  }
  
}

void stepper1_changedirection(){
  
  digitalWrite(STEPPER1_DIRECTION, HIGH);
  delayMicroseconds(3);
  digitalWrite(STEPPER1_DIRECTION, LOW);
  
  STEPPER1_FORWARD = !STEPPER1_FORWARD;
}

void stepper2_changedirection(){
  
  digitalWrite(STEPPER2_DIRECTION, HIGH);
  delayMicroseconds(3);
  digitalWrite(STEPPER2_DIRECTION, LOW);
  
  STEPPER2_FORWARD = !STEPPER2_FORWARD;
}

// ___[INTERRUPTS]______________________________________________________________________

void stepperlinefault_ISR(){
  // Turn off relay
  digitalWrite(51, LOW);
  
  // Shut down steppers before anything else. If there's a fault in the stepper line,
  // odds are that the steppers will get hurt first
  digitalWrite(44, HIGH);
  digitalWrite(50, HIGH);

  // Shut down regulator if it hasn't shut down already
  digitalWrite(7, HIGH);

  // Shut down the rest of the board, starting with the brushed motor
  stopMotor();
  LEDoff();
  laserOff();
  solenoidOff();

  STEPPER_LINE_FAULT = 1;

  // Maybe some code here to tell the power board and the main computer that something
  // went wrong with the stepper line?

  // Hang until power is cycled
  while(1){
    // Or maybe until the main computer tells the board to keep going?
    }
}

void otherlinefault_ISR(){
  // Turn off relay
  digitalWrite(51, LOW);

  // Shut down the board, starting with the brushed motor
  stopMotor();
  LEDoff();
  laserOff();
  solenoidOff();
  digitalWrite(44, HIGH);
  digitalWrite(45, HIGH);

  // Shut down regulator if it hasn't shut down already
  digitalWrite(7, HIGH);

  OTHER_LINE_FAULT = 1;

  // Maybe some code here to tell the power board and the main computer that something
  // went wrong with the other 5V line?

  // Hang until power is cycled
  while(1){
    // Or maybe until the main computer tells the board to keep going?
    }
}

void stepperonefault_ISR(){
  // Turn off relay
  digitalWrite(51, LOW);
  
  // Shut down steppers (starting with #1) before anything else. If there's a fault in that stepper,
  // odds are that it will get hurt first
  digitalWrite(44, HIGH);
  digitalWrite(50, HIGH);

  // Shut down regulator if it hasn't shut down already
  digitalWrite(7, HIGH);

  // Shut down the rest of the board, starting with the brushed motor
  stopMotor();
  LEDoff();
  laserOff();
  solenoidOff();

  STEPPER1_FAULT = 1;

  // Maybe some code here to tell the power board and the main computer that something
  // went wrong with stepper #1?

  // Hang until power is cycled
  while(1){
    // Or maybe until the main computer tells the board to keep going?
    }
}

void steppertwofault_ISR(){
  // Turn off relay
  digitalWrite(51, LOW);
  
  // Shut down steppers (starting with #1) before anything else. If there's a fault in that stepper,
  // odds are that it will get hurt first
  digitalWrite(50, HIGH);
  digitalWrite(44, HIGH);

  // Shut down regulator if it hasn't shut down already
  digitalWrite(7, HIGH);

  // Shut down the rest of the board, starting with the brushed motor
  stopMotor();
  LEDoff();
  laserOff();
  solenoidOff();

  STEPPER2_FAULT = 1;

  // Maybe some code here to tell the power board and the main computer that something
  // went wrong with stepper #2?

  // Hang until power is cycled
  while(1){
    // Or maybe until the main computer tells the board to keep going?
    }
}
