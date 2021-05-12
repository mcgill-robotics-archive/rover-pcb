/*
 * ----------[PINOUT]----------
 * (Also see documentation: 
 * https://github.com/mcgill-robotics/rover-pcb/blob/science-dev/Projects/science.pdf)
 * 
 * ----[Fault Detection/Shutdown Lines]----
 * D51 - POWER_ON: Set high to enable the relay, allowing the rest of the board to get power.
 * 
 * D38 - notFAULT1: High when Stepper #1 is not in a fault condition
 * D36 - notFAULT2: High when Stepper #2 is not in a fault condition
 * D44 - STEPPER1_notENABLE: Stepper #1 is enabled when this pin is low.
 * D50 - STEPPER2_notENABLE: Stepper #2 is enabled when this pin is low.
 * 
 * ----[Stepper Control Lines]----
 * D42 - STEPPER1_STEP: Stepper #1 will advance one step on the rising edge of this pin.
 * D40 - STEPPER1_DIRECTION: Stepper #1 will change direction on the rising edge of this pin.
 * D48 - STEPPER2_STEP: Stepper #2 will advance one step on the rising edge of this pin.
 * D46 - STEPPER2_DIRECTION: Stepper #2 will change direction on the rising edge of this pin.
 * 
 * ----[Limit Switches]----
 * D34 - LIMIT_SWITCH_1
 * D52 - LIMIT_SWITCH_2
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
 * D6 - phiM(CCD_Clock): Clock output to CCD sensor.
 * D20 - SH: “Shift Gate” (???)
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
 * 
 * Toggle SH:
 * REG_PIOB_SODR |= (0x01 << 12); // turn on
 * REG_PIOB_CODR |= (0x01 << 12); // turn off
 * 
 * Toggle ICG:
 * REG_PIOB_SODR |= (0x01 << 13); // turn on
 * REG_PIOB_CODR |= (0x01 << 13); // turn off
 */

#include <pwm_lib.h>
#include <pwm_defs.h>
#include <Wire.h>
#include <SPI.h>
#include <IfxMotorControlShield.h>
using namespace arduino_due::pwm_lib;

const int POWER_ON = 51;
const int LIMIT_SWITCH_1 = 34;
const int LIMIT_SWITCH_2 = 52;
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
const int CCD_Clock = 16;
const int SH = 20;
const int ICG = 21;
const int SOLENOID_ON = 22;
const int LED_CONTROL = 32;

// CCD sensor defines
#define OutputSignal_out A3
#define CCD_CLOCK_PERIOD 50 // hundredth of usecs (1e-8 secs)
#define CCD_CLOCK_DUTY_CYCLE 5 // 100 msecs in hundredth of usecs (1e-8 secs)

#define SH_PERIOD 100 // some integer multiple of the clock period
#define SH_DUTY_CYCLE 50

#define ICG_PERIOD 200
#define ICG_DUTY_CYCLE 50

// The CCD pin selections are dictated by the ATSAM3X8E hardware PWM capabilities and
// the least awkward possible way to route the wires.
// See https://github.com/antodom/pwm_lib/blob/master/pwm_defs.h
// This library is used to access high frequency hardware PWM which isn't normally enabled on Due.

pwm<pwm_pin::PWMH2_PA13> pwm_CCDclock; // pin D16

// Failure conditions, in case they need to be communicated to the central computer
volatile int STEPPER1_FAULT = 0;
volatile int STEPPER2_FAULT = 0;

const int UART_BAUD_RATE = 9600;

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
  pinMode(LIMIT_SWITCH_1, INPUT);
  pinMode(LIMIT_SWITCH_2, INPUT);

  // TODO: Communications setup (UART, I2C, SPI)
  Serial.begin(UART_BAUD_RATE);
  Serial1.begin(UART_BAUD_RATE);

  // Initial conditions for active low pins
  digitalWrite(LASER1_CONTROL, HIGH);
  digitalWrite(STEPPER1_notENABLE, HIGH);
  digitalWrite(STEPPER2_notENABLE, HIGH);
  digitalWrite(LED_CONTROL, HIGH);

  // Update(18/03/2021): We now have an onboard relay
  digitalWrite(POWER_ON, HIGH);

  while(!(notFAULT1 and notFAULT2)){}           // Wait until both stepper IC's start up properly

  //Attach interrupts to their ISR's (Interrupt Service Routines):
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

  // Start CCD sensor's clock
  pwm_CCDclock.start(CCD_CLOCK_PERIOD, CCD_CLOCK_DUTY_CYCLE);

  //Trying to set up output signal through digital pin using clock signal
  //Enabling the peripheral, output function, and writing function for the timer pin we use
  PMC -> PMC_PCER0 |= PMC_PCER0_PID##; (turning on the peripheral corresponding to the particular pin)

  PIO -> PIO_OER |= PIO_OER_P##; (enabling the specific output pin)

  PIO -> PIO_OWER |= PIO_OWER_P##; (enabling writing functionality to the output pin)

  //Setting up the timer counter channel and pin

  PMC -> PMC_PCER0 |= PMC_PCER0_PID# // turning on power for timer counter channel (see page 859 for PIDs)

  PIO -> PIO_PDR |= PIO_PDR_P## //turning of driving of pin by GPIO

  PIO -> PIO_ABSR |= PIO_PinIdentification_TIOA# //multiplexing peripheral functions for pin config (value of ID depends on pin used)
  //See page 859 for TC and TIO I/O line connections 

  TC# -> TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVESEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET; 
  //Clock select internal MCK/2 clock signal

  TC# -> TC_CHANNEL[0].TC_RC = 21;  // Freqency = (Mck/2)/TC_RC Hz = 2 MHz

  TC# -> TC_CHANNEL[0].TC_RA = 10.5 //Duty cycle = (TC_RA/TC_RC) * 100%

  TC# -> TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // SOftware trigger TC counter and en


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
  
  // Enable this stepper if it's not enabled and give it time to turn on
  if(digitalRead(STEPPER1_notENABLE) == HIGH){
    digitalWrite(STEPPER1_notENABLE, LOW);
    delay(MINIMUM_ENABLE_TIME);
  }

  // Pulse this stepper
  
  digitalWrite(STEPPER1_STEP, HIGH);
  delayMicroseconds(MINIMUM_STEP_PULSE);
  digitalWrite(STEPPER1_STEP, LOW);
  

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
  
  // Enable this stepper if it's not enabled and give it time to turn on
  if(digitalRead(STEPPER2_notENABLE) == HIGH){
    digitalWrite(STEPPER2_notENABLE, LOW);
    delay(MINIMUM_ENABLE_TIME);
  }

  // Pulse this stepper
  
  digitalWrite(STEPPER2_STEP, HIGH);
  delayMicroseconds(MINIMUM_STEP_PULSE);
  digitalWrite(STEPPER2_STEP, LOW);
  

  // Internally keep track of the current angle,
  // after everything else is done

  if(STEPPER2_FORWARD){
    
    STEPPER2_CURRENT_ANGLE += ANGLE_PER_STEP;
    if(STEPPER2_CURRENT_ANGLE >= 360){
      STEPPER2_CURRENT_ANGLE -= 360;
    }
    
  }else{
    
    STEPPER2_CURRENT_ANGLE -= ANGLE_PER_STEP;
    if(STEPPER2_CURRENT_ANGLE < 0){
      STEPPER2_CURRENT_ANGLE += 360;
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

//____[DELAY FUNCTIONS FOR CCD SIGNALS]_________________________________________________

// See https://forum.arduino.cc/t/delay-minimum-100ns/462449/2
// I changed the delayNanoseconds() function implemented by that answer to precalculate and
// hardcode 500 and 1000 ns, to avoid the extra ~100 ns delay that could be introduced by
// calculating n. The delayNanoseconds(ns) function is still here in case we need to use it.

static inline void delay500ns() __attribute__((always_inline, unused));
static inline void delay500ns(){
    /*
     * Based on Paul Stoffregen's implementation
     * for Teensy 3.0 (http://www.pjrc.com/)
     */
     
  uint32_t n = 14;
  asm volatile(
    "L_%=_delayNanos:"        "\n\t"
    "subs   %0, #1"                    "\n\t"
    "bne    L_%=_delayNanos" "\n"
    : "+r" (n) :
  );
}

static inline void delay1000ns() __attribute__((always_inline, unused));
static inline void delay1000ns(){
    /*
     * Based on Paul Stoffregen's implementation
     * for Teensy 3.0 (http://www.pjrc.com/)
     */
     
  uint32_t n = 28;
  asm volatile(
    "L_%=_delayNanos:"        "\n\t"
    "subs   %0, #1"                    "\n\t"
    "bne    L_%=_delayNanos" "\n"
    : "+r" (n) :
  );
}

static inline void delayNanoseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayNanoseconds(uint32_t nsec){
    /*
     * Based on Paul Stoffregen's implementation
     * for Teensy 3.0 (http://www.pjrc.com/)
     */
    if (nsec == 0) return;
    uint32_t n = (nsec * 1000) / 35714;
    asm volatile(
        "L_%=_delayNanos:"       "\n\t"
        "subs   %0, #1"                 "\n\t"
        "bne    L_%=_delayNanos" "\n"
        : "+r" (n) :
    );
}

//____[CCD SENSOR]______________________________________________________________________

// Half done function just to write down thoughts and get something started.
// We should try to find a way to probe the CCD clock signal as defined above.

const int INTEGRATION_TIME = 2000; //in ms

void CCDread(){
  // On the rising edge of the CCD clock;
  REG_PIOB_CODR |= (0x01 << 13); // ICG signal goes low
  delay500ns();
  REG_PIOB_SODR |= (0x01 << 12); // SH signal goes high
  delay1000ns();
  REG_PIOB_CODR |= (0x01 << 12); // SH signal goes low
  delay1000ns();

  // On the rising edge of the CCD clock;
  REG_PIOB_SODR |= (0x01 << 13); // ICG signal goes high.
  CCDSensorReadDataStream(); // Start listening to A3, keep listening for (INTEGRATION_TIME - 1500 ns)

  // On the rising edge of the CCD clock;
  REG_PIOB_CODR |= (0x01 << 13); // ICG signal goes low
  delay500ns();
  REG_PIOB_SODR |= (0x01 << 12); // SH signal goes high
  delay1000ns();
  REG_PIOB_CODR |= (0x01 << 12); // SH signal goes low
  delay1000ns();
  
  
}

void CCDSensorReadDataStream(){
  // This function should get the data from the A3 pin and put it somewhere, and should execute for tINT - 1500 ns (as specified
  // in the TCD1304DG datasheet)
}
  
// ___[INTERRUPTS]______________________________________________________________________

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
