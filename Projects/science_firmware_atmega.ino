/* OUTDATED - go to https://github.com/mcgill-robotics/rover-embedded/tree/EP56_science_system_firmware
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
 * A0 - AD_1
 * A1 - AD_2
 * 
 * ----[Laser]----
 * D8 - LASER1_CONTROL: Laser is on when this pin is high
 * 
 * ----[LED's]----
 * D4 - LED_CONTROL: LED's are on when this pin is high
 * 
 * ----[Solenoid]----
 * D22 - SOLENOID_ON: Solenoid is on when this pin is high
 * 
 * ----[CCD Sensor]----
 * TODO: Figure out how the sensor works
 * A3 - OutputSignal_out: “out” as in “out to Arduino”.
 * D2 - phiM(CCD_Clock): Clock output to CCD sensor.
 * D20 - SH: “Shift Gate” (???)
 * D21 - ICG: “Integration Clear Gate” (???)
 * 
 * ----[Peltier cooler]----
 * D14 - PELTIER_ON
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
 * ----[ADC]----
 * Analog pins can do ADC by default.
 * 
 */

#include <Wire.h>
#include <SPI.h>
#include <IfxMotorControlShield.h>

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
const int CCD_Clock = 2;
const int SH = 20;
const int ICG = 21;
const int SOLENOID_ON = 22;
const int LED_CONTROL = 4;
const int PELTIER_ON = 14;

#define AD_1 A0
#define AD_2 A1

// CCD sensor defines
#define OutputSignal_out A3
#define CCD_CLOCK_PERIOD 50 // hundredth of usecs (1e-8 secs)
#define CCD_CLOCK_DUTY_CYCLE 5 // 100 msecs in hundredth of usecs (1e-8 secs)

#define SH_PERIOD 100 // some integer multiple of the clock period
#define SH_DUTY_CYCLE 50

#define ICG_PERIOD 200
#define ICG_DUTY_CYCLE 50

#define MCLK 0x10
#define ICG 0x01
#define SH  0x02
#define PIXELS 3691
#define MIN_SIGNAL 10


//16-bit buffer for pixels
uint16_t pixelBuffer[PIXELS];
uint16_t avg = 0;

char cmdBuffer[16];
int exposureTime = 20; 
int cmdIndex; 
int cmdR = 0;

// Failure conditions, in case they need to be communicated to the central computer
volatile int STEPPER1_FAULT = 0;
volatile int STEPPER2_FAULT = 0;

const int UART_BAUD_RATE = 115200;

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
  pinMode(AD_1, INPUT);
  pinMode(AD_2, INPUT);
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
  digitalWrite(STEPPER1_notENABLE, HIGH);
  digitalWrite(STEPPER2_notENABLE, HIGH);

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
  digitalWrite(LED_CONTROL, HIGH);

  //Initialize the clocks
  DDRD |= (SH | ICG); //Set ICG and SH lines to outputs for port D
  DDRE |= (MCLK); //Set Master Clock line to output for port E
  PORTD |= ICG; set ICG line high

  //TODO: Set up timer to generate frquency on MCLK pin D2
  //No clock prescaling, clear timer on compare mode
  TCCR2A = (0 << COM2A1) | (1 << COM2A0) | (1 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22) |(1 << CS20); //no prescaling on the clock
  OCR2A = 3; //used to calculate frequency of output through MCLK pin (don't know what fclk is)
  TCNT2 = 0; //Reset timer2
  
  //Set ADC clock rate to sysclk/32
  ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);
  
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

//___Reading Data from CCD_____________________________________________





// ___[LED]____________________________________________________________

void LEDon(){
  digitalWrite(LED_CONTROL, HIGH);
}

void LEDoff(){
  digitalWrite(LED_CONTROL, LOW);
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
  digitalWrite(LASER1_CONTROL, HIGH);
}

void laserOff(){
  digitalWrite(LASER1_CONTROL, LOW);
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
double ANGLE_PER_STEP = 0.1125;

void stepper1_goto_angle(double angle){
  int n = round(angle/ANGLE_PER_STEP);
  
  if(n == 0) return;
  if(n < 0 && STEPPER1_FORWARD){
    stepper1_changedirection();
  }
  if(n > 0 && !STEPPER1_FORWARD){
    stepper1_changedirection();
  }
  
  for(int i = 0; i < n; i++){
    stepper1_step();
  }
}

void stepper2_goto_angle(double angle){
  int n = round(angle/ANGLE_PER_STEP);
  
  if(n == 0) return;
  if(n < 0 && STEPPER2_FORWARD){
    stepper2_changedirection();
  }
  if(n > 0 && !STEPPER2_FORWARD){
    stepper2_changedirection();
  }
  
  for(int i = 0; i < n; i++){
    stepper2_step();
  }
}

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

//____[CCD SENSOR]______________________________________________________________________

// Half done function just to write down thoughts and get something started.
// We should try to find a way to probe the CCD clock signal as defined above.

void CCDread(){
//   int x; 
//   uint16_t result;
  
  
//   PORTD &= ~ICG;      // set ICG line low
//   PORTD |= SH         // turn SH line high
//   TCNT2 = 0;          // On the rising edge of the CCD clock;
  
//   _delay_loop_1(8);   // delay 500ns on ATMega 2560
//   PORTD |= SH         // turn SH line high after delay
//   _delay_loop_1(16);  // delay 1000 ns on ATMega 2560
//   PORTD &= ~SH        // turn SH line low
//   _delay_loop_1(16);  // delay 1000 ns on ATMega 2560

//   TCNT2 = 0;                  // On the rising edge of the CCD clock;
//   PORTD |= ICG                // ICG signal goes high.
//   CCDSensorReadDataStream();  // Start listening to A3, keep listening for (INTEGRATION_TIME - 1500 ns)

  // On the rising edge of the CCD clock;
  PORTD &= ~ICG;      //set ICG line low
  _delay_loop_1(8);   // delay 500ns on ATMega 2560
  PORTD |= SH         //turn SH line high after delay
  _delay_loop_1(16);  // delay 1000 ns on ATMega 2560
  PORTD &= ~SH        //turn SH line low
  _delay_loop_1(16);  // delay 1000 ns on ATMega 2560
  PORTD |= ICG;       //set ICG line high
  
  for (x = 0; x < PIXELS; x++) {
        PORTD |= SH;
        if (x == 0) {
            result = (uint16_t)(1023 - analogRead(A3));
            if (result > MIN_SIGNAL) {
                avg = result - MIN_SIGNAL;
            } else {
                avg = result;
            }
        } else {
            result = (uint16_t)(1023 - analogRead(A3));
        }
        if (result < avg) {
            result = 0;
        } else {
            result -= avg;
        }
        pixelBuffer[x] = result;
        delayMicroseconds(21);
    }
    PORTD &= ~SH; 
}

//TODO: Write Method to Send Data via USB
void sendData(){
  
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
