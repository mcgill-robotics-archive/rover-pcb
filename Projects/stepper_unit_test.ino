/*
 * ---[STEPPER UNIT TEST PINOUT]---
 * 
 * D2 - notFAULT1: High when Stepper #1 is not in a fault condition
 * D3 - notFAULT2: High when Stepper #2 is not in a fault condition
 * (Don't change these two pins because Arduino Uno only has two interrupt
 * pins. Maybe some buttons could simulate?)
 * 
 * D4 - STEPPER1_STEP: Stepper #1 will advance one step on the rising edge of this pin.
 * D5 - STEPPER1_DIRECTION: Stepper #1 will change direction on the rising edge of this pin.
 * D6 - STEPPER2_STEP: Stepper #2 will advance one step on the rising edge of this pin.
 * D7 - STEPPER2_DIRECTION: Stepper #2 will change direction on the rising edge of this pin.
 * D8 - STEPPER1_notENABLE: Stepper #1 is enabled when this pin is low.
 * D9 - STEPPER2_notENABLE: Stepper #2 is enabled when this pin is low.
 * 
 * Either these or whichever pinout is more convenient
 * 
  */

const int notFAULT1 = 2;
const int notFAULT2 = 3;
const int STEPPER1_STEP = 4;
const int STEPPER1_DIRECTION = 5;
const int STEPPER2_STEP = 6;
const int STEPPER2_DIRECTION = 7;
const int STEPPER1_notENABLE = 8;
const int STEPPER2_notENABLE = 9;

int STEPPER1_FAULT = 0;
int STEPPER2_FAULT = 0;

unsigned long t = 0;
unsigned long temp = 0;

bool STEPPER1_FORWARD = true;
bool STEPPER2_FORWARD = true;

unsigned int MINIMUM_STEP_PULSE = 3; // Microseconds
int MINIMUM_DISABLE_TIME = 1;        // Milliseconds
int MINIMUM_ENABLE_TIME = 1;         // Milliseconds

double STEPPER1_CURRENT_ANGLE = 45;
double STEPPER2_CURRENT_ANGLE = 45;

int TIME_EXPANSION_FACTOR = 1000;

double ANGLE_PER_STEP = 1.8;

void setup() {
  // put your setup code here, to run once:
  pinMode(notFAULT1, INPUT);
  pinMode(notFAULT2, INPUT);
  pinMode(STEPPER1_STEP, OUTPUT);
  pinMode(STEPPER2_STEP, OUTPUT);
  pinMode(STEPPER1_DIRECTION, OUTPUT);
  pinMode(STEPPER2_DIRECTION, OUTPUT);
  pinMode(STEPPER1_notENABLE, OUTPUT);
  pinMode(STEPPER2_notENABLE, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(notFAULT1), stepperonefault_ISR, LOW);
  attachInterrupt(digitalPinToInterrupt(notFAULT2), steppertwofault_ISR, LOW);

  Serial.begin(9600);
  Serial.println("Serial communication OK");

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    Serial.println("Press A to pulse stepper at D4");
    Serial.println("Press S to pulse stepper at D5");
    Serial.println("Press Z to reverse direction of the stepper at D4");
    Serial.println("Press X to reverse direction of the stepper at D4");
    Serial.println("Press G for status report");

    while(1){
      
      if(Serial.available()){
        if(Serial.read() == 'A'){
          stepper1_step();
          break;
        }
        if(Serial.read() == 'S'){
          stepper2_step();
          break;
        }
        if(Serial.read() == 'Z'){
          stepper1_changedirection();
          break;
        }
        if(Serial.read() == 'X'){
          stepper2_changedirection();
          break;
        }
        if(Serial.read() == 'G'){
          StatusReport();
          break;
        }
      }
    }
  }
}

// ___[STEPPERS]_______________________________________________________________________________
// Important note: There isn't enough current to run both steppers at once,
// so it's very important to make sure at most one of them is active at any given time.
// (For the unit test, all times are expanded by a factor of 1000 so the changes are visible on
// the LED's.)

void stepper1_step(){

  // Disable the other stepper and give it time to
  // shut down, just to make sure
  digitalWrite(STEPPER2_notENABLE, HIGH);
  delay(MINIMUM_DISABLE_TIME * TIME_EXPANSION_FACTOR);

  // Enable this stepper and give it time to turn on
  digitalWrite(STEPPER1_notENABLE, LOW);
  delay(MINIMUM_ENABLE_TIME * TIME_EXPANSION_FACTOR);
  
  // One last check if the other stepper is disabled, and then pulse
  // the driver to step forward

  // The checks might seem paranoid, but they could be the difference
  // between a working board and e-waste
  
  if(STEPPER1_notENABLE){
    digitalWrite(STEPPER1_STEP, HIGH);
    //delayMicroseconds(MINIMUM_STEP_PULSE);
    //( Expand by 1000 by converting to delay() )
    delay(MINIMUM_STEP_PULSE * (TIME_EXPANSION_FACTOR/1000));
    
    digitalWrite(STEPPER1_STEP, LOW);
  }

  // Finally, disable this stepper
  digitalWrite(STEPPER1_notENABLE, HIGH);
  delay(MINIMUM_DISABLE_TIME * TIME_EXPANSION_FACTOR);

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
  delay(MINIMUM_DISABLE_TIME * 1000);

  // Enable this stepper and give it time to turn on
  digitalWrite(STEPPER2_notENABLE, LOW);
  delay(MINIMUM_ENABLE_TIME * 1000);
  
  // One last check if the other stepper is disabled, and then pulse
  // the driver to step forward

  // The checks might seem paranoid, but they could be the difference
  // between a working board and e-waste
  
  if(STEPPER1_notENABLE){
    digitalWrite(STEPPER2_STEP, HIGH);
    //delayMicroseconds(MINIMUM_STEP_PULSE);
    //( Expand by 1000 by converting to delay() )
    delay(MINIMUM_STEP_PULSE * (TIME_EXPANSION_FACTOR/1000));
    
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
  delay(500);
  //delayMicroseconds(3);
  digitalWrite(STEPPER1_DIRECTION, LOW);
  
  STEPPER1_FORWARD = !STEPPER1_FORWARD;
}

void stepper2_changedirection(){
  
  digitalWrite(STEPPER2_DIRECTION, HIGH);
  //delayMicroseconds(3);
  delay(3*(TIME_EXPANSION_FACTOR / 1000));
  digitalWrite(STEPPER2_DIRECTION, LOW);
  
  STEPPER2_FORWARD = !STEPPER2_FORWARD;
}

// --[PRINT UTILITY FUNCTIONS]--

void printDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
       mult *=10;
       
    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
      padding--;
    while(  padding--)
      Serial.print("0");
    Serial.print(frac,DEC) ;
  }
}

void StatusReport(){
  Serial.println("");
  
  Serial.println("---[Current status of steppers]---");
  Serial.println("");
  Serial.print("Angle of stepper #1: ");
  printDouble(STEPPER1_CURRENT_ANGLE, 3);
  
  if(STEPPER1_FORWARD){
    Serial.println("Direction of stepper #1: Forward");
  }else{
    Serial.println("Direction of stepper #1: Backward");
  }
  
  Serial.println("");
  Serial.print("Angle of stepper #2: ");
  printDouble(STEPPER2_CURRENT_ANGLE, 3);
  if(STEPPER2_FORWARD){
    Serial.println("Direction of stepper #2: Forward");
  }else{
    Serial.println("Direction of stepper #2: Backward");
  }

  Serial.println("");
}


// ---[INTERRUPTS]---

void steppertwofault_ISR(){
  // Shut down steppers (starting with #1) before anything else. If there's a fault in that stepper,
  // odds are that it will get hurt first
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);

//  // Shut down regulator if it hasn't shut down already
//  digitalWrite(7, HIGH);
//
//  // Shut down the rest of the board, starting with the brushed motor
//  digitalWrite(12, LOW);
//  digitalWrite(13, LOW);
//  digitalWrite(8, HIGH);

  STEPPER2_FAULT = 1;

  // Maybe some code here to tell the power board and the main computer that something
  // went wrong with stepper #2?
  Serial.println("Fault detected in Stepper #2, interrupt triggered!");
  delay(100);

  // Hang until power is cycled
  while(1){
    Serial.println("Board hung until power is cycled");
    // Or maybe until the main computer tells the board to keep going?
    }
}

void stepperonefault_ISR(){
  // Shut down steppers (starting with #1) before anything else. If there's a fault in that stepper,
  // odds are that it will get hurt first
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);

//  // Shut down regulator if it hasn't shut down already
//  digitalWrite(7, HIGH);
//
//  // Shut down the rest of the board, starting with the brushed motor
//  digitalWrite(12, LOW);
//  digitalWrite(13, LOW);
//  digitalWrite(8, HIGH);

  STEPPER1_FAULT = 1;

  // Maybe some code here to tell the power board and the main computer that something
  // went wrong with stepper #1?
  Serial.println("Fault detected in Stepper #1, interrupt triggered!");
  delay(100);

  // Hang until power is cycled
  while(1){
    Serial.println("Board hung until power is cycled");
    // Or maybe until the main computer tells the board to keep going?
    }
}
