/*  
Based on code from 
Joshua Vasquez
Rotary Encoder With Interrupts
May 26 to 28, 2012
https://github.com/Poofjunior/ArduinoQuadratureRotaryEncoder
and also concepts from
https://www.mathworks.com/help/supportpkg/freescalefrdmk64fboard/ref/quadratureencoder.html
https://www.mathworks.com/help/supportpkg/arduinoio/quadrature-encoders.html
https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
https://www.arduino.cc/reference/en/language/functions/interrupts/nointerrupts/
*/

const byte chan_A = 2;
const byte chan_A2 = 3;
const byte chan_I = 21;
volatile boolean falling = false; // true if one revolution. will reset after reaching 1 revolution to prevent integer overflow of variable.
volatile boolean rising = false; // true if one cycle occurs. will reset each cycle.
volatile unsigned long cycle_cnt = 0;
volatile unsigned long rising_time = 0;
volatile unsigned long falling_time = 0;
unsigned long cycle_time = 0;
unsigned long curr_time = 0;
float curr_angle = 0;


void setup() {
  pinMode(chan_A, INPUT);
  pinMode(chan_A2, INPUT);
  pinMode(chan_I, INPUT);
  attachInterrupt(0, CounterA, RISING); // pin 2 on Arduino Mega
  attachInterrupt(1, CounterA2, FALLING); // pin 3 on Arduino Mega
  Serial.begin(9600);
  Serial.print("Setting up encoder program to read speed");
}

// Counter A interrupt service routine for rising edge
void CounterA() { 
  if(!rising){
    rising_time = micros();
    rising = true;
  }
}
// Counter A interrupt service routine for falling edge
void CounterA2(){
   if(rising && !falling){ 
     falling_time = micros();
     falling = true;
   }
   
}

// get speed, angle
void loop() {
  // Displays Current angle of motor
  if (falling){
     noInterrupts(); // disable interrupts for the time so that can set rising and falling variables to false
     rising = false;
     falling = false;
     interrupts(); // enables interrupts again.
     cycle_time = 2*(falling_time - rising_time);
     curr_time += cycle_time; 
     curr_angle += (360.0/500);
     cycle_cnt += 1;
     if (cycle_cnt == 500){
        Serial.println("Speed (RPS)");
        Serial.println(1000000.0 /(curr_time));
        curr_time = 0;
        curr_angle = 0;
        cycle_cnt = 0;
     }
  }

}
