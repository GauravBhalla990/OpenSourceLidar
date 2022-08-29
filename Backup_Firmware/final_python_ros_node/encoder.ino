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
*/

// This code measures the speed, and angle of the encoder by counting the time it takes for 500 Cycles.
// This code can measures the speed in two different ways: by the index signal and the channel A/B signal.
// So can also get an error.
// although cannot measure direction (clockwise or counterclockwise) by checking phase difference of Channel A and B.
// as with Poofjunior's code above which uses a finite state machine model.
const float pi = 3.14159;
const byte chan_A = 2; // channel A
const byte chan_I = 3; // chanel Index. Or can change to chan_B if you want to find the directions.
volatile bool rev = false; // true if one revolution. will reset after reaching 1 revolution to prevent integer overflow of variable.
volatile bool cycle = false; // true if one cycle occurs. will reset each cycle.
volatile unsigned long cycle_cnt = 0; // counting cycles per revolution to get angle
unsigned long cycle_time = 0;
unsigned long rev_time_I = 0; //keeps track of current time and resets after 1 revolution from channel I


float curr_angle = 0;
float speed_A = 0;
float speed_I = 0;
float speed_error = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("Setting up encoder program to read angle and speed");
  delay(100);
  pinMode(chan_A, INPUT);
  pinMode(chan_I, INPUT);
  attachInterrupt(0, CounterA, RISING); // pin 2 on Arduino Mega
  attachInterrupt(1, CounterI, RISING); // pin 3 on Arduino Mega

}

// Counter A interrupt service routine
void CounterA() { // cannot do millis operation in ISR because will not work. see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  cycle = true;
}
// Counter I interrupt service routine
void CounterI() {
  rev = true;
}


// get speed, angle
void loop() {
  // Displays Current angle of motor
  if (cycle){
    cycle_cnt += 1;
    if (cycle_time == 0){
      cycle_time = millis();    
    }
    else{
      cycle_time = millis() - cycle_time; // millis gets time since program started so, getting time difference to be more accurate. If speed of motor is too high consider changing to micros()
    }
    curr_angle += (360.0/500);
    
    Serial.println("Current Angle (degrees): ");
    Serial.println(curr_angle);
    Serial.println("Cycle count");
    Serial.println(cycle_cnt);
    cycle = false;
  }
  // Resetting count and rev_time if 1 revolution of the encoder disc occurs
  if (rev && cycle_cnt == 500){ // change number for different encoder
    if (rev_time_I == 0){
      rev_time_I = millis();
    }
    else{
      rev_time_I = millis() - rev_time_I;
    }
    speed_A = (curr_angle*1000.0/cycle_time) * (pi/curr_angle);
    speed_I = (pi/180) *((360.0*1000.0)/rev_time_I);
    speed_error = fabs(speed_A - speed_I);
  
    Serial.println("Speed from Channel A: "); // displays speed from channel A
    Serial.println(speed_A);
    Serial.println("Speed from Channel I: "); // displays speed from channel I
    Serial.println(speed_I);
    Serial.println("Speed error between channel I and channel A: "); // displays approximate error in speed in rad/s
    Serial.println(speed_error);

    // resets values
    curr_angle = 0;
    cycle_cnt = 0;
    rev = false;
    
  }

}