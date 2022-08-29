#include <iostream>
#include <Motor.h>
#include <Encoder.h>
#define KP 0.00001
#define KI 0.0000001
#define KD 0.00001

int pidCalc(double desiredRPM, double currentRPM){

    error = desiredRPM - currentRPM;
    errorSum = errorSum + (error*CONTROL_LOOP_TIME);
    errorRateOfChange = (error - previousError)/CONTROL_LOOP_TIME;
    previousError = error;
    output += ((KP*error)+(KI*errorSum)+(KD*errorRateOfChange));
}

void set_motor(Motor* motor, Encoder* encoder, double desiredRPM){
    double curr = encoder->getRPM():
    motor->setRPM(pidCalc(desiredRPM, curr));

}

int main() {
    Motor* motor = new Motor();
    Encoder* encoder = new Encoder():
    double desired = 500;
    while(true){
        set_motor(motor, 500);


    }

}
