#include "PID.h"

#ifndef _INC_STDIO
#include<stdio.h>
#endif

void PIDController_Init(PIDController *pid) {

    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;

    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;

}
//Check theory by the below link
//https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

    /*
    * Error signal
    */
    float error = setpoint - measurement;

    /*
    * Proportional
    */
    float proportional = pid->Kp * error;


    /*
    * Integral
    */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


    /*
    * Derivative (band-limited differentiator)
    */

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)   /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

//    pid->differentiator = pid->Kd * (error - pid->prevError) * -1.0;

//    printf("%f\n", pid->differentiator);


    /*
    * Compute output and apply limits
    */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

    /* Store error and measurement for later use */
    pid->measurement = pid->prevMeasurement;
    pid->error = pid->prevError;
    pid->prevError       = error;
    pid->prevMeasurement = measurement;


    /* Return controller output */
    return pid->out;

}
