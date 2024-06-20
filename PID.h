#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//#define DEBUG_PID
#ifdef DEBUG_PID
#define debug(text_format, ...)  printf( text_format, ##__VA_ARGS__ )
#else
  #define debug(text_format, ...)
#endif

typedef struct {

    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Integrator limits */
    float limMinInt;
    float limMaxInt;

    /* Sample time (in seconds) */
    float T;

    /* Controller "memory" */
    float integrator;
    float prevError;            /* Required for integrator */
    float differentiator;
    float prevMeasurement;      /* Required for differentiator */

    /* Controller output */
    float out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
