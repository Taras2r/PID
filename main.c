#include <stdio.h>
#include <stdlib.h>

#include "PID.h"

//https://github.com/pms67/PID/tree/master
//https://youtu.be/zOByx3Izf5U?si=gVV6eAvSPnZyn5_2

/* Controller parameters */
#define PID_KP  20.0f
#define PID_KI  0.2f
#define PID_KD  0.0f

#define PID_TAU 0.15f

#define PID_LIM_MIN -3.40282347e+38F//-10000000000.0f
#define PID_LIM_MAX  3.40282347e+38F

#define PID_LIM_MIN_INT -10000.0f
#define PID_LIM_MAX_INT  10000.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 150.0f

#define ITERATIONS 1501 // SIMULATION_TIME_MAX * SAMPLE_TIME_S todo recalculate
                        //ITERATIONS whenever SIMULATION_TIME_MAX or SAMPLE_TIME_S
                        //is changed

/* Simulated dynamical system (first order) */
float TestSystem_Update(float inp);


//float secondOrderSystem( float input );

struct secOrderDerevative
{
    float U[ITERATIONS];
    float Ud[ITERATIONS];
    float Y[ITERATIONS];
    float Yd[ITERATIONS];
    float Ydd[ITERATIONS];

    int i;
};

static struct secOrderDerevative openLoopDataVector;
static struct secOrderDerevative closedLoopDataVector;
float secondOrderSystem( struct secOrderDerevative *storage, float input )
{
//    static int i = 0;

//    static float U0 = 0;
    static float Ud0 = 0;
    static float Y0 = 0;
    static float Yd0 = 0;
//    static float Ydd0 = 0;


    storage->U[storage->i] = input;

    if( storage->i == 0 )
    {
        storage->Ud[storage->i] = Ud0;
        storage->Yd[storage->i] = Yd0;
        storage->Y[storage->i] = Y0;

        storage->Ydd[storage->i] = storage->Ud[storage->i] + 2.0 * storage->U[storage->i] -
                                                storage->Yd[storage->i] - 2.0 * storage->Y[storage->i];
        debug("%f\n", storage->Ydd[0]);
    }
    else
    {
        storage->Ud[storage->i] = ( storage->U[storage->i] - storage->U[storage->i-1] ) / SAMPLE_TIME_S;
        storage->Yd[storage->i] = storage->Yd[storage->i-1] + SAMPLE_TIME_S * storage->Ydd[storage->i-1];
        storage->Y[storage->i] = storage->Y[storage->i-1] + SAMPLE_TIME_S * storage->Yd[storage->i-1];

        storage->Ydd[storage->i] = storage->Ud[storage->i] + 2.0 * storage->U[storage->i] -
                                                storage->Yd[storage->i] - 2.0 * storage->Y[storage->i];
        debug("%f;\t", storage->Ud[storage->i]);
        debug("%f;\t", storage->Yd[storage->i]);
        debug("%f;\t", storage->Y[storage->i]);
        debug("%f\n\r", storage->Ydd[storage->i]);
    }

    storage->i++;

    return storage->Y[storage->i-1];
}

int main()
{
    /* Initialise PID controller */
    PIDController pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
                          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };

    PIDController_Init(&pid);

    /* Simulate response using test system */
    float setpoint = 1.0f;

    FILE *fp;
    fp = fopen("PID.csv", "w");
    fprintf(fp, "Time (s),System Output,ControllerOutput\n");
    for (float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S)
    {
        static float measurement;
        static unsigned int counter;

        /* Get measurement from system */
//        float measurement = TestSystem_Update(pid.out);

        float stepInput = 0.0;

        if( t > 0.05 )
        {
            stepInput = t * 10.0;
        }

//        stepInput = t * 10.0;
        if( stepInput > 1.0 )
        {
            stepInput = 1.0;
        }
        debug("Input %f;\t", stepInput);

//        float measurement = secondOrderSystem(stepInput);


//        float measurement = TestSystem_Update(10);


        /* Compute new control signal */
        PIDController_Update(&pid, setpoint, measurement);

        counter++;
        if( counter >= 10 )
        {
            float secondOrder = 0;
//            measurement = secondOrderSystem(stepInput);
            secondOrder = secondOrderSystem(&closedLoopDataVector,stepInput);
            measurement = secondOrderSystem(&openLoopDataVector, pid.out);
            fprintf(fp, "%f,%f,%f\n", t, measurement, secondOrder);
            counter = 0;
        }




//        printf("%f\t%f\t%f\r\n", t, measurement, pid.out);
//        fprintf(fp, "%f,%f,%f\n", t, measurement, pid.out);

    }

    fclose(fp);

    return 0;
}

float TestSystem_Update(float inp) {

    static float output = 0.0f;
    static const float alpha = 0.02f;

    output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);

    return output;
}

// Y(s)/U(s) = s + 2 / s^2 + s + 2
// https://youtu.be/nkq4WkX7CFU?si=adVHCcCmSzxs5mi9


//float secondOrderSystem( float input )
//{
//    static int i = 0;
//
////    static float U0 = 0;
//    static float Ud0 = 0;
//    static float Y0 = 0;
//    static float Yd0 = 0;
////    static float Ydd0 = 0;
//
//    static float U[ITERATIONS] = {0};
//    static float Ud[ITERATIONS] = {0};
//    static float Y[ITERATIONS] = {0};
//    static float Yd[ITERATIONS] = {0};
//    static float Ydd[ITERATIONS] = {0};
//
//
//    U[i] = input;
//
//    if( i == 0 )
//    {
//        Ud[i] = Ud0;
//        Yd[i] = Yd0;
//        Y[i] = Y0;
//
//        Ydd[i] = Ud[i] + 2.0 * U[i] - Yd[i] - 2.0 * Y[i];
//        debug("%f\n", Ydd[0]);
//    }
//    else
//    {
//        Ud[i] = ( U[i] - U[i-1] ) / SAMPLE_TIME_S;
//        Yd[i] = Yd[i-1] + SAMPLE_TIME_S * Ydd[i-1];
//        Y[i] = Y[i-1] + SAMPLE_TIME_S * Yd[i-1];
//
//        Ydd[i] = Ud[i] + 2.0 * U[i] - Yd[i] - 2.0 * Y[i];
//        debug("%f;\t", Ud[i]);
//        debug("%f;\t", Yd[i]);
//        debug("%f;\t", Y[i]);
//        debug("%f\n\r", Ydd[i]);
//    }
//
//    i++;
//
//    return Y[i-1];
//}
