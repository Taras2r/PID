#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "PID.h"
#include "Filters\LP1Order.h"
#include"Filters\Butterworth2Order.h"

//https://github.com/pms67/PID/tree/master
//https://youtu.be/zOByx3Izf5U?si=gVV6eAvSPnZyn5_2

/* Controller parameters */
#define PID_KP  90.0f
#define PID_KI  0.0f
#define PID_KD 0.001f

#define PID_TAU 0.0000001f

#define PID_LIM_MIN -3.40282347e+38F
#define PID_LIM_MAX  3.40282347e+38F

#define PID_LIM_MIN_INT -10000.0f
#define PID_LIM_MAX_INT  10000.0f

#define SAMPLE_TIME_S 0.001f //1ms used for calculation response of the chosen system

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 15.0f //set in seconds

//Determine the needed buffer size to store data vectors
#define ITERATIONS 15001 // SIMULATION_TIME_MAX * SAMPLE_TIME_S todo recalculate
                        //ITERATIONS whenever SIMULATION_TIME_MAX or SAMPLE_TIME_S
                        //is changed
#define PID_SAMPLE_RATE 25
#define SYSTEM_TIME_CONSTANT 0.5

/* Simulated dynamic system (first order) */
float TestSystem_Update( float inp );

/*Generate the sine function to introduce a noise to the system response*/
double sinus( double currTime )
{
    double freq = 1.0;

    double angle = sin( 2 * M_PI * freq * currTime );

    return angle;
}
//float secondOrderSystem( float input );

struct secondOrderDerevative
{
    float U[ITERATIONS];
    float Ud[ITERATIONS];
    float Y[ITERATIONS];
    float Yd[ITERATIONS];
    float Ydd[ITERATIONS];

    const float timeConstant;

    int i;
};

struct firstOrderDerevative
{
    float U[ITERATIONS];
    float Y[ITERATIONS];
    float Yd[ITERATIONS];

    //A time which represents the speed with which a particular system can respond to
    //change, typically equal to the time taken for a specified parameter to vary by
    //a factor of 1− 1 / e (approximately 0.6321).
    const float timeConstant; // set in seconds

    int i;
};

struct integratorDerevative
{
    float U[ITERATIONS];
    float Y[ITERATIONS];

    //A time which represents the speed with which a particular system can respond to
    //change, typically equal to the time taken for a specified parameter to vary by
    //a factor of 1− 1 / e (approximately 0.6321).
    const float timeConstant; // set in seconds

    int i;
};

// 1 / (timeConstant*s + 1)
static struct firstOrderDerevative openLoopFirstOrder =
{
        .timeConstant = SYSTEM_TIME_CONSTANT,
};
static struct firstOrderDerevative closedLoopFirstOrder =
{
        .timeConstant = SYSTEM_TIME_CONSTANT,
};
float firstOrderSystem( struct firstOrderDerevative *storage, float input )
{

    storage->U[storage->i] = input;

    if( storage->i == 0 )
    {
        float U0 = 0;
        float Y0 = 0;

        storage->U[storage->i] = U0;
        storage->Y[storage->i] = Y0;

        storage->Yd[storage->i] = (storage->U[storage->i] - storage->Y[storage->i]) /
                                                                storage->timeConstant;
        debugFirstOrder( "%f\n", Yd[0] );
    }
    else
    {
        storage->Y[storage->i] = storage->Y[storage->i-1] + SAMPLE_TIME_S *
                                                            storage->Yd[storage->i-1];

        storage->Yd[storage->i] = (storage->U[storage->i] - storage->Y[storage->i]) /
                                                                storage->timeConstant;
        debugFirstOrder( "%f;\t", Y[storage->i] );
        debugFirstOrder( "%f\n\r", Yd[storage->i] );
    }

    storage->i++;

    return storage->Y[storage->i-1];
}

// Y(s)/U(s) = s + 2 / s^2 + s + 2
static struct secondOrderDerevative openLoopSecondOrder =
{
    .timeConstant = SYSTEM_TIME_CONSTANT,
};
static struct secondOrderDerevative closedLoopSecondOrder =
{
    .timeConstant = SYSTEM_TIME_CONSTANT,
};

float secondOrderSystem( struct secondOrderDerevative *storage, float input )
{
    float Ud0 = 0;
    float Y0 = 0;
    float Yd0 = 0;

    storage->U[storage->i] = input;

    if( storage->i == 0 )
    {
        storage->Ud[storage->i] = Ud0;
        storage->Yd[storage->i] = Yd0;
        storage->Y[storage->i] = Y0;

        storage->Ydd[storage->i] = storage->Ud[storage->i] +
                                                     2.0 * storage->U[storage->i] -
                                                          storage->Yd[storage->i] -
                                                         2.0 * storage->Y[storage->i];
        debug( "%f\n", storage->Ydd[0] );
    }
    else
    {
        storage->Ud[storage->i] =
                ( storage->U[storage->i] - storage->U[storage->i-1] ) / SAMPLE_TIME_S;
        storage->Yd[storage->i] = storage->Yd[storage->i-1] + SAMPLE_TIME_S *
                                                           storage->Ydd[storage->i-1];
        storage->Y[storage->i] = storage->Y[storage->i-1] + SAMPLE_TIME_S *
                                                            storage->Yd[storage->i-1];

        storage->Ydd[storage->i] = storage->Ud[storage->i] +
                                                        2.0 * storage->U[storage->i] -
                                                             storage->Yd[storage->i] -
                                                         2.0 * storage->Y[storage->i];
        debug( "%f;\t", storage->Ud[storage->i] );
        debug( "%f;\t", storage->Yd[storage->i] );
        debug( "%f;\t", storage->Y[storage->i] );
        debug( "%f\n\r", storage->Ydd[storage->i] );
    }

    storage->i++;

    return storage->Y[storage->i-1];
}

static struct integratorDerevative openLoopIntegrator =
{
    .timeConstant = SYSTEM_TIME_CONSTANT,
};
static struct integratorDerevative closedLoopIntegrator =
{
    .timeConstant = SYSTEM_TIME_CONSTANT,
};
float integratorTransferFunction( struct integratorDerevative *storage, float input )
{
//    static unsigned int i;
//    static float timeConst = 1;

//    static float U[ITERATIONS] = {0};
//    static float Y[ITERATIONS] = {0};

    if( storage->i == 0 )
    {
        storage->U[storage->i] = 0;
        storage->Y[storage->i] = storage->U[storage->i] * SAMPLE_TIME_S /
                                       storage->timeConstant + storage->Y[storage->i];
    }
    else
    {
        storage->U[storage->i] = input;
        storage->Y[storage->i] = storage->U[storage->i] * SAMPLE_TIME_S /
                                    storage->timeConstant + storage->Y[storage->i-1];
    }

    storage->i++;

    return storage->Y[storage->i-1];
}

/******************************************************/
//Main signal to test 1st Order LP-Filter
struct Signal
{
    double freq;
    double magnitude;
    double phase;
};
static struct Signal signal =
{
        .freq = 2.0,
        .magnitude = 1.0,
        .phase = 0.0,
};
static struct Signal noise =
{
        .freq = 50.0,
        .magnitude = 0.2,
        .phase = 0.0,
};
double sinus2( double currTime, struct Signal signal )
{
    double angle =
           signal.magnitude * sin( 2 * M_PI * signal.freq * currTime + signal.phase );

    return angle;
}
/******************************************************/
int main( void )
{
    /*Sore the data to the file for further processing them with Python*/
    FILE *fp;
    fp = fopen( "PID.csv", "w" );
    fprintf( fp,
             "Time (s),"\
             "First Order System Output Open Loop,"\
             "First Order System Output Closed Loop,"\
             "Second Order System Output Open Loop,"\
             "Second Order System Output Closed Loop,"\
             "Integrator Output Open Loop,"\
             "Integrator Output Closed Loop,"\
             "Sinus,"\
             "Noisy Fisrt Order,"\
             "Derivative,"\
             "Prev Err,"\
             "Curr Err,"\
             "Filter in,"\
             "Filter out,"\
             "Filter BW out\n"
            );

    /* Initialize PID controllers */
    PIDController pid1 = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
                          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S*PID_SAMPLE_RATE };
                                             //PID controller is processed with 10 times
                                             //slower sample rate than the system response
                                             //calculations are done to simulate discrete
                                             //implementation of the controller

    PIDController pid2 = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
                          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S*PID_SAMPLE_RATE };
                                             //PID controller is processed with 10 times
                                             //slower sample rate than the system response
                                             //calculations are done to simulate discrete
                                             //implementation of the controller

    PIDController pid3 = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
                          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S*PID_SAMPLE_RATE };
                                             //PID controller is processed with 10 times
                                             //slower sample rate than the system response
                                             //calculations are done to simulate discrete
                                             //implementation of the controller

    float setpoint = 1.0f;
    PIDController_Init( &pid1 );
    PIDController_Init( &pid2 );

    for( float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S )
    {
        static float measurement1order;
        static float measurement2order;
        static float measurementIntegrator;
        static unsigned int counter;

        //Unit Step at send to the systems after 1 second
        float stepInput = 0.0;

        if( t >= 1 )
        {
            stepInput = t * 10.0;
        }
        if( stepInput > 1.0 )
        {
            stepInput = 1.0;
        }
        debug( "Input %f;\t", stepInput );


        float firstOrder = 0;
        float secondOrder = 0;
        float integrator = 0;

        double sinSig;
        double mainSig;
        double noiseSig;
        static double testSig;
        sinSig = sinus( t );
        mainSig = sinus2( t, signal );
        noiseSig = sinus2( t, noise );
        testSig = mainSig + noiseSig;

        static double filterOut;
        static double filterBWOut;
        static struct LP1Order filter =
        {
                .in = &testSig,
                .out = &filterOut,

                .a1 = 0.96906992,
                .b[0] = 0.01546504,
                .b[1] = 0.01546504,
        };
        static struct BW2Order filterBW =
        {
                .in = &testSig,
                .out = &filterBWOut,

                .a[0] = 1.95558189,
                .a[1] = -0.95654717,
                .b[0] = 0.00024132,
                .b[1] = 0.00048264,
                .b[2] = 0.00024132,
        };

        LP1Order( &filter );
        Butterworth2Order( &filterBW );

        firstOrder = firstOrderSystem( &openLoopFirstOrder, stepInput );
        measurement1order = firstOrderSystem( &closedLoopFirstOrder, pid1.out );

        secondOrder = secondOrderSystem( &openLoopSecondOrder, stepInput );
        double noisyFirstOrder = ( 0.1 * sinSig ) + (double)secondOrder;

        if( 5.0 < t && t < 8.0 )
        {
        measurement2order = ( 0.4 * sinSig ) + secondOrderSystem( &closedLoopSecondOrder, pid2.out );
        }
        else
        {
        measurement2order = secondOrderSystem( &closedLoopSecondOrder, pid2.out );
        }

        integrator = integratorTransferFunction( &openLoopIntegrator, stepInput );
        measurementIntegrator =
                        integratorTransferFunction( &closedLoopIntegrator, pid3.out );


        counter++;
        if( counter >= PID_SAMPLE_RATE && t >= 1 )
        {
            /* Compute new control signal */
//            PIDController_Update( &pid1, setpoint, measurement1order );
            PIDController_Update( &pid2, setpoint, measurement2order );
//            PIDController_Update( &pid3, setpoint, measurementIntegrator );
            counter = 0;
        }

        fprintf( fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                 t,
                 firstOrder,
                 measurement1order,
                 secondOrder,
                 measurement2order,
                 integrator,
                 measurementIntegrator,
                 sinSig,
                 noisyFirstOrder,
                 pid2.differentiator,
                 pid2.measurement,
                 pid2.prevMeasurement,
                 testSig,
                 filterOut,
                 filterBWOut
                );
    }

    fclose( fp );

    return 0;
}

float TestSystem_Update(float inp)
{
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
