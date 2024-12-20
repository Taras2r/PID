#ifndef _INC_STDIO
#include<stdio.h>
#endif

#ifndef LP_1_ORDER_H
#include"LP1Order.h"
#endif

// The all details are described in ReadMe.md file.

// H(s) = w0 / ( s + w0 ) - Transfer Function Laplace Domain
// General representation of discrete time differential equation
// y[n] = a1y[n − 1] + a2y[n − 2] + ... + b0x[n] + b1x[n − 1] + ...
// After applying Tustin's transform the Transfer Function, the following equation has
// been gotten
// y[n] = 0.96906992y[n − 1] + 0.1546504x[n] + 0.01546504x[n − 1]

// a0 = -1, a1 = 0.96906992
// b0 = 0.1546504, b1 = 0.01546504



//static float tmpIn;
//static float tmpOut;
//static struct LP1Order filter =
//{
//        .in = &tmpIn,
//        .in = &tmpOut,
//
//        .a1 = 0.96906992,
//        .b0 = 0.1546504,
//        .b1 = 0.01546504,
//};
void LP1Order( struct LP1Order *filter )
{
    *filter->out =
       filter->a1*filter->yn1 + filter->b[0]*(*filter->in) + filter->b[1]*filter->xn1;

    filter->yn1 = *filter->out;
    filter->xn1 = *filter->in;
}
