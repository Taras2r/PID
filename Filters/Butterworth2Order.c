#ifndef _INC_STDIO
#include<stdio.h>
#endif

#ifndef LP_BW_2_ORDER_H
#include"Butterworth2Order.h"
#endif



void Butterworth2Order( struct BW2Order *filter )
{
    *filter->out = filter->a[0]*filter->y[0] + filter->a[1]*filter->y[1]
                                + filter->b[0]*(*filter->in) +
                                filter->b[1]*filter->x[0] + filter->b[2]*filter->x[1];

    filter->x[1] = filter->x[0];
    filter->y[1] = filter->y[0];
    filter->x[0] = *filter->in;
    filter->y[0] = *filter->out;
}
