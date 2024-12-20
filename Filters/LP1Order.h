#ifndef LP_1_ORDER_H
#define LP_1_ORDER_H

struct LP1Order
{
    double *in;
    double *out;

    float a1;
    float b[2];

    double xn1;
    double yn1;
};

void LP1Order( struct LP1Order *filter );
#endif
