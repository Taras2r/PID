#ifndef LP_BW_2_ORDER_H
#define LP_BW_2_ORDER_H
#endif

//https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/LowPass/ButterworthFilter.ipynb
struct BW2Order
{
    double *in;
    double *out;

    float a[2];
    float b[3];

    double x[2];
    double y[2];
};

void Butterworth2Order( struct BW2Order *filter );
