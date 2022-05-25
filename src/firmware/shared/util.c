#include "firmware/shared/util.h"

#include "firmware/shared/math/tbots_math.h"

float fmax_of_array(float array[], unsigned size)
{
    unsigned i;
    float max_value = 0.0f;
    for (i = 0; i < size; i++)
    {
        if (i == 0)
        {
            max_value = array[i];
        }
        else if (array[i] > max_value)
        {
            max_value = array[i];
        }
    }
    return max_value;
}


float fmin_of_array(float array[], unsigned size)
{
    unsigned i;
    float min_value = 0.0f;
    for (i = 0; i < size; i++)
    {
        if (i == 0)
        {
            min_value = array[i];
        }
        else if (array[i] < min_value)
        {
            min_value = array[i];
        }
    }
    return min_value;
}

unsigned argmax(float array[], unsigned size)
{
    unsigned i;
    float max_value;
    unsigned max_index = 0;
    for (i = 0; i < size; i++)
    {
        if (i == 0)
        {
            max_value = array[i];
        }
        else if (array[i] > max_value)
        {
            max_value = array[i];
            max_index = i;
        }
    }
    return max_index;
}

unsigned argmin(float array[], unsigned size)
{
    unsigned i;
    float min_value;
    unsigned min_index = 0;
    for (i = 0; i < size; i++)
    {
        if (i == 0)
        {
            min_value = array[i];
        }
        else if (array[i] < min_value)
        {
            min_value = array[i];
            min_index = i;
        }
    }
    return min_index;
}


void fabs_of_array(float array[], float abs_array[], unsigned size)
{
    unsigned i;
    for (i = 0; i < size; i++)
    {
        abs_array[i] = fabsf(array[i]);
    }
}


void limit(float *value, float limiting_value)
{
    clamp(value, -limiting_value, limiting_value);
}


void clamp(float *value, float lower, float upper)
{
    if (*value > upper)
    {
        *value = upper;
    }
    if (*value < lower)
    {
        *value = lower;
    }
}

float radians(float degrees)
{
    return degrees * P_PI / 180.0f;
}
