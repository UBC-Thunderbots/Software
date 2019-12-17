#include "dsp.h"

float runDF2(float input, const float* num, const float* den, float* state, size_t order)
{
    float accum = 0;
    for (size_t i = 0; i < order; ++i)
    {
        input -= state[i] * den[i + 1];
    }
    input /= den[0];

    for (size_t i = order - 1; i != 0; --i)
    {
        accum += state[i] * num[i + 1];
        state[i] = state[i - 1];
    }
    accum += state[0] * num[1];
    state[0] = input;
    accum += input * num[0];
    return accum;
}
