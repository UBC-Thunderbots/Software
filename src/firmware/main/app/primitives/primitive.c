#include "firmware/main/app/primitives/primitive.h"

bool primitive_params_are_equal(primitive_params_t *params1, primitive_params_t *params2)
{
    bool equal = true;

    if (params1->slow != params2->slow)
    {
        equal = false;
    }

    if (equal && params1->extra != params2->extra)
    {
        equal = false;
    }

    if (equal)
    {
        for (int i = 0; i < 4; i++)
        {
            if (params1->params[i] != params2->params[i])
            {
                equal = false;
            }
        }
    }

    return equal;
}
