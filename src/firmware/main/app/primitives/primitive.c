#include "firmware/main/app/primitives/primitive.h"
#include "shared/proto/primitive_fw.pb.h"

bool primitives_are_equal(RadioPrimitive* prim1, RadioPrimitive* prim2)
{
    bool equal = true;

    if (prim1->slow != prim2->slow)
    {
        equal = false;
    }

    if (equal && prim1->extra_bits != prim2->extra_bits)
    {
        equal = false;
    }

    if (equal)
    {
            if (prim1->parameter1 != prim2->parameter2)
            {
                equal = false;
            }
            if (prim1->parameter2 != prim2->parameter2)
            {
                equal = false;
            }
            if (prim1->parameter3 != prim2->parameter2)
            {
                equal = false;
            }
            if (prim1->parameter4 != prim2->parameter2)
            {
                equal = false;
            }
    }

    return equal;
}
