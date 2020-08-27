#include "software/simulated_tests/validation_functions/in_region_validation.h"

#include "software/geom/algorithms/contains.h"

void objectInRegion(Point object_position, Rectangle region,
                     ValidationCoroutine::push_type& yield)
{
    auto object_in_region = [object_position, region] {
        return contains(region, object_position);
    };

    while (!object_in_region()) 
    {
        yield();
    }
}