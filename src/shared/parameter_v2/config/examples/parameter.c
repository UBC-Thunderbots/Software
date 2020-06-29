#include "shared/parameter_v2/config/examples/parameter.h"

#include <memory.h>
#include <stdlib.h>

typedef struct BoolParameter
{
    const bool value;
} BoolParameter_t;

typedef struct IntParameter
{
    const int value;
} IntParameter_t;

typedef struct UIntParameter
{
    const unsigned value;
} UIntParameter_t;

typedef struct FloatParameter
{
    const float value;
} FloatParameter_t;

typedef struct StringParameter
{
    const char* value;
} StringParameter_t;


float app_dynamic_parameters_getFloat(const FloatParameter_t* param)
{
    return param->value;
}
int app_dynamic_parameters_getInt(const IntParameter_t* param)
{
    return param->value;
}
unsigned app_dynamic_parameters_getUInt(const UIntParameter_t* param)
{
    return param->value;
}
const char* app_dynamic_parameters_getString(const StringParameter_t* param)
{
    return param->value;
}
bool app_dynamic_parameters_getBool(const BoolParameter_t* param)
{
    return param->value;
}

const FloatParameter_t* app_dynamic_parameters_createFloatParameter(float value)
{
    // This is the safest way to initialize a const member after malloc
    // https://stackoverflow.com/questions/9691404/how-to-initialize-const-in-a-struct-in-c-with-malloc
    FloatParameter_t* param     = (FloatParameter_t*)malloc(sizeof(FloatParameter_t));
    FloatParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(FloatParameter_t));
    return param;
}

const IntParameter_t* app_dynamic_parameters_createIntParameter(int value)
{
    // This is the safest way to initialize a const member after malloc
    // https://stackoverflow.com/questions/9691404/how-to-initialize-const-in-a-struct-in-c-with-malloc
    IntParameter_t* param     = (IntParameter_t*)malloc(sizeof(IntParameter_t));
    IntParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(IntParameter_t));
    return param;
}

const UIntParameter_t* app_dynamic_parameters_createUIntParameter(unsigned value)
{
    // This is the safest way to initialize a const member after malloc
    // https://stackoverflow.com/questions/9691404/how-to-initialize-const-in-a-struct-in-c-with-malloc
    UIntParameter_t* param     = (UIntParameter_t*)malloc(sizeof(UIntParameter_t));
    UIntParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(UIntParameter_t));
    return param;
}
const StringParameter_t* app_dynamic_parameters_createStringParameter(const char* value)
{
    // This is the safest way to initialize a const member after malloc
    // https://stackoverflow.com/questions/9691404/how-to-initialize-const-in-a-struct-in-c-with-malloc
    StringParameter_t* param     = (StringParameter_t*)malloc(sizeof(StringParameter_t));
    StringParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(StringParameter_t));
    return param;
}

const BoolParameter_t* app_dynamic_parameters_createBoolParameter(bool value)
{
    // This is the safest way to initialize a const member after malloc
    // https://stackoverflow.com/questions/9691404/how-to-initialize-const-in-a-struct-in-c-with-malloc
    BoolParameter_t* param     = (BoolParameter_t*)malloc(sizeof(BoolParameter_t));
    BoolParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(BoolParameter_t));
    return param;
}

void app_dynamic_parameters_destroyFloatParameter(const FloatParameter_t* param)
{
    free((void*)param);
}
void app_dynamic_parameters_destroyIntParameter(const IntParameter_t* param)
{
    free((void*)param);
}
void app_dynamic_parameters_destroyUIntParameter(const UIntParameter_t* param)
{
    free((void*)param);
}
void app_dynamic_parameters_destroyStringParameter(const StringParameter_t* param)
{
    free((void*)param);
}
void app_dynamic_parameters_destroyBoolParameter(const BoolParameter_t* param)
{
    free((void*)param);
}
