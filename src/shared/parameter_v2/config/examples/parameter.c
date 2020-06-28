#include "shared/parameter_v2/config/examples/parameter.h"

#include <memory.h>
#include <stdlib.h>

typedef struct BoolParameter
{
    const bool value;
} BoolParameter_t;

typedef struct IntegerParameter
{
    const int value;
} IntegerParameter_t;

typedef struct UnsignedIntegerParameter
{
    const unsigned value;
} UnsignedIntegerParameter_t;

typedef struct FloatParameter
{
    const float value;
} FloatParameter_t;

typedef struct StringParameter
{
    const char* value;
} StringParameter_t;


float app_dynamic_parameters_getFloatValue(const FloatParameter_t* param)
{
    return param->value;
}
int app_dynamic_parameters_getIntegerValue(const IntegerParameter_t* param)
{
    return param->value;
}
unsigned app_dynamic_parameters_getUnsignedIntegerValue(
    const UnsignedIntegerParameter_t* param)
{
    return param->value;
}
const char* app_dynamic_parameters_getStringValue(const StringParameter_t* param)
{
    return param->value;
}
bool app_dynamic_parameters_getBoolValue(const BoolParameter_t* param)
{
    return param->value;
}

const FloatParameter_t* app_dynamic_parameters_createFloatParameter(float value)
{
    FloatParameter_t* param     = (FloatParameter_t*)malloc(sizeof(FloatParameter_t));
    FloatParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(FloatParameter_t));
    return param;
}

const IntegerParameter_t* app_dynamic_parameters_createIntegerParameter(int value)
{
    IntegerParameter_t* param = (IntegerParameter_t*)malloc(sizeof(IntegerParameter_t));
    IntegerParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(IntegerParameter_t));
    return param;
}

const UnsignedIntegerParameter_t* app_dynamic_parameters_createUnsignedIntegerParameter(
    unsigned value)
{
    UnsignedIntegerParameter_t* param =
        (UnsignedIntegerParameter_t*)malloc(sizeof(UnsignedIntegerParameter_t));
    UnsignedIntegerParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(UnsignedIntegerParameter_t));
    return param;
}
const StringParameter_t* app_dynamic_parameters_createStringParameter(const char* value)
{
    StringParameter_t* param     = (StringParameter_t*)malloc(sizeof(StringParameter_t));
    StringParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(StringParameter_t));
    return param;
}

const BoolParameter_t* app_dynamic_parameters_createBoolParameter(bool value)
{
    BoolParameter_t* param     = (BoolParameter_t*)malloc(sizeof(BoolParameter_t));
    BoolParameter_t param_init = {.value = value};
    memcpy(param, &param_init, sizeof(BoolParameter_t));
    return param;
}

void app_dynamic_parameters_destroyFloatParameter(const FloatParameter_t* param)
{
    free((void*)param);
}
void app_dynamic_parameters_destroyIntegerParameter(const IntegerParameter_t* param)
{
    free((void*)param);
}
void app_dynamic_parameters_destroyUnsignedIntegerParameter(
    const UnsignedIntegerParameter_t* param)
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
