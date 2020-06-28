#pragma once
#include <stdbool.h>

// Parameter for C
//
// Parameters for C will ALWAYS be immutable. We won't have a UI updating these
// values on the robot.
//
// In the future if we chose to support sending dynamic parameters over the
// network, we can simply replace the existing param tree on the robot
// with the updated one.
//
// We don't use an psuedo-class here to keep things simple as we can make use of
// compile time checks to provide immutability. (without hiding the contents of
// the struct in the c file)
//
// It is garunteed that the values don't change after `app_dynamic_parameters_create()`
// has been called
//
// The compiler will NOT allow assignment to these structs without a struct initializer.
// These struct initializers are auto generated from the yaml, OR can be created manually
// when needed for testing.
typedef struct BoolParameter BoolParameter_t;
typedef struct StringParameter StringParameter_t;
typedef struct FloatParameter FloatParameter_t;
typedef struct IntegerParameter IntegerParameter_t;
typedef struct UnsignedIntegerParameter UnsignedIntegerParameter_t;

float app_dynamic_parameters_getFloatValue(const FloatParameter_t* param);
int app_dynamic_parameters_getIntegerValue(const IntegerParameter_t* param);
unsigned app_dynamic_parameters_getUnsignedIntegerValue(
    const UnsignedIntegerParameter_t* param);
const char* app_dynamic_parameters_getStringValue(const StringParameter_t* param);
bool app_dynamic_parameters_getBoolValue(const BoolParameter_t* param);

const FloatParameter_t* app_dynamic_parameters_createFloatParameter(float value);
const IntegerParameter_t* app_dynamic_parameters_createIntegerParameter(int value);
const UnsignedIntegerParameter_t* app_dynamic_parameters_createUnsignedIntegerParameter(
    unsigned value);
const StringParameter_t* app_dynamic_parameters_createStringParameter(const char* value);
const BoolParameter_t* app_dynamic_parameters_createBoolParameter(bool value);

void app_dynamic_parameters_destroyFloatParameter(const FloatParameter_t* param);
void app_dynamic_parameters_destroyIntegerParameter(const IntegerParameter_t* param);
void app_dynamic_parameters_destroyUnsignedIntegerParameter(
    const UnsignedIntegerParameter_t* param);
void app_dynamic_parameters_destroyStringParameter(const StringParameter_t* param);
void app_dynamic_parameters_destroyBoolParameter(const BoolParameter_t* param);
