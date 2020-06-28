#pragma once
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
