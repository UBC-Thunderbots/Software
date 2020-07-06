#pragma once
#include <memory.h>
#include <stdlib.h>

#include "shared/parameter_v2/c/parameter.h"

///////////////////////////////////////////////////////////////////////
//                 Generated C Config Structs                        //
///////////////////////////////////////////////////////////////////////
//
// We achieve immutability by setting IntegerParameter_t, BoolParameter_t etc
// to const. They can only be set during initialization. We also set the ptrs
// pointing to included configs as const.
//
// This allows us to expose the "internals" of the generated FooConfig struct,
// BarConfig struct, etc.. to let libraries know what parameters and configs
// are present inside.
//
typedef struct FooConfig_s
{
    const IntParameter_t* foo_int;
    const BoolParameter_t* foo_bool;
} FooConfig_t;

typedef struct BarConfig_s
{
    const IntParameter_t* bar_int;
    const BoolParameter_t* bar_bool;
} BarConfig_t;

typedef struct ExampleConfig_s
{
    // generated from an "include" statement in the yaml,
    // include:
    //      - "foo.yaml"
    //      - "bar.yaml"
    // It does NOT indicate hierarchy.
    const FooConfig_t* FooConfig;
    const BarConfig_t* BarConfig;

    // the parameters defined in example.yaml
    const BoolParameter_t* example_bool_param;
    const UIntParameter_t* example_uint_param;
    const IntParameter_t* example_int_param;
    const FloatParameter_t* example_float_param;
    const StringParameter_t* example_string_param;
} ExampleConfig_t;

typedef struct ThunderbotsConfig
{
    const ExampleConfig_t* ExampleConfig;
} ThunderbotsConfig_t;

// autogen
const ThunderbotsConfig_t* app_dynamic_parameters_create(void);
void app_dynamic_parameters_destroy(const ThunderbotsConfig_t* tbots_config);
