#pragma once
#include "shared/parameter_v2/parameter.h"

///////////////////////////////////////////////////////////////////////
//                 Generated C Config Structs                        //
///////////////////////////////////////////////////////////////////////
//
// We acheive immutability by setting IntegerParameter_t, BoolParameter_t etc
// to const. They can only be set during initialization. We also set the ptrs
// pointing to included configs as const.
//
// This allows us to expose the "internals" of the generated FooConfig struct,
// BarConfig struct, etc.. to let libraries know what parameters and configs 
// are present inside.
//
typedef struct FooConfig_s {
    IntegerParameter_t foo_int;
    BoolParameter_t foo_bool;
} FooConfig_t;

typedef struct BarConfig_s {
    IntegerParameter_t bar_int;
    BoolParameter_t bar_bool;
} BarConfig_t;

typedef struct ExampleConfig_s {
    // generated from an "include" statement in the yaml,
    // include:
    //      - "foo.yaml"
    //      - "bar.yaml"
    // It does NOT indicate heirarchy.
    //
    // If FooConfig is included in a bunch of other configs,
    // it will still point to the same config.
    const FooConfig_t* FooConfig;
    const BarConfig_t* BarConfig;

    // the parameters defined in example.yaml
    const BoolParameter_t example_bool_param;
    const UnsignedIntegerParameter_t example_unsigned_interger_param;
    const IntegerParameter_t example_interger_param;
    const FloatParameter_t example_float_param;
    const StringParameter_t example_string_param;
} ExampleConfig_t;

typedef struct ThunderbotsConfig {
    const ExampleConfig_t* ExampleConfig;
} ThunderbotsConfig_t;

// this function should get called in the initAppLayer() call in main.c
const ThunderbotsConfig_t* initAppDynamicParameters() {

    /****
     * Generate lines to allocate memory for all configs *
     ****/

    ThunderbotsConfig_t* thunderbots_config = (ThunderbotsConfig_t*)malloc(sizeof(ThunderbotsConfig_t));
    ExampleConfig_t* example_config = (ExampleConfig_t*)malloc(sizeof(ExampleConfig_t));
    FooConfig_t* foo_config = (FooConfig_t*)malloc(sizeof(FooConfig_t));
    BarConfig_t* bar_config = (BarConfig_t*)malloc(sizeof(BarConfig_t));

    /****
    * Create "initialization structs" *
    ****/
    
    ExampleConfig example_config_init = {
        .example_bool_param = {
            .value = true
        },
        .example_int_param = {
            .min = 0,
            .max = 5,
            .value = 3
        },
        .example_unsigned_int_param = {
            .min = 0,
            .max = 5,:
                .value = 3
        },
        .example_float_param {
            .min = 1.1,
            .max = 9.01,
            .value = 4.04
        }
        .example_string_param {
            .value = "Hello World"
        }
    };

    BarConfig_t bar_config_init = {
        .bar_bool = {
            .value = true
        },
        .bar_int = {
            .min = 0,
            .max = 5,
            .value = 3
        }
    };

    FooConfig foo_config_init = {
        .foo_bool = {
            .value = true
        },
        .foo_int = {
            .min = 0,
            .max = 5,
            .value = 3
        }
    };

    /****
    * Initialize Configs *
    * 
    * Memcpy allows us to NOT hit any undefined behaviour when initializing a const param after malloc
    * We copy all the values from the stack-allocated struct into the heap-malloc'd struct.
    *
    * https://stackoverflow.com/questions/9691404/how-to-initialize-const-in-a-struct-in-c-with-malloc
    ****/

    memcpy(example_config, &example_config_init, sizeof(struct ExampleConfig_t));
    memcpy(bar_config, &bar_config_init, sizeof(struct BarConfig_t));
    memcpy(foo_config, &foo_config_init, sizeof(struct FooConfig_t));

    /****
    * Resolve Includes *
    *
    * We "link" the configs together when requested w/ an include statement in the yaml
    ****/
    thunderbots_config->ExampleConfig = example_config;
    example_config->FooConfig = foo_config;
    example_config->BarConfig = bar_config;
}
