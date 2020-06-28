#pragma once
// Parameter for C
//
// NOTE: Parameters for C will ALWAYS be immutable.
// We don't use an API here
// they have been created. Because:
//
// - we don't want parameters to be a way of passing data between tasks, there are plenty of rtos
//   message passing techniques we can use (queues, notifications, binary-sempahores, etc...)
// - keeps the api simpler, only allow create, access and destroy of parameters/configs in c
//
// It is garunteed that don't change after `initAppDynamicParameters()` has been called
//
// legal usage:
//      ExampleConfig->FooConfig->example_bool.value
//      ExampleConfig->FooConfig->example_int.value
//
// illegal usage
//      ExampleConfig->FooConfig->example_int.value = 100
//
// The compiler will NOT allow assignment to these structs without a struct initializer.
//
// However: nothing is preventing someone from "re-initializing" the struct leading to 
// undefined behaviour. This can NOT be caught at compile time for obvious reasons.

typedef struct BoolParameter {
    const bool value;
} BoolParameter_t;

typedef struct IntegerParameter {
    const int value;
} IntegerParameter_t;

typedef struct UnsignedIntegerParameter {
    const uint32_t value;
} UnsignedIntegerParameter_t;

typedef struct FloatParameter {
    const int value;
} FloatParameter_t;

typedef struct StringParameter {
    const char* value;
} StringParameter_t;

