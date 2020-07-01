#pragma once
#include <stdbool.h>

// Parameters for C
//
// Parameters for C will ALWAYS be immutable. We won't have a UI updating these
// values on the robot.
//
// In the future if we chose to support sending dynamic parameters over the
// network, we can simply replace the existing param tree on the robot
// with the updated one that was received over the network.
//
// It is guaranteed that the values can NOT change after `app_dynamic_parameters_create()`
// has been called
typedef struct BoolParameter BoolParameter_t;
typedef struct StringParameter StringParameter_t;
typedef struct FloatParameter FloatParameter_t;
typedef struct IntParameter IntParameter_t;
typedef struct UIntParameter UIntParameter_t;

/*
 * Create the parameter from the raw value
 *
 * @param value The value to assign to the parameter
 */
const FloatParameter_t* app_dynamic_parameters_createFloatParameter(float value);
const IntParameter_t* app_dynamic_parameters_createIntParameter(int value);
const UIntParameter_t* app_dynamic_parameters_createUIntParameter(unsigned value);
const StringParameter_t* app_dynamic_parameters_createStringParameter(const char* value);
const BoolParameter_t* app_dynamic_parameters_createBoolParameter(bool value);

/*
 * Destroy the given parameter
 *
 * @param param The parameter to destroy
 */
void app_dynamic_parameters_destroyFloatParameter(const FloatParameter_t* param);
void app_dynamic_parameters_destroyIntParameter(const IntParameter_t* param);
void app_dynamic_parameters_destroyUIntParameter(const UIntParameter_t* param);
void app_dynamic_parameters_destroyStringParameter(const StringParameter_t* param);
void app_dynamic_parameters_destroyBoolParameter(const BoolParameter_t* param);

/*
 * Get the raw value from parameter
 *
 * @param param The parameter to get the value from
 */
float app_dynamic_parameters_getFloat(const FloatParameter_t* param);
int app_dynamic_parameters_getInt(const IntParameter_t* param);
unsigned app_dynamic_parameters_getUInt(const UIntParameter_t* param);
const char* app_dynamic_parameters_getString(const StringParameter_t* param);
bool app_dynamic_parameters_getBool(const BoolParameter_t* param);
