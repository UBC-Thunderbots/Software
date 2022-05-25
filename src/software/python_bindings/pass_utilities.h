#pragma once
#include <pybind11/pybind11.h>

#include "software/ai/passing/pass.h"

/**
 * Creates a Pass from a Python dict with the following keys and value types:
 * 'passer_point': Python bindings Point type
 * 'receiver_point': Python bindings Point type
 * 'pass_speed': double or float
 *
 * @param pass_dict: a Python dict as above.
 * @return a Pass with the same corresponding fields as the dict.
 */
Pass createPassFromDict(pybind11::dict pass_dict);

/**
 * Converts a Pass to a dict according to the keys and value types of createPassFromDict.
 * @param pass a Pass
 * @return a Python dict with the keys and values as in createPassFromDict.
 */
pybind11::dict convertPassToDict(const Pass& pass);
