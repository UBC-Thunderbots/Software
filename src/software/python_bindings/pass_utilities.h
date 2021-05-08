#pragma once
#include <pybind11/pybind11.h>

#include "software/ai/passing/pass.h"

Pass createPassFromDict(pybind11::dict pass_dict);

pybind11::dict convertPassToDict(const Pass& pass);
