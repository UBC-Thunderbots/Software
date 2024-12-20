#pragma once

#include <torch/torch.h>

// PyTorch defines its own LOG macro, which conflicts with our custom LOG in logger.h.
// Override LOG with our custom macro that we saved onto the macro stack
// (if the preprocessor included logger.h before this header file).
#pragma pop_macro("LOG")
