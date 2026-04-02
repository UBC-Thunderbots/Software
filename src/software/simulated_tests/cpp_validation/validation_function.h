#pragma once

#include <boost/coroutine2/all.hpp>
#include <functional>

#include "software/world/world.h"

// The Validation Coroutine yields an error message in the form of a string when
// the passing condition is not true
using ValidationCoroutine = boost::coroutines2::coroutine<std::string>;
// The Validation Function analyses the world and yields an error message as appropriate
using ValidationFunction =
    std::function<void(std::shared_ptr<World>, ValidationCoroutine::push_type&)>;
