#pragma once

#include <boost/coroutine2/all.hpp>
#include <functional>

#include "software/world/world.h"

using ValidationCoroutine = boost::coroutines2::coroutine<std::string>;
using ValidationFunction =
    std::function<void(std::shared_ptr<World>, ValidationCoroutine::push_type&)>;
