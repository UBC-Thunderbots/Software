#pragma once

#include "software/util/make_enum/reflective_enum.h"

template <typename TState, typename TAction>
class QFunction
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");

   public:
    QFunction() = delete;
    ~QFunction() = delete;
    
    virtual void update(const TState& state, const TState& new_state,
                        const TAction::Enum& action, double reward) = 0;

    virtual double getQValue(const TState& state, const TAction::Enum& action) = 0;

    virtual double getMaxQValue(const TState& state) = 0;
};
