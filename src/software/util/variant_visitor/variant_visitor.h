#pragma once

// Creates a struct which inherits all lambda function given to it and uses their
// Ts::operator(). This can be passed to std::visit to easily write multiple different
// lambdas for each type of motion controller commands below. See
// https://en.cppreference.com/w/cpp/utility/variant/visit for more details.
template <class... Ts>
struct overload : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overload(Ts...) -> overload<Ts...>;
