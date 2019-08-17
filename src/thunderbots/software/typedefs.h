#pragma once

/**
 * We use this typedef to pass lists of Primitives around
 *
 * Since `std::unique_ptr<Primitive>` does not have a copy constructor, we
 * need to pass a `shared_ptr` to the vector. To prevent people modifiying
 * the vector we declare it `const`
 */
using ConstPrimitiveVectorPtr = std::shared_ptr<
        const std::vector<std::unique_ptr<Primitive>>>;
