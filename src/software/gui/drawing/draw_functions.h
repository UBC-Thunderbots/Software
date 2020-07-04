#pragma once

#include <QtWidgets/QGraphicsScene>
#include <functional>

using DrawFunction = std::function<void(QGraphicsScene* scene)>;

/**
 * This class is used to represent a "draw function", which is a function
 * provided to various GUI components that tells them how to draw things.
 *
 * Draw functions exist because certain objects we would normally like to
 * draw (like the AI) are non-copyable, so they can't be sent over
 * the Observer system. We'd like to keep GUI / drawing-specific code
 * out of our logic as much as possible, so DrawFunctions are the
 * solution to that.
 *
 * DrawFunctions can capture any data that supports copying and
 * "store" it for later. That way the entire DrawFunctionWrapper can be copied
 * and passed around the system, until some consumer calls it to
 * draw its contents. This allows us to draw larger non-copyable objects
 * by creating a DrawFunctionWrapper from it's copyable components / members.
 *
 * This class is made abstract by making the destructor pure virtual,
 * but providing an implementation. The subclasses will have a default
 * destructor automatically generated which will implement the virtual
 * destructor. All together this lets the base class be pure virtual
 * without needing an extra pure-virtual "dummy" function
 * See https://stackoverflow.com/a/4641108
 */
class DrawFunctionWrapper
{
   public:
    /**
     * Creates a DrawFunctionWrapper
     *
     * @pre draw_function must be callable (ie. operator bool(draw_function) == true)
     *
     * @param draw_function The function to use for drawing
     */
    inline explicit DrawFunctionWrapper(
        const std::function<void(QGraphicsScene* scene)>& draw_function)
    {
        if (!draw_function)
        {
            throw std::invalid_argument(
                "Created a DrawFunctionWrapper with a non-callable function");
        }

        draw_function_ = draw_function;
    }
    DrawFunctionWrapper()          = delete;
    virtual ~DrawFunctionWrapper() = 0;

    /**
     * Returns the internal draw_function
     *
     * @return the internal draw_function
     */
    inline std::function<void(QGraphicsScene* scene)> getDrawFunction()
    {
        return draw_function_;
    }

   private:
    DrawFunction draw_function_;
};

inline DrawFunctionWrapper::~DrawFunctionWrapper() = default;

/**
 * We inherit from the generic DrawFunctionWrapper to create "strong" types
 * for different functions. This is required by the Observer system because
 * it operates on types, and simply using a typedef would result in
 * conflicting functions is an Observer tried to observe two different
 * DrawFunctions at once.
 *
 * Additionally, it is useful to be able to distinguish different DrawFunctions
 * because we can make more intelligent decisions about ordering when drawing.
 * For example, we can make sure we always draw the Field before anything from
 * the AI, so the AI drawings always show on top.
 */

/**
 * This class identifies anything that we may want to show about
 * the AI or its internal state. For example, navigation paths,
 * highlighting threatening enemies, etc.
 */
class AIDrawFunction : public DrawFunctionWrapper
{
   public:
    inline explicit AIDrawFunction(
        const std::function<void(QGraphicsScene* scene)>& draw_function)
        : DrawFunctionWrapper(draw_function)
    {
    }
};

/**
 * This class identifies anything that we may want to show about
 * the state of the World / things happening on the field. For example,
 * robot positions, ball position, field lines, etc.
 */
class WorldDrawFunction : public DrawFunctionWrapper
{
   public:
    inline explicit WorldDrawFunction(
        const std::function<void(QGraphicsScene* scene)>& draw_function)
        : DrawFunctionWrapper(draw_function)
    {
    }
};
