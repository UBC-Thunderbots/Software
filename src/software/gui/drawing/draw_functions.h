#pragma once

#include <QtWidgets/QGraphicsScene>
#include <functional>

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
 * "store" it for later. That way the entire DrawFunction can be copied
 * and passed around the system, until some consumer calls it to
 * draw its contents. This allows us to draw larger non-copyable objects
 * by creating a DrawFunction from it's copyable components / members.
 */
class DrawFunction
{
   public:
    DrawFunction() = default;
    inline explicit DrawFunction(
        const std::function<void(QGraphicsScene* scene)>& draw_function)
    {
        draw_function_ = draw_function;
    }
    inline void execute(QGraphicsScene* scene)
    {
        if (draw_function_)
        {
            draw_function_(scene);
        }
    }

   private:
    std::function<void(QGraphicsScene* scene)> draw_function_;
};

// Note: Generally the top-level DrawFunction class can be used, but if we need
// separate concrete types (for example so an Observer can explicitly observe
// two different DrawFunctions), they can be defined here.

class AIDrawFunction : public DrawFunction
{
   public:
    AIDrawFunction() = default;
    inline explicit AIDrawFunction(
        const std::function<void(QGraphicsScene* scene)>& draw_function)
        : DrawFunction(draw_function)
    {
    }
};

class WorldDrawFunction : public DrawFunction
{
   public:
    WorldDrawFunction() = default;
    inline explicit WorldDrawFunction(
        const std::function<void(QGraphicsScene* scene)>& draw_function)
        : DrawFunction(draw_function)
    {
    }
};
