#pragma once

#include <QtWidgets/QGraphicsScene>
#include <functional>

// This class is used to represent a "draw function", which is a function
// provided to the visualizer that tells is how to draw things.
// All functionality is implemented in this class so
// subclasses only need to call the superconstructor.
//
// This class is made abstract by making the desctructor pure virtual,
// but providing an implementation. The subclasses will have a default
// destructor automatically generated which will implement the virtual
// destructor. All together this lets the base class be pure virtual
// without needing an extra pure-virtual "dummy" function
// See https://stackoverflow.com/a/4641108
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
    virtual ~DrawFunction() = 0;

   protected:
    std::function<void(QGraphicsScene* scene)> draw_function_;
};

inline DrawFunction::~DrawFunction() = default;

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
