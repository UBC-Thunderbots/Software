#pragma once

#include <QtWidgets/QGraphicsScene>
#include <functional>

# An abstract base class of a DrawFunction that is used to draw objects
# in the Visualizer. All functionality is implemented in this class so
# subclasses only need to call the superconstructor.
#
# This class is made abstract by making the desctructor pure virtual,
# but providing an implementation. The subclasses will have a default
# destructor automatically generated which will implement the virtual
# destructor. Alltogether this lets the base class be pure virtual
# without needing an extra pure-virtual "dummy" function
# See https://stackoverflow.com/a/4641108
class DrawFunction {
public:
    DrawFunction() = delete;
    inline DrawFunction(const std::function<void(QGraphicsScene* scene)>& draw_function) {
        draw_function_ = draw_function;
    }
    inline void execute(QGraphicsScene* scene) {
        draw_function_(scene);
    }
    virtual ~DrawFunction() = 0;

protected:
    std::function<void(QGraphicsScene* scene)> draw_function_;
};

inline DrawFunction::~DrawFunction() {}

class AIDrawFunction : public DrawFunction {
    inline AIDrawFunction(const std::function<void(QGraphicsScene* scene)>& draw_function) : DrawFunction(draw_function)
    {}
};

class WorldDrawFunction : public DrawFunction {
    inline WorldDrawFunction(const std::function<void(QGraphicsScene* scene)>& draw_function) : DrawFunction(draw_function)
    {}
};


// TODO: comment
//using DrawFunction = std::function<void(QGraphicsScene* scene)>;
//using AIDrawFunction = std::function<void(QGraphicsScene* scene)>;
//BOOST_STRONG_TYPEDEF(std::function<void(QGraphicsScene* scene)>, DrawFunction);
//BOOST_STRONG_TYPEDEF(std::function<void(QGraphicsScene* scene)>, AIDrawFunction);
