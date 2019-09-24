#pragma once

#include <QtWidgets/QGraphicsScene>
#include <functional>

// TODO: comment
using DrawFunction = std::function<void(QGraphicsScene* scene)>;
