#pragma once

#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QOpenGLWidget>

/**
 * Sets up the SceneView which draws the AI's view of the world, such as robot
 * positions, ball velocity, etc.
 *
 * @param view The view to display the QGraphicsScene with
 * @param scene The QGraphicsScene that will be displayed by the view
 * @param gl_widget A QOpenGLWidget that can be used to help display items in the view
 */
void setupSceneView(QGraphicsView* view, QGraphicsScene* scene, QOpenGLWidget* gl_widget);
