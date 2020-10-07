#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/drawing/colors.h"
#include "software/world/field.h"

/**
 * This file contains all the functions that allow us to draw the Field in a
 * QGraphicsScene in Qt
 */

/**
 * Draws the outer field lines in the scene
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 * @param pen The pen used to draw
 */
void drawOuterFieldLines(QGraphicsScene* scene, const Field& field, QPen pen);

/**
 * Draws the lines showing the physical boundary around the field
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 * @param pen The pen used to draw
 */
void drawFieldPhysicalBoundaryLines(QGraphicsScene* scene, const Field& field, QPen pen);

/**
 * Draws the field's defense areas in the scene
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 * @param pen The pen used to draw
 */
void drawDefenseAreas(QGraphicsScene* scene, const Field& field, QPen pen);

/**
 * Draws the goals of the field in the scene
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 * @param pen The pen used to draw
 */
void drawGoals(QGraphicsScene* scene, const Field& field, QPen pen);

/**
 * Highlights each team's goal on the field based on whether they are the friendly or
 * enemy team
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 * @param friendly_goal_colour The colour to highlight the friendly goal
 * @param enemy_goal_colour The colour to highlight the enemy goal
 */
void highlightGoalsByTeam(QGraphicsScene* scene, const Field& field,
                          const QColor& friendly_goal_colour,
                          const QColor& enemy_goal_colour);

/**
 * Draws the center line of the field in the scene
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 * @param pen The pen used to draw
 */
void drawCenterLine(QGraphicsScene* scene, const Field& field, QPen pen);

/**
 * Draws the center circle of the field in the scene
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 * @param pen The pen used to draw
 */
void drawCenterCircle(QGraphicsScene* scene, const Field& field, QPen pen);

/**
 * Draws the field in the GraphicsScene
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 */
void drawField(QGraphicsScene* scene, const Field& field);

/**
 * Marks the friendly and enemy goals on the field with text
 *
 * @param scene The scene to draw on
 * @param field The field to draw
 */
void drawTeamGoalText(QGraphicsScene* scene, const Field& field);
