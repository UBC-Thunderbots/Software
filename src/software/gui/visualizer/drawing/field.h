#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/visualizer/drawing/colors.h"
#include "software/gui/visualizer/drawing/geom.h"
#include "software/gui/visualizer/geom/geometry_conversion.h"
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
 * @param friendly_team_brush The brush used to highlight the friendly team's goal
 * @param enemy_team_brush The brush used to highlight the enemy team's goal
 */
void highlightGoalsByTeam(QGraphicsScene* scene, const Field& field,
                          QBrush friendly_team_brush, QBrush enemy_team_brush);

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
