#pragma once

#include <QGraphicsScene>

#include "ai/world/field.h"

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
