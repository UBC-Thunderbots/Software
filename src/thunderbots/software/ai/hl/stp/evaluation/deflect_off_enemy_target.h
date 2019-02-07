//
// Created by roark on 02/02/19.
//


#ifndef THUNDERBOTS_ALL_DEFLECT_OFF_ENEMY_TARGET_H
#define THUNDERBOTS_ALL_DEFLECT_OFF_ENEMY_TARGET_H


/* Returns the point to which the player taking the friendly
 * indirect kick should chip to, to chip over the first blocker (and aim at the
 * net)
 */
std::pair<Point, Angle> indirect_chip_target(World world, Player player);
