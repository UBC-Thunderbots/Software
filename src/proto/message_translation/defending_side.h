#pragma once

#include <memory>

#include "proto/defending_side_msg.pb.h"
#include "software/world/field.h"

/**
 * Creates a DefendingSideProto from the given data
 *
 * @param defending_side The side of the field the team is defending
 *
 * @return A DefendingSideProto containing the given data
 */
std::unique_ptr<DefendingSideProto> createDefendingSide(const FieldSide& defending_side);
