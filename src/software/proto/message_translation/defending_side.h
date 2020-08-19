#pragma once

#include <memory>

#include "software/proto/defending_side_msg.pb.h"

/**
 * Creates a TeamSideMsg from the given data
 *
 * @param defending_positive_side Whether or not this team is defending the
 * positive side of the field
 *
 * @return A TeamSideMsg containing the given data
 */
std::unique_ptr<DefendingSideProto> createDefendingSideProto(bool defending_positive_side);
