/*
 * goal.cpp
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

#include "extlibs/hrvo/goal.h"

#include <stdexcept>
#include <utility>

Goal::Goal(const Vector &position) : position_(position)
{
    positions_.push_back(position);
    speedAtPosition_.push_back(0.f);
}

Goal::Goal(std::vector<Vector> positions) : positions_(std::move(positions))
{
    speedAtPosition_ = std::vector<float>(positions_.size(), 0.f);
}

Goal::Goal(std::vector<Vector> positions, std::vector<float> speed_at_positions)
    : positions_(std::move(positions)), speedAtPosition_(std::move(speed_at_positions))
{
    if (positions_.size() != speedAtPosition_.size())
    {
        throw std::invalid_argument(
            "positions and speed_at_positions arrays have to be the same size");
    }
}

Vector Goal::getNextGoalPostion()
{
    currGoalIndex++;
    return getCurrentGoalPosition();
}

Vector Goal::getCurrentGoalPosition()
{
    if (currGoalIndex >= positions_.size())
    {
        return Vector();
    }
    else
    {
        return positions_[currGoalIndex];
    }
}

bool Goal::isGoingToFinalGoal()
{
    if (currGoalIndex >= positions_.size() - 1)
    {
        return true;
    }
    return false;
}

float Goal::getDesiredSpeedAtCurrentGoal()
{
    if (currGoalIndex >= speedAtPosition_.size())
    {
        return 0.f;
    }
    else
    {
        return speedAtPosition_[currGoalIndex];
    }
}
