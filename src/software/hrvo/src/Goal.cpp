/*
 * Goal.cpp
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

/**
 * \file   Goal.cpp
 * \brief  Defines the Goal class.
 */

#include "Goal.h"

#include <utility>
#include <stdexcept>

namespace hrvo {
	Goal::Goal(const Vector2 &position) : position_(position)
    {
        positions_.push_back(position);
        speedAtPosition_.push_back(0.f);
    }

	Goal::Goal(std::vector<Vector2> positions) : positions_(std::move(positions))
    {
        speedAtPosition_ = std::vector<float>(positions_.size(), 0.f);
    }

    Goal::Goal(std::vector<Vector2> positions, std::vector<float> speedAtPostition) :
    positions_(std::move(positions)),
    speedAtPosition_(std::move(speedAtPostition))
    {
        if (positions_.size() != speedAtPosition_.size())
        {
            throw std::invalid_argument("positions and speedAtPostition arrays have to be the same size");
        }
    }

    Vector2 Goal::getNextGoalPostion() {
        currGoalIndex++;
        return getCurrentGoalPosition();
    }

    Vector2 Goal::getCurrentGoalPosition() {
        if (currGoalIndex >= positions_.size())
        {
            return Vector2();
        }
        else
        {
            return positions_[currGoalIndex];
        }
    }

    bool Goal::isGoingToFinalGoal() {
        if(currGoalIndex >= positions_.size() - 1)
        {
            return true;
        }
        return false;
    }

    float Goal::getDesiredSpeedAtCurrentGoal() {
        if (currGoalIndex >= speedAtPosition_.size())
        {
            return 0.f;
        }
        else
        {
            return speedAtPosition_[currGoalIndex];
        }
    }
}
