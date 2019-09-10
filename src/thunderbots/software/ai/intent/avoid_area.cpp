#include "ai/intent/avoid_area.h"

#include <map>
#include <string>

static const std::map<AvoidArea, std::string> avoid_area_names = {
    {AvoidArea::ENEMY_ROBOTS, "ENEMY_ROBOTS"},
    {AvoidArea::FRIENDLY_DEFENSE_AREA, "FRIENDLY_DEFENSE_AREA"},
    {AvoidArea::ENEMY_DEFENSE_AREA, "ENEMY_DEFENSE_AREA"},
    {AvoidArea::INFLATED_ENEMY_DEFENSE_AREA, "INFLATED_ENEMY_DEFENSE_AREA"},
    {AvoidArea::CENTER_CIRCLE, "CENTER_CIRCLE"},
    {AvoidArea::HALF_METER_AROUND_BALL, "HALF_METER_AROUND_BALL"},
    {AvoidArea::BALL, "BALL"},
    {AvoidArea::ENEMY_HALF, "ENEMY_HALF"},
    {AvoidArea::FRIENDLY_HALF, "FRIENDLY_HALF"}};

std::ostream& operator<<(std::ostream& os, const AvoidArea& area)
{
    auto iter = avoid_area_names.find(area);
    if (iter != avoid_area_names.end())
    {
        os << iter->second;
    }
    else
    {
        os << "No String Representation For AvoidArea: " << static_cast<int>(area);
    }
    return os;
}

avoid_area_mask_t getAvoidAreasFromGameState(const GameState& game_state)
{
    avoid_area_mask_t areas_to_avoid = 0;

    MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::FRIENDLY_DEFENSE_AREA);

    if (game_state.stayAwayFromBall())
    {
        MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::HALF_METER_AROUND_BALL);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::ENEMY_HALF);
        }
        else
        {
            // Is their penalty
            MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::FRIENDLY_HALF);
        }
    }
    else if (game_state.isKickoff())
    {
        MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::HALF_METER_AROUND_BALL);
        MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::CENTER_CIRCLE);
        MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::ENEMY_HALF);
    }
    else
    {
        if (game_state.stayAwayFromBall() || game_state.isOurKickoff())
        {
            MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::HALF_METER_AROUND_BALL);
        }

        if (game_state.isOurPenalty())
        {
            MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::ENEMY_DEFENSE_AREA);
        }
        else
        {
            MERGE_AVOID_AREAS(areas_to_avoid, AvoidArea::INFLATED_ENEMY_DEFENSE_AREA);
        }
    }

    return areas_to_avoid;
}
