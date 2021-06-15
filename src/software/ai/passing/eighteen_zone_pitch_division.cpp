#include "software/ai/passing/eighteen_zone_pitch_division.h"

#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

EighteenZonePitchDivision::EighteenZonePitchDivision(const Field& field)
{
    const double zone_width  = field.xLength() / 6;
    const double zone_height = field.yLength() / 3;

    for (double pos_x = -field.xLength() / 2; pos_x < field.xLength() / 2;
         pos_x += zone_width)
    {
        for (double pos_y = field.yLength() / 2; pos_y > -field.yLength() / 2;
             pos_y -= zone_height)
        {
            pitch_division_.emplace_back(Rectangle(
                Point(pos_x, pos_y), Point(pos_x + zone_width, pos_y - zone_height)));
        }
    }

    zones_       = allValuesEighteenZoneId();
    field_lines_ = std::make_shared<Rectangle>(field.fieldLines());
}

const Rectangle& EighteenZonePitchDivision::getZone(EighteenZoneId zone_id) const
{
    return pitch_division_[static_cast<unsigned>(zone_id)];
}

EighteenZoneId EighteenZonePitchDivision::getZoneId(const Point& position) const
{
    if (!contains(*field_lines_, position))
    {
        throw std::invalid_argument("requested position not on field!");
    }

    auto zone_id = *std::find_if(zones_.begin(), zones_.end(),
                                 [this, position](const EighteenZoneId& id) {
                                     return contains(getZone(id), position);
                                 });
    return zone_id;
}

const std::vector<EighteenZoneId>& EighteenZonePitchDivision::getAllZoneIds() const
{
    return zones_;
}

const std::vector<EighteenZoneId> EighteenZonePitchDivision::getAdjacentZoneIds(EighteenZoneId zone_id) const {
    std::vector<EighteenZoneId> adjacent_zones;
    switch(zone_id) {
        case EighteenZoneId::ZONE_1:
            adjacent_zones = {
                    EighteenZoneId::ZONE_4,
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_2
            };
            break;
        case EighteenZoneId::ZONE_2:
            adjacent_zones = {
                    EighteenZoneId::ZONE_1,
                    EighteenZoneId::ZONE_4,
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_3,
                    EighteenZoneId::ZONE_6
            };
            break;
        case EighteenZoneId::ZONE_3:
            adjacent_zones = {
                    EighteenZoneId::ZONE_2,
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_6
            };
            break;
        case EighteenZoneId::ZONE_4:
            adjacent_zones = {
                    EighteenZoneId::ZONE_1,
                    EighteenZoneId::ZONE_2,
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_7,
                    EighteenZoneId::ZONE_8,
            };
            break;
        case EighteenZoneId::ZONE_5:
            adjacent_zones = {
                    EighteenZoneId::ZONE_1,
                    EighteenZoneId::ZONE_2,
                    EighteenZoneId::ZONE_3,
                    EighteenZoneId::ZONE_4,
                    EighteenZoneId::ZONE_6,
                    EighteenZoneId::ZONE_7,
                    EighteenZoneId::ZONE_8,
                    EighteenZoneId::ZONE_9
            };
            break;
        case EighteenZoneId::ZONE_6:
            adjacent_zones = {
                    EighteenZoneId::ZONE_2,
                    EighteenZoneId::ZONE_3,
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_8,
                    EighteenZoneId::ZONE_9
            };
            break;
        case EighteenZoneId::ZONE_7:
            adjacent_zones = {
                    EighteenZoneId::ZONE_4,
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_8,
                    EighteenZoneId::ZONE_10,
                    EighteenZoneId::ZONE_11
            };
            break;
        case EighteenZoneId::ZONE_8:
            adjacent_zones = {
                    EighteenZoneId::ZONE_4,
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_6,
                    EighteenZoneId::ZONE_7,
                    EighteenZoneId::ZONE_9,
                    EighteenZoneId::ZONE_10,
                    EighteenZoneId::ZONE_11,
                    EighteenZoneId::ZONE_12
            };
            break;
        case EighteenZoneId::ZONE_9:
            adjacent_zones = {
                    EighteenZoneId::ZONE_5,
                    EighteenZoneId::ZONE_6,
                    EighteenZoneId::ZONE_8,
                    EighteenZoneId::ZONE_11,
                    EighteenZoneId::ZONE_12
            };
            break;
        case EighteenZoneId::ZONE_10:
            adjacent_zones = {
                    EighteenZoneId::ZONE_7,
                    EighteenZoneId::ZONE_8,
                    EighteenZoneId::ZONE_11,
                    EighteenZoneId::ZONE_13,
                    EighteenZoneId::ZONE_14
            };
            break;
        case EighteenZoneId::ZONE_11:
            adjacent_zones = {
                    EighteenZoneId::ZONE_7,
                    EighteenZoneId::ZONE_8,
                    EighteenZoneId::ZONE_9,
                    EighteenZoneId::ZONE_10,
                    EighteenZoneId::ZONE_12,
                    EighteenZoneId::ZONE_13,
                    EighteenZoneId::ZONE_14,
                    EighteenZoneId::ZONE_15
            };
            break;
        case EighteenZoneId::ZONE_12:
            adjacent_zones = {
                    EighteenZoneId::ZONE_8,
                    EighteenZoneId::ZONE_9,
                    EighteenZoneId::ZONE_11,
                    EighteenZoneId::ZONE_14,
                    EighteenZoneId::ZONE_15
            };
            break;
        case EighteenZoneId::ZONE_13:
            adjacent_zones = {
                    EighteenZoneId::ZONE_10,
                    EighteenZoneId::ZONE_11,
                    EighteenZoneId::ZONE_14,
                    EighteenZoneId::ZONE_16,
                    EighteenZoneId::ZONE_17
            };
            break;
        case EighteenZoneId::ZONE_14:
            adjacent_zones = {
                    EighteenZoneId::ZONE_10,
                    EighteenZoneId::ZONE_11,
                    EighteenZoneId::ZONE_12,
                    EighteenZoneId::ZONE_13,
                    EighteenZoneId::ZONE_15,
                    EighteenZoneId::ZONE_16,
                    EighteenZoneId::ZONE_17,
                    EighteenZoneId::ZONE_18
            };
            break;
        case EighteenZoneId::ZONE_15:
            adjacent_zones = {
                    EighteenZoneId::ZONE_11,
                    EighteenZoneId::ZONE_12,
                    EighteenZoneId::ZONE_14,
                    EighteenZoneId::ZONE_17,
                    EighteenZoneId::ZONE_18
            };
            break;
        case EighteenZoneId::ZONE_16:
            adjacent_zones = {
                    EighteenZoneId::ZONE_13,
                    EighteenZoneId::ZONE_14,
                    EighteenZoneId::ZONE_17
            };
            break;
        case EighteenZoneId::ZONE_17:
            adjacent_zones = {
                    EighteenZoneId::ZONE_13,
                    EighteenZoneId::ZONE_14,
                    EighteenZoneId::ZONE_15,
                    EighteenZoneId::ZONE_16,
                    EighteenZoneId::ZONE_18
            };
            break;
        case EighteenZoneId::ZONE_18:
            adjacent_zones = {
                    EighteenZoneId::ZONE_14,
                    EighteenZoneId::ZONE_15,
                    EighteenZoneId::ZONE_17
            };
            break;
    }
    return adjacent_zones;
}


FourZonePitchDivision::FourZonePitchDivision(const Field& field)
{
    const double zone_width  = field.xLength() / 2;
    const double zone_height = field.yLength() / 2;

    pitch_division_.emplace_back(Rectangle(
            field.centerPoint() + Vector(0, -1),
            Point(field.xLength()/4.0, -field.yLength()/2)
            ));
    pitch_division_.emplace_back(Rectangle(
            field.enemyCornerNeg(),
            field.centerPoint() + Vector(field.xLength()/4.0, -1)
    ));
    pitch_division_.emplace_back(Rectangle(
            field.centerPoint() + Vector(0, 1),
            Point(field.xLength()/4.0, field.yLength()/2)
    ));
    pitch_division_.emplace_back(Rectangle(
            field.enemyCornerPos(),
            field.centerPoint() + Vector(field.xLength()/4.0, 1)
    ));

//    for (double pos_x = -field.xLength() / 2; pos_x < field.xLength() / 2;
//         pos_x += zone_width)
//    {
//        for (double pos_y = field.yLength() / 2; pos_y > -field.yLength() / 2;
//             pos_y -= zone_height)
//        {
//            pitch_division_.emplace_back(Rectangle(
//                    Point(pos_x, pos_y), Point(pos_x + zone_width, pos_y - zone_height)));
//        }
//    }

    zones_       = allValuesFourZoneId();
    field_lines_ = std::make_shared<Rectangle>(field.fieldLines());
}

const Rectangle& FourZonePitchDivision::getZone(FourZoneId zone_id) const
{
    return pitch_division_[static_cast<unsigned>(zone_id)];
}

FourZoneId FourZonePitchDivision::getZoneId(const Point& position) const
{
    if (!contains(*field_lines_, position))
    {
        throw std::invalid_argument("requested position not on field!");
    }

    auto zone_id = *std::find_if(zones_.begin(), zones_.end(),
                                 [this, position](const FourZoneId& id) {
                                     return contains(getZone(id), position);
                                 });
    return zone_id;
}

const std::vector<FourZoneId>& FourZonePitchDivision::getAllZoneIds() const
{
    return zones_;
}

const std::vector<FourZoneId> FourZonePitchDivision::getAdjacentZoneIds(FourZoneId zone_id) const {
    return std::vector<FourZoneId>();
}
