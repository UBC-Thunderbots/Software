#include "software/ai/hl/stp/tactic/tactic_factory.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"

std::shared_ptr<Tactic> createTactic(const TbotsProto::AttackerTactic &tactic_proto)
{
    auto config = std::make_shared<AttackerTacticConfig>();
    config->loadFromProto(tactic_proto.attacker_tactic_config());
    auto tactic = std::make_shared<AttackerTactic>(config);

    if (tactic_proto.has_best_pass_so_far())
    {
        tactic->updateControlParams(
            Pass(createPoint(tactic_proto.best_pass_so_far().passer_point()),
                 createPoint(tactic_proto.best_pass_so_far().receiver_point()),
                 tactic_proto.best_pass_so_far().pass_speed_m_per_s()),
            tactic_proto.pass_committed());
    }
    if (tactic_proto.has_chip_target())
    {
        tactic->updateControlParams(createPoint(tactic_proto.chip_target()));
    }

    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::ChipTactic &tactic_proto)
{
    auto tactic = std::make_shared<ChipTactic>();
    tactic->updateControlParams(createPoint(tactic_proto.chip_origin()),
                                createAngle(tactic_proto.chip_direction()),
                                tactic_proto.chip_distance_meters());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::CreaseDefenderTactic &tactic_proto)
{
    auto config = std::make_shared<RobotNavigationObstacleConfig>();

    config->loadFromProto(tactic_proto.robot_navigation_obstacle_config());
    auto tactic = std::make_shared<CreaseDefenderTactic>(config);

    tactic->updateControlParams(createPoint(tactic_proto.enemy_threat_origin()),
                                tactic_proto.crease_defender_alignment(),
                                tactic_proto.max_allowed_speed_mode());

    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::DribbleTactic &tactic_proto)
{
    auto tactic                                    = std::make_shared<DribbleTactic>();
    std::optional<Point> dribble_destination       = std::nullopt;
    std::optional<Angle> final_dribble_orientation = std::nullopt;
    if (tactic_proto.has_dribble_destination())
    {
        dribble_destination = createPoint(tactic_proto.dribble_destination());
    }
    if (tactic_proto.has_final_dribble_orientation())
    {
        final_dribble_orientation = createAngle(tactic_proto.final_dribble_orientation());
    }

    tactic->updateControlParams(dribble_destination, final_dribble_orientation,
                                tactic_proto.allow_excessive_dribbling());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::GetBehindBallTactic &tactic_proto)
{
    auto tactic = std::make_shared<GetBehindBallTactic>();
    tactic->updateControlParams(createPoint(tactic_proto.ball_location()),
                                createAngle(tactic_proto.chick_direction()));
    return tactic;
}

// std::shared_ptr<Tactic> createTactic(const TbotsProto::GoalieTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(const TbotsProto::KickTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(
//    const TbotsProto::MoveGoalieToGoalLineTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(const TbotsProto::MoveTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(const TbotsProto::PenaltyKickTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(const TbotsProto::PivotKickTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(const TbotsProto::ReceiverTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(const TbotsProto::ShadowEnemyTactic &tactic_proto)
//{
//}
//
// std::shared_ptr<Tactic> createTactic(const TbotsProto::StopTactic &tactic_proto)
//{
//}
//
