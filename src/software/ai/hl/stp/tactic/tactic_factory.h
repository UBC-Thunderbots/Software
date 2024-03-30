#pragma once

#include "proto/tactic.pb.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"

/**
 * Creates a tactic given a tactic proto
 *
 * @param tactic_proto the tactic proto
 * @param ai_config The AI config
 *
 * @return a pointer to the tactic
 */
std::shared_ptr<Tactic> createTactic(const TbotsProto::Tactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::AttackerTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::ChipTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::CreaseDefenderTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::GetBehindBallTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::GoalieTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::KickTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(
    const TbotsProto::MoveGoalieToGoalLineTactic &tactic_proto,
    std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::MoveTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::PassDefenderTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::PenaltyKickTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::ReceiverTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::ShadowEnemyTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::StopTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::DribbleSkillTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);
std::shared_ptr<Tactic> createTactic(const TbotsProto::PivotKickSkillTactic &tactic_proto,
                                     std::shared_ptr<Strategy> strategy);

/**
 * Creates the corresponding C++ struct/class from a proto
 *
 * @param the proto representation of that class
 *
 * @return the C++ struct/class
 */
AutoChipOrKick createAutoChipOrKick(
    const TbotsProto::AutoChipOrKick &auto_chip_or_kick_proto);
Pass createPass(const TbotsProto::Pass &pass_proto);
EnemyThreat createEnemyThreat(const TbotsProto::EnemyThreat &enemy_threat_proto);
