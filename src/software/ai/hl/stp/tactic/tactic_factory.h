#pragma once

#include "proto/tactic.pb.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"

std::shared_ptr<Tactic> createTactic(const TbotsProto::AttackerTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::ChipTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(
    const TbotsProto::CreaseDefenderTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::DribbleTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::GetBehindBallTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::GoalieTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::KickTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(
    const TbotsProto::MoveGoalieToGoalLineTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::MoveTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::PenaltyKickTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::PivotKickTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::ReceiverTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::ShadowEnemyTactic &tactic_proto);
std::shared_ptr<Tactic> createTactic(const TbotsProto::StopTactic &tactic_proto);
