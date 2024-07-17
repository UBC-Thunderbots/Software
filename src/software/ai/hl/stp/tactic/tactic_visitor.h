#pragma once

#include "software/ai/hl/stp/skill/all_skills.h"

// We forward-declare all the tactics because if we include them we induce a circular
// dependency between the individual library for each tactic and this visitor. This is
// because tactic.h includes tactic_visitor.h, and each individual library includes
// tactic.h. Note: every subclass of this visitor must include all of the classes listed
// below
class Tactic;
class CreaseDefenderTactic;
class GoalieTactic;
class MoveTactic;
class AttackerTactic;
class PassDefenderTactic;
class PenaltyKickTactic;
class PenaltySetupTactic;
class ReceiverTactic;
class ShadowEnemyTactic;
class StopTactic;
class MoveGoalieToGoalLineTactic;
class PrepareKickoffMoveTactic;
class BallPlacementMoveTactic;
class AvoidInterferenceTactic;

template <typename TSkill>
class AssignedSkillTactic;

class KickoffChipSkillTactic;
class BallPlacementDribbleTactic;

/**
 * Refer to the docs about why we use the Visitor Design Pattern
 */
class TacticVisitor
{
   public:
    virtual ~TacticVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param tactic The tactic to visit
     */
    virtual void visit(const CreaseDefenderTactic &tactic)                    = 0;
    virtual void visit(const GoalieTactic &tactic)                            = 0;
    virtual void visit(const MoveTactic &tactic)                              = 0;
    virtual void visit(const AttackerTactic &tactic)                          = 0;
    virtual void visit(const PassDefenderTactic &tactic)                      = 0;
    virtual void visit(const PenaltyKickTactic &tactic)                       = 0;
    virtual void visit(const PenaltySetupTactic &tactic)                      = 0;
    virtual void visit(const ReceiverTactic &tactic)                          = 0;
    virtual void visit(const ShadowEnemyTactic &tactic)                       = 0;
    virtual void visit(const StopTactic &tactic)                              = 0;
    virtual void visit(const MoveGoalieToGoalLineTactic &tactic)              = 0;
    virtual void visit(const PrepareKickoffMoveTactic &tactic)                = 0;
    virtual void visit(const AvoidInterferenceTactic &tactic)                 = 0;
    virtual void visit(const AssignedSkillTactic<ChipSkill> &tactic)          = 0;
    virtual void visit(const AssignedSkillTactic<DribbleSkill> &tactic)       = 0;
    virtual void visit(const AssignedSkillTactic<GetBehindBallSkill> &tactic) = 0;
    virtual void visit(const AssignedSkillTactic<KeepAwaySkill> &tactic)      = 0;
    virtual void visit(const AssignedSkillTactic<KickSkill> &tactic)          = 0;
    virtual void visit(const AssignedSkillTactic<KickPassSkill> &tactic)      = 0;
    virtual void visit(const AssignedSkillTactic<ChipPassSkill> &tactic)      = 0;
    virtual void visit(const AssignedSkillTactic<PivotKickSkill> &tactic)     = 0;
    virtual void visit(const AssignedSkillTactic<ShootSkill> &tactic)         = 0;
    virtual void visit(const AssignedSkillTactic<DribbleShootSkill> &tactic)  = 0;
    virtual void visit(const KickoffChipSkillTactic &tactic)                  = 0;
    virtual void visit(const BallPlacementDribbleTactic &tactic)              = 0;
    virtual void visit(const BallPlacementMoveTactic &tactic)                 = 0;
};
